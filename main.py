"""
main.py —— PICO_Driver 系统调度层

数据流:
    H743 --UART 921600--> Pico main.py --> BDShot(主方案, PIO1+PIO2, 4路)
                                       --> 降级时 Signal_DShot
    遥测: 转速 BDShot 优先; 电压/电流/温度 优先用 EDT(同一根信号线),
          没有则回退 KISS_Telemetry(PIO0, 4路常驻)。
    EscStatus --UART--> H743

本版改动:
  1. EscBridge 自动开启 EDT(DShot 命令13): 已enable、油门0、还没拿到EDT数据时重试。
  2. _fill_status 改为混合来源: erpm=BDShot, V/I/T=EDT 优先, KISS 回退。
     bidir_valid / kiss_valid 现在分别表示各自数据源是否在线, 可同时为 True。
  3. 指令油门为0超过 ZERO_ERPM_MS 后, BDShot 来源的 erpm 上报 0
     (解决停转后 erpm 卡在旧值)。
  4. selftest() 增加 EDT 检查与 V/I/T 汇总。

不修改 BDShot.py / Uart_link.py / KISS_Telemetry.py。
"""

import time
import micropython

from Uart_link import (
    UartLink,
    EscStatus,
    MAX_ESC,
)
import KISS_Telemetry as kiss
from BDShot import DShotBus
import Signal_DShot as sig_mod

micropython.alloc_emergency_exception_buf(100)

# ============================================================
# 配置
# ============================================================
MOTOR_PINS = [3, 7, 11, 15]
ESC_NAMES = ["ESC%d" % (i + 1) for i in range(MAX_ESC)]

if len(MOTOR_PINS) != MAX_ESC:
    raise ValueError(
        "MOTOR_PINS 配置了 %d 路引脚，MAX_ESC=%d 不一致" % (len(MOTOR_PINS), MAX_ESC)
    )

# 单路降级时 Signal_DShot 复用的 (global tx sm_id, pio_num)
DEGRADE_SM = [(4, 1), (6, 1), (8, 2), (10, 2)]

POLE_PAIRS = 1
UPDATE_HZ = 500

BDSHOT_STALE_MS = 300
KISS_STALE_MS = 1000
STATUS_PERIOD_MS = 2

# EDT 自动开启
EDT_RETRY_MS = 1000     # 两次尝试间隔
EDT_MAX_TRIES = 8       # 每路最多尝试次数

# 指令油门为0持续这么久后, 上报 erpm=0
ZERO_ERPM_MS = 1000

MODE_BDSHOT = 0
MODE_BACKUP = 1


# ============================================================
# BDShot 遥测新鲜度旁路监控
# ============================================================
class BdshotHealthTracker:
    def __init__(self, n):
        now = time.ticks_ms()
        self._prev_count = [0] * n
        self._last_change_ms = [now] * n

    def update(self, motors):
        now = time.ticks_ms()
        for i, m in enumerate(motors):
            if m.valid_frames != self._prev_count[i]:
                self._prev_count[i] = m.valid_frames
                self._last_change_ms[i] = now

    def is_alive(self, i, timeout_ms=BDSHOT_STALE_MS):
        return time.ticks_diff(time.ticks_ms(), self._last_change_ms[i]) < timeout_ms


# ============================================================
# 系统调度主体
# ============================================================
class EscBridge:
    def __init__(self):
        self.link = UartLink()
        self.statuses = [EscStatus() for _ in range(MAX_ESC)]

        self.bdshot = None
        self.bdshot_health = None
        self.channel_mode = [MODE_BDSHOT] * MAX_ESC
        self.signal_escs = [None] * MAX_ESC

        now = time.ticks_ms()
        self._edt_tries = [0] * MAX_ESC
        self._edt_last = [now] * MAX_ESC
        self._zero_since = [now] * MAX_ESC     # 指令油门持续为0的起点; None=当前在给油
        self._last_status_ms = now

        self._init_bdshot()

    # -------------------- 初始化 / 单路降级 --------------------
    def _init_bdshot(self):
        try:
            self.bdshot = DShotBus(
                motor_pins=MOTOR_PINS,
                update_hz=UPDATE_HZ,
                names=ESC_NAMES,
                pole_pairs=POLE_PAIRS,
            )
            self.bdshot_health = BdshotHealthTracker(MAX_ESC)
            self.channel_mode = [MODE_BDSHOT] * MAX_ESC
            print("[main] BDShot 主方案初始化成功, 固定4路")
        except Exception as e:
            print("[main] BDShot 整体初始化失败，4路全部直接降级到 Signal_DShot:", e)
            self.bdshot = None
            self.bdshot_health = None
            for i in range(MAX_ESC):
                self._degrade_channel(i)

    def _degrade_channel(self, i):
        if self.channel_mode[i] == MODE_BACKUP:
            return
        print("[main] ESC%d 单路降级: BDShot -> Signal_DShot" % (i + 1))

        if self.bdshot is not None and self.bdshot.is_motor_active(i):
            try:
                self.bdshot.disable_motor(i)
            except Exception as e:
                print("[main] bdshot.disable_motor(%d) 异常(继续降级):" % i, e)

        sm_id, pio_num = DEGRADE_SM[i]
        self.signal_escs[i] = sig_mod.DShot600(
            pin_num=MOTOR_PINS[i], sm_id=sm_id, pio_num=pio_num
        )
        self.channel_mode[i] = MODE_BACKUP

    # -------------------- 主循环 tick --------------------
    def step(self):
        self.link.poll_commands()
        self._drive_escs()
        self._ensure_edt()
        self._check_bdshot_health()
        self._send_status_slice()

    def _drive_escs(self):
        now = time.ticks_ms()
        for i in range(MAX_ESC):
            cmd = self.link.commands[i]
            thr = cmd.throttle if cmd.enabled else 0

            # 记录"指令油门持续为0"的起点
            if thr == 0:
                if self._zero_since[i] is None:
                    self._zero_since[i] = now
            else:
                self._zero_since[i] = None

            if self.channel_mode[i] == MODE_BDSHOT and self.bdshot is not None \
                    and self.bdshot.is_motor_active(i):
                self.bdshot.set_throttle(i, thr)
            elif self.signal_escs[i] is not None:
                self.signal_escs[i].set_throttle(thr)

    def _ensure_edt(self):
        """某路已enable、油门0、还没拿到EDT数据时发命令13。
        每次 step 最多处理一路(enable_extended_telemetry 会阻塞约6~8ms)。"""
        if self.bdshot is None:
            return
        now = time.ticks_ms()
        for i in range(MAX_ESC):
            if self.channel_mode[i] != MODE_BDSHOT or not self.bdshot.is_motor_active(i):
                continue
            m = self.bdshot.motors[i]
            if m.edt_enabled or m.type_counts.get("voltage"):
                continue
            c = self.link.commands[i]
            if not c.enabled or c.throttle != 0:
                continue
            if self._edt_tries[i] >= EDT_MAX_TRIES:
                continue
            if time.ticks_diff(now, self._edt_last[i]) < EDT_RETRY_MS:
                continue
            self._edt_last[i] = now
            self._edt_tries[i] += 1
            self.bdshot.enable_extended_telemetry(i, True)
            return

    def _check_bdshot_health(self):
        if self.bdshot is None:
            return
        self.bdshot_health.update(self.bdshot.motors)
        for i in range(MAX_ESC):
            if self.channel_mode[i] != MODE_BDSHOT:
                continue
            if not self.bdshot.is_motor_active(i):
                continue
            if not self.bdshot_health.is_alive(i, BDSHOT_STALE_MS):
                self._degrade_channel(i)

    def _send_status_slice(self):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_status_ms) < STATUS_PERIOD_MS:
            return
        self._last_status_ms = now

        i = self.link._send_idx
        self._fill_status(i, self.statuses[i])
        self.link.send_next_status(self.statuses)

    def _fill_status(self, i, st):
        kp = kiss.escs[ESC_NAMES[i]]
        kiss_alive = kp.is_alive(KISS_STALE_MS)

        bd_alive = False
        m = None
        if self.channel_mode[i] == MODE_BDSHOT and self.bdshot is not None \
                and self.bdshot.is_motor_active(i):
            bd_alive = self.bdshot_health.is_alive(i)
            m = self.bdshot.motors[i]

        st.enabled = self.link.commands[i].enabled
        st.bidir_valid = bd_alive
        st.kiss_valid = kiss_alive

        # ---- 转速: BDShot 优先, 其次 KISS ----
        if bd_alive:
            erpm = m.latest_erpm or 0
            zs = self._zero_since[i]
            if zs is not None and time.ticks_diff(time.ticks_ms(), zs) > ZERO_ERPM_MS:
                erpm = 0          # 指令油门为0已久, 避免显示卡住的旧值
            st.erpm = erpm
        elif kiss_alive:
            st.erpm = kp.erpm
        else:
            st.erpm = 0

        # ---- 电压/电流/温度: EDT 有值用 EDT, 否则回退 KISS ----
        if bd_alive and m.latest_voltage_v is not None:
            st.voltage = m.latest_voltage_v
        elif kiss_alive:
            st.voltage = kp.voltage
        else:
            st.voltage = 0.0

        if bd_alive and m.latest_current_a is not None:
            st.current = m.latest_current_a
        elif kiss_alive:
            st.current = kp.current
        else:
            st.current = 0.0

        if bd_alive and m.latest_temp_c is not None:
            st.temperature = m.latest_temp_c
        elif kiss_alive:
            st.temperature = kp.temperature
        else:
            st.temperature = 0


def main():
    bridge = EscBridge()
    print("[main] 系统启动完成，进入主循环")
    while True:
        bridge.step()
        time.sleep_ms(1)


# ============================================================
# 第二步测试：Pico 单板测试（不接 H743）
# 直接往 link.commands 写 enable/throttle，跑真实的 bridge.step()。
# 用法：REPL 里 import main; main.selftest()  或下面 RUN_SELFTEST=True
# ============================================================
RUN_SELFTEST = False
TEST_CHANNELS = [0]         # 要测哪几路 (0~3)
TEST_THR = 1900                 # 测试油门（先拆桨！）
TEST_SPIN_MS = 10000
TEST_ARM_MS = 3000
TEST_EDT_WAIT_MS = 1500        # arm 后等待 EDT 自动开启的时间(油门0)


def _src_of(s):
    if s.bidir_valid and s.kiss_valid:
        return "BD+KISS"
    if s.bidir_valid:
        return "BD"
    if s.kiss_valid:
        return "KISS"
    return "无回传"


def selftest():
    bridge = EscBridge()
    link = bridge.link
    # ch -> [最大erpm, 见过的最大V, 见过的最大I, 见过的最大T]
    result = {}

    def run(ms, thr_list, tag, watch=None):
        """thr_list[i]=None 表示该路不 enable。"""
        t_end = time.ticks_add(time.ticks_ms(), ms)
        last = time.ticks_ms()
        while time.ticks_diff(t_end, time.ticks_ms()) > 0:
            now = time.ticks_ms()
            for i in range(MAX_ESC):
                c = link.commands[i]
                c.enabled = thr_list[i] is not None
                c.throttle = thr_list[i] or 0
            link.last_rx_ms = now          # 防止 100ms 软超时清零油门
            bridge.step()

            if watch is not None:
                s = bridge.statuses[watch]
                r = result[watch]
                r[0] = max(r[0], s.erpm)
                r[1] = max(r[1], s.voltage)
                r[2] = max(r[2], s.current)
                r[3] = max(r[3], s.temperature)

            if time.ticks_diff(now, last) >= 500:
                last = now
                print("[%s]" % tag)
                for i in range(MAX_ESC):
                    c = link.commands[i]
                    s = bridge.statuses[i]
                    print("  ESC%d %s en=%d thr=%4d | %-7s erpm=%7d T=%3d V=%5.2f I=%5.2f"
                          % (i + 1,
                             "BD " if bridge.channel_mode[i] == MODE_BDSHOT else "SIG",
                             c.enabled, c.throttle, _src_of(s),
                             s.erpm, s.temperature, s.voltage, s.current))
            time.sleep_ms(1)

    zero = [0] * MAX_ESC
    try:
        print("!! 将让电机转动，确认已拆桨 !!  3 秒后开始")
        time.sleep(3)

        print("== arm：所有路 enable，油门0，%d ms ==" % TEST_ARM_MS)
        run(TEST_ARM_MS, zero, "arm")

        print("\n== 等待 EDT 自动开启（油门0，%d ms）==" % TEST_EDT_WAIT_MS)
        run(TEST_EDT_WAIT_MS, zero, "EDT")
        if bridge.bdshot is not None:
            for i in range(MAX_ESC):
                if bridge.channel_mode[i] == MODE_BDSHOT and bridge.bdshot.is_motor_active(i):
                    m = bridge.bdshot.motors[i]
                    print("  ESC%d edt=%s tries=%d types=%s"
                          % (i + 1, m.edt_enabled, bridge._edt_tries[i], m.type_counts))

        for ch in TEST_CHANNELS:
            print("\n== 单独转 ESC%d，油门 %d ==" % (ch + 1, TEST_THR))
            result[ch] = [0, 0.0, 0.0, 0]
            thr = list(zero)
            thr[ch] = TEST_THR
            run(TEST_SPIN_MS, thr, "ESC%d 转" % (ch + 1), watch=ch)
            run(1500, zero, "停")

        print("\n== 所选路同时转，油门 %d ==" % TEST_THR)
        thr = list(zero)
        for ch in TEST_CHANNELS:
            thr[ch] = TEST_THR
        run(TEST_SPIN_MS, thr, "同时转")

        print("\n== 停转后 erpm 归零检查（等 %d ms）==" % (ZERO_ERPM_MS + 1000))
        run(ZERO_ERPM_MS + 1000, zero, "停转")

    except KeyboardInterrupt:
        print("用户中断")
    finally:
        print("\n停机")
        try:
            run(300, [None] * MAX_ESC, "停机")
        except Exception:
            pass
        for s in bridge.signal_escs:
            if s is not None:
                try:
                    s.set_throttle(0)
                    s.stop()
                except Exception:
                    pass
        if bridge.bdshot is not None:
            try:
                bridge.bdshot.stop()
            except Exception:
                pass

    print("\n========== 汇总 ==========")
    for ch in TEST_CHANNELS:
        erpm, v, a, t = result.get(ch, [0, 0.0, 0.0, 0])
        mode = "BDShot" if bridge.channel_mode[ch] == MODE_BDSHOT else "已降级Signal_DShot"
        edt = None
        if bridge.bdshot is not None:
            edt = bridge.bdshot.motors[ch].edt_enabled
        print("ESC%d: %s | 最大erpm=%d | 最大V=%.2f I=%.2f T=%d | edt=%s | %s"
              % (ch + 1, mode, erpm, v, a, t, edt,
                 "erpm OK" if erpm > 0 else "没有 erpm 回传"))
        if v == 0:
            print("       ↑ 电压为0: edt 不是 True 就是 EDT 没开; 且 KISS 线也没数据")


if __name__ == "__main__":
    if RUN_SELFTEST:
        selftest()      # 测试用
    else:
        main()          # 正式运行
