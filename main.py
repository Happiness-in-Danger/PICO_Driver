"""
main.py —— PICO_Driver 系统调度层

整体数据流:

    H743
      | UART 921600 (Uart_link.py 协议)
      v
    Pico main.py
      | throttle[4]
      v
    BDShot.py (主方案, Bidirectional DShot600, PIO1+PIO2)
      | ESC telemetry (erpm/voltage/current/temperature)
      v
    合并 KISS_Telemetry.py (备用遥测, PIO0, GPIO4~7, 常驻运行)
      v
    EscStatus (Part_link.py) -> UART -> H743

故障降级:

    BDShot 单路遥测失效  -> 该路改用 KISS 遥测，油门继续走 BDShot TX
    BDShot 整体失效/初始化失败
        -> bdshot.stop() 释放 PIO1(SM4~7)
        -> Signal_DShot.DShot600 x4 复用 PIO1(SM4~7) 做单向 DShot TX
        -> 遥测全部依赖 KISS_Telemetry (PIO0, GPIO4~7, 与TX方案无关，一直在跑)

本文件不修改任何现有模块，只做调度/整合。
"""

import time
import micropython
from Uart_link import (
    UartLink,
    EscStatus,
    NUM_ESC,
)
import KISS_Telemetry as kiss          # import 即启动 PIO0 4路 KISS RX (GPIO4~7)，常驻备用遥测源
from BDShot import DShotBus
import Signal_DShot as sig_mod         # 只用到其中的 DShot600 类，真正实例化延迟到需要降级时

micropython.alloc_emergency_exception_buf(100)

# ============================================================
# 配置
# ============================================================
# BDShot(主方案) 与 Signal_DShot(备用TX方案) 共用同一组物理信号线，
# 两者互斥使用(降级时先 stop 主方案再启用备用方案)，不会同时驱动同一GPIO。
# 注意: 必须避开 KISS_Telemetry.py 里硬编码的 GPIO2~6~10~14 (那是独立的
# KISS遥测输入线，需要单独接线到 ESC 的 telemetry 输出脚)。
MOTOR_PINS = [3, 7, 11, 15]

if NUM_ESC > len(MOTOR_PINS_ALL):
    raise ValueError(
        "MOTOR_PINS_ALL 只配置了 %d 路引脚，NUM_ESC=%d 超出，"
        "请先在这里补上对应引脚" % (len(MOTOR_PINS_ALL), NUM_ESC)
    )
 
MOTOR_PINS = MOTOR_PINS_ALL[:NUM_ESC]
ESC_NAMES = ["ESC%d" % (i + 1) for i in range(NUM_ESC)]

POLE_PAIRS = 1
UPDATE_HZ = 2000                # BDShot 内部 Timer 更新率

BDSHOT_STALE_MS = 300           # 单路 BDShot 遥测超过这个时间没有新的有效帧 -> 判定该路 stale
BDSHOT_GLOBAL_FAIL_MS = 1000     # 4路同时 stale 超过这个时间 -> 判定 BDShot 整体失效，触发降级
KISS_STALE_MS = 1000            # 直接复用 KissTelemParser.is_alive() 的默认超时量级

STATUS_PERIOD_MS = 2            # Pico -> H743 状态帧发送周期(每路)，4路一轮共8ms，
                                 # 13字节@921600bps约140us/帧，带宽远够用，可按需调大调小

MODE_BDSHOT = 0
MODE_BACKUP = 1


# ============================================================
# BDShot 遥测新鲜度旁路监控（不修改 BDShot.py，只在外部采样计数器）
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

    def all_stale_duration_ms(self):
        """用"最近一次任意电机有新遥测"的时间做基准，
        返回"所有电机都没有新遥测"已经持续了多久。"""
        newest = max(self._last_change_ms)
        return time.ticks_diff(time.ticks_ms(), newest)


# ============================================================
# 系统调度主体
# ============================================================
class EscBridge:
    def __init__(self):
        self.link = UartLink()
        self.throttle = [0, 0, 0, 0]

        self.mode = MODE_BDSHOT
        self.bdshot = None
        self.bdshot_health = None
        self.signal_escs = None   # 只有降级后才会创建

        self._init_bdshot()

        self.statuses = [EscStatus() for _ in range(NUM_ESC)]

    # -------------------- 初始化 / 降级切换 --------------------
    def _init_bdshot(self):
        try:
            self.bdshot = DShotBus(
                motor_pins=MOTOR_PINS,
                update_hz=UPDATE_HZ,
                names=["ESC1", "ESC2", "ESC3", "ESC4"],
                pole_pairs=POLE_PAIRS,
            )
            self.bdshot_health = BdshotHealthTracker(NUM_ESC)
            self.mode = MODE_BDSHOT
            print("[main] BDShot 主方案初始化成功")
        except Exception as e:
            print("[main] BDShot 初始化失败，直接降级到备用方案:", e)
            self.bdshot = None
            self.bdshot_health = None
            self._switch_to_backup()

    def _switch_to_backup(self):
        if self.mode == MODE_BACKUP:
            return
        print("[main] 降级: Signal_DShot(TX) + KISS_Telemetry(RX)")

        if self.bdshot is not None:
            try:
                self.bdshot.stop()   # 关 Timer + 关所有 tx_sm/rx_sm，释放 PIO1(SM4~7)/PIO2(SM8~11)
            except Exception as e:
                print("[main] bdshot.stop() 异常(继续降级):", e)
            self.bdshot = None
            self.bdshot_health = None

        # Signal_DShot 改用 PIO1 的4个SM(全局id 4~7)，此时 BDShot 已经
        # stop()，这4个SM已经释放，不会和残留的BDShot状态冲突。
        # PIO2(SM8~11)在备用模式下完全空闲，未使用。
        self.signal_escs = []
        for i, pin in enumerate(MOTOR_PINS):
            esc = sig_mod.DShot600(pin_num=pin, sm_id=4 + i, pio_num=1)
            self.signal_escs.append(esc)
        self.mode = MODE_BACKUP

    # -------------------- 每个主循环 tick 调用一次 --------------------
    def step(self):
        self._update_commands()
        self._drive_escs()
        if self.mode == MODE_BDSHOT:
            self._check_bdshot_health()
        self._send_status_slice()

    def _update_commands(self):
        # UartLink.poll_commands() 内部已经实现:
        #   - 从UART读数据 -> FrameReceiver切帧+CRC8校验
        #   - 应用到 self.link.commands[i].throttle
        #   - 超过 LINK_TIMEOUT_MS(默认100ms) 没收到有效命令 -> 全部油门清零
        # 这正是 Pico 侧 failsafe 的需求，直接复用，不重复实现。
        self.link.poll_commands()
        for i in range(NUM_ESC):
            self.throttle[i] = self.link.commands[i].throttle

    def _drive_escs(self):
        if self.mode == MODE_BDSHOT and self.bdshot is not None:
            for i in range(NUM_ESC):
                self.bdshot.set_throttle(i, self.throttle[i])
        elif self.mode == MODE_BACKUP and self.signal_escs is not None:
            for i in range(NUM_ESC):
                self.signal_escs[i].set_throttle(self.throttle[i])

    def _check_bdshot_health(self):
        self.bdshot_health.update(self.bdshot.motors)
        if self.bdshot_health.all_stale_duration_ms() > BDSHOT_GLOBAL_FAIL_MS:
            print(
                "[main] 4路BDShot遥测同时失联超过 %dms，判定BDShot整体失效"
                % BDSHOT_GLOBAL_FAIL_MS
            )
            self._switch_to_backup()

    def _send_status_slice(self):
        # 每 STATUS_PERIOD_MS 发一路状态帧，round-robin 由
        # Part_link.UartLink.send_next_status() 自己维护 _send_idx。
        now = time.ticks_ms()
        if not hasattr(self, "_last_status_ms"):
            self._last_status_ms = now
        if time.ticks_diff(now, self._last_status_ms) < STATUS_PERIOD_MS:
            return
        self._last_status_ms = now

        i = self.link._send_idx
        self._fill_status(i, self.statuses[i])
        self.link.send_next_status(self.statuses)

    def _fill_status(self, i, st):
        kiss_name = "ESC%d" % (i + 1)
        kp = kiss.escs[kiss_name]
        kiss_alive = kp.is_alive(KISS_STALE_MS)

        bd_alive = False
        if self.mode == MODE_BDSHOT and self.bdshot is not None:
            bd_alive = self.bdshot_health.is_alive(i)

        if bd_alive:
            m = self.bdshot.motors[i]
            st.erpm = m.latest_erpm or 0
            st.temperature = m.latest_temp_c or 0
            st.voltage = m.latest_voltage_v or 0.0
            st.current = m.latest_current_a or 0.0
            st.bidir_valid = True
            st.kiss_valid = False
        elif kiss_alive:
            st.erpm = kp.erpm
            st.temperature = kp.temperature
            st.voltage = kp.voltage
            st.current = kp.current
            st.bidir_valid = False
            st.kiss_valid = True
        else:
            st.erpm = 0
            st.temperature = 0
            st.voltage = 0.0
            st.current = 0.0
            st.bidir_valid = False
            st.kiss_valid = False


def main():
    bridge = EscBridge()
    print("[main] 系统启动完成，进入主循环")
    while True:
        bridge.step()
        time.sleep_ms(1)


if __name__ == "__main__":
    main()
