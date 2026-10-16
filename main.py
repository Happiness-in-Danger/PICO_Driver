"""
main.py —— PICO_Driver 系统调度层

整体数据流:

    H743
      | UART 921600 (Uart_link.py 协议, 含ENABLE/DISABLE)
      v
    Pico main.py
      | 每路: enabled / throttle
      v
    BDShot.py (主方案, Bidirectional DShot600, PIO1+PIO2, 固定4路)
      | ESC telemetry (erpm/voltage/current/temperature)
      v
    合并 KISS_Telemetry.py (备用遥测, PIO0, 4路常驻运行)
      v
    EscStatus (Uart_link.py) -> UART -> H743

硬件规模:
    固定 MAX_ESC=4，4路硬件资源(BDShot TX/RX、KISS RX)在开机时
    全部初始化好、随时待命。H743实际用几路，通过UART命令帧里的
    enable位随时启用/禁用，Pico程序本身不需要因为"少接几个电机"
    而修改代码或重新烧录。

故障降级(单路独立):
    某一路 BDShot 遥测异常(该路超过 BDSHOT_STALE_MS 没有新的有效帧)
        -> 只把这一路切到 Signal_DShot(单向DShot TX)，
           其余路继续用 BDShot，互不影响。
        -> 遥测转为依赖 KISS_Telemetry (PIO0，与TX方案无关，一直在跑)。
        -> 不做运行时自动恢复，只能重启后重新走一次BDShot初始化。
    BDShot 整体初始化失败(比如PIO资源冲突/硬件异常)
        -> 4路全部直接使用 Signal_DShot，遥测全部依赖 KISS_Telemetry。

enable/disable:
    - H743可以在运行过程中随时对任意一路发 enable/disable。
    - disabled的通道，无论BDShot还是Signal_DShot，实际下发的throttle
      都强制为0（在 Uart_link.py._apply_cmd_frame 和本文件双重保证）。
    - UART链路超时：
        100ms 无有效命令  -> 所有throttle清零，enabled状态保留
        2000ms 无有效命令 -> 所有throttle清零，同时enabled也被清除
      (以上均在 Uart_link.UartLink.poll_commands() 里实现)

本文件不修改 BDShot.py 的 PIO 汇编/DMA/底层时序，只使用它新增的
disable_motor()/is_motor_active() 这两个纯Python层面的接口做单路降级。
"""

import time
import micropython

from Uart_link import (
    UartLink,
    EscStatus,
    MAX_ESC,
)
import KISS_Telemetry as kiss          # import 即启动 PIO0 的4路 KISS RX，常驻备用遥测源
from BDShot import DShotBus
import Signal_DShot as sig_mod         # 只用到其中的 DShot600 类，真正实例化延迟到需要降级时

micropython.alloc_emergency_exception_buf(100)

# ============================================================
# 配置
# ============================================================
# 硬件上实际焊好的4路信号线引脚，顺序固定对应 ESC0~ESC3。
# 以当前项目最新硬件为准，不要恢复旧版GPIO。
MOTOR_PINS = [3, 7, 11, 15]
ESC_NAMES = ["ESC%d" % (i + 1) for i in range(MAX_ESC)]

if len(MOTOR_PINS) != MAX_ESC:
    raise ValueError(
        "MOTOR_PINS 配置了 %d 路引脚，MAX_ESC=%d 不一致" % (len(MOTOR_PINS), MAX_ESC)
    )

# 单路降级时，Signal_DShot 复用该路原本被BDShot占用的TX状态机
# (global sm_id, pio_num)，与 BDShot.py 里 _SLOTS=((1,0),(1,1),(2,0),(2,1))
# 算出的 tx_sm 完全对应：
#   ESC0(motor0) -> PIO1 local_pair0 -> tx_sm = 1*4+0*2 = 4
#   ESC1(motor1) -> PIO1 local_pair1 -> tx_sm = 1*4+1*2 = 6
#   ESC2(motor2) -> PIO2 local_pair0 -> tx_sm = 2*4+0*2 = 8
#   ESC3(motor3) -> PIO2 local_pair1 -> tx_sm = 2*4+1*2 = 10
# disable_motor() 会先释放该 tx_sm/rx_sm，这里再原地复用 tx_sm 这一个
# 状态机给 Signal_DShot，不会跟其余仍在用BDShot的通道冲突。
DEGRADE_SM = [(4, 1), (6, 1), (8, 2), (10, 2)]

POLE_PAIRS = 1
UPDATE_HZ = 2000                # BDShot 内部 Timer 更新率

BDSHOT_STALE_MS = 300           # 单路 BDShot 遥测超过这个时间没有新的有效帧 -> 该路判定异常，触发单路降级
KISS_STALE_MS = 1000            # 直接复用 KissTelemParser.is_alive() 的默认超时量级

STATUS_PERIOD_MS = 2            # Pico -> H743 状态帧发送周期(每路)，MAX_ESC路一轮共 2*MAX_ESC ms

MODE_BDSHOT = 0
MODE_BACKUP = 1


# ============================================================
# BDShot 遥测新鲜度旁路监控（不修改 BDShot.py 底层，只在外部采样计数器）
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
        self.signal_escs = [None] * MAX_ESC   # 只有降级的那一路才会被创建

        self._init_bdshot()

    # -------------------- 初始化 / 单路降级切换 --------------------
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
        """只把第i路从BDShot切到Signal_DShot，其余路不受影响。"""
        if self.channel_mode[i] == MODE_BACKUP:
            return
        print("[main] ESC%d 单路降级: BDShot -> Signal_DShot" % (i + 1))

        if self.bdshot is not None and self.bdshot.is_motor_active(i):
            try:
                self.bdshot.disable_motor(i)   # 只关这一路的tx_sm/rx_sm，其余路PIO SM不受影响
            except Exception as e:
                print("[main] bdshot.disable_motor(%d) 异常(继续降级):" % i, e)

        sm_id, pio_num = DEGRADE_SM[i]
        self.signal_escs[i] = sig_mod.DShot600(
            pin_num=MOTOR_PINS[i], sm_id=sm_id, pio_num=pio_num
        )
        self.channel_mode[i] = MODE_BACKUP

    # -------------------- 每个主循环 tick 调用一次 --------------------
    def step(self):
        self.link.poll_commands()
        self._drive_escs()
        self._check_bdshot_health()
        self._send_status_slice()

    def _drive_escs(self):
        for i in range(MAX_ESC):
            cmd = self.link.commands[i]
            # disabled通道throttle已经在Uart_link里强制为0，这里再兜底一次
            thr = cmd.throttle if cmd.enabled else 0

            if self.channel_mode[i] == MODE_BDSHOT and self.bdshot is not None \
                    and self.bdshot.is_motor_active(i):
                self.bdshot.set_throttle(i, thr)
            elif self.signal_escs[i] is not None:
                self.signal_escs[i].set_throttle(thr)

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
        # 每 STATUS_PERIOD_MS 发一路状态帧，round-robin 由
        # Uart_link.UartLink.send_next_status() 自己维护 _send_idx。
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
        kiss_name = ESC_NAMES[i]
        kp = kiss.escs[kiss_name]
        kiss_alive = kp.is_alive(KISS_STALE_MS)

        bd_alive = False
        if self.channel_mode[i] == MODE_BDSHOT and self.bdshot is not None \
                and self.bdshot.is_motor_active(i):
            bd_alive = self.bdshot_health.is_alive(i)

        st.enabled = self.link.commands[i].enabled

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