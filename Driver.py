'''
注意此文件运行在主控部份！！！注意此文件运行在主控部份！！！注意此文件运行在主控部份！！！
STM32H743 主控侧 UART 代码
H743通过此文件与Pico通讯，Pico 是负责电调控制和电调遥测的从机。

===================== 协议说明 =====================
固定支持 MAX_ESC=4 路 ESC（与Pico侧Uart_link.py的MAX_ESC=4完全对应，
两边都固定为4，不再需要"改一个数字要记得改另一边"）。

命令帧 (H743 -> Pico)，CMD_FRAME_LEN=5:
    buf[0] = SYNC_CMD (0xA5)
    buf[1] = device_id (0~3)
    buf[2] = bit7: enable(1=启用该路 / 0=禁用该路)
             bit6~3: 保留(0)
             bit2~0: throttle 的 bit10~8 (高3位)
    buf[3] = throttle 的 bit7~0 (低8位)
    buf[4] = crc8(buf[0:4])

状态帧 (Pico -> H743)，STATUS_FRAME_LEN=13:
    buf[0] = SYNC_STATUS (0x5A)
    buf[1] = device_id
    buf[2] = flags: bit0=BDShot有效  bit1=KISS有效  bit2=该路已enable
    buf[3:7]  = erpm (uint32, 大端)
    buf[7]    = temperature (int8)
    buf[8:10] = voltage*100 (uint16, 大端)
    buf[10:12]= current*100 (uint16, 大端)
    buf[12]   = crc8(buf[0:12])

===================== 超时规则(Pico侧实现，这里只是文档) ==============
    100ms  Pico未收到合法命令 -> 该端throttle清零，enabled保留
    2000ms Pico未收到合法命令 -> 该端throttle清零，同时enabled清除
    本文件(H743侧)同样对"多久没收到Pico状态帧"做超时判定(见link_ok)。
'''
from pyb import UART
import time

# ================= 配置 =================
UART_ID = 6
UART_BAUD = 921600

# 硬件上限固定为4路，必须和 Pico 端 Uart_link.py 的 MAX_ESC 保持一致。
# 两边都固定写死为4，不再是"可调数字"，所以不存在改一边忘改另一边的问题。
MAX_ESC = 4

SYNC_CMD = 0xA5
SYNC_STATUS = 0x5A

CMD_FRAME_LEN = 5
STATUS_FRAME_LEN = 13

STATUS_FLAG_BIDIR_VALID = 0x01
STATUS_FLAG_KISS_VALID = 0x02
STATUS_FLAG_ENABLED = 0x04

CMD_ENABLE_BIT = 0x80
CMD_THR_HI_MASK = 0x07
THROTTLE_MIN = 48       # 电机有效调速下限，0 仅用于停机
THROTTLE_MAX = 0x07FF   # 电机有效调速上限


def crc8_update(crc, byte):
    crc ^= byte
    for _ in range(8):
        crc = (0x07 ^ (crc << 1)) if (crc & 0x80) else (crc << 1)
        crc &= 0xFF
    return crc


def crc8(buf):
    crc = 0
    for b in buf:
        crc = crc8_update(crc, b)
    return crc


class EscStatus:
    """H743侧接收到的、来自Pico的电调状态镜像"""
    __slots__ = (
        "bidir_valid",
        "kiss_valid",
        "enabled",
        "raw_erpm",
        "temperature",
        "voltage",
        "current",
    )

    def __init__(self):
        self.bidir_valid = False
        self.kiss_valid = False
        self.enabled = False
        self.raw_erpm = 0
        self.temperature = 0
        self.voltage = 0.0
        self.current = 0.0


class EscCommand:
    """H743侧要下发给Pico某一路的期望状态"""
    __slots__ = ("throttle", "enabled")

    def __init__(self):
        self.throttle = 0
        self.enabled = False


class FrameReceiver:
    def __init__(self, sync_byte, frame_len):
        self.sync = sync_byte
        self.frame_len = frame_len
        self.buf = bytearray()

    def feed(self, data):
        if data:
            self.buf.extend(data)

        frames = []

        while True:
            while len(self.buf) > 0 and self.buf[0] != self.sync:
                del self.buf[0]

            if len(self.buf) < self.frame_len:
                break

            candidate = bytes(self.buf[:self.frame_len])

            if crc8(candidate[:-1]) == candidate[-1]:
                del self.buf[:self.frame_len]
                frames.append(candidate)
            else:
                del self.buf[0]

        return frames


# Driver link 全局实例（供 main.py / 设备模块共享，避免循环导入）
_driver_link = None

def get_driver_link():
    """惰性初始化 Driver link（失败降级）"""
    global _driver_link
    if _driver_link is None:
        try:
            _driver_link = UartLink()
            print("[Driver] UART6 link 已初始化")
        except Exception as e:
            print("[Driver] 初始化失败，链路降级:", e)
    return _driver_link


class UartLink:
    def __init__(self):
        self.uart = UART(UART_ID, UART_BAUD)

        self.statuses = [EscStatus() for _ in range(MAX_ESC)]
        self.commands = [EscCommand() for _ in range(MAX_ESC)]
        self.link_ok = False

        self._rx = FrameReceiver(SYNC_STATUS, STATUS_FRAME_LEN)

        self.last_rx_ms = time.ticks_ms()
        self.LINK_TIMEOUT_MS = 100

    # -------------------- 接收 Pico 状态帧 --------------------
    def poll_status(self):
        n = self.uart.any()
        data = self.uart.read(n) if n else None

        frames = self._rx.feed(data)

        for f in frames:
            self._apply_status_frame(f)
            self.last_rx_ms = time.ticks_ms()
            self.link_ok = True

        if time.ticks_diff(
            time.ticks_ms(),
            self.last_rx_ms
        ) > self.LINK_TIMEOUT_MS:
            self.link_ok = False

        return self.link_ok

    def _apply_status_frame(self, buf):
        device_id = buf[1]

        if device_id >= MAX_ESC:
            return

        st = self.statuses[device_id]

        flags = buf[2]

        st.bidir_valid = bool(flags & STATUS_FLAG_BIDIR_VALID)
        st.kiss_valid = bool(flags & STATUS_FLAG_KISS_VALID)
        st.enabled = bool(flags & STATUS_FLAG_ENABLED)

        st.raw_erpm = (buf[3] << 24) | (buf[4] << 16) | (buf[5] << 8) | buf[6]
        st.temperature = buf[7]
        st.voltage = ((buf[8] << 8) | buf[9]) / 100.0
        st.current = ((buf[10] << 8) | buf[11]) / 100.0

    # -------------------- 下发命令给 Pico --------------------
    def enable(self, device_id):
        """启用某一路(下一次send_command/send_all_commands时生效)"""
        self.commands[device_id].enabled = True

    def disable(self, device_id):
        """禁用某一路，同时把期望油门清零"""
        self.commands[device_id].enabled = False
        self.commands[device_id].throttle = 0

    def set_throttle(self, device_id, throttle):
        """只有enabled=True时，这个throttle才会真正被Pico采用
        
        Args:
            device_id: 0-3
            throttle: 0-2047（油门值）
        """
        throttle = int(throttle)
        # 0 表示停机；只要请求运行，就不得落入 1-47 的无效区间。
        self.commands[device_id].throttle = (
            0 if throttle <= 0 else max(THROTTLE_MIN, min(THROTTLE_MAX, throttle))
        )

    def send_command(self, device_id, cmd=None):
        """把 self.commands[device_id] (或显式传入的cmd) 编码发送给Pico"""
        c = cmd if cmd is not None else self.commands[device_id]
        requested = int(c.throttle)
        thr = (0 if requested <= 0 else
               max(THROTTLE_MIN, min(THROTTLE_MAX, requested))) if c.enabled else 0

        buf = bytearray(CMD_FRAME_LEN)
        buf[0] = SYNC_CMD
        buf[1] = device_id
        buf[2] = (CMD_ENABLE_BIT if c.enabled else 0x00) | ((thr >> 8) & CMD_THR_HI_MASK)
        buf[3] = thr & 0xFF
        buf[-1] = crc8(buf[:-1])

        self.uart.write(buf)

    def send_all_commands(self):
        for device_id in range(MAX_ESC):
            self.send_command(device_id)

    # -------------------- Step5a: ESC 遥测数据换算与写入 state --------------------
    def update_telemetry_to_state(self, state_module):
        """
        将 ESC 遥测数据换算后写入 state.state_data[16-31]

        Args:
            state_module: Board.state 模块引用

        职责：
        - erpm → rpm 换算（根据 state.ESC_TELEMETRY_CFG 列表配置）
        - 电压/电流按配置倍率转换后 ×10 写入
        - 无效通道 / 链路断 / 配置禁用字段写 999
        """
        if not self.link_ok:
            # 链路断时所有 ESC 遥测标记为无效
            for i in range(16, 32):
                state_module.state_data[i] = 999
            return

        cfg_table = state_module.ESC_TELEMETRY_CFG

        # 4 个 ESC × 4 字段
        for esc_id in range(MAX_ESC):
            status = self.statuses[esc_id]
            base_idx = 16 + esc_id * 4  # ESC0→16, ESC1→20, ESC2→24, ESC3→28

            cfg = cfg_table[esc_id] if esc_id < len(cfg_table) else {}
            pole_pairs = cfg.get("pole_pairs", 1) or 1
            erpm_scale = cfg.get("erpm_scale", 1.0) or 1.0
            voltage_scale = cfg.get("voltage_scale", 1.0) or 1.0
            current_scale = cfg.get("current_scale", 1.0) or 1.0

            # erpm → rpm 换算（可配置）
            if cfg.get("erpm", True) and status.raw_erpm != 0 and status.bidir_valid:
                rpm = int(status.raw_erpm / pole_pairs * erpm_scale)
            else:
                rpm = 999

            # 温度
            if cfg.get("temp", True) and status.kiss_valid:
                temp = status.temperature
            else:
                temp = 999

            # 电压：帧内已是 V，按配置倍率后 ×10 写入
            if cfg.get("voltage", True) and status.kiss_valid:
                voltage = int(status.voltage * voltage_scale * 10)
            else:
                voltage = 999

            # 电流：帧内已是 A，按配置倍率后 ×10 写入；ESC1/ESC2 由 ACS712 统一采样
            if (cfg.get("current", True) and status.kiss_valid and
                    status.current != 0 and esc_id not in (1, 2)):
                current = int(status.current * current_scale * 10)
            else:
                current = 999

            # 写入 state_data[16-31]
            state_module.state_data[base_idx + 0] = rpm      # 反馈机械 rpm
            state_module.state_data[base_idx + 1] = temp     # 温度 (°C)
            state_module.state_data[base_idx + 2] = voltage  # 电压×10
            state_module.state_data[base_idx + 3] = current  # 电流×10


# ================= 使用示例 =================
if __name__ == "__main__":
    link = UartLink()

    # 示例: 只启用ESC0/ESC1，ESC2/ESC3保持禁用(throttle强制为0)
    link.enable(0)
    link.enable(1)

    last_cmd_ms = time.ticks_ms()
    CMD_PERIOD_MS = 2
    send_idx = 0

    while True:
        link.poll_status()

        now = time.ticks_ms()

        if time.ticks_diff(now, last_cmd_ms) >= CMD_PERIOD_MS:
            link.send_command(send_idx)
            send_idx = (send_idx + 1) % MAX_ESC
            last_cmd_ms = now

        time.sleep_ms(1)
