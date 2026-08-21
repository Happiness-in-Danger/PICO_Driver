'''
注意此文件运行在主控部份！！！注意此文件运行在主控部份！！！注意此文件运行在主控部份！！！
STM32H743 主控侧 UART 代码
H743通过此文件与Pico通讯，Pico 是负责电调控制和电调遥测的从机。
'''
from pyb import UART
import time

# ================= 配置 =================
UART_ID = 2
UART_BAUD = 921600

# [重要] NUM_ESC 必须和 Pico 端 Uart_link.py 里的 NUM_ESC 完全一致。
# 两块MCU是各自独立的代码，没有办法共享同一个常量，只能人工同步。
# 改了这里一定要记得同时改 Pico 端，否则 device_id 过滤范围两边不一致，
# 会出现"Pico发的某些ESC状态帧被H743当非法ID丢弃"或反之的情况。
NUM_ESC = 2

SYNC_CMD = 0xA5
SYNC_STATUS = 0x5A

CMD_FRAME_LEN = 5
STATUS_FRAME_LEN = 13

STATUS_FLAG_BIDIR_VALID = 0x01
STATUS_FLAG_KISS_VALID = 0x02


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
    # [FIX] 原来 __slots__ 里漏了 bidir_valid / kiss_valid，
    # 但 _apply_status_frame() 里会给这两个属性赋值——用了 __slots__
    # 却没声明就赋值会直接抛 AttributeError。
    # 也就是说原代码只要收到一帧合法的Pico状态帧就会立刻崩溃，
    # 这是和ESC数量无关的独立bug，必须修。
    __slots__ = (
        "bidir_valid",
        "kiss_valid",
        "erpm",
        "temperature",
        "voltage",
        "current",
    )

    def __init__(self):
        self.bidir_valid = False
        self.kiss_valid = False
        self.erpm = 0
        self.temperature = 0
        self.voltage = 0.0
        self.current = 0.0


class EscCommand:
    __slots__ = ("throttle",)

    def __init__(self):
        self.throttle = 0


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


class UartLink:
    def __init__(self):
        self.uart = UART(UART_ID, UART_BAUD)

        self.statuses = [EscStatus() for _ in range(NUM_ESC)]
        self.link_ok = False

        self._rx = FrameReceiver(SYNC_STATUS, STATUS_FRAME_LEN)

        self.last_rx_ms = time.ticks_ms()
        self.LINK_TIMEOUT_MS = 100

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

        if device_id >= NUM_ESC:
            return

        st = self.statuses[device_id]

        flags = buf[2]

        st.bidir_valid = bool(flags & STATUS_FLAG_BIDIR_VALID)
        st.kiss_valid = bool(flags & STATUS_FLAG_KISS_VALID)

        st.erpm = (buf[3] << 8) | buf[4]
        st.temperature = buf[5]
        st.voltage = ((buf[6] << 8) | buf[7]) / 100.0
        st.current = ((buf[8] << 8) | buf[9]) / 100.0

    def send_command(self, device_id, cmd):
        thr = max(0, min(0x07FF, int(cmd.throttle)))

        buf = bytearray(CMD_FRAME_LEN)

        buf[0] = SYNC_CMD
        buf[1] = device_id
        buf[2] = (thr >> 8) & 0xFF
        buf[3] = thr & 0xFF
        buf[-1] = crc8(buf[:-1])

        self.uart.write(buf)


# ================= 使用示例 =================
if __name__ == "__main__":
    link = UartLink()

    commands = [EscCommand() for _ in range(NUM_ESC)]

    last_cmd_ms = time.ticks_ms()
    CMD_PERIOD_MS = 2
    send_idx = 0

    while True:
        link.poll_status()

        now = time.ticks_ms()

        if time.ticks_diff(now, last_cmd_ms) >= CMD_PERIOD_MS:
            link.send_command(send_idx, commands[send_idx])
            send_idx = (send_idx + 1) % NUM_ESC
            last_cmd_ms = now

        time.sleep_ms(1)
