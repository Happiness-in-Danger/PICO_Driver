#uart_link.py
from machine import UART, Pin
import time

# ================= 配置 =================
UART_ID = 0
UART_TX = 16
UART_RX = 17
UART_BAUD = 921600

# NUM_ESC 是整个 Pico 项目里 ESC 数量的唯一信息源：
# KISS_Telemetry.py / main.py 都从这里 import NUM_ESC，不再各自硬编码。
# 改这一个数字（2/3/4）即可切换实际运行的 ESC 路数。
# 注意：H743 端 Driver.py 是独立的MCU/独立代码，没有共享import的办法，
# 必须手动把 Driver.py 里的 NUM_ESC 改成完全相同的数值，否则两端协议对不上。
NUM_ESC = 3

SYNC_CMD = 0xA5
SYNC_STATUS = 0x5A

CMD_FRAME_LEN = 5
STATUS_FRAME_LEN = 13   # 原11字节 + 电流2字节

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
    __slots__ = ("erpm", "temperature", "voltage", "current",
                 "bidir_valid", "kiss_valid")

    def __init__(self):
        self.erpm = 0
        self.temperature = 0
        self.voltage = 0.0
        self.current = 0.0
        self.bidir_valid = False
        self.kiss_valid = False

    def pack(self, device_id):
        # [eRPM uint16->uint32] 原来buf[10]/buf[11]是恒为0的保留字节，
        # 现在把它们并入eRPM，让eRPM变成完整的4字节uint32，帧总长度
        # 仍然是13字节不变，不影响UART带宽和发送周期。
        flags = 0
        if self.bidir_valid:
            flags |= STATUS_FLAG_BIDIR_VALID
        if self.kiss_valid:
            flags |= STATUS_FLAG_KISS_VALID
        erpm = max(0, min(0xFFFFFFFF, int(self.erpm)))
        volt = max(0, min(0xFFFF, int(self.voltage * 100)))
        curr = max(0, min(0xFFFF, int(self.current * 100)))

        buf = bytearray(STATUS_FRAME_LEN)
        buf[0] = SYNC_STATUS
        buf[1] = device_id
        buf[2] = flags
        buf[3] = (erpm >> 24) & 0xFF
        buf[4] = (erpm >> 16) & 0xFF
        buf[5] = (erpm >> 8) & 0xFF
        buf[6] = erpm & 0xFF
        buf[7] = self.temperature & 0xFF
        buf[8] = (volt >> 8) & 0xFF
        buf[9] = volt & 0xFF
        buf[10] = (curr >> 8) & 0xFF
        buf[11] = curr & 0xFF
        buf[-1] = crc8(buf[:-1])
        return buf


class EscCommand:
    __slots__ = ("throttle",)

    def __init__(self):
        self.throttle = 0


class FrameReceiver:
    """按sync字节+固定长度+CRC8从字节流里切帧，自动丢字节重新同步。
    返回本次feed()里解析出的所有合法帧（不能只留最新一帧，
    否则同一批数据里不同设备的帧会被互相顶掉）。"""

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
        self.uart = UART(UART_ID, baudrate=UART_BAUD,
                          tx=Pin(UART_TX), rx=Pin(UART_RX))

        self.commands = [EscCommand() for _ in range(NUM_ESC)]
        self.link_ok = False

        self._rx = FrameReceiver(SYNC_CMD, CMD_FRAME_LEN)
        self.last_rx_ms = time.ticks_ms()
        self.LINK_TIMEOUT_MS = 100

        self._send_idx = 0

    def poll_commands(self):
        n = self.uart.any()
        data = self.uart.read(n) if n else None
        frames = self._rx.feed(data)

        for f in frames:
            self._apply_cmd_frame(f)
            self.last_rx_ms = time.ticks_ms()
            self.link_ok = True

        if time.ticks_diff(time.ticks_ms(), self.last_rx_ms) > self.LINK_TIMEOUT_MS:
            if self.link_ok:
                self._failsafe()
            self.link_ok = False

        return self.link_ok

    def _apply_cmd_frame(self, buf):
        device_id = buf[1]
        if device_id >= NUM_ESC:
            return
        thr = ((buf[2] << 8) | buf[3]) & 0x07FF
        self.commands[device_id].throttle = thr

    def _failsafe(self):
        for cmd in self.commands:
            cmd.throttle = 0

    def send_next_status(self, statuses):
        i = self._send_idx
        buf = statuses[i].pack(i)
        self.uart.write(buf)
        self._send_idx = (i + 1) % NUM_ESC


# ================= 使用示例 =================
if __name__ == "__main__":
    link = UartLink()
    statuses = [EscStatus() for _ in range(NUM_ESC)]

    last_status_ms = time.ticks_ms()
    STATUS_PERIOD_MS = 2

    while True:
        link.poll_commands()

        now = time.ticks_ms()
        if time.ticks_diff(now, last_status_ms) >= STATUS_PERIOD_MS:
            link.send_next_status(statuses)
            last_status_ms = now

        time.sleep_ms(1)
