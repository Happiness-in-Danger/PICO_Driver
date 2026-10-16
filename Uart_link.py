#uart_link.py
"""
Pico 侧 UART 协议实现。
与 H743 侧 Driver.py 的协议保持完全一致，任何协议改动都必须两边同步改。

===================== 协议说明 =====================
固定支持 MAX_ESC=4 路 ESC，不再有 NUM_ESC 配置项。

命令帧 (H743 -> Pico)，CMD_FRAME_LEN=5:
    buf[0] = SYNC_CMD (0xA5)
    buf[1] = device_id (0~3)
    buf[2] = bit7: enable(1=启用该路 / 0=禁用该路)
             bit6~3: 保留(0)
             bit2~0: throttle 的 bit10~8 (高3位)
    buf[3] = throttle 的 bit7~0 (低8位)
    buf[4] = crc8(buf[0:4])
    throttle 范围 0~2047(11bit)，与原协议完全兼容——原来 buf[2] 的
    高5位一直是0(因为 thr>>8 最大是7)，这里只是把最高位(bit7)利用起来
    表示 enable，不占用原有 throttle 编码空间。

状态帧 (Pico -> H743)，STATUS_FRAME_LEN=13，字段布局不变:
    buf[0] = SYNC_STATUS (0x5A)
    buf[1] = device_id
    buf[2] = flags: bit0=BDShot有效  bit1=KISS有效  bit2=该路已enable
    buf[3:7]  = erpm (uint32, 大端)
    buf[7]    = temperature (int8)
    buf[8:10] = voltage*100 (uint16, 大端)
    buf[10:12]= current*100 (uint16, 大端)
    buf[12]   = crc8(buf[0:12])

===================== 超时规则 =====================
    100ms  内没有收到合法命令帧 -> 所有路 throttle 清零，enabled 保留
    2000ms 内没有收到合法命令帧 -> 所有路 throttle 清零，同时 enabled 清除
"""
from machine import UART, Pin
import time

# ================= 配置 =================
UART_ID = 0
UART_TX = 16
UART_RX = 17
UART_BAUD = 921600

# 硬件上限固定为4路，不再有可配置的 NUM_ESC。
# 实际启用几路完全由 H743 通过命令帧里的 enable 位动态决定，
# Pico 侧代码不需要因为"少插几个电机"而修改。
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
                 "bidir_valid", "kiss_valid", "enabled")

    def __init__(self):
        self.erpm = 0
        self.temperature = 0
        self.voltage = 0.0
        self.current = 0.0
        self.bidir_valid = False
        self.kiss_valid = False
        self.enabled = False

    def pack(self, device_id):
        flags = 0
        if self.bidir_valid:
            flags |= STATUS_FLAG_BIDIR_VALID
        if self.kiss_valid:
            flags |= STATUS_FLAG_KISS_VALID
        if self.enabled:
            flags |= STATUS_FLAG_ENABLED

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
    __slots__ = ("throttle", "enabled")

    def __init__(self):
        self.throttle = 0
        self.enabled = False


class FrameReceiver:
    """按sync字节+固定长度+CRC8从字节流里切帧，自动丢字节重新同步。"""

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

        self.commands = [EscCommand() for _ in range(MAX_ESC)]
        self.link_ok = False

        self._rx = FrameReceiver(SYNC_CMD, CMD_FRAME_LEN)
        self.last_rx_ms = time.ticks_ms()
        self.LINK_TIMEOUT_MS = 100         # 软超时: 清throttle, 保留enabled
        self.LINK_HARD_TIMEOUT_MS = 2000   # 硬超时: 清throttle, 同时清enabled

        self._send_idx = 0

    def poll_commands(self):
        n = self.uart.any()
        data = self.uart.read(n) if n else None
        frames = self._rx.feed(data)

        for f in frames:
            self._apply_cmd_frame(f)
            self.last_rx_ms = time.ticks_ms()
            self.link_ok = True

        since = time.ticks_diff(time.ticks_ms(), self.last_rx_ms)
        if since > self.LINK_HARD_TIMEOUT_MS:
            self._hard_failsafe()
            self.link_ok = False
        elif since > self.LINK_TIMEOUT_MS:
            self._soft_failsafe()
            self.link_ok = False

        return self.link_ok

    def _apply_cmd_frame(self, buf):
        device_id = buf[1]
        if device_id >= MAX_ESC:
            return
        enabled = bool(buf[2] & CMD_ENABLE_BIT)
        thr = ((buf[2] & CMD_THR_HI_MASK) << 8) | buf[3]

        cmd = self.commands[device_id]
        cmd.enabled = enabled
        # disabled 通道实际throttle必须为0(需求2)
        cmd.throttle = thr if enabled else 0

    def _soft_failsafe(self):
        """100ms无有效通信: throttle清零，保留enabled状态"""
        for cmd in self.commands:
            cmd.throttle = 0

    def _hard_failsafe(self):
        """2000ms无有效通信: throttle清零，同时清除enabled"""
        for cmd in self.commands:
            cmd.throttle = 0
            cmd.enabled = False

    def send_next_status(self, statuses):
        i = self._send_idx
        buf = statuses[i].pack(i)
        self.uart.write(buf)
        self._send_idx = (i + 1) % MAX_ESC


# ================= 使用示例 =================
if __name__ == "__main__":
    link = UartLink()
    statuses = [EscStatus() for _ in range(MAX_ESC)]

    last_status_ms = time.ticks_ms()
    STATUS_PERIOD_MS = 2

    while True:
        link.poll_commands()

        now = time.ticks_ms()
        if time.ticks_diff(now, last_status_ms) >= STATUS_PERIOD_MS:
            link.send_next_status(statuses)
            last_status_ms = now

        time.sleep_ms(1)