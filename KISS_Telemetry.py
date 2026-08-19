"""
KISS ESC 4路遥测读取 - RP2350 MicroPython
全部使用 PIO0 的4个状态机做软件UART RX
PIO1 和PIO2 的8个状态机预留给 DShot 输出，互不冲突
"""

from machine import Pin
import rp2
import utime

# ============================================================
# 配置
# ============================================================
PIO_RX_CONFIG = {
    "ESC1": {"sm_id": 0, "pin": 4},
    "ESC2": {"sm_id": 1, "pin": 5},
    "ESC3": {"sm_id": 2, "pin": 6},
    "ESC4": {"sm_id": 3, "pin": 7},
}
# sm_id 0~3 属于 PIO0，DShot那边用 sm_id 4~7（对应PIO1）

BAUDRATE = 115200
FRAME_LEN = 10
TELEM_TIMEOUT_MS = 1000


# ============================================================
# CRC8 —— 多项式 0x07（实测跑通的版本）
# ============================================================
def update_crc8(crc, seed):
    crc_u = crc ^ seed
    for _ in range(8):
        crc_u = (0x7 ^ (crc_u << 1)) if (crc_u & 0x80) else (crc_u << 1)
        crc_u &= 0xFF
    return crc_u

def get_crc8(buf, length):
    crc = 0
    for i in range(length):
        crc = update_crc8(crc, buf[i])
    return crc


# ============================================================
# 通用解析器
# ============================================================
class KissTelemParser:
    FRAME_LEN = FRAME_LEN

    def __init__(self, name="ESC"):
        self.name = name
        self.buf = bytearray(self.FRAME_LEN)
        self.idx = 0
        self.temperature = 0
        self.voltage = 0.0
        self.current = 0.0
        self.consumption = 0
        self.erpm = 0.0
        self.valid = False
        self.last_update_ms = 0
        self.crc_err_count = 0
        self.frame_count = 0

    def feed(self, data):
        for b in data:
            if self.idx < self.FRAME_LEN:
                self.buf[self.idx] = b
                self.idx += 1
            if self.idx == self.FRAME_LEN:
                self._try_parse()

    def _try_parse(self):
        if get_crc8(self.buf, self.FRAME_LEN - 1) == self.buf[self.FRAME_LEN - 1]:
            self.temperature = self.buf[0]
            self.voltage = ((self.buf[1] << 8) | self.buf[2]) / 100.0
            self.current = ((self.buf[3] << 8) | self.buf[4]) / 100.0
            self.consumption = (self.buf[5] << 8) | self.buf[6]
            self.erpm = ((self.buf[7] << 8) | self.buf[8]) / 100.0
            self.valid = True
            self.last_update_ms = utime.ticks_ms()
            self.frame_count += 1
            self.idx = 0
        else:
            self.crc_err_count += 1
            self.buf[:-1] = self.buf[1:]
            self.idx = self.FRAME_LEN - 1

    def is_alive(self, timeout_ms=TELEM_TIMEOUT_MS):
        return self.valid and utime.ticks_diff(utime.ticks_ms(), self.last_update_ms) < timeout_ms

    def __str__(self):
        if not self.is_alive():
            return "{:<5s}: offline (crc_err={})".format(self.name, self.crc_err_count)
        return "{:<5s}: T={:3d}C  V={:5.2f}V  I={:5.2f}A  mAh={:5d}  eRPM={:7.1f}  (err={})".format(
            self.name, self.temperature, self.voltage, self.current,
            self.consumption, self.erpm, self.crc_err_count
        )


# ============================================================
# PIO 软件 UART RX
# ============================================================
@rp2.asm_pio(autopush=True, push_thresh=8, in_shiftdir=rp2.PIO.SHIFT_RIGHT)
def uart_rx_mini():
    wait(0, pin, 0)
    set(x, 7) [10]
    label("bitloop")
    in_(pins, 1) [6]
    jmp(x_dec, "bitloop")


class PioUartRx:
    def __init__(self, sm_id, pin_num, baudrate, parser):
        self.parser = parser
        self.pin = Pin(pin_num, Pin.IN, Pin.PULL_UP)
        freq = 8 * baudrate
        self.sm = rp2.StateMachine(sm_id, uart_rx_mini, freq=freq, in_base=self.pin)
        self.sm.irq(self._on_rx)
        self.sm.active(1)

    def _on_rx(self, sm):
        while sm.rx_fifo():
            self.parser.feed(bytes([sm.get() & 0xFF]))


# ============================================================
# 初始化
# ============================================================
escs = {name: KissTelemParser(name) for name in PIO_RX_CONFIG}

pio_rx = {}
for name, cfg in PIO_RX_CONFIG.items():
    pio_rx[name] = PioUartRx(
        sm_id=cfg["sm_id"], pin_num=cfg["pin"],
        baudrate=BAUDRATE, parser=escs[name]
    )


# ============================================================
# 主循环
# ============================================================
def main_loop():
    last_print = utime.ticks_ms()
    while True:
        if utime.ticks_diff(utime.ticks_ms(), last_print) > 200:
            for name, p in escs.items():
                print(p)
            print("-" * 60)
            last_print = utime.ticks_ms()
        utime.sleep_ms(5)


if __name__ == "__main__":
    main_loop()
