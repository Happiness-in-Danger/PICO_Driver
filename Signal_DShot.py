"""
RP2350 (MicroPython, rp2 PIO) DShot600 驱动
    DShot600      - 单向，只发送油门帧
16 个 PIO 时钟周期 = 1 bit，PIO 时钟设为 9.6MHz (=600kHz*16，正好整除)：
    T1 (公共高电平)                     = 6  cycles
    T2 (bit=1 额外高电平 / bit=0 额外低电平) = 6  cycles
    T3 (公共低电平，在每个 bit 最前面)         = 4  cycles
    合计 = 16 cycles = 1.6667us = 1/600000s   (DShot600 位周期，精确)
    bit=1 高电平 = T1+T2 = 12 cycles = 1.25us  (75%,   规范 T1H=1250ns)
    bit=0 高电平 = T1    = 6  cycles = 0.625us (37.5%, 规范 T0H=625ns)
两条分支总周期数都是 T1+T2+T3=16，完全相等，不会有频率漂移。
====================================================================
"""

import rp2
from rp2 import PIO, StateMachine
import machine
import uctypes
from array import array
import time

_PIO_FREQ = 9_600_000  # 600kHz * 16 cycles/bit


# =====================================================================
# 1) 单向 DShot600
# =====================================================================

@rp2.asm_pio(
    sideset_init=PIO.OUT_LOW,
    out_shiftdir=PIO.SHIFT_LEFT,
    autopull=True,
    pull_thresh=16,
)
def _dshot600_tx():
    label("bitloop")
    out(x, 1)               .side(0) [3]   # T3: 4 cycles 低电平, 同时把下一位移入 x
    jmp(not_x, "do_zero")   .side(1) [5]   # T1: 6 cycles 高电平 (两条分支公共部分)
    jmp(y_dec, "bitloop")   .side(1) [5]   # bit=1: 再 6 cycles 高电平, 顺便递减帧内位计数 y
    jmp("gap_start")        .side(0)       # 只有"最后一位且为1"时才会走到这里
    label("do_zero")
    jmp(y_dec, "bitloop")   .side(0) [5]   # bit=0: 再 6 cycles 低电平, 顺便递减帧内位计数 y
    label("gap_start")
    # 显式帧间隙: 21 * 16 = 336 cycles ≈ 35us 的空闲低电平，
    # 让帧与帧之间有清晰、恒定的分界，方便示波器识别，也符合DShot帧间应留间隙的习惯做法。
    # 想改帧间隔就调这个"20"，数字越大帧间隙越长、整体更新率越低。
    set(x, 30)               .side(0)
    label("gap")
    jmp(x_dec, "gap")        .side(0) [15]
    set(y, 15)                .side(0)     # 为下一帧重置 16 位计数器
    jmp("bitloop")             .side(0)


class DShot600:
    """单向 DShot600，DMA 循环重复发送 self._buf 中的 16bit 帧 (set_throttle 更新它)"""

    def __init__(self, pin_num: int, sm_id: int = 0, pio_num: int = 0):
        self.sm = StateMachine(
            sm_id, _dshot600_tx, freq=_PIO_FREQ,
            sideset_base=machine.Pin(pin_num, machine.Pin.OUT),
        )
        # 关键修复: SM 复位后 y=0，不预置的话第一帧发完 1 个 bit 就会结束
        self.sm.exec("set(y, 15)")
        self.sm.active(1)

        dreq = (pio_num << 3) + sm_id
        self.dma_a = rp2.DMA()
        self.dma_b = rp2.DMA()
        self._buf = array('L', [0])

        ctrl_a = self.dma_a.pack_ctrl(
            size=2, inc_read=False, inc_write=False,
            treq_sel=dreq, chain_to=self.dma_b.channel,
        )
        self._reset_block = array('L', [ctrl_a])
        ctrl_b = self.dma_b.pack_ctrl(
            size=2, inc_read=False, inc_write=False, treq_sel=0x3F,
        )
        dma_a_ctrl_trig_addr = uctypes.addressof(self.dma_a.registers) + 0x0C
        self.dma_b.config(
            read=self._reset_block, write=dma_a_ctrl_trig_addr,
            count=1, ctrl=ctrl_b, trigger=False,
        )
        self.dma_a.config(
            read=self._buf, write=self.sm, count=1, ctrl=ctrl_a, trigger=True,
        )

    @staticmethod
    def encode(throttle: int, telemetry: bool = False) -> int:
        throttle = max(0, min(2047, throttle))
        tel = 1 if telemetry else 0
        pkt = (throttle << 1) | tel
        crc = (pkt ^ (pkt >> 4) ^ (pkt >> 8)) & 0x0F
        return ((throttle << 5) | (tel << 4) | crc) << 16

    def set_throttle(self, throttle: int, telemetry: bool = False):
        self._buf[0] = self.encode(throttle, telemetry)

    def arm(self, duration_ms: int = 2000):
        self.set_throttle(0)
        time.sleep_ms(duration_ms)

    def stop(self):
        self.dma_a.close()
        self.dma_b.close()


if __name__ == "__main__":
    # --- 单向示例 ---
    esc = DShot600(pin_num=4, sm_id=0, pio_num=0)
    print("Arming (3s)...")
    esc.arm(3000)
    time.sleep(1)
    esc.set_throttle(200)
    time.sleep(2)
    esc.set_throttle(0)
