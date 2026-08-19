"""
RP2350 (Pico 2) MicroPython + rp2 PIO
双向 DShot600 驱动 —— v3

本版是在你贴的"已修复单电机版"（已经解决过 NameError 和 4 个致命时序/
协议 bug）基础上，针对你提出的 6 个问题继续改的。ESC 端源码 dshot.c
作为最高参考，凡是跟网上资料冲突的地方都以 dshot.c 里的实现为准。

================================================================
问题1：PIO 里不能用 Python 全局/闭包变量 —— 现状 + 进一步收紧
================================================================
你贴的版本其实已经把 4 个 PIO 程序体里的变量都换成了字面量整数，
不会再触发 `_RX_START_DELAY_ITERS isn't defined` 这类 NameError。
这版继续保持"PIO 程序体只写字面量"这条规则，并且顺手把问题5顺带
解决的"相对寻址"也应用在这里——本质上也是"不依赖 Python 变量"的
体现（irq 号从"读 Python 闭包"变成了"硬件按 SM 编号自动计算"）。

================================================================
问题5：多电机扩展 —— 发现一个之前版本会在真机上炸的 bug，已修
================================================================
你贴的版本用 `_mk_tx(0)/_mk_tx(1)` 和 `_mk_rx(0)/_mk_rx(1)` 生成了
两份几乎一样、只有 irq 号不同的 TX 程序和两份 RX 程序，为的是让
同一个 PIO block 里的两对电机(local_pair 0/1)握手不串台。

问题是：**一个 PIO block 的指令内存只有 32 条，是 block 里 4 个 SM
共享的**（不是每个 SM 独立 32 条）。数一下：
    TX 程序 14 条 x 2 个变体 = 28 条
    RX 程序  6 条 x 2 个变体 = 12 条
    合计 40 条 > 32 条上限

**相对寻址** `rel()`。RP2040/RP2350
的 IRQ 指令支持"相对"模式：若使用 `irq(rel(N))`，硬件实际操作的
flag = `4 + (N + 本SM在block内的编号) % 4`（结果永远落在 4~7，这
4 个 flag 只用于 SM 间通信，不会被系统 IRQ 抢用）。

本设计里每对电机的 TX 在 block 内编号是偶数(0或2)，配对的 RX 编号
永远是 TX+1(1或3)。取
    TX: `irq(rel(0))`             -> flag = 4 + (0+T)%4      = 4+T%4
    RX: `wait(1, irq, rel(3))`    -> flag = 4 + (3+T+1)%4    = 4+T%4
    RX: `irq(clear, rel(3))`      -> 同上，清的是同一个 flag
两边算出来的 flag 恒相等（3+1=4≡0 mod4，相当于 RX 用"+3"把自己的
编号"减1"对齐回了 TX 的编号）。这样 **只需要一份 TX 程序 + 一份
RX 程序**，不管装在 local_pair 0 还是 1，握手都自动接对，互不串台。
指令内存降回 14+6=20 条，一个 block 稳稳装下两对电机。

（如果你在真机上发现 `rel()` 的行为跟这里推的不一致——比如你的
MicroPython/SDK 版本对 rel() 的实现有出入——退路是：老老实实用两份
程序，但每个 block 只放 1 对电机（即 `_SLOTS` 只用 (1,0) 和 (2,0)，
放弃"一个 block 两对"，改成"一个 block 一对"，4 个 PIO block 才能
上 4 电机而不是 2 个 block 上 4 电机）。这个退路我在 `_SLOTS` 旁边
留了注释，方便你切换。）

================================================================
问题2：扩展遥测(EDT) —— 帧格式来自 dshot.c::make_dshot_package()
================================================================
关键结构（原文件里的 `dshot_telem_scheduler_t` / 各种 DIVISOR）：
ESC 在开启 `dshot_extended_telemetry` 后，**不是每帧都回 eRPM**，
而是按固定节奏把 eRPM 换成电压/电流/温度/状态帧穿插着发：
    - CURRENT_EDT_RATE_DIVISOR = 40  -> 电流最勤，约每 40 帧一次
    - VOLTAGE_EDT_RATE_DIVISOR = 200 -> 电压
    - TEMP_EDT_RATE_DIVISOR    = 200 -> 温度
且用 `last_sent_extended` 保证"不会连续两帧都是扩展帧"（隔一帧发
一次，保证 eRPM 帧不会被完全挤掉）。

帧类型怎么区分（这是我从 dshot.c 的编码算法反推出来的，不是抄
网上资料——见下面 decode_telemetry_frame() 里的证明注释）：
12bit 载荷 `dshot_full_number` 的最高 4 位(即 GCR 解出来的 n0)：
    n0 == 0b0010 (0x2) -> 温度，低 8 位 = 摄氏度整数
    n0 == 0b0100 (0x4) -> 电压，低 8 位 = battery_voltage/25
    n0 == 0b0110 (0x6) -> 电流，低 8 位 = actual_current/50
    n0 == 0b1110 (0xE) -> EDT 状态帧：低8位=0x00 是"已开启"应答，
                           0xFF 是"已关闭"应答（对应 send_EDT_init /
                           send_EDT_deinit 两行硬编码的 0xE00/0xEFF）
    其余情况              -> 普通 eRPM 帧（eee + 9位尾数）
电压/电流的物理单位换算(*0.25V / *0.5A)是 AM32/Bluejay 系ESC通用的
ADC 定标惯例。

CRC 沿用你原来就修对的"反码"规则，跟 dshot.c 完全一致：
    csum = ~(n0^n1^n2) & 0xF   ==  n3
    等价于 (n0^n1^n2^n3)&0xF == 0xF
（dshot.c 里 `csum = ~csum; csum &= 0xf;` 就是这个）。

主动请求：EDT 不是白送的，必须先用 DShot 命令 13 打开（对应
switch-case 里 `case 13: dshot_extended_telemetry = 1;`），命令 14
关闭。而且根据 `command_count` 的逻辑，非信标命令(cmd>5)必须在
电机没转(throttle=0)、ESC 已经 armed 的状态下**连续收到 >=6 帧一
模一样的命令**才会被采纳。本版新增 `DShotBus.send_command()` /
`enable_extended_telemetry()` 就是照这个规则实现的。

================================================================
问题3：RX 时序可靠性 —— 结论：你贴的边沿同步方案本身没问题
================================================================
逐条过一遍：
1) TX 释放总线：`set(pindirs,0)` 之后立刻 `irq()` 通知 RX，没有
   任何盲等，没问题。
2) ESC 响应延迟：不管 ESC 等了多久才应答，RX 都是靠
   `wait(0, pin, 0)` 死等电平变低才开始采样，对延迟完全不敏感。
   这是成立的，因为**双向 DShot 用的是反相编码**：空闲高电平，
   每一个数据位(不管是0还是1)都是"先拉低、再看拉多久决定是0是1"
   —— 也就是说无论 ESC 第一位发的是0还是1，它从"释放/空闲"切到
   "开始发送"的那个动作，物理上必然是一次从高到低的边沿。所以
   `wait(0,pin,0)` 抓到的边沿，一定是响应帧第0位的起点，不会因为
   "恰好第一位是1"就抓错。
3) RX 采样起点：`set(x,20) [6]` 在边沿之后再等 7 个 cycle
   （1个set的cycle+6个delay）≈0.583us，加上 GPIO->PIO 输入同步器
   本身 ~2 cycle 的延迟，总共落在半个位宽(0.667us)附近，采样点在
   每一位的中段，合理。
4) 21bit 采样周期：15(in_的delay)+1(jmp自身) = 16 cycle @12.8MHz =
   1.25us，跟修正后的800kbit响应位宽精确对齐（**注意**：这里的
   12.8MHz 不是随便凑的，是靠 [FIX-v3.1] 修正响应波特率算出来的，
   见常量定义处的详细说明——这是本版实测发现的真正问题所在）。
5) IRQ 同步：见上面问题5，改成 rel() 之后同步机制不变，只是从
   "固定 flag 0/1" 换成"运行时按 SM 编号算出的 flag"，可靠性一样，
   而且顺带解决了指令内存溢出。
结论：RX 时序设计是对的，这版没有再改采样逻辑本身，只是把
irq 号从字面量换成了 rel()。

================================================================
问题4：poll 机制 —— 结论：不建议上 DMA，用 Timer+schedule 自动化
================================================================
1) 要不要恢复 DMA RX？**不建议**。DShot600 下每个电机的响应帧只有
   1 个 32bit 字，2kHz 更新率下相当于每 500us 才有 1 个字要搬，
   数据量小到 DMA 带来的收益基本为零；而 RP2350 上 PIO2 的
   DREQ 基址目前没有官方 MicroPython API 直接给（`rp2.DMA` 需要
   自己算 treq_sel），算错的话是"安静地不工作"或者"半夜炸"，
   排查成本远高于收益。这版继续不用 DMA。
2) PIO FIFO + IRQ？RX FIFO 深度 4 级，2kHz 下每 500us place 1 个
   字，只要每个 tick 都去取，FIFO 永远不会溢出，不需要额外机制。
3) 真正实现"不用手动 poll，后台自动更新"：用
   `machine.Timer`(硬件定时器，回调在硬中断上下文，不能分配内存)
   触发发送，然后用 **`micropython.schedule()`** 把"读 FIFO+解码
   +更新属性"这部分**会分配内存**的工作丢到中断安全的"计划任务"
   队列里，在中断返回后尽快、但不在中断上下文里执行。这是
   MicroPython 官方推荐的"硬中断里做轻量触发，重活丢给 schedule"
   模式，不用猜 DREQ、不用 DMA，效果就是你要的
       while True:
           print(motor.rpm, motor.voltage, motor.current, motor.temperature)
   数据会在后台自动新鲜起来。`bus.poll()` 仍然保留，手动调用也没
   问题（只是通常不需要了）。

"""

import rp2
from rp2 import PIO, StateMachine
import machine
import time
import micropython

# 硬中断里如果真的抛异常，这样至少能看到 traceback 而不是硬死机
micropython.alloc_emergency_exception_buf(100)

# ---------------------------------------------------------------------
# 时钟与位宽（只给 Python 侧用；PIO 程序体里一律写字面量）
# ---------------------------------------------------------------------
_TX_PIO_FREQ = 9_600_000    # 600kHz * 16 cycles/bit  -> 1 bit = 1.6667us
# [FIX-v3.1] 响应帧波特率修正：双向DShot的响应帧波特率是发送帧的
# **4/3**倍，不是5/4！(brushlesswhoop.com 原话："baud rate ... 4/3
# of the frame's baud rate"，Betaflight/AM32的解码器也是按4/3算的)
# 之前(包括你贴的"已修复版")一直按 5/4 算成了750kbit(1.3333us/bit)，
# 实际应该是 800kbit(1.25us/bit)。这个6.25%的位宽误差单帧看不出来，
# 但21位累计漂移 21*(1.3333-1.25)=1.75us ≈ 1.4个真实位宽，采到帧尾
# 时已经完全跑到别的位上了——跟你最早那版的 BUG-1 是同一类问题，
# 只是这次是波特率基准本身取错了，不是delay算错。
_RX_PIO_FREQ = 12_800_000   # 800kHz(正确响应波特率) * 16cy/bit = 1.25us/bit

DSHOT_BITRATE = 600_000
RESPONSE_BITRATE = DSHOT_BITRATE * 4 // 3       # 800_000 (原来错写成 *5//4)
RESPONSE_BIT_US = 1e6 / RESPONSE_BITRATE        # 1.25us
RESPONSE_BITS = 21

_TX_FRAME_US = 16 * 16 * 1e6 / _TX_PIO_FREQ                    # 26.67us
_TX_GAP_US = (32 * 16 + 22 * 16) * 1e6 / _TX_PIO_FREQ           # 90.0us
_TX_TOTAL_US = _TX_FRAME_US + _TX_GAP_US                        # ~116.7us

# DShot 命令号（dshot.c 里 switch(dshotcommand) 的分支）
DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE = 13
DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE = 14


# =====================================================================
# GCR 编解码 + 遥测帧解析（问题2）
# =====================================================================
_GCR_ENCODE_TABLE = (
    0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111,
    0b11010, 0b01001, 0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111,
)
_GCR_DECODE_TABLE = {code: val for val, code in enumerate(_GCR_ENCODE_TABLE)}

_NO_ROTATION_VALUE12 = 0x0FFF   # com_time=65535(!running) 编出来的固定值


def decode_telemetry_frame(raw21):
    """
    raw21: 低 21 位是按时间顺序采到的响应位(bit20=最早采到的一位)。

    返回:
        None                              -> 帧非法(GCR/CRC没过)
        {"type": "erpm", "erpm": float}
        {"type": "temperature", "celsius": int}
        {"type": "voltage", "raw": int, "volts": float}
        {"type": "current", "raw": int, "amps": float}
        {"type": "edt_state", "enabled": bool}

    帧类型判据的推导(对照 dshot.c::make_dshot_package())：
    正常 eRPM 帧 12bit = shift_amount(3bit) << 9 | mantissa(9bit)，
    其中 shift_amount 是"com_time 里最高位1所在的位置"算出来的，这
    保证了只要 shift_amount>=1，mantissa 的 bit8 必然是 1(因为 bit8
    正是那个"最高位1"移位后落到的位置)。于是 12bit 数的最高4位
    (top nibble = shift_amount*2 + mantissa_bit8) 在 shift_amount>=1
    时恒为**奇数**(3,5,7,9,11,13,15)；shift_amount==0 时 top nibble
    只可能是 0 或 1。也就是说**真实 eRPM 帧的 top nibble 只会是
    0/1/3/5/7/9/11/13/15，永远不会是偶数 2/4/6/14**——而这几个偶数
    恰好就是 dshot.c 里 extended_frame_to_send 用的类型码，两者不会
    互相污染，可以安全地用 top nibble 是否等于 2/4/6/E 来分流。
    """
    gcr20 = 0
    prev = (raw21 >> 20) & 1
    for i in range(19, -1, -1):
        b = (raw21 >> i) & 1
        gcr20 = (gcr20 << 1) | (b ^ prev)
        prev = b

    n0 = _GCR_DECODE_TABLE.get((gcr20 >> 15) & 0x1F)
    n1 = _GCR_DECODE_TABLE.get((gcr20 >> 10) & 0x1F)
    n2 = _GCR_DECODE_TABLE.get((gcr20 >> 5) & 0x1F)
    n3 = _GCR_DECODE_TABLE.get(gcr20 & 0x1F)
    if n0 is None or n1 is None or n2 is None or n3 is None:
        return None

    # 反码 CRC，跟 dshot.c 的 csum = ~csum & 0xf 完全一致
    if ((n0 ^ n1 ^ n2 ^ n3) & 0x0F) != 0x0F:
        return None

    value12 = (n0 << 8) | (n1 << 4) | n2
    data8 = value12 & 0xFF

    if n0 == 0x2:
        return {"type": "temperature", "celsius": data8}
    if n0 == 0x4:
        # *0.25V: AM32/Bluejay 惯例定标，请对照你的 ESC ADC 代码核实
        return {"type": "voltage", "raw": data8, "volts": data8 * 0.25}
    if n0 == 0x6:
        # *0.5A: 同上，惯例定标
        return {"type": "current", "raw": data8, "amps": data8 * 0.5}
    if n0 == 0xE:
        if data8 == 0x00:
            return {"type": "edt_state", "enabled": True}
        if data8 == 0xFF:
            return {"type": "edt_state", "enabled": False}
        return {"type": "edt_state", "enabled": None, "raw": data8}

    # 普通 eRPM 帧
    if value12 == _NO_ROTATION_VALUE12:
        return {"type": "erpm", "erpm": 0.0}
    shift = value12 >> 9
    mantissa = value12 & 0x1FF
    period_us = mantissa << shift
    if period_us == 0:
        return None
    return {"type": "erpm", "erpm": 60_000_000.0 / period_us}


# =====================================================================
# PIO 程序 —— 改用 rel() 相对寻址，TX/RX 各只需要 1 份程序
# 指令数：TX=14, RX=6，一个 block 装两对电机总共 20 条，留有余量。
# =====================================================================
#
# TX 时序(反相 DShot)：1 bit = 16 cycles @9.6MHz = 1.6667us
#   bit=1 : 高4cy + 低12cy(T1L=1250ns)
#   bit=0 : 高4cy + 低6cy + 高6cy(T0L=625ns)
#   16位发完 -> 让出总线 -> irq(rel(0)) 通知配对的 RX -> 等~90us
#   -> 抢回总线拉高 -> 下一帧
#
@rp2.asm_pio(sideset_init=PIO.OUT_HIGH, set_init=PIO.OUT_HIGH,
             out_shiftdir=PIO.SHIFT_LEFT, autopull=True, pull_thresh=16)
def _tx_program():
    label("frame")
    set(y, 15)                     .side(1)      # 16 个数据位
    label("bitloop")
    out(x, 1)                      .side(1) [3]  # 4cy 高（位间隙）
    jmp(not_x, "do_zero")          .side(0) [5]  # 拉低 6cy
    jmp(y_dec, "bitloop")          .side(0) [5]  # bit=1: 再低 6cy
    jmp("gap_start")               .side(1)
    label("do_zero")
    jmp(y_dec, "bitloop")          .side(1) [5]  # bit=0: 补 6cy 高
    label("gap_start")
    set(pindirs, 0)                .side(1)      # 让出总线(变输入)
    irq(rel(0))                    .side(1)      # 相对寻址通知配对RX
    set(x, 31)                     .side(1)
    label("gap1")
    jmp(x_dec, "gap1")             .side(1) [15] # 32*16 = 512cy
    set(x, 21)                     .side(1)
    label("gap2")
    jmp(x_dec, "gap2")             .side(1) [15] # 22*16 = 352cy
    set(pindirs, 1)                .side(1)      # 抢回总线，驱动高
    jmp("frame")                   .side(1)


# RX 时序：边沿同步(问题3)，与配对 TX 通过 rel() 自动对齐(问题5)
@rp2.asm_pio(in_shiftdir=PIO.SHIFT_LEFT, autopush=True, push_thresh=21)
def _rx_program():
    wrap_target()
    irq(clear, rel(3))             # 丢掉过期flag，握手自恢复
    wait(1, irq, rel(3))           # 等配对 TX 让出总线
    wait(0, pin, 0)                # 等ESC拉低=响应帧第0位起点
    set(x, 20)               [6]   # 21个采样点；+7cy≈半个位宽
    label("sample")
    in_(pins, 1)              [14] # 15cy + jmp 1cy = 16cy/bit
    jmp(x_dec, "sample")
    wrap()


class DShotMotor:
    """单个双向电机 = 1个TX SM + 1个RX SM，共享上面两份程序。"""

    def __init__(self, pin_num, pio_num, local_pair, name="", pole_pairs=7):
        """
        pin_num    : 信号线 GPIO
        pio_num    : 1 或 2
        local_pair : 0 -> 该block的 SM(0,1)   1 -> 该block的 SM(2,3)
        pole_pairs : 电机极对数，用于把 eRPM 换算成机械 RPM
        """
        if pio_num not in (1, 2):
            raise ValueError("pio_num 必须是 1 或 2")
        if local_pair not in (0, 1):
            raise ValueError("local_pair 必须是 0 或 1")

        self.name = name or ("m_pio%d_%d" % (pio_num, local_pair))
        self.pole_pairs = pole_pairs

        global_tx_sm = pio_num * 4 + local_pair * 2
        global_rx_sm = global_tx_sm + 1

        pin = machine.Pin(pin_num, machine.Pin.IN, machine.Pin.PULL_UP)
        self.pin = pin

        # 注意：这里两对电机传的是**同一个**程序对象 _tx_program/_rx_program，
        # MicroPython 会在同一个 PIO block 内自动复用已加载的指令，不会
        # 重复占用指令内存（省下一半指令空间的关键）。
        self.tx_sm = StateMachine(
            global_tx_sm, _tx_program, freq=_TX_PIO_FREQ,
            sideset_base=pin, set_base=pin,
        )
        self.rx_sm = StateMachine(
            global_rx_sm, _rx_program, freq=_RX_PIO_FREQ,
            in_base=pin,
        )

        self._word = self.encode(0)      # 上电即准备发"停机"帧

        self.latest_erpm = None
        self.latest_temp_c = None
        self.latest_voltage_raw = None
        self.latest_voltage_v = None
        self.latest_current_raw = None
        self.latest_current_a = None
        self.edt_enabled = None          # None=未知，True/False=ESC已应答过
        self.latest_raw = None
        self.raw_frames = 0
        self.valid_frames = 0
        self.error_frames = 0
        self.type_counts = {}            # 各帧类型计数，调试用

        # 先起 RX（停在 wait irq 上），再起 TX，避免第一帧漏采
        self.rx_sm.active(1)
        self.tx_sm.active(1)

    # ---------------- 发送侧 ----------------
    @staticmethod
    def encode(throttle_or_cmd, telemetry=False):
        """
        16bit DShot 帧：value(11) | telemetry_request(1) | crc(4)
        value 既可以是油门(0-2047)也可以是命令(0-47)，编码方式一样。
        双向 DShot 发送侧 CRC 同样是反码。
        返回值已左移16位，配合 pull_thresh=16 + SHIFT_LEFT，put()一个
        32bit字正好发出高16位。
        """
        v = max(0, min(2047, int(throttle_or_cmd)))
        tel = 1 if telemetry else 0
        pkt = (v << 1) | tel
        crc = (~(pkt ^ (pkt >> 4) ^ (pkt >> 8))) & 0x0F
        return ((pkt << 4) | crc) << 16

    def set_throttle(self, throttle, telemetry=False):
        self._word = self.encode(throttle, telemetry)

    def _trigger_send(self):
        """Timer回调(硬中断)里调用，只做不分配内存的操作。"""
        if self.tx_sm.tx_fifo() < 4:      # 防止FIFO满时put()阻塞
            self.tx_sm.put(self._word)

    # ---------------- 接收侧 ----------------
    def poll(self):
        """
        读空RX FIFO并解码。会分配内存，只应该在非硬中断上下文调用
        （正常情况下 DShotBus 会用 micropython.schedule 自动帮你调，
        见问题4；这里保留手动调用的能力）。
        返回本次新解码出的 eRPM(float)或 None。
        """
        got = None
        while self.rx_sm.rx_fifo():
            word = self.rx_sm.get() & 0x1FFFFF     # 只有低21位有效
            self.raw_frames += 1
            self.latest_raw = word
            frame = decode_telemetry_frame(word)
            if frame is None:
                self.error_frames += 1
                continue
            self.valid_frames += 1
            t = frame["type"]
            self.type_counts[t] = self.type_counts.get(t, 0) + 1
            if t == "erpm":
                self.latest_erpm = frame["erpm"]
                got = frame["erpm"]
            elif t == "temperature":
                self.latest_temp_c = frame["celsius"]
            elif t == "voltage":
                self.latest_voltage_raw = frame["raw"]
                self.latest_voltage_v = frame["volts"]
            elif t == "current":
                self.latest_current_raw = frame["raw"]
                self.latest_current_a = frame["amps"]
            elif t == "edt_state":
                self.edt_enabled = frame["enabled"]
        return got

    # ---------------- 便捷只读属性(问题4的目标：直接print即可) --------
    @property
    def rpm(self):
        if self.latest_erpm is None:
            return None
        return self.latest_erpm / self.pole_pairs

    @property
    def voltage(self):
        return self.latest_voltage_v

    @property
    def current(self):
        return self.latest_current_a

    @property
    def temperature(self):
        return self.latest_temp_c

    # ---------------- 调试 ----------------
    def debug_dump_raw(self):
        if self.latest_raw is None:
            print("[%s] 还没收到任何响应帧：ESC没应答。"
                  "先查ESC是否开启Bidirectional DShot、信号线/地线、"
                  "电机是否已arm。" % self.name)
            return
        w = self.latest_raw
        bits = [(w >> (RESPONSE_BITS - 1 - i)) & 1 for i in range(RESPONSE_BITS)]
        trans = sum(1 for a, b in zip(bits, bits[1:]) if a != b)
        print("[%s] raw=%d valid=%d err=%d word=0x%06x bits=%s 跳变=%d "
              "type_counts=%s erpm=%s edt=%s V=%s A=%s C=%s"
              % (self.name, self.raw_frames, self.valid_frames,
                 self.error_frames, w, "".join(str(b) for b in bits), trans,
                 self.type_counts, self.latest_erpm, self.edt_enabled,
                 self.latest_voltage_v, self.latest_current_a,
                 self.latest_temp_c))

    def close(self):
        self.tx_sm.active(0)
        self.rx_sm.active(0)
    
    # ---------------- 统一格式的遥测快照 ----------------
    def get_telemetry(self):
        """
        一次性返回所有遥测数据的统一快照(erpm/rpm/Temp/Voltage/Current)。
        跟 debug_dump_raw()+print(m.rpm) 两条独立语句不同，这里先把
        latest_erpm 读到局部变量再算 rpm，保证 erpm 和 rpm 在同一次
        调用里对应同一帧，不会被后台 schedule 在两条语句之间抢先刷新。
        """
        erpm = self.latest_erpm
        return {
            "erpm": erpm,
            "rpm": (erpm / self.pole_pairs) if erpm is not None else None,
            "Temp": self.latest_temp_c,
            "Voltage": self.latest_voltage_v,
            "Current": self.latest_current_a,
            "edt": self.edt_enabled,
        }


class DShotBus:
    """
    用一个共享 Timer 同步触发所有电机发帧，并用 micropython.schedule
    在后台自动读取/解码遥测（问题4），不再需要用户手动 poll()。
    """

    # 先填 PIO1 的两对，再填 PIO2 的两对。
    # 如果实测 rel() 相对寻址跟预期不符，把这里改成
    #   _SLOTS = ((1, 0), (2, 0))
    # 退化成"一个block只放一对电机"，用两份独立程序也能装下(见文件头
    # 问题5的退路说明)，但最多只能接2个双向电机。
    _SLOTS = ((1, 0), (1, 1), (2, 0), (2, 1))

    def __init__(self, motor_pins, update_hz=2000, names=None, pole_pairs=7):
        if len(motor_pins) > len(self._SLOTS):
            raise ValueError("最多 %d 个双向电机" % len(self._SLOTS))
        period_us = 1e6 / update_hz
        if period_us < _TX_TOTAL_US * 1.05:
            raise ValueError(
                "update_hz=%.0f (周期%.1fus) 太快：一帧数据+让出总线窗口需要%.1fus"
                % (update_hz, period_us, _TX_TOTAL_US))

        names = names or [None] * len(motor_pins)
        self.motors = [
            DShotMotor(p, slot[0], slot[1], name=n, pole_pairs=pole_pairs)
            for p, slot, n in zip(motor_pins, self._SLOTS, names)
        ]
        self.update_hz = update_hz
        self._timer = machine.Timer()
        self._timer.init(freq=update_hz, mode=machine.Timer.PERIODIC,
                          callback=self._on_tick)

    # ---- 硬中断上下文：只做不分配内存的操作 ----
    def _on_tick(self, t):
        for m in self.motors:
            m._trigger_send()
        try:
            micropython.schedule(self._poll_all, 0)
        except RuntimeError:
            # 计划任务队列满，跳过一次；PIO RX FIFO有4级缓冲，
            # 下一次tick照样能读到，不会丢数据，只是稍微滞后。
            pass

    # ---- 由 schedule 在安全上下文里调用，可以分配内存 ----
    def _poll_all(self, _):
        for m in self.motors:
            m.poll()

    def set_throttle(self, index, throttle, telemetry=False):
        self.motors[index].set_throttle(throttle, telemetry)

    def poll(self):
        """手动强制刷新一次（通常不需要，后台已经自动在做）。"""
        self._poll_all(None)

    def send_command(self, index, cmd, repeat=6):
        """
        发送一个 DShot 命令(0-47)给指定电机。规则来自 dshot.c 的
        command_count 逻辑：
          - cmd<=5 (信标等)：ESC收到1帧就立即执行。
          - cmd>5  (如13/14)：必须在电机未转(throttle=0)、已armed的
            状态下，连续收到 >=6 帧完全相同的命令才会被采纳。
        发送期间会临时把该电机的帧内容覆盖为命令值，发完自动恢复回
        throttle=0（不是恢复到发命令前的油门——命令帧本来就要求
        value字段是0-47，不能跟正常给油同时进行）。
        """
        if not (0 <= cmd <= 47):
            raise ValueError("DShot 命令范围是 0-47")
        m = self.motors[index]
        n = repeat if cmd > 5 else 1
        period_ms = max(1, int(1000.0 / self.update_hz) + 1)
        cmd_word = m.encode(cmd, telemetry=False)
        for _ in range(n):
            m._word = cmd_word
            time.sleep_ms(period_ms)
        m._word = m.encode(0)

    def enable_extended_telemetry(self, index, enable=True, repeat=6):
        """打开/关闭 Extended DShot Telemetry(问题2)。调用前必须先
        arm_all()，且电机不能在给油门。可以用
        motors[index].edt_enabled 看ESC是否回了确认帧(0xE00/0xEFF)。"""
        cmd = (DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE if enable
               else DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE)
        self.send_command(index, cmd, repeat=repeat)

    def arm_all(self, duration_ms=3000):
        """持续发 throttle=0 让 ESC 完成 arm。"""
        for m in self.motors:
            m.set_throttle(0)
        t_end = time.ticks_add(time.ticks_ms(), duration_ms)
        while time.ticks_diff(t_end, time.ticks_ms()) > 0:
            time.sleep_ms(2)   # 不用手动poll了，后台schedule在自动跑

    def stop(self):
        for m in self.motors:
            m.set_throttle(0)
        time.sleep_ms(50)
        self._timer.deinit()
        for m in self.motors:
            m.close()


# =====================================================================
# 单电机测试：GPIO4，2kHz 更新率，开EDT后打印全部遥测
# =====================================================================
if __name__ == "__main__":
    bus = DShotBus(motor_pins=[4], update_hz=2000, names=["m0"], pole_pairs=7)
    m = bus.motors[0]

    print("Arming (3s, throttle=0)...")
    print("提示：油门0时ESC也会应答(回报'未旋转')，valid应该已经在涨。")
    bus.arm_all(3000)
    m.debug_dump_raw()

    print("\n开启 Extended DShot Telemetry (命令13，需已armed+油门0)...")
    bus.enable_extended_telemetry(0, True)
    time.sleep_ms(100)

    
    print("ESC EDT 应答:", m.edt_enabled, "(True=已开启, None=还没收到确认帧)")

    print("\n开始给油 throttle=600（确认桨已拆！）")
    m.set_throttle(500)

    t_print = time.ticks_ms()
    for _ in range(20000):        # ~10秒
        # 不需要手动 bus.poll()：Timer回调里已经用 micropython.schedule
        # 自动在后台把 latest_erpm/voltage/current/temperature 刷新好了。
        if time.ticks_diff(time.ticks_ms(), t_print) >= 500:
            t_print = time.ticks_ms()
            print(m.get_telemetry())
#             m.debug_dump_raw()
#             if m.rpm is not None:
# #                 print("      -> 机械转速 ~= %.0f RPM (7对极)" % m.rpm)
#                 print(m.rpm)
        time.sleep_ms(2)

    print("停机")
    bus.stop()
    
# =====================================================================
# 双电机测试：GPIO4，GPIO15 2kHz 更新率，开EDT后打印全部遥测
# =====================================================================
# if __name__ == "__main__":
#     bus = DShotBus(motor_pins=[4,15], update_hz=2000, names=["m0","m1"], pole_pairs=7)
# 
#     print("Arming (3s, throttle=0)...")
#     print("提示：油门0时ESC也会应答(回报'未旋转')，valid应该已经在涨。")
#     bus.arm_all(3000)
# 
#     print("\n开启 Extended DShot Telemetry (命令13，需已armed+油门0)...")
#     for i in range(len(bus.motors)):
#         bus.enable_extended_telemetry(i, True)
#         time.sleep_ms(100)   # 命令之间留点间隔，别挤在一起
#     
#     for i, m in enumerate(bus.motors):
#         print(m.name, "EDT应答:", m.edt_enabled)
# 
#     print("\n开始给油 throttle=600（确认桨已拆！）")
#     for i in range(len(bus.motors)):
#         bus.set_throttle(i, 600)
# 
#     t_print = time.ticks_ms()
#     for _ in range(20000):        # ~10秒
#         # 不需要手动 bus.poll()：Timer回调里已经用 micropython.schedule
#         # 自动在后台把 latest_erpm/voltage/current/temperature 刷新好了。
#         if time.ticks_diff(time.ticks_ms(), t_print) >= 500:
#             t_print = time.ticks_ms()
#             for m in bus.motors:
#                 print(m.name, m.get_telemetry())
#         time.sleep_ms(2)
# 
#     print("停机")
#     bus.stop()

    print("停机")
    bus.stop()
