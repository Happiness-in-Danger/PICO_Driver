'''
注意此文件运行在主控部份！！！注意此文件运行在主控部份！！！注意此文件运行在主控部份！！！
STM32H743 主控侧 UART 驱动（正式版）
H743 通过此文件与 Pico 通讯，Pico 是负责电调控制和电调遥测的从机。
联调/验收测试见同目录的 Driver_test.py。

===================== 协议（与 Pico 侧 Uart_link.py 完全一致）=====================
固定支持 MAX_ESC=4 路 ESC。实际用几路由 enable 位动态决定。

命令帧 (H743 -> Pico)，CMD_FRAME_LEN=5:
    buf[0] = SYNC_CMD (0xA5)
    buf[1] = device_id (0~3)
    buf[2] = bit7: enable(1=启用 / 0=禁用)  bit6~3: 保留(0)  bit2~0: throttle bit10~8
    buf[3] = throttle bit7~0
    buf[4] = crc8(buf[0:4])

状态帧 (Pico -> H743)，STATUS_FRAME_LEN=13:
    buf[0] = SYNC_STATUS (0x5A)
    buf[1] = device_id
    buf[2] = flags: bit0=BDShot有效  bit1=KISS有效  bit2=该路已enable
    buf[3:7]  = erpm (uint32, 大端)
    buf[7]    = temperature (℃)
    buf[8:10] = voltage*100 (uint16, 大端)
    buf[10:12]= current*100 (uint16, 大端)
    buf[12]   = crc8(buf[0:12])

===================== 超时规则（Pico 侧实现）=====================
    100ms  Pico 未收到合法命令 -> 所有路 throttle 清零，enabled 保留
    2000ms Pico 未收到合法命令 -> 所有路 throttle 清零，同时 enabled 清除
  => H743 必须持续刷新命令。主循环每圈调用 link.update() 即可（每路约 8ms 刷新一次）。

===================== 重要：DShot 油门取值 =====================
    DShot 值 0      = 停转
    DShot 值 1~47   = 特殊命令(蜂鸣/换向等)，不是油门！
    DShot 值 48~2047 = 油门
  本驱动的 set_throttle() 会把 1~47 自动抬到 48，避免误触发命令。

===================== 多电机用法 =====================
    link = UartLink()
    link.enable_channels([0, 1, 2])              # 启用 3 路
    while True:
        link.update()                            # 每圈调用，≤2ms 一次
        link.set_throttles({0: 200, 1: 200, 2: 200})
        t = link.get_telemetry(0)                # 带"新鲜度"的遥测
        if t["fresh"]: print(t["rpm"], t["voltage"])
        time.sleep_ms(1)
    # 结束/异常时：
    link.emergency_stop()

遥测注意：
  * 状态帧每路约 60Hz(合计约 250 帧/s)。fresh=False 表示这路遥测已过期，不要使用。
  * source: "BDShot"(主) / "KISS"(备用) / "none"(两者都无效，数值无意义)。
'''
from pyb import UART
import time

# ================= 配置 =================
UART_ID = 6                 # 按你的硬件接线修改(本项目实测为 UART6)
UART_BAUD = 921600

# 硬件上限固定为4路，必须和 Pico 端 Uart_link.py 的 MAX_ESC 一致。
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

MIN_THROTTLE = 48           # DShot 油门最小值(1~47是命令)
MAX_THROTTLE = 2047

CMD_PERIOD_MS = 2           # 每 2ms 发一路命令 -> 每路 8ms 刷新一次(远小于Pico 100ms超时)
STATUS_FRESH_MS = 200       # 超过这个时间没收到某路状态帧，视为该路遥测过期
LINK_TIMEOUT_MS = 100       # H743 侧：超过此时间没收到任何状态帧 -> link_ok=False

# 注意：UART_ID 对应的引脚、与 Pico 的 TX/RX 交叉接线、共地，请在硬件上确认。


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


def build_cmd_frame(device_id, enabled, throttle):
    """按协议编码一帧命令(测试里也用它构造伪造/损坏帧)。"""
    thr = max(0, min(0x07FF, int(throttle)))
    buf = bytearray(CMD_FRAME_LEN)
    buf[0] = SYNC_CMD
    buf[1] = device_id & 0xFF
    buf[2] = (CMD_ENABLE_BIT if enabled else 0x00) | ((thr >> 8) & CMD_THR_HI_MASK)
    buf[3] = thr & 0xFF
    buf[4] = crc8(buf[:4])
    return buf


class EscStatus:
    """H743侧接收到的、来自Pico的电调状态镜像"""
    __slots__ = (
        "bidir_valid", "kiss_valid", "enabled",
        "erpm", "temperature", "voltage", "current",
    )

    def __init__(self):
        self.bidir_valid = False
        self.kiss_valid = False
        self.enabled = False
        self.erpm = 0
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
    """按sync字节+固定长度+CRC8从字节流里切帧，自动丢字节重新同步。
    注意：部分MicroPython端口的bytearray不支持 del / 变长切片赋值，
    所以这里用 bytes + 下标游走，最后用切片重新赋值，不做任何原地删除。"""

    def __init__(self, sync_byte, frame_len):
        self.sync = sync_byte
        self.frame_len = frame_len
        self.buf = b""
        self.dropped_bytes = 0     # 因找不到sync而丢弃的字节数
        self.crc_fail = 0          # sync对上但CRC不过的次数

    def feed(self, data):
        if data:
            self.buf = self.buf + bytes(data)

        frames = []
        buf = self.buf
        n = len(buf)
        fl = self.frame_len
        sync = self.sync
        pos = 0

        while True:
            while pos < n and buf[pos] != sync:
                pos += 1
                self.dropped_bytes += 1

            if n - pos < fl:
                break

            candidate = buf[pos:pos + fl]

            if crc8(candidate[:-1]) == candidate[-1]:
                frames.append(candidate)
                pos += fl
            else:
                pos += 1
                self.crc_fail += 1

        self.buf = buf[pos:]
        return frames


class UartLink:
    def __init__(self):
        self.uart = UART(UART_ID, UART_BAUD, read_buf_len=256)

        self.statuses = [EscStatus() for _ in range(MAX_ESC)]
        self.commands = [EscCommand() for _ in range(MAX_ESC)]
        self.link_ok = False
        self.throttle_limit = MAX_THROTTLE     # 可在应用层调低作为软件限幅

        self._rx = FrameReceiver(SYNC_STATUS, STATUS_FRAME_LEN)
        self._send_idx = 0
        self._last_cmd_ms = time.ticks_ms()

        self.last_rx_ms = time.ticks_ms()
        self.LINK_TIMEOUT_MS = LINK_TIMEOUT_MS

        # 每路最近一次收到状态帧的时间(判断遥测是否过期)
        self._status_seen = [False] * MAX_ESC
        self._status_ms = [0] * MAX_ESC

        # 统计(诊断用，不影响功能)
        self.rx_frames = 0
        self.rx_frames_dev = [0] * MAX_ESC
        self.last_frame_ms_dev = [time.ticks_ms()] * MAX_ESC
        self.gap_max_ms_dev = [0] * MAX_ESC
        self.link_drop_count = 0       # link_ok True->False 的次数
        self.tx_frames = 0

    # -------------------- 统计 --------------------
    def reset_stats(self):
        now = time.ticks_ms()
        self.rx_frames = 0
        self.rx_frames_dev = [0] * MAX_ESC
        self.last_frame_ms_dev = [now] * MAX_ESC
        self.gap_max_ms_dev = [0] * MAX_ESC
        self.link_drop_count = 0
        self.tx_frames = 0
        self._rx.dropped_bytes = 0
        self._rx.crc_fail = 0

    @property
    def crc_fail(self):
        return self._rx.crc_fail

    @property
    def dropped_bytes(self):
        return self._rx.dropped_bytes

    # -------------------- 主循环入口 --------------------
    def update(self):
        """主循环每圈调用一次：收状态帧 + 按节奏轮询发命令(每2ms一路)。返回 link_ok。"""
        self.poll_status()
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_cmd_ms) >= CMD_PERIOD_MS:
            self.send_command(self._send_idx)
            self._send_idx = (self._send_idx + 1) % MAX_ESC
            self._last_cmd_ms = now
        return self.link_ok

    # -------------------- 接收 Pico 状态帧 --------------------
    def poll_status(self):
        n = self.uart.any()
        data = self.uart.read(n) if n else None

        frames = self._rx.feed(data)

        for f in frames:
            self._apply_status_frame(f)
            now = time.ticks_ms()
            self.last_rx_ms = now
            self.link_ok = True

            self.rx_frames += 1
            d = f[1]
            if d < MAX_ESC:
                self.rx_frames_dev[d] += 1
                gap = time.ticks_diff(now, self.last_frame_ms_dev[d])
                if gap > self.gap_max_ms_dev[d]:
                    self.gap_max_ms_dev[d] = gap
                self.last_frame_ms_dev[d] = now

        if time.ticks_diff(time.ticks_ms(), self.last_rx_ms) > self.LINK_TIMEOUT_MS:
            if self.link_ok:
                self.link_drop_count += 1
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

        st.erpm = (buf[3] << 24) | (buf[4] << 16) | (buf[5] << 8) | buf[6]
        st.temperature = buf[7]
        st.voltage = ((buf[8] << 8) | buf[9]) / 100.0
        st.current = ((buf[10] << 8) | buf[11]) / 100.0

        self._status_seen[device_id] = True
        self._status_ms[device_id] = time.ticks_ms()

    # -------------------- 遥测读取（带新鲜度）--------------------
    def status_age_ms(self, device_id):
        """距离上次收到该路状态帧过了多久(ms)；从未收到返回一个很大的数。"""
        if not self._status_seen[device_id]:
            return 0x7FFFFFFF
        return time.ticks_diff(time.ticks_ms(), self._status_ms[device_id])

    def status_fresh(self, device_id, max_age_ms=STATUS_FRESH_MS):
        return self.status_age_ms(device_id) <= max_age_ms

    @staticmethod
    def source_of(st):
        if st.bidir_valid:
            return "BDShot"
        if st.kiss_valid:
            return "KISS"
        return "none"

    def get_telemetry(self, device_id, pole_pairs=1, max_age_ms=STATUS_FRESH_MS):
        """
        一次性取某一路的遥测快照(dict)。
          fresh : 该路状态帧是否在 max_age_ms 内收到过；False 时其余数值不应使用
          source: "BDShot"/"KISS"/"none"。none 表示 Pico 两路遥测都无效，数值恒为0
          erpm  : 电气转速；rpm = erpm / pole_pairs (pole_pairs 为电机极对数)
        """
        st = self.statuses[device_id]
        age = self.status_age_ms(device_id)
        return {
            "fresh": age <= max_age_ms,
            "age_ms": age,
            "enabled": st.enabled,
            "source": self.source_of(st),
            "erpm": st.erpm,
            "rpm": st.erpm / pole_pairs if pole_pairs else st.erpm,
            "temperature": st.temperature,
            "voltage": st.voltage,
            "current": st.current,
        }

    # -------------------- 下发命令给 Pico --------------------
    def _check_id(self, device_id):
        if not (0 <= device_id < MAX_ESC):
            raise ValueError("device_id 必须在 0~%d" % (MAX_ESC - 1))

    def enable(self, device_id):
        """启用某一路(下一次发送时生效)"""
        self._check_id(device_id)
        self.commands[device_id].enabled = True

    def disable(self, device_id):
        """禁用某一路，同时把期望油门清零"""
        self._check_id(device_id)
        self.commands[device_id].enabled = False
        self.commands[device_id].throttle = 0

    def enable_channels(self, channels):
        """只启用 channels 里的路，其余全部禁用(油门清零)。"""
        chs = list(channels)
        for c in chs:
            self._check_id(c)
        for c in range(MAX_ESC):
            if c in chs:
                self.commands[c].enabled = True
            else:
                self.disable(c)

    def set_throttle(self, device_id, throttle):
        """
        设置某一路期望油门。只有 enabled=True 时才会真正被 Pico 采用。
        0 = 停转；1~47 自动抬到 48(这些是DShot命令)；上限 self.throttle_limit。
        """
        self._check_id(device_id)
        t = int(throttle)
        if t <= 0:
            t = 0
        elif t < MIN_THROTTLE:
            t = MIN_THROTTLE
        elif t > self.throttle_limit:
            t = self.throttle_limit
        self.commands[device_id].throttle = t

    def set_throttles(self, mapping):
        """批量设置。mapping 可以是 {通道: 油门} 的字典。"""
        for c, t in mapping.items():
            self.set_throttle(c, t)

    def stop_all(self):
        """把所有路油门清零(保持enable状态)。下一轮 update() 会下发。"""
        for c in range(MAX_ESC):
            self.commands[c].throttle = 0

    def emergency_stop(self):
        """立刻全部禁用并连续下发数次(阻塞约30ms)。异常/退出时务必调用。"""
        for c in range(MAX_ESC):
            self.commands[c].enabled = False
            self.commands[c].throttle = 0
        for _ in range(5):
            self.send_all_commands()
            time.sleep_ms(5)

    def send_raw(self, data):
        """直接发原始字节(故障注入/调试用)"""
        self.uart.write(data)

    def send_command(self, device_id, cmd=None):
        """把 self.commands[device_id] (或显式传入的cmd) 编码发送给Pico"""
        c = cmd if cmd is not None else self.commands[device_id]
        thr = max(0, min(0x07FF, int(c.throttle))) if c.enabled else 0
        self.uart.write(build_cmd_frame(device_id, c.enabled, thr))
        self.tx_frames += 1

    def send_all_commands(self):
        for device_id in range(MAX_ESC):
            self.send_command(device_id)


# ================= 使用示例：多电机缓升 =================
# 运行前：拆桨、固定电机！DEMO_THROTTLE 先给小值。
DEMO_CHANNELS = [0, 1, 2]
DEMO_THROTTLE = 200
DEMO_RAMP_MS = 3000
DEMO_HOLD_MS = 5000
DEMO_POLE_PAIRS = 7          # 电机极对数(用于把 eRPM 换算成机械 RPM)


def demo():
    link = UartLink()
    link.enable_channels(DEMO_CHANNELS)
    t0 = time.ticks_ms()
    last_print = t0
    try:
        print("等待链路与 ESC 解锁(油门0, 3s)...")
        t_arm = time.ticks_add(t0, 3000)
        while time.ticks_diff(t_arm, time.ticks_ms()) > 0:
            link.update()
            time.sleep_ms(1)
        if not link.link_ok:
            print("!! 没有收到 Pico 状态帧，已中止")
            return

        t_start = time.ticks_ms()
        total = DEMO_RAMP_MS + DEMO_HOLD_MS
        while True:
            el = time.ticks_diff(time.ticks_ms(), t_start)
            if el >= total:
                break
            thr = DEMO_THROTTLE * min(el, DEMO_RAMP_MS) // DEMO_RAMP_MS
            for c in DEMO_CHANNELS:
                link.set_throttle(c, thr)
            link.update()

            now = time.ticks_ms()
            if time.ticks_diff(now, last_print) >= 500:
                last_print = now
                for c in DEMO_CHANNELS:
                    t = link.get_telemetry(c, DEMO_POLE_PAIRS)
                    print("ESC%d %s fresh=%d thr=%d rpm=%.0f V=%.2f I=%.2f T=%d" % (
                        c + 1, t["source"], t["fresh"], link.commands[c].throttle,
                        t["rpm"], t["voltage"], t["current"], t["temperature"]))
            time.sleep_ms(1)
    except KeyboardInterrupt:
        print("用户中断")
    finally:
        link.emergency_stop()
        print("已停机")


if __name__ == "__main__":
    demo()