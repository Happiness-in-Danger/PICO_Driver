'''
注意此文件运行在主控部份(H743)！Pico 模块联调/验收测试套件。
依赖同目录下的正式版 Driver.py（协议与 UartLink 都来自那里）。

用法 (在 H743 REPL / Thonny 里):
    >>> import Driver_test as T
    >>> T.run_menu()                       # 交互菜单
    >>> T.run(["link", "enable"])          # 只跑指定项(不转电机)
    >>> T.run(["ramp"], channels=[0])      # 指定通道
    >>> T.quick(0, 120, 6)                 # 单路手动试转(通道, 油门, 秒)
    >>> T.diag()                           # 链路不通时：直接看UART收到的原始字节
    >>> T.monitor()                        # 只监视4路状态

测试项 key:
    link      T1 通讯链路: 状态帧速率/最大间隔/CRC丢帧
    enable    T2 enable/disable 逐路控制
    robust    T3 抗干扰: 坏CRC帧/垃圾字节重同步/非法device_id
    latency   T4 命令->状态回读延迟
    ramp      T5 油门阶梯 + 遥测合理性(需电机, 已拆桨)
    indep     T6 通道独立性(需>=2路电机)
    failsafe  T7 软超时100ms/硬超时2s(需电机)
    degrade   T8 单路 BDShot->Signal_DShot 降级(手动配合)
    soak      T9 长时间稳定性

!!! 安全 !!! 带电机的测试必须拆桨并固定电机。DShot 值 1~47 是命令不是油门，
    clamp_thr() 会自动抬到 48。测试异常/Ctrl-C 时 finally 里会 safe_stop()。
'''
from pyb import UART
import time

from Driver import (
    UartLink, FrameReceiver, build_cmd_frame,
    MAX_ESC, SYNC_STATUS, STATUS_FRAME_LEN, UART_ID, UART_BAUD,
)

# =====================================================================
#                         测 试 套 件
# =====================================================================
# ----------------------------- 测试配置 -----------------------------
TEST_CHANNELS = [0, 1, 2]        # 实际接了电机的通道(0~3)。2路就写[0,1]
TEST_THR_STEPS = (80, 120, 160)  # 油门阶梯(必须>=48)。先小后大，确认安全再加
TEST_MAX_THR = 400               # 任何测试都不会超过此油门(安全上限)
TEST_MIN_ERPM = 100              # 认为"电机在转"的最小 eRPM
VBUS_RANGE = (5.0, 30.0)         # 电压合理范围(V)，按你的电池改
TEMP_RANGE = (0, 100)            # 温度合理范围(℃)
STATUS_MIN_HZ = 40               # 每路状态帧最低速率(理论约125Hz)
SPIN_HOLD_MS = 1500              # 每个油门台阶保持时间
ARM_WAIT_MS = 3000               # 带电机测试前先发0油门让ESC解锁
CMD_PERIOD_MS = 2                # 与原示例一致：每2ms发一路命令(每路8ms一次)


def clamp_thr(t):
    """DShot 1~47 是命令，不是油门；这里保证要么0要么>=48，且不超安全上限"""
    t = int(t)
    if t <= 0:
        return 0
    if t < 48:
        t = 48
    return min(t, TEST_MAX_THR)


class EscTester:
    def __init__(self, link, channels=None):
        self.link = link
        self.channels = list(channels if channels is not None else TEST_CHANNELS)
        for c in self.channels:
            if not (0 <= c < MAX_ESC):
                raise ValueError("TEST_CHANNELS 里的通道必须在 0~%d" % (MAX_ESC - 1))
        self.results = []
        self._send_idx = 0
        self._last_cmd_ms = time.ticks_ms()
        self._props_ok = False

    # ------------------------- 结果输出 -------------------------
    def section(self, title):
        print("\n" + "=" * 62)
        print(title)
        print("=" * 62)

    def check(self, name, ok, detail=""):
        self.results.append((name, ok, detail))
        print("  [%s] %s%s" % ("PASS" if ok else "FAIL", name,
                               ("  -> " + detail) if detail else ""))
        return ok

    def info(self, msg):
        print("  [INFO] " + msg)

    def skip(self, name, why):
        self.results.append((name, None, why))
        print("  [SKIP] %s  -> %s" % (name, why))

    def summary(self):
        p = sum(1 for r in self.results if r[1] is True)
        f = sum(1 for r in self.results if r[1] is False)
        s = sum(1 for r in self.results if r[1] is None)
        print("\n" + "#" * 62)
        print("测试汇总: PASS=%d  FAIL=%d  SKIP=%d" % (p, f, s))
        for name, ok, detail in self.results:
            if ok is False:
                print("  FAIL: %s  (%s)" % (name, detail))
        print("#" * 62)

    # ------------------------- 基础设施 -------------------------
    def pump(self, ms, send=True, skip=(), forged=None, sample=None):
        """
        主循环跑 ms 毫秒：收状态帧 +（可选）按原示例节奏轮询发命令。
          send=False : 完全不发命令（用来触发Pico超时保护）
          skip       : 这些通道不发常规命令
          forged     : {device_id: bytes} 轮到该通道时发这帧伪造数据，替代常规命令
          sample     : 每个循环调用一次的回调（采集数据）
        """
        forged = forged or {}
        t_end = time.ticks_add(time.ticks_ms(), ms)
        while time.ticks_diff(t_end, time.ticks_ms()) > 0:
            self.link.poll_status()
            now = time.ticks_ms()
            if send and time.ticks_diff(now, self._last_cmd_ms) >= CMD_PERIOD_MS:
                d = self._send_idx
                if d in forged:
                    self.link.send_raw(forged[d])
                elif d not in skip:
                    self.link.send_command(d)
                self._send_idx = (d + 1) % MAX_ESC
                self._last_cmd_ms = now
            if sample:
                sample()
            time.sleep_ms(1)

    def measure(self, ms, chs, **kw):
        """跑 ms 毫秒并统计 chs 各路 erpm 的 平均/最小/最大。返回 {ch: (avg,min,max,status)}"""
        acc = {}
        for c in chs:
            acc[c] = [0, 0, 0x7FFFFFFF, 0]

        def s():
            for c in chs:
                e = self.link.statuses[c].raw_erpm
                a = acc[c]
                a[0] += e
                a[1] += 1
                if e < a[2]:
                    a[2] = e
                if e > a[3]:
                    a[3] = e
        self.pump(ms, sample=s, **kw)
        out = {}
        for c in chs:
            a = acc[c]
            n = a[1]
            out[c] = ((a[0] / n) if n else 0, a[2] if n else 0, a[3], self.link.statuses[c])
        return out

    def all_off(self):
        for c in range(MAX_ESC):
            self.link.disable(c)

    def safe_stop(self):
        """任何情况下都能把所有路关掉"""
        try:
            self.all_off()
            for _ in range(10):
                self.link.send_all_commands()
                time.sleep_ms(2)
            self.pump(200)
        except Exception as e:
            print("[safe_stop] 异常:", e)

    def confirm(self, msg):
        try:
            ans = input("\n>>> " + msg + "  (输入 y 继续, 其它跳过): ")
        except Exception:
            return False
        return ans.strip().lower() in ("y", "yes")

    def need_props_removed(self):
        if self._props_ok:
            return True
        ok = self.confirm("【安全】已拆除所有桨叶、电机已固定、周围无人？")
        self._props_ok = ok
        return ok

    def arm(self):
        """所有测试通道 enable + 油门0，等待ESC解锁"""
        self.all_off()
        for c in self.channels:
            self.link.enable(c)
            self.link.set_throttle(c, 0)
        self.info("发送 0 油门 %d ms 等待 ESC 解锁..." % ARM_WAIT_MS)
        self.pump(ARM_WAIT_MS)

    @staticmethod
    def src(st):
        if st.bidir_valid:
            return "BDShot"
        if st.kiss_valid:
            return "KISS"
        return "none"

    # ===================== T1 通讯链路 =====================
    def t_link(self):
        self.section("T1 通讯链路: 状态帧速率 / 最大间隔 / CRC")
        self.all_off()
        self.pump(300)
        self.link.reset_stats()
        dur = 2000
        self.pump(dur)

        self.check("link_ok (H743能持续收到Pico状态帧)", self.link.link_ok)
        for d in range(MAX_ESC):
            hz = self.link.rx_frames_dev[d] * 1000.0 / dur
            self.check("ESC%d 状态帧速率>=%dHz" % (d + 1, STATUS_MIN_HZ),
                       hz >= STATUS_MIN_HZ,
                       "%.1fHz, 最大间隔%dms" % (hz, self.link.gap_max_ms_dev[d]))
        gmax = max(self.link.gap_max_ms_dev)
        self.check("状态帧最大间隔<100ms (否则H743自身link_ok会闪断)", gmax < 100,
                   "max=%dms" % gmax)
        self.check("H743侧CRC错误率<1%",
                   self.link.crc_fail <= max(1, self.link.rx_frames // 100),
                   "crc_fail=%d / frames=%d, 丢弃字节=%d" %
                   (self.link.crc_fail, self.link.rx_frames, self.link.dropped_bytes))
        self.check("测试期间 link 没有掉线", self.link.link_drop_count == 0,
                   "drop=%d" % self.link.link_drop_count)

    # ===================== T2 enable / disable =====================
    def t_enable(self):
        self.section("T2 enable/disable 逐路控制（油门恒为0，不转电机）")
        self.all_off()
        self.pump(400)
        self.check("初始: 所有路 enabled=False",
                   all(not self.link.statuses[c].enabled for c in range(MAX_ESC)),
                   str([self.link.statuses[c].enabled for c in range(MAX_ESC)]))

        for c in range(MAX_ESC):
            self.link.enable(c)
            self.pump(300)
            en = [self.link.statuses[i].enabled for i in range(MAX_ESC)]
            ok = en[c] and all((not en[i]) for i in range(MAX_ESC) if i != c)
            self.check("只enable ESC%d -> 仅该路 enabled" % (c + 1), ok, str(en))
            self.link.disable(c)
            self.pump(300)
            self.check("disable ESC%d -> enabled=False" % (c + 1),
                       not self.link.statuses[c].enabled)

        for c in range(MAX_ESC):
            self.link.enable(c)
        self.pump(400)
        self.check("4路同时enable",
                   all(self.link.statuses[c].enabled for c in range(MAX_ESC)))
        self.all_off()
        self.pump(300)
        self.check("4路同时disable",
                   all(not self.link.statuses[c].enabled for c in range(MAX_ESC)))

    # ===================== T3 抗干扰 =====================
    def t_robust(self):
        self.section("T3 抗干扰: 坏CRC / 垃圾字节重同步 / 非法device_id")
        ch = self.channels[0]
        spare = [c for c in range(MAX_ESC) if c != ch][0]
        self.all_off()
        self.pump(300)

        # 先让 ch enable(油门0)，然后停发命令；100ms~2s 之间 enabled 会被保留
        self.link.enable(ch)
        self.link.send_command(ch)
        self.pump(200, send=False)
        self.check("前置: ESC%d enabled" % (ch + 1), self.link.statuses[ch].enabled)

        # 1) 坏CRC的"disable"帧必须被忽略
        f = build_cmd_frame(ch, False, 0)
        f[4] ^= 0xFF
        self.link.send_raw(f)
        self.pump(300, send=False)
        self.check("坏CRC的disable帧被忽略(enabled仍为True)", self.link.statuses[ch].enabled)

        # 2) 垃圾字节 + 假sync 之后紧跟合法帧，应能重新同步并被接受
        self.link.send_raw(bytes([0x00, 0xFF, 0xA5, 0xA5, 0x01, 0x7F]))
        self.link.send_raw(build_cmd_frame(spare, True, 0))
        self.pump(300, send=False)
        self.check("垃圾字节后自动重同步，合法enable帧被接受",
                   self.link.statuses[spare].enabled)

        # 3) 非法 device_id(>=4) 的合法CRC帧：忽略，Pico不崩溃，链路继续
        before = self.link.rx_frames
        en_before = [self.link.statuses[i].enabled for i in range(MAX_ESC)]
        self.link.send_raw(build_cmd_frame(4, True, 500))
        self.link.send_raw(build_cmd_frame(7, True, 500))
        self.link.send_raw(build_cmd_frame(255, True, 500))
        self.pump(400, send=False)
        en_after = [self.link.statuses[i].enabled for i in range(MAX_ESC)]
        self.check("非法device_id不影响任何通道", en_before == en_after,
                   "%s -> %s" % (en_before, en_after))
        self.check("非法device_id后Pico仍在发状态帧(未崩溃)",
                   self.link.rx_frames - before > 10 and self.link.link_ok)

        # 4) 合法disable帧被接受
        self.link.send_raw(build_cmd_frame(ch, False, 0))
        self.link.send_raw(build_cmd_frame(spare, False, 0))
        self.pump(300, send=False)
        self.check("合法disable帧被接受",
                   (not self.link.statuses[ch].enabled) and (not self.link.statuses[spare].enabled))

        # 5) 命令帧风暴(各路高速狂发)后链路仍正常
        self.link.reset_stats()
        for _ in range(200):
            for c in range(MAX_ESC):
                self.link.send_raw(build_cmd_frame(c, False, 0))
            self.link.poll_status()
            time.sleep_ms(1)
        self.pump(300)
        self.check("命令高速连发(~800帧/s)后链路正常", self.link.link_ok,
                   "rx=%d crc_fail=%d" % (self.link.rx_frames, self.link.crc_fail))

    # ===================== T4 往返延迟 =====================
    def t_latency(self):
        self.section("T4 命令 -> 状态回读延迟 (enable 位翻转, 油门0)")
        ch = self.channels[0]
        self.all_off()
        self.pump(400)
        lat = []
        fail = 0
        for k in range(20):
            want = (k % 2 == 0)
            if want:
                self.link.enable(ch)
            else:
                self.link.disable(ch)
            t0 = time.ticks_us()
            self.link.send_command(ch)
            got = False
            while time.ticks_diff(time.ticks_us(), t0) < 200000:
                self.link.poll_status()
                if self.link.statuses[ch].enabled == want:
                    got = True
                    break
            if got:
                lat.append(time.ticks_diff(time.ticks_us(), t0) / 1000.0)
            else:
                fail += 1
            self.pump(100, send=False)   # 等状态稳定
        self.link.disable(ch)
        self.pump(200)
        if lat:
            self.info("延迟 min=%.1fms avg=%.1fms max=%.1fms (含Pico状态帧轮询周期~8ms)" %
                      (min(lat), sum(lat) / len(lat), max(lat)))
        self.check("20次翻转全部回读成功", fail == 0, "失败%d次" % fail)
        self.check("最大往返延迟<100ms", bool(lat) and max(lat) < 100)

    # ===================== T5 油门阶梯 =====================
    def t_ramp(self):
        self.section("T5 油门阶梯 + 遥测合理性 (需电机，已拆桨)")
        if not self.need_props_removed():
            self.skip("ramp", "未确认拆桨")
            return
        self.arm()
        self.pump(500)
        self.info("---- 零油门遥测检查 (ESC在0油门时也应回包) ----")
        for c in self.channels:
            st = self.link.statuses[c]
            print("   ESC%d en=%d bd=%d kiss=%d  V=%.2f I=%.2f T=%d erpm=%d" %
                  (c + 1, st.enabled, st.bidir_valid, st.kiss_valid,
                   st.voltage, st.current, st.temperature, st.raw_erpm))
            self.check("ESC%d enabled 标志已置位" % (c + 1), st.enabled)
            self.check("ESC%d 零油门时已有遥测来源" % (c + 1),
                       st.bidir_valid or st.kiss_valid, self.src(st))
        hold = SPIN_HOLD_MS
        steps = [clamp_thr(t) for t in TEST_THR_STEPS]

        for c in self.channels:
            self.info("---- ESC%d ----" % (c + 1))
            prev = -1
            last_avg = 0
            for t in steps:
                self.link.set_throttle(c, t)
                self.pump(hold - 600)
                r = self.measure(600, [c])[c]
                avg, mn, mx, st = r
                print("   thr=%4d  erpm avg=%8.0f [%d..%d]  V=%5.2f  I=%5.2f  T=%3d  src=%s en=%d bd=%d kiss=%d" %
                      (t, avg, mn, mx, st.voltage, st.current, st.temperature, self.src(st),
                       st.enabled, st.bidir_valid, st.kiss_valid))
                self.check("ESC%d thr=%d 电机在转(erpm>%d)" % (c + 1, t, TEST_MIN_ERPM),
                           avg > TEST_MIN_ERPM, "%.0f" % avg)
                self.check("ESC%d thr=%d 有有效遥测来源" % (c + 1, t),
                           st.bidir_valid or st.kiss_valid, self.src(st))
                self.check("ESC%d thr=%d 电压在%.0f~%.0fV" % (c + 1, t, VBUS_RANGE[0], VBUS_RANGE[1]),
                           VBUS_RANGE[0] <= st.voltage <= VBUS_RANGE[1], "%.2fV" % st.voltage)
                self.check("ESC%d thr=%d 温度在%d~%d℃" % (c + 1, t, TEMP_RANGE[0], TEMP_RANGE[1]),
                           TEMP_RANGE[0] <= st.temperature <= TEMP_RANGE[1], "%dC" % st.temperature)
                if prev >= 0:
                    self.check("ESC%d 油门升高 erpm 随之升高 (thr %d)" % (c + 1, t),
                               avg > prev, "%.0f -> %.0f" % (prev, avg))
                prev = avg
                last_avg = avg
                # 稳定性: 同一油门下 erpm 波动不应过大
                if avg > 0:
                    self.check("ESC%d thr=%d erpm稳定(波动<30%%)" % (c + 1, t),
                               (mx - mn) < 0.3 * avg, "span=%d" % (mx - mn))
            self.link.set_throttle(c, 0)
            self.pump(1500)
            e = self.link.statuses[c].raw_erpm
            self.check("ESC%d 油门回0后停转(erpm<%d)" % (c + 1, TEST_MIN_ERPM), e < TEST_MIN_ERPM, "%d" % e)

        # 所有路同时同油门
        t = steps[len(steps) // 2]
        self.info("---- 所有测试通道同时 thr=%d ----" % t)
        for c in self.channels:
            self.link.set_throttle(c, t)
        self.pump(hold)
        r = self.measure(600, self.channels)
        for c in self.channels:
            avg, mn, mx, st = r[c]
            print("   ESC%d erpm=%8.0f V=%5.2f I=%5.2f T=%3d src=%s" %
                  (c + 1, avg, st.voltage, st.current, st.temperature, self.src(st)))
            self.check("多路同时运行 ESC%d 在转" % (c + 1), avg > TEST_MIN_ERPM, "%.0f" % avg)
        self.safe_stop()

    # ===================== T6 通道独立性 =====================
    def t_indep(self):
        self.section("T6 通道独立性 + Pico侧 disabled 强制油门0 (需电机)")
        if len(self.channels) < 2:
            self.skip("indep", "至少需要2个测试通道")
            return
        if not self.need_props_removed():
            self.skip("indep", "未确认拆桨")
            return
        self.arm()
        a, b = self.channels[0], self.channels[1]
        others = self.channels[2:]
        t = clamp_thr(TEST_THR_STEPS[len(TEST_THR_STEPS) // 2])

        # a 正常运行; b 被喂"enabled=0 但油门=t"的伪造帧(模拟H743异常/协议被破坏)
        self.link.set_throttle(a, t)
        forged = {b: build_cmd_frame(b, False, t)}
        self.info("ESC%d 正常 thr=%d；ESC%d 持续收到 enable=0/thr=%d 的伪造帧" % (a + 1, t, b + 1, t))
        self.pump(SPIN_HOLD_MS, forged=forged)
        r = self.measure(800, [a, b] + others, forged=forged)
        self.check("ESC%d 正常转动" % (a + 1), r[a][0] > TEST_MIN_ERPM, "%.0f" % r[a][0])
        self.check("ESC%d (disabled但带油门) 必须不转" % (b + 1), r[b][2] < TEST_MIN_ERPM,
                   "max erpm=%d" % r[b][2])
        self.check("ESC%d enabled 标志为False" % (b + 1), not r[b][3].enabled)
        for c in others:
            self.check("ESC%d (enabled,油门0) 不转" % (c + 1), r[c][2] < TEST_MIN_ERPM,
                       "max erpm=%d" % r[c][2])

        # 动态: 转动中 disable a，应立即停；其余路不受影响
        self.link.set_throttle(a, t)
        self.link.enable(b)                # 启用b，并给油门(disable() 会清油门，所以要重新设)
        self.link.set_throttle(b, t)
        self.pump(SPIN_HOLD_MS)
        r = self.measure(600, [a, b])
        self.check("ESC%d/%d 同时运行" % (a + 1, b + 1),
                   r[a][0] > TEST_MIN_ERPM and r[b][0] > TEST_MIN_ERPM,
                   "%.0f / %.0f" % (r[a][0], r[b][0]))
        ea_before = r[a][0]
        self.link.disable(b)
        self.pump(1500)
        r = self.measure(600, [a, b])
        self.check("运行中disable ESC%d -> 停转" % (b + 1), r[b][2] < TEST_MIN_ERPM, "max=%d" % r[b][2])
        self.check("ESC%d 不受影响，仍在转(>=60%%原转速)" % (a + 1),
                   r[a][0] > 0.6 * ea_before, "%.0f -> %.0f" % (ea_before, r[a][0]))
        self.safe_stop()

    # ===================== T7 failsafe =====================
    def t_failsafe(self):
        self.section("T7 链路超时保护: 软超时100ms / 硬超时2000ms (需电机)")
        if not self.need_props_removed():
            self.skip("failsafe", "未确认拆桨")
            return
        self.arm()
        ch = self.channels[0]
        t = clamp_thr(TEST_THR_STEPS[len(TEST_THR_STEPS) // 2])
        self.link.set_throttle(ch, t)
        self.pump(SPIN_HOLD_MS)
        base = self.measure(500, [ch])[ch][0]
        self.check("基准: 电机在转", base > TEST_MIN_ERPM, "%.0f" % base)
        if base <= TEST_MIN_ERPM:
            self.safe_stop()
            return

        # ---- 软超时：停发命令 -> 油门清零(电机停)，enabled 保留 ----
        self.info("停止发送命令 800ms (>100ms软超时, <2s硬超时)...")
        self.pump(800, send=False)
        st = self.link.statuses[ch]
        self.check("软超时: 电机停转/减速(<30%基准)", st.raw_erpm < 0.3 * base, "%d (基准%.0f)" % (st.raw_erpm, base))
        self.check("软超时: enabled 标志保留", st.enabled)
        self.check("软超时: Pico仍持续发状态帧(H743 link_ok)", self.link.link_ok)

        # ---- 恢复发送：应自动恢复 ----
        self.pump(SPIN_HOLD_MS)
        r = self.measure(500, [ch])[ch]
        self.check("恢复发命令后电机重新运转(>50%基准)", r[0] > 0.5 * base, "%.0f" % r[0])

        # ---- 硬超时：停发2.5s -> enabled 被清除 ----
        self.info("停止发送命令 2500ms (>2s硬超时)...")
        self.pump(1000, send=False)
        self.check("硬超时前(1s): enabled 仍保留", self.link.statuses[ch].enabled)
        self.pump(1500, send=False)
        st = self.link.statuses[ch]
        self.check("硬超时后: enabled 被清除", not st.enabled)
        self.check("硬超时后: 电机已停", st.raw_erpm < TEST_MIN_ERPM, "%d" % st.raw_erpm)

        # 恢复前先把油门清0，避免突然重新起转
        self.link.set_throttle(ch, 0)
        self.pump(500)
        self.check("硬超时后恢复发命令: enabled 自动重新置位(由H743命令决定)",
                   self.link.statuses[ch].enabled)
        self.safe_stop()

    # ===================== T8 单路降级 (手动) =====================
    def t_degrade(self):
        self.section("T8 单路 BDShot -> Signal_DShot 降级 (手动配合，需电机)")
        if not self.need_props_removed():
            self.skip("degrade", "未确认拆桨")
            return
        ch = self.channels[0]
        print("  方法: 让 ESC%d 的 BDShot 失去回包但电机仍受DShot驱动，例如:" % (ch + 1))
        print("    - 在 ESC 配置器里关闭该电调的 Bidirectional DShot（推荐）")
        print("    - 或让该路遥测回包不可用而信号线保持连接")
        print("  Pico应在 BDSHOT_STALE_MS(300ms) 后只把该路切到 Signal_DShot；")
        print("  KISS遥测线接着的话 kiss_valid 应置位，电机应继续运转，其它路不受影响。")
        if not self.confirm("准备好了吗？（测试开始后你有 25 秒操作）"):
            self.skip("degrade", "用户跳过")
            return
        self.arm()
        t = clamp_thr(TEST_THR_STEPS[0])
        self.link.set_throttle(ch, t)
        self.pump(SPIN_HOLD_MS)
        others = [c for c in self.channels if c != ch]
        for c in others:
            self.link.set_throttle(c, t)
        self.pump(SPIN_HOLD_MS)
        base_bd = [self.link.statuses[c].bidir_valid for c in others]

        state = {"lost_t": None, "kiss_t": None, "bd_seen": False, "t0": time.ticks_ms()}

        def s():
            st = self.link.statuses[ch]
            now = time.ticks_diff(time.ticks_ms(), state["t0"])
            if st.bidir_valid:
                state["bd_seen"] = True
            if state["bd_seen"] and (not st.bidir_valid) and state["lost_t"] is None:
                state["lost_t"] = now
            if st.kiss_valid and state["kiss_t"] is None and state["lost_t"] is not None:
                state["kiss_t"] = now

        self.info("现在请操作 ESC%d ...（25秒）" % (ch + 1))
        self.pump(25000, sample=s)
        st = self.link.statuses[ch]
        r = self.measure(500, [ch] + others)

        if not state["bd_seen"]:
            self.check("开始时 ESC%d 的 BDShot 有效" % (ch + 1), False, "测试期间从未见到bidir_valid")
        else:
            self.check("检测到 ESC%d BDShot 失效" % (ch + 1), state["lost_t"] is not None,
                       "t=%sms" % state["lost_t"])
            if state["lost_t"] is not None:
                self.check("降级后 KISS 遥测接管(kiss_valid)", state["kiss_t"] is not None or st.kiss_valid,
                           "kiss_valid=%s" % st.kiss_valid)
                self.check("降级后 ESC%d 仍被驱动(erpm>%d)" % (ch + 1, TEST_MIN_ERPM),
                           r[ch][0] > TEST_MIN_ERPM, "%.0f" % r[ch][0])
        for i, c in enumerate(others):
            self.check("ESC%d 不受降级影响(BDShot状态不变)" % (c + 1),
                       self.link.statuses[c].bidir_valid == base_bd[i] and r[c][0] > TEST_MIN_ERPM,
                       "bidir=%s erpm=%.0f" % (self.link.statuses[c].bidir_valid, r[c][0]))
        self.info("注意：降级后不会自动恢复，需重启Pico才会重新走BDShot初始化。")
        self.safe_stop()

    # ===================== T9 浸泡 =====================
    def t_soak(self, seconds=60):
        self.section("T9 长时间稳定性 (%ds)" % seconds)
        if not self.need_props_removed():
            self.skip("soak", "未确认拆桨")
            return
        self.arm()
        t = clamp_thr(TEST_THR_STEPS[0])
        for c in self.channels:
            self.link.set_throttle(c, t)
        self.pump(SPIN_HOLD_MS)
        self.link.reset_stats()
        dropout = {c: 0 for c in self.channels}
        zero_cnt = {c: 0 for c in self.channels}
        samples = [0]

        def s():
            samples[0] += 1
            for c in self.channels:
                st = self.link.statuses[c]
                if not (st.bidir_valid or st.kiss_valid):
                    dropout[c] += 1
                if st.raw_erpm < TEST_MIN_ERPM:
                    zero_cnt[c] += 1

        t0 = time.ticks_ms()
        next_print = 10000
        while time.ticks_diff(time.ticks_ms(), t0) < seconds * 1000:
            self.pump(1000, sample=s)
            el = time.ticks_diff(time.ticks_ms(), t0)
            if el >= next_print:
                next_print += 10000
                print("   t=%3ds " % (el // 1000) + "  ".join(
                    "ESC%d:%7.0f/%s" % (c + 1, self.link.statuses[c].raw_erpm, self.src(self.link.statuses[c]))
                    for c in self.channels) + "  crc_fail=%d" % self.link.crc_fail)

        self.check("链路无掉线", self.link.link_drop_count == 0, "drop=%d" % self.link.link_drop_count)
        self.check("H743侧CRC错误率<1%",
                   self.link.crc_fail <= max(1, self.link.rx_frames // 100),
                   "crc_fail=%d / %d" % (self.link.crc_fail, self.link.rx_frames))
        n = max(1, samples[0])
        for c in self.channels:
            self.check("ESC%d 遥测来源无丢失(<2%%时间)" % (c + 1), dropout[c] * 50 < n,
                       "%.2f%%" % (dropout[c] * 100.0 / n))
            self.check("ESC%d 全程在转(停转占比<2%%)" % (c + 1), zero_cnt[c] * 50 < n,
                       "%.2f%%" % (zero_cnt[c] * 100.0 / n))
        self.safe_stop()


# ----------------------------- 链路诊断 -----------------------------
def diag(seconds=3):
    """不依赖任何协议逻辑，直接看 H743 的 UART RX 上到底收到了什么字节"""
    u = UART(UART_ID, UART_BAUD, read_buf_len=256)
    total = 0
    head = b""
    rx = FrameReceiver(SYNC_STATUS, STATUS_FRAME_LEN)
    good = 0
    ids = {}
    t_end = time.ticks_add(time.ticks_ms(), int(seconds * 1000))
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        n = u.any()
        if n:
            d = u.read(n)
            total += len(d)
            if len(head) < 52:
                head += d[:52 - len(head)]
            for f in rx.feed(d):
                good += 1
                ids[f[1]] = ids.get(f[1], 0) + 1
        time.sleep_ms(2)
    print("UART%d @%d, %.1fs: 收到字节=%d, 合法状态帧=%d, CRC失败=%d, 帧计数(按device_id)=%s" %
          (UART_ID, UART_BAUD, seconds, total, good, rx.crc_fail, ids))
    if head:
        print("前%d字节:" % len(head), " ".join("%02x" % b for b in head))
        print("(正常应看到 5a 00/01/02/03 开头、13字节一组循环)")
    if total == 0:
        print("-> RX上完全没有字节：检查 Pico 是否在运行main.py(看Pico串口输出)、")
        print("   Pico GP16(TX)->H743 RX 是否接对、UART_ID 对应的引脚、两板共地。")
    elif good == 0:
        print("-> 有字节但没有合法状态帧：多半是噪声(RX悬空/没共地)、波特率不一致，或TX/RX接反。")
    return good


def link_precheck(link, timeout_ms=1500):
    """H743 能否收到至少一帧合法状态帧。收不到就别做后面的测试了。"""
    t_end = time.ticks_add(time.ticks_ms(), timeout_ms)
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        link.poll_status()
        if link.rx_frames > 0:
            return True
        time.sleep_ms(2)
    print("\n!! %dms 内没有收到任何合法的Pico状态帧(rx_frames=0, crc_fail=%d, 丢弃字节=%d)" %
          (timeout_ms, link.crc_fail, link.dropped_bytes))
    print("!! 后面的测试没有意义，已中止。请先运行 Driver.diag() 查看原始字节。")
    return False


def quick(ch=0, thr=120, seconds=6):
    """单路手动试转：解锁 -> 缓升到 thr -> 每0.5s打印状态 -> 停机。
    用它先确认"电机到底转没转"，再看遥测。请拆桨。"""
    link = UartLink()
    t = EscTester(link, [ch])
    if not link_precheck(link):
        return
    if not t.need_props_removed():
        return
    thr = clamp_thr(thr)
    try:
        t.arm()
        steps = 8
        for k in range(1, steps + 1):          # 缓升，约2.4秒
            link.set_throttle(ch, thr * k // steps if thr * k // steps >= 48 else 48)
            t.pump(300)
        print("到达 thr=%d，保持 %ds，观察电机是否转动、遥测是否有数" % (thr, seconds))
        end = time.ticks_add(time.ticks_ms(), seconds * 1000)
        while time.ticks_diff(end, time.ticks_ms()) > 0:
            t.pump(500)
            s = link.statuses[ch]
            print("  en=%d bd=%d kiss=%d erpm=%d V=%.2f I=%.2f T=%d" %
                  (s.enabled, s.bidir_valid, s.kiss_valid, s.raw_erpm, s.voltage, s.current, s.temperature))
    except KeyboardInterrupt:
        print("用户中断")
    finally:
        t.safe_stop()


# ----------------------------- 入口 -----------------------------
TESTS = (
    ("link", "T1 通讯链路", "t_link"),
    ("enable", "T2 enable/disable", "t_enable"),
    ("robust", "T3 抗干扰", "t_robust"),
    ("latency", "T4 往返延迟", "t_latency"),
    ("ramp", "T5 油门阶梯+遥测(电机)", "t_ramp"),
    ("indep", "T6 通道独立性(电机)", "t_indep"),
    ("failsafe", "T7 超时保护(电机)", "t_failsafe"),
    ("degrade", "T8 单路降级(电机,手动)", "t_degrade"),
    ("soak", "T9 长时间稳定性(电机)", "t_soak"),
)


def run(names=None, channels=None):
    """names=None 跑全部；或传 key 列表，如 ['link','enable']"""
    link = UartLink()
    t = EscTester(link, channels)
    print("测试通道(0基): %s   UART%d @ %d" % (t.channels, UART_ID, UART_BAUD))
    try:
        if not link_precheck(link):
            return t
        for key, desc, fn in TESTS:
            if names is not None and key not in names:
                continue
            getattr(t, fn)()
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        t.safe_stop()
        t.summary()
    return t


def run_menu():
    link = UartLink()
    t = EscTester(link)
    if not link_precheck(link):
        return
    try:
        while True:
            print("\n===== Pico 联调测试菜单 =====")
            for i, (key, desc, fn) in enumerate(TESTS):
                print("  %d) %-9s %s" % (i + 1, key, desc))
            print("  a) 全部   m) 实时监视   q) 退出并汇总")
            ans = input("选择: ").strip().lower()
            if ans == "q":
                break
            elif ans == "a":
                for key, desc, fn in TESTS:
                    getattr(t, fn)()
            elif ans == "m":
                monitor(link, 15)
            elif ans.isdigit() and 1 <= int(ans) <= len(TESTS):
                getattr(t, TESTS[int(ans) - 1][2])()
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        t.safe_stop()
        t.summary()


def monitor(link=None, seconds=15, interval_ms=500):
    """纯监视：不发油门，只持续保活(enabled由当前commands决定)并打印4路状态"""
    own = link is None
    if own:
        link = UartLink()
    t = EscTester(link)
    t_end = time.ticks_add(time.ticks_ms(), seconds * 1000)
    last = 0
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        t.pump(interval_ms)
        print("link=%s crc_fail=%d" % (link.link_ok, link.crc_fail))
        for c in range(MAX_ESC):
            s = link.statuses[c]
            print("  ESC%d en=%d src=%-6s erpm=%8d T=%3d V=%5.2f I=%5.2f" %
                  (c + 1, s.enabled, EscTester.src(s), s.raw_erpm, s.temperature, s.voltage, s.current))


if __name__ == "__main__":
    run_menu()