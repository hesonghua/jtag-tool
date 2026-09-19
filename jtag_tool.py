#!/usr/bin/env python3
"""jtag_tool.py — EBAZ4205 jtag_swd_dbg IP 的独立 JTAG 调试工具（不依赖 OpenOCD）。

可直接运行（CLI，用法见文末），也可 `import jtag_tool` 当作库引用——
公共接口全部列在 `__all__`，各类内部方法以 `_` 前缀标记，勿依赖。

层次结构（JTAG 层架构无关，ARM 层独立于其上，可按目标替换）：

    JtagMaster   IP 寄存器访问 / 原始移位 / TAP 状态机 / 链扫描 / 位级引脚
    Tap          链上单个 TAP 的 IR/DR 访问，其余 TAP 自动置 BYPASS
    Dap          ADIv5 JTAG-DP 协议（DPACC/APACC）—— ARM 专属层
    MemAp        MEM-AP 内存读写 8/16/32 位 + LPAE 64 位地址窗口 —— ARM 专属层
    CortexM      内核运行控制 halt/resume/step + 寄存器读写 —— ARM 专属层
    CoreSight    ROM 表遍历与组件发现（dapinfo）—— ARM 专属层
    ScanDump     Arise2 私有 TAP 人格（scan_* 命令组，对齐 OpenOCD 命名）

二次开发示例：

    with JtagMaster(speed_khz=1000) as jtag:
        for v in jtag.scan_idcodes():
            print(decode_idcode(v))

        tap = Tap(jtag, irlen=4, ir_after=5, dr_after=1)  # STM32F1 链
        dap = Dap(tap)
        dap.powerup()
        mem = MemAp(dap)
        mem.write32(0x20000000, 0xDEADBEEF)
        print(f"{mem.read32(0x20000000):08x}")

用法（桥板上 root 运行，命令与输出格式同 openocd）：

    python3 jtag_tool.py                    # ★ 交互 shell（无参数进入，stc12-jtag 风格）
    python3 jtag_tool.py -c probe -c "mdw 0x20000000 4"    # 脚本方式执行后退出

    python3 jtag_tool.py idcode             # 一次性子命令（兼容旧 CLI）
    python3 jtag_tool.py probe
    python3 jtag_tool.py aps
    python3 jtag_tool.py dapinfo [0]
    python3 jtag_tool.py mdw 0x20000000 32
    python3 jtag_tool.py mww 0x20000000 0xAA 0xBB
    python3 jtag_tool.py halt / step / resume / reg [pc]

shell 会话示例（一次 open，DAP 状态保持，命令间无需重复 probe）：
    jtag> probe
    jtag> ap 4
    jtag> mdw 0x80000000 4
    jtag> halt
    jtag> reg pc
    jtag> resume
    jtag> quit
"""

import argparse
import ctypes
import mmap
import os
import struct
import sys
import time


class JtagError(Exception):
    """JTAG/DAP 各层统一的异常。"""


__version__ = "1.1"

# 模块公共接口：from jtag_tool import * 只导出这些；
# 带 _ 前缀的方法（_scan/_acc/_select 等）是各类的内部实现，勿依赖。
__all__ = [
    "JtagError",
    "JtagMaster", "Tap", "Dap", "MemAp", "CortexM", "CoreSight", "ScanDump",
    "decode_idcode", "dump", "arm_context",
    "MFG", "KNOWN_IDCODES",
    "__version__",
]


# ---------------------------------------------------------------------------
# 公共辅助（无状态，模块用户可直接用）
# ---------------------------------------------------------------------------

MFG = {0x049: "Xilinx", 0x23B: "ARM Ltd", 0x020: "STMicroelectronics",
       0x015: "NXP", 0x017: "Texas Instruments"}

KNOWN_IDCODES = {
    0x03722093: "Xilinx Zynq-7010 PL TAP",
    0x13722093: "Xilinx Zynq-7010 PL TAP",
    0x03727093: "Xilinx Zynq-7020 PL TAP",
    0x4BA00477: "ARM JTAG-DP r2p0 (Cortex-A9 DAP / STM32F4 等)",
    0x3BA00477: "ARM Cortex-M3 SWJ-DP (JTAG 模式)",
    0x1BA00477: "ARM Cortex-M3 SWJ-DP (SWD 模式)",
    0x16410041: "STM32F1x boundary scan TAP",
}


def decode_idcode(v):
    """IDCODE 整数 → 可读描述字符串（厂商/部件/版本）。"""
    if v & 1 == 0:
        return f"0x{v:08X}  (非 IDCODE：全 0 填充或旁路位)"
    mfg = (v >> 1) & 0x7FF
    return (f"0x{v:08X}  ver={(v >> 28):#x} part=0x{(v >> 12) & 0xFFFF:04X} "
            f"mfg=0x{mfg:03X}({MFG.get(mfg, '?')})  {KNOWN_IDCODES.get(v, '')}")


def dump(addr, size, vals):
    """按 openocd 风格打印内存值：word/halfword 8 个一行，byte 16 个一行。"""
    per_line = 16 if size == 1 else 8
    for i in range(0, len(vals), per_line):
        row = " ".join(f"{v:0{size * 2}x}" for v in vals[i:i + per_line])
        print(f"0x{addr + size * i:08x}: {row}")


# ---------------------------------------------------------------------------
# 第一层：IP 驱动（架构无关）
# ---------------------------------------------------------------------------

class JtagMaster:
    """jtag_swd_dbg IP 底层驱动。

    只做三件事：mmap 寄存器、控制 TCK 速度、提供原始移位原语 shift()。
    IP 寄存器语义见 jtag_swd_dbg.v 文件头注释。
    """

    PAGE_SIZE = 4096
    REG_MAGIC, REG_VERSION, REG_CTRL, REG_STATUS = 0x00, 0x04, 0x08, 0x0C
    REG_CLKDIV, REG_BITCNT, REG_PIN_DIR, REG_PIN_OUT = 0x10, 0x14, 0x18, 0x1C
    REG_PIN_IN = 0x20
    REG_TX_FIFO, REG_RX_FIFO = 0x24, 0x28
    REG_SWO_CTRL, REG_SWO_DIV, REG_SWO_STAT, REG_SWO_FIFO = 0x30, 0x34, 0x38, 0x3C
    SWO_CTRL_EN, SWO_CTRL_CLRERR = 1 << 0, 1 << 1
    SWO_STAT_OVR, SWO_STAT_FE = 1 << 16, 1 << 17
    PIN_OUT_SRST = 1 << 3          # 低有效（srst_n）

    CTRL_START, CTRL_ABORT, CTRL_CAPTURE = 1 << 0, 1 << 1, 1 << 4
    ST_BUSY, ST_RX_READY, ST_TX_FULL = 1 << 0, 1 << 3, 1 << 4
    MAGIC_VAL = 0x4A534447  # "JSDG"

    def __init__(self, dev_file="/dev/mem", base=0x43C00000, speed_khz=1000,
                 axi_hz=None):
        self.fd = os.open(dev_file, os.O_RDWR | os.O_SYNC)
        self.mm = mmap.mmap(self.fd, self.PAGE_SIZE, mmap.MAP_SHARED,
                            prot=mmap.PROT_READ | mmap.PROT_WRITE, offset=base)
        magic = self.rd(self.REG_MAGIC)
        if magic != self.MAGIC_VAL:
            raise JtagError(f"IP MAGIC 不匹配: 0x{magic:08X}")
        ver = self.rd(self.REG_VERSION)
        # BITCNT 寄存器 10 位（RTL bitcnt_reg[9:0]）→ 单次 shift 上限 1023 位，
        # FIFO 深度反而不是瓶颈；VERSION 里的 fifo log2 只作参考
        self.max_bits = min((1 << (ver & 0xFF)) * 16, 1023)
        # max_tck = floor(AXI/2) 有截断（本板 FCLK0=125M 只报 62 → 124M，
        # 差 0.8%）。SWO 分频对真实 AXI 敏感，调用方可传 axi_hz 覆盖。
        self.axi_hz = axi_hz or ((ver >> 8) & 0xFF) * 2_000_000
        self.has_swo = (ver >> 16) >= 2
        self.swo_cdr = (ver >> 16) >= 3   # SWO_DIV = 位周期-1（边沿 CDR）
        self.set_speed(speed_khz)
        self._pin_out = 0x8
        self.wr(self.REG_PIN_OUT, self._pin_out)
        self.wr(self.REG_PIN_DIR, 0x1F)
        # 每次打开都完整复位：上个进程异常退出会在 TX/RX FIFO 留残留数据
        # 毒化后续事务，目标 TAP 也可能停在半移位状态。abort 清引擎 +
        # goto_tlr 拉回目标，一步到位。
        self.reset_engine()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        if self.mm:
            self.mm.close()
            self.mm = None
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def rd(self, off):
        self.mm.seek(off)
        return struct.unpack("<I", self.mm.read(4))[0]

    def wr(self, off, val):
        self.mm.seek(off)
        self.mm.write(struct.pack("<I", val & 0xFFFFFFFF))

    def set_speed(self, speed_khz):
        """TCK = AXI / (2*(clkdiv+1))。"""
        self.clkdiv = max(0, round(self.axi_hz / (2 * speed_khz * 1000)) - 1)
        self.wr(self.REG_CLKDIV, self.clkdiv)

    def reset_engine(self):
        """完整复位（引擎 + 目标侧）：
        1. CTRL.abort —— 硬件清 TX/RX FIFO、移位 FSM 回 idle、TCK 停低
        2. goto_tlr —— 补 6 拍 TMS=1 把目标 TAP 拉回 Test-Logic-Reset，
           清掉异常时可能装了一半的 IR（abort 管不到目标 TAP 状态）
        3. goto_tlr 的 epoch 递增顺带失效软件侧 Tap IR 缓存
        调用前 PIN_DIR/PIN_OUT 应已配置（__init__ 里在引脚设置之后调用）。"""
        self.wr(self.REG_CTRL, self.CTRL_CAPTURE | self.CTRL_ABORT)
        self.wr(self.REG_CTRL, self.CTRL_CAPTURE)
        self.goto_tlr()

    def wait_idle(self, timeout_s=1.0):
        deadline = time.monotonic() + timeout_s
        while self.rd(self.REG_STATUS) & self.ST_BUSY:
            if time.monotonic() > deadline:
                raise JtagError("移位引擎忙超时")

    def shift(self, tms, tdi):
        """核心原语：逐 bit 驱动 TMS/TDI，返回捕获的 TDO 位列表（LSB 在前）。

        TX word 打包：[15:0]=TDI 位，[31:16]=TMS 位，bit0 = 最先移出的位；
        RX word 同序。一次最多移 max_bits 位。
        """
        n = len(tms)
        if n == 0 or n > self.max_bits:
            raise JtagError(f"移位长度 {n} 超范围 (1..{self.max_bits})")
        try:
            self.wait_idle()
            self.wr(self.REG_CTRL, self.CTRL_CAPTURE)
            self.wr(self.REG_BITCNT, n)
            for w in range((n + 15) // 16):
                low = high = 0
                for b in range(16):
                    i = w * 16 + b
                    if i >= n:
                        break
                    if tdi[i]:
                        low |= 1 << b
                    if tms[i]:
                        high |= 1 << b
                t_deadline = time.monotonic() + 1.0
                while self.rd(self.REG_STATUS) & self.ST_TX_FULL:
                    if time.monotonic() > t_deadline:
                        raise JtagError("TX FIFO 满超时（引擎未消费，状态异常）")
                self.wr(self.REG_TX_FIFO, (high << 16) | low)
            self.wr(self.REG_CTRL, self.CTRL_CAPTURE | self.CTRL_START)
            self.wait_idle()
            cap = []
            deadline = time.monotonic() + 1.0
            for _ in range((n + 15) // 16):
                while not self.rd(self.REG_STATUS) & self.ST_RX_READY:
                    if time.monotonic() > deadline:
                        raise JtagError("RX FIFO 超时")
                v = self.rd(self.REG_RX_FIFO)
                for b in range(16):
                    if len(cap) < n:
                        cap.append(1 if v & (1 << b) else 0)
            return cap
        except JtagError:
            # 异常路径必须清 FIFO：残留数据会毒化后续所有事务
            self.reset_engine()
            raise

    # ---- TAP 状态机辅助（假设稳态为 Run-Test/Idle）----

    def goto_tlr(self):
        self.shift([1] * 5, [0] * 5)  # 任意状态 5 个 TMS=1 必到 TLR
        self.shift([0], [0])          # TLR → Run-Test/Idle
        self._tlr_count = getattr(self, "_tlr_count", 0) + 1

    def idle(self, n=1):
        """在 Run-Test/Idle 停 n 个 TCK。"""
        self.shift([0] * n, [0] * n)

    def scan_idcodes(self, max_taps=2):
        """TLR 后所有 TAP 装载 IDCODE，扫 32*max_taps 位。

        返回各 TAP 的 IDCODE 整数列表（离 TDO 由近到远）。
        无 IDCODE 的 TAP 对应位串最低位为 0（BYPASS 特性），可用于判断链长。
        """
        n = 32 * max_taps
        tms = [1] * 5 + [0, 1, 0, 0] + [0] * (n - 1) + [1] + [1] * 5
        cap = self.shift(tms, [0] * len(tms))
        d = cap[9:9 + n]
        return [sum(d[32 * i + j] << j for j in range(32))
                for i in range(max_taps)]

    # ---- 位/引脚级（自 stc12-jtag jtag.c 移植）----

    def tdo_pin(self):
        """不打时钟直接读 TDO 引脚电平（PIN_IN bit1）。"""
        return (self.rd(self.REG_PIN_IN) >> 1) & 1

    # ---- SWO 接收器（IP VERSION >= 2；SWD 模式下 TDO 引脚 = TRACESWO）----

    def swo_div_for(self, baud):
        """baud → SWO_DIV 值。
        v3（边沿 CDR）：一个位 = SWO_DIV+1 个 AXI 周期，任意整数档。
        v2（16x 过采样）：tick = AXI/(div+1)，16 tick/bit，
        相邻档位间隔 AXI/16/k(k+1)，选完要回读 actual 评估失配。"""
        if self.swo_cdr:
            k = (self.axi_hz + baud // 2) // baud   # 位周期拍数
            if k < 2:
                raise JtagError(f"SWO {baud} Hz 超上限 {self.axi_hz // 2} Hz")
            return k - 1
        k = (self.axi_hz + 8 * baud) // (16 * baud)
        if k < 1:
            raise JtagError(f"SWO {baud} Hz 超上限 {self.axi_hz // 16} Hz")
        return k - 1

    def swo_enable(self, baud):
        """配分频并使能（0→1 沿 flush FIFO + 清 sticky），返回实际 RX 波特率。"""
        if not self.has_swo:
            raise JtagError("bitstream 无 SWO 接收器（IP VERSION < 2），需重刷")
        div = self.swo_div_for(baud)
        self.wr(self.REG_SWO_CTRL, 0)
        self.wr(self.REG_SWO_DIV, div)
        self.wr(self.REG_SWO_CTRL, self.SWO_CTRL_EN)
        return self.axi_hz // ((16 if not self.swo_cdr else 1) * (div + 1))

    def swo_disable(self):
        self.wr(self.REG_SWO_CTRL, 0)

    def swo_stat(self):
        """返回 (可读字节数, overrun, frame_err)。"""
        v = self.rd(self.REG_SWO_STAT)
        return v & 0xFFFF, bool(v & self.SWO_STAT_OVR), bool(v & self.SWO_STAT_FE)

    def swo_clear_err(self):
        self.wr(self.REG_SWO_CTRL, self.SWO_CTRL_EN | self.SWO_CTRL_CLRERR)

    def swo_read(self, n):
        """读最多 n 字节（n=None 全读）。整字弹出：剩余空间不足一次 pop
        的弹出量时提前停，字节留在 FIFO 不丢。"""
        out = bytearray()
        if n is None:
            n = 1 << 20
        while len(out) < n:
            cnt = self.rd(self.REG_SWO_STAT) & 0xFFFF
            if cnt == 0:
                break
            pop = min(4, cnt)
            if pop > n - len(out):
                break
            out += self.rd(self.REG_SWO_FIFO).to_bytes(4, "little")[:pop]
        return bytes(out)

    def srst(self, released):
        """SRST（低有效）：True=释放（引脚高），False=拉低复位。"""
        if released:
            self._pin_out |= self.PIN_OUT_SRST
        else:
            self._pin_out &= ~self.PIN_OUT_SRST & 0xFFFFFFFF
        self.wr(self.REG_PIN_OUT, self._pin_out)

    def _shift_dr_raw(self, n, tdi_bits):
        """RTI→ShiftDR→移 n 位（TDI=tdi_bits，末位 TMS=1）→Update→RTI。
        返回 n 位捕获（stc12 jtag_goto_shift_dr + jtag_shift_dr_read）。"""
        cap = self.shift([1, 0, 0] + [0] * (n - 1) + [1, 1, 0],
                         [0, 0, 0] + list(tdi_bits) + [0, 0])
        return cap[3:3 + n]

    def scan_chain(self, scan_bits=512):
        """通用链扫描（stc12 cmd_scan）：TLR 后 ShiftDR 读长位流逐位解析——
        bit=1 → 32 位 IDCODE 器件；bit=0 → 1 位 BYPASS 器件。
        返回 (idcode 列表, bypass 数)。"""
        self.goto_tlr()          # stc12 原版第一步；没有它 TAP 不在 RTI，扫出全 0
        d = self._shift_dr_raw(scan_bits, [0] * scan_bits)
        ids, bypass, pos = [], 0, 0
        while pos + 32 <= scan_bits and len(ids) + bypass < 16:
            if d[pos]:
                v = sum(d[pos + k] << k for k in range(32))
                if v == 0xFFFFFFFF or v == 0:   # 全 1 填充 → 链结束
                    pos += 1
                    if pos > 64:
                        break
                    continue
                ids.append(v)
                pos += 32
            else:
                bypass += 1
                pos += 1
        return ids, bypass

    def chain_probe(self):
        """链拓扑探测（stc12 cmd_chain），不依赖架构假设：
        1) ShiftIR 灌全 1 → 所有 TAP 进 BYPASS
        2) ShiftDR 冲 0 注单 1，数冒出延迟 = BYPASS 位宽（= TAP 数）
        3) TLR 后 ShiftIR 移出 48 位，IR capture pattern 的孤立 1 间隔
           = 各 TAP IR 长度；连续 1 起点 = TDI 回显 = 全链 IR 总宽
        """
        print("chain probe:")
        # 1) 全员 BYPASS：TLR → ShiftIR(4) + 64×1 → Update → RTI
        self.goto_tlr()
        self.shift([1, 1, 0, 0] + [0] * 63 + [1, 1, 0],
                   [0, 0, 0, 0] + [1] * 64 + [0, 0])
        # 2) BYPASS 位宽：ShiftDR 冲 64 个 0、注入单 1、数延迟
        cap = self.shift([1, 0, 0] + [0] * 64 + [0] +
                         [0] * 199 + [1, 1, 0],
                         [0, 0, 0] + [0] * 64 + [1] + [0] * 200 + [0, 0])
        rd = cap[3 + 64 + 1:]
        delay = next((i + 1 for i, b in enumerate(rd[:200]) if b), None)
        if delay is None:
            print("  no response: chain broken or TDO undriven")
            return
        print(f"  bypass width: {delay} bit ({delay} TAPs)")
        # 3) IR capture：TLR → ShiftIR + 48 位（TDI=1）→ Update → RTI
        self.goto_tlr()
        cap = self.shift([1, 1, 0, 0] + [0] * 47 + [1, 1, 0],
                         [0, 0, 0, 0] + [1] * 48 + [0, 0])
        d = cap[4:4 + 48]
        print("  IR capture   : " + " ".join(
            f"{sum(d[i * 8 + j] << j for j in range(8) if i * 8 + j < 48):02X}"
            for i in range(6)) + " (LSB first)")
        pos1, echo = [], 48
        for i in range(44):
            if not d[i]:
                continue
            if i + 1 < 48 and d[i + 1]:
                echo = i              # 连续 1 起点 = TDI 回显区
                break
            pos1.append(i)            # 孤立 1 = TAP 的 IR LSB 边界
        if not pos1:
            print("  IR scan: no capture pattern (non-std TAP?)")
            return
        for i, p in enumerate(pos1):
            ln = (pos1[i + 1] if i + 1 < len(pos1) else echo) - p
            print(f"  TAP{i}: irlen={ln}{'' if i else ' (nearest TDO)'}")
        print(f"  total: {len(pos1)} TAPs, IR width = {echo} bit")


# ---------------------------------------------------------------------------
# 第二层：TAP 访问（架构无关）
# ---------------------------------------------------------------------------

class Tap:
    """链上单个 TAP 的 IR/DR 访问。

    链拓扑参数（其它 TAP 一律置于 BYPASS）：
      ir_after / dr_after : 本 TAP 靠 TDI 一侧其余 TAP 的 IR 总长 / DR 旁路位数
      ir_before / dr_before: 本 TAP 靠 TDO 一侧同上

    位移位顺序：TDO 侧旁路位在前，本 TAP 的位居中，TDI 侧旁路位在后。
    例：STM32F1（TDI → bs TAP → SWJ-DP → TDO）中的 SWJ-DP：
        Tap(jtag, irlen=4, ir_after=5, dr_after=1)
    """

    _DR_PREFIX = [1, 0, 0]      # RTI → SelDR → CapDR → ShiftDR
    _IR_PREFIX = [1, 1, 0, 0]   # RTI → SelDR → SelIR → CapIR → ShiftIR
    _TAIL = [1, 0]              # 末位 TMS=1 退出 Shift → Update → RTI

    def __init__(self, jtag, irlen, ir_before=0, ir_after=0,
                 dr_before=0, dr_after=0):
        self.jtag = jtag
        self.irlen = irlen
        self.ir_before, self.ir_after = ir_before, ir_after
        self.dr_before, self.dr_after = dr_before, dr_after
        self._cur_ir = None
        self._ir_epoch = None

    def _scan(self, prefix, bits):
        n = len(bits)
        tms = prefix + [0] * (n - 1) + [1] + self._TAIL
        tdi = [0] * len(prefix) + bits + [0] * len(self._TAIL)
        cap = self.jtag.shift(tms, tdi)
        return cap[len(prefix):len(prefix) + n]

    def ir(self, value):
        """加载 IR（带缓存；TLR 后自动失效）。"""
        epoch = getattr(self.jtag, "_tlr_count", 0)
        if self._cur_ir == value and self._ir_epoch == epoch:
            return
        bits = [1] * self.ir_before
        bits += [(value >> i) & 1 for i in range(self.irlen)]
        bits += [1] * self.ir_after          # 全 1 = 任何 IR 长度的 BYPASS
        self._scan(self._IR_PREFIX, bits)
        self._cur_ir, self._ir_epoch = value, epoch

    def dr(self, bits):
        """移位本 TAP 的 DR（bits 为 TDI 位列表，LSB 先），返回捕获的 TDO 位。"""
        stream = [0] * self.dr_before + list(bits) + [0] * self.dr_after
        return self._scan(self._DR_PREFIX, stream)[self.dr_before:
                                                   self.dr_before + len(bits)]


# ---------------------------------------------------------------------------
# 第三层：ADIv5 JTAG-DP 协议（ARM 专属，只在目标为 ARM DAP 时使用）
# ---------------------------------------------------------------------------

class Dap:
    """通过某个 TAP 访问 ARM JTAG-DP（SWJ-DP）。

    协议要点（实测标定，与 OpenOCD adi_v5_jtag.c 一致，勿凭规范记忆改）：
      * DPACC/APACC 帧长 35 位：3 位 {A[3:2]<<1 | RnW} + 32 位数据，LSB 先移
      * ACK（先移出位当 LSB 组装）：OK/FAULT = 0b010 = 2，WAIT = 0b001 = 1
      * 读数据在"下一个事务"的响应里返回（posted read），配一次 RDBUFF 读收数
      * DPACC 读 DPIDR 在部分老 SWJ-DP（如 STM32F1）上恒返回 0，判活用
        CTRL/STAT 上电轮询
    """

    IR_IDCODE, IR_DPACC, IR_APACC, IR_ABORT = 0xE, 0xA, 0xB, 0x8
    DP_CTRL_STAT, DP_SELECT, DP_RDBUFF = 0x4, 0x8, 0xC
    ACK_OK, ACK_WAIT = 2, 1
    CSYSPWRUP_REQ_ACK = 0xA0000000  # CSYSPWRUP/CDBGPWRUP 的 REQ|ACK 全到位

    def __init__(self, tap):
        self.tap = tap
        self.trace = False
        self._sel = None  # SELECT 寄存器缓存（APSEL<<24 | APBANK<<4）

    @staticmethod
    def id_is_arm_dp(v):
        """ARM JTAG-DP 家族判据：IDCODE[27:12]=0xBA00、[11:1]=0x23B、bit0=1。
        版本号 3/4/6 都在用（0x3BA=M3/M4/M7 DPv1，0x4BA=DPv2，
        0x6BA=CM4+A 系 SoC 的 router DP 等）。"""
        return ((v >> 12) & 0xFFFF) == 0xBA00 and \
               ((v >> 1) & 0x7FF) == 0x23B and (v & 1) == 1

    @classmethod
    def probe(cls, jtag):
        """三种链拓扑逐一试探（stc12 cmd_probe 同款）：
          side=0  DP 靠近 TDO（旁路 TAP 在 TDI 侧）
          side=1  DP 靠近 TDI（旁路 TAP 在 TDO 侧）
          side=2  单 TAP 直连——多核 SoC 的 JTAG-DP 常为此拓扑
                  （如 Arise2：单 TAP irlen=4 = router DP）
        side=1 的读数有两种对齐可能（旁路回显位在前或在后），两种都试。
        """
        lens = (5, 4, 6, 3, 8)          # 旁路 TAP 的 IR 长度候选
        for side in range(3):
            for k in (range(1) if side == 2 else range(len(lens))):
                jtag.goto_tlr()
                if side == 0:
                    tap = Tap(jtag, 4, ir_after=lens[k], dr_after=1)
                elif side == 1:
                    tap = Tap(jtag, 4, ir_before=lens[k], dr_before=1)
                else:
                    tap = Tap(jtag, 4)   # 单 TAP：IR/DR 窗口直贴
                tap.ir(cls.IR_IDCODE)
                r = tap.dr([0] * 33)
                v = sum(r[i] << i for i in range(32))
                if side == 1 and not cls.id_is_arm_dp(v):
                    v = (v >> 1) | (r[32] << 31)   # 另一种对齐解释
                if cls.id_is_arm_dp(v):
                    desc = ("bypass@TDI", "bypass@TDO", "single TAP")[side]
                    print(f"JTAG-DP found: IDCODE=0x{v:08X} ({desc}, "
                          f"bs irlen={0 if side == 2 else lens[k]})")
                    return cls(tap)
        raise JtagError("链上未找到 ARM JTAG-DP")

    # ---- DP/AP 寄存器事务 ----

    def _acc_once(self, ap, reg, rnw, wdata=0):
        self.tap.ir(self.IR_APACC if ap else self.IR_DPACC)
        v3 = (((reg >> 2) & 3) << 1) | rnw
        bits = [(v3 >> i) & 1 for i in range(3)]
        bits += [(wdata >> i) & 1 for i in range(32)]
        r = self.tap.dr(bits)
        ack = r[0] | r[1] << 1 | r[2] << 2
        data = sum(r[3 + i] << i for i in range(32))
        if self.trace:
            print(f"  [{'AP' if ap else 'DP'}:{'R' if rnw else 'W'} 0x{reg:X}] "
                  f"ack={ack} data=0x{data:08X}")
        return ack, data

    def _acc(self, ap, reg, rnw, wdata=0):
        for _ in range(200):
            ack, data = self._acc_once(ap, reg, rnw, wdata)
            if ack == self.ACK_OK:
                return data
            if ack != self.ACK_WAIT:
                raise JtagError(f"非法 ACK {ack}")
        raise JtagError("DAP WAIT 超时")

    def dp_read(self, reg):
        self._acc(False, reg, 1)
        return self._acc(False, self.DP_RDBUFF, 1)

    def dp_write(self, reg, val):
        self._acc(False, reg, 0, val)

    def _select(self, apsel, bank=0):
        """DP.SELECT = APSEL<<24 | APBANK<<4，APACC 事务按它路由（带缓存）。"""
        sel = (apsel << 24) | (bank << 4)
        if self._sel != sel:
            self.dp_write(self.DP_SELECT, sel)
            self._sel = sel

    def ap_read(self, reg, apsel=0, bank=0):
        self._select(apsel, bank)
        self._acc(True, reg, 1)
        return self._acc(False, self.DP_RDBUFF, 1)

    def ap_write(self, reg, val, apsel=0, bank=0):
        self._select(apsel, bank)
        self._acc(True, reg, 0, val)

    MEM_AP_TYPES = {1: "AHB3-AP", 2: "APB-AP", 4: "AXI-AP", 5: "AHB5-AP",
                    6: "APB4-AP", 7: "AXI5-AP"}

    @classmethod
    def ap_type_name(cls, idr):
        """IDR 解码（布局同 OpenOCD arm_adi_v5.h）：
        DESIGNER[27:17]，CLASS[16:13]（0=JTAG-AP 1=COM-AP 8=MEM-AP），
        VARIANT[7:4]，TYPE[3:0]（MEM-AP 内：1=AHB3 2=APB 4=AXI...）。"""
        cls_ = (idr >> 13) & 0xF
        typ = idr & 0xF
        if cls_ == 8:
            return cls.MEM_AP_TYPES.get(typ, f"MEM-AP(type {typ})")
        return {0: "JTAG-AP", 1: "COM-AP"}.get(cls_, f"CLASS {cls_}")

    def scan_aps(self):
        """枚举 DAP 里的所有 AP：逐个 APSEL 读 IDR（bank 0xF 的 0xFC）。

        不存在的 AP 读回 0。返回 [(apsel, idr)]。
        """
        aps = []
        for apsel in range(256):
            idr = self.ap_read(0xFC, apsel=apsel, bank=0xF)
            if idr:
                aps.append((apsel, idr))
        return aps

    # ---- 常用流程 ----

    def abort(self, val=0x1F):
        """DAPABORT + 清全部 sticky 错误。DAP 死锁（持续 WAIT）时用，TLR 清不掉。"""
        self.tap.ir(self.IR_ABORT)
        v3 = 0  # 写事务，地址无意义
        bits = [(v3 >> i) & 1 for i in range(3)]
        bits += [(val >> i) & 1 for i in range(32)]
        self.tap.dr(bits)

    def powerup(self, timeout=100):
        """上电调试域。顺序 = v1.0 在本链路验证过的序列（不带前导 abort：
        F1 老 SWJ-DP 在本 IP 的批量移位下，前导 IR_ABORT 事务后首个 DPACC
        会回 ACK=0）；F2 写保留 sticky 清除能力，错误兜底走 check_sticky()。"""
        self._sel = None
        self.dp_write(self.DP_CTRL_STAT, 0x500000F2)  # 双 REQ + W1C 清 sticky
        self.dp_write(self.DP_CTRL_STAT, 0x50000042)
        cs = 0
        for _ in range(timeout):
            cs = self.dp_read(self.DP_CTRL_STAT)
            if cs & self.CSYSPWRUP_REQ_ACK == self.CSYSPWRUP_REQ_ACK:
                return cs
        raise JtagError(f"调试域上电超时 (CTRL/STAT=0x{cs:08X})")

    STICKY_ERR_MASK = 0xB0  # STICKYCMP|STICKYERR|WDATAERR（READOK bit6 不算错）

    def check_sticky(self):
        """事务后检查并自动清 sticky 错误（读非法地址会锁存，此后 AP 读恒 0）。"""
        try:
            cs = self.dp_read(self.DP_CTRL_STAT)
        except JtagError:
            return
        if cs & self.STICKY_ERR_MASK:
            print(f"(sticky err=0x{cs:08X}, auto-cleared; earlier reads may be 0)")
            self.abort()
            self.dp_write(self.DP_CTRL_STAT, 0x500000F2)
            self.dp_write(self.DP_CTRL_STAT, 0x50000042)


# ---------------------------------------------------------------------------
# 第四层：AHB-AP 内存访问（ARM 专属）
# ---------------------------------------------------------------------------

class MemAp:
    """经由 MEM-AP 读写目标内存（含 LPAE 64 位地址窗口，自 stc12 arm.c 移植）。

    CSW 基值恒用 0xA2000000（AHB 基值；OpenOCD 的 CSW_AXI_DEFAULT 从未用作
    默认，且 AXI-AP 加 ARPROT1 非安全位会被安全过滤器静默读零）。
    LPAE（CFG.LA=1 的 AP，如 Arise2 AP4 AXI-AP）：TAR64(bank0 0x08) 放地址
    [63:32]，'hi' 命令设定窗口。★TAR64 必须先于 TAR 写——本板 AP4 实测在
    TAR 写入瞬间锁存完整地址，先写 TAR 会带上旧高位，首次访问落错窗口。
    """

    AP_CSW, AP_TAR, AP_TAR64, AP_DRW = 0x00, 0x04, 0x08, 0x0C
    AP_BASE64, AP_CFG = 0xF0, 0xF4        # bank 0xF
    CFG_LA = 0x02                          # CFG bit1：Long Address
    CSW_BASE = 0xA2000000

    def __init__(self, dap, apsel=0):
        self.dap = dap
        self.apsel = apsel
        self._size = None
        self._la = None                    # None=未探测 CFG.LA
        self._tar_hi_cache = None          # 已写入 TAR64 的高 32 位

    def long_addr(self):
        """惰性探测当前 AP 的 CFG.LA（无 CFG 的旧 MEM-AP 读回 0 → 按 32 位降级）。"""
        if self._la is None:
            self._la = False               # 读失败也不重试，按 32 位降级
            try:
                cfg = self.dap.ap_read(self.AP_CFG, apsel=self.apsel, bank=0xF)
                self._la = bool(cfg & self.CFG_LA)
            except JtagError:
                pass
        return self._la

    def _set_size(self, code):
        if self._size != code:
            self.dap.ap_write(self.AP_CSW, self.CSW_BASE | code,
                              apsel=self.apsel)
            self._size = code

    def _set_tar(self, addr):
        """接受完整 64 位地址（Python int 直通；stc12 的 hi/lo 拆分在内部完成）。
        TAR64 高位变化才写（先于 TAR，含写回 0 的情况）；非 LPAE 的 AP 传
        >32 位地址直接报错。"""
        hi, lo = addr >> 32, addr & 0xFFFFFFFF
        if hi and not self.long_addr():
            raise JtagError(f"AP{self.apsel} 非 LPAE，地址超 32 位: 0x{addr:X}")
        if self.long_addr() and self._tar_hi_cache != hi:
            self.dap.ap_write(self.AP_TAR64, hi, apsel=self.apsel)
            self._tar_hi_cache = hi
        self.dap.ap_write(self.AP_TAR, lo, apsel=self.apsel)

    def read32(self, addr):
        self._set_size(2)
        self._set_tar(addr)
        return self.dap.ap_read(self.AP_DRW, apsel=self.apsel)

    def write32(self, addr, val):
        self._set_size(2)
        self._set_tar(addr)
        self.dap.ap_write(self.AP_DRW, val, apsel=self.apsel)

    def read16(self, addr):
        self._set_size(1)
        self._set_tar(addr)
        return (self.dap.ap_read(self.AP_DRW, apsel=self.apsel)
                >> ((addr & 2) * 8)) & 0xFFFF

    def write16(self, addr, val):
        self._set_size(1)
        self._set_tar(addr)
        self.dap.ap_write(self.AP_DRW, val << ((addr & 2) * 8),
                          apsel=self.apsel)

    def read8(self, addr):
        self._set_size(0)
        self._set_tar(addr)
        return (self.dap.ap_read(self.AP_DRW, apsel=self.apsel)
                >> ((addr & 3) * 8)) & 0xFF

    def write8(self, addr, val):
        self._set_size(0)
        self._set_tar(addr)
        self.dap.ap_write(self.AP_DRW, val << ((addr & 3) * 8),
                          apsel=self.apsel)

    def read(self, addr, count=1, size=4):
        fn = {4: self.read32, 2: self.read16, 1: self.read8}[size]
        return [fn(addr + size * i) for i in range(count)]

    def write(self, addr, vals, size=4):
        fn = {4: self.write32, 2: self.write16, 1: self.write8}[size]
        for i, v in enumerate(vals):
            fn(addr + size * i, v)


# ---------------------------------------------------------------------------
# 第五层：Cortex-M 运行控制（ARM 专属）
# ---------------------------------------------------------------------------

class CortexM:
    """Cortex-M 内核运行控制：halt/resume/step + 核内寄存器访问。

    经由 AHB-AP 访问 CPU 私有调试寄存器（SCS）：
      DHCSR  0xE000EDF0  调试控制/状态（写操作须带 DBGKEY）
      DCRSR  0xE000EDF4  寄存器选择（REGSEL | REGWnR<<16）
      DCRDR  0xE000EDF8  寄存器数据
    内存访问不会停机；要看稳定的寄存器快照先 halt()。
    """

    DHCSR, DCRSR, DCRDR = 0xE000EDF0, 0xE000EDF4, 0xE000EDF8
    DBGKEY = 0xA05F0000
    C_DEBUGEN, C_HALT, C_STEP = 1 << 0, 1 << 1, 1 << 2
    S_REGRDY, S_HALT = 1 << 16, 1 << 17
    S_SLEEP, S_LOCKUP = 1 << 18, 1 << 19

    # STM32 家族的 DBGMCU（RM0008：0xE0042004）。halt 冻不住独立时钟的
    # 看门狗，IWDG/WWDG 继续计数会把停机中的芯片复位掉（表现为 pc 自己变）。
    # 其它厂商的 Cortex-M 需按其手册调整。
    DBGMCU_CR = 0xE0042004
    DBG_IWDG_STOP, DBG_WWDG_STOP = 1 << 8, 1 << 9

    REGS = [(f"r{i}", i) for i in range(13)] + [
        ("sp", 13), ("lr", 14), ("pc", 15),                  # ARMv7-M DCRSR REGSEL
        ("xPSR", 0x10), ("MSP", 0x11), ("PSP", 0x12),
        ("PRIMASK", 0x14), ("BASEPRI", 0x15),
        ("FAULTMASK", 0x16), ("CONTROL", 0x17),
    ]

    def __init__(self, mem):
        self.mem = mem
        self.halted_for_read = False
        # 使能调试（C_DEBUGEN）。注意：写 DHCSR 必须带上当前的停机状态，
        # 否则对已 halt 的核，C_HALT=0 的写等于把它 resume 掉。
        dhcsr = self._dhcsr()
        self.mem.write32(self.DHCSR, self.DBGKEY | self.C_DEBUGEN |
                         (self.C_HALT if dhcsr & self.S_HALT else 0))
        # halt 期间冻结看门狗，防止停机中被 IWDG/WWDG 复位。
        # DBGMCU_CR 是 STM32 家族地址，非 ST 平台可能总线错——失败跳过。
        try:
            cr = mem.read32(self.DBGMCU_CR)
            mem.write32(self.DBGMCU_CR,
                        cr | self.DBG_IWDG_STOP | self.DBG_WWDG_STOP)
        except JtagError:
            pass

    CPUID = 0xE000ED00
    # CPUID[15:4]=part number（判 CM 的白名单），[31:24]=implementer 0x41=ARM
    PART_NAMES = {0xC20: "M0", 0xC21: "M1", 0xC23: "M3", 0xC24: "M4",
                  0xC27: "M7", 0xC60: "M0+", 0xD20: "M23", 0xD21: "M33",
                  0xD22: "M55"}

    @classmethod
    def detect(cls, mem):
        """读 CPUID 判定当前 AP 后是否 Cortex-M（非 CM 平台该地址未映射，
        读回 0/垃圾被 part 白名单排除）。返回 (是否 CM, cpuid)。"""
        try:
            cpuid = mem.read32(cls.CPUID)
        except JtagError:
            return False, 0
        ok = ((cpuid >> 24) & 0xFF) == 0x41 and \
             ((cpuid >> 4) & 0xFFF) in cls.PART_NAMES
        return ok, cpuid

    def _dhcsr(self):
        return self.mem.read32(self.DHCSR)

    def halted(self):
        return bool(self._dhcsr() & self.S_HALT)

    def halt(self):
        """停机并等待 S_HALT 置位，返回 DHCSR。"""
        self.mem.write32(self.DHCSR, self.DBGKEY | self.C_DEBUGEN | self.C_HALT)
        for _ in range(100):
            if self.halted():
                return self._dhcsr()
        raise JtagError("halt 超时")

    def resume(self):
        self.mem.write32(self.DHCSR, self.DBGKEY | self.C_DEBUGEN)

    def step(self):
        self.mem.write32(self.DHCSR, self.DBGKEY | self.C_DEBUGEN | self.C_STEP)

    def reg_read(self, sel, wake=True):
        """读单个核内寄存器，sel 为 DCRSR REGSEL 编码（REGS 表的第二列）。

        实测本核（M3 r1p1）运行中 DCRSR 传输不可靠：S_REGRDY 不反映新请求，
        DCRDR 里是上一次传输的过期数据。wake=True 时先 halt 再读（结果可信），
        并置 halted_for_read 提示调用方内核被停住了。
        """
        if wake and not self.halted():
            self.halt()
            self.halted_for_read = True
        self.mem.write32(self.DCRSR, sel)  # REGWnR=0，读
        for _ in range(100):
            if self._dhcsr() & self.S_REGRDY:
                return self.mem.read32(self.DCRDR)
        raise JtagError(f"寄存器读超时 (REGSEL=0x{sel:X})")

    def reg_write(self, sel, val):
        """写核内寄存器（stc12 cm_reg_write 同序：DCRDR=值 → DCRSR=sel|REGWnR）。"""
        self.mem.write32(self.DCRDR, val)
        self.mem.write32(self.DCRSR, sel | 0x10000)   # REGWnR=1 写
        for _ in range(100):
            if self._dhcsr() & self.S_REGRDY:
                return
        raise JtagError(f"寄存器写超时 (REGSEL=0x{sel:X})")

    def regs(self):
        """读全部核内寄存器，返回 {名字: 值}。"""
        return {name: self.reg_read(sel) for name, sel in self.REGS}


# ---------------------------------------------------------------------------
# 第六层：CoreSight 组件发现（ARM 专属，对应 openocd 的 dap info）
# ---------------------------------------------------------------------------

class CoreSight:
    """读 AP 的 BASE 找到 ROM 表，遍历发现 SoC 里的调试组件。

    组件 ID 寄存器（每个 4KB 组件页的尾部，字节宽）：
      PIDR0-3 @ 0xFE0-0xFEC，PIDR4-7 @ 0xFD0-0xFDC，CIDR0-3 @ 0xFF0-0xFFC
    拼接：pid = pidr0 | pidr1<<8 | ... | pidr4<<32；cid 同理 32 位。
    字段（公式同 openocd arm_coresight.h）：
      PART     = pid[11:0]
      DESIGNER = ((pid>>25)&0x780) | ((pid>>12)&0x7F)   # JEP106
      JEDEC    = pid[19]
      CLASS    = cid[15:12]，CID 合法值 = 0xB105000D
    ROM 表项：bit0=Present，bit1=Format，[31:12]=有符号偏移（4KB 单位，相对表基址）。
    """

    DEVTYPE = 0xFCC  # class 1 时是 MEMTYPE(bit0=系统总线内存)，class 9 时是 DEVTYPE
    CID_VALID_MASK = 0xB105000D

    CLASS_NAMES = {0x1: "ROM table", 0x9: "CoreSight component",
                   0xE: "Generic IP component", 0xF: "PrimeCell peripheral"}
    DEVTYPE_NAMES = {0x11: "Trace Sink, Port", 0x12: "Trace Sink, Buffer",
                     0x13: "Trace Link", 0x14: "Debug Control, Trigger",
                     0x15: "Debug Logic", 0x21: "Trace Source, Processor"}
    PART_NAMES = {
        (0x23B, 0x000): "Cortex-M3 SCS (System Control Space)",
        (0x23B, 0x001): "Cortex-M3 ITM (Instrumentation Trace)",
        (0x23B, 0x002): "Cortex-M3 DWT (Data Watchpoint and Trace)",
        (0x23B, 0x003): "Cortex-M3 FPB (Flash Patch and Breakpoint)",
        (0x23B, 0x912): "Cortex-M3 ETM (Embedded Trace)",
        (0x23B, 0x923): "Cortex-M3 TPIU (Trace Port Interface Unit)",
        (0x23B, 0x924): "Cortex-M4 TPIU (Trace Port Interface Unit)",
        (0x23B, 0x4A13): "Cortex-M4 SCS",
    }

    @staticmethod
    def read_ids(mem, base):
        pid = 0
        for i in range(4):
            pid |= (mem.read32(base + 0xFE0 + 4 * i) & 0xFF) << (8 * i)
        pid |= (mem.read32(base + 0xFD0) & 0xFF) << 32
        cid = 0
        for i in range(4):
            cid |= (mem.read32(base + 0xFF0 + 4 * i) & 0xFF) << (8 * i)
        return pid, cid

    @classmethod
    def describe(cls, mem, comp, depth):
        ind = "    " * depth
        print(f"{ind}Component base address 0x{comp:08X}")
        pid, cid = cls.read_ids(mem, comp)
        if (cid & 0xFFFF0FFF) != cls.CID_VALID_MASK:
            print(f"{ind}    Invalid CID 0x{cid:08X}")
            return
        part = pid & 0xFFF
        designer = ((pid >> 25) & 0x780) | ((pid >> 12) & 0x7F)
        print(f"{ind}    Peripheral ID 0x{pid:010X}")
        print(f"{ind}    Designer is 0x{designer:03X}, {MFG.get(designer, '?')}")
        print(f"{ind}    Part is 0x{part:03X}, "
              f"{cls.PART_NAMES.get((designer, part), '未识别')}")
        klass = (cid >> 12) & 0xF
        print(f"{ind}    Component class is 0x{klass:X}, "
              f"{cls.CLASS_NAMES.get(klass, '?')}")
        if klass == 0x1:  # ROM 表：继续往下遍历
            if mem.read32(comp + cls.DEVTYPE) & 1:
                print(f"{ind}    MEMTYPE system memory present on bus")
            cls.walk_rom_table(mem, comp, depth + 1)
        elif klass == 0x9:  # CoreSight：打印功能类型
            devtype = mem.read32(comp + cls.DEVTYPE) & 0xFF
            print(f"{ind}    Type is 0x{devtype:02X}, "
                  f"{cls.DEVTYPE_NAMES.get(devtype, '未知')}")

    @classmethod
    def walk_rom_table(cls, mem, base, depth):
        ind = "    " * depth
        off = 0
        while off <= 960:
            entry = mem.read32(base + off)
            print(f"{ind}ROMTABLE[0x{off:X}] = 0x{entry:08X}")
            off += 4
            if entry == 0:
                print(f"{ind}End of ROM table")
                return
            if not entry & 1:
                print(f"{ind}    Component not present")
                continue
            rel = entry >> 12
            if rel & 0x80000:  # 20 位有符号偏移
                rel -= 0x100000
            cls.describe(mem, base + (rel << 12), depth + 1)

    @classmethod
    def info(cls, dap, apsel=0):
        """对应 openocd 的 dap info：AP 头 + BASE + ROM 表组件发现。"""
        mem = MemAp(dap, apsel)
        idr = dap.ap_read(0xFC, apsel=apsel, bank=0xF)
        print(f"AP # 0x{apsel:X}")
        print(f"    AP ID register 0x{idr:08X}")
        print(f"    Type is {Dap.ap_type_name(idr)}")
        base = dap.ap_read(0xF8, apsel=apsel, bank=0xF)
        base_str = f"0x{base:08X}"
        if mem.long_addr():   # LPAE：补读 BASE64，打印完整 64 位基址
            try:
                hi = dap.ap_read(MemAp.AP_BASE64, apsel=apsel, bank=0xF)
                if hi:
                    base_str = f"0x{hi:08X}{base:08X}"
            except JtagError:
                pass
        print(f"MEM-AP BASE {base_str}")
        if not base & 1:
            print("    ROM table not present")
            return
        print("    Valid ROM table present")
        cls.describe(mem, base & ~0xFFF, 1)


# 名字查找表：大小写不敏感 + r13/r14/r15 别名（类体内推导式看不到类属性，放这里）
CortexM.REG_LOOKUP = {n.lower(): s for n, s in CortexM.REGS}
CortexM.REG_LOOKUP.update({"r13": 13, "r14": 14, "r15": 15})


# ---------------------------------------------------------------------------
# Arise2 scan dump（自 stc12-jtag scandump.c 移植；移位原语 = Zynq IP shift）
# ---------------------------------------------------------------------------

class ScanDump:
    """Arise2 私有 TAP 人格：芯片 hang 死后把 scan chain 的 DFF 逐拍移出，
    供后端按 chain 长度表逐列还原定位。命名对齐 OpenOCD scan_*。

    时序要点（与 stc12/OpenOCD 原版一致）：
      * IR/DR 一律 LSB first；每次访问独立走 进Shift→移位→Exit1→Update→RTI
      * CFG 读的 TDO 滞后一轮，连发两帧取第二帧
      * SCAN_O 的全部位必须一次 Capture-DR 内移完（每次 Capture 重新采样）
      * dump 序列 CFG0 → CFG2 → CFG1：CFG2 一写即 gate 功能时钟，
        模式字必须在 gate 前写好（反序实测读到全 0）
    """

    IR_LEN = 4
    INSTR_IDCODE, INSTR_CHAIN_SELECT = 0x2, 0x3
    INSTR_DEBUG, INSTR_BYPASS = 0x8, 0xF
    CHAIN_CFG, CHAIN_SCAN_IN, CHAIN_SCAN_OUT = 0, 1, 2
    CFG_MODE, CFG_TRIGGER, CFG_ENABLE = 0, 1, 2
    CFG0_AUTO_1CLK, CFG0_MANUAL, CFG0_BURST_SHIFT = 0x10, 0x20, 2
    CFG2_DUMP, CFG2_MIU_SR, CFG2_MIU_PADDET = 0x01, 0x02, 0x04
    CFG2_BIU_RST, CFG2_PERI_RST, CFG2_MIU_LOCK = 0x08, 0x10, 0x20
    IDCODE_EXPECTED = 0xAABBCCDD
    SCAN_OUT_BITS = 119           # Arise2 read_u119 现行版（E3K 为 169）
    MEMKEEP_STEPS = [
        (CFG2_MIU_SR, "miu_sr"), (CFG2_MIU_PADDET, "miu_paddet"),
        (CFG2_DUMP, "dump on"), (0x00, "dump off"),
        (CFG2_BIU_RST, "biu rst"), (0x00, "biu rst off"),
        (CFG2_MIU_LOCK, "miu lock"), (0x00, "miu unlock"),
        (CFG2_PERI_RST, "peri rst"),
    ]

    def __init__(self, jtag):
        self.jtag = jtag
        self.fmt_bit = False

    # ---- TAP 导航 / 移位（RTI 基准，一次 shift 完成）----

    def _dr(self, tdi_bits):
        """RTI→ShiftDR→移位→Exit1→Update→RTI，返回捕获位。"""
        n = len(tdi_bits)
        cap = self.jtag.shift([1, 0, 0] + [0] * (n - 1) + [1, 1, 0],
                              [0, 0, 0] + list(tdi_bits) + [0, 0])
        return cap[3:3 + n]

    def _ir_load(self, instr):
        """装私有 IR（4bit LSB first）。原版不做 IR busy 轮询，保持一致。
        顺带 bump TLR epoch 使 ARM 层的 Tap IR 缓存失效（IR 被外部改动）。"""
        bits = [(instr >> i) & 1 for i in range(self.IR_LEN)]
        self.jtag.shift([1, 1, 0, 0] + [0] * (self.IR_LEN - 1) + [1, 1, 0],
                        [0, 0, 0, 0] + bits + [0, 0])
        self.jtag._tlr_count = getattr(self.jtag, "_tlr_count", 0) + 1

    def _path_setup(self, chain):
        """选通 DR 通路：IR=CHAINSEL → DR=chain → IR=DEBUG。"""
        self._ir_load(self.INSTR_CHAIN_SELECT)
        self._dr([(chain >> i) & 1 for i in range(4)])
        self._ir_load(self.INSTR_DEBUG)

    # ---- CFG 寄存器（14bit 帧：data<<6 | RW<<5 | addr）----

    def _cfg_frame(self, rw, addr, data):
        return ((data & 0xFF) << 6) | ((rw & 1) << 5) | (addr & 0x1F)

    def cfg_write(self, reg, data):
        self._path_setup(self.CHAIN_CFG)
        frame = [(self._cfg_frame(1, reg, data) >> i) & 1 for i in range(14)]
        self._dr(frame)

    def cfg_read(self, reg):
        frame = [(self._cfg_frame(0, reg, 0) >> i) & 1 for i in range(14)]
        self._path_setup(self.CHAIN_CFG)
        self._dr(frame)              # 第一帧：上一轮残影
        raw = self._dr(frame)        # 第二帧：当前值
        return sum(raw[6 + i] << i for i in range(8))

    # ---- 数据通路 ----

    def scan_in_write(self, w0, w1):
        """写 REG_SCAN_I 64bit（w0=低 32 位；必须一次移完 64 拍）。"""
        self._path_setup(self.CHAIN_SCAN_IN)
        self._dr([(w0 >> i) & 1 for i in range(32)] +
                 [(w1 >> i) & 1 for i in range(32)])

    def scan_out_read(self):
        """读 REG_SCAN_O 全部位（一次 Capture-DR 内连移），返回位列表。"""
        return self._dr([0] * self.SCAN_OUT_BITS)

    # ---- 自检 / dump ----

    def read_idcode(self):
        self._ir_load(self.INSTR_IDCODE)
        r = self._dr([0] * 32)
        v = sum(r[i] << i for i in range(32))
        return v, v == self.IDCODE_EXPECTED

    def test_bypass(self):
        self._ir_load(self.INSTR_BYPASS)
        r = self._dr([(0xAAAA >> i) & 1 for i in range(16)])
        echo = sum(r[i] << i for i in range(16))
        # 1bit BYPASS 延迟一拍 = 右移一位：期望 0x5554
        return echo == 0x5554, echo

    def print_row(self, row, bits):
        print(f"{row:05d}:", end="")
        if self.fmt_bit:     # openocd xml 同款逐位
            print("".join(str(b) for b in bits))
        else:                # MSB 字节在前（同 stc12 逐字节十六进制）
            v = 0
            for b in reversed(bits):
                v = (v << 1) | b
            print(f"{v:0{(self.SCAN_OUT_BITS + 3) // 4}X}")

    def dump(self, rows, cfg2=CFG2_DUMP, burst=None):
        """burst=None → auto（每次 req 1 拍）；否则 manual burst_shift(0..3)。"""
        if burst is None:
            mode = self.CFG0_AUTO_1CLK
        else:
            mode = self.CFG0_MANUAL | (burst << self.CFG0_BURST_SHIFT)
        self.cfg_write(self.CFG_MODE, mode)
        self.cfg_write(self.CFG_ENABLE, cfg2)
        self.cfg_write(self.CFG_TRIGGER, 1)
        try:
            for row in range(rows):        # 每行重选通路（同原版）
                self._path_setup(self.CHAIN_SCAN_OUT)
                self.print_row(row, self.scan_out_read())
        finally:
            self.cfg_write(self.CFG_ENABLE, 0)   # Ctrl+C 中止也要停 dump

    def stop(self):
        self.cfg_write(self.CFG_ENABLE, 0)

    def mem_keep(self):
        """dump 后保持 SoC 内存（文档 §3.1 十步序列的固化）。"""
        for v, desc in self.MEMKEEP_STEPS:
            self.cfg_write(self.CFG_ENABLE, v)
            print(f"  {desc}")
        print("mem_keep done; memory retained (access via bus/PMP)")

    def test_verify(self):
        """SCAN_I 全 1 写入 + SCAN_O 读回自检（短链低位先出 0 属正常）。"""
        for _ in range(3):                 # 119bit ≈ 3×64bit 全 1
            self.scan_in_write(0xFFFFFFFF, 0xFFFFFFFF)
        self._path_setup(self.CHAIN_SCAN_OUT)
        self.print_row(0, self.scan_out_read())


# ---------------------------------------------------------------------------
# 交互 shell（stc12-jtag 串口 shell 风格；一次 open，命令间保持 DAP 状态）
# ---------------------------------------------------------------------------

class _SpiXfer(ctypes.Structure):
    """struct spi_ioc_transfer（6.12 uapi 布局：tx/rx_nbits 是 u8，还有
    word_delay_usecs+pad，共 32B——nbits 写成 u32 会导致 40%32!=0 被
    spidev 以 EINVAL 拒掉）+ 一次性构造。
    板上没编 python-spidev，用 ctypes + fcntl.ioctl 直发。"""
    _fields_ = [("tx_buf", ctypes.c_uint64), ("rx_buf", ctypes.c_uint64),
                ("len", ctypes.c_uint32), ("speed_hz", ctypes.c_uint32),
                ("delay_usecs", ctypes.c_uint16),
                ("bits_per_word", ctypes.c_uint8),
                ("cs_change", ctypes.c_uint8),
                ("tx_nbits", ctypes.c_uint8), ("rx_nbits", ctypes.c_uint8),
                ("word_delay_usecs", ctypes.c_uint8), ("pad", ctypes.c_uint8)]

    def __init__(self, tx, length, rx=None):
        """tx: 发送字节；rx: 可变 bytearray 或 None（不收）。"""
        super().__init__()
        self._tx = bytes(tx)                     # 持有缓冲生命周期
        self._txbuf = ctypes.create_string_buffer(self._tx, max(length, 1))
        self.tx_buf = ctypes.addressof(self._txbuf)
        if rx is not None:
            self._rxbuf = ctypes.create_string_buffer(length)
            self.rx_buf = ctypes.addressof(self._rxbuf)
            self._rx = rx                       # ioctl 后从 _rxbuf 拷回
        self.len = length
        self.speed_hz = 1000000
        self.bits_per_word = 8

    def xfer(self, fd):
        """一次 SPI_IOC_MESSAGE(1) 传输；rx 模式把数据拷回 self._rx。"""
        import fcntl
        cmd = 0x40000000 | (ctypes.sizeof(self) << 16) | 0x6B00
        fcntl.ioctl(fd, cmd, self)
        if self.rx_buf and self._rx is not None:
            self._rx[:] = self._rxbuf.raw[:len(self._rx)]


def parse_ints(args, lo, hi, usage):
    """参数 → int 列表（支持 0x..）；个数不在 [lo,hi] 视为用法错误。"""
    try:
        vals = [int(a, 0) for a in args]
    except ValueError:
        raise JtagError(f"非数字参数；用法: {usage}")
    if not lo <= len(vals) <= hi:
        raise JtagError(f"用法: {usage}")
    return vals


class Shell:
    """交互 shell：JtagMaster 只开一次，probe/DAP 上电/IR 缓存全程复用。

    ARM 相关命令懒探测——没 probe 过就自动做（省去时序约束）。
    运行控制命令首次使用时读 CPUID 确认 Cortex-M（stc12 cm_ready 门卫）；
    'ap n' 换域后自动重检。
    """

    def __init__(self, jtag, apsel=0):
        self.jtag = jtag
        self.apsel = apsel
        self.dap = None
        self.mem = None
        self.cm = None
        self.sd = None                 # ScanDump（scan_* 命令懒创建）
        self._cm_checked = False
        self._cm_ok = False

    # ---- 状态构造（懒）----

    def _dap(self):
        if self.dap is None:
            self.dap = Dap.probe(self.jtag)
            cs = self.dap.powerup()
            print(f"DAP OK: CTRL/STAT=0x{cs:08X}")
        return self.dap

    def _mem(self):
        if self.mem is None or self.mem.apsel != self.apsel:
            self.mem = MemAp(self._dap(), self.apsel)
            self.cm = None          # 换 AP 后 CortexM 上下文重建
        return self.mem

    def _cpu(self, cmd=""):
        mem = self._mem()
        if not self._cm_checked:    # 首次用运行控制命令才探测（核在哪个 AP 用户知道）
            self._cm_checked, self._cm_ok = True, False
            ok, cpuid = CortexM.detect(mem)
            if ok:
                part = CortexM.PART_NAMES.get((cpuid >> 4) & 0xFFF, "?")
                print(f"Cortex-{part} CPUID=0x{cpuid:08X} via AP{self.apsel}")
                self._cm_ok = True
            else:
                self.dap.check_sticky()   # 非 CM 平台读 0xE000ED00 可能挂 sticky
        if not self._cm_ok:
            raise JtagError(
                f"{cmd or 'cpu'}: AP{self.apsel} 后没有 Cortex-M 核"
                f"（'ap <n>' 换域后重试）")
        if self.cm is None:
            self.cm = CortexM(mem)
        return self.cm

    def _sd(self):
        if self.sd is None:
            self.sd = ScanDump(self.jtag)
        return self.sd

    # ---- 命令实现（args 为已切分的字符串列表）----

    def c_idcode(self, a):
        ids, bypass = self.jtag.scan_chain()
        if not ids and not bypass:
            print("no devices found (TDO all 0s? check chain)")
            return
        for i, v in enumerate(ids):
            print(f"#{i + 1} IDCODE={decode_idcode(v)}")
        for i in range(bypass):
            print(f"#{len(ids) + i + 1} BYPASS device (1 bit, no IDCODE)")
        print(f"{len(ids)} IDCODE + {bypass} bypass device(s)")

    def c_probe(self, a):
        self._dap()
        print("mem/AP cmds ready; 'aps' + 'ap <n>' to select domain")

    def c_aps(self, a):
        for apsel, idr in self._dap().scan_aps():
            print(f"APSEL {apsel:3d}: IDR=0x{idr:08X}  {Dap.ap_type_name(idr)}  "
                  f"designer=0x{(idr >> 17) & 0x3FF:03X}  "
                  f"rev={(idr >> 28) & 0xF} var={(idr >> 4) & 0xF}")

    def c_ap(self, a):
        if a:
            self.apsel = parse_ints(a, 1, 1, "ap [n]")[0]
            self.cm = None
            self._cm_checked = False   # CM 标志随 AP 失效：换域自动重检
        print(f"默认 APSEL = {self.apsel}")

    def c_dapinfo(self, a):
        apsel = parse_ints(a, 0, 1, "dapinfo [apsel]")
        apsel = apsel[0] if apsel else self.apsel
        CoreSight.info(self._dap(), apsel)
        self.dap.check_sticky()

    def c_speed(self, a):
        if a:
            self.jtag.set_speed(parse_ints(a, 1, 1, "speed [khz]")[0])
        print(f"TCK = {self.jtag.axi_hz // (2 * (self.jtag.clkdiv + 1)) // 1000} kHz")

    def _mem_access(self, name, args):
        size = {"w": 4, "h": 2, "b": 1}[name[2]]
        writes = name[0] == "w"
        vals = parse_ints(args, 1, 99 if writes else 2,
                          f"{name} ADDR {'V...' if writes else '[COUNT]'}")
        addr = vals[0]
        if writes:
            if addr % size:
                raise JtagError(f"地址必须 {size} 字节对齐")
            self._mem().write(addr, vals[1:], size)
        else:
            count = vals[1] if len(vals) > 1 else 1
            dump(addr, size, self._mem().read(addr, count, size))
        self.dap.check_sticky()

    def c_dpr(self, a):
        v = parse_ints(a, 1, 1, "dpr <reg>")
        print(f"DP[0x{v[0]:X}] = 0x{self._dap().dp_read(v[0]):08X}")
        self.dap.check_sticky()

    def c_dpw(self, a):
        r, val = parse_ints(a, 2, 2, "dpw <reg> <val>")
        self._dap().dp_write(r, val)
        if r == Dap.DP_SELECT:
            self.dap._sel = None      # SELECT 被直写，路由缓存失效
        print(f"DP[0x{r:X}] ← 0x{val:08X}")
        self.dap.check_sticky()

    def c_apr(self, a):
        v = parse_ints(a, 2, 3, "apr <bank> <reg> [apsel]")
        apsel = v[2] if len(v) > 2 else self.apsel
        d = self._dap().ap_read(v[1], apsel=apsel, bank=v[0])
        print(f"AP[{apsel}] bank 0x{v[0]:X} reg 0x{v[1]:X} = 0x{d:08X}")
        self.dap.check_sticky()

    def c_apw(self, a):
        v = parse_ints(a, 3, 4, "apw <bank> <reg> <val> [apsel]")
        apsel = v[3] if len(v) > 3 else self.apsel
        self._dap().ap_write(v[1], v[2], apsel=apsel, bank=v[0])
        print(f"AP[{apsel}] bank 0x{v[0]:X} reg 0x{v[1]:X} ← 0x{v[2]:08X}")
        self.dap.check_sticky()

    def c_halt(self, a):
        cm = self._cpu("halt")
        cm.halt()
        print(f"halted: pc=0x{cm.reg_read(15):08X}")

    def c_step(self, a):
        cm = self._cpu("step")
        cm.step()
        print(f"pc=0x{cm.reg_read(15):08X}")

    def c_resume(self, a):
        self._cpu("resume").resume()
        print("resumed")

    def c_reg(self, a):
        cm = self._cpu("reg")
        if not cm.halted():
            # stc12-jtag 语义：运行中 DCRSR 传送不发生，拒绝（先 halt）
            print("target is running")
            return
        if len(a) >= 2:              # reg <name> <val> 写
            sel = CortexM.REG_LOOKUP.get(a[0].lower())
            if sel is None:
                raise JtagError(f"未知寄存器 {a[0]}")
            val = parse_ints(a[1:2], 1, 1, "reg <name> <val>")[0]
            cm.reg_write(sel, val)
            print(f"{a[0]} ← 0x{val:08X}")
        elif a:
            sel = CortexM.REG_LOOKUP.get(a[0].lower())
            if sel is None:
                raise JtagError(f"未知寄存器 {a[0]}")
            print(f"{a[0]} = 0x{cm.reg_read(sel):08X}")
        else:
            items = list(cm.regs().items())
            for i in range(0, len(items), 3):
                print("   ".join(f"{n:9s}0x{v:08X}" for n, v in items[i:i + 3]))

    def c_target(self, a):
        dhcsr = self._mem().read32(CortexM.DHCSR)
        st = "halted" if dhcsr & CortexM.S_HALT else "running"
        if dhcsr & CortexM.S_SLEEP:
            st += " (sleeping)"
        if dhcsr & CortexM.S_LOCKUP:
            st += " (locked up)"
        print(f"target is {st} (DHCSR=0x{dhcsr:08X})")

    # ---- SWO（bring-up，不依赖 OpenOCD）----

    def c_swo(self, a):
        """swo <baud_hz|off> — 配接收分频并使能/关闭。"""
        if a and a[0] != "off":
            baud = parse_ints(a[:1], 1, 1, "swo <baud_hz|off>")[0]
            actual = self.jtag.swo_enable(baud)
            print(f"SWO on: 请求 {baud} Hz → 实际 RX {actual} Hz")
        else:
            self.jtag.swo_disable()
            print("SWO off")

    def c_swo_stat(self, a):
        cnt, ovr, fe = self.jtag.swo_stat()
        print(f"SWO_STAT: count={cnt} overrun={int(ovr)} frame_err={int(fe)}")
        if ovr or fe:
            self.jtag.swo_clear_err()
            print("  (sticky errors cleared)")

    def c_swo_read(self, a):
        """swo_read [n] — 读 n 字节 hex dump（默认全部可读）。"""
        n = parse_ints(a, 0, 1, "swo_read [n]")
        n = n[0] if n else None
        data = self.jtag.swo_read(n)
        if not data:
            print("(empty)")
            return
        for i in range(0, len(data), 16):
            row = data[i:i + 16]
            print(f"{i:04x}: " + " ".join(f"{b:02X}" for b in row) +
                  "  " + "".join(chr(b) if 32 <= b < 127 else "." for b in row))

    def c_swo_tpiu(self, a):
        """swo_tpiu <traceclk_hz> <baud> — 一条龙配目标侧 ITM/TPIU + 本地 RX。

        目标侧寄存器（ARMv7-M，STM32F1 同）：DEMCR.TRCENA → ITM LAR/TCR/TER
        → TPIU SPPR=UART / ACPR / FFCR.EnFCont。分频策略与 OpenOCD 驱动一致：
        TPIU prescaler 逼近 RX 实际值（v3 边沿 CDR 任意整数档，失配通常
        <1%，容差 ~±5%）。需先 probe。"""
        v = parse_ints(a, 2, 2, "swo_tpiu <traceclk_hz> <baud>")
        tclk, baud = v[0], v[1]
        mem = self._mem()
        # 本地 RX 先配（拿到 actual）
        actual = self.jtag.swo_enable(baud)
        presc = (tclk + actual // 2) // actual
        tpiu_baud = tclk // presc
        # ARMv7-M trace 寄存器
        DEMCR, TRCENA = 0xE000EDFC, 1 << 24
        mem.write32(DEMCR, mem.read32(DEMCR) | TRCENA)
        mem.write32(0xE0000FB0, 0xC5ACCE55)   # ITM LAR 解锁（CM3 无此寄存器，写无害）
        # F1 目标：AFIO_MAPR SWJ_CFG=010 释放 PB3 给 TPIU（固件不重映射时
        # SWO 出不了引脚，线上只见 SWD 轮询漏流；SWJ_CFG 在 bits[26:24]）
        AFIO_MAPR = 0x40010004
        mem.write32(AFIO_MAPR, (mem.read32(AFIO_MAPR) & ~(7 << 24)) | (2 << 24))
        # ITM TCR：ITMena|TXENA|BusID=1。M3 r1p1 实测 0x17 缺 TXENA(bit3)，
        # ITM 出不了 ATB（见 swotest.c 头注释），必须 0x00010009
        mem.write32(0xE0000E80, 0x00010009)
        mem.write32(0xE0000E00, 0x1)          # ITM TER: 端口 0
        mem.write32(0xE00400F0, 2)            # TPIU SPPR: async NRZ/UART
        mem.write32(0xE0040010, presc - 1)    # TPIU ACPR
        mem.write32(0xE0040304, 1 << 1)       # TPIU FFCR: EnFCont
        print(f"traceclk={tclk / 1e6:.2f} MHz  ACPR={presc - 1} → "
              f"TPIU 输出 {tpiu_baud} Hz；RX {actual} Hz "
              f"(失配 {abs(tpiu_baud - actual) / actual * 100:.2f}%)")
        print("目标固件往 0xE0000000 (ITM port0) 写字节即出 SWO；swo_read 查看")
        self.dap.check_sticky()

    # ---- 位级 JTAG / 链（自 stc12-jtag 移植）----

    def c_chain(self, a):
        self.jtag.chain_probe()

    def c_vref(self, a):
        """vref [dev] — SPI 查 ESP32 采的 JTAG Vref（两段式协议，同 C 版）。"""
        import fcntl

        dev = a[0] if a else "/dev/spidev0.0"
        fd = os.open(dev, os.O_RDWR)
        fcntl.ioctl(fd, 0x40016B01, b"\x03")   # SPI_IOC_WR_MODE = _IOW('k',1,u8), mode3
        try:
            for seq in (1, 2):
                _SpiXfer([0xA5, 0x01, seq, 0, 0, 0, 0, 0], 8).xfer(fd)
                time.sleep(0.002)               # 从机装应答帧余量
                resp = bytearray(16)
                _SpiXfer(bytes(16), 16, rx=resp).xfer(fd)
                if (resp[0] == 0x5A and resp[1] == 0x01 and
                        resp[2] == seq and resp[3] == 0):
                    mv = int.from_bytes(resp[4:8], "little")
                    print(f"vref = {mv // 1000}.{mv % 1000:03d} V")
                    return
            raise JtagError("ESP32 未应答（检查从机固件/接线/CS）")
        finally:
            os.close(fd)

    def c_rst(self, a):
        v = parse_ints(a, 1, 1, "rst <0|1>")[0] & 1
        self.jtag.srst(bool(v))
        print(f"SRST={'released' if v else 'asserted (low)'}")

    def c_treset(self, a):
        self.jtag.goto_tlr()
        print("TAP reset (TLR)")

    def c_reset(self, a):
        """完整复位：IP 引擎（FIFO+FSM，CTRL.abort）+ 目标 TAP（TLR）+ 引脚恢复。"""
        self.jtag.reset_engine()
        # ARM 层的 IR/路由缓存全部失效（TLR epoch 已递增，Tap 缓存自动失效）
        if self.dap is not None:
            self.dap._sel = None
        print("engine aborted (FIFO/FSM cleared), target TAP -> TLR, pins restored")

    def c_tbit(self, a):
        v = parse_ints(a, 2, 2, "tbit <tms> <tdi>")
        print(f"TDO={self.jtag.shift([v[0] & 1], [v[1] & 1])[0]}")

    def c_tms(self, a):
        if not a:
            raise JtagError("用法: tms <0|1> ...（TDI=1，逐拍打印 TDO）")
        seq = parse_ints(a, 1, 32, "tms <0|1> ...")
        cap = self.jtag.shift([v & 1 for v in seq], [1] * len(seq))
        print("TDO: " + "".join(map(str, cap)))

    def c_tdr(self, a):
        n = parse_ints(a, 1, 1, "tdr <n>")[0]
        if not 1 <= n <= 256:
            raise JtagError("n range: 1..256")
        cap = self.jtag.shift([0] * (n - 1) + [1], [1] * n)
        print("DR: " + "".join(map(str, cap)))

    def c_tir(self, a):
        v, n = parse_ints(a, 2, 2, "tir <hex> <nbits>")
        if not 1 <= n <= 32:
            raise JtagError("nbits range: 1..32")
        bits = [(v >> i) & 1 for i in range(n)]
        # TLR → RTI → SelDR → SelIR → CapIR → ShiftIR → 移位 → Update → RTI
        self.jtag.shift([1] * 5 + [0, 1, 1, 0, 0] + [0] * (n - 1) + [1, 1, 0],
                        [0] * 10 + bits + [0, 0])
        print(f"IR<=0x{v:X} ({n} bits), updated")

    def c_tdo(self, a):
        print(f"TDO={self.jtag.tdo_pin()} (no clock)")

    def c_dump(self, a):
        """dump <start> <count> [file] — 内存 dump 到文件（32 位 word，LE 二进制）。"""
        v = parse_ints(a, 2, 3, "dump <start> <count> [file]")
        start, count = v[0], v[1]
        fname = a[2] if len(a) > 2 else f"dump_{start:08X}_{count}w.bin"
        mem = self._mem()
        t0 = time.perf_counter()
        try:
            with open(fname, "wb") as f:
                done = 0
                while done < count:
                    n = min(256, count - done)
                    f.write(struct.pack(f"<{n}I",
                                        *mem.read(start + 4 * done, n, 4)))
                    done += n
                    print(f"\r{done}/{count}", end="", flush=True)
        except KeyboardInterrupt:
            print("\n(已中止，部分数据已写入)")
        dt = time.perf_counter() - t0
        print(f"{fname}: {done * 4} bytes, {dt:.2f}s, {done * 4 / dt / 1024:.1f} KB/s")
        self.dap.check_sticky()

    def c_bench(self, a):
        """链路时序基准（stc12 bench 的 host 版）：裸 TCK 位率 + DPACC/APACC。
        TCK 位率用 4×512 位分块测（单次 shift 受 BITCNT 10 位上限 1023）。"""
        j, n = self.jtag, 4 * 512
        t0 = time.perf_counter()
        for _ in range(4):
            j.shift([0] * 512, [0] * 512)
        dt = time.perf_counter() - t0
        print(f"raw TCK        : {n / dt / 1000:.0f} kHz ({dt / n * 1e6:.2f} us/bit)")
        dap = self._dap()
        t0 = time.perf_counter()
        for _ in range(10):
            dap._acc(False, Dap.DP_RDBUFF, 1)
        dt = time.perf_counter() - t0
        print(f"10x DPACC(RDBUFF): {dt / 10 * 1e3:.2f} ms/txn")
        t0 = time.perf_counter()
        for _ in range(10):
            dap._acc(True, MemAp.AP_TAR, 0, 0x20000000)
        dt = time.perf_counter() - t0
        print(f"10x APACC(TAR wr): {dt / 10 * 1e3:.2f} ms/txn")

    def c_dbg(self, a):
        """IR 路径对照实验（stc12 cmd_dbg）：链方向/TAP 顺序诊断。"""
        j = self.jtag
        # A) TLR 后裸 DR 70 位：各 IDCODE 的真实位置
        j.goto_tlr()
        d = j._shift_dr_raw(70, [0] * 70)
        print("A raw DR post-TLR : " + " ".join(
            f"{sum(d[k * 8 + i] << i for i in range(8) if k * 8 + i < 70):02X}"
            for k in range(9)))
        # B) 旁路在 TDO 侧（ir_before=6）拓扑：IR 装载期间捕获 DP 的 IR 回读
        j.goto_tlr()
        cap = j.shift([0, 1, 1, 0, 0] + [0] * 6 + [0, 0, 0, 1] + [1, 0],
                      [1] * 11 + [(0xE >> i) & 1 for i in range(4)] + [1, 1])
        ir_cap = sum(cap[11 + i] << i for i in range(4))
        print(f"B IR shift TDO    : DP-IR capture=0x{ir_cap:X}")
        # C) 同拓扑下裸 DR 40 位
        d = j._shift_dr_raw(40, [0] * 40)
        print("C raw DR after IR : " + " ".join(
            f"{sum(d[k * 8 + i] << i for i in range(8) if k * 8 + i < 40):02X}"
            for k in range(5)))
        j.goto_tlr()

    # ---- scandump（Arise2 私有人格）----

    def c_scan_switch(self, a):
        """经 AP2 写 0x50430064=1，TAP 从 ARM 人格切到 scandump（须先 probe）。"""
        if self.dap is None:
            raise JtagError("scan_switch: 先 probe（需经 AP2 写切换寄存器）")
        mem2 = MemAp(self.dap, 2)
        try:
            mem2.write32(0x50430064, 1)
        except JtagError:
            pass      # 切换瞬间 DAP 掉线、末拍 ACK=0 属正常伴生现象
        self.dap.check_sticky()
        # TAP 人格已换：ARM 层状态全部失效
        self.dap = self.mem = self.cm = None
        self._cm_checked = False
        print("TAP switched to scandump persona (wrote 0x50430064=1 via AP2)")

    def c_scan_id(self, a):
        v, ok = self._sd().read_idcode()
        print(f"IDCODE=0x{v:08X} "
              f"{'(match)' if ok else '(MISMATCH, expect 0xAABBCCDD)'}")

    def c_scan_bypass(self, a):
        ok, echo = self._sd().test_bypass()
        print(f"BYPASS {'OK' if ok else 'FAIL'} (echo=0x{echo:04X}, expect 0x5554)")

    def c_scan_sel(self, a):
        c = parse_ints(a, 1, 1, "scan_sel <0=CFG 1=SCAN_I 2=SCAN_O>")[0]
        self._sd()._path_setup(c)
        print(f"chain {c} selected, DEBUG mode")

    def c_scan_dbg(self, a):
        self._sd()._ir_load(ScanDump.INSTR_DEBUG)
        print("DEBUG instruction loaded")

    def _scan_wreg(self, a, reg):
        v = parse_ints(a, 1, 1, f"scan_w{reg} <val>")[0]
        self._sd().cfg_write(reg, v)
        print(f"CFG{reg} ← 0x{v:02X}")

    def _scan_rreg(self, a, reg):
        v = self._sd().cfg_read(reg)
        print(f"CFG{reg} = 0x{v:02X}")
        if reg == ScanDump.CFG_TRIGGER:
            print("  (trigger 位硬件自清，读到 0x0 属正常)")

    def c_scan_in(self, a):
        v = parse_ints(a, 1, 2, "scan_in <lo32> [hi32]")
        w0 = v[0] & 0xFFFFFFFF
        w1 = v[1] & 0xFFFFFFFF if len(v) > 1 else 0
        self._sd().scan_in_write(w0, w1)
        print(f"scan_in 0x{w1:08X}_{w0:08X}")

    def c_scan_out(self, a):
        sd = self._sd()
        sd._path_setup(sd.CHAIN_SCAN_OUT)
        sd.print_row(0, sd.scan_out_read())

    def c_scan_t(self, a):
        sd = self._sd()
        sd.cfg_write(sd.CFG_TRIGGER, 1)
        sd._path_setup(sd.CHAIN_SCAN_OUT)
        sd.print_row(0, sd.scan_out_read())

    def c_scan_m(self, a):
        v = parse_ints(a, 1, 3, "scan_m <0-3> [reads] [cfg2]")
        burst = v[0]
        if burst > 3:
            raise JtagError("burst 0..3")
        reads = v[1] if len(v) > 1 else (1, 4, 16, 64)[burst]
        cfg2 = v[2] if len(v) > 2 else ScanDump.CFG2_DUMP
        self._sd().dump(reads, cfg2=cfg2, burst=burst)

    def c_scan_a(self, a):
        v = parse_ints(a, 1, 2, "scan_a <rows> [cfg2]")
        cfg2 = v[1] if len(v) > 1 else ScanDump.CFG2_DUMP
        self._sd().dump(v[0], cfg2=cfg2)

    def c_scan_stop(self, a):
        self._sd().stop()
        print("dump stopped")

    def c_scan_memkeep(self, a):
        self._sd().mem_keep()

    def c_scan_fmt(self, a):
        self._sd().fmt_bit = not self._sd().fmt_bit
        print(f"row format: {'bit' if self._sd().fmt_bit else 'hex'}")

    def c_scan_verify(self, a):
        self._sd().test_verify()

    def c_help(self, a):
        """分组按状态显示（stc12 arm_help_hook 逻辑）：ARM 组 probe 后、
        Cortex-M 运行控制组确认核后才出现。"""
        print("-- general --")
        print("  scan              通用链扫描（IDCODE+BYPASS 解析）")
        print("  chain             链拓扑：BYPASS 位宽 + 逐 TAP IR 长度")
        print("  probe             链扫描 + DAP 上电（3 拓扑自动试探）")
        print("  speed [khz]       查看/设定 TCK")
        print("  vref [dev]        SPI 查 ESP32 采的 JTAG Vref")
        print("  rst <0|1>         SRST 复位线（0=拉低复位）")
        print("  reset             完整复位：IP 引擎(FIFO+FSM) + 目标 TAP")
        print("-- SWO trace（IP VERSION>=2；SWD 模式下 TDO 引脚 = TRACESWO）--")
        print("  swo <baud|off>    接收器使能/关闭（0→1 沿 flush FIFO）")
        print("  swo_stat          字节数 + overrun/frame_err sticky")
        print("  swo_read [n]      读 n 字节 hex dump（默认全部）")
        print("  swo_tpiu <tclk> <baud>  配目标 ITM/TPIU + 本地 RX（须先 probe）")
        print("-- bit-level JTAG（私有 IR / 非标准 TAP 探索）--")
        print("  treset            TAP 复位到 Test-Logic-Reset")
        print("  tbit <tms> <tdi>  一拍 TCK，打印 TDO")
        print("  tms <0|1>...      走 TMS 序列（TDI=1），逐拍打印 TDO")
        print("  tdr <n>           ShiftDR 移 n 位（TDI=1）")
        print("  tir <hex> <nbits> 手工装 IR（LSB first）然后 Update")
        print("  tdo               不打时钟读 TDO 引脚")
        if self.dap is None:
            print("-- ARM CoreSight：先 'probe' 解锁本组命令 --")
        else:
            print("-- ARM CoreSight（target detected，'probe' 重新探测）--")
            print("  aps               枚举 DAP 的 AP")
            print("  ap [n]            查看/设定默认 APSEL（换域重检 CM）")
            print("  dapinfo [apsel]   ROM 表组件发现（LPAE 打 64 位 BASE）")
            print("  dpr <r> / dpw <r> <v>            DP 寄存器读写")
            print("  apr <bank> <r> [v] [apsel]       AP 寄存器读写")
            print("  mdw|mdh|mdb <a> [n]              读内存（8/8/16 每行）")
            print("  mww|mwh|mwb <a> <v>...           写内存（成功静默）")
            print("  dump <a> <n> [f]  内存 dump 到文件（word，LE bin）")
            print("  bench             链路时序基准")
            print("  dbg               IR 路径对照实验（链方向诊断）")
            if self._cm_ok:
                print("-- Cortex-M run control --")
                print("  target            halted/running 状态")
                print("  halt / step / resume   运行控制")
                print("  reg [name] [val] 核内寄存器 读/写（须先 halt）")
            else:
                print("  (Cortex-M 运行控制：在核所在 AP 上执行 halt/reg 等自动确认)")
        print("-- scandump（Arise2 私有人格，命名对齐 openocd scan_*）--")
        print("  scan_switch       TAP 切到 scandump 人格（经 AP2，须先 probe）")
        print("  scan_id / scan_bypass        IDCODE 校验 / BYPASS 自检")
        print("  scan_sel <c> / scan_dbg      选 DR 通路 / 装 DEBUG 指令")
        print("  scan_w0/1/2 <v> / scan_r0/1/2  CFG0/1/2 读写")
        print("  scan_in <lo32> [hi32]        写 SCAN_I 64bit")
        print("  scan_out / scan_t            读一行 SCAN_O / 触发一拍再读")
        print("  scan_m <0-3> [n] [cfg2]      manual dump（burst 1/4/16/64 拍）")
        print("  scan_a <rows> [cfg2]         auto dump（openocd 风格）")
        print("  scan_stop / scan_memkeep     停止 / dump 后保内存")
        print("  scan_fmt / scan_verify       hex/bit 行格式 / 全 1 自检")


def _build_cmds():
    cmds = {
        "scan":    (Shell.c_idcode,  "scan              通用链扫描（IDCODE+BYPASS 解析）"),
        "chain":   (Shell.c_chain,   "chain             链拓扑：BYPASS 位宽 + 逐 TAP IR 长度"),
        "probe":   (Shell.c_probe,   "probe             链扫描 + DAP 上电（3 拓扑自动试探）"),
        "aps":     (Shell.c_aps,     "aps               枚举 DAP 的 AP"),
        "ap":      (Shell.c_ap,      "ap [n]            查看/设定默认 APSEL（换域重检 CM）"),
        "dapinfo": (Shell.c_dapinfo, "dapinfo [apsel]   ROM 表组件发现（LPAE 打 64 位 BASE）"),
        "speed":   (Shell.c_speed,   "speed [khz]       查看/设定 TCK"),
        "dump":    (Shell.c_dump,    "dump <a> <n> [f]  内存 dump 到文件（word，LE bin）"),
        "bench":   (Shell.c_bench,   "bench             链路时序基准（TCK 位率 + DPACC）"),
        "dbg":     (Shell.c_dbg,     "dbg               IR 路径对照实验（链方向诊断）"),
        "dpr":     (Shell.c_dpr,     "dpr <reg>         读 DP 寄存器"),
        "dpw":     (Shell.c_dpw,     "dpw <reg> <val>   写 DP 寄存器"),
        "apr":     (Shell.c_apr,     "apr <bank> <reg> [apsel]       读 AP 寄存器"),
        "apw":     (Shell.c_apw,     "apw <bank> <reg> <val> [apsel] 写 AP 寄存器"),
        "halt":    (Shell.c_halt,    "halt              停住 CPU"),
        "step":    (Shell.c_step,    "step              单步一条指令"),
        "resume":  (Shell.c_resume,  "resume            继续运行"),
        "reg":     (Shell.c_reg,     "reg [name] [val]  核内寄存器 读/写（须先 halt）"),
        "target":  (Shell.c_target,  "target            halted/running 状态"),
        "vref":    (Shell.c_vref,    "vref [dev]        SPI 查 ESP32 采的 JTAG Vref"),
        "rst":     (Shell.c_rst,     "rst <0|1>         SRST 复位线（0=拉低复位）"),
        "reset":   (Shell.c_reset,   "reset             完整复位：IP 引擎+目标 TAP"),
        "swo":     (Shell.c_swo,     "swo <baud|off>    SWO 接收器使能/关闭"),
        "swo_stat": (Shell.c_swo_stat, "swo_stat          SWO FIFO 状态"),
        "swo_read": (Shell.c_swo_read, "swo_read [n]      读 SWO 字节 hex dump"),
        "swo_tpiu": (Shell.c_swo_tpiu, "swo_tpiu <tclk> <baud>  配目标 ITM/TPIU+RX"),
        "treset":  (Shell.c_treset,  "treset            TAP 复位到 TLR"),
        "tbit":    (Shell.c_tbit,    "tbit <tms> <tdi>  一拍 TCK，打印 TDO"),
        "tms":     (Shell.c_tms,     "tms <0|1>...      走 TMS 序列（TDI=1）"),
        "tdr":     (Shell.c_tdr,     "tdr <n>           ShiftDR 移 n 位（TDI=1）"),
        "tir":     (Shell.c_tir,     "tir <hex> <nbits> 手工装 IR（LSB first）"),
        "tdo":     (Shell.c_tdo,     "tdo               不打时钟读 TDO 引脚"),
        "scan_switch":  (Shell.c_scan_switch,  "scan_switch       TAP 切到 scandump 人格（经 AP2）"),
        "scan_id":      (Shell.c_scan_id,      "scan_id           读 IDCODE 并校验"),
        "scan_bypass":  (Shell.c_scan_bypass,  "scan_bypass       BYPASS 链路自检（0xAAAA→0x5554）"),
        "scan_sel":     (Shell.c_scan_sel,     "scan_sel <c>      选 DR 通路 0=CFG 1=SCAN_I 2=SCAN_O"),
        "scan_dbg":     (Shell.c_scan_dbg,     "scan_dbg          装 DEBUG 指令"),
        "scan_r0":      (lambda sh, a: sh._scan_rreg(a, 0), "scan_r0/1/2       读 CFG0/1/2"),
        "scan_r1":      (lambda sh, a: sh._scan_rreg(a, 1), None),
        "scan_r2":      (lambda sh, a: sh._scan_rreg(a, 2), None),
        "scan_w0":      (lambda sh, a: sh._scan_wreg(a, 0), "scan_w0/1/2 <v>   写 CFG0/1/2"),
        "scan_w1":      (lambda sh, a: sh._scan_wreg(a, 1), None),
        "scan_w2":      (lambda sh, a: sh._scan_wreg(a, 2), None),
        "scan_in":      (Shell.c_scan_in,      "scan_in <lo32> [hi32]   写 SCAN_I 64bit"),
        "scan_out":     (Shell.c_scan_out,     "scan_out          读一行 SCAN_O（119bit）"),
        "scan_t":       (Shell.c_scan_t,       "scan_t            触发一拍 shift_clk + 读"),
        "scan_m":       (Shell.c_scan_m,       "scan_m <0-3> [n] [cfg2]  manual dump（burst 1/4/16/64）"),
        "scan_a":       (Shell.c_scan_a,       "scan_a <rows> [cfg2]    auto dump（openocd 风格）"),
        "scan_stop":    (Shell.c_scan_stop,    "scan_stop         停止 dump（CFG2=0）"),
        "scan_memkeep": (Shell.c_scan_memkeep, "scan_memkeep      dump 后保内存（MIU 序列）"),
        "scan_fmt":     (Shell.c_scan_fmt,     "scan_fmt          hex/bit 行格式切换"),
        "scan_verify":  (Shell.c_scan_verify,  "scan_verify       SCAN_I 全 1 写读自检"),
        "help":    (Shell.c_help,    "help              本表"),
    }
    for sfx, sname in (("w", "word"), ("h", "halfword"), ("b", "byte")):
        size = {"w": 4, "h": 2, "b": 1}[sfx]
        for op, verb in (("d", "读"), ("w", "写")):
            name = f"m{op}{sfx}"
            cmds[name] = (
                lambda sh, a, n=name: sh._mem_access(n, a),
                f"{name} ADDR {'V...' if op == 'w' else '[COUNT]':14s} "
                f"{verb}目标内存（{size*8} 位，经 MEM-AP）")
    return cmds

Shell.CMDS = _build_cmds()


def sh_exec(sh, line):
    """执行一行 shell 命令；返回 False 表示 quit。异常由调用方处理。"""
    name, *args = line.split()
    if name in ("quit", "exit", "q"):
        return False
    entry = Shell.CMDS.get(name)
    if entry is None:
        raise JtagError(f"未知命令 {name}（help 查看）")
    entry[0](sh, args)
    return True


def run_shell(sh):
    print("jtag_tool shell — help 查看命令；quit / Ctrl+D 退出")
    try:
        import readline  # noqa: F401  历史记录（板上有 python3-readline）
    except ImportError:
        pass
    while True:
        try:
            line = input("jtag> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not line or line.startswith("#"):
            continue
        try:
            sh_exec(sh, line)
        except JtagError as e:
            print(f"ERR: {e}")
        except KeyboardInterrupt:
            print("\n(命令已中止)")


# ---------------------------------------------------------------------------
# 命令行界面（仅作为脚本运行时使用；作为模块引用时与下面代码无关）
# ---------------------------------------------------------------------------

def cmd_probe(jtag, full):
    for i, v in enumerate(jtag.scan_idcodes()):
        print(f"TAP#{i}: {decode_idcode(v)}")
    if not full:
        return
    try:
        dap = Dap.probe(jtag)
        cs = dap.powerup()
        print(f"ARM DAP OK: CTRL/STAT=0x{cs:08x}")
    except JtagError as e:
        print(f"ARM DAP: 不可用 ({e})")


def arm_context(jtag, apsel=0):
    """构造 ARM 目标的完整访问链：DAP → 上电 → 指定 APSEL 的 AHB-AP。

    单 AP 目标的便利一行糖。多 AP 系统请共享同一个 DAP，避免重复
    probe/powerup：
        dap = Dap.probe(jtag); dap.powerup()
        mem0 = MemAp(dap, 0); mem1 = MemAp(dap, 1)
    """
    dap = Dap.probe(jtag)
    dap.powerup()
    return MemAp(dap, apsel)


def main():
    ap = argparse.ArgumentParser(
        description="jtag_swd_dbg IP 独立调试工具（命令与输出格式同 openocd）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("用法")[1] if "用法" in __doc__ else None)
    ap.add_argument("--speed", type=int, default=1000, help="TCK kHz")
    ap.add_argument("--axi-hz", type=lambda s: int(s, 0), default=125_000_000,
                    help="S_AXI_ACLK（默认 125M=本板 FCLK0；VERSION 自动探测"
                         "会截断成 124M，SWO 分频差 0.8%%，0=用自动值）")
    ap.add_argument("--base", type=lambda s: int(s, 0), default=0x43C00000,
                    help="IP AXI 基址")
    ap.add_argument("--dev", default="/dev/mem")
    ap.add_argument("--ap", type=int, default=0,
                    help="AHB-AP 的 APSEL（用 aps 命令枚举）")
    ap.add_argument("-c", "--command", action="append", default=[],
                    help="shell 模式下执行一条命令（可重复），执行完退出")
    sub = ap.add_subparsers(dest="cmd", required=False)

    sub.add_parser("idcode", help="扫描 JTAG 链")
    sub.add_parser("probe", help="链扫描 + ARM DAP 诊断")
    sub.add_parser("aps", help="枚举 DAP 里的所有 AP")
    sp = sub.add_parser("dapinfo", help="对应 openocd dap info：AP + ROM 表组件发现")
    sp.add_argument("apsel", type=int, nargs="?", default=0)

    def mem_cmd(name, help_txt, writes):
        sp = sub.add_parser(name, help=help_txt)
        sp.add_argument("addr", type=lambda s: int(s, 0))
        if writes:
            sp.add_argument("vals", type=lambda s: int(s, 0), nargs="+")
        else:
            sp.add_argument("count", type=int, nargs="?", default=1)

    for w in (4, 2, 1):
        sfx = {4: 'w', 2: 'h', 1: 'b'}[w]
        mem_cmd(f"md{sfx}", f"读 {w*8} 位", False)
        mem_cmd(f"mw{sfx}", f"写 {w*8} 位", True)

    sub.add_parser("halt", help="停住 CPU")
    sub.add_parser("resume", help="继续运行")
    sub.add_parser("step", help="单步一条指令")
    sp = sub.add_parser("reg", help="读 CPU 寄存器（无参数=全部）")
    sp.add_argument("name", nargs="?", help="如 pc / sp / r0 / xPSR / MSP")

    args = ap.parse_args()

    try:
        with JtagMaster(dev_file=args.dev, base=args.base,
                        speed_khz=args.speed,
                        axi_hz=args.axi_hz or None) as jtag:
            if args.cmd is None:          # shell 模式（无子命令）
                sh = Shell(jtag, apsel=args.ap)
                for line in args.command:
                    print(f"jtag> {line}")
                    try:
                        if not sh_exec(sh, line):
                            break
                    except JtagError as e:
                        print(f"ERR: {e}")
                if not args.command:
                    run_shell(sh)
            elif args.cmd == "idcode":
                cmd_probe(jtag, full=False)
            elif args.cmd == "probe":
                cmd_probe(jtag, full=True)
            elif args.cmd == "aps":
                dap = Dap.probe(jtag)
                dap.powerup()
                found = dap.scan_aps()
                if not found:
                    print("未发现任何 AP")
                for apsel, idr in found:
                    print(f"APSEL {apsel:3d}: IDR=0x{idr:08X}  "
                          f"{Dap.ap_type_name(idr)}  "
                          f"designer=0x{(idr >> 17) & 0x3FF:03X}  "
                          f"rev={(idr >> 28) & 0xF} var={(idr >> 4) & 0xF}")
            elif args.cmd == "dapinfo":
                dap = Dap.probe(jtag)
                dap.powerup()
                CoreSight.info(dap, args.apsel)
            elif args.cmd in ("halt", "resume", "step", "reg"):
                cm = CortexM(arm_context(jtag, args.ap))
                if args.cmd == "halt":
                    cm.halt()
                    print(f"halted: pc=0x{cm.reg_read(15):08X}")
                elif args.cmd == "resume":
                    cm.resume()
                elif args.cmd == "step":
                    cm.step()
                    print(f"pc=0x{cm.reg_read(15):08X}")
                elif args.name:
                    if not cm.halted():
                        print("target is running")
                    else:
                        sel = CortexM.REG_LOOKUP.get(args.name.lower())
                        if sel is None:
                            raise JtagError(f"未知寄存器 {args.name}")
                        print(f"{args.name} = 0x{cm.reg_read(sel):08X}")
                else:
                    if not cm.halted():
                        print("target is running")
                    else:
                        regs = cm.regs()
                        items = list(regs.items())
                        for i in range(0, len(items), 3):
                            print("   ".join(f"{n:9s}0x{v:08X}"
                                             for n, v in items[i:i + 3]))
            else:
                size = {"w": 4, "h": 2, "b": 1}[args.cmd[2]]
                mem = arm_context(jtag, args.ap)
                if args.cmd.startswith("md"):
                    dump(args.addr, size,
                         mem.read(args.addr, args.count, size))
                else:
                    if args.addr % size:
                        raise JtagError(f"地址必须 {size} 字节对齐")
                    mem.write(args.addr, args.vals, size)
    except JtagError as e:
        sys.exit(f"ERR: {e}")
    except OSError as e:
        sys.exit(f"ERR: 打不开 {args.dev}（需要 root / 板上运行）：{e}")


if __name__ == "__main__":
    main()
