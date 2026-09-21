#define _GNU_SOURCE
/*
 * jtag_tool.c — jtag_tool.py 的 C 实现（性能版），逻辑逐层对齐 Python 版 v1.1：
 *
 *   JtagMaster  IP 寄存器/移位引擎/链扫描/位级引脚
 *   Tap         单 TAP IR/DR 窗口访问（其余 TAP 置 BYPASS）
 *   Dap         ADIv5 JTAG-DP（DPACC/APACC，3 拓扑 probe，sticky 自动清）
 *   MemAp       MEM-AP 内存 8/16/32 位 + LPAE 64 位地址直通
 *   CortexM     CPUID 探测/halt/resume/step/reg 读写
 *   CoreSight   ROM 表组件发现（dapinfo）
 *   ScanDump    Arise2 私有 TAP 人格（scan_* 命令组）
 *
 * 编译（静态链接，避免与板端 libc 版本不匹配）:
 *   make                 # 交叉编译 → ./jtag_tool
 *   make host            # 本机编译（语法/逻辑自检用）
 * 板上运行（root）:
 *   ./jtag_tool                # 交互 shell（'help' 查看命令）
 *   ./jtag_tool -c probe -c "mdw 0x20000000 4"
 *   ./jtag_tool --serve        # 网络服务器（swo_web GUI 后端）：
 *                              #   命令口 :5555（行文本 + Z<seq>D 标记应答）
 *                              #   SWO 流口 :5556（原始字节直推）
 * 选项: --base 0x43C00000 --speed 10000 --dev /dev/mem --axi-hz 125000000 --ap 0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>
#include <stdarg.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/uio.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <errno.h>
#include <linux/spi/spidev.h>
#ifndef NO_READLINE         /* host 自检无 arm readline，退回 fgets */
#include <readline/readline.h>
#include <readline/history.h>
#endif
#include "target_port.h"

/* ------------------------------------------------------------------ */
/* 工具                                                                */
/* ------------------------------------------------------------------ */

static char g_err_last[128];   /* 最近一条 err() 文本：BIN 错误应答带上真实原因 */

static void err(const char *fmt, ...)
{
    va_list ap;
    char buf[sizeof(g_err_last)];
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    printf("ERR: %s\n", buf);
    strcpy(g_err_last, buf);
}

static double now_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static uint64_t parse_num(const char *s)
{
    return strtoull(s, NULL, 0);
}

/* ------------------------------------------------------------------ */
/* JtagMaster                                                          */
/* ------------------------------------------------------------------ */

#define R_MAGIC   0x00
#define R_VERSION 0x04
#define R_CTRL    0x08
#define R_STATUS  0x0C
#define R_CLKDIV  0x10
#define R_BITCNT  0x14
#define R_PIN_DIR 0x18
#define R_PIN_OUT 0x1C
#define R_PIN_IN  0x20
#define R_TX_FIFO 0x24
#define R_RX_FIFO 0x28
#define R_SWO_CTRL 0x30
#define R_SWO_DIV  0x34
#define R_SWO_STAT 0x38
#define R_SWO_FIFO 0x3C

#define CTRL_START   (1u << 0)
#define CTRL_ABORT   (1u << 1)
#define CTRL_MODE_SWD (1u << 3)
#define CTRL_CAPTURE (1u << 4)
#define ST_BUSY      (1u << 0)
#define ST_RX_READY  (1u << 3)
#define ST_TX_FULL   (1u << 4)
#define MAGIC_VAL    0x4A534447u
#define PIN_OUT_SRST (1u << 3)

#define SWO_CTRL_EN     (1u << 0)
#define SWO_CTRL_CLRERR (1u << 1)
#define SWO_STAT_OVR    (1u << 16)
#define SWO_STAT_FE     (1u << 17)
#define SWO_FIFO_DEPTH  1024

#define MAX_BITS 1023            /* BITCNT 寄存器 10 位上限 */

typedef struct {
    int fd;
    volatile uint32_t *regs;
    int clkdiv;
    uint32_t axi_hz;
    uint32_t pin_out;
    int tlr_count;
    int has_swo;                 /* VERSION[31:16] >= 2 */
    int swo_cdr;                 /* VERSION[31:16] >= 3：SWO_DIV=位周期-1（边沿 CDR） */
} jtag_t;

static uint32_t jrd(const jtag_t *j, int off)
{
    return j->regs[off / 4];
}

static void jwr(jtag_t *j, int off, uint32_t v)
{
    j->regs[off / 4] = v;
}

static int j_wait_idle(jtag_t *j, double timeout_s)
{
    double dl = now_sec() + timeout_s;
    while (jrd(j, R_STATUS) & ST_BUSY) {
        if (now_sec() > dl) {
            err("移位引擎忙超时");
            return -1;
        }
    }
    return 0;
}

/* 核心原语：逐 bit 驱动 TMS/TDI，捕获 TDO（LSB 在前，与 py 版一致）。
 * swd=1 时语义变为 SWD 模式：tms[]=SWDIO 方向（1=驱动 tdi[] 的值，
 * 0=hi-Z 听线），cap[] 收 SWDIO pad——与 openocd zynq_dbg 驱动一致。
 * 失败时复位引擎（清 FIFO 残留）再返回 -1。 */
static int j_shift(jtag_t *j, const uint8_t *tms, const uint8_t *tdi, int n,
                   uint8_t *cap, int swd)
{
    uint32_t mode = swd ? CTRL_MODE_SWD : 0;
    if (n <= 0 || n > MAX_BITS) {
        err("移位长度 %d 超范围 (1..%d)", n, MAX_BITS);
        return -1;
    }
    if (j_wait_idle(j, 1.0) < 0)
        goto fail;
    jwr(j, R_CTRL, CTRL_CAPTURE | mode);
    jwr(j, R_BITCNT, (uint32_t)n);
    for (int w = 0; w < (n + 15) / 16; w++) {
        uint32_t low = 0, high = 0;
        for (int b = 0; b < 16; b++) {
            int i = w * 16 + b;
            if (i >= n)
                break;
            if (tdi[i])
                low |= 1u << b;
            if (tms[i])
                high |= 1u << b;
        }
        double dl = now_sec() + 1.0;
        while (jrd(j, R_STATUS) & ST_TX_FULL) {
            if (now_sec() > dl) {
                err("TX FIFO 满超时");
                goto fail;
            }
        }
        jwr(j, R_TX_FIFO, (high << 16) | low);
    }
    jwr(j, R_CTRL, CTRL_CAPTURE | CTRL_START | mode);
    if (j_wait_idle(j, 1.0) < 0)
        goto fail;
    double dl = now_sec() + 1.0;
    for (int w = 0; w < (n + 15) / 16; w++) {
        while (!(jrd(j, R_STATUS) & ST_RX_READY)) {
            if (now_sec() > dl) {
                err("RX FIFO 超时");
                goto fail;
            }
        }
        uint32_t v = jrd(j, R_RX_FIFO);
        if (!cap)
            continue;               /* 只排水（SWD 序列等不需要回读） */
        for (int b = 0; b < 16 && w * 16 + b < n; b++)
            cap[w * 16 + b] = (v >> b) & 1;
    }
    return 0;
fail:
    /* CTRL.abort：硬件清 TX+RX FIFO + 移位 FSM 回 idle（防残留毒化后续） */
    jwr(j, R_CTRL, CTRL_CAPTURE | CTRL_ABORT);
    jwr(j, R_CTRL, CTRL_CAPTURE);
    return -1;
}

static int j_shift1(jtag_t *j, int tms, int tdi)   /* 单 bit 便捷 */
{
    uint8_t t = tms & 1, d = tdi & 1, cap;
    return j_shift(j, &t, &d, 1, &cap, 0) < 0 ? -1 : cap;
}

static void j_goto_tlr(jtag_t *j)
{
    uint8_t tms[6] = { 1, 1, 1, 1, 1, 0 };
    uint8_t tdi[6] = { 0 };
    uint8_t cap[6];
    j_shift(j, tms, tdi, 6, cap, 0); /* 任意态 5×TMS=1 必到 TLR，+1 拍到 RTI */
    j->tlr_count++;
}

static void j_reset_engine(jtag_t *j)
{
    /* 完整复位：abort（FIFO+FSM）+ 目标 TAP 拉回 TLR */
    jwr(j, R_CTRL, CTRL_CAPTURE | CTRL_ABORT);
    jwr(j, R_CTRL, CTRL_CAPTURE);
    j_goto_tlr(j);
}

static void j_set_speed(jtag_t *j, int speed_khz)
{
    long div = (long)((double)j->axi_hz / (2.0 * speed_khz * 1000.0)) - 1;
    if (div < 0)
        div = 0;
    j->clkdiv = (int)div;
    jwr(j, R_CLKDIV, (uint32_t)j->clkdiv);
}

static int j_open(jtag_t *j, const char *dev, uint32_t base, int speed_khz,
                  uint32_t axi_hz)
{
    memset(j, 0, sizeof(*j));
    j->fd = open(dev, O_RDWR | O_SYNC);
    if (j->fd < 0) {
        err("打不开 %s（需要 root / 板上运行）", dev);
        return -1;
    }
    void *p = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, j->fd, base);
    if (p == MAP_FAILED) {
        err("mmap 0x%08X 失败", base);
        close(j->fd);
        return -1;
    }
    j->regs = (volatile uint32_t *)p;
    uint32_t magic = jrd(j, R_MAGIC);
    if (magic != MAGIC_VAL) {
        err("IP MAGIC 不匹配: 0x%08X", magic);
        return -1;
    }
    uint32_t ver = jrd(j, R_VERSION);
    int fifo_bits = (1 << (ver & 0xFF)) * 16;
    /* max_tck = floor(AXI/2) 有截断（本板 FCLK0=125M 只报 62 → 124M，差
     * 0.8%）。SWO 分频对真实 AXI 敏感，--axi-hz 覆盖成精确值。 */
    j->axi_hz = axi_hz ? axi_hz : ((ver >> 8) & 0xFF) * 2000000u;
    j->has_swo = (ver >> 16) >= 2;
    j->swo_cdr = (ver >> 16) >= 3;
    (void)fifo_bits;                       /* BITCNT 10 位才是真上限 */
    j_set_speed(j, speed_khz);
    j->pin_out = PIN_OUT_SRST;             /* SRST 释放 */
    jwr(j, R_PIN_OUT, j->pin_out);
    jwr(j, R_PIN_DIR, 0x1F);
    j_reset_engine(j);
    return 0;
}

static int j_tdo_pin(jtag_t *j)
{
    return (jrd(j, R_PIN_IN) >> 1) & 1;
}

static void j_srst(jtag_t *j, int released)
{
    if (released)
        j->pin_out |= PIN_OUT_SRST;
    else
        j->pin_out &= ~PIN_OUT_SRST;
    jwr(j, R_PIN_OUT, j->pin_out);
}

/* ---- SWO 接收器（IP VERSION >= 2；SWD 模式下 TDO 引脚 = TRACESWO）---- */

/* baud → SWO_DIV 值；超范围返回 -1。
 * v3（边沿 CDR）：一个位 = SWO_DIV+1 个 AXI 周期，任意整数档。
 * v2（16x 过采样）：tick = AXI/(div+1)，16 tick/bit。 */
static uint32_t j_swo_enable(jtag_t *j, uint32_t baud)
{
    if (!j->has_swo) {
        err("bitstream 无 SWO 接收器（IP VERSION < 2），需重刷");
        return 0;
    }
    /* v4 小数-N：精确位周期 = AXI/baud（可能非整数）。
     * 整数部分写 DIV[15:0]，小数量化到 DIV[31:16]（/65536）。
     * v3 IP 写高 16 位无效应为 0 → 自然退化整数模式。
     * 例：4M 档 AXI=125M → 周期 31.25 → DIV=30, FRAC=16384 */
    uint64_t period_x65536 = ((uint64_t)j->axi_hz << 16) / baud;
    uint32_t period_int = (uint32_t)(period_x65536 >> 16);
    uint32_t frac = (uint32_t)(period_x65536 & 0xFFFFull);
    if (period_int < 2) {
        err("SWO %u Hz 超上限 %u Hz", baud, j->axi_hz / 2u);
        return 0;
    }
    if (period_int > 0x10000ull) {
        err("SWO %u Hz 超下限（SWO_DIV 16 位）", baud);
        return 0;
    }
    jwr(j, R_SWO_CTRL, 0);
    jwr(j, R_SWO_DIV, (frac << 16) | (period_int - 1u));
    jwr(j, R_SWO_CTRL, SWO_CTRL_EN);
    /* 实际平均波特率 = AXI / (period_int + frac/65536) */
    return (uint32_t)(((uint64_t)j->axi_hz << 16) / period_x65536);
}

static void j_swo_disable(jtag_t *j)
{
    jwr(j, R_SWO_CTRL, 0);
}

static void j_swo_stat(jtag_t *j, int *cnt, int *ovr, int *fe)
{
    uint32_t v = jrd(j, R_SWO_STAT);
    *cnt = v & 0xFFFF;
    *ovr = !!(v & SWO_STAT_OVR);
    *fe  = !!(v & SWO_STAT_FE);
}

static void j_swo_clear_err(jtag_t *j)
{
    jwr(j, R_SWO_CTRL, SWO_CTRL_EN | SWO_CTRL_CLRERR);
}

/* 读最多 n 字节（n<=0 全读）。整字弹出：剩余空间不足一次 pop 的弹出量时
 * 提前停，字节留在 FIFO 不丢。返回实际读取字节数。 */
static int j_swo_read(jtag_t *j, uint8_t *out, int n)
{
    int len = 0;
    if (n <= 0)
        n = SWO_FIFO_DEPTH;
    while (len < n) {
        int cnt = (int)(jrd(j, R_SWO_STAT) & 0xFFFFu);
        if (cnt == 0)
            break;
        int pop = cnt < 4 ? cnt : 4;
        if (pop > n - len)
            break;
        uint32_t w = jrd(j, R_SWO_FIFO);
        for (int i = 0; i < pop; i++)
            out[len++] = (uint8_t)(w >> (8 * i));
    }
    return len;
}

/* RTI→ShiftDR→移 n 位→Update→RTI；tdi 位填 0 */
static int j_dr_raw(jtag_t *j, int n, uint8_t *cap)
{
    uint8_t tms[MAX_BITS + 8], tdi[MAX_BITS + 8], capall[MAX_BITS + 8];
    int len = 3 + n + 2;
    memset(tdi, 0, sizeof(tdi));
    tms[0] = 1; tms[1] = 0; tms[2] = 0;
    for (int i = 3; i < 3 + n - 1; i++)
        tms[i] = 0;
    tms[3 + n - 1] = 1;                    /* 末位数据 TMS=1 退 Exit1 */
    tms[3 + n] = 1;                        /* Update */
    tms[3 + n + 1] = 0;                    /* RTI */
    if (j_shift(j, tms, tdi, len, capall, 0) < 0)
        return -1;
    memcpy(cap, capall + 3, n);
    return 0;
}

/* 通用链扫描：TLR 后位流逐位解析。返回 idcode 数，*bypass_out 为旁路器件数 */
static int j_scan_chain(jtag_t *j, uint32_t *ids, int id_max, int *bypass_out)
{
    static uint8_t d[512];
    int ids_n = 0, bypass = 0, pos = 0;
    j_goto_tlr(j);
    if (j_dr_raw(j, 512, d) < 0)
        return -1;
    while (pos + 32 <= 512 && ids_n + bypass < 16) {
        if (d[pos]) {
            uint32_t v = 0;
            for (int k = 0; k < 32; k++)
                v |= (uint32_t)d[pos + k] << k;
            if (v == 0xFFFFFFFFu || v == 0) {
                pos++;
                if (pos > 64)
                    break;
                continue;
            }
            if (ids_n < id_max)
                ids[ids_n++] = v;
            pos += 32;
        } else {
            bypass++;
            pos++;
        }
    }
    *bypass_out = bypass;
    return ids_n;
}

static const char *vendor_name(uint32_t id)
{
    switch (id) {
    case 0x049: return "Xilinx";
    case 0x23B: return "ARM Ltd";
    case 0x020: return "STMicroelectronics";
    case 0x015: return "NXP";
    case 0x017: return "Texas Instruments";
    default:    return "?";
    }
}

static void decode_idcode(uint32_t v)
{
    if (!(v & 1)) {
        printf("0x%08X  (非 IDCODE)\n", v);
        return;
    }
    uint32_t mfg = (v >> 1) & 0x7FF;
    printf("0x%08X  ver=%X part=0x%04X mfg=0x%03X(%s)\n",
           v, v >> 28, (v >> 12) & 0xFFFF, mfg, vendor_name(mfg));
}

/* 链拓扑探测（stc12 cmd_chain） */
static void j_chain_probe(jtag_t *j)
{
    uint8_t tms[MAX_BITS + 8], tdi[MAX_BITS + 8], cap[MAX_BITS + 8];

    printf("chain probe:\n");
    /* 1) 全员 BYPASS：TLR→ShiftIR + 64×1 → Update → RTI */
    j_goto_tlr(j);
    memset(tdi, 0, sizeof(tdi));
    for (int i = 0; i < 4; i++) tdi[i] = 0;
    memset(tdi + 4, 1, 64);
    tms[0] = 1; tms[1] = 1; tms[2] = 0; tms[3] = 0;
    memset(tms + 4, 0, 63);
    tms[67] = 1; tms[68] = 1; tms[69] = 0;
    j_shift(j, tms, tdi, 70, cap, 0);

    /* 2) BYPASS 位宽：ShiftDR 冲 64 个 0、注入单 1、数延迟 */
    memset(tdi, 0, sizeof(tdi));
    memset(tms, 0, sizeof(tms));
    tms[0] = 1; tms[1] = 0; tms[2] = 0;         /* RTI→SelDR→CapDR→ShiftDR */
    tdi[3 + 64] = 1;                             /* 冲 0 后注入的 1 */
    for (int i = 3 + 65; i < 3 + 65 + 199; i++)
        tms[i] = 0;
    tms[3 + 65 + 199] = 1;                       /* 末位退出 */
    tms[3 + 65 + 200] = 1;                       /* Update */
    tms[3 + 65 + 201] = 0;                       /* RTI */
    if (j_shift(j, tms, tdi, 3 + 65 + 202, cap, 0) < 0)
        return;
    int delay = -1;
    for (int i = 0; i < 200; i++)
        if (cap[3 + 64 + 1 + i]) { delay = i + 1; break; }
    if (delay < 0) {
        printf("  no response: chain broken or TDO undriven\n");
        return;
    }
    printf("  bypass width: %d bit (%d TAPs)\n", delay, delay);

    /* 3) IR capture：TLR→ShiftIR + 48 位（TDI=1）→ Update → RTI */
    j_goto_tlr(j);
    memset(tdi, 0, sizeof(tdi));
    memset(tdi + 4, 1, 48);
    tms[0] = 1; tms[1] = 1; tms[2] = 0; tms[3] = 0;
    memset(tms + 4, 0, 47);
    tms[51] = 1; tms[52] = 1; tms[53] = 0;
    if (j_shift(j, tms, tdi, 54, cap, 0) < 0)
        return;
    uint8_t *d = cap + 4;
    printf("  IR capture   : ");
    for (int i = 0; i < 6; i++) {
        int v = 0;
        for (int b = 0; b < 8 && i * 8 + b < 48; b++)
            v |= d[i * 8 + b] << b;
        printf("%02X ", v);
    }
    printf("(LSB first)\n");

    int pos1[8], taps = 0, echo = 48;
    for (int i = 0; i < 44 && taps < 8; i++) {
        if (!d[i])
            continue;
        if (i + 1 < 48 && d[i + 1]) { echo = i; break; }
        pos1[taps++] = i;
    }
    if (!taps) {
        printf("  IR scan: no capture pattern (non-std TAP?)\n");
        return;
    }
    for (int i = 0; i < taps; i++) {
        int len = (i + 1 < taps ? pos1[i + 1] : echo) - pos1[i];
        printf("  TAP%d: irlen=%d%s\n", i, len, i ? "" : " (nearest TDO)");
    }
    printf("  total: %d TAPs, IR width = %d bit\n", taps, echo);
}

/* ------------------------------------------------------------------ */
/* Tap                                                                 */
/* ------------------------------------------------------------------ */

typedef struct {
    jtag_t *j;
    int irlen, ir_before, ir_after, dr_before, dr_after;
    int cur_ir, ir_epoch;
} tap_t;

static void tap_init(tap_t *t, jtag_t *j, int irlen, int ir_before, int ir_after,
                     int dr_before, int dr_after)
{
    memset(t, 0, sizeof(*t));
    t->j = j;
    t->irlen = irlen;
    t->ir_before = ir_before;
    t->ir_after = ir_after;
    t->dr_before = dr_before;
    t->dr_after = dr_after;
    t->cur_ir = -1;
}

static int tap_ir(tap_t *t, int value)
{
    if (t->cur_ir == value && t->ir_epoch == t->j->tlr_count)
        return 0;
    uint8_t tms[64], tdi[64], cap[64];
    int n = t->ir_before + t->irlen + t->ir_after;
    int len = 4 + n + 2;
    tms[0] = 1; tms[1] = 1; tms[2] = 0; tms[3] = 0;   /* RTI→ShiftIR */
    memset(tdi, 0, sizeof(tdi));
    for (int i = 0; i < n; i++) {
        tms[4 + i] = (i == n - 1) ? 1 : 0;
        int v;
        if (i < t->ir_before || i >= t->ir_before + t->irlen)
            v = 1;                                    /* 窗口外全 1 = BYPASS */
        else
            v = (value >> (i - t->ir_before)) & 1;
        tdi[4 + i] = v;
    }
    tms[4 + n] = 1;                                   /* Update */
    tms[4 + n + 1] = 0;                               /* RTI */
    if (j_shift(t->j, tms, tdi, len, cap, 0) < 0)
        return -1;
    t->cur_ir = value;
    t->ir_epoch = t->j->tlr_count;
    return 0;
}

/* 移位本 TAP 的 DR（LSB 先）；bits_in 可为 NULL（全 0） */
static int tap_dr(tap_t *t, const uint8_t *bits_in, int n, uint8_t *bits_out)
{
    uint8_t tms[MAX_BITS + 16], tdi[MAX_BITS + 16], cap[MAX_BITS + 16];
    int total = t->dr_before + n + t->dr_after;
    int len = 3 + total + 2;
    memset(tdi, 0, sizeof(tdi));
    tms[0] = 1; tms[1] = 0; tms[2] = 0;
    for (int i = 0; i < total; i++) {
        tms[3 + i] = (i == total - 1) ? 1 : 0;
        int idx = i - t->dr_before;
        if (idx >= 0 && idx < n && bits_in)
            tdi[3 + i] = bits_in[idx];
    }
    tms[3 + total] = 1;
    tms[3 + total + 1] = 0;
    if (j_shift(t->j, tms, tdi, len, cap, 0) < 0)
        return -1;
    if (bits_out)
        memcpy(bits_out, cap + 3 + t->dr_before, n);
    return 0;
}

/* ------------------------------------------------------------------ */
/* SWD（ADIv5；IP SWD 模式下 tms[]=SWDIO 方向、tdi[]=驱动值、cap=pad） */
/* 位级时序照 openocd zynq_dbg 驱动（2026-09-14 板上验证过的那套）      */
/* ------------------------------------------------------------------ */

#define ACK_OK 2
#define ACK_WAIT 1
/* SWD 的 ACK 编码与 JTAG-DP 不同（首位传输位=LSB）：OK=001 WAIT=010 FAULT=100 */
#define SWD_ACK_OK    1
#define SWD_ACK_WAIT  2
#define SWD_ACK_FAULT 4

static const uint8_t SWD_SEQ_J2S[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   /* ≥50 高 = 线复位 */
    0x9e, 0xe7,                                 /* JTAG→SWD 魔数 */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   /* 再线复位（已在 SWD 时也无害） */
    0x00,
};
#define SWD_SEQ_J2S_BITS 136
static const uint8_t SWD_SEQ_S2J[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x3c, 0xe7,                                 /* SWD→JTAG 魔数 */
    0xff,
};
#define SWD_SEQ_S2J_BITS 80
static const uint8_t SWD_SEQ_LR[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00,
};
#define SWD_SEQ_LR_BITS 64

static int swd_send_seq(jtag_t *j, const uint8_t *seq, int nbits)
{
    uint8_t tdi[MAX_BITS + 8], dir[MAX_BITS + 8];
    for (int i = 0; i < nbits; i++) {
        tdi[i] = (seq[i / 8] >> (i % 8)) & 1;
        dir[i] = 1;
    }
    return j_shift(j, dir, tdi, nbits, NULL, 1);
}

static int parity32(uint32_t v)
{
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return v & 1;
}

static int swd_acc_raw_abort(jtag_t *j);

/* 一次 DP/AP 寄存器访问（reg 为字节偏移，A[2:3]=(reg>>2)&3）。
 * 读：数据在本包返回（SWD 与 JTAG-DP 不同，无 RDBUFF 延迟）；
 * WAIT 重试同 JTAG 路径；FAULT 清 sticky 后报错。 */
static int swd_acc(jtag_t *j, int ap, int reg, int rnw, uint32_t wdata,
                   uint32_t *out)
{
    int a = (reg >> 2) & 3;
    int par = (ap ^ rnw ^ (a & 1) ^ ((a >> 1) & 1)) & 1;
    int hdr = 1 | (ap << 1) | (rnw << 2) | (a << 3) | (par << 5) | (1u << 7);
    uint8_t tdi[56], dir[56], cap[56];
    memset(tdi, 0, sizeof(tdi));
    memset(dir, 0, sizeof(dir));
    int n = 0;
    for (int i = 0; i < 8; i++, n++) {          /* 8 位 header（host） */
        tdi[n] = (hdr >> i) & 1;
        dir[n] = 1;
    }
    dir[n] = 0; tdi[n++] = 0;                   /* trn（目标接管） */
    int ack_off = n;
    for (int i = 0; i < 3; i++, n++) {          /* 3 位 ACK（目标） */
        tdi[n] = 0;
        dir[n] = 0;
    }
    int data_off = 0, par_off = 0;
    if (rnw) {
        data_off = n;
        for (int i = 0; i < 32; i++, n++) {     /* 32 位数据（目标） */
            tdi[n] = 0;
            dir[n] = 0;
        }
        par_off = n;
        dir[n++] = 0;                           /* 奇偶（目标） */
        dir[n++] = 0;                           /* trn（host 接管） */
        dir[n++] = 1;                           /* 2 拍 idle */
        dir[n++] = 1;
    } else {
        dir[n++] = 0;                           /* trn（host 接管） */
        for (int i = 0; i < 32; i++, n++) {     /* 32 位数据（host） */
            tdi[n] = (wdata >> i) & 1;
            dir[n] = 1;
        }
        tdi[n] = parity32(wdata);
        dir[n++] = 1;                           /* 奇偶（host） */
        dir[n++] = 1;                           /* 2 拍 idle */
        dir[n++] = 1;
    }
    int faulted = 0;
    for (int t = 0; t < 64; t++) {              /* WAIT 预算同 JTAG 路径 */
        if (j_shift(j, dir, tdi, n, cap, 1) < 0)
            return -1;
        int ack = cap[ack_off] | (cap[ack_off + 1] << 1) |
                  (cap[ack_off + 2] << 2);
        if (ack == SWD_ACK_FAULT && !faulted) {
            faulted = 1;    /* 残留 sticky（line reset 清不掉）：ABORT 后重试一次 */
            swd_acc_raw_abort(j);
            continue;
        }
        if (ack == SWD_ACK_OK) {
            if (rnw) {
                uint32_t v = 0;
                for (int i = 0; i < 32; i++)
                    v |= (uint32_t)cap[data_off + i] << i;
                if (cap[par_off] != parity32(v)) {
                    err("SWD 读奇偶错");
                    return -1;
                }
                if (out)
                    *out = v;
            }
            return 0;
        }
        if (ack != SWD_ACK_WAIT) {
            err("SWD ACK FAULT/非法 (ack=%d)，清 sticky", ack);
            /* 诊断：dump 完整位流（dir=1 host 驱动；cap 为采样） */
            fprintf(stderr, "DIAG n=%d hdr=", n);
            for (int i = 0; i < n; i++)
                fprintf(stderr, "%d%s", tdi[i],
                        (i == 7 || i == 8 || i == 11 || i == 12) ? "|" : "");
            fprintf(stderr, "\nDIAG dir=");
            for (int i = 0; i < n; i++)
                fprintf(stderr, "%d%s", dir[i],
                        (i == 7 || i == 8 || i == 11 || i == 12) ? "|" : "");
            fprintf(stderr, "\nDIAG cap=");
            for (int i = 0; i < n; i++)
                fprintf(stderr, "%d%s", cap[i],
                        (i == 7 || i == 8 || i == 11 || i == 12) ? "|" : "");
            fprintf(stderr, "\n");
            swd_acc_raw_abort(j);
            return -1;
        }
    }
    err("SWD WAIT 超时");
    return -1;
}

/* 写 DP ABORT 寄存器（A=0 写方向）清 sticky/DAPABORT。
 * 必须**裸事务**：不能经 swd_acc——FAULT 时 swd_acc 会调本函数，递归层
 * 各自 faulted=0 又再递归 = 无限递归栈溢出段错误（目标按在复位里必触发，
 * gdb 实锤 SIGSEGV/corrupt stack）。位序列 = swd_acc 写分支同构。 */
static int swd_acc_raw_abort(jtag_t *j)
{
    const uint32_t wdata = 0x0000001Fu;
    const int hdr = 1 | (1u << 7);        /* start|park，AP=0 rnw=0 A=0 par=0 */
    uint8_t tdi[56], dir[56], cap[56];
    memset(tdi, 0, sizeof(tdi));
    memset(dir, 0, sizeof(dir));
    int n = 0;
    for (int i = 0; i < 8; i++, n++) {    /* 8 位 header（host） */
        tdi[n] = (hdr >> i) & 1;
        dir[n] = 1;
    }
    dir[n++] = 0;                         /* trn（与写分支同构） */
    for (int i = 0; i < 32; i++, n++) {
        tdi[n] = (wdata >> i) & 1;
        dir[n] = 1;
    }
    tdi[n] = parity32(wdata);
    dir[n++] = 1;                         /* 奇偶 */
    dir[n++] = 1;                         /* 2 拍 idle */
    dir[n++] = 1;
    return j_shift(j, dir, tdi, n, cap, 1);
}

/* ------------------------------------------------------------------ */
/* Dap（ADIv5；JTAG-DP / SWD 双 transport）                            */
/* ------------------------------------------------------------------ */

#define IR_IDCODE 0xE
#define IR_DPACC  0xA
#define IR_APACC  0xB
#define IR_ABORT  0x8
#define DP_CTRL_STAT 0x4
#define DP_SELECT    0x8
#define DP_RDBUFF    0xC
#define CSYSPWRUP_REQ_ACK 0xA0000000u

typedef struct {
    tap_t tap;
    int swd;                /* 1 = SWD transport（tap_* 不用） */
    int sel_key;            /* (apsel<<4)|bank 缓存，-1 失效 */
} dap_t;

static int id_is_arm_dp(uint32_t v)
{
    return ((v >> 12) & 0xFFFF) == 0xBA00 &&
           ((v >> 1) & 0x7FF) == 0x23B && (v & 1) == 1;
}

/* SWD 建链：JTAG→SWD 魔数 + 线复位 + DPIDR 校验（失败可重试） */
static int swd_connect(jtag_t *j, dap_t *d)
{
    memset(d, 0, sizeof(*d));
    d->swd = 1;
    d->tap.j = j;
    d->sel_key = -1;
    if (swd_send_seq(j, SWD_SEQ_J2S, SWD_SEQ_J2S_BITS) < 0)
        return -1;
    for (int t = 0; t < 5; t++) {
        uint32_t id = 0;
        if (swd_acc(j, 0, 0x0, 1, 0, &id) == 0 && id != 0) {
            printf("SWD-DP found: DPIDR=0x%08X (rev=%u ver=%u "
                   "designer=0x%03X)\n",
                   id, id >> 28, id & 0xF, (id >> 1) & 0x7FF);
            return 0;
        }
        swd_send_seq(j, SWD_SEQ_LR, SWD_SEQ_LR_BITS);
    }
    err("SWD 无响应（DPIDR 读不到）");
    return -1;
}

static int dap_acc(dap_t *d, int ap, int reg, int rnw, uint32_t wdata,
                   uint32_t *out)
{
    if (d->swd)
        return swd_acc(d->tap.j, ap, reg, rnw, wdata, out);
    uint8_t in[35], r[35];
    uint32_t v3 = (((reg >> 2) & 3) << 1) | rnw;
    for (int i = 0; i < 3; i++)
        in[i] = (v3 >> i) & 1;
    for (int i = 0; i < 32; i++)
        in[3 + i] = (wdata >> i) & 1;
    for (int t = 0; t < 64; t++) {          /* WAIT 预算：目标睡眠期别磨分钟级 */
        if (tap_ir(&d->tap, ap ? IR_APACC : IR_DPACC) < 0)
            return -1;
        if (tap_dr(&d->tap, in, 35, r) < 0)
            return -1;
        int ack = r[0] | (r[1] << 1) | (r[2] << 2);
        if (ack == ACK_OK) {
            if (out) {
                uint32_t v = 0;
                for (int i = 0; i < 32; i++)
                    v |= (uint32_t)r[3 + i] << i;
                *out = v;
            }
            return 0;
        }
        if (ack != ACK_WAIT) {
            err("非法 ACK %d", ack);
            return -1;
        }
    }
    err("DAP WAIT 超时");
    return -1;
}

static int dp_read(dap_t *d, int reg, uint32_t *out)
{
    uint32_t v;
    if (dap_acc(d, 0, reg, 1, 0, &v) < 0)
        return -1;
    if (d->swd) {                /* SWD：DP 读数据本包即得，RDBUFF 是旧值 */
        if (out)
            *out = v;
        return 0;
    }
    return dap_acc(d, 0, DP_RDBUFF, 1, 0, out);
}

static int dp_write(dap_t *d, int reg, uint32_t val)
{
    return dap_acc(d, 0, reg, 0, val, NULL);
}

static int ap_route(dap_t *d, int apsel, int bank)
{
    int key = (apsel << 4) | bank;
    if (d->sel_key != key) {
        if (dp_write(d, DP_SELECT, ((uint32_t)apsel << 24) | ((uint32_t)bank << 4)) < 0)
            return -1;
        d->sel_key = key;
    }
    return 0;
}

static int ap_read(dap_t *d, int reg, int apsel, int bank, uint32_t *out)
{
    uint32_t v;
    if (ap_route(d, apsel, bank) < 0)
        return -1;
    if (dap_acc(d, 1, reg, 1, 0, &v) < 0)
        return -1;
    return dap_acc(d, 0, DP_RDBUFF, 1, 0, out);
}

static int ap_write(dap_t *d, int reg, int apsel, int bank, uint32_t val)
{
    if (ap_route(d, apsel, bank) < 0)
        return -1;
    return dap_acc(d, 1, reg, 0, val, NULL);
}

static void dap_abort(dap_t *d)
{
    if (d->swd) {
        dp_write(d, 0x0, 0x0000001Fu);   /* DP ABORT：DAPABORT+清 sticky */
        return;
    }
    uint8_t in[35], out[35];
    memset(in, 0, sizeof(in));
    for (int i = 0; i < 32; i++)
        in[3 + i] = (0x1Fu >> i) & 1;
    if (tap_ir(&d->tap, IR_ABORT) == 0)
        tap_dr(&d->tap, in, 35, out);
}

static int dap_powerup(dap_t *d, uint32_t *cs_out)
{
    uint32_t cs = 0;
    d->sel_key = -1;
    if (dp_write(d, DP_CTRL_STAT, 0x500000F2u) < 0)   /* REQ + W1C 清 sticky */
        return -1;
    if (dp_write(d, DP_CTRL_STAT, 0x50000042u) < 0)
        return -1;
    for (int t = 0; t < 100; t++) {
        if (dp_read(d, DP_CTRL_STAT, &cs) < 0)
            return -1;
        if ((cs & CSYSPWRUP_REQ_ACK) == CSYSPWRUP_REQ_ACK) {
            if (cs_out)
                *cs_out = cs;
            return 0;
        }
    }
    err("调试域上电超时 (CTRL/STAT=0x%08X)", cs);
    return -1;
}

static void dap_check_sticky(dap_t *d)
{
    uint32_t cs;
    if (dp_read(d, DP_CTRL_STAT, &cs) < 0)
        return;
    if (cs & 0xB0u) {
        printf("(sticky err=0x%08X, auto-cleared; earlier reads may be 0)\n", cs);
        dap_abort(d);
        dp_write(d, DP_CTRL_STAT, 0x500000F2u);
        dp_write(d, DP_CTRL_STAT, 0x50000042u);
    }
}

/* 3 拓扑 probe（stc12 cmd_probe） */
static int dap_probe(dap_t *d, jtag_t *j)
{
    static const int lens[5] = { 5, 4, 6, 3, 8 };
    uint8_t in[33], r[33];
    memset(in, 0, sizeof(in));
    for (int side = 0; side < 3; side++) {
        int kmax = (side == 2) ? 1 : 5;
        for (int k = 0; k < kmax; k++) {
            j_goto_tlr(j);
            if (side == 0)
                tap_init(&d->tap, j, 4, 0, lens[k], 0, 1);
            else if (side == 1)
                tap_init(&d->tap, j, 4, lens[k], 0, 1, 0);
            else
                tap_init(&d->tap, j, 4, 0, 0, 0, 0);
            if (tap_ir(&d->tap, IR_IDCODE) < 0)
                return -1;
            if (tap_dr(&d->tap, in, 33, r) < 0)
                return -1;
            uint32_t v = 0;
            for (int i = 0; i < 32; i++)
                v |= (uint32_t)r[i] << i;
            if (side == 1 && !id_is_arm_dp(v))
                v = (v >> 1) | ((uint32_t)r[32] << 31);
            if (id_is_arm_dp(v)) {
                printf("JTAG-DP found: IDCODE=0x%08X (%s, bs irlen=%d)\n", v,
                       side == 0 ? "bypass@TDI" :
                       side == 1 ? "bypass@TDO" : "single TAP",
                       side == 2 ? 0 : lens[k]);
                return 0;
            }
        }
    }
    err("链上未找到 ARM JTAG-DP");
    return -1;
}

static const char *ap_type_name(uint32_t idr)
{
    int cls = (idr >> 13) & 0xF, typ = idr & 0xF;
    if (cls == 0) return typ == 0 ? "JTAG-AP" : "unknown";
    if (cls == 1) return typ == 0 ? "COM-AP" : "unknown";
    if (cls == 8) {
        switch (typ) {
        case 1: return "AHB3-AP";
        case 2: return "APB-AP";
        case 4: return "AXI3/AXI4-AP";
        case 5: return "AHB5-AP";
        case 6: return "APB4-AP";
        case 7: return "AXI5-AP";
        default: return "MEM-AP(?)";
        }
    }
    return "unknown";
}

/* ------------------------------------------------------------------ */
/* MemAp（含 LPAE 64 位地址直通）                                       */
/* ------------------------------------------------------------------ */

#define AP_CSW    0x00
#define AP_TAR    0x04
#define AP_TAR64  0x08
#define AP_DRW    0x0C
#define AP_BASE64 0xF0
#define AP_CFG    0xF4
#define AP_BASE   0xF8
#define AP_IDR    0xFC
#define CFG_LA    0x02
#define CSW_BASE  0xA2000000u

typedef struct {
    dap_t *dap;
    int apsel;
    int size_code;          /* -1 失效 */
    int la;                 /* -1 未探测 */
    uint32_t tar_hi_cache;  /* 已写入 TAR64 的高 32 位 */
    int tar_hi_valid;
} memap_t;

static void memap_init(memap_t *m, dap_t *d, int apsel)
{
    memset(m, 0, sizeof(*m));
    m->dap = d;
    m->apsel = apsel;
    m->size_code = -1;
    m->la = -1;
}

static int memap_long_addr(memap_t *m)
{
    if (m->la < 0) {
        m->la = 0;                      /* 读失败按 32 位降级 */
        uint32_t cfg;
        if (ap_read(m->dap, AP_CFG, m->apsel, 0xF, &cfg) == 0)
            m->la = (cfg & CFG_LA) ? 1 : 0;
    }
    return m->la;
}

static int memap_set_size(memap_t *m, int code)
{
    if (m->size_code != code) {
        if (ap_write(m->dap, AP_CSW, m->apsel, 0, CSW_BASE | (uint32_t)code) < 0)
            return -1;
        m->size_code = code;
    }
    return 0;
}

static int memap_set_tar(memap_t *m, uint64_t addr)
{
    uint32_t hi = (uint32_t)(addr >> 32), lo = (uint32_t)addr;
    if (hi && !memap_long_addr(m)) {
        err("AP%d 非 LPAE，地址超 32 位: 0x%llX", m->apsel,
            (unsigned long long)addr);
        return -1;
    }
    if (memap_long_addr(m) && (!m->tar_hi_valid || m->tar_hi_cache != hi)) {
        if (ap_write(m->dap, AP_TAR64, m->apsel, 0, hi) < 0)
            return -1;
        m->tar_hi_cache = hi;
        m->tar_hi_valid = 1;
    }
    return ap_write(m->dap, AP_TAR, m->apsel, 0, lo);
}

static int mem_read32(memap_t *m, uint64_t addr, uint32_t *out)
{
    if (memap_set_size(m, 2) < 0 || memap_set_tar(m, addr) < 0)
        return -1;
    return ap_read(m->dap, AP_DRW, m->apsel, 0, out);
}

static int mem_write32(memap_t *m, uint64_t addr, uint32_t val)
{
    if (memap_set_size(m, 2) < 0 || memap_set_tar(m, addr) < 0)
        return -1;
    return ap_write(m->dap, AP_DRW, m->apsel, 0, val);
}

static int mem_read16(memap_t *m, uint64_t addr, uint32_t *out)
{
    uint32_t v;
    if (memap_set_size(m, 1) < 0 || memap_set_tar(m, addr) < 0 ||
        ap_read(m->dap, AP_DRW, m->apsel, 0, &v) < 0)
        return -1;
    *out = (v >> ((addr & 2) * 8)) & 0xFFFF;
    return 0;
}

static int mem_write16(memap_t *m, uint64_t addr, uint32_t val)
{
    if (memap_set_size(m, 1) < 0 || memap_set_tar(m, addr) < 0)
        return -1;
    return ap_write(m->dap, AP_DRW, m->apsel, 0, val << ((addr & 2) * 8));
}

static int mem_read8(memap_t *m, uint64_t addr, uint32_t *out)
{
    uint32_t v;
    if (memap_set_size(m, 0) < 0 || memap_set_tar(m, addr) < 0 ||
        ap_read(m->dap, AP_DRW, m->apsel, 0, &v) < 0)
        return -1;
    *out = (v >> ((addr & 3) * 8)) & 0xFF;
    return 0;
}

static int mem_write8(memap_t *m, uint64_t addr, uint32_t val)
{
    if (memap_set_size(m, 0) < 0 || memap_set_tar(m, addr) < 0)
        return -1;
    return ap_write(m->dap, AP_DRW, m->apsel, 0, val << ((addr & 3) * 8));
}

/* ------------------------------------------------------------------ */
/* CortexM                                                             */
/* ------------------------------------------------------------------ */

#define DHCSR   0xE000EDF0u
#define DCRSR   0xE000EDF4u
#define DCRDR   0xE000EDF8u
#define CPUID   0xE000ED00u
/* DBGMCU_CR → target_port.h */
#define DBGKEY  0xA05F0000u
#define S_REGRDY (1u << 16)
#define S_HALT   (1u << 17)
#define S_SLEEP  (1u << 18)
#define S_LOCKUP (1u << 19)

/* FPB 硬件断点 / DWT 观察点 / DFSR（编码对齐 openocd cortex_m.c——同一颗
 * M3 r1p1 上验证过的写法；FP_COMP rev0 的 REPLACE 位域在 [31:30]） */
#define DFSR     0xE000ED30u
#define DEMCR2   0xE000EDFCu
#define FP_CTRL2 0xE0002000u
#define FP_COMP0 0xE0002008u
#define DWT_CTRL2 0xE0001000u
#define DWT_COMP0 0xE0001020u

static const struct { const char *name; int sel; } cm_regs[] = {
    { "r0", 0 }, { "r1", 1 }, { "r2", 2 }, { "r3", 3 }, { "r4", 4 },
    { "r5", 5 }, { "r6", 6 }, { "r7", 7 }, { "r8", 8 }, { "r9", 9 },
    { "r10", 10 }, { "r11", 11 }, { "r12", 12 }, { "sp", 13 }, { "lr", 14 },
    { "pc", 15 }, { "xpsr", 0x10 }, { "msp", 0x11 }, { "psp", 0x12 },
    { "primask", 0x14 }, { "basepri", 0x15 }, { "faultmask", 0x16 },
    { "control", 0x17 },
};

static int cm_lookup(const char *name)
{
    for (unsigned i = 0; i < sizeof(cm_regs) / sizeof(cm_regs[0]); i++)
        if (!strcasecmp(name, cm_regs[i].name))
            return cm_regs[i].sel;
    if (!strcasecmp(name, "r13")) return 13;
    if (!strcasecmp(name, "r14")) return 14;
    if (!strcasecmp(name, "r15")) return 15;
    return -1;
}

static const char *cm_part_name(uint32_t id)
{
    switch ((id >> 4) & 0xFFF) {
    case 0xC20: return "M0";
    case 0xC21: return "M1";
    case 0xC23: return "M3";
    case 0xC24: return "M4";
    case 0xC27: return "M7";
    case 0xC60: return "M0+";
    case 0xD20: return "M23";
    case 0xD21: return "M33";
    case 0xD22: return "M55";
    default:    return "?";
    }
}

static int cm_valid_id(uint32_t id)
{
    uint32_t impl = (id >> 24) & 0xFF;
    return impl == 0x41 && cm_part_name(id)[0] != '?';
}

static int cm_detect(memap_t *m, uint32_t *cpuid)
{
    if (mem_read32(m, CPUID, cpuid) < 0)
        return 0;
    return cm_valid_id(*cpuid);
}

/* ------------------------------------------------------------------ */
/* CoreSight（dapinfo）                                                */
/* ------------------------------------------------------------------ */

static const struct { uint32_t d, p; const char *n; } cs_parts[] = {
    { 0x23B, 0x000, "Cortex-M3 SCS" },   { 0x23B, 0x001, "Cortex-M3 ITM" },
    { 0x23B, 0x002, "Cortex-M3 DWT" },   { 0x23B, 0x003, "Cortex-M3 FPB" },
    { 0x23B, 0x912, "Cortex-M3 ETM" },   { 0x23B, 0x923, "Cortex-M3 TPIU" },
    { 0x23B, 0x924, "Cortex-M4 TPIU" },  { 0x23B, 0x4A13, "Cortex-M4 SCS" },
};

static const char *cs_part_name(uint32_t designer, uint32_t part)
{
    for (unsigned i = 0; i < sizeof(cs_parts) / sizeof(cs_parts[0]); i++)
        if (cs_parts[i].p == part &&
            (cs_parts[i].d & 0x7F) == (designer & 0x7F))
            return cs_parts[i].n;
    return "未识别";
}

static const char *cs_class_name(int k)
{
    switch (k) {
    case 0x1: return "ROM table";
    case 0x9: return "CoreSight component";
    case 0xE: return "Generic IP component";
    case 0xF: return "PrimeCell peripheral";
    default:  return "?";
    }
}

static const char *cs_dtype_name(int t)
{
    switch (t) {
    case 0x11: return "Trace Sink, Port";
    case 0x12: return "Trace Sink, Buffer";
    case 0x13: return "Trace Link";
    case 0x14: return "Debug Ctrl, Trigger";
    case 0x15: return "Debug Logic";
    case 0x21: return "Trace Source, CPU";
    default:   return "未知";
    }
}

static int cs_count;                       /* 组件数护栏 */

static void cs_indent(int depth)
{
    for (int i = 0; i < depth * 2; i++)
        putchar(' ');
}

static void cs_describe(memap_t *m, uint32_t comp, int depth)
{
    uint32_t pid = 0, cid = 0, v;
    if (depth > 3 || cs_count >= 16)
        return;
    cs_count++;
    cs_indent(depth);
    printf("Component base address 0x%08X\n", comp);
    for (int i = 0; i < 4; i++) {
        if (mem_read32(m, comp + 0xFE0 + 4 * i, &v) < 0)
            return;
        pid |= (v & 0xFF) << (8 * i);
    }
    for (int i = 0; i < 4; i++) {
        if (mem_read32(m, comp + 0xFF0 + 4 * i, &v) < 0)
            return;
        cid |= (v & 0xFF) << (8 * i);
    }
    if ((cid & 0xFFFF0FFF) != 0xB105000Du) {
        cs_indent(depth);
        printf("  Invalid CID 0x%08X\n", cid);
        return;
    }
    uint32_t part = pid & 0xFFF;
    uint32_t designer = ((pid >> 25) & 0x780) | ((pid >> 12) & 0x7F);
    int klass = (cid >> 12) & 0xF;
    cs_indent(depth);
    printf("  Peripheral ID 0x%08X\n", pid);
    cs_indent(depth);
    printf("  Designer is 0x%03X, %s\n", designer, vendor_name(designer));
    cs_indent(depth);
    printf("  Part is 0x%03X, %s\n", part, cs_part_name(designer, part));
    cs_indent(depth);
    printf("  Component class is 0x%X, %s\n", klass, cs_class_name(klass));
    if (klass == 1) {
        if (mem_read32(m, comp + 0xFCC, &v) == 0 && (v & 1)) {
            cs_indent(depth);
            printf("  MEMTYPE system memory present on bus\n");
        }
        for (int off = 0; off <= 240; off += 4) {
            if (mem_read32(m, comp + off, &v) < 0)
                return;
            cs_indent(depth);
            printf("  ROMTABLE[0x%02X] = 0x%08X\n", off, v);
            if (v == 0) {
                cs_indent(depth);
                printf("  End of ROM table\n");
                return;
            }
            if (!(v & 1)) {
                cs_indent(depth);
                printf("    Component not present\n");
                continue;
            }
            int32_t rel = (int32_t)(v >> 12);
            if (rel & 0x80000)
                rel -= 0x100000;
            cs_describe(m, comp + ((uint32_t)rel << 12), depth + 1);
        }
    } else if (klass == 9) {
        if (mem_read32(m, comp + 0xFCC, &v) == 0) {
            cs_indent(depth);
            printf("  Type is 0x%02X, %s\n", v & 0xFF, cs_dtype_name(v & 0xFF));
        }
    }
}

static void cs_info(dap_t *d, int apsel)
{
    memap_t m;
    uint32_t idr, base;
    memap_init(&m, d, apsel);
    if (ap_read(d, AP_IDR, apsel, 0xF, &idr) < 0)
        return;
    printf("AP # 0x%X\n", apsel);
    printf("    AP ID register 0x%08X\n", idr);
    printf("    Type is %s\n", ap_type_name(idr));
    if (ap_read(d, AP_BASE, apsel, 0xF, &base) < 0)
        return;
    if (memap_long_addr(&m)) {
        uint32_t hi = 0;
        if (ap_read(d, AP_BASE64, apsel, 0xF, &hi) == 0 && hi)
            printf("MEM-AP BASE 0x%08X%08X\n", hi, base);
        else
            printf("MEM-AP BASE 0x%08X\n", base);
    } else {
        printf("MEM-AP BASE 0x%08X\n", base);
    }
    if (!(base & 1)) {
        printf("    ROM table not present\n");
        return;
    }
    printf("    Valid ROM table present\n");
    cs_count = 0;
    cs_describe(&m, base & ~0xFFFu, 1);
}

/* ------------------------------------------------------------------ */
/* ScanDump（Arise2 私有人格）                                          */
/* ------------------------------------------------------------------ */

#define SD_IR_LEN 4
#define SD_INSTR_IDCODE        0x2
#define SD_INSTR_CHAIN_SELECT  0x3
#define SD_INSTR_DEBUG         0x8
#define SD_INSTR_BYPASS        0xF
#define SD_CHAIN_CFG      0
#define SD_CHAIN_SCAN_IN  1
#define SD_CHAIN_SCAN_OUT 2
#define SD_CFG_MODE    0
#define SD_CFG_TRIGGER 1
#define SD_CFG_ENABLE  2
#define SD_CFG0_AUTO_1CLK   0x10
#define SD_CFG0_MANUAL      0x20
#define SD_CFG0_BURST_SHIFT 2
#define SD_CFG2_DUMP       0x01
#define SD_CFG2_MIU_SR     0x02
#define SD_CFG2_MIU_PADDET 0x04
#define SD_CFG2_BIU_RST    0x08
#define SD_CFG2_PERI_RST   0x10
#define SD_CFG2_MIU_LOCK   0x20
#define SD_IDCODE_EXPECTED 0xAABBCCDDu
#define SD_SCAN_OUT_BITS   119

typedef struct {
    jtag_t *j;
    int fmt_bit;
} sd_t;

/* RTI→ShiftDR→移 n 位→Update→RTI（TDI=bits，可为 NULL=全 0；返回捕获） */
static int sd_dr(sd_t *sd, const uint8_t *bits, int n, uint8_t *out)
{
    uint8_t tms[MAX_BITS + 8], tdi[MAX_BITS + 8], cap[MAX_BITS + 8];
    int len = 3 + n + 2;
    memset(tdi, 0, sizeof(tdi));
    if (bits)
        memcpy(tdi + 3, bits, n);
    tms[0] = 1; tms[1] = 0; tms[2] = 0;
    memset(tms + 3, 0, n - 1);
    tms[3 + n - 1] = 1;
    tms[3 + n] = 1;
    tms[3 + n + 1] = 0;
    if (j_shift(sd->j, tms, tdi, len, cap, 0) < 0)
        return -1;
    if (out)
        memcpy(out, cap + 3, n);
    return 0;
}

static int sd_ir_load(sd_t *sd, int instr)
{
    uint8_t tms[16], tdi[16], cap[16];
    tms[0] = 1; tms[1] = 1; tms[2] = 0; tms[3] = 0;
    memset(tdi, 0, sizeof(tdi));
    for (int i = 0; i < SD_IR_LEN; i++) {
        tms[4 + i] = (i == SD_IR_LEN - 1) ? 1 : 0;
        tdi[4 + i] = (instr >> i) & 1;
    }
    tms[4 + SD_IR_LEN] = 1;
    tms[4 + SD_IR_LEN + 1] = 0;
    if (j_shift(sd->j, tms, tdi, 4 + SD_IR_LEN + 2, cap, 0) < 0)
        return -1;
    sd->j->tlr_count++;              /* IR 被外部改动 → ARM Tap 缓存失效 */
    return 0;
}

static int sd_path_setup(sd_t *sd, int chain)
{
    uint8_t c[4];
    if (sd_ir_load(sd, SD_INSTR_CHAIN_SELECT) < 0)
        return -1;
    for (int i = 0; i < 4; i++)
        c[i] = (chain >> i) & 1;
    if (sd_dr(sd, c, 4, NULL) < 0)
        return -1;
    return sd_ir_load(sd, SD_INSTR_DEBUG);
}

static uint32_t sd_cfg_frame(int rw, int addr, int data)
{
    return ((uint32_t)(data & 0xFF) << 6) | ((uint32_t)(rw & 1) << 5) |
           (uint32_t)(addr & 0x1F);
}

static int sd_cfg_write(sd_t *sd, int reg, int data)
{
    uint8_t bits[14];
    uint32_t f = sd_cfg_frame(1, reg, data);
    if (sd_path_setup(sd, SD_CHAIN_CFG) < 0)
        return -1;
    for (int i = 0; i < 14; i++)
        bits[i] = (f >> i) & 1;
    return sd_dr(sd, bits, 14, NULL);
}

static int sd_cfg_read(sd_t *sd, int reg, uint8_t *val)
{
    uint8_t bits[14], raw[14];
    uint32_t f = sd_cfg_frame(0, reg, 0);
    if (sd_path_setup(sd, SD_CHAIN_CFG) < 0)
        return -1;
    for (int i = 0; i < 14; i++)
        bits[i] = (f >> i) & 1;
    if (sd_dr(sd, bits, 14, raw) < 0)     /* 残影帧 */
        return -1;
    if (sd_dr(sd, bits, 14, raw) < 0)     /* 当前值帧 */
        return -1;
    *val = 0;
    for (int i = 0; i < 8; i++)
        *val |= raw[6 + i] << i;
    return 0;
}

static int sd_scan_in_write(sd_t *sd, uint32_t w0, uint32_t w1)
{
    uint8_t bits[64];
    if (sd_path_setup(sd, SD_CHAIN_SCAN_IN) < 0)
        return -1;
    for (int i = 0; i < 32; i++) {
        bits[i] = (w0 >> i) & 1;
        bits[32 + i] = (w1 >> i) & 1;
    }
    return sd_dr(sd, bits, 64, NULL);
}

static void sd_print_row(sd_t *sd, int row, const uint8_t *bits)
{
    printf("%05d:", row);
    if (sd->fmt_bit) {
        for (int i = 0; i < SD_SCAN_OUT_BITS; i++)
            putchar('0' + bits[i]);
    } else {
        uint8_t b[(SD_SCAN_OUT_BITS + 7) / 8];
        memset(b, 0, sizeof(b));
        for (int i = 0; i < SD_SCAN_OUT_BITS; i++)
            b[i >> 3] |= bits[i] << (i & 7);
        for (int i = (SD_SCAN_OUT_BITS + 7) / 8 - 1; i >= 0; i--)
            printf("%02X", b[i]);
    }
    putchar('\n');
}

static int sd_read_idcode(sd_t *sd, uint32_t *id)
{
    uint8_t r[32];
    if (sd_ir_load(sd, SD_INSTR_IDCODE) < 0)
        return -1;
    memset(r, 0, sizeof(r));
    /* 移 32 位（TDI=0） */
    {
        uint8_t tms[40], tdi[40], cap[40];
        tms[0] = 1; tms[1] = 0; tms[2] = 0;
        memset(tdi, 0, sizeof(tdi));
        for (int i = 3; i < 3 + 31; i++) tms[i] = 0;
        tms[34] = 1; tms[35] = 1; tms[36] = 0;
        if (j_shift(sd->j, tms, tdi, 37, cap, 0) < 0)
            return -1;
        *id = 0;
        for (int i = 0; i < 32; i++)
            *id |= (uint32_t)cap[3 + i] << i;
    }
    return *id == SD_IDCODE_EXPECTED;
}

static int sd_test_bypass(sd_t *sd, uint32_t *echo)
{
    uint8_t bits[16], r[16];
    if (sd_ir_load(sd, SD_INSTR_BYPASS) < 0)
        return -1;
    for (int i = 0; i < 16; i++)
        bits[i] = (0xAAAAu >> i) & 1;
    if (sd_dr(sd, bits, 16, r) < 0)
        return -1;
    *echo = 0;
    for (int i = 0; i < 16; i++)
        *echo |= (uint32_t)r[i] << i;
    return *echo == 0x5554u;
}

/* ------------------------------------------------------------------ */
/* Shell                                                               */
/* ------------------------------------------------------------------ */

typedef struct {
    jtag_t jtag;
    dap_t dap;
    int dap_ok;
    memap_t mem;
    int mem_ok;
    int apsel;
    int cm_checked, cm_ok;
    uint32_t swo_baud;   /* 最近一次 SWO RX 使能波特率（0=关）——serve 模式
                            在 SWO 客户端接入时按此重使能（0→1 沿 flush） */
    int dbgmcu_done;     /* DBGMCU_CR=0x27 已写成功（睡眠调试，写一次即可） */
    int swd;             /* 1 = SWD transport（SWO 必需：SWJ_CFG 会关 JTAG） */
    sd_t sd;
} shell_t;

static void sh_invalidate_arm(shell_t *sh)
{
    sh->dap_ok = 0;
    sh->mem_ok = 0;
    sh->cm_checked = 0;
    sh->cm_ok = 0;
}

static dap_t *sh_dap(shell_t *sh)
{
    if (!sh->dap_ok) {
        if (sh->swd ? swd_connect(&sh->jtag, &sh->dap) < 0
                    : dap_probe(&sh->dap, &sh->jtag) < 0)
            return NULL;
        dp_write(&sh->dap, 0x0, 0x0000001Eu);  /* 对齐 openocd：DPIDR 后先清 sticky（ABORT W1C 位） */
        uint32_t cs;
        if (dap_powerup(&sh->dap, &cs) < 0)
            return NULL;
        printf("DAP OK: CTRL/STAT=0x%08X\n", cs);
        sh->dap_ok = 1;
        sh->mem_ok = 0;
    }
    return &sh->dap;
}

static memap_t *sh_mem(shell_t *sh)
{
    if (!sh->mem_ok || sh->mem.apsel != sh->apsel) {
        dap_t *d = sh_dap(sh);
        if (!d)
            return NULL;
        memap_init(&sh->mem, d, sh->apsel);
        sh->mem_ok = 1;
    }
    return &sh->mem;
}

static int sh_halted(shell_t *sh)
{
    uint32_t dhcsr;
    if (mem_read32(sh_mem(sh), DHCSR, &dhcsr) < 0)
        return -1;
    return (dhcsr & S_HALT) ? 1 : 0;
}

/* CPUID 门卫（stc12 cm_ready）：返回 1 = 当前 AP 后是 Cortex-M */
static int sh_cm_ready(shell_t *sh, const char *cmd)
{
    memap_t *m = sh_mem(sh);
    if (!m)
        return 0;
    if (!sh->cm_checked) {
        uint32_t cpuid = 0;
        sh->cm_checked = 1;
        sh->cm_ok = cm_detect(m, &cpuid);
        if (sh->cm_ok)
            printf("Cortex-%s CPUID=0x%08X via AP%d\n",
                   cm_part_name(cpuid), cpuid, sh->apsel);
        else
            dap_check_sticky(&sh->dap);
    }
    if (!sh->cm_ok) {
        err("%s: AP%d 后没有 Cortex-M 核（'ap <n>' 换域后重试）", cmd, sh->apsel);
        return 0;
    }
    return 1;
}

static int cm_reg_read(memap_t *m, int sel, uint32_t *out)
{
    uint32_t d;
    if (mem_write32(m, DCRSR, (uint32_t)sel) < 0)
        return -1;
    for (int t = 0; t < 100; t++) {
        if (mem_read32(m, DHCSR, &d) < 0)
            return -1;
        if (d & S_REGRDY)
            return mem_read32(m, DCRDR, out);
    }
    err("寄存器读超时 (REGSEL=0x%X)", sel);
    return -1;
}

static int cm_reg_write(memap_t *m, int sel, uint32_t val)
{
    uint32_t d;
    if (mem_write32(m, DCRDR, val) < 0)
        return -1;
    if (mem_write32(m, DCRSR, (uint32_t)sel | 0x10000u) < 0)
        return -1;
    for (int t = 0; t < 100; t++) {
        if (mem_read32(m, DHCSR, &d) < 0)
            return -1;
        if (d & S_REGRDY)
            return 0;
    }
    err("寄存器写超时 (REGSEL=0x%X)", sel);
    return -1;
}

static int cm_halt(memap_t *m, uint32_t *dhcsr)
{
    uint32_t d;
    if (mem_write32(m, DHCSR, DBGKEY | 0x3u) < 0)
        return -1;
    for (int t = 0; t < 100; t++) {
        if (mem_read32(m, DHCSR, &d) < 0)
            return -1;
        if (d & S_HALT) {
            if (dhcsr)
                *dhcsr = d;
            return 0;
        }
    }
    err("halt 超时");
    return -1;
}

static void cmd_scan(shell_t *sh, int argc, char **argv);
static void cmd_probe(shell_t *sh, int argc, char **argv);
static void cmd_chain(shell_t *sh, int argc, char **argv);
static void cmd_help(shell_t *sh, int argc, char **argv);

static void cmd_scan(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    uint32_t ids[16];
    int bypass = 0, n = j_scan_chain(&sh->jtag, ids, 16, &bypass);
    if (n < 0)
        return;
    if (!n && !bypass) {
        printf("no devices found (TDO all 0s? check chain)\n");
        return;
    }
    for (int i = 0; i < n; i++) {
        printf("#%d IDCODE=", i + 1);
        decode_idcode(ids[i]);
    }
    for (int i = 0; i < bypass; i++)
        printf("#%d BYPASS device (1 bit, no IDCODE)\n", n + i + 1);
    printf("%d IDCODE + %d bypass device(s)\n", n, bypass);
}

static void cmd_probe(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    if (sh_dap(sh))
        printf("mem/AP cmds ready; 'aps' + 'ap <n>' to select domain\n");
}

/* swd：切 SWD transport（SWO 必需——swo_tpiu 写 SWJ_CFG=010 会关目标
 * JTAG 口，JTAG transport 随即失联；SWD 下 SWCLK/SWDIO 不受影响） */
static void cmd_swd(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    sh->swd = 1;
    sh_invalidate_arm(sh);
    sh->dbgmcu_done = 0;
    if (sh_dap(sh))
        printf("SWD 模式；mem/AP cmds ready\n");
}

static void cmd_jtag(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    sh->swd = 0;
    sh_invalidate_arm(sh);
    sh->dbgmcu_done = 0;
    swd_send_seq(&sh->jtag, SWD_SEQ_S2J, SWD_SEQ_S2J_BITS);
    if (sh_dap(sh))
        printf("JTAG 模式；mem/AP cmds ready\n");
}

/* swd_dbg：dump 一次 DPIDR 读的原始 48 位（调 SWD 位级对齐用） */
static void cmd_swd_dbg(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    jtag_t *j = &sh->jtag;
    int hdr = 0xA5;                    /* DPIDR 读标准 header */
    uint8_t tdi[56], dir[56], cap[56];
    memset(tdi, 0, sizeof(tdi));
    memset(dir, 0, sizeof(dir));
    int n = 0;
    for (int i = 0; i < 8; i++, n++) {
        tdi[n] = (hdr >> i) & 1;
        dir[n] = 1;
    }
    dir[n++] = 0;                      /* trn */
    for (int i = 0; i < 3; i++, n++)
        dir[n] = 0;                    /* ack */
    for (int i = 0; i < 32; i++, n++)
        dir[n] = 0;                    /* data */
    dir[n++] = 0;                      /* par */
    dir[n++] = 0;                      /* trn */
    dir[n++] = 1; dir[n++] = 1;        /* idle */
    if (j_shift(j, dir, tdi, n, cap, 1) < 0)
        return;
    printf("captured %d bits:\n", n);
    for (int i = 0; i < n; i++)
        printf("%d ", cap[i]);
    printf("\nack@9..11 = %d%d%d  ", cap[9], cap[10], cap[11]);
    uint32_t v = 0;
    for (int i = 0; i < 32; i++)
        v |= (uint32_t)cap[12 + i] << i;
    printf("data@12..43 = 0x%08X  par@44 = %d\n", v, cap[44]);
}


static void cmd_chain(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    j_chain_probe(&sh->jtag);
}

static void cmd_aps(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    dap_t *d = sh_dap(sh);
    if (!d)
        return;
    int found = 0;
    for (int apsel = 0; apsel < 8; apsel++) {
        uint32_t idr;
        if (ap_read(d, AP_IDR, apsel, 0xF, &idr) == 0 && idr) {
            printf("APSEL %3d: IDR=0x%08X  %s  designer=0x%03X  rev=%X var=%X\n",
                   apsel, idr, ap_type_name(idr), (idr >> 17) & 0x3FF,
                   (idr >> 28) & 0xF, (idr >> 4) & 0xF);
            found++;
        }
    }
    if (!found)
        printf("no AP found\n");
    dap_check_sticky(d);
}

static void cmd_ap(shell_t *sh, int argc, char **argv)
{
    if (argc >= 2) {
        sh->apsel = (int)parse_num(argv[1]);
        sh->mem_ok = 0;
        sh->cm_checked = 0;
        sh->cm_ok = 0;
    }
    printf("默认 APSEL = %d\n", sh->apsel);
}

static void cmd_dapinfo(shell_t *sh, int argc, char **argv)
{
    dap_t *d = sh_dap(sh);
    if (!d)
        return;
    int apsel = argc >= 2 ? (int)parse_num(argv[1]) : sh->apsel;
    cs_info(d, apsel);
    dap_check_sticky(d);
}

static void cmd_speed(shell_t *sh, int argc, char **argv)
{
    if (argc >= 2)
        j_set_speed(&sh->jtag, (int)parse_num(argv[1]));
    printf("TCK = %u kHz\n", sh->jtag.axi_hz / (2u * (sh->jtag.clkdiv + 1)) / 1000u);
}

static void mem_cmd(shell_t *sh, int argc, char **argv, int size, int writes)
{
    if (argc < 2) {
        err("用法: %s ADDR %s", argv[0], writes ? "V..." : "[COUNT]");
        return;
    }
    memap_t *m = sh_mem(sh);
    if (!m) {
        err("DAP 未连接（先 probe / swd）");
        return;
    }
    uint64_t addr = parse_num(argv[1]);
    if (writes) {
        if (addr % size) {
            err("地址必须 %d 字节对齐", size);
            return;
        }
        for (int i = 2; i < argc; i++) {
            uint32_t v = (uint32_t)parse_num(argv[i]);
            int r = size == 4 ? mem_write32(m, addr, v) :
                    size == 2 ? mem_write16(m, addr, v) :
                                mem_write8(m, addr, v);
            if (r < 0)
                return;
            addr += size;
        }
    } else {
        int count = argc >= 3 ? (int)parse_num(argv[2]) : 1;
        int per_line = size == 1 ? 16 : 8;
        for (int i = 0; i < count; i++) {
            uint64_t a = addr + (uint64_t)size * i;
            uint32_t v;
            int r = size == 4 ? mem_read32(m, a, &v) :
                    size == 2 ? mem_read16(m, a, &v) :
                                mem_read8(m, a, &v);
            if (r < 0)
                return;
            if (i % per_line == 0)
                printf("0x%08llX:", (unsigned long long)a);
            printf(" %0*X", size * 2, v);
            if (i % per_line == per_line - 1 || i + 1 == count)
                putchar('\n');
        }
    }
    if (sh->dap_ok)
        dap_check_sticky(&sh->dap);
}

static void cmd_mdw(shell_t *sh, int argc, char **argv) { mem_cmd(sh, argc, argv, 4, 0); }
static void cmd_mdh(shell_t *sh, int argc, char **argv) { mem_cmd(sh, argc, argv, 2, 0); }
static void cmd_mdb(shell_t *sh, int argc, char **argv) { mem_cmd(sh, argc, argv, 1, 0); }
static void cmd_mww(shell_t *sh, int argc, char **argv) { mem_cmd(sh, argc, argv, 4, 1); }
static void cmd_mwh(shell_t *sh, int argc, char **argv) { mem_cmd(sh, argc, argv, 2, 1); }
static void cmd_mwb(shell_t *sh, int argc, char **argv) { mem_cmd(sh, argc, argv, 1, 1); }

static void cmd_dpr(shell_t *sh, int argc, char **argv)
{
    if (argc < 2) {
        err("用法: dpr <reg>");
        return;
    }
    uint32_t v;
    dap_t *d = sh_dap(sh);
    if (!d)
        return;
    if (dp_read(d, (int)parse_num(argv[1]), &v) == 0)
        printf("DP[0x%X] = 0x%08X\n", (uint32_t)parse_num(argv[1]), v);
    dap_check_sticky(d);
}

static void cmd_dpw(shell_t *sh, int argc, char **argv)
{
    if (argc < 3) {
        err("用法: dpw <reg> <val>");
        return;
    }
    dap_t *d = sh_dap(sh);
    if (!d)
        return;
    int reg = (int)parse_num(argv[1]);
    uint32_t val = (uint32_t)parse_num(argv[2]);
    if (dp_write(d, reg, val) == 0) {
        if (reg == DP_SELECT)
            d->sel_key = -1;
        printf("DP[0x%X] <- 0x%08X\n", reg, val);
    }
    dap_check_sticky(d);
}

static void cmd_apr(shell_t *sh, int argc, char **argv)
{
    if (argc < 3) {
        err("用法: apr <bank> <reg> [apsel]");
        return;
    }
    dap_t *d = sh_dap(sh);
    if (!d)
        return;
    int apsel = argc >= 4 ? (int)parse_num(argv[3]) : sh->apsel;
    uint32_t v;
    if (ap_read(d, (int)parse_num(argv[2]), apsel, (int)parse_num(argv[1]), &v) == 0)
        printf("AP[%d] bank 0x%X reg 0x%X = 0x%08X\n", apsel,
               (uint32_t)parse_num(argv[1]), (uint32_t)parse_num(argv[2]), v);
    dap_check_sticky(d);
}

static void cmd_apw(shell_t *sh, int argc, char **argv)
{
    if (argc < 4) {
        err("用法: apw <bank> <reg> <val> [apsel]");
        return;
    }
    dap_t *d = sh_dap(sh);
    if (!d)
        return;
    int apsel = argc >= 5 ? (int)parse_num(argv[4]) : sh->apsel;
    if (ap_write(d, (int)parse_num(argv[2]), apsel, (int)parse_num(argv[1]),
                 (uint32_t)parse_num(argv[3])) == 0)
        printf("AP[%d] bank 0x%X reg 0x%X <- 0x%08X\n", apsel,
               (uint32_t)parse_num(argv[1]), (uint32_t)parse_num(argv[2]),
               (uint32_t)parse_num(argv[3]));
    dap_check_sticky(d);
}

static void cmd_halt(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    if (!sh_cm_ready(sh, "halt"))
        return;
    memap_t *m = sh_mem(sh);
    uint32_t dhcsr;
    if (cm_halt(m, &dhcsr) == 0) {
        uint32_t pc;
        if (cm_reg_read(m, 15, &pc) == 0)
            printf("halted: pc=0x%08X\n", pc);
    }
}

static void cmd_resume(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    if (!sh_cm_ready(sh, "resume"))
        return;
    if (mem_write32(sh_mem(sh), DHCSR, DBGKEY | 0x1u) == 0)
        printf("resumed\n");
}

static void cmd_step(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    if (!sh_cm_ready(sh, "step"))
        return;
    memap_t *m = sh_mem(sh);
    uint32_t d;
    if (mem_write32(m, DHCSR, DBGKEY | 0x5u) < 0)
        return;
    for (int t = 0; t < 100; t++) {
        if (mem_read32(m, DHCSR, &d) < 0)
            return;
        if (d & S_HALT)
            break;
    }
    uint32_t pc;
    if (cm_reg_read(m, 15, &pc) == 0)
        printf("pc=0x%08X\n", pc);
}

static void cmd_reg(shell_t *sh, int argc, char **argv)
{
    if (!sh_cm_ready(sh, "reg"))
        return;
    memap_t *m = sh_mem(sh);
    int h = sh_halted(sh);
    if (h < 0)
        return;
    if (!h) {
        printf("target is running\n");
        return;
    }
    if (argc >= 3) {                     /* reg <name> <val> 写 */
        int sel = cm_lookup(argv[1]);
        if (sel < 0) {
            err("未知寄存器 %s", argv[1]);
            return;
        }
        uint32_t val = (uint32_t)parse_num(argv[2]);
        if (cm_reg_write(m, sel, val) == 0)
            printf("%s <- 0x%08X\n", argv[1], val);
    } else if (argc >= 2) {
        int sel = cm_lookup(argv[1]);
        if (sel < 0) {
            err("未知寄存器 %s", argv[1]);
            return;
        }
        uint32_t v;
        if (cm_reg_read(m, sel, &v) == 0)
            printf("%s = 0x%08X\n", argv[1], v);
    } else {
        unsigned n = sizeof(cm_regs) / sizeof(cm_regs[0]);
        for (unsigned i = 0; i < n; i++) {
            uint32_t v;
            if (cm_reg_read(m, cm_regs[i].sel, &v) < 0) {
                printf("\nabort at %s\n", cm_regs[i].name);
                return;
            }
            printf("%s = 0x%08X\n", cm_regs[i].name, v);
        }
    }
}

/* ---- run-control / FPB 断点 / DWT 观察点核心（shell 与二进制协议共用；
 * FP_COMP rev0 编码：[31:30] REPLACE（1=低半字 2=高半字 3=全字）---- */
/* DBGMCU_CR=DBG_SLEEP|STOP|STANDBY|TRACE_IOEN：固件 WFI 期间内核时钟不
 * 停，AP 访问才不会全 WAIT。目标睡着时写入失败（~50ms 预算内），每轮
 * haltinfo 顺带重试，目标醒来的瞬间落地，一次成功终身有效。 */
static void ensure_dbgmcu(shell_t *sh)
{
    if (sh->dbgmcu_done || !sh->dap_ok)
        return;
#if TGT_HAS_DBGMCU
    if (mem_write32(sh_mem(sh), TGT_DBGMCU_ADDR, TGT_DBGMCU_VALUE) == 0)
        sh->dbgmcu_done = 1;
    else
        dap_check_sticky(&sh->dap);
#else
    sh->dbgmcu_done = 1;  /* 无 DBGMCU 的芯片直接标记完成 */
#endif
}

/* 全量重连 target：清 shell 粘滞标志（下次访问重建 SWD 连接/DP 上电/
 * AP memap/核检测/DBGMCU）+ 复位 SWD 引擎。幂等，~百 ms。
 * 场景：目标复位/掉电后链路卡死——命令报 eio，重探即恢复 */
static void core_reprobe(shell_t *sh)
{
    sh->dap_ok = 0;
    sh->mem_ok = 0;
    sh->cm_checked = 0;
    sh->cm_ok = 0;
    sh->dbgmcu_done = 0;
    j_reset_engine(&sh->jtag);
}

#define HI_RUN   0
#define HI_HALT  1
enum { HR_NONE = 0, HR_HALREQ, HR_BKPT, HR_DWTTRAP, HR_VCATCH, HR_EXTERNAL };

static int core_haltinfo(shell_t *sh, int *state, int *reason, uint32_t *pc)
{
    if (!sh_cm_ready(sh, "haltinfo"))
        return -1;
    memap_t *m = sh_mem(sh);
    uint32_t dhcsr, dfsr = 0;
    if (mem_read32(m, DHCSR, &dhcsr) < 0)
        return -1;
    mem_read32(m, DFSR, &dfsr);      /* 读失败按 0 */
    mem_write32(m, DFSR, dfsr);      /* DFSR 写 1 清除：每次轮询只看新事件 */
    *reason = HR_NONE;
    if (dfsr & (1u << 1))
        *reason = HR_BKPT;
    else if (dfsr & (1u << 2))
        *reason = HR_DWTTRAP;
    else if (dfsr & (1u << 3))
        *reason = HR_VCATCH;
    else if (dfsr & (1u << 4))
        *reason = HR_EXTERNAL;
    else if (dfsr & (1u << 0))
        *reason = HR_HALREQ;
    if (dhcsr & S_HALT) {
        *state = HI_HALT;
        uint32_t v = 0;
        cm_reg_read(m, 15, &v);
        *pc = v;
    } else {
        *state = HI_RUN;
    }
    return 0;
}

static const char *hr_name(int r)
{
    switch (r) {
    case HR_HALREQ: return "halreq";
    case HR_BKPT: return "bkpt";
    case HR_DWTTRAP: return "dwttrap";
    case HR_VCATCH: return "vcatch";
    case HR_EXTERNAL: return "external";
    }
    return "";
}

static int fp_numcode(memap_t *m)
{
    uint32_t v;
    if (mem_read32(m, FP_CTRL2, &v) < 0)
        return -1;
    return (int)(((v >> 8) & 0x70u) | ((v >> 4) & 0xFu));
}

static int dwt_numcomp(memap_t *m)
{
    uint32_t v;
    if (mem_read32(m, DWT_CTRL2, &v) < 0)
        return -1;
    return (int)(v >> 28);
}

/* 返回 0=成功（*comp 比较器号），-1=参数/资源错误（err 已打印） */
static int core_bp_add(shell_t *sh, uint32_t addr, uint32_t len, int *comp)
{
    if (!sh_cm_ready(sh, "bp"))
        return -1;
    memap_t *m = sh_mem(sh);
    int nc = fp_numcode(m);
    if (nc <= 0) {
        err("无 FPB 指令比较器");
        return -1;
    }
    if (len != 2 && len != 4) {
        err("len 只支持 2 或 4（M3 指令断点半字粒度）");
        return -1;
    }
    if (addr & 1u) {
        err("地址必须半字对齐");
        return -1;
    }
    if (addr > 0x1FFFFFFFu) {
        err("FPB rev0 比较上限 0x1FFFFFFE");
        return -1;
    }
    /* FP_COMP[31:30]：bit31=上半字、bit30=双半字(4B)，全 0=下半字——
     * 曾写成 addr&2?(2<<30):(1<<30)，bit31/bit30 语义颠倒 */
    uint32_t vcomp = (addr & 0x1FFFFFFCu) | 1u;
    if (len == 4)
        vcomp |= 2u << 30;
    else if (addr & 2u)
        vcomp |= 1u << 30;
    for (int i = 0; i < nc; i++) {
        uint32_t v;
        if (mem_read32(m, FP_COMP0 + 4u * i, &v) < 0)
            return -1;
        if (!(v & 1u)) {                          /* 空槽：占用（勿删——幂等
                                                      补丁曾把这段吃掉，add 永远
                                                      报"已满"的回归） */
            mem_write32(m, FP_COMP0 + 4u * i, vcomp);
            mem_write32(m, FP_CTRL2, 3u);         /* KEY | EN */
            *comp = i;
            return 0;
        }
        /* 同字同半字已覆盖（FP_COMP 只存字基址，半字选择在 bit30/31）：
         * 幂等复用，防地址量化误差导致重复堆比较器 */
        if ((v & 0x1FFFFFFCu) == (addr & 0x1FFFFFFCu) &&
            (v & 0xC0000000u) == (vcomp & 0xC0000000u)) {
            *comp = i;
            return 0;
        }
        mem_write32(m, FP_COMP0 + 4u * i, vcomp);
        mem_write32(m, FP_CTRL2, 3u);    /* KEY | EN */
        *comp = i;
        return 0;
    }
    err("FPB 比较器已满（%d 个）", nc);
    return -1;
}

static int core_bp_del(shell_t *sh, uint32_t addr, int all, int *removed)
{
    if (!sh_cm_ready(sh, "rbp"))
        return -1;
    memap_t *m = sh_mem(sh);
    int nc = fp_numcode(m);
    int n = 0;
    uint32_t base = addr & 0x1FFFFFFCu;
    for (int i = 0; nc > 0 && i < nc; i++) {
        uint32_t v;
        if (mem_read32(m, FP_COMP0 + 4u * i, &v) < 0)
            return -1;
        if (!(v & 1u))
            continue;
        if (!all && (v & 0x1FFFFFFCu) != base)
            continue;
        mem_write32(m, FP_COMP0 + 4u * i, 0);
        n++;
        if (!all)
            break;
    }
    *removed = n;
    if (!n && !all) {
        err("未找到 0x%08X 处的断点", addr);
        return -1;
    }
    return 0;
}

static int core_wp_add(shell_t *sh, uint32_t addr, uint32_t len, int acc,
                       int *comp)
{
    if (!sh_cm_ready(sh, "wp"))
        return -1;
    memap_t *m = sh_mem(sh);
    int nc = dwt_numcomp(m);
    if (nc <= 0) {
        err("无 DWT 比较器");
        return -1;
    }
    if (len == 0 || (len & (len - 1))) {
        err("len 必须是 2 的幂");
        return -1;
    }
    if (addr % len) {
        err("地址必须按 len 对齐");
        return -1;
    }
    uint32_t mask = 0;
    while ((1u << (mask + 1)) <= len)
        mask++;
    if (mask > 0xFu) {
        err("len 超出 DWT MASK 上限");
        return -1;
    }
    if (acc != 5 && acc != 6 && acc != 7) {
        err("acc 须为 5(r)/6(w)/7(a)");
        return -1;
    }
    uint32_t demcr;
    if (mem_read32(m, DEMCR2, &demcr) == 0)
        mem_write32(m, DEMCR2, demcr | (1u << 24));   /* TRCENA：DWT 供电 */
    for (int i = 0; i < nc; i++) {
        uint32_t base = DWT_COMP0 + 0x10u * i, f;
        if (mem_read32(m, base + 8u, &f) < 0)
            return -1;
        if (f)
            continue;
        mem_write32(m, base, addr);
        mem_write32(m, base + 4u, mask);
        mem_write32(m, base + 8u, (uint32_t)acc);
        *comp = i;
        return 0;
    }
    err("DWT 比较器已满（%d 个）", nc);
    return -1;
}

static int core_wp_del(shell_t *sh, uint32_t addr, int all, int *removed)
{
    if (!sh_cm_ready(sh, "rwp"))
        return -1;
    memap_t *m = sh_mem(sh);
    int nc = dwt_numcomp(m);
    int n = 0;
    for (int i = 0; nc > 0 && i < nc; i++) {
        uint32_t base = DWT_COMP0 + 0x10u * i, comp, f;
        if (mem_read32(m, base, &comp) < 0 || mem_read32(m, base + 8u, &f) < 0)
            return -1;
        if (!f)
            continue;
        if (!all && comp != addr)
            continue;
        mem_write32(m, base + 8u, 0);
        mem_write32(m, base + 4u, 0);
        n++;
        if (!all)
            break;
    }
    *removed = n;
    if (!n && !all) {
        err("未找到 0x%08X 处的观察点", addr);
        return -1;
    }
    return 0;
}

/* 一条龙配目标侧 ITM/TPIU + 本地 RX；返回实际 RX 波特率（0=失败） */
static uint32_t core_swo_tpiu(shell_t *sh, uint32_t tclk, uint32_t baud)
{
    memap_t *m = sh_mem(sh);
    if (!m)
        return 0;
    uint32_t actual = j_swo_enable(&sh->jtag, baud);
    if (!actual)
        return 0;
    sh->swo_baud = baud;
    uint32_t presc = (tclk + actual / 2u) / actual;
    uint32_t v;
    if (mem_read32(m, 0xE000EDFCu, &v) < 0)
        return 0;
    /* 热重配陷阱（实测）：TPIU 输出中直接改 SPPR/ACPR 会把输出搞挂（此后
     * 永久静默）；且 TRCENA=0 期间 trace 域断电，ITM/TPIU 寄存器写入全部
     * 丢弃（ACPR 卡旧值）。安全序列：先停 DWT 产出 → TRCENA 关→开复位
     * trace 域 → 在 TRCENA=1 下全量重配。 */
    mem_write32(m, 0xE0001000u, 0u);               /* DWT_CTRL off */
    mem_write32(m, 0xE000EDFCu, v & ~(1u << 24));  /* TRCENA off（域断电复位） */
    mem_write32(m, 0xE000EDFCu, v | (1u << 24));   /* TRCENA on */
    mem_write32(m, 0xE0000FB0u, 0xC5ACCE55u);      /* ITM LAR（CM3 无，写无害） */
    /* DBGMCU_CR: DBG_SLEEP|STOP|STANDBY + TRACE_IOEN——用户固件 WFI 睡眠时
     * 内核时钟不停，AP 访问才不会全部 WAIT（重启后固件常睡，实测踩坑） */
    #if TGT_HAS_DBGMCU
    mem_write32(m, TGT_DBGMCU_ADDR, TGT_DBGMCU_VALUE);
    #endif
    /* F1 目标：AFIO_MAPR SWJ_CFG=010 释放 PB3 给 TPIU（SWO 复用 JTDO 引脚，
     * 固件不做此重映射时 TPIU 出不了引脚，线上只见 SWD 轮询漏流）。
     * 必须先经过 000 再到 010：实测 SWO 输出会静默卡死（所有 trace 寄存器
     * 读回全对、CYCCNT 在走，但线上 idle 零字节），SWJ_CFG 循环一踢即活 */
    #if TGT_HAS_AFIO
    if (mem_read32(m, TGT_AFIO_MAPR_ADDR, &v) == 0) {
        uint32_t mapr = v & ~(7u << TGT_SWJ_CFG_SHIFT);
        mem_write32(m, TGT_AFIO_MAPR_ADDR, mapr);
        mem_write32(m, TGT_AFIO_MAPR_ADDR,
                    mapr | (TGT_SWJ_CFG_SWO << TGT_SWJ_CFG_SHIFT));
    }
    #endif
    /* ITM TCR：ITMena|TSEna|TXENA|BusID=1（0x17 缺 TXENA；TSEna 跟固件
     * swotest 的 0x0001000B 一致——它超时重配会自己写回，不一致会打架；
     * ts 包也是 swo_web GTC 时间线的来源）。
     * TER 全开：只开 port0 会把固件的 port1/2 流（g_ticks/事件）全禁掉 */
    mem_write32(m, 0xE0000E80u, 0x0001000Bu);
    mem_write32(m, 0xE0000E00u, 0xFFFFFFFFu);      /* ITM TER: 全端口 */
    mem_write32(m, 0xE00400F0u, 2u);               /* TPIU SPPR: async NRZ/UART */
    mem_write32(m, 0xE0040010u, presc - 1u);       /* TPIU ACPR */
    mem_write32(m, 0xE0040304u, 1u << 1);          /* TPIU FFCR: EnFCont */
    /* 上面安全序列停了 DWT：恢复 CYCCNTENA 基线（PC 采样/异常跟踪由
     * 客户端 set_trace 重放，但 CYCCNT 是 dwt 轮询测频的依赖，不能留 0） */
    mem_write32(m, 0xE0001000u, 1u);
    return actual;
}

/* ---- shell 包装（打印）---- */
static void cmd_haltinfo(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    int state, reason;
    uint32_t pc;
    if (core_haltinfo(sh, &state, &reason, &pc) < 0)
        return;
    if (state == HI_HALT)
        printf("state=halted reason=%s pc=0x%08X\n", hr_name(reason), pc);
    else
        printf("state=running\n");
}

static void cmd_bp(shell_t *sh, int argc, char **argv)
{
    if (argc < 2) {
        err("用法: bp <addr> [len=2|4]");
        return;
    }
    uint32_t addr = (uint32_t)parse_num(argv[1]);
    uint32_t len = argc >= 3 ? (uint32_t)parse_num(argv[2]) : 2;
    int comp;
    if (core_bp_add(sh, addr, len, &comp) == 0)
        printf("BP 0x%08X len=%u comp=%d\n", addr, len, comp);
}

static void cmd_rbp(shell_t *sh, int argc, char **argv)
{
    if (argc < 2) {
        err("用法: rbp <addr|all>");
        return;
    }
    int all = !strcmp(argv[1], "all");
    int removed = 0;
    core_bp_del(sh, all ? 0 : (uint32_t)parse_num(argv[1]), all, &removed);
    if (removed || all)
        printf("removed %d\n", removed);
}

static void cmd_wp(shell_t *sh, int argc, char **argv)
{
    if (argc < 3) {
        err("用法: wp <addr> <len> [r|w|a]");
        return;
    }
    uint32_t addr = (uint32_t)parse_num(argv[1]);
    uint32_t len = (uint32_t)parse_num(argv[2]);
    const char *a = argc >= 4 ? argv[3] : "w";
    int acc = a[0] == 'r' ? 5 : a[0] == 'w' ? 6 : a[0] == 'a' ? 7 : 0;
    int comp;
    if (core_wp_add(sh, addr, len, acc, &comp) == 0)
        printf("WP 0x%08X len=%u acc=%c comp=%d\n", addr, len, a[0], comp);
}

static void cmd_rwp(shell_t *sh, int argc, char **argv)
{
    if (argc < 2) {
        err("用法: rwp <addr|all>");
        return;
    }
    int all = !strcmp(argv[1], "all");
    int removed = 0;
    core_wp_del(sh, all ? 0 : (uint32_t)parse_num(argv[1]), all, &removed);
    if (removed || all)
        printf("removed %d\n", removed);
}

static void cmd_bps(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    if (!sh_cm_ready(sh, "bps"))
        return;
    memap_t *m = sh_mem(sh);
    int nc = fp_numcode(m);
    for (int i = 0; nc > 0 && i < nc; i++) {
        uint32_t v;
        if (mem_read32(m, FP_COMP0 + 4u * i, &v) < 0)
            break;
        if (v & 1u) {
            uint32_t repl = (v >> 30) & 3u;
            printf("BP 0x%08X len=%u comp=%d\n",
                   v & 0x1FFFFFFCu, repl == 3u ? 4u : 2u, i);
        }
    }
    int nd = dwt_numcomp(m);
    for (int i = 0; nd > 0 && i < nd; i++) {
        uint32_t base = DWT_COMP0 + 0x10u * i, comp, msk, fn;
        if (mem_read32(m, base, &comp) < 0 ||
            mem_read32(m, base + 4u, &msk) < 0 ||
            mem_read32(m, base + 8u, &fn) < 0)
            break;
        if (fn >= 5u && fn <= 7u)
            printf("WP 0x%08X len=%u acc=%c comp=%d\n",
                   comp, 1u << msk, fn == 5u ? 'r' : fn == 6u ? 'w' : 'a', i);
    }
}

static void cmd_target(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    memap_t *m = sh_mem(sh);
    if (!m)
        return;
    uint32_t d;
    if (mem_read32(m, DHCSR, &d) < 0)
        return;
    const char *st = (d & S_HALT) ? "halted" : "running";
    printf("target is %s", st);
    if (d & S_SLEEP)
        printf(" (sleeping)");
    if (d & S_LOCKUP)
        printf(" (locked up)");
    printf(" (DHCSR=0x%08X)\n", d);
}

/* ---- SWO bring-up（不依赖 OpenOCD；IP VERSION>=2，SWD 模式下 TDO=TRACESWO）---- */

static void cmd_swo(shell_t *sh, int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "off") != 0) {
        uint32_t baud = (uint32_t)parse_num(argv[1]);
        uint32_t actual = j_swo_enable(&sh->jtag, baud);
        if (actual) {
            sh->swo_baud = baud;
            printf("SWO on: 请求 %u Hz → 实际 RX %u Hz\n", baud, actual);
        }
    } else {
        j_swo_disable(&sh->jtag);
        sh->swo_baud = 0;
        printf("SWO off\n");
    }
}

static void cmd_reprobe(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    core_reprobe(sh);
    if (sh_dap(sh) && sh_cm_ready(sh, "reprobe")) {
        ensure_dbgmcu(sh);
        printf("reprobe: 目标已重连（AP%d）\n", sh->apsel);
    } else {
        printf("reprobe: 目标未应答（掉电/未上电？稍后再试）\n");
    }
}

static void cmd_swo_stat(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    int cnt, ovr, fe;
    j_swo_stat(&sh->jtag, &cnt, &ovr, &fe);
    printf("SWO_STAT: count=%d overrun=%d frame_err=%d\n", cnt, ovr, fe);
    if (ovr || fe) {
        j_swo_clear_err(&sh->jtag);
        printf("  (sticky errors cleared)\n");
    }
}

static void cmd_swo_read(shell_t *sh, int argc, char **argv)
{
    static uint8_t buf[SWO_FIFO_DEPTH];
    int n = argc >= 2 ? (int)parse_num(argv[1]) : 0;
    if (n <= 0 || n > SWO_FIFO_DEPTH)
        n = SWO_FIFO_DEPTH;
    int len = j_swo_read(&sh->jtag, buf, n);
    if (len == 0) {
        printf("(empty)\n");
        return;
    }
    for (int i = 0; i < len; i += 16) {
        int row = len - i < 16 ? len - i : 16;
        printf("%04x: ", i);
        for (int k = 0; k < 16; k++) {
            if (k < row)
                printf("%02X ", buf[i + k]);
            else
                printf("   ");
        }
        printf(" ");
        for (int k = 0; k < row; k++)
            putchar(buf[i + k] >= 32 && buf[i + k] < 127 ? buf[i + k] : '.');
        putchar('\n');
    }
}

/* 一条龙配目标侧 ITM/TPIU + 本地 RX（ARMv7-M，STM32F1 同）。
 * 分频策略与 OpenOCD 驱动一致：TPIU prescaler 逼近 RX 实际值（v3 边沿
 * CDR 任意整数档，失配通常 <1%，容差 ~±5%）。须先 probe。 */
static void cmd_swo_tpiu(shell_t *sh, int argc, char **argv)
{
    if (argc < 3) {
        err("用法: swo_tpiu <traceclk_hz> <baud>");
        return;
    }
    uint32_t tclk = (uint32_t)parse_num(argv[1]);
    uint32_t baud = (uint32_t)parse_num(argv[2]);
    uint32_t actual = core_swo_tpiu(sh, tclk, baud);
    if (!actual)
        return;
    uint32_t presc = (tclk + actual / 2u) / actual;
    uint32_t tpiu_baud = tclk / presc;
    printf("traceclk=%.2f MHz  ACPR=%u → TPIU 输出 %u Hz；RX %u Hz (失配 %.2f%%)\n",
           tclk / 1e6, presc - 1u, tpiu_baud, actual,
           100.0 * (tpiu_baud > actual ? tpiu_baud - actual : actual - tpiu_baud) / actual);
    printf("目标固件往 0xE0000000 (ITM port0) 写字节即出 SWO；swo_read 查看\n");
    dap_check_sticky(&sh->dap);
}

/* ---- vref：经 SPI 查 ESP32 上的 JTAG Vref（ADC 采，10k/10k 分压）----
 * 协议：两段式。段1 发 8B 命令帧 [A5|cmd|seq|0...]，等 2ms 从机装应答，
 * 段2 读 16B [5A|cmd|seq|status|vref_mv LE32|...]。seq 校验，失败重试一次。 */
#define VREF_CMD_MAGIC  0xA5
#define VREF_RESP_MAGIC 0x5A
#define VREF_CMD_GET_VREF 0x01

static int spi_xfer(int fd, const uint8_t *tx, uint8_t *rx, int len)
{
    struct spi_ioc_transfer x = { 0 };
    x.tx_buf = (unsigned long)tx;
    x.rx_buf = (unsigned long)rx;
    x.len = len;
    x.speed_hz = 1000000;
    x.bits_per_word = 8;
    return ioctl(fd, SPI_IOC_MESSAGE(1), &x) < 1 ? -1 : 0;
}

/* 读 Vref，返回 mV；失败 -1（err() 已报具体原因）。cmd_vref 和 BIN_VREF 共用 */
static int vref_read_mv(const char *dev)
{
    int fd = open(dev, O_RDWR);
    if (fd < 0) {
        err("打不开 %s（SPI 驱动/spidev 没起来？）", dev);
        return -1;
    }
    uint8_t mode = SPI_MODE_3;
    ioctl(fd, SPI_IOC_WR_MODE, &mode);

    for (uint8_t seq = 1, attempt = 0; attempt < 2; attempt++, seq++) {
        uint8_t cmd[8] = { VREF_CMD_MAGIC, VREF_CMD_GET_VREF, seq };
        uint8_t dummy[8];
        if (spi_xfer(fd, cmd, dummy, 8) < 0) {
            err("SPI 传输失败");
            close(fd);
            return -1;
        }
        usleep(2000);                       /* 从机装应答帧的余量 */
        uint8_t resp[16] = { 0 };
        if (spi_xfer(fd, resp, resp, 16) < 0) {
            err("SPI 传输失败");
            close(fd);
            return -1;
        }
        if (resp[0] == VREF_RESP_MAGIC && resp[1] == VREF_CMD_GET_VREF &&
            resp[2] == seq && resp[3] == 0) {
            uint32_t mv = resp[4] | resp[5] << 8 | resp[6] << 16 |
                          (uint32_t)resp[7] << 24;
            close(fd);
            return (int)mv;
        }
    }
    err("ESP32 未应答（检查从机固件/接线/CS）");
    close(fd);
    return -1;
}

static void cmd_vref(shell_t *sh, int argc, char **argv)
{
    (void)sh;
    const char *dev = argc >= 2 ? argv[1] : "/dev/spidev0.0";
    int mv = vref_read_mv(dev);
    if (mv >= 0)
        printf("vref = %u.%03u V\n", (unsigned)mv / 1000, (unsigned)mv % 1000);
}

static void cmd_rst(shell_t *sh, int argc, char **argv)
{
    if (argc < 2) {
        err("用法: rst <0|1>");
        return;
    }
    int v = (int)parse_num(argv[1]) & 1;
    j_srst(&sh->jtag, v);
    printf("SRST=%s\n", v ? "released" : "asserted (low)");
}

static void cmd_reset(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    j_reset_engine(&sh->jtag);
    if (sh->dap_ok)
        sh->dap.sel_key = -1;
    printf("engine aborted (FIFO/FSM cleared), target TAP -> TLR, pins restored\n");
}

static void cmd_treset(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    j_goto_tlr(&sh->jtag);
    printf("TAP reset (TLR)\n");
}

static void cmd_tbit(shell_t *sh, int argc, char **argv)
{
    if (argc < 3) {
        err("用法: tbit <tms> <tdi>");
        return;
    }
    int tdo = j_shift1(&sh->jtag, (int)parse_num(argv[1]), (int)parse_num(argv[2]));
    if (tdo >= 0)
        printf("TDO=%d\n", tdo);
}

static void cmd_tms(shell_t *sh, int argc, char **argv)
{
    if (argc < 2) {
        err("用法: tms <0|1> ...");
        return;
    }
    int n = argc - 1 > 32 ? 32 : argc - 1;
    uint8_t tms[32], tdi[32], cap[32];
    for (int i = 0; i < n; i++) {
        tms[i] = (uint8_t)(parse_num(argv[i + 1]) & 1);
        tdi[i] = 1;
    }
    if (j_shift(&sh->jtag, tms, tdi, n, cap, 0) < 0)
        return;
    printf("TDO: ");
    for (int i = 0; i < n; i++)
        putchar('0' + cap[i]);
    putchar('\n');
}

static void cmd_tdr(shell_t *sh, int argc, char **argv)
{
    if (argc < 2) {
        err("用法: tdr <n>");
        return;
    }
    int n = (int)parse_num(argv[1]);
    if (n < 1 || n > 256) {
        err("n range: 1..256");
        return;
    }
    uint8_t tms[256], tdi[256], cap[256];
    memset(tdi, 1, n);
    memset(tms, 0, n - 1);
    tms[n - 1] = 1;
    if (j_shift(&sh->jtag, tms, tdi, n, cap, 0) < 0)
        return;
    printf("DR: ");
    for (int i = 0; i < n; i++)
        putchar('0' + cap[i]);
    putchar('\n');
}

static void cmd_tir(shell_t *sh, int argc, char **argv)
{
    if (argc < 3) {
        err("用法: tir <hex> <nbits>");
        return;
    }
    uint64_t v = parse_num(argv[1]);
    int n = (int)parse_num(argv[2]);
    if (n < 1 || n > 32) {
        err("nbits range: 1..32");
        return;
    }
    uint8_t tms[64], tdi[64], cap[64];
    memset(tdi, 0, sizeof(tdi));
    memset(tms, 0, sizeof(tms));
    for (int i = 0; i < 5; i++)
        tms[i] = 1;                      /* TLR */
    tms[5] = 0; tms[6] = 1; tms[7] = 1; tms[8] = 0; tms[9] = 0;  /* →ShiftIR */
    for (int i = 0; i < n; i++) {
        tms[10 + i] = (i == n - 1) ? 1 : 0;
        tdi[10 + i] = (v >> i) & 1;
    }
    tms[10 + n] = 1;                     /* Update */
    tms[10 + n + 1] = 0;                 /* RTI */
    if (j_shift(&sh->jtag, tms, tdi, 10 + n + 2, cap, 0) < 0)
        return;
    printf("IR<=0x%llX (%d bits), updated\n", (unsigned long long)v, n);
}

static void cmd_tdo(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    printf("TDO=%d (no clock)\n", j_tdo_pin(&sh->jtag));
}

static volatile sig_atomic_t g_dump_abort;

static void dump_sigint(int sig)
{
    (void)sig;
    g_dump_abort = 1;
}

static void cmd_dump(shell_t *sh, int argc, char **argv)
{
    if (argc < 3) {
        err("用法: dump <start> <count> [file]");
        return;
    }
    uint64_t start = parse_num(argv[1]);
    uint32_t count = (uint32_t)parse_num(argv[2]);
    char defname[64];
    snprintf(defname, sizeof(defname), "dump_%08llX_%uw.bin",
             (unsigned long long)start, count);
    const char *fname = argc >= 4 ? argv[3] : defname;
    FILE *f = fopen(fname, "wb");
    if (!f) {
        err("打不开 %s", fname);
        return;
    }
    memap_t *m = sh_mem(sh);
    if (!m) {
        fclose(f);
        return;
    }
    double t0 = now_sec();
    uint32_t done = 0;
    g_dump_abort = 0;
    signal(SIGINT, dump_sigint);       /* Ctrl+C 优雅中止，不杀 shell */
    for (; done < count && !g_dump_abort; done++) {
        uint32_t v;
        if (mem_read32(m, start + 4ull * done, &v) < 0) {
            printf("\n");
            break;
        }
        fwrite(&v, 4, 1, f);        /* 目标与主机同为小端 */
        if ((done % 1024) == 1023) {
            printf("\r%u/%u", done + 1, count);
            fflush(stdout);
        }
    }
    fclose(f);
    signal(SIGINT, SIG_DFL);
    if (g_dump_abort)
        printf("\n(已中止，部分数据已写入) ");
    double dt = now_sec() - t0;
    printf("%s: %u bytes, %.2fs, %.1f KB/s\n", fname, done * 4, dt,
           done * 4 / dt / 1024.0);
    dap_check_sticky(&sh->dap);
}

static void cmd_bench(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    jtag_t *j = &sh->jtag;
    int n = 4 * 512;
    uint8_t tms[512], tdi[512], cap[512];
    memset(tms, 0, sizeof(tms));
    memset(tdi, 0, sizeof(tdi));
    double t0 = now_sec();
    for (int i = 0; i < 4; i++)
        if (j_shift(j, tms, tdi, 512, cap, 0) < 0)
            return;
    double dt = now_sec() - t0;
    printf("raw TCK        : %.0f kHz (%.2f us/bit)\n", n / dt / 1000.0,
           dt / n * 1e6);
    dap_t *d = sh_dap(sh);
    if (!d)
        return;
    t0 = now_sec();
    for (int i = 0; i < 10; i++)
        if (dap_acc(d, 0, DP_RDBUFF, 1, 0, NULL) < 0)
            return;
    dt = now_sec() - t0;
    printf("10x DPACC(RDBUFF): %.2f ms/txn\n", dt / 10 * 1e3);
    t0 = now_sec();
    for (int i = 0; i < 10; i++)
        if (dap_acc(d, 1, AP_TAR, 0, 0x20000000u, NULL) < 0)
            return;
    dt = now_sec() - t0;
    printf("10x APACC(TAR wr): %.2f ms/txn\n", dt / 10 * 1e3);
}

static void cmd_dbg(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    jtag_t *j = &sh->jtag;
    uint8_t d[70];
    j_goto_tlr(j);
    if (j_dr_raw(j, 70, d) < 0)
        return;
    printf("A raw DR post-TLR : ");
    for (int k = 0; k < 9; k++) {
        int v = 0;
        for (int i = 0; i < 8 && k * 8 + i < 70; i++)
            v |= d[k * 8 + i] << i;
        printf("%02X ", v);
    }
    printf("\n");
    j_goto_tlr(j);
    uint8_t tms[16], tdi[16], cap[16];
    memset(tdi, 0, sizeof(tdi));
    memset(tms, 0, sizeof(tms));
    tms[0] = 0; tms[1] = 1; tms[2] = 1; tms[3] = 0; tms[4] = 0;  /* →ShiftIR */
    for (int i = 0; i < 6; i++) {
        tdi[5 + i] = 1;
        tms[5 + i] = 0;
    }
    for (int i = 0; i < 4; i++) {
        tdi[11 + i] = (0xE >> i) & 1;
        tms[11 + i] = (i == 3) ? 1 : 0;
    }
    tms[15] = 1;
    if (j_shift(j, tms, tdi, 17, cap, 0) < 0)
        return;
    int ir_cap = 0;
    for (int i = 0; i < 4; i++)
        ir_cap |= cap[11 + i] << i;
    printf("B IR shift TDO    : DP-IR capture=0x%X\n", ir_cap);
    if (j_dr_raw(j, 40, d) < 0)
        return;
    printf("C raw DR after IR : ");
    for (int k = 0; k < 5; k++) {
        int v = 0;
        for (int i = 0; i < 8 && k * 8 + i < 40; i++)
            v |= d[k * 8 + i] << i;
        printf("%02X ", v);
    }
    printf("\n");
    j_goto_tlr(j);
}

/* ---- scandump 命令 ---- */

static sd_t *sh_sd(shell_t *sh)
{
    sh->sd.j = &sh->jtag;
    return &sh->sd;
}

static void cmd_scan_switch(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    if (!sh->dap_ok) {
        err("scan_switch: 先 probe（需经 AP2 写切换寄存器）");
        return;
    }
    memap_t m2;
    memap_init(&m2, &sh->dap, 2);
    mem_write32(&m2, 0x50430064u, 1);    /* 切换瞬间 ACK=0 属正常，忽略返回 */
    dap_check_sticky(&sh->dap);
    sh_invalidate_arm(sh);
    printf("TAP switched to scandump persona (wrote 0x50430064=1 via AP2)\n");
}

static void cmd_scan_id(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    uint32_t id = 0;
    int rc = sd_read_idcode(sh_sd(sh), &id);
    if (rc < 0)
        return;
    printf("IDCODE=0x%08X %s\n", id,
           rc > 0 ? "(match)" : "(MISMATCH, expect 0xAABBCCDD)");
}

static void cmd_scan_bypass(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    uint32_t echo = 0;
    int rc = sd_test_bypass(sh_sd(sh), &echo);
    if (rc < 0)
        return;
    printf("BYPASS %s (echo=0x%04X, expect 0x5554)\n",
           rc > 0 ? "OK" : "FAIL", echo);
}

static void cmd_scan_sel(shell_t *sh, int argc, char **argv)
{
    if (argc < 2) {
        err("用法: scan_sel <0=CFG 1=SCAN_I 2=SCAN_O>");
        return;
    }
    int c = (int)parse_num(argv[1]);
    if (sd_path_setup(sh_sd(sh), c) == 0)
        printf("chain %d selected, DEBUG mode\n", c);
}

static void cmd_scan_dbg(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    if (sd_ir_load(sh_sd(sh), SD_INSTR_DEBUG) == 0)
        printf("DEBUG instruction loaded\n");
}

static void sd_rwreg(shell_t *sh, int argc, char **argv, int reg, int writes)
{
    if (writes) {
        if (argc < 2) {
            err("用法: scan_w%d <val>", reg);
            return;
        }
        sd_cfg_write(sh_sd(sh), reg, (int)parse_num(argv[1]));
        printf("CFG%d <- 0x%02X\n", reg, (uint32_t)parse_num(argv[1]));
    } else {
        uint8_t v;
        if (sd_cfg_read(sh_sd(sh), reg, &v) == 0) {
            printf("CFG%d = 0x%02X\n", reg, v);
            if (reg == SD_CFG_TRIGGER)
                printf("  (trigger 位硬件自清，读到 0x0 属正常)\n");
        }
    }
}

static void cmd_scan_w0(shell_t *sh, int a, char **v) { sd_rwreg(sh, a, v, 0, 1); }
static void cmd_scan_w1(shell_t *sh, int a, char **v) { sd_rwreg(sh, a, v, 1, 1); }
static void cmd_scan_w2(shell_t *sh, int a, char **v) { sd_rwreg(sh, a, v, 2, 1); }
static void cmd_scan_r0(shell_t *sh, int a, char **v) { sd_rwreg(sh, a, v, 0, 0); }
static void cmd_scan_r1(shell_t *sh, int a, char **v) { sd_rwreg(sh, a, v, 1, 0); }
static void cmd_scan_r2(shell_t *sh, int a, char **v) { sd_rwreg(sh, a, v, 2, 0); }

static int sd_out_row(sd_t *sd, uint8_t *bits)
{
    uint8_t zeros[SD_SCAN_OUT_BITS];
    memset(zeros, 0, sizeof(zeros));
    return sd_dr(sd, zeros, SD_SCAN_OUT_BITS, bits);
}

static void cmd_scan_out(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    uint8_t bits[SD_SCAN_OUT_BITS];
    sd_t *sd = sh_sd(sh);
    if (sd_path_setup(sd, SD_CHAIN_SCAN_OUT) == 0 && sd_out_row(sd, bits) == 0)
        sd_print_row(sd, 0, bits);
}

static void cmd_scan_t(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    uint8_t bits[SD_SCAN_OUT_BITS];
    sd_t *sd = sh_sd(sh);
    if (sd_cfg_write(sd, SD_CFG_TRIGGER, 1) == 0 &&
        sd_path_setup(sd, SD_CHAIN_SCAN_OUT) == 0 && sd_out_row(sd, bits) == 0)
        sd_print_row(sd, 0, bits);
}

static void cmd_scan_in(shell_t *sh, int argc, char **argv)
{
    if (argc < 2) {
        err("用法: scan_in <lo32> [hi32]");
        return;
    }
    uint32_t w0 = (uint32_t)parse_num(argv[1]);
    uint32_t w1 = argc >= 3 ? (uint32_t)parse_num(argv[2]) : 0;
    if (sd_scan_in_write(sh_sd(sh), w0, w1) == 0)
        printf("scan_in 0x%08X_%08X\n", w1, w0);
}

static int sd_dump(shell_t *sh, int rows, int cfg2, int burst)  /* burst -1=auto */
{
    sd_t *sd = sh_sd(sh);
    int mode = burst < 0 ? SD_CFG0_AUTO_1CLK :
               SD_CFG0_MANUAL | (burst << SD_CFG0_BURST_SHIFT);
    if (sd_cfg_write(sd, SD_CFG_MODE, mode) < 0)
        return -1;
    if (sd_cfg_write(sd, SD_CFG_ENABLE, cfg2) < 0)
        return -1;
    if (sd_cfg_write(sd, SD_CFG_TRIGGER, 1) < 0)
        return -1;
    uint8_t bits[SD_SCAN_OUT_BITS];
    for (int r = 0; r < rows; r++) {
        if (sd_path_setup(sd, SD_CHAIN_SCAN_OUT) < 0)
            break;
        if (sd_out_row(sd, bits) < 0)
            break;
        sd_print_row(sd, r, bits);
    }
    sd_cfg_write(sd, SD_CFG_ENABLE, 0);   /* Ctrl+C 由信号退出，atexit 兜底难做，
                                             主循环重入时首命令前 reset 即可 */
    return 0;
}

static void cmd_scan_m(shell_t *sh, int argc, char **argv)
{
    if (argc < 2) {
        err("用法: scan_m <0-3> [reads] [cfg2]");
        return;
    }
    int burst = (int)parse_num(argv[1]);
    if (burst > 3) {
        err("burst 0..3");
        return;
    }
    static const int rpb[4] = { 1, 4, 16, 64 };
    int reads = argc >= 3 ? (int)parse_num(argv[2]) : rpb[burst];
    int cfg2 = argc >= 4 ? (int)parse_num(argv[3]) : SD_CFG2_DUMP;
    sd_dump(sh, reads, cfg2, burst);
}

static void cmd_scan_a(shell_t *sh, int argc, char **argv)
{
    if (argc < 2) {
        err("用法: scan_a <rows> [cfg2]");
        return;
    }
    int cfg2 = argc >= 3 ? (int)parse_num(argv[2]) : SD_CFG2_DUMP;
    sd_dump(sh, (int)parse_num(argv[1]), cfg2, -1);
}

static void cmd_scan_stop(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    sd_cfg_write(sh_sd(sh), SD_CFG_ENABLE, 0);
    printf("dump stopped\n");
}

static void cmd_scan_memkeep(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    static const struct { int v; const char *d; } steps[] = {
        { SD_CFG2_MIU_SR, "miu_sr" }, { SD_CFG2_MIU_PADDET, "miu_paddet" },
        { SD_CFG2_DUMP, "dump on" }, { 0x00, "dump off" },
        { SD_CFG2_BIU_RST, "biu rst" }, { 0x00, "biu rst off" },
        { SD_CFG2_MIU_LOCK, "miu lock" }, { 0x00, "miu unlock" },
        { SD_CFG2_PERI_RST, "peri rst" },
    };
    for (unsigned i = 0; i < sizeof(steps) / sizeof(steps[0]); i++) {
        sd_cfg_write(sh_sd(sh), SD_CFG_ENABLE, steps[i].v);
        printf("  %s\n", steps[i].d);
    }
    printf("mem_keep done; memory retained (access via bus/PMP)\n");
}

static void cmd_scan_fmt(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    sh->sd.fmt_bit = !sh->sd.fmt_bit;
    printf("row format: %s\n", sh->sd.fmt_bit ? "bit" : "hex");
}

static void cmd_scan_verify(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    sd_t *sd = sh_sd(sh);
    uint8_t bits[SD_SCAN_OUT_BITS];
    for (int i = 0; i < 3; i++)
        if (sd_scan_in_write(sd, 0xFFFFFFFFu, 0xFFFFFFFFu) < 0)
            return;
    if (sd_path_setup(sd, SD_CHAIN_SCAN_OUT) == 0 && sd_out_row(sd, bits) == 0)
        sd_print_row(sd, 0, bits);
}

/* ---- 命令表 / help / 主循环 ---- */

enum { G_GEN, G_ARM, G_CM, G_SD };

typedef struct {
    const char *name;
    void (*fn)(shell_t *, int, char **);
    int group;
} cmd_ent_t;

static const cmd_ent_t cmds[] = {
    { "scan", cmd_scan, G_GEN },
    { "probe", cmd_probe, G_GEN },
    { "swd", cmd_swd, G_GEN },
    { "swd_dbg", cmd_swd_dbg, G_GEN },
    { "jtag", cmd_jtag, G_GEN },
    { "chain", cmd_chain, G_GEN },
    { "speed", cmd_speed, G_GEN },
    { "vref", cmd_vref, G_GEN },
    { "rst", cmd_rst, G_GEN },
    { "reset", cmd_reset, G_GEN },
    { "swo", cmd_swo, G_GEN },
    { "swo_stat", cmd_swo_stat, G_GEN },
    { "reprobe", cmd_reprobe, G_GEN },
    { "swo_read", cmd_swo_read, G_GEN },
    { "swo_tpiu", cmd_swo_tpiu, G_GEN },
    { "treset", cmd_treset, G_GEN },
    { "tbit", cmd_tbit, G_GEN },
    { "tms", cmd_tms, G_GEN },
    { "tdr", cmd_tdr, G_GEN },
    { "tir", cmd_tir, G_GEN },
    { "tdo", cmd_tdo, G_GEN },
    { "dbg", cmd_dbg, G_GEN },
    { "aps", cmd_aps, G_ARM },
    { "ap", cmd_ap, G_ARM },
    { "dapinfo", cmd_dapinfo, G_ARM },
    { "dpr", cmd_dpr, G_ARM },
    { "dpw", cmd_dpw, G_ARM },
    { "apr", cmd_apr, G_ARM },
    { "apw", cmd_apw, G_ARM },
    { "mdw", cmd_mdw, G_ARM },
    { "mdh", cmd_mdh, G_ARM },
    { "mdb", cmd_mdb, G_ARM },
    { "mww", cmd_mww, G_ARM },
    { "mwh", cmd_mwh, G_ARM },
    { "mwb", cmd_mwb, G_ARM },
    { "dump", cmd_dump, G_ARM },
    { "bench", cmd_bench, G_ARM },
    { "target", cmd_target, G_CM },
    { "haltinfo", cmd_haltinfo, G_CM },
    { "halt", cmd_halt, G_CM },
    { "step", cmd_step, G_CM },
    { "resume", cmd_resume, G_CM },
    { "reg", cmd_reg, G_CM },
    { "bp", cmd_bp, G_CM },
    { "rbp", cmd_rbp, G_CM },
    { "wp", cmd_wp, G_CM },
    { "rwp", cmd_rwp, G_CM },
    { "bps", cmd_bps, G_CM },
    { "scan_switch", cmd_scan_switch, G_SD },
    { "scan_id", cmd_scan_id, G_SD },
    { "scan_bypass", cmd_scan_bypass, G_SD },
    { "scan_sel", cmd_scan_sel, G_SD },
    { "scan_dbg", cmd_scan_dbg, G_SD },
    { "scan_w0", cmd_scan_w0, G_SD },
    { "scan_w1", cmd_scan_w1, G_SD },
    { "scan_w2", cmd_scan_w2, G_SD },
    { "scan_r0", cmd_scan_r0, G_SD },
    { "scan_r1", cmd_scan_r1, G_SD },
    { "scan_r2", cmd_scan_r2, G_SD },
    { "scan_in", cmd_scan_in, G_SD },
    { "scan_out", cmd_scan_out, G_SD },
    { "scan_t", cmd_scan_t, G_SD },
    { "scan_m", cmd_scan_m, G_SD },
    { "scan_a", cmd_scan_a, G_SD },
    { "scan_stop", cmd_scan_stop, G_SD },
    { "scan_memkeep", cmd_scan_memkeep, G_SD },
    { "scan_fmt", cmd_scan_fmt, G_SD },
    { "scan_verify", cmd_scan_verify, G_SD },
    { "help", cmd_help, G_GEN },
};

static void cmd_help(shell_t *sh, int argc, char **argv)
{
    (void)argc; (void)argv;
    printf("-- general --\n");
    printf("  scan              通用链扫描（IDCODE+BYPASS 解析）\n");
    printf("  chain             链拓扑：BYPASS 位宽 + 逐 TAP IR 长度\n");
    printf("  probe             链扫描 + DAP 上电（3 拓扑自动试探）\n");
    printf("  swd               切 SWD transport（SWO 必需）\n");
    printf("  jtag              切回 JTAG transport\n");
    printf("  speed [khz]       查看/设定 TCK\n");
    printf("  vref [dev]         SPI 查询 ESP32 采的 JTAG Vref\n");
    printf("  rst <0|1>         SRST 复位线（0=拉低复位）\n");
    printf("  reset             完整复位：IP 引擎(FIFO+FSM) + 目标 TAP\n");
    printf("-- SWO trace（IP VERSION>=2；SWD 模式下 TDO 引脚 = TRACESWO）--\n");
    printf("  swo <baud|off>    接收器使能/关闭（0→1 沿 flush FIFO）\n");
    printf("  swo_stat          字节数 + overrun/frame_err sticky\n");
    printf("  swo_read [n]      读 n 字节 hex dump（默认全部）\n");
    printf("  swo_tpiu <tclk> <baud>  配目标 ITM/TPIU + 本地 RX（须先 probe）\n");
    printf("-- bit-level JTAG（私有 IR / 非标准 TAP 探索）--\n");
    printf("  treset            TAP 复位到 Test-Logic-Reset\n");
    printf("  tbit <tms> <tdi>  一拍 TCK，打印 TDO\n");
    printf("  tms <0|1>...      走 TMS 序列（TDI=1），逐拍打印 TDO\n");
    printf("  tdr <n>           ShiftDR 移 n 位（TDI=1）\n");
    printf("  tir <hex> <nbits> 手工装 IR（LSB first）然后 Update\n");
    printf("  tdo               不打时钟读 TDO 引脚\n");
    printf("  dbg               IR 路径对照实验（链方向诊断）\n");
    if (!sh->dap_ok) {
        printf("-- ARM CoreSight：先 'probe' 解锁本组命令 --\n");
    } else {
        printf("-- ARM CoreSight（target detected，'probe' 重新探测）--\n");
        printf("  aps               枚举 DAP 的 AP\n");
        printf("  ap [n]            查看/设定默认 APSEL（换域重检 CM）\n");
        printf("  dapinfo [apsel]   ROM 表组件发现（LPAE 打 64 位 BASE）\n");
        printf("  dpr <r>           读 DP 寄存器\n");
        printf("  dpw <r> <v>       写 DP 寄存器\n");
        printf("  apr <bank> <r> [apsel]        读 AP 寄存器\n");
        printf("  apw <bank> <r> <v> [apsel]    写 AP 寄存器\n");
        printf("  mdw|mdh|mdb <a> [n]           读内存（8/8/16 每行，64 位地址直通）\n");
        printf("  mww|mwh|mwb <a> <v>...        写内存（成功静默）\n");
        printf("  dump <a> <n> [f]  内存 dump 到文件（word，LE bin）\n");
    printf("  bench             链路时序基准\n");
        printf("  dbg               IR 路径对照实验\n");
        if (!sh->cm_ok) {
            printf("  (Cortex-M 运行控制：在核所在 AP 上执行 halt/reg 等自动确认)\n");
        } else {
            printf("-- Cortex-M run control --\n");
            printf("  target            halted/running 状态\n");
            printf("  haltinfo          state/reason(DCSR+DFSR 解码)/pc（GUI 轮询用，DFSR 读清）\n");
            printf("  halt              停住 CPU\n");
            printf("  step              单步一条指令\n");
            printf("  resume            继续运行\n");
            printf("  reg [name] [val]  核内寄存器 读/写（须先 halt）\n");
            printf("  bp <a> [2|4]      FPB 硬件断点；rbp <a|all>；bps 列表\n");
            printf("  wp <a> <len> [r|w|a]  DWT 观察点；rwp <a|all>\n");
        }
    }
    printf("-- scandump（Arise2 私有人格，命名对齐 openocd scan_*）--\n");
    printf("  scan_switch       TAP 切到 scandump 人格（经 AP2，须先 probe）\n");
    printf("  scan_id           读 IDCODE 并校验\n");
    printf("  scan_bypass       BYPASS 链路自检（0xAAAA→0x5554）\n");
    printf("  scan_sel <c>      选 DR 通路 0=CFG 1=SCAN_I 2=SCAN_O\n");
    printf("  scan_dbg          装 DEBUG 指令\n");
    printf("  scan_w0/1/2 <v>   写 CFG0/1/2\n");
    printf("  scan_r0/1/2       读 CFG0/1/2\n");
    printf("  scan_in <lo32> [hi32]   写 SCAN_I 64bit\n");
    printf("  scan_out          读一行 SCAN_O（119bit）\n");
    printf("  scan_t            触发一拍 shift_clk + 读\n");
    printf("  scan_m <0-3> [n] [cfg2]  manual dump（burst 1/4/16/64 拍）\n");
    printf("  scan_a <rows> [cfg2]    auto dump（openocd 风格）\n");
    printf("  scan_stop         停止 dump（CFG2=0）\n");
    printf("  scan_memkeep      dump 后保内存（MIU 序列）\n");
    printf("  scan_fmt          hex/bit 行格式切换\n");
    printf("  scan_verify       SCAN_I 全 1 写读自检\n");
}

static void execute(shell_t *sh, char *line)
{
    char *argv[16];
    int argc = 0;
    char *p = line;
    while (*p && argc < 15) {
        while (*p == ' ' || *p == '\t')
            *p++ = 0;
        if (!*p)
            break;
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t')
            p++;
    }
    if (!argc)
        return;
    argv[argc] = NULL;
    for (unsigned i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++)
        if (!strcmp(argv[0], cmds[i].name)) {
            cmds[i].fn(sh, argc, argv);
            return;
        }
    printf("未知命令 %s（help 查看）\n", argv[0]);
}

/* ================================================================== */
/* 网络服务器模式：jtag_tool --serve [port]（swo_web GUI 的后端）        */
/*   命令口 port（默认 5555）：行文本协议——客户端每发一行命令，服务器   */
/*     执行（输出经 dup2 捕获直写 socket）后追加标记行 "Z<seq>D\n"；     */
/*     无回显、无异步日志，比 openocd telnet 干净                       */
/*   SWO 流口 port+1（5556）：主循环每 2ms 从 IP FIFO 排水直推原始字节  */
/*     （SWD 引擎与 SWO RX 硬件并行，调试命令不影响采集）               */
/* 单线程 select；bench/dump 类长命令会占住循环暂停排水。单客户端槽位。 */
/* ================================================================== */
static int srv_listener(int port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
        return -1;
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in a;
    memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons((uint16_t)port);
    if (bind(fd, (struct sockaddr *)&a, sizeof(a)) < 0 || listen(fd, 1) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

/* ---- 二进制协议命令处理 ----
 * 包格式（双向）：[seq:1][cmd:1][len:2 LE][payload:len]
 * 请求 cmd：01 PING / 02 CMD(文本) / 03 HALTINFO / 04 HALT / 05 RESUME /
 *   06 STEP[n] / 07 REG_READ[sel...] / 08 REG_WRITE[sel][val] /
 *   09 MEM_READ[addr:4][w:1][n:2] / 0A MEM_WRITE[+data] /
 *   0B BP_ADD[addr:4][len:1] / 0C BP_DEL[addr:4|FFFFFFFF] /
 *   0D WP_ADD[addr:4][len:4][acc:1] / 0E WP_DEL[addr|FFFFFFFF] /
 *   0F BPS / 10 SWO_TPIU[tclk:4][baud:4] / 11 SWO_STAT
 * 响应 cmd = req|0x80；错误 0x7F + 文本。 */
#define BIN_PING      0x01
#define BIN_CMD       0x02
#define BIN_HALTINFO  0x03
#define BIN_HALT      0x04
#define BIN_RESUME    0x05
#define BIN_STEP      0x06
#define BIN_REG_RD    0x07
#define BIN_REG_WR    0x08
#define BIN_MEM_RD    0x09
#define BIN_MEM_WR    0x0A
#define BIN_BP_ADD    0x0B
#define BIN_BP_DEL    0x0C
#define BIN_WP_ADD    0x0D
#define BIN_WP_DEL    0x0E
#define BIN_BPS       0x0F
#define BIN_SWO_TPIU  0x10
#define BIN_SWO_STAT  0x11
#define BIN_REPROBE   0x12
#define BIN_VREF      0x13
#define BIN_ERR       0x7F
#define BIN_MAXPL     16384

/* CMD(0x02)：经 pipe 捕获 execute 的 printf 输出（单线程，命令结束后
 * 一次性读干；F_SETPIPE_SZ 提到 1MB 防 dump 类大输出中途塞死管道） */
static int srv_text_out(shell_t *sh, const char *line, uint8_t *out,
                        int max)
{
    int pfd[2];
    if (pipe(pfd) < 0)
        return -1;
    fcntl(pfd[1], F_SETPIPE_SZ, 1u << 20);
    fflush(stdout);
    int saved = dup(1);
    dup2(pfd[1], 1);
    close(pfd[1]);
    execute(sh, (char *)line);
    fflush(stdout);
    dup2(saved, 1);
    close(saved);
    /* 循环读到 EOF：写端已全关（fd1 已恢复），单次 read 理论上也能拿全，
     * 但 pipe 数据分块到达时防御性循环更稳（且未来 execute 若 fork 子进程
     * 继承写端，单次 read 会短读甚至提前返回空） */
    int n = 0, r;
    while (n < max && (r = (int)read(pfd[0], out + n, (size_t)(max - n))) > 0)
        n += r;
    close(pfd[0]);
    return n;
}

static uint32_t rd32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put32le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static int srv_binary(shell_t *sh, uint8_t cmd, const uint8_t *p, int len,
                      uint8_t *out, int max)
{
    /* 返回响应长度；负值 = 错误（out 里放文本） */
    memap_t *m;
    int rc, i;

    g_err_last[0] = 0;

    switch (cmd) {
    case BIN_PING:
        rc = snprintf((char *)out, (size_t)max, "jtag_serve_bin v1");
        return rc < 0 ? 0 : rc;
    case BIN_CMD: {
        char line[512];
        int n = len < (int)sizeof(line) - 1 ? len : (int)sizeof(line) - 1;
        memcpy(line, p, (size_t)n);
        line[n] = 0;
        return srv_text_out(sh, line, out, max);
    }
    case BIN_HALTINFO: {
        ensure_dbgmcu(sh);
        int state, reason;
        uint32_t pc = 0;
        if (core_haltinfo(sh, &state, &reason, &pc) < 0)
            goto eio;
        out[0] = (uint8_t)state;
        out[1] = (uint8_t)reason;
        put32le(out + 2, pc);
        return 6;
    }
    case BIN_HALT: {
        uint32_t dhcsr, pc = 0;
        if (!sh_cm_ready(sh, "halt"))
            goto eio;
        m = sh_mem(sh);
        if (cm_halt(m, &dhcsr) < 0)
            goto eio;
        cm_reg_read(m, 15, &pc);
        put32le(out, pc);
        return 4;
    }
    case BIN_RESUME: {
        if (!sh_cm_ready(sh, "resume"))
            goto eio;
        m = sh_mem(sh);
        /* 停在断点上 resume：比较器匹配当前 PC，放行瞬间原地再命中。
         * 先摘掉命中断点、单步越过一条指令、装回，再全速运行 */
        uint32_t pc0 = 0, d;
        cm_reg_read(m, 15, &pc0);
        int nc = fp_numcode(m), masked = 0;
        uint32_t saved[8] = {0};
        for (int i = 0; nc > 0 && i < nc && i < 8; i++) {
            uint32_t v;
            if (mem_read32(m, FP_COMP0 + 4u * i, &v) < 0)
                break;
            if ((v & 1u) && (v & 0x1FFFFFFCu) == (pc0 & 0x1FFFFFFCu)) {
                saved[i] = v;
                masked++;
                mem_write32(m, FP_COMP0 + 4u * i, 0u);
            }
        }
        if (masked) {
            mem_write32(m, DHCSR, DBGKEY | 0x5u);
            for (int t = 0; t < 100; t++) {
                if (mem_read32(m, DHCSR, &d) < 0)
                    break;
                if (d & S_HALT)
                    break;
            }
            for (int i = 0; i < 8; i++)
                if (saved[i])
                    mem_write32(m, FP_COMP0 + 4u * i, saved[i]);
        }
        if (mem_write32(m, DHCSR, DBGKEY | 0x1u) < 0)
            goto eio;
        return 0;
    }
    case BIN_STEP: {
        if (!sh_cm_ready(sh, "step"))
            goto eio;
        m = sh_mem(sh);
        int n = len >= 1 ? p[0] : 1;
        if (n < 1)
            n = 1;
        uint32_t d, pc = 0;
        cm_reg_read(m, 15, &pc);
        /* 停在断点上单步：FPB 比较器仍匹配当前 PC，一放行立刻原地再命中，
         * PC 不动形同"没效果"（实测）。先把命中当前 PC 的比较器临时摘掉，
         * 走完一步再装回 */
        int nc = fp_numcode(m);
        uint32_t saved[8] = {0};
        int saved_n = 0;
        for (int i = 0; nc > 0 && i < nc && i < 8; i++) {
            uint32_t v;
            if (mem_read32(m, FP_COMP0 + 4u * i, &v) < 0)
                break;
            if ((v & 1u) && (v & 0x1FFFFFFCu) == (pc & 0x1FFFFFFCu)) {
                saved[i] = v;
                saved_n++;
                mem_write32(m, FP_COMP0 + 4u * i, 0u);
            }
        }
        for (int k = 0; k < n; k++) {
            if (mem_write32(m, DHCSR, DBGKEY | 0x5u) < 0)
                goto eio;
            for (int t = 0; t < 100; t++) {
                if (mem_read32(m, DHCSR, &d) < 0)
                    goto eio;
                if (d & S_HALT)
                    break;
            }
        }
        for (int i = 0; i < 8; i++)          /* 装回（中途 k 步不会再命中，
                                                比较器只在起点 PC 匹配） */
            if (saved[i])
                mem_write32(m, FP_COMP0 + 4u * i, saved[i]);
        (void)saved_n;
        cm_reg_read(m, 15, &pc);
        put32le(out, pc);
        return 4;
    }
    case BIN_REG_RD: {
        if (!sh_cm_ready(sh, "reg"))
            goto eio;
        m = sh_mem(sh);
        if (sh_halted(sh) != 1) {
            i = snprintf((char *)out, (size_t)max, "target is running");
            return -(i < 0 ? 0 : i) - 1000;   /* 特判：非错误文本 */
        }
        if (len * 4 > max)
            goto etoolong;
        for (i = 0; i < len; i++) {
            uint32_t v;
            if (cm_reg_read(m, p[i], &v) < 0)
                goto eio;
            put32le(out + 4 * i, v);
        }
        return len * 4;
    }
    case BIN_REG_WR: {
        if (len < 5)
            goto elen;
        if (!sh_cm_ready(sh, "reg"))
            goto eio;
        uint32_t v = rd32le(p + 1);
        if (cm_reg_write(sh_mem(sh), p[0], v) < 0)
            goto eio;
        return 0;
    }
    case BIN_MEM_RD: {
        if (len < 7)
            goto elen;
        if (!sh_cm_ready(sh, "mdw"))
            goto eio;
        m = sh_mem(sh);
        uint32_t addr = rd32le(p);
        int w = p[4];
        unsigned cnt = (unsigned)p[5] | ((unsigned)p[6] << 8);
        if (w != 1 && w != 2 && w != 4 && w != 8)
            goto ebad;
        if (cnt == 0 || cnt * (unsigned)w > (unsigned)max)
            goto etoolong;
        for (unsigned k = 0; k < cnt; k++) {
            uint64_t a = addr + (uint64_t)w * k;
            if (w == 8) {
                /* 8 字节：两次 32 位读合并（小端低字在前） */
                uint32_t lo, hi;
                if (mem_read32(m, a, &lo) < 0)
                    goto eio;
                if (mem_read32(m, a + 4, &hi) < 0)
                    goto eio;
                uint64_t v = (uint64_t)lo | ((uint64_t)hi << 32);
                for (int b = 0; b < 8; b++)
                    out[k * 8 + (unsigned)b] = (uint8_t)(v >> (8 * b));
            } else {
                uint32_t v;
                int r = w == 4 ? mem_read32(m, a, &v) :
                        w == 2 ? mem_read16(m, a, &v) :
                                 mem_read8(m, a, &v);
                if (r < 0)
                    goto eio;
                for (int b = 0; b < w; b++)
                    out[k * (unsigned)w + (unsigned)b] = (uint8_t)(v >> (8 * b));
            }
        }
        return (int)(cnt * (unsigned)w);
    }
    case BIN_MEM_WR: {
        if (len < 7)
            goto elen;
        if (!sh_cm_ready(sh, "mww"))
            goto eio;
        m = sh_mem(sh);
        uint32_t addr = rd32le(p);
        int w = p[4];
        unsigned cnt = (unsigned)p[5] | ((unsigned)p[6] << 8);
        if (w != 1 && w != 2 && w != 4)
            goto ebad;
        if (4 + (int)(cnt * (unsigned)w) > len)
            goto elen;
        const uint8_t *d = p + 7;
        for (unsigned k = 0; k < cnt; k++) {
            uint32_t v = 0;
            for (int b = 0; b < w; b++)
                v |= (uint32_t)d[k * (unsigned)w + (unsigned)b] << (8 * b);
            int r = w == 4 ? mem_write32(m, addr + (uint64_t)w * k, v) :
                    w == 2 ? mem_write16(m, addr + (uint64_t)w * k, v) :
                             mem_write8(m, addr + (uint64_t)w * k, v);
            if (r < 0)
                goto eio;
        }
        return 0;
    }
    case BIN_BP_ADD: {
        if (len < 5)
            goto elen;
        int comp = 0;
        uint32_t ln = p[4] == 4 ? 4u : 2u;
        if (core_bp_add(sh, rd32le(p), ln, &comp) < 0)
            goto eio;
        out[0] = (uint8_t)comp;
        return 1;
    }
    case BIN_BP_DEL: {
        if (len < 4)
            goto elen;
        int removed = 0, all = rd32le(p) == 0xFFFFFFFFu;
        if (core_bp_del(sh, rd32le(p), all, &removed) < 0 && !all)
            goto eio;
        out[0] = (uint8_t)removed;
        return 1;
    }
    case BIN_WP_ADD: {
        if (len < 9)
            goto elen;
        int comp = 0, acc = p[8];
        if (core_wp_add(sh, rd32le(p), rd32le(p + 4), acc, &comp) < 0)
            goto eio;
        out[0] = (uint8_t)comp;
        return 1;
    }
    case BIN_WP_DEL: {
        if (len < 4)
            goto elen;
        int removed = 0, all = rd32le(p) == 0xFFFFFFFFu;
        if (core_wp_del(sh, rd32le(p), all, &removed) < 0 && !all)
            goto eio;
        out[0] = (uint8_t)removed;
        return 1;
    }
    case BIN_BPS: {
        if (!sh_cm_ready(sh, "bps"))
            goto eio;
        m = sh_mem(sh);
        int nc = fp_numcode(m), o = 0, nb = 0;
        uint8_t *cntp = out;        /* 先占位数，最后回填 */
        o = 1;
        for (i = 0; nc > 0 && i < nc && o + 6 <= max; i++) {
            uint32_t v;
            if (mem_read32(m, FP_COMP0 + 4u * i, &v) < 0)
                break;
            if (v & 1u) {
                put32le(out + o, v & 0x1FFFFFFCu);
                out[o + 4] = ((v >> 30) & 3u) == 3u ? 4 : 2;
                o += 5;
                nb++;
            }
        }
        *cntp = (uint8_t)nb;
        cntp = out + o;
        o++;
        int nd = dwt_numcomp(m), nw = 0;
        for (i = 0; nd > 0 && i < nd && o + 10 <= max; i++) {
            uint32_t base = DWT_COMP0 + 0x10u * i, comp, msk, fn;
            if (mem_read32(m, base, &comp) < 0 ||
                mem_read32(m, base + 4u, &msk) < 0 ||
                mem_read32(m, base + 8u, &fn) < 0)
                break;
            if (fn >= 5u && fn <= 7u) {
                put32le(out + o, comp);
                put32le(out + o + 4, 1u << msk);
                out[o + 8] = (uint8_t)fn;
                o += 9;
                nw++;
            }
        }
        *cntp = (uint8_t)nw;
        return o;
    }
    case BIN_SWO_TPIU: {
        if (len < 8)
            goto elen;
        uint32_t actual = core_swo_tpiu(sh, rd32le(p), rd32le(p + 4));
        if (!actual)
            goto eio;
        put32le(out, actual);
        return 4;
    }
    case BIN_SWO_STAT: {
        int cnt, ovr, fe;
        j_swo_stat(&sh->jtag, &cnt, &ovr, &fe);
        if (ovr || fe)
            j_swo_clear_err(&sh->jtag);
        out[0] = (uint8_t)(cnt & 0xFF);
        out[1] = (uint8_t)((cnt >> 8) & 0xFF);
        out[2] = (uint8_t)ovr;
        out[3] = (uint8_t)fe;
        return 4;
    }
    case BIN_REPROBE: {
        core_reprobe(sh);
        if (sh_dap(sh) && sh_cm_ready(sh, "reprobe")) {
            ensure_dbgmcu(sh);
            rc = snprintf((char *)out, (size_t)max, "reconnected AP%d", sh->apsel);
            return rc < 0 ? 0 : rc;
        }
        err("reprobe: 目标未应答");
        goto eio;
    }
    case BIN_VREF: {
        int mv = vref_read_mv("/dev/spidev0.0");
        if (mv < 0)
            goto eio;
        put32le(out, (uint32_t)mv);
        return 4;
    }
    default:
        i = snprintf((char *)out, (size_t)max, "unknown cmd 0x%02X", cmd);
        goto eout;
    }
eio:
    i = snprintf((char *)out, (size_t)max, "cmd 0x%02X: %s", cmd,
                 g_err_last[0] ? g_err_last : "I/O error");
    goto eout;
ebad:
    i = snprintf((char *)out, (size_t)max, "bad width (cmd 0x%02X)", cmd);
    goto eout;
elen:
    i = snprintf((char *)out, (size_t)max, "bad payload len (cmd 0x%02X)", cmd);
    goto eout;
etoolong:
    i = snprintf((char *)out, (size_t)max, "payload too large (cmd 0x%02X)", cmd);
eout:
    return -(i < 0 ? 0 : i) - 1;
}

static void srv_send(int fd, uint8_t seq, uint8_t cmd, const uint8_t *pl,
                     int len)
{
    uint8_t hdr[4] = { seq, cmd, (uint8_t)(len & 0xFF),
                       (uint8_t)((len >> 8) & 0xFF) };
    struct iovec iov[2] = {
        { hdr, 4 }, { (void *)pl, (size_t)len }
    };
    (void)!writev(fd, iov, len > 0 ? 2 : 1);
}

static void srv_run(shell_t *sh, int cmd_port)
{
    signal(SIGPIPE, SIG_IGN);
    int cmd_lfd = srv_listener(cmd_port);
    int swo_lfd = srv_listener(cmd_port + 1);
    if (cmd_lfd < 0 || swo_lfd < 0) {
        fprintf(stderr, "serve: 端口 %d/%d 绑定失败: %s\n",
                cmd_port, cmd_port + 1, strerror(errno));
        return;
    }
    printf("serve: 二进制命令口 :%d（[seq][cmd][len16]+载荷）  SWO 流口 :%d\n",
           cmd_port, cmd_port + 1);
    int cmd_fd = -1, swo_fd = -1;
    int eio_streak = 0;
    static uint8_t rbuf[BIN_MAXPL + 8];
    int rlen = 0;
    static uint8_t out[BIN_MAXPL + 64];
    static uint8_t swo_buf[SWO_FIFO_DEPTH];

    for (;;) {
        fd_set rf;
        FD_ZERO(&rf);
        FD_SET(cmd_lfd, &rf);
        FD_SET(swo_lfd, &rf);
        if (cmd_fd >= 0)
            FD_SET(cmd_fd, &rf);
        if (swo_fd >= 0)
            FD_SET(swo_fd, &rf);   /* EOF 探活：纯抽头也随 FIN 可读 */
        struct timeval tv = { 0, 800 };    /* 800µs：SWO FIFO 1KB @4M 满 2.05ms，
                                              留 2.5 倍余量防 select 抖动 overrun */
        int mx = cmd_fd > cmd_lfd ? cmd_fd : cmd_lfd;
        if (swo_fd > mx)
            mx = swo_fd;
        if (swo_lfd > mx)
            mx = swo_lfd;
        if (select(mx + 1, &rf, NULL, NULL, &tv) < 0) {
            if (errno == EINTR)
                continue;
            perror("serve: select");
            break;
        }
        /* 新连接：每口单客户端，多余的当场拒 */
        if (FD_ISSET(cmd_lfd, &rf)) {
            int c = accept(cmd_lfd, NULL, NULL);
            if (c >= 0) {
                if (cmd_fd >= 0) {
                    close(c);
                } else {
                    cmd_fd = c;
                    rlen = 0;
                    int one = 1;
                    setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
                    printf("serve: 命令客户端接入\n");
                }
            }
        }
        if (FD_ISSET(swo_lfd, &rf)) {
            int c = accept(swo_lfd, NULL, NULL);
            if (c >= 0) {
                if (swo_fd >= 0) {
                    /* 最新连接接管：SWO 口是纯数据抽头，旧槽可能是没被
                     * FIN/写错误及时收割的死连接，硬拒会让唯一客户端
                     * （GUI 重连）永久闪断 */
                    close(swo_fd);
                    printf("serve: SWO 客户端被新连接接管\n");
                }
                swo_fd = c;
                int one = 1;
                static const int ka_idle = 10, ka_intvl = 2, ka_cnt = 3;
                setsockopt(c, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
                setsockopt(c, IPPROTO_TCP, TCP_KEEPIDLE,
                           &ka_idle, sizeof(ka_idle));
                setsockopt(c, IPPROTO_TCP, TCP_KEEPINTVL,
                           &ka_intvl, sizeof(ka_intvl));
                setsockopt(c, IPPROTO_TCP, TCP_KEEPCNT,
                           &ka_cnt, sizeof(ka_cnt));
                /* RX 已开则重使能（0→1 沿 flush），客户端拿到全新数据 */
                if (sh->swo_baud)
                    j_swo_enable(&sh->jtag, sh->swo_baud);
                printf("serve: SWO 客户端接入\n");
            }
        }
        /* SWO 客户端探活：对端 close 的 FIN 随可读上报，read 0 即收割；
         * SWO 静默期（目标 halt）也照常检测，不再占死单客户端槽位 */
        if (swo_fd >= 0 && FD_ISSET(swo_fd, &rf)) {
            ssize_t n = read(swo_fd, swo_buf, 64);
            if (n <= 0) {
                close(swo_fd);
                swo_fd = -1;
                printf("serve: SWO 客户端断开\n");
            }
            /* n>0：客户端不应发数据，丢弃 */
        }
        /* 二进制命令：解析尽量多的完整包，逐包应答 */
        if (cmd_fd >= 0 && FD_ISSET(cmd_fd, &rf)) {
            ssize_t n = read(cmd_fd, rbuf + rlen, (size_t)(BIN_MAXPL - rlen));
            if (n <= 0) {
                close(cmd_fd);
                cmd_fd = -1;
                printf("serve: 命令客户端断开\n");
            } else {
                rlen += (int)n;
                while (rlen >= 4) {
                    uint8_t seq = rbuf[0], cmd = rbuf[1];
                    unsigned plen = (unsigned)rbuf[2] | ((unsigned)rbuf[3] << 8);
                    if (plen > BIN_MAXPL) {          /* 帧损坏：丢弃缓冲重同步 */
                        rlen = 0;
                        break;
                    }
                    if (rlen < 4 + (int)plen)
                        break;
                    int olen = srv_binary(sh, cmd, rbuf + 4, (int)plen,
                                          out, (int)sizeof(out));
                    /* 连续命令级失败 = target 链路卡死（复位/掉电后 SWD
                     * 无应答）：自动全量重探，恢复后无需人工干预 */
                    if (olen < 0) {
                        if (++eio_streak >= 8) {
                            eio_streak = 0;
                            printf("serve: 连续失败，自动 reprobe target\n");
                            core_reprobe(sh);
                        }
                    } else {
                        eio_streak = 0;
                    }
                    if (olen >= 0)
                        srv_send(cmd_fd, seq, cmd | 0x80, out, olen);
                    else if (olen <= -1000)          /* "软错误"：负号+偏移 */
                        srv_send(cmd_fd, seq, BIN_ERR, out, -olen - 1000);
                    else
                        srv_send(cmd_fd, seq, BIN_ERR, out, -olen - 1);
                    memmove(rbuf, rbuf + 4 + plen,
                            (size_t)(rlen - 4 - (int)plen));
                    rlen -= 4 + (int)plen;
                }
            }
        }
        /* SWO 排水直推 */
        if (swo_fd >= 0) {
            int len = j_swo_read(&sh->jtag, swo_buf, (int)sizeof(swo_buf));
            if (len > 0 && write(swo_fd, swo_buf, (size_t)len) < 0) {
                close(swo_fd);
                swo_fd = -1;
                printf("serve: SWO 客户端断开\n");
            }
            /* OVR 后 STAT.CNT 与 FIFO pop 脱钩（RTL 实测：count 卡 1024、
             * pop 无效、流静默死）：见 OVR 即重使能 RX（0→1 沿 flush）自愈。
             * 先排水再 flush，能取的先取走 */
            if (sh->swo_baud && (jrd(&sh->jtag, R_SWO_STAT) & SWO_STAT_OVR)) {
                j_swo_enable(&sh->jtag, sh->swo_baud);
                printf("serve: SWO overrun 自愈（RX flush）\n");
            }
        }
    }
}

int main(int argc, char **argv)
{
    const char *dev = "/dev/mem";
    uint32_t base = 0x43C00000u;
    uint32_t axi_hz = 125000000u;   /* 本板 FCLK0；VERSION 自动探测截断成 124M */
    int speed = 10000, apsel = 0;
    int serve_port = 0;
    int use_swd = 0;
    const char *ccmds[64];
    int ncc = 0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--base") && i + 1 < argc)
            base = (uint32_t)parse_num(argv[++i]);
        else if (!strcmp(argv[i], "--speed") && i + 1 < argc)
            speed = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--dev") && i + 1 < argc)
            dev = argv[++i];
        else if (!strcmp(argv[i], "--axi-hz") && i + 1 < argc)
            axi_hz = (uint32_t)parse_num(argv[++i]);
        else if (!strcmp(argv[i], "--ap") && i + 1 < argc)
            apsel = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--serve")) {
            serve_port = 5555;
            if (i + 1 < argc && argv[i + 1][0] >= '0' && argv[i + 1][0] <= '9')
                serve_port = atoi(argv[++i]);
        }
        else if (!strcmp(argv[i], "--swd"))
            use_swd = 1;
        else if (!strcmp(argv[i], "-c") && i + 1 < argc && ncc < 64)
            ccmds[ncc++] = argv[++i];
        else {
            fprintf(stderr,
                    "用法: %s [--base 0x43C00000] [--speed 10000] [--dev /dev/mem]"
                    " [--axi-hz 125000000] [--ap 0] [--serve [port]] [-c CMD]...\n"
                    "  --serve [port]  网络服务器模式：命令口 port(默认5555) + SWO 流口 port+1\n",
                    argv[0]);
            return 1;
        }
    }

    static shell_t sh;
    memset(&sh, 0, sizeof(sh));
    sh.apsel = apsel;
    sh.swd = use_swd || serve_port;   /* serve 一律 SWD：SWO 需关目标 JTAG 口 */
    if (j_open(&sh.jtag, dev, base, speed, axi_hz) < 0)
        return 1;

    if (serve_port) {
        /* 直调底层（不走 execute 文本层）：DAP 探测 + 睡眠调试使能 */
        sh_dap(&sh);
        ensure_dbgmcu(&sh);
        srv_run(&sh, serve_port);
        return 0;
    }

    if (ncc) {
        for (int i = 0; i < ncc; i++) {
            printf("jtag> %s\n", ccmds[i]);
            execute(&sh, (char *)ccmds[i]);
        }
        return 0;
    }

    printf("jtag_tool shell（C 版）— help 查看命令；quit / Ctrl+D 退出\n");
    char fline[256];
#ifndef NO_READLINE
    int tty = isatty(STDIN_FILENO);
#endif
    for (;;) {
        char *line;
#ifndef NO_READLINE
        if (tty) {
            line = readline("jtag> ");
            if (!line)
                break;
            if (*line)
                add_history(line);
        } else
#endif
        {
            if (!fgets(fline, sizeof(fline), stdin))
                break;
            fline[strcspn(fline, "\r\n")] = 0;
            line = fline;
        }
        int quit = !strcmp(line, "quit") || !strcmp(line, "exit") ||
                   !strcmp(line, "q");
        if (!quit && line[0] && line[0] != '#')
            execute(&sh, line);
        if (line != fline)
            free(line);
        if (quit)
            break;
    }
    return 0;
}
