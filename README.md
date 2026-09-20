# jtag_tool — On-Board JTAG/SWD Debugger

Turns a Xilinx Zynq FPGA board into a standalone debug probe. No external
J-Link/FTDI needed — the tool runs directly on the Zynq's ARM cores and
drives a custom PL IP (`jtag_swd_dbg`) that implements an MPSSE-style
batch shift engine for JTAG and SWD protocols.

```
┌────────────── Zynq Board ──────────────┐
│  ARM (Linux)                            │
│    jtag_tool ←── TCP :5555/:5556 ──→ PC│
│       │ mmap                            │
│  PL: jtag_swd_dbg IP @0x43C00000        │
│       ├── JTAG/SWD shift engine         │
│       ├── SWO receiver (v4 frac-N CDR)  │
│       └── FIFO (256B JTAG / 1KB SWO)    │
└──────────┬──────────────────────────────┘
           │ TCK/TDI/TDO/TMS/SRST pins
           ▼
     Target MCU (Cortex-M)
```

## What It Can Do

### Target Control
- Halt, resume, single-step (×1/×N)
- Read/write all core registers (r0–r12, sp, lr, pc, xpsr, msp, psp, primask, etc.)
- Reset target (SYSRESETREQ), halt-on-reset
- Detect halt reason (debug-request / breakpoint / watchpoint / vector-catch)

### Memory Access
- Read/write memory at 8/16/32-bit widths via MEM-AP
- Symbol name lookup (when ELF provided by frontend)
- LPAE 64-bit address pass-through for targets >4GB address space

### Breakpoints & Watchpoints
- FPB hardware instruction breakpoints (up to 6 on Cortex-M3 r1p1)
- DWT data watchpoints (read/write/access, up to 4 comparators)
- Idempotent add (same address → reuse comparator, no duplicates)
- Step-over-breakpoint (temporarily disables matching FPB comparator)
- Resume-past-breakpoint (steps one instruction past before resuming)

### SWO Trace Receiver
- UART/NRZ protocol, any baud rate (500k–12M+)
- v4 fractional-N CDR: exact frequency matching for non-integer AXI/trace
  clock ratios (e.g. 125M/4M = 31.25 → integer 31 + fraction 0.25)
- Overrun auto-recovery (RX flush on sticky OVR)
- Target-side auto-configuration (ITM/TPIU/DBGMCU/SWJ_CFG via AP writes)
- 10s silence watchdog auto-recovery (target reset detection)

### Network Server Mode (`--serve`)
- **Port 5555**: Binary framed command protocol (see below)
- **Port 5556**: Raw SWO byte stream (ITM/TPIU output, drained every 800µs)
- Single client per port; SWO port supports client replacement
- Auto-reprobe on 8 consecutive command failures

### Chain Scanning & Bit-Level Control
- Multi-TAP topology probe (chain scan, IDCODE detection)
- Bit-level TMS/TDI/TDO manipulation for non-standard protocols
- SRST control (active-low pin)
- Three-topology DAP probe (for non-standard JTAG-DP configurations)

## Shell Commands (52+)

```
# Target control
halt, resume, step [n], haltinfo, reset

# Memory
mdw/mdh/mdb <addr> [count]    # read word/half/byte
mww/mwh/mwb <addr> <val>       # write
reg <name>                     # read core register
reg <name> <val>               # write core register
regs                           # dump all registers

# Breakpoints & watchpoints
bp <addr> [len]                # add hardware breakpoint
rbp <addr|all>                 # remove breakpoint
wp <addr> <len> <r|w|a>       # add watchpoint
rwp <addr|all>                 # remove watchpoint
bps                            # list all breakpoints/watchpoints

# SWO trace
swo_tpiu <traceclk> <baud>     # configure target ITM/TPIU + local RX
swo_stat                       # FIFO count + overrun/frame_err
swo_read [n]                   # read n bytes (hex dump)
swo_stop                       # disable SWO receiver

# JTAG/SWD low-level
probe                          # scan chain / detect DAP
ap [n]                         # select AP domain
dpr/dpw <reg> [val]            # DP register read/write
apr/apw <bank> <reg> [val]     # AP register read/write
dapinfo                        # CoreSight ROM table discovery

# Bit-level (private TAP exploration)
tbit, tms, tdr, tir, tdo      # single-bit JTAG ops
rst <0|1>                      # SRST assert/release
speed <khz>                    # TCK frequency

# Utility
reprobe                        # full SWD reconnect
help                           # full command list
```

## Build

```bash
# Cross-compile (needs ARM cross toolchain)
make              # → binary in overlay/root/jtag_tool

# Host syntax check (no readline dependency)
make host         # → jtag_tool_host

# Or specify your own toolchain
CC=arm-linux-gnueabihf-gcc make
```

## Usage

```bash
# Interactive shell (on the board, as root)
./jtag_tool
jtag> probe
jtag> halt
jtag> mdw 0x20000000 4
jtag> resume

# One-shot commands
./jtag_tool -c probe -c "mdw 0x20000000 4" -c resume

# Network server (for swo_web GUI)
./jtag_tool --serve            # cmd=:5555  SWO=:5556
./jtag_tool --serve 9876       # custom port
```

## Server Binary Protocol

Command port frame format:
```
[seq:1][cmd:1][len:2 LE][payload:len]
```
Response: `cmd | 0x80`; errors: `0x7F` + text.

| cmd | name | payload |
|-----|------|---------|
| 0x01 | PING | — |
| 0x02 | CMD (text) | shell command line |
| 0x03 | HALTINFO | — |
| 0x04 | HALT | — |
| 0x05 | RESUME | — |
| 0x06 | STEP | [count:1] |
| 0x07 | REG_READ | [sel:N] |
| 0x08 | REG_WRITE | [sel:1][val:4] |
| 0x09 | MEM_READ | [addr:4][width:1][count:2] |
| 0x0A | MEM_WRITE | [addr:4][width:1][count:2]+data |
| 0x0B | BP_ADD | [addr:4][len:1] |
| 0x0C | BP_DEL | [addr:4 or FFFFFFFF] |
| 0x0D | WP_ADD | [addr:4][len:4][acc:1] |
| 0x0E | WP_DEL | [addr:4 or FFFFFFFF] |
| 0x0F | BPS (list) | — |
| 0x10 | SWO_TPIU | [traceclk:4][baud:4] |
| 0x11 | SWO_STAT | — |
| 0x12 | REPROBE | — |

SWO port delivers raw ITM bytes from IP FIFO every 800µs (1KB buffer).

## Files

| File | Description |
|------|-------------|
| `jtag_tool.c` | C implementation — the primary binary (~3800 lines) |
| `jtag_tool.py` | Python reference implementation (~2000 lines) |
| `Makefile` | Cross-compile + host check targets |

## Hardware Requirements

- Xilinx Zynq-7000 with `jtag_swd_dbg` IP (HW_VERSION ≥ 0x0002) in PL
- IP base address configurable via `--base` (default `0x43C00000`)
- AXI clock auto-detected from VERSION; override with `--axi-hz` if truncated
- Target MCU connected via JTAG or SWD pins (TCK/TDI/TDO/TMS/SRST)
