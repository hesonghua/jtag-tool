# jtag_tool — On-Board JTAG/SWD Debugger

Turns a Xilinx Zynq FPGA board into a standalone JTAG/SWD debug probe. Runs
directly on the Zynq's ARM cores, talking to a custom PL IP (`jtag_swd_dbg`)
that implements an MPSSE-style batch shift engine — no external probe needed.

## Features

- **Interactive shell** (52+ commands) + CLI subcommands + `-c` script mode
- **Network server mode** (`--serve`): binary command port + raw SWO stream port
- **SWD native**: SWDIO bidirectional with per-bit direction, DP/AP access
- **Cortex-M support**: halt/resume/step, register read/write, FPB hardware
  breakpoints, DWT watchpoints, memory read/write (8/16/32-bit)
- **SWO receiver**: v4 fractional-N CDR for exact frequency matching at any
  baud rate (500k–12M+), with overrun auto-recovery
- **Chain scanning**: multi-TAP topology probe, LPAE 64-bit addressing
- **Auto-recovery**: reprobe on consecutive failures, step-over-breakpoint

## Build

```bash
# Cross-compile (needs ARM cross toolchain, links against buildroot staging readline)
make              # → ../overlay/root/jtag_tool

# Host syntax check (no readline)
make host         # → jtag_tool_host

# Or use your own toolchain
CC=arm-linux-gnueabihf-gcc make
```

## Usage

```bash
# Interactive shell (on the Zynq board, as root)
./jtag_tool

# One-shot commands
./jtag_tool -c probe -c "mdw 0x20000000 4"

# Network server mode (swo_web GUI backend)
./jtag_tool --serve            # cmd=:5555  SWO=:5556

# Custom port
./jtag_tool --serve 9876
```

## Server Protocol

Binary framed protocol on the command port:
```
[seq:1][cmd:1][len:2 LE][payload:len]
```
Response uses `cmd | 0x80`; errors use `0x7F` + text.

| cmd | name | payload |
|-----|------|---------|
| 0x01 | PING | — |
| 0x02 | CMD (text) | shell command line |
| 0x03 | HALTINFO | — |
| 0x04 | HALT | — |
| 0x05 | RESUME | — |
| 0x06 | STEP | [count:1] |
| 0x07 | REG_READ | [sel:N] |
| 0x09 | MEM_READ | [addr:4][width:1][count:2] |
| 0x0A | MEM_WRITE | [addr:4][width:1][count:2]+data |
| 0x0B | BP_ADD | [addr:4][len:1] |
| 0x0F | BPS (list) | — |
| 0x10 | SWO_TPIU | [traceclk:4][baud:4] |
| 0x11 | SWO_STAT | — |
| 0x12 | REPROBE | — |

SWO port delivers raw ITM/TPIU bytes from the IP FIFO, drained every 800µs.

## Files

| File | Description |
|------|-------------|
| `jtag_tool.c` | C implementation (performance version) |
| `jtag_tool.py` | Python reference implementation |
| `Makefile` | Build for ARM target / host check |

## Hardware Requirements

- Xilinx Zynq-7000 board with `jtag_swd_dbg` IP (v2+) in the PL
- IP register base typically at `0x43C00000` (configurable via `--base`)
- AXI clock reported by IP VERSION register; override with `--axi-hz` if
  the auto-detected value is truncated (e.g. 125 MHz reported as 124M)
