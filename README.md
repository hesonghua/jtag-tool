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

## Server Binary Protocol (API)

### Command Port (default :5555)

**Frame format** (bidirectional):
```
[seq:1][cmd:1][len:2 LE][payload:len bytes]
```

**Response**: `cmd | 0x80` with same `seq`. Errors: `0x7F` + human-readable text.

**Convenience**: `seq` auto-increments per connection; server echoes it back for request-response matching.

#### Commands

| cmd | name | request payload | response payload | description |
|-----|------|----------------|-----------------|-------------|
| 0x01 | PING | — | version string | Connection test / version query |
| 0x02 | CMD | text line (≤200B) | text output | Execute any shell command (same as interactive) |
| 0x03 | HALTINFO | — | [state:1][reason:1][pc:4] | Target status poll (read-clears DFSR) |
| 0x04 | HALT | — | [pc:4] | Halt target, returns halt PC |
| 0x05 | RESUME | — | — | Resume target (steps over bp if needed) |
| 0x06 | STEP | [count:1] (1-16) | [pc:4] | Single-step N instructions (masks bp at current PC) |
| 0x07 | REG_READ | [sel:1 × N] | [val:4 × N] | Read N core registers by selector ID |
| 0x08 | REG_WRITE | [sel:1][val:4] | — | Write one core register |
| 0x09 | MEM_READ | [addr:4][width:1][count:2] | raw bytes (count × width) | Read memory; width: 1/2/4/8 |
| 0x0A | MEM_WRITE | [addr:4][width:1][count:2]+data | — | Write memory; width: 1/2/4/8 |
| 0x0B | BP_ADD | [addr:4][len:1] | [comp:1] | Add FPB breakpoint (len 2 or 4); idempotent |
| 0x0C | BP_DEL | [addr:4 or FFFFFFFF=all] | [removed:1] | Remove breakpoint |
| 0x0D | WP_ADD | [addr:4][len:4][acc:1] | [comp:1] | Add DWT watchpoint (acc: 5=r 6=w 7=a) |
| 0x0E | WP_DEL | [addr:4 or FFFFFFFF=all] | [removed:1] | Remove watchpoint |
| 0x0F | BPS | — | [nb:1]([addr:4][len:1])× [nw:1]([addr:4][len:4][acc:1])× | List all breakpoints/watchpoints |
| 0x10 | SWO_TPIU | [traceclk:4][baud:4] | [actual_baud:4] | Configure target ITM/TPIU + local SWO RX |
| 0x11 | SWO_STAT | — | [cnt:2][ovr:1][fe:1] | SWO FIFO status (auto-clears sticky errors) |
| 0x12 | REPROBE | — | "reconnected AP*n" / error text | Full SWD reconnect (recovery) |

#### Register Selector IDs (for 0x07/0x08)

| sel | register | sel | register | sel | register |
|-----|----------|-----|----------|-----|----------|
| 0-12 | r0-r12 | 13 | sp (msp) | 14 | lr |
| 15 | pc | 16 | xpsr (flags) | 17 | msp |
| 18 | psp | 20 | primask | 21 | basepri |
| 22 | faultmask | 23 | control | | |

#### HALTINFO reason codes

| value | meaning |
|-------|---------|
| 0 | running (no halt) |
| 1 | debug-request (manual halt) |
| 2 | breakpoint (FPB hit) |
| 3 | watchpoint (DWT hit) |
| 4 | vector-catch |
| 5 | external reset |

#### Example: Read PC and registers in one batch

```python
import socket, struct

sock = socket.create_connection(("board", 5555))
seq = 0

def xchg(cmd, payload=b""):
    global seq
    seq += 1
    sock.sendall(bytes([seq & 0xFF, cmd, len(payload), 0]) + payload)
    hdr = b""
    while len(hdr) < 4: hdr += sock.recv(4 - len(hdr))
    rlen = hdr[2] | hdr[3] << 8
    body = b""
    while len(body) < rlen: body += sock.recv(rlen - len(body))
    return body

# Halt
pc_bytes = xchg(0x04)
pc = struct.unpack("<I", pc_bytes)[0]

# Read r0-r3 + pc + xpsr (6 registers)
body = xchg(0x07, bytes([0, 1, 2, 3, 15, 16]))
vals = struct.unpack("<6I", body)
print(f"pc=0x{vals[4]:08x} xpsr=0x{vals[5]:08x} r0=0x{vals[0]:08x}")

# Read 16 words from SRAM
data = xchg(0x09, struct.pack("<IBH", 0x20000000, 4, 16))

# Resume
xchg(0x05)
```

### SWO Stream Port (default :5556)

Raw byte stream from the SWO receiver IP FIFO, drained every 800µs.

- **No framing** — just raw ITM/TPIU bytes (NRZ/UART decoded)
- **Single client** — new connection replaces old (client replacement)
- **TCP keepalive** enabled on server side (idle detection)
- FIFO: 1KB in IP, auto-flushes on overrun

#### Connecting
```python
swo = socket.create_connection(("board", 5556))
while True:
    data = swo.recv(4096)  # raw ITM bytes
    if not data: break
    parse_itm(data)
```

### Error Handling

- All errors return `0x7F` + descriptive text (e.g., `"cmd 0x0B: FPB comparator full (6 units)"`)
- After 8 consecutive command failures, server auto-reprobes the SWD connection
- TCP connection loss → client should reconnect and re-send `SWO_TPIU` to reconfigure

## Files

| File | Description |
|------|-------------|
| `jtag_tool.c` | C implementation — the primary binary (~3800 lines) |
| `jtag_tool.py` | Python reference implementation (~2000 lines) |
| `target_port.h` | Target MCU porting layer (vendor-specific config) |
| `Makefile` | Cross-compile + host check targets |

## Target Compatibility

### Works Out of the Box

| Component | M3 | M4 | Why |
|-----------|----|----|-----|
| SWD/JTAG protocol | ✅ | ✅ | ADIv5 is ARM standard |
| halt/resume/step | ✅ | ✅ | DHCSR/DFSR same |
| Register read/write | ✅ | ✅ | DCRSR/DCRDR same |
| Memory access (8/16/32/64-bit) | ✅ | ✅ | AHB-AP standard |
| FPB breakpoints | ✅ | ✅ | rev1 encoding compatible |
| DWT watchpoints | ✅ | ✅ | 4 comparators, same |
| ITM/TPIU trace | ✅ | ✅ | ARM standard |
| SWO receiver | ✅ | ✅ | Protocol identical |
| FPU registers | N/A | ✅ | Only integer regs accessed |

### Porting Layer (`target_port.h`)

Vendor-specific configuration is isolated in a single file:

| Setting | STM32F1 (current) | GD32F1/CH32F1 | STM32F4/L4/H7 | NXP LPC |
|---------|-------------------|---------------|---------------|---------|
| DBGMCU | `0xE0042004`, `0x27` | Same (clone) | Same addr, check bits | Not needed (`TGT_HAS_DBGMCU=0`) |
| SWO pin mux | AFIO_MAPR + SWJ_CFG | Same (clone) | GPIO AFR mechanism | Default active (`TGT_HAS_AFIO=0`) |

**To port**: edit `target_port.h` only. Core code (`jtag_tool.c`) needs zero changes.

### M4 Bonus

DWT `POSTPRESET` is programmable on M4 (fixed ~1024 on M3), enabling
higher PC sampling rates (~280K/s vs ~70K/s at 72MHz).

## Hardware Requirements

- Xilinx Zynq-7000 with `jtag_swd_dbg` IP (HW_VERSION ≥ 0x0002) in PL
- IP base address configurable via `--base` (default `0x43C00000`)
- AXI clock auto-detected from VERSION; override with `--axi-hz` if truncated
- Target MCU connected via JTAG or SWD pins (TCK/TDI/TDO/TMS/SRST)
