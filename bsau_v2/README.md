# BSAU — Bio-Signal Acquisition Unit

**The transmitter half of the prosthetic hand system.** An STM32L432KC
Nucleo-32 board that samples 8 EMG electrodes at 2 kHz, packs 2 samples
per channel into a 32-byte wireless frame, and fires 1000 packets per
second over an NRF24L01+ radio to the CPCU.

This directory is a complete STM32CubeIDE project. It compiles to a
single firmware image whose behaviour is controlled at build time by
one `#define` in `Core/Inc/bsau_config.h`.

> **First time with this repo?** Start from the root
> [`SYSTEM_GUIDE.md`](../SYSTEM_GUIDE.md) — it covers both halves of the
> system end to end. This README is the BSAU-specific quick reference.

---

## Hardware

| Item | Part | Notes |
|------|------|-------|
| MCU board | **NUCLEO-L432KC** | Cortex-M4F @ 80 MHz, on-board ST-LINK/V2-1 |
| Radio | **NRF24L01+** on SPI1 (5 MHz) | 2 Mbps, channel 76 (2.476 GHz), addr `0xE7×5` |
| EMG front-end | 8× INA333 (or AD8221) | Vref = 1.65 V, outputs to PA0–PA7 |
| Battery monitor | 100 kΩ / 100 kΩ divider on PB0 | Reads via ADC rank 9 |
| Status LED | On-board LD3 (PA11) | 1 kHz blink = main loop is alive |

Full wiring tables with silkscreen labels and multimeter pre-checks
live in [`../SYSTEM_GUIDE.md` §3.2](../SYSTEM_GUIDE.md).

---

## Build modes

`Core/Inc/bsau_config.h` must have **exactly one** `BSAU_MODE_*` macro
uncommented. The compiler emits `#error "Define exactly one
BSAU_MODE_*."` if you leave two or none active.

| Mode | Radio | LOG | CSV | Purpose |
|------|-------|-----|-----|---------|
| `BSAU_MODE_RELEASE` | TX | off | off | **Production.** Zero debug overhead |
| `BSAU_MODE_DEBUG` | TX | on | on | Dev: periodic status lines + decimated CSV |
| `BSAU_MODE_TEST_ADC_CSV` | off | off | binary | ADC binary stream for SerialPlot |
| `BSAU_MODE_TEST_PKT_LOG` | off | on | off | Codec round-trip verification (TB-104) |
| `BSAU_MODE_TEST_CSV` | off | off | ASCII | ADC CSV with drop counter (TB-102) |
| `BSAU_MODE_TEST_DFT_LOG` | off | on | off | Goertzel DFT — verify ADC rate with a sine (TB-103) |
| `BSAU_MODE_TEST_NRF_LOG` | TX | on | off | NRF self-test + stress loop (TB-105/106) |
| **`BSAU_MODE_DATASET`** | **TX** | **off** | **ASCII** | **v2.1**: Production TX **AND** channels-only UART CSV for data collection |

All test modes with the radio off say so explicitly — don't try to
verify link health in a radio-off mode.

---

## Quick start

### First-time build (STM32CubeIDE)

```
File → Import → Existing Projects into Workspace
  → browse to bsau_v2/
  → Finish

Ctrl+B          (Build All — expect zero errors)
Green play ▶    (Flash + run)
```

The on-board ST-LINK/V2-1 handles both flashing and the UART. Plug the
Micro-B cable into your laptop once and you get:

- `/dev/ttyACM0` — virtual COM port = USART1 @ 921600 8N1 (Linux)
- `NODE_L432KC` — mass-storage mount for drag-and-drop flashing (rarely used)
- SWD debug via the same cable

### Opening the serial console

```bash
# Linux:
picocom -b 921600 /dev/ttyACM0
# macOS:
screen /dev/tty.usbmodem<serial> 921600
# Windows: PuTTY, COM<n>, 921600 baud, 8N1, no flow control
```

### Dataset collection (v2.1)

Flash `BSAU_MODE_DATASET`, then on your laptop:

```bash
pip install pyserial
python3 ../cpcu_v2/scripts/bsau_dataset_collector.py \
    --port /dev/ttyACM0 \
    --label REST \
    --output ./datasets
# Subject holds the gesture. Ctrl+C when done.
# File appears at ./datasets/REST_0.csv (next free N auto-picked).
```

Simultaneously, on the Pi, open `cpcu_tui` → press `7` → use `←/→` to
pick the same label → SPACE to start/stop. You get matching CSV files
from both sides of the radio link.

Full walkthrough: [`../SYSTEM_GUIDE.md` §7](../SYSTEM_GUIDE.md) and
[`docs/BSAU_RUN_GUIDE.md` §6](docs/BSAU_RUN_GUIDE.md).

---

## Directory layout

```
bsau_v2/
├── InfiniTech_BSAU_Skeleton_v1.0.ioc     # CubeMX config (open in CubeIDE to edit clocks/pins)
├── STM32L432KCUX_FLASH.ld                # Linker script
│
├── Core/
│   ├── Inc/                              # Public headers
│   │   ├── bsau_config.h                 # ← Edit this to pick build mode
│   │   ├── bsau_app.h                    # BSAU_Init / BSAU_Run
│   │   ├── bsau_adc.h                    # ADC pipeline globals
│   │   ├── bsau_test.h                   # Test-mode entry points
│   │   ├── wireless_packet.h             # Packet layout + codec macros (shared with CPCU)
│   │   ├── nrf24l01.h                    # NRF driver API
│   │   ├── nrf24l01_test.h               # NRF self-test (TB-105/106)
│   │   ├── log.h                         # Mode-aware LOG / LOG_CSV macros
│   │   └── *.h                           # CubeMX-generated peripheral headers
│   │
│   ├── Src/                              # Implementations
│   │   ├── bsau_app.c                    # Init + main loop (packet fill + TX)
│   │   ├── bsau_adc.c                    # TIM6 → ADC → DMA → snapshot pipeline
│   │   ├── bsau_test.c                   # 5 test-mode entry points
│   │   ├── nrf24l01.c                    # NRF SPI driver
│   │   ├── nrf24l01_test.c               # NRF self-test implementation
│   │   ├── wireless_packet.c             # WL_Pack / WL_Unpack (12-bit codec)
│   │   ├── main.c                        # CubeMX main: calls BSAU_Init / BSAU_Run
│   │   └── *.c                           # CubeMX-generated peripheral init
│   │
│   └── Startup/
│       └── startup_stm32l432kcux.s       # Cortex-M4 vector table
│
├── Drivers/                              # ST-provided HAL + CMSIS (generated, don't edit)
│
├── Debug/                                # Build artefacts (gitignored)
│
└── docs/
    ├── BSAU_ARCHITECTURE.md              # Full design (ADC pipeline, clock tree, NRF, power)
    ├── BSAU_RUN_GUIDE.md                 # Build / flash / operate
    └── BSAU_TEST_GUIDE.md                # Test walkthrough + full TB-XXX reference
```

---

## Key files to know

| File | What it does | When you'd touch it |
|------|--------------|---------------------|
| `Core/Inc/bsau_config.h` | Selects build mode + DATASET decimation | Switching modes; tuning UART rate in DATASET |
| `Core/Inc/bsau_app.h` | `ADC_DMA_CHANNELS`, `BATT_LOW_THRESHOLD`, NRF address/channel | Radio channel change, battery threshold tuning |
| `Core/Src/bsau_app.c` | Main acquisition-and-transmit loop | Changing the packet-assembly sequence |
| `Core/Src/bsau_adc.c` | ADC DMA ISRs, sample snapshot | Changing sample rate or scan order |
| `Core/Src/nrf24l01.c` | NRF SPI driver | Radio debugging |
| `wireless_packet.h/c` | 32-byte codec | **Only touch if you also update CPCU's copy** |
| `Core/Inc/log.h` | LOG / LOG_CSV macros | Adding new log outputs (read the mode table at top) |

---

## Wireless packet format (v2.1, 32 bytes)

```
Byte    Field        Size    Description
[0]     seq          1 B     Sequence number (0-255, wraps)
[1]     flags        1 B     Status flags + 2-bit battery level
[2]     tx_retry     1 B     NRF ARC_CNT (from previous TX)
[3]     pkt_loss     1 B     NRF PLOS_CNT (cumulative, saturates at 15)
[4-5]   timestamp    2 B     TIM2 1 MHz counter, little-endian
[6-7]   vbat_raw     2 B     12-bit battery ADC (high-nibble aligned)
[8-19]  sample[0]    12 B    8 channels × 12-bit packed
[20-31] sample[1]    12 B    8 channels × 12-bit packed
```

Encoded by `WL_Pack` on this side, decoded by `WL_Unpack` on the CPCU.
Both sides link the same `wireless_packet.c` source file.

Flag bits (byte [1]):

```
7: FIRST_PACKET    (session-first packet after boot)
6: CLIPPING        (at least one channel hit an ADC rail)
5: ELEC_OFF        (electrode off, impedance spike detected)
4: ADC_OVRN        (DMA could not keep up — firmware bug)
3: TX_SAT          (prior packet did not leave the radio)
2: CAL             (calibration packet, not user data)
1-0: BATT_LVL      00=OK 01=LOW 10=CRIT 11=CHARG
```

---

## Testing

Run tests in this order — nothing downstream lies if you skip a layer:

| Phase | Mode | What it proves |
|-------|------|---------------|
| 1 | Any | Boot sanity: MCU runs, clock works, UART speaks |
| 2 | `TEST_PKT_LOG` | 12-bit codec is bit-exact for every value 0–4095 |
| 3 | `TEST_ADC_CSV` | All 8 EMG channels read independently at 2 kHz |
| 4 | `TEST_DFT_LOG` | ADC sample rate is actually 2 kHz (feed a 200 Hz sine) |
| 5 | `TEST_NRF_LOG` | NRF SPI, registers, power cycle, TX state machine all work |
| 6 | `RELEASE` + CPCU | End-to-end link establishes, seq counter monotonic |
| 7 | `DATASET` + CPCU + collector script | Dual-path capture works end-to-end |

Every test has a TB-XXX ID with exact pass criteria, expected UART
lines, and failure-analysis tables in
[`docs/BSAU_TEST_GUIDE.md`](docs/BSAU_TEST_GUIDE.md) **Part B**.

---

## Further reading

- **[`docs/BSAU_ARCHITECTURE.md`](docs/BSAU_ARCHITECTURE.md)** — every
  design decision with reasoning. ADC ranks, DMA choice, NRF SPI
  timing budget, clock tree, packet wire format, battery monitoring,
  power budget.
- **[`docs/BSAU_RUN_GUIDE.md`](docs/BSAU_RUN_GUIDE.md)** — build/flash
  operational workflow. Covers RELEASE, DEBUG, and the full v2.1
  DATASET workflow.
- **[`docs/BSAU_TEST_GUIDE.md`](docs/BSAU_TEST_GUIDE.md)** — phase-by-phase
  test walkthrough (Part A) + exhaustive TB-100 through TB-309
  reference (Part B).
- **[`../SYSTEM_GUIDE.md`](../SYSTEM_GUIDE.md)** — whole-system go-to
  guide covering both BSAU and CPCU.

---

## License

Part of the [InfiniTech Prosthetic Hand](../) project. MIT — see
[`../LICENSE`](../LICENSE).
