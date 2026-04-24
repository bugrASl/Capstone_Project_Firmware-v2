# InfiniTech Prosthetic Hand — BSAU + CPCU (v2.1)

**Real-time myoelectric prosthetic hand controller. 8-channel EMG acquisition on
an STM32L432KC, 2.4 GHz wireless link to a Raspberry Pi 5, DSP + ML gesture
classification, and servo-driven fingers — end to end.**

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform: RPi5](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-c51a4a.svg)](cpcu_v2/)
[![MCU: STM32L432KC](https://img.shields.io/badge/MCU-STM32L432KC-03234b.svg)](bsau_v2/)
[![Language: C11 / Python 3](https://img.shields.io/badge/Language-C11%20%7C%20Python%203-green.svg)](#)
[![Version: v2.1](https://img.shields.io/badge/Version-v2.1-brightgreen.svg)](SYSTEM_GUIDE.md)

---

## 👉 New here? Start with [`SYSTEM_GUIDE.md`](SYSTEM_GUIDE.md)

`SYSTEM_GUIDE.md` is the single go-to document. It assumes no prior
knowledge and walks you through:

1. What the system does and how the pieces fit together
2. What to buy (bill of materials, cables, tools)
3. How to wire everything up (pin-by-pin, both sides)
4. How to bring up the toolchains on your laptop and the Pi
5. How to test each layer in the correct order
6. How to run the system day-to-day
7. How to collect training datasets (v2.1 feature)
8. What to do when things go wrong

Everything else in this repo is either architectural detail, per-layer
reference, or source code.

---

## Overview

The system is split into two physical units, connected by a 2.4 GHz radio link:

| Unit | Role | Platform | Summary |
|------|------|----------|---------|
| **BSAU** | Bio-Signal Acquisition Unit | NUCLEO-L432KC (Cortex-M4F @ 80 MHz) | Acquires 8-channel EMG at 2 kHz with 32× hardware oversampling; streams 1000 packets/s over NRF24L01+ |
| **CPCU** | Central Processing & Control Unit | Raspberry Pi 5 (4× Cortex-A76 @ 2.8 GHz OC) | Receives wireless packets, runs DSP filtering and RandomForest ML inference, drives 6 servo motors via a PCA9685 |

### Signal chain

```
Electrode → InAmp → ADC (32× OS) → DMA → WL_Pack → SPI → NRF24L01+      [BSAU]
                                                           │
                         2.4 GHz Enhanced ShockBurst, 2 Mbps, ch 76
                                                           │
NRF24L01+ → SPI → WL_Unpack → SPSC Ring → DSP + ML → PCA9685 → Servos  [CPCU]
```

### Headline numbers

- **≤ 51 ms** worst-case ADC-to-servo latency (typical 26 ms; target was 300 ms)
- **1000 pkt/s** sustained wireless throughput at 2 Mbps
- **10 Hz** gesture inference rate with 70 % headroom on cores 1–2
- **~66 hours** battery life on the wearable unit (500 mAh LiPo)
- **7 fault sources** monitored by a deterministic safety FSM

---

## What's new in v2.1 — Dataset mode

v2.1 adds `BSAU_MODE_DATASET`: the BSAU runs the full production transmit
loop (CPCU sees a normal packet stream, inference still runs, servos still
move) AND *simultaneously* emits one ASCII CSV line per packet on its UART:
eight raw 12-bit ADC channels per line, at 921600 baud. A single
capture session now yields two complementary CSV files of the same
contraction:

- **`./datasets/REST_0.csv`** on the laptop — raw ADC, 1000 Hz, 8 int cols
- **`~/datasets/REST_0.csv`** on the Pi — filtered (20–450 Hz Butterworth + 50 Hz notch), 2000 Hz, 8 float cols

Same 8-column shape on both sides, no header, no metadata — so `predict.py`
and any pandas loader works on either file without code changes.

See [SYSTEM_GUIDE.md §7](SYSTEM_GUIDE.md) for the full workflow.

---

## Repository layout

```
prosthetic_hand/
├── SYSTEM_GUIDE.md           ← Start here. Complete end-to-end guide.
├── PROJECT_STRUCTURE.txt     ← Detailed file-by-file layout
├── README.md                 ← (this file)
├── LICENSE
│
├── bsau_v2/                  ← STM32 firmware (STM32CubeIDE project)
│   ├── Core/                 → Application + HAL glue
│   ├── Drivers/              → ST-provided HAL + CMSIS
│   ├── docs/                 → BSAU_ARCHITECTURE, BSAU_RUN_GUIDE, BSAU_TEST_GUIDE
│   └── README.md             → BSAU quick-start
│
└── cpcu_v2/                  ← Raspberry Pi application (CMake project)
    ├── src/ include/         → C sources for cpcu_kernel/io/tui + drivers
    ├── scripts/              → Python DSP, IPC bridge, dataset collector
    ├── test/                 → Unit + integration tests
    ├── datasets/             → (v2.1) Collected EMG recordings
    ├── docs/                 → CPCU_ARCHITECTURE, CPCU_RUN_GUIDE, CPCU_TEST_GUIDE + PDF diagrams
    └── README.md             → CPCU quick-start
```

A detailed breakdown with every file's purpose lives in
[`PROJECT_STRUCTURE.txt`](PROJECT_STRUCTURE.txt).

---

## Quick start

### BSAU side (your laptop, STM32CubeIDE)

```bash
# 1. Open the project:
#    File → Import → Existing Projects into Workspace → bsau_v2/

# 2. Pick a build mode in Core/Inc/bsau_config.h:
#    #define BSAU_MODE_RELEASE       (production)
#    #define BSAU_MODE_DEBUG         (dev: LOG + decimated CSV)
#    #define BSAU_MODE_DATASET       (v2.1: TX + UART CSV stream)

# 3. Build (Ctrl+B) and flash (green play button).
# 4. Plug a UART terminal at 921600 8N1 on /dev/ttyACM0.
```

See [`bsau_v2/README.md`](bsau_v2/README.md) for the full BSAU workflow and
[`bsau_v2/docs/BSAU_RUN_GUIDE.md`](bsau_v2/docs/BSAU_RUN_GUIDE.md) for
pin-by-pin detail.

### CPCU side (the Raspberry Pi 5)

```bash
git clone <your-repo-url> ~/prosthetic_hand
cd ~/prosthetic_hand/cpcu_v2
sudo bash setup_pi.sh
sudo reboot

# After reboot:
mkdir build && cd build
cmake .. && make -j4
sudo make install

# Deploy the ML model (not in the repo):
sudo cp /path/to/emg_rf_model.pkl /opt/cpcu/models/

# Start the system:
sudo systemctl enable cpcu
sudo systemctl start cpcu
/opt/cpcu/bin/cpcu_tui            # Live dashboard — 7 pages
```

See [`cpcu_v2/README.md`](cpcu_v2/README.md) for the full CPCU workflow and
[`cpcu_v2/docs/CPCU_RUN_GUIDE.md`](cpcu_v2/docs/CPCU_RUN_GUIDE.md) for
deployment detail.

### No hardware? Preview everything in demo mode

```bash
# On the Pi (or any Linux box after building):
/opt/cpcu/bin/cpcu_tui --demo
#   Full 7-page TUI fed with synthetic packets. Press 1-7 for pages,
#   F/B/G/O/I to inject faults, w/[/]/R to cycle demo waveforms.
```

---

## Documentation map

| Document | Audience | Purpose |
|----------|----------|---------|
| **[SYSTEM_GUIDE.md](SYSTEM_GUIDE.md)** | Everyone | **The go-to.** End-to-end guide: overview → hardware → software → testing → operation → troubleshooting |
| [PROJECT_STRUCTURE.txt](PROJECT_STRUCTURE.txt) | Contributors | File-by-file breakdown of the whole repo |
| [bsau_v2/README.md](bsau_v2/README.md) | BSAU developers | BSAU quick-start + build-mode picker |
| [bsau_v2/docs/BSAU_ARCHITECTURE.md](bsau_v2/docs/BSAU_ARCHITECTURE.md) | Firmware engineers | Full BSAU design: ADC pipeline, clock tree, NRF SPI, packet format, power budget |
| [bsau_v2/docs/BSAU_RUN_GUIDE.md](bsau_v2/docs/BSAU_RUN_GUIDE.md) | BSAU operators | Build / flash / operate the STM32 firmware |
| [bsau_v2/docs/BSAU_TEST_GUIDE.md](bsau_v2/docs/BSAU_TEST_GUIDE.md) | BSAU testers | Test walkthrough + full TB-XXX reference (merged from old testbench spec) |
| [cpcu_v2/README.md](cpcu_v2/README.md) | CPCU developers | CPCU quick-start + feature overview |
| [cpcu_v2/docs/CPCU_ARCHITECTURE.md](cpcu_v2/docs/CPCU_ARCHITECTURE.md) | Software engineers | Full CPCU design: IPC layout, core allocation, safety FSM, timing budgets |
| [cpcu_v2/docs/CPCU_RUN_GUIDE.md](cpcu_v2/docs/CPCU_RUN_GUIDE.md) | CPCU operators | Pi setup, build, deploy, run, monitor |
| [cpcu_v2/docs/CPCU_TEST_GUIDE.md](cpcu_v2/docs/CPCU_TEST_GUIDE.md) | CPCU testers | 5-phase test execution guide |
| [cpcu_v2/docs/export/*.pdf](cpcu_v2/docs/export/) | Reviewers | Rendered block diagrams, subsystem flowcharts |

---

## Hardware at a glance

```
         ┌─────────────────────────────────┐        ┌──────────────────────────────────┐
         │               BSAU              │        │               CPCU               │
         │  NUCLEO-L432KC + NRF24L01+      │        │  Raspberry Pi 5 + NRF + PCA9685  │
         │                                 │        │                                  │
         │   8× EMG electrodes             │        │   6× servo motors                │
         │   │                             │        │   ↑                              │
         │   ▼                             │        │   │ PWM (I²C / PCA9685)          │
         │   INA333 amps + RC filter       │        │   │                              │
         │   │                             │        │   Core 3: cpcu_io (SCHED_FIFO)   │
         │   ▼                             │        │   ↑                              │
         │   STM32 ADC (2 kHz × 8, 32× OS) │        │   │ SPSC ring (/dev/shm)         │
         │   │                             │        │   │                              │
         │   ▼                             │        │   Cores 1-2: cpcu_dsp.py         │
         │   TIM2 timestamp + WL_Pack      │   ≈5m  │   (Butterworth + notch + RF)     │
         │   │                             │ ◀───▶  │   ↑                              │
         │   ▼                             │  2.4G  │   │ Seqlock motor command        │
         │   NRF24L01+ (1000 pkt/s)        │        │   │                              │
         │                                 │        │   Core 0: cpcu_kernel + cpcu_tui │
         │   Status: 1 kHz LED blink       │        │                                  │
         └─────────────────────────────────┘        └──────────────────────────────────┘
             Worn on upper arm                          Inside the prosthetic forearm
             500 mAh LiPo · ~66 h life                  USB-C 5V + separate 6V servo PSU
```

---

## Authors

- **bugrASl** — System architecture, firmware, software, hardware integration

## Acknowledgments

Developed as part of the **InfiniTech** prosthetic hand capstone project at
**METU (Middle East Technical University)**, Electrical - Electronics Engineering, 2026.

## License

MIT — see [LICENSE](LICENSE).
