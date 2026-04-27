# CPCU — Central Processing & Control Unit

**The receiver half of the prosthetic hand system.** A Raspberry Pi 5
that receives wireless packets from the BSAU, runs a real-time DSP + ML
pipeline, and commands up to 6 servo motors via a PCA9685 PWM driver.

[![Platform: RPi5](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-c51a4a.svg)](#hardware)
[![Language: C11 / Python 3](https://img.shields.io/badge/Language-C11%20%7C%20Python%203-green.svg)](#software-architecture)
[![Version: v2.3.5](https://img.shields.io/badge/Version-v2.3.5-brightgreen.svg)](#)

> **First time with this repo?** Start from the root
> [`../SYSTEM_GUIDE.md`](../SYSTEM_GUIDE.md) — it covers the whole system
> end to end. This README is the CPCU-specific quick reference.

---

## What this side does

The CPCU receives 1000 packets per second from the BSAU over a 2.4 GHz
NRF24L01+ link, decodes them into 8-channel EMG samples, runs a 20–450 Hz
Butterworth band-pass + 50 Hz notch filter, extracts features on a 200 ms
sliding window, feeds them to a trained RandomForest classifier, maps the
gesture class to servo positions, and commands 6 servos through a PCA9685
at 50 Hz PWM — all under deterministic safety supervision.

```
NRF24L01+ → SPI → WL_Unpack → SPSC Ring → DSP + ML → PCA9685 → Servos
                                   ↑                    ↑
                            /dev/shm/cpcu_ipc     I²C @ 400 kHz
```

### Headline numbers

- **≤ 51 ms** worst-case ADC-to-servo latency (typical 26 ms)
- **1000 pkt/s** sustained wireless input
- **10 Hz** gesture inference rate (70 % headroom on cores 1–2)
- **7 fault sources** monitored by a deterministic safety FSM, **all individually recoverable** (radio, battery, DSP stall, I²C, thermal, ring-overflow, NRF hardware) — see Safety section
- **66 240 B** shared-memory region for all IPC

---

## Hardware

### Bill of materials

| Component | Part | Interface | Notes |
|-----------|------|-----------|-------|
| Processing unit | Raspberry Pi 5 (≥ 1 GB RAM) | — | **Active cooler mandatory** for 2.8 GHz OC |
| Radio module | NRF24L01+ | SPI0 (8 MHz) + GPIO 25 (CE) | PRX role, busy-poll receive |
| Servo driver | PCA9685 | I²C1 (400 kHz, addr `0x40`) | 50 Hz PWM, 16 channels |
| Servo motors | MG995 × 4 + SG90 × 2 | PCA9685 channels 0–5 | **Separate 6 V / 3 A PSU — common GND only** |
| PSU (Pi) | Official USB-C 27 W | — | Needed for sustained 2.8 GHz |
| PSU (servos) | 6 V / 3 A bench or buck | — | Never share the Pi's 5 V rail |

### GPIO pinout

```
NRF24L01+ (SPI0):                     PCA9685 (I²C1):
  GPIO 8   SPI0_CE0  → CSN              GPIO 2   SDA1 → SDA
  GPIO 9   SPI0_MISO → MISO              GPIO 3   SCL1 → SCL
  GPIO 10  SPI0_MOSI → MOSI
  GPIO 11  SPI0_SCLK → SCK (8 MHz)
  GPIO 25            → CE (held HIGH in RX)
  GPIO 24            → IRQ (unused — busy-poll)
```

Both the NRF and PCA9685 are **3.3 V logic only**. Full wiring tables
with pre-power multimeter checks live in
[`../SYSTEM_GUIDE.md` §3.3](../SYSTEM_GUIDE.md).

---

## Software architecture

### Core allocation (4× Cortex-A76)

| Core | Process | Scheduler | Role |
|------|---------|-----------|------|
| **Core 0** | `cpcu_kernel`, `cpcu_tui`, Linux | CFS (default) | Supervisor, watchdog, SSH, logging |
| **Cores 1–2** | `cpcu_dsp.py` | `SCHED_FIFO 80`, isolated + tickless | DSP filtering + ML inference (10 Hz) |
| **Core 3** | `cpcu_io` | `SCHED_FIFO 90`, isolated + tickless | NRF SPI busy-poll + PCA9685 + safety monitor |

Cores 1–3 are isolated from the Linux scheduler via
`isolcpus=1,2,3 nohz_full=1,2,3 rcu_nocbs=1,2,3` in the kernel command
line (applied automatically by `setup_pi.sh`).

### Inter-process communication

All IPC goes through one 66 240 B POSIX shared-memory region at
`/dev/shm/cpcu_ipc`:

| Section | Size | Mechanism | Direction |
|---------|------|-----------|-----------|
| Control block | 192 B | Atomic flags, heartbeats | Kernel ↔ All |
| Sensor ring buffer | 64 KB (1024 × 64 B) | Lock-free SPSC (C11 atomics) | IO → DSP + TUI |
| Motor command | 128 B | Seqlock | DSP → IO |
| Diagnostics | 128 B | Atomic counters | All → Kernel/TUI |
| DSP export | 256 B | Atomic telemetry | DSP → TUI |

### DSP / ML pipeline

```
Ring buffer consume → ADC-to-voltage → Bandpass (20–450 Hz, Butterworth)
  → 50 Hz notch (Q=30) → 200 ms sliding window (100 ms stride, 50% overlap)
  → Feature extraction (MAV, RMS, WL, ZC, SSC, mean freq, spectral bands)
  → StandardScaler → RandomForest (100 trees, 10 classes)
  → Gesture lookup → Servo target commands (seqlock-published)
```

**10 gesture classes:** `REST`, `HAND_SLOW`, `HAND_HARD`, `HAND_OPEN`,
`ARM_BEND_LESS`, `ARM_BEND_MIDDLE`, `ARM_BEND_MOST`, `ARM_SLOW`,
`ARM_FAST`, `BICEPS_ONLY`.

### Safety monitor (7 fault sources, all recoverable)

The safety subsystem runs on Core 3 and enforces a deterministic
fail-safe (servos to neutral, then limp) on any single fault. Once the
condition clears and stays clear for `SAFETY_SAFE_RECOVER_MS = 3000 ms`,
the FSM transitions back through `RECOVERING` to `RUNNING` on its own —
no manual reset required.

| Source | Trip | Action | Recovery (v2.3) |
|--------|------|--------|------------------|
| Radio silence | 750 ms → DEGRADED, 1500 ms → SAFE | Servos neutral → limp | 10 consecutive OK packets |
| Battery critical | V_batt ≤ 2.7 V | Immediate SAFE | V_batt > 3.0 V (hysteresis) |
| DSP stall | No motor command for 2000 ms | Servos neutral | First fresh motor cmd |
| I²C fault | 5 consecutive PCA9685 failures | Servos neutral | First successful write |
| Thermal | CPU > 82 °C | Immediate SAFE | T < 70 °C (hysteresis) |
| Ring overflow | > 100 overflows since baseline | Servos neutral | 5 s of no new overflows, baseline reset |
| NRF hardware | SPI readback mismatch at init | Tracked in diagnostics | cpcu_io re-init at 3 s interval |

**v2.3 ring-overflow fix.** Earlier versions tied the ring fault to the
all-time cumulative `io_ring_overflows` counter. Because that counter
is monotonic, once the threshold tripped the FSM latched in SAFE
forever even after the producer/consumer rebalanced. The new logic
applies the threshold to the *delta* since the last quiescent baseline,
and clears the fault after 5 s of no new growth (then re-baselines).
Public `SAFETY_FeedRingOverflow(ctx, count)` API is unchanged.

**v2.4 BSAU NRF init non-fatal.** On the BSAU side, a failed
`NRF_Init()` at boot used to call `Error_Handler()` (hard lock). It now
sets `g_nrf_alive = false` and lets the periodic health check inside
`BSAU_Run` retry every 500 packets. This means a BSAU board with a
sagging radio rail will boot, keep ADC + UART alive (and the DATASET
CSV stream flowing), and pick up the radio link as soon as it
becomes reachable.

---

## Directory layout

```
cpcu_v2/
├── CMakeLists.txt                 # Build system (cmake ≥ 3.16, C11)
├── setup_pi.sh                    # One-time Raspberry Pi setup
├── run_tests.sh                   # Master test runner
│
├── include/                       # C headers
│   ├── wireless_packet.h          # Layer 0: 32-byte packet codec (shared with BSAU)
│   ├── nrf24l01_linux.h           # Layer 1: NRF24L01+ SPI driver (spidev + gpiod)
│   ├── cpcu_pca9685.h             # Layer 1: PCA9685 I²C servo driver
│   ├── cpcu_ipc.h                 # Layer 2: POSIX shared-memory IPC (SPSC + seqlock)
│   ├── cpcu_smooth.h              # Layer 2: Servo slew-rate smoother
│   ├── cpcu_safety.h              # Layer 3: System-wide safety monitor (v2.3.1)
│   ├── cpcu_log.h                 # Layer 3: Structured logging + CSV sinks
│   ├── cpcu_tui.h                 # Layer 5: Shared TUI types + cross-file API (v3.4)
│   └── demo_signals.h             # Shared 8-waveform generator (TUI + testbench)
│
├── src/                           # C sources
│   ├── wireless_packet.c          # 12-bit packed codec, round-trip verified
│   ├── nrf24l01_linux.c           # NRF SPI register driver for Linux userspace
│   ├── cpcu_pca9685.c             # PCA9685 register driver, 50 Hz PWM output
│   ├── cpcu_ipc.c                 # mmap'd ring buffer + seqlock motor commands
│   ├── cpcu_smooth.c              # Slew-rate limiter (2000 µs/s default)
│   ├── cpcu_safety.c              # Radio FSM, battery, thermal, I²C, DSP, ring (v2.3.1)
│   ├── cpcu_log.c                 # Log formatting backend
│   ├── cpcu_io.c                  # Core 3: real-time I/O main loop (v2.3)
│   ├── cpcu_kernel.c              # Core 0: process supervisor + watchdog
│   ├── cpcu_tui.c                 # Core 0: TUI main + key dispatch + splash (v3.4)
│   ├── cpcu_tui_render.c          # Core 0: TUI drawing primitives + page draws
│   └── cpcu_tui_data.c            # Core 0: TUI demo + dataset capture + wave ring
│
├── scripts/
│   ├── cpcu_dsp.py                # Cores 1–2: Python DSP + ML inference pipeline
│   ├── cpcu_ipc_bridge.py         # Python shared-memory bridge (mmap offsets)
│   ├── bsau_dataset_collector.py  # v2.1: laptop-side BSAU UART capture
│   ├── launch.sh                  # Systemd boot script
│   └── cpcu.service               # Systemd unit file
│
├── test/
│   ├── test_codec.c               # C: codec round-trip, vbat, seq-gap tests
│   ├── test_ipc_bridge.py         # Python: IPC struct offset validation
│   ├── test_dsp_pipeline.py       # Python: DSP filter + feature + model tests
│   ├── pca_testbench.c            # Interactive PCA9685 servo calibration TUI
│   ├── signal_testbench.c         # End-to-end signal integrity TUI
│   └── safety_testbench.c         # Automated safety-FSM harness (38 checks, v2.3.1)
│   ├── smooth_testbench.c        # Smoother + deadband unit harness (28 checks, v2.3.2)
│
├── datasets/                      # v2.1: collected EMG recordings ({label}_{N}.csv)
├── models/                        # emg_rf_model.pkl lives here (not in repo)
├── config/                        # Reference /boot/firmware/*.txt snippets
│
└── docs/
    ├── CPCU_ARCHITECTURE.md       # Full CPCU system architecture (v3.4)
    ├── CPCU_CONFIGURATION.md      # Tunable thresholds + recipes for common tweaks
    ├── CPCU_RUN_GUIDE.md          # Step-by-step deployment guide
    ├── CPCU_TEST_GUIDE.md         # Test execution guide (5 phases)
    └── export/                    # Rendered design documents (PDF)
```

---

## Getting started

### Prerequisites

- Raspberry Pi 5 running Raspberry Pi OS (64-bit, Lite or Full)
- GCC, CMake ≥ 3.16, `libncurses-dev`
- Python 3 with `numpy`, `scipy`, `scikit-learn`, `joblib` (installed by `setup_pi.sh`)

### 1. Clone and set up the Pi

```bash
git clone <your-repo-url> ~/prosthetic_hand
cd ~/prosthetic_hand/cpcu_v2

./setup_pi.sh                         # self-elevates via sudo (one prompt)
sudo reboot                           # the only sudo you'll type today
```

`setup_pi.sh` self-elevates to root via `sudo` for the operations that
need it (apt, /boot/firmware, udev, group creation), then drops back
to your user. After reboot you'll be in the `spi`, `i2c`, and `gpio`
groups so `cpcu_io` can talk to the peripherals without root, and
`/opt/cpcu` plus `/var/log/cpcu` are owned by your user so you can
install + tail logs with no sudo.

### 2. Build and install

```bash
cmake -S . -B build                   # configure (out-of-tree)
cmake --build build -j4               # compile all 7 binaries
cmake --install build                 # writes to /opt/cpcu/{bin,scripts}
                                      # — no sudo because /opt/cpcu is yours
```

This produces seven binaries under `build/`:

- `cpcu_io` — Core 3 RT controller (NRF + PCA + safety)
- `cpcu_kernel` — Core 0 supervisor + watchdog
- `cpcu_tui` — ncurses dashboard (3-file v3.4)
- `test_codec` — codec round-trip tests (7 PASS)
- `safety_testbench` — safety-FSM tests (38 PASS, v2.3.1) + smoother tests (28 PASS, v2.3.2)
- `pca_testbench` — interactive servo calibration TUI
- `signal_testbench` — interactive signal integrity TUI

### 3. Grant the binaries RT capabilities (one-shot, after first install)

```bash
./scripts/launch.sh grant-caps        # self-elevates; runs setcap
```

This adds `CAP_SYS_NICE` (for `SCHED_FIFO`) and `CAP_IPC_LOCK` (for
`mlockall`) to the installed `cpcu_io` and `cpcu_kernel` binaries so
they can take real-time priority without running as root.

### 4. Deploy the ML model

```bash
cp /path/to/emg_rf_model.pkl /opt/cpcu/models/    # /opt/cpcu is owned by you
```

The `.pkl` is never committed — keep it outside the repo.

### 5. Run the test suite

```bash
chmod +x run_tests.sh

./run_tests.sh 1            # Phase 1: software-only (any machine)
                            #   → test_codec       (7 PASS)
                            #   → safety_testbench (38 PASS, v2.3.1)
                            #   → smooth_testbench   (28 PASS, v2.3.2)
                            #   → test_dsp_pipeline.py (65 PASS, v2.3)
./run_tests.sh 1 2          # Phase 1 + IPC validation (needs kernel running)
./run_tests.sh              # All phases (Pi with all peripherals wired)
./run_tests.sh pca          # Interactive PCA9685 servo calibration TUI
./run_tests.sh signal       # Live signal integrity TUI (needs BSAU)
./run_tests.sh signal-demo  # Synthetic signal integrity TUI (no hardware)
./run_tests.sh safety-demo  # cpcu_tui --demo with fault hotkeys
```

### 6. Start the system

Two ways — pick one:

**A — interactive (development):**

```bash
./scripts/launch.sh tui      # tmux: KERNEL window + TUI window
./scripts/launch.sh attach   # later, re-attach to that session
./scripts/launch.sh stop     # later, shut everything down
```

**B — systemd auto-start (production):**

```bash
./scripts/launch.sh install-service    # self-elevates: writes unit file + setcap
sudo systemctl start cpcu              # one-shot start
sudo systemctl status cpcu             # live status
journalctl -u cpcu -f                  # live logs (no sudo)
```

The systemd path needs `sudo` for `start`/`stop`/`status` because
that's `systemctl`'s contract — the `cpcu.service` itself runs as
your user, not root.

### 7. Open the dashboard standalone

If `cpcu_kernel` and `cpcu_io` are already running (via `launch.sh tui`
or systemd), you can open *just* the dashboard from any other terminal
or SSH session:

```bash
/opt/cpcu/bin/cpcu_tui
```

The TUI has **7 pages**, press number keys to switch:

| Key | Page | Purpose |
|-----|------|---------|
| `1` | Overview | Rolled-up HEALTH banner + EMG bars + servos + gesture |
| `2` | Radio / I/O | NRF internals, packet stats, last-packet hex dump, BSAU flags |
| `3` | DSP / AI | DSP rate, inference rate, per-class confidence, motor cmds |
| `4` | Waveforms | Live 8-channel rolling scope; `UP/DN` select, `TAB` zoom |
| `5` | Health | 10-row traffic-light dashboard |
| **`6`** | **Dataset** | **collect EMG recordings; `←/→` label, `s`/SPACE start/stop, `t` raw/filt, `r` cancel** |
| `7` | Config | Static spec-sheet reference *(moved to last tab in v3.4)* |

Universal keys: `q` quits. Demo-mode hotkeys: `w [ ]` cycle waveforms,
`F B G O I` inject faults, `R` resets everything.

### 8. No hardware? Try demo mode

```bash
/opt/cpcu/bin/cpcu_tui --demo
```

All 7 pages animate with synthetic packets. Use this for screenshots,
learning key bindings, or verifying a TUI change didn't break
rendering.

---

## Dataset collection (v2.1)

New in v2.1: the TUI Dataset page (page 6 since v3.4) and the
`bsau_dataset_collector.py` script together let you capture two
synchronised CSV files of each gesture — one from the BSAU UART
(raw) and one from the CPCU TUI (filtered):

```bash
# On the laptop (BSAU side, while BSAU is in BSAU_MODE_DATASET):
python3 scripts/bsau_dataset_collector.py \
    --port /dev/ttyACM0 \
    --label REST \
    --output ./datasets
# Ctrl+C to stop.

# On the Pi (CPCU side, cpcu_tui running):
# Press '6' → use '←/→' to pick REST → SPACE → hold gesture → SPACE.
# File appears in the directory cpcu_tui was launched from:
# ./datasets/REST_0.csv
```

Both files have 8 columns, no header, same label stem — so the DSP/AI
team's existing `predict.py` loader works on either without changes.
Full walkthrough: [`../SYSTEM_GUIDE.md` §7](../SYSTEM_GUIDE.md).

---

## Wireless packet format (v2.1, 32 bytes)

```
Byte    Field       Size    Description
[0]     seq         1 B     Sequence number (0–255, wraps)
[1]     flags       1 B     Status flags + 2-bit battery level
[2]     tx_retry    1 B     NRF ARC_CNT (from previous TX)
[3]     pkt_loss    1 B     NRF PLOS_CNT (cumulative, saturates at 15)
[4–5]   timestamp   2 B     TIM2 µs counter, little-endian
[6–7]   vbat_raw    2 B     12-bit battery ADC (high-nibble aligned)
[8–19]  sample[0]   12 B    8 channels × 12-bit packed
[20–31] sample[1]   12 B    8 channels × 12-bit packed
```

Flag bits: FIRST_PACKET, CLIPPING, ELEC_OFF, ADC_OVRN, TX_SAT, CAL +
2-bit battery level (OK / LOW / CRIT / CHARG). The codec
(`wireless_packet.h/c`) compiles to identical bytes on the
Cortex-M4F (BSAU) and Cortex-A76 (CPCU) — both link the same source
file.

---

## Testing

Five phases of increasing hardware dependency:

| Phase | Scope | Hardware | Key tests |
|-------|-------|----------|-----------|
| 1 | Pure software | None | Codec round-trip, DSP validation, safety FSM (33 checks, v2.3), TUI demo render |
| 2 | IPC validation | Shared memory (`cpcu_kernel` running) | Struct offset matching between C and Python |
| 3 | Pi hardware | RPi5 + SPI + I²C | Core isolation, PCA9685 detection, thermal |
| 4 | Integration | BSAU transmitting | End-to-end packet flow, latency measurement |
| 5 | Qualification | Full system, 1 hour | Endurance, thermal soak, process recovery |

Full procedures + pass criteria: [`docs/CPCU_TEST_GUIDE.md`](docs/CPCU_TEST_GUIDE.md).
For BSAU-side tests (TB-100 through TB-309): [`../bsau_v2/docs/BSAU_TEST_GUIDE.md`](../bsau_v2/docs/BSAU_TEST_GUIDE.md) Part B.

---

## Further reading

### Standing references
- **[`../SYSTEM_GUIDE.md`](../SYSTEM_GUIDE.md)** — whole-system go-to
  guide (both BSAU and CPCU). Start here if you don't know the project.
- **[`docs/CPCU_ARCHITECTURE.md`](docs/CPCU_ARCHITECTURE.md)** — every
  design decision with reasoning: IPC layout, core allocation, safety
  FSM, timing budgets, TUI page logic.
- **[`docs/CPCU_CONFIGURATION.md`](docs/CPCU_CONFIGURATION.md)** —
  every tunable constant in `cpcu_safety.h` etc., what it does, when
  to change it, and the consequences. Includes recipes for common
  tweaks ("relax the radio timeout for a 100 m bench test", etc.).
- **[`docs/CPCU_RUN_GUIDE.md`](docs/CPCU_RUN_GUIDE.md)** — step-by-step
  Pi deployment guide with verification at every step.
- **[`docs/CPCU_TEST_GUIDE.md`](docs/CPCU_TEST_GUIDE.md)** — 5-phase
  test execution guide.
- **[`docs/export/`](docs/export/)** — rendered block diagrams,
  flowcharts, subsystem sketches (PDF).

### Topic deep-dives (one feature per doc)
- **[`docs/GESTURE_MAPPING.md`](docs/GESTURE_MAPPING.md)** — first-
  principles guide to `GESTURE_SERVO_MAP`: what a hobby servo is,
  how pulse widths relate to angles, the physical-electrode contract,
  the discovery workflow with `pca_testbench`.
- **[`docs/BOOT_AND_SYNC.md`](docs/BOOT_AND_SYNC.md)** — *v2.3.1.*
  Why you no longer need to coordinate BSAU/CPCU power-on order.
  Cold-start grace period explained.
- **[`docs/JITTER_MITIGATION.md`](docs/JITTER_MITIGATION.md)** —
  *v2.3.2.* Why the static arm "buzzes", why hobby servos do that,
  and how the hold-pose deadband suppresses it. Per-servo deadband
  tuning, what it doesn't fix.
- **[`docs/RUNTIME_CONFIG.md`](docs/RUNTIME_CONFIG.md)** — *v2.3.3.*
  Runtime-tunable knobs (`runtime.json`) vs compile-time safety
  thresholds (`configure.sh`). The split, the loader, the seqlock
  pattern, the bias-then-clamp invariant.
- **[`docs/EDIT_MODE.md`](docs/EDIT_MODE.md)** — *v2.3.4.* Press `e`
  on the TUI's CONFIG page → arm parks at neutral → editor unlocks.
  Cross-process handshake protocol, safety-FSM-has-priority, the
  500 ms DSP UNRESPONSIVE timeout.
- **[`docs/VELOCITY_MODE.md`](docs/VELOCITY_MODE.md)** — *v2.3.5.*
  Hybrid velocity-mode gestures. Per-class per-servo rates in JSON
  drive a stateful target integrator in cpcu_dsp.py scaled by SVM
  confidence. Hold gesture longer = arm closes deeper. Backwards-
  compatible: classes without velocity rows stay fixed-pose.

### Cross-references
- **[`../bsau_v2/README.md`](../bsau_v2/README.md)** — transmitter
  quick-start.

---

## License

Part of the [InfiniTech Prosthetic Hand](../) project. MIT — see
[`../LICENSE`](../LICENSE).
