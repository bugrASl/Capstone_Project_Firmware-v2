# Prosthetic Hand Capstone — InfiniTech (BSAU + CPCU)

[![Status: v2.3.1](https://img.shields.io/badge/Status-v2.3.1-brightgreen.svg)](#)
[![BSAU: v2.4](https://img.shields.io/badge/BSAU-v2.4-blue.svg)](bsau_v2/README.md)
[![CPCU: v2.3.1](https://img.shields.io/badge/CPCU-v2.3.1-blue.svg)](cpcu_v2/README.md)
[![Tests: 110 PASS](https://img.shields.io/badge/Tests-110%20PASS-brightgreen.svg)](cpcu_v2/docs/CPCU_TEST_GUIDE.md)

**EE493/494 Capstone Design Project · METU, Spring 2026.**

A real-time prosthetic hand controller built from two cooperating
subsystems: a **BSAU** (Bio-Signal Acquisition Unit) running on an
STM32L432KC that samples 8 EMG channels at 2 kHz and transmits 1000
packets per second over a 2.4 GHz NRF24L01+ link, and a **CPCU**
(Central Processing & Control Unit) running on a Raspberry Pi 5 that
receives, filters, classifies via a trained RandomForest model, and
drives 6 servo motors through a PCA9685 PWM driver — all under
deterministic safety supervision.

## Quick start

> **Want to read the project end-to-end before touching anything?**
> Open [`SYSTEM_GUIDE.md`](SYSTEM_GUIDE.md). It's the single, go-to
> walkthrough — assumes no prior knowledge, covers both halves.

```bash
# 1. Get the code
git clone <your-repo-url> ~/prosthetic_hand
cd ~/prosthetic_hand

# 2. Set up the Pi (one-time, ~3 minutes)
cd cpcu_v2
./setup_pi.sh                       # self-elevates via sudo (one prompt)
sudo reboot                         # the only manual sudo you'll type

# 3. Build, install, grant capabilities
cmake -S . -B build
cmake --build build -j4
cmake --install build               # No sudo: /opt/cpcu is owned by you
./scripts/launch.sh grant-caps      # self-elevates: setcap CAP_SYS_NICE + CAP_IPC_LOCK

# 4. Drop your trained model
cp /path/to/emg_rf_model.pkl /opt/cpcu/models/

# 5. Run the test suite (no hardware needed for Phase 1)
./run_tests.sh 1                    # → 7+38+65 = 110 PASS

# 6. Launch the system
./scripts/launch.sh tui             # tmux: KERNEL window + TUI window
```

The BSAU side is built in STM32CubeIDE — open
`bsau_v2/InfiniTech_BSAU_Skeleton_v1.0.ioc` and click Build / Run.

## What you should know

### Headline numbers

- **≤ 51 ms** worst-case ADC-to-servo latency (typical 26 ms)
- **1000 pkt/s** sustained wireless input
- **10 Hz** gesture inference rate (70% headroom on cores 1–2)
- **7 fault sources** monitored by a deterministic safety FSM, *all
  individually recoverable* (radio, battery, DSP stall, I²C, thermal,
  ring-overflow, NRF hardware) — see CPCU README §Safety
- **66 240 B** shared-memory region for all IPC
- **105 automated tests** PASS in Phase 1 (codec + safety FSM + DSP)

### Two codebases, one repo

```
prosthetic_hand/
├── README.md            ← this file
├── SYSTEM_GUIDE.md      ← single end-to-end walkthrough (start here)
├── PROJECT_STRUCTURE.txt ← exhaustive file inventory + layout notes
├── LICENSE
│
├── bsau_v2/             ← STM32L432KC firmware (built in STM32CubeIDE)
│   ├── Core/Inc/        ← bsau_app.h, bsau_config.h, drivers, …
│   ├── Core/Src/        ← bsau_app.c (v2.4 — non-fatal NRF init), …
│   ├── Drivers/         ← STM32 HAL + CMSIS (ST-provided)
│   ├── docs/            ← BSAU_ARCHITECTURE.md, BSAU_RUN_GUIDE.md, BSAU_TEST_GUIDE.md
│   ├── README.md        ← BSAU-specific quick reference
│   ├── InfiniTech_BSAU_Skeleton_v1.0.ioc
│   └── STM32L432KCUX_FLASH.ld
│
└── cpcu_v2/             ← Raspberry Pi 5 application (CMake + Python)
    ├── CMakeLists.txt   ← v2.3 build system
    ├── setup_pi.sh      ← v2.3 — self-elevates, idempotent
    ├── run_tests.sh     ← v2.3 — sudo-wrapped internally
    ├── README.md        ← CPCU-specific quick reference
    ├── include/         ← 10 C headers (cpcu_safety.h v2.3, cpcu_tui.h v3.4, …)
    ├── src/             ← 12 C sources (cpcu_io.c v2.3, cpcu_tui*.{c} v3.4, …)
    ├── test/            ← 4 testbenches + 2 Python tests (110 PASS total)
    ├── scripts/         ← Python DSP, IPC bridge, launch.sh v2.5
    ├── docs/            ← CPCU_ARCHITECTURE.md v3.4, CPCU_RUN_GUIDE.md, …
    │   └── export/      ← 9 PDF design diagrams
    ├── config/          ← /boot/firmware/{config,cmdline}.txt snippets
    ├── datasets/        ← collected EMG recordings (.csv, gitignored)
    └── models/          ← emg_rf_model.pkl lives here (not in repo)
```

For the deeper file-by-file rationale, see
[`PROJECT_STRUCTURE.txt`](PROJECT_STRUCTURE.txt).

## No-sudo policy

Every script in this project is invoked **as your regular user**. The
two scripts that need root for specific operations (`setup_pi.sh` for
apt + `/boot/firmware`, `launch.sh install-service` and
`launch.sh grant-caps` for `/etc/systemd` and `setcap`) self-elevate
via `sudo` and prompt you exactly once. After `setup_pi.sh`:

- You're in the `spi`, `i2c`, `gpio` groups so the binaries can talk to
  the peripherals without root.
- `/opt/cpcu/{bin,scripts,models}` and `/var/log/cpcu/` are owned by
  you so `cmake --install` and log-tailing don't need sudo.
- The systemd unit (generated by `launch.sh install-service`) runs as
  your user with `AmbientCapabilities=CAP_SYS_NICE CAP_IPC_LOCK` so
  it can take `SCHED_FIFO` and `mlockall` without being root.

The only sudo prompts you'll see at the prompt are `sudo systemctl
{start,stop,status,restart} cpcu`, `sudo reboot`, and (rarely)
`sudo raspi-config` — Linux requires sudo for those specific actions
and there's no way around it.

## Recent changes

| Version | Date | Where | What |
|---|---|---|---|
| **v2.3.1 (CPCU safety)** | Apr 2026 | `cpcu_safety.{h,c}`, `safety_testbench.c` | Cold-start radio grace period (`SAFETY_RADIO_BOOT_GRACE_MS = 5 s`). Eliminates BSAU/CPCU power-on coordination. New TB-SAF09 (5 sub-checks). 110/110 PASS. See [`cpcu_v2/docs/BOOT_AND_SYNC.md`](cpcu_v2/docs/BOOT_AND_SYNC.md). |
| **v2.4 (BSAU)** | Apr 2026 | `bsau_app.c` | NRF init no longer fatal. Bounded retry + periodic 500-packet health check. Profile-uniform across DATASET / RELEASE / DEBUG. |
| **v2.3 (CPCU safety)** | Apr 2026 | `cpcu_safety.{h,c}` | Ring-overflow recoverable (delta + 5 s quiescence); `SAFETY_VBAT_DIVIDER` restored to 2.0 (was wrongly 1.0); `SAFETY_UpdateState()` now called from `cpcu_io.c` step 5. |
| **v3.4 (CPCU TUI)** | Apr 2026 | `cpcu_tui*.{c,h}` | Split into 3 .c + 1 .h (was 2900-line monolith); CONFIG → page 7; live-data on keys 1-6; IO heartbeat thresholds expressed relative to 100 ms period. |
| **v2.3 (build/scripts)** | Apr 2026 | `setup_pi.sh`, `run_tests.sh`, `launch.sh` | All scripts self-elevate via sudo internally. New `launch.sh install-service` and `launch.sh grant-caps` modes. |
| **v2.3 (tests)** | Apr 2026 | `safety_testbench.c`, `test_dsp_pipeline.py` | TB-SAF02 extended to 7 sub-checks for SAFE-recovery path; DSP test rewritten against current `cpcu_dsp.py` API (was importing a long-removed `get_features` function). 105/105 PASS at the time; v2.3.1 adds 5 → **110/110 PASS** now. |

## Documentation map

The docs split into **standing references** (one per major area, kept up to
date with every change) and **topic deep-dives** (one per feature, written
once to explain that feature in full).

### Standing references

| Where | What |
|---|---|
| [`README.md`](README.md) | This file — top-level project landing |
| [`SYSTEM_GUIDE.md`](SYSTEM_GUIDE.md) | End-to-end walkthrough; start here |
| [`PROJECT_STRUCTURE.txt`](PROJECT_STRUCTURE.txt) | File inventory + deployed-vs-flat layout note |
| [`bsau_v2/README.md`](bsau_v2/README.md) | BSAU quick reference |
| [`bsau_v2/docs/BSAU_ARCHITECTURE.md`](bsau_v2/docs/BSAU_ARCHITECTURE.md) | BSAU design rationale |
| [`bsau_v2/docs/BSAU_RUN_GUIDE.md`](bsau_v2/docs/BSAU_RUN_GUIDE.md) | Bring-up + DATASET workflow + troubleshooting |
| [`bsau_v2/docs/BSAU_TEST_GUIDE.md`](bsau_v2/docs/BSAU_TEST_GUIDE.md) | TB-100..TB-309 test procedures |
| [`cpcu_v2/README.md`](cpcu_v2/README.md) | CPCU quick reference |
| [`cpcu_v2/docs/CPCU_ARCHITECTURE.md`](cpcu_v2/docs/CPCU_ARCHITECTURE.md) | CPCU design rationale |
| [`cpcu_v2/docs/CPCU_CONFIGURATION.md`](cpcu_v2/docs/CPCU_CONFIGURATION.md) | Tunable-constant reference |
| [`cpcu_v2/docs/CPCU_RUN_GUIDE.md`](cpcu_v2/docs/CPCU_RUN_GUIDE.md) | Pi deployment walkthrough |
| [`cpcu_v2/docs/CPCU_TEST_GUIDE.md`](cpcu_v2/docs/CPCU_TEST_GUIDE.md) | 5-phase test execution guide |

### Topic deep-dives (one feature per doc)

| Where | What | Introduced |
|---|---|---|
| [`cpcu_v2/docs/GESTURE_MAPPING.md`](cpcu_v2/docs/GESTURE_MAPPING.md) | First-principles guide to `GESTURE_SERVO_MAP`: hobby servos, pulse widths, the discovery workflow | v2.3 |
| [`cpcu_v2/docs/BOOT_AND_SYNC.md`](cpcu_v2/docs/BOOT_AND_SYNC.md) | Cold-start choreography — why you no longer need to coordinate BSAU/CPCU power-on order | v2.3.1 |

## Hardware

| Component | Part | Where it lives | Interface |
|---|---|---|---|
| Acquisition MCU | STM32L432KC (Nucleo-32) | BSAU | — |
| 8 EMG amps | (custom front-end) | BSAU | ADC PA0–PA7 |
| Battery sense | 2:1 divider on PB0 | BSAU | ADC IN15 |
| Radio | NRF24L01+ | both sides | SPI + 1 GPIO (CE) |
| Servo driver | PCA9685 | CPCU | I²C @ 400 kHz, addr 0x40 |
| Servos | MG995 × 4 + SG90 × 2 | CPCU side | PCA channels 0–5, separate 6 V / 3 A PSU |
| Compute | Raspberry Pi 5 (≥1 GB) | CPCU | active cooler mandatory |

Full BoM, wiring tables, pre-power multimeter checks:
[`SYSTEM_GUIDE.md` §3](SYSTEM_GUIDE.md).

## License

MIT — see [`LICENSE`](LICENSE).
