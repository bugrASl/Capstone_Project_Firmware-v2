# CPCU — Central Processing & Control Unit

**The receiver half of the InfiniTech prosthetic hand system.** A Raspberry Pi 5
that receives wireless EMG packets from the BSAU, runs a real-time DSP + ML
pipeline, and commands 6 servo motors via a PCA9685 PWM driver.

> **First time?** Read `docs/USER_GUIDE.md` — it covers setup through operation.
> This README is the quick reference.

---

## Signal Chain

```
  Forearm (EMG)
       │
       ▼
  ┌─────────────┐    2.4 GHz     ┌──────────────┐    I²C     ┌──────────┐
  │    BSAU     │───────────────▶│     CPCU     │──────────▶│  6 Servos │
  │ STM32L432KC │  1000 pkt/s    │  Raspberry   │  50 Hz    │  MG995 +  │
  │ 8-ch sEMG   │  32 B/pkt      │  Pi 5        │  PCA9685  │  SG90     │
  └─────────────┘                └──────────────┘           └──────────┘
```

**Pipeline:** NRF24L01+ → SPI → Unpack → SPSC Ring → DSP + ML → Smoother → PCA9685 → Servos

## Key Numbers

| Metric | Value |
|--------|-------|
| End-to-end latency | ≤ 51 ms worst, 26 ms typical |
| Wireless input rate | 1000 pkt/s sustained |
| Inference rate | 10 Hz (70% headroom) |
| Safety fault sources | 7, all individually recoverable |
| IPC shared memory | 66 KB at `/dev/shm/cpcu_ipc` |
| Servo update rate | 50 Hz with trapezoidal smoothing |

---

## Quick Start

```bash
# One-time Pi setup (installs deps, enables SPI/I2C, isolates cores)
./launch.sh setup

# Build everything
./launch.sh build

# Verify readiness
./launch.sh check

# Run with the TUI dashboard
./launch.sh tui

# Run with web dashboard (friends can watch at http://<pi-ip>:8765)
./launch.sh tui --with-ws

# Stop
./launch.sh stop
```

`./launch.sh help` lists all commands. `./launch.sh help <cmd>` gives detail.

---

## Core Allocation

| Core | Process | Scheduler | Role |
|------|---------|-----------|------|
| **0** | `cpcu_kernel` + `cpcu_tui` + Linux | CFS | Supervisor, watchdog, TUI, SSH |
| **1–2** | `cpcu_dsp.py` | SCHED_FIFO 80, isolated | DSP filtering + ML inference |
| **3** | `cpcu_io` | SCHED_FIFO 90, isolated | NRF SPI + safety + PCA9685 servo |

Cores 1–3 isolated via `isolcpus=1,2,3 nohz_full=1,2,3` (applied by `setup_pi.sh`).

---

## IPC Layout (`/dev/shm/cpcu_ipc`)

| Region | Size | Mechanism | Direction |
|--------|------|-----------|-----------|
| Control block | 192 B | Atomic flags | Kernel ↔ All |
| Sensor ring | 64 KB | Lock-free SPSC | IO → DSP + TUI |
| Motor command | 128 B | SeqLock | DSP → IO |
| Diagnostics | 128 B | Atomic counters | All → Kernel/TUI |
| DSP export | 256 B | Atomic snapshot | DSP → TUI |
| Runtime config | 512+ B | SeqLock | Kernel → IO/DSP |
| Tool presence | 512 B | Per-slot heartbeat | Tools → WS bridge |
| DSP filtered | 6.4 KB | SeqLock snapshot | DSP → WS bridge |

---

## Safety Monitor

Seven fault sources, all recoverable with hysteresis:

| Source | Threshold | Recovery |
|--------|-----------|----------|
| Radio timeout | 750 ms silence | 10 good packets |
| Battery | ≤ 2.7 V critical | Voltage ≥ 3.0 V |
| DSP stall | No motor cmd for 2 s | Next command received |
| I²C bus | 5 consecutive failures | Next success |
| Thermal | CPU > 82°C | CPU < 70°C |
| Ring overflow | 100 overflows (delta) | 5 s quiescence |
| NRF hardware | SPI register mismatch | Re-init success |

Cold-start grace: 5-second radio timeout suppression on boot (BSAU and CPCU can power on in any order).

---

## DSP / ML Pipeline

Runs on Cores 1–2 in Python:

1. Pop sensor batch from SPSC ring (up to 100 entries).
2. Convert 12-bit ADC → voltage, subtract 1.65 V DC bias.
3. Bandpass filter 20–450 Hz (4th-order Butterworth).
4. 50 Hz notch filter (Q = 30).
5. Sliding window: 200 ms window, 50 ms stride.
6. Extract 7 features × N channels (MAV, RMS, WL, ZC, SSC, VAR, LOG_DET).
7. StandardScaler + RandomForest inference (100 trees).
8. Confidence-scaled velocity integration (gesture → servo rate).
9. Publish motor command via SeqLock.

Velocity mode: holding a gesture integrates servo position over time (graded control). Releasing snaps to "rest" which holds position.

---

## Servo Control

- **Trapezoidal smoother:** acceleration-limited ramp to target, per-servo configurable velocity/accel.
- **Hold-pose deadband:** suppresses redundant PCA writes when settled (kills static jitter).
- **Gravity compensation:** asymmetric velocity bias for weight-bearing joints.
- **Gripper stall watchdog:** if gripper is pinned at mechanical floor for 2 s, retreats to safe position.
- **Runtime tuning:** edit `config/runtime.json` or use the TUI live editor (`./launch.sh tui` → page 7 → `e`).

---

## Web Dashboard

Browser-accessible at `http://<pi-ip>:8765`. Read-only, multi-viewer.

| Tab | Content |
|-----|---------|
| Overview | System state, gesture + confidence, per-channel RMS, diagnostics |
| Waves | 8-channel rolling raw + filtered envelopes |
| Spectrum | Per-channel 256-pt FFT + waterfall (browser-side) |
| Tools | Live tool presence (pca_testbench, signal_testbench) |

Requires Mongoose: `./launch.sh vendor` (one-time fetch + rebuild).

---

## File Map

```
cpcu_v2/
├── launch.sh              ← Unified entry point (all commands)
├── CMakeLists.txt         ← Build system
├── config/runtime.json    ← Runtime configuration
├── src/                   ← C production code (16 files)
├── include/               ← C headers (14 files)
├── test/                  ← Testbenches + Python tests (12 files)
├── python/                ← DSP pipeline + IPC bridge (4 files)
├── web/static/            ← Dashboard HTML/CSS/JS
├── web/vendor/            ← Mongoose (fetched on demand)
├── scripts/               ← Shell helpers (setup, configure, run_tests)
└── docs/                  ← Architecture, testing, user guide, web dashboard
    └── diagrams/          ← System block diagrams (SVG)
```

---

## Documentation

| Doc | Content |
|-----|---------|
| `docs/USER_GUIDE.md` | Setup → build → test → operate + TUI editor reference |
| `docs/ARCHITECTURE.md` | Core allocation, IPC, safety FSM, smoother, velocity mode, soft-grip, jitter |
| `docs/TESTING.md` | Phase-by-phase test guide (233 PASS expected) |
| `docs/WEB_DASHBOARD.md` | Web bridge architecture, JSON protocol, deployment |
| `docs/CONFIGURATION.md` | Runtime config schema (keep your existing copy) |

---

## Hardware

| Component | Part | Interface |
|-----------|------|-----------|
| Processing unit | Raspberry Pi 5 (≥ 1 GB) | — |
| Radio | NRF24L01+ | SPI0 @ 8 MHz + GPIO 25 (CE) |
| Servo driver | PCA9685 | I²C1 @ 400 kHz, addr 0x40 |
| Servos | 3× MG995 + 3× SG90 | PCA9685 channels 0–5 |
| PSU (Pi) | USB-C 27 W | — |
| PSU (servos) | 6 V / 3 A separate | **Common GND only** |
