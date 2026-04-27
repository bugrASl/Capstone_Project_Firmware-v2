# SYSTEM GUIDE — Prosthetic Hand Capstone (BSAU v2.4 + CPCU v2.3.6)

**Author:** bugrASl
**Date:** April 2026
**Status:** v2.3.6 (pca_testbench round-trip + live smoother tuning — bench-discovered calibration persists to runtime.json)

---

## How to read this document

This is the single, go-to document for the whole system. It assumes you
know nothing about the project specifically, nothing about the hardware
choices specifically, and possibly nothing about embedded systems in
general. It tries to explain both what to do *and* why you're doing it
— so when something goes wrong (and it will), you have a model in your
head instead of a recipe you can't modify.

> **What changed since v2.1.** This document was extended several times
> as the project matured. The biggest deltas:
>
> - **v2.3.6 (pca_testbench round-trip + live smoother tuning)** — Adds
>   `CFG_PatchFile()`, a surgical JSON edit that preserves fields
>   it doesn't know about. `pca_testbench` loads `runtime.json` on
>   startup, gains nine new keys (`[`/`]`/`b`/`B`/`v`/`a`/`d`/`S`/`L`)
>   to tune servo limits, gravity-sag bias, and the smoother
>   (velocity/accel/deadband) at the bench and save back. `cpcu_io`
>   re-applies smoother values on `config_seq` change so a `kill -HUP`
>   takes effect within ~20 ms. The workflow: stop the live system,
>   tune at the bench, save, restart — and the values cpcu_io applies
>   are exactly what you found by physically watching the arm. 13 new
>   unit tests for the patcher. See
>   [`cpcu_v2/docs/RUNTIME_CONFIG.md`](cpcu_v2/docs/RUNTIME_CONFIG.md) §10.
> - **v2.3.5 (CPCU DSP, velocity-mode gestures)** — Per-class per-servo
>   velocity rates in `runtime.json` drive a stateful target integrator
>   in `cpcu_dsp.py`, scaled by SVM confidence. Holding a gesture
>   longer makes the arm move further (instead of snapping to a fixed
>   pose); brief detection dropouts hold position rather than
>   resetting. `rest` stays freeze-mode and drains targets to neutral.
>   Hybrid: classes without velocity rows preserve v2.3.4 fixed-pose
>   behaviour. 18 new unit tests for the loader. See
>   [`cpcu_v2/docs/VELOCITY_MODE.md`](cpcu_v2/docs/VELOCITY_MODE.md).
> - **v2.3.4 (CPCU TUI, edit-mode handshake)** — Pressing `e` on the
>   CONFIG page (page 7) raises an IPC flag; cpcu_io parks the
>   smoother at neutral and acks once `SMOOTH_AllSettled()`; cpcu_dsp.py
>   commits to "rest" and stops publishing motor commands. The TUI
>   banner walks LOCKED → PARKING → EDITING (or → DSP UNRESPONSIVE
>   on a 500 ms ack timeout). Safety FSM has priority — any fault
>   forces edit mode off. Three new atomic bytes + timestamp added
>   to `IPC_ControlBlock`'s reserve region (no layout change;
>   `IPC_VERSION` 0x0203 → 0x0204). See
>   [`cpcu_v2/docs/EDIT_MODE.md`](cpcu_v2/docs/EDIT_MODE.md).
> - **v2.3.3 (CPCU runtime config)** — JSON-backed runtime tunables
>   (`cpcu_v2/config/runtime.json`) mirrored to a shared-memory IPC
>   region, parsed once on startup and on `SIGHUP`. Per-servo bias
>   offsets are the first runtime consumer (cpcu_io applies before
>   clamping to compile-time hardware limits). Compile-time safety
>   thresholds get a dedicated editor: `./configure.sh` interactive
>   or flag-driven, with rebuild reminders. New `config_testbench`
>   adds 30 unit tests for the JSON loader. See
>   [`cpcu_v2/docs/RUNTIME_CONFIG.md`](cpcu_v2/docs/RUNTIME_CONFIG.md).
> - **v2.3.2 (CPCU smoother, hold-pose deadband)** — Settled servos no
>   longer get redundant PWM refreshes, killing the host-induced
>   static jitter that made the arm "buzz". Per-servo deadband
>   (default 10 µs ≈ 0.9°). New `smooth_testbench` adds 28 unit tests
>   for the deadband + trapezoidal-motion behaviour. See
>   [`cpcu_v2/docs/JITTER_MITIGATION.md`](cpcu_v2/docs/JITTER_MITIGATION.md).
> - **v2.3.1 (CPCU safety, boot grace)** — `SAFETY_RADIO_BOOT_GRACE_MS`
>   (5 s) suppresses the radio fault on cold boot until either the
>   first packet arrives or the grace expires. Eliminates the
>   "hold BSAU reset while CPCU boots" dance — power on either side
>   in any order with seconds of slack. Genuinely-dead BSAU still
>   flagged within 6 seconds. See
>   [`cpcu_v2/docs/BOOT_AND_SYNC.md`](cpcu_v2/docs/BOOT_AND_SYNC.md).
> - **v2.4 (BSAU)** — `NRF_Init` failure at boot is no longer fatal.
>   The board boots with `g_nrf_alive = false`, ADC + UART + DATASET
>   CSV stream stay alive, and a periodic health check inside
>   `BSAU_Run` retries the radio every 500 packets. A bad-rail / dead-
>   chip prosthesis now does *something* useful instead of locking up
>   in `Error_Handler()`.
> - **v2.3 (CPCU safety)** — Ring-overflow fault is recoverable
>   (delta-since-baseline + 5 s quiescence timer); `SAFETY_VBAT_DIVIDER`
>   restored to 2.0 (was wrongly 1.0 in v2.2 — every healthy 4 V battery
>   was reading as 2.00 V, latching `battery.critical`); `cpcu_io.c`
>   now actually calls `SAFETY_UpdateState()` per loop, so the FSM
>   `state` shown in the TUI finally tracks non-radio fault transitions.
> - **v3.4 (CPCU TUI)** — Split into three .c files plus `cpcu_tui.h`
>   (was a 2900-line monolith); CONFIG moved from page 5 to page 7;
>   live-data pages now occupy keys 1-6; IO heartbeat thresholds
>   (`IO_HB_WARN_MS = 200`, `IO_HB_BAD_MS = 500`) expressed relative
>   to the 100 ms heartbeat period, fixing a frame where >20 ms always
>   fired WARN.
> - **v2.3 (build / scripts)** — All `setup_pi.sh` / `run_tests.sh` /
>   `launch.sh` now self-elevate via `sudo` internally. **Users never
>   have to type `sudo` manually** for any project script. The only
>   exceptions are `sudo systemctl <action> cpcu`, `sudo reboot`, and
>   `sudo apt` — which Linux requires for those specific operations.
> - **safety_testbench** now exercises the full RUNNING → SAFE →
>   RUNNING recovery path (extended from 5 to 7 sub-checks in TB-SAF02);
>   total automated tests: **186 PASS** across `test_codec`,
>   `safety_testbench`, and `test_dsp_pipeline.py`.

If you are ever stuck on a step, check the corresponding **"What can go
wrong"** box directly below that step before trying something random.
The error you are seeing is almost always one that has been seen
before.

The document is organised as a journey, not a reference:

```
  Part 1: What is this system?             (read before touching anything)
  Part 2: What you need                    (the shopping list)
  Part 3: Hardware setup                   (wire everything together)
  Part 4: Software setup                   (install toolchains)
  Part 5: Testing — in the correct order   (prove each layer works)
  Part 6: Running the system               (daily operation)
  Part 7: Dataset collection               (new in v2.1)
  Part 8: Troubleshooting                  (when reality doesn't match)
  Part 9: Reference                        (cheatsheet + glossary pointers)
```

For a deeper architectural "why," see `BSAU_ARCHITECTURE.md` and
`CPCU_ARCHITECTURE.md`. For exhaustive per-testbench pass/fail
numbers (exact wire bytes, register audit tables, ADC calibration
specs), see **Part B of `BSAU_TEST_GUIDE.md`** — the full TB-XXX
reference. Everything you actually need to *operate* the system is
in this document, though.

---

## Table of Contents

- **Part 1 — System Overview**
  - 1.1 [What this system does](#11-what-this-system-does)
  - 1.2 [The two boxes](#12-the-two-boxes)
  - 1.3 [How they talk to each other](#13-how-they-talk-to-each-other)
  - 1.4 [Build modes — a preview](#14-build-modes--a-preview)
  - 1.5 [Dataset mode — what's new in v2.1](#15-dataset-mode--whats-new-in-v21)
  - 1.6 [Project vocabulary (must-knows)](#16-project-vocabulary-must-knows)

- **Part 2 — What You Need**
  - 2.1 [Bill of materials — BSAU side](#21-bill-of-materials--bsau-side)
  - 2.2 [Bill of materials — CPCU side](#22-bill-of-materials--cpcu-side)
  - 2.3 [Cables, connectors, tools](#23-cables-connectors-tools)
  - 2.4 [Software you'll install](#24-software-youll-install)

- **Part 3 — Hardware Setup**
  - 3.1 [Safety first](#31-safety-first)
  - 3.2 [BSAU — wiring the STM32 Nucleo](#32-bsau--wiring-the-stm32-nucleo)
  - 3.3 [CPCU — wiring the Raspberry Pi 5](#33-cpcu--wiring-the-raspberry-pi-5)
  - 3.4 [Pre-power checks (before you plug anything in)](#34-pre-power-checks-before-you-plug-anything-in)
  - 3.5 [First power-on](#35-first-power-on)

- **Part 4 — Software Setup**
  - 4.1 [Your laptop (dev workstation)](#41-your-laptop-dev-workstation)
  - 4.2 [BSAU toolchain (STM32CubeIDE)](#42-bsau-toolchain-stm32cubeide)
  - 4.3 [Raspberry Pi OS bring-up](#43-raspberry-pi-os-bring-up)
  - 4.4 [Cloning the project](#44-cloning-the-project)

- **Part 5 — Testing (in the correct order)**
  - 5.1 [Why bottom-up matters](#51-why-bottom-up-matters)
  - 5.2 [The full test sequence at a glance](#52-the-full-test-sequence-at-a-glance)
  - 5.3 [Software-only tests (no hardware)](#53-software-only-tests-no-hardware)
  - 5.4 [BSAU standalone tests (just the STM32, no Pi)](#54-bsau-standalone-tests-just-the-stm32-no-pi)
  - 5.5 [CPCU standalone tests (just the Pi, no STM32)](#55-cpcu-standalone-tests-just-the-pi-no-stm32)
  - 5.6 [Integration tests (both boards)](#56-integration-tests-both-boards)
  - 5.7 [Qualification — endurance and recovery](#57-qualification--endurance-and-recovery)

- **Part 6 — Running the System (daily operation)**
  - 6.1 [Pre-flight checklist](#61-pre-flight-checklist)
  - 6.2 [Typical startup sequence](#62-typical-startup-sequence)
  - 6.3 [Production run (RELEASE mode, systemd)](#63-production-run-release-mode-systemd)
  - 6.4 [Development run (DEBUG mode, manual)](#64-development-run-debug-mode-manual)
  - 6.5 [Monitoring while it's running](#65-monitoring-while-its-running)
  - 6.6 [Clean shutdown](#66-clean-shutdown)

- **Part 7 — Dataset Collection (v2.1)**
  - 7.1 [Why two captures](#71-why-two-captures)
  - 7.2 [Capturing BSAU-side (UART)](#72-capturing-bsau-side-uart)
  - 7.3 [Capturing CPCU-side (TUI Page 6)](#73-capturing-cpcu-side-tui-page-6)
  - 7.4 [Verifying a capture pair](#74-verifying-a-capture-pair)

- **Part 8 — Troubleshooting**
  - 8.1 [Symptom to cause, BSAU side](#81-symptom-to-cause-bsau-side)
  - 8.2 [Symptom to cause, CPCU side](#82-symptom-to-cause-cpcu-side)
  - 8.3 [Symptom to cause, integration](#83-symptom-to-cause-integration)

- **Part 9 — Reference**
  - 9.1 [Command cheatsheet](#91-command-cheatsheet)
  - 9.2 [Where to go next](#92-where-to-go-next)

---

# Part 1 — System Overview

## 1.1 What this system does

This is a **myoelectric prosthetic hand controller**. It reads the
electrical signals ("EMG" — electromyography) that muscles produce
when they contract, figures out what gesture the user is intending,
and drives finger and wrist servos to execute that gesture.

There are two physical units:

- **BSAU** (Biosignal Sampling & Acquisition Unit) — a small
  STM32-based board worn on the upper arm near the electrodes. It
  reads the 8 electrodes many times a second and transmits the
  measurements wirelessly.

- **CPCU** (Central Processing & Control Unit) — a Raspberry Pi 5
  inside the prosthetic forearm. It receives the wireless
  measurements, runs a digital-signal-processing and machine-learning
  pipeline, and drives the servos that move the fingers.

```
   User's muscles                        Prosthesis
   ──────────────                        ──────────
                                          ┌─────────────────────────────────┐
   ┌──────┐  electrodes  ┌────┐           │   ┌────────┐   ┌─────────┐      │
   │ EMG  │─────────────▶│BSAU│ ─ 2.4GHz ─┼──▶│  CPCU  │──▶│ servos  │      │
   └──────┘              └────┘   radio   │   │ (Pi 5) │   │ (PCA)   │      │
                                          │   └────────┘   └─────────┘      │
                                          └─────────────────────────────────┘
```

The wireless link is what lets the forearm move freely without cables
trailing to the upper arm.

## 1.2 The two boxes

### BSAU in one paragraph

A NUCLEO-L432KC development board (STM32L432KC MCU: ARM Cortex-M4F
running at 80 MHz) with an NRF24L01+ radio module wired to SPI1. It
reads 8 analog inputs (PA0–PA7) with the on-chip 12-bit ADC at
**2 kHz per channel**, packs 2 samples per channel into a 32-byte
wireless packet, and transmits **1000 packets per second**. The
whole acquisition-and-transmit loop is written in C, HAL-based,
compiled in STM32CubeIDE.

### CPCU in one paragraph

A Raspberry Pi 5 (BCM2712, quad-core Cortex-A76 at 2.4 GHz,
overclocked to 2.8 GHz). Linux cores 1, 2, 3 are isolated from the
scheduler (`isolcpus=1,2,3`). **Core 3** runs the C program
`cpcu_io` which polls the NRF radio via SPI, decodes each 32-byte
payload, and pushes the samples into a lock-free ring buffer in
`/dev/shm/cpcu_ipc`. **Cores 1 and 2** run a Python program
`cpcu_dsp.py` which does a 20–450 Hz bandpass filter, feature
extraction, and a scikit-learn RandomForest classifier over a 200-ms
sliding window. Gestures are written to a seqlock-protected motor-
command block that `cpcu_io` reads and forwards to a PCA9685
16-channel PWM driver (on I²C) which commands up to 6 servo motors.
**Core 0** runs `cpcu_kernel`, the supervisor that spawns, watchdogs
and restarts the other processes; it also launches the user-facing
`cpcu_tui` dashboard on demand.

## 1.3 How they talk to each other

```
   [BSAU]                          2.4 GHz                         [CPCU]
    STM32           ┌─────────┐   NRF24L01+     ┌─────────┐           Pi 5
    ADC ─► WL_Pack ►│ NRF SPI │═══ airlink ═══▶ │ NRF SPI │─► WL_Unpack
                    └─────────┘   (2 Mbps)      └─────────┘
    Packet: 32 bytes, 1000 pkt/s, channel 76 (2.476 GHz), address 0xE7E7E7E7E7
```

Inside the Pi:

```
    cpcu_io  ─► 1024-entry SPSC ring ─►  cpcu_dsp.py
   (Core 3)                              (Cores 1-2)
                                              │ ML result
                                              ▼
    cpcu_io  ◄─ motor seqlock ◄── cpcu_dsp.py
      │
      ▼
    PCA9685 ─► 6 servos
    (I²C, 400 kHz, addr 0x40)
```

Everything a user sees happens through **`cpcu_tui`**, an ncurses
dashboard with 7 pages (overview / radio / DSP / waveforms / config /
health / **dataset**, new in v2.1). The TUI is read-only and
attaches/detaches at will; leave one running on a second monitor
during development.

## 1.4 Build modes — a preview

The BSAU firmware is the same codebase for every use case; which
build mode you pick in `bsau_config.h` decides what the main loop
does. At any time, **exactly one** of these `#define` macros must be
active:

| Mode | Radio | LOG | CSV | Purpose |
|------|-------|-----|-----|---------|
| `BSAU_MODE_RELEASE` | TX | off | off | **Production.** Zero debug overhead. |
| `BSAU_MODE_DEBUG` | TX | on | on | Development. Periodic status lines + decimated CSV. |
| `BSAU_MODE_TEST_ADC_CSV` | off | off | binary | ADC binary stream for SerialPlot. |
| `BSAU_MODE_TEST_PKT_LOG` | off | on | off | Codec round-trip verification. |
| `BSAU_MODE_TEST_CSV` | off | off | ASCII | ADC CSV with drop counter. |
| `BSAU_MODE_TEST_DFT_LOG` | off | on | off | Goertzel DFT — check ADC rate by feeding a sine. |
| `BSAU_MODE_TEST_NRF_LOG` | TX | on | off | NRF self-test + stress loop. |
| `BSAU_MODE_DATASET` | TX | off | ASCII | **v2.1**: Production TX + channels-only UART CSV for collecting. |

You edit one `#define` in `bsau_config.h`, rebuild, flash, and reset.
`log.h` enforces mutual exclusion at compile time — if you
accidentally leave two modes active you get `#error "Define exactly
one BSAU_MODE_*."`.

## 1.5 Dataset mode — what's new in v2.1

The machine-learning side of the project needs **training data** — a
lot of CSV files labelled with what gesture the user was making
while that capture was recorded. v2.1 adds a new build mode,
`BSAU_MODE_DATASET`, that does two things simultaneously:

1. Runs the full production transmit loop, exactly as RELEASE
   mode does — so the CPCU sees a normal packet stream, the DSP
   inference runs, servos still move.
2. *Additionally* emits one ASCII CSV line per packet on the BSAU's
   USART1 (that's the same virtual COM port you already have over
   the ST-LINK debug cable), at 921600 baud. Each line is 8 integers,
   comma-separated, one per channel: `c0,c1,c2,c3,c4,c5,c6,c7`.

That means a single capture session produces two CSV files of the
same muscle contraction:

- the **raw** stream captured directly at the BSAU UART (`REST_0.csv`
  on the laptop), and
- the **filtered** stream captured at the CPCU (`REST_0.csv` on the
  Pi, Butterworth 20-450 Hz + 50 Hz notch already applied).

The DSP team uses the raw one as a "golden reference" (no wireless
in the loop) and the filtered one as the ready-to-train dataset.
`predict.py` — the script they already use — now just works on the
same file format.

Part 7 of this guide walks through a capture session step by step.

## 1.6 Project vocabulary (must-knows)

-   **MCU** — Microcontroller. The STM32 chip on the BSAU.
-   **SPI** — 4-wire serial bus (MISO, MOSI, SCK, CS). Used to talk
    to the NRF radio on both boards.
-   **I²C** — 2-wire serial bus (SDA, SCL). Used on the Pi to talk
    to the PCA9685.
-   **UART** — Asynchronous serial link (TX, RX). Used for
    debugging and, in DATASET mode, for streaming samples to the
    laptop.
-   **ADC** — Analog-to-digital converter. The STM32's on-chip ADC
    is what reads the electrode voltages.
-   **DMA** — Direct Memory Access. The STM32 moves ADC readings
    into memory without the CPU's help.
-   **NRF / NRF24L01+** — 2.4 GHz radio module, one on each side.
-   **PCA9685** — 16-channel 12-bit PWM driver IC. Turns commands
    from the Pi into the pulses that servos understand.
-   **Servo** — A motor with closed-loop position control that
    takes 1.0-2.0 ms PWM pulses every 20 ms (50 Hz).
-   **EMG** — Electromyography. The tiny voltages (10 µV – 2 mV)
    that muscles produce when they contract.
-   **isolcpus** — A Linux kernel option that excludes certain CPU
    cores from the general scheduler. We reserve cores 1, 2, 3 so
    our real-time code has them to itself.
-   **SCHED_FIFO** — Linux real-time scheduling policy. Keeps our
    I/O process from being preempted by anything non-critical.
-   **Packet / sequence number / seq gap** — The 32-byte frame the
    BSAU sends over the air. Each one has a counter that increments
    by 1; a "seq gap" means the counter jumped, which means a
    packet was lost or corrupted.
-   **IPC** — Inter-Process Communication. On the Pi this is a
    shared-memory region at `/dev/shm/cpcu_ipc` that all CPCU
    processes read and write.
-   **Ring buffer (SPSC)** — A circular buffer with one producer
    and one consumer. Ours holds 1024 sensor samples.
-   **Seqlock** — A lock-free synchronisation primitive. The
    writer bumps a counter before and after writing; the reader
    retries if the counters don't match.

A longer glossary lives at the end of `CPCU_RUN_GUIDE.md`.

---

# Part 2 — What You Need

## 2.1 Bill of materials — BSAU side

| Item | Part number / note | Qty |
|------|--------------------|-----|
| STM32 Nucleo-32 development board | **NUCLEO-L432KC** (ST) | 1 |
| 2.4 GHz radio module | **NRF24L01+** (SOIC or PA+LNA variant) | 1 |
| Bulk capacitor on NRF VCC | 10 µF electrolytic or tantalum | 1 |
| Decoupling capacitor on NRF VCC | 100 nF ceramic | 1 |
| EMG front-end (amplifier + filter) | INA333 or AD8221 based, Vref = 1.65 V | 1 per 8-channel system |
| EMG electrodes | Ag/AgCl gel electrodes or dry EMG electrodes | 8 (plus 1 reference) |
| USB A-to-Micro-B cable (data, not charge-only) | Any quality brand | 1 |
| Female-to-female jumper wires | 20 cm, Dupont-terminated | ~10 |
| 3.7 V Li-ion battery (optional, for untethered) | 2S pack if you want ~7.4 V headroom for the front-end | 1 |
| 100 kΩ resistors (for battery divider if using a battery) | 1/4 W, 1 % tolerance | 2 |

**Note:** a Nucleo-32 board powers from USB by default. If you use an
external battery, move the jumper JP3 to **E5V** and feed 5 V in on
pin E5V (see UM1956 in the Nucleo docs).

## 2.2 Bill of materials — CPCU side

| Item | Part number / note | Qty |
|------|--------------------|-----|
| Raspberry Pi 5 (any RAM) | 1 GB minimum; 4 GB recommended | 1 |
| **Active cooler** (heatsink + fan) | Official or equivalent — **mandatory** for 2.8 GHz | 1 |
| USB-C power supply | 5 V / 5 A, PD 27 W (official Pi 5 PSU is fine) | 1 |
| microSD card | 16 GB minimum, Class 10 or better | 1 |
| 2.4 GHz radio module | **NRF24L01+**, same as BSAU side | 1 |
| 16-channel PWM driver | **PCA9685** breakout board | 1 |
| I²C pull-up resistors on PCA9685 | 4.7 kΩ — **verify the breakout already has them**, otherwise add externally | 2 |
| Servo motors (up to 6) | SG90 for joints, MG995 for base/palm if more torque needed | up to 6 |
| **Separate servo PSU** | 6 V / 3 A minimum benchtop PSU or buck module | 1 |
| Ethernet or Wi-Fi | Pi 5 has both; Wi-Fi works but Ethernet is lower-latency for `ssh` | 1 |
| Female-to-female jumper wires | 20 cm, Dupont | ~15 |

**Why a separate servo PSU?** Servos draw 1–2 A in transient bursts
when they stall or reverse direction. That spike would brown out the
Pi. The two PSUs share **ground only**; the 5 V / 6 V rails must
stay separate. This is non-negotiable.

## 2.3 Cables, connectors, tools

-   Multimeter (any auto-ranging one works).
-   Breadboard (half-size is enough for BSAU, full-size for CPCU).
-   Wire strippers.
-   Soldering iron (optional but helpful for the EMG front-end
    headers).
-   Oscilloscope (very optional; lets you look at the PWM output and
    SPI traffic directly).

## 2.4 Software you'll install

On your **laptop** (the dev workstation):

-   **STM32CubeIDE** ≥ 1.13.0 (free, from st.com) — for BSAU firmware
    development.
-   **Git** — to clone the project repo.
-   **SSH client** — Terminal on Linux/Mac, or PuTTY on Windows.
-   **A serial terminal** at 921600 baud 8N1 — `picocom`, `minicom`,
    `screen`, PuTTY, Tera Term are all fine.
-   **Python 3.10+** with `pyserial` (for the dataset collector);
    optionally `matplotlib` if you want a live scope view.

On the **Raspberry Pi 5**: the `setup_pi.sh` script in the CPCU
repo does everything for you. Section 4.3 walks through it.

---

# Part 3 — Hardware Setup

## 3.1 Safety first

Before you plug anything into anything:

1.  Keep one hand in your pocket when probing live circuits. One
    hand's worth of current through your chest can stop your heart;
    both hands' worth definitely will.
2.  **The NRF24L01+ is 3.3 V tolerant only.** Applying 5 V will
    burn it out instantly. Quick sanity check: the NRF module has
    "VCC" silk-screened on one of the corner pins; that pin must
    connect to 3.3 V, not 5 V.
3.  Always power the board from USB first, PSU second. The Pi
    doesn't enumerate USB peripherals that appear before it's
    booted.
4.  Never work with a battery connected and a power supply
    connected at the same time.

## 3.2 BSAU — wiring the STM32 Nucleo

The NUCLEO-L432KC is a 32-pin breadboard-friendly board (it fits a
standard breadboard with room for jumpers on either side). Pin
labels are printed on the silkscreen: each side has a double-row of
labels, the inner row showing the Arduino-style name (`D0`, `A0`,
etc.) and the outer row showing the STM32 pin (`PA0`, `PB6`, ...).
**Always wire to the STM32 pin name** — that's what the code
references.

If you need to cross-reference a silkscreen Arduino label (`D5`) to
a STM32 pin (`PB6`), the `NUCLEO-L432KC` user manual **UM1956**
has the full table in section 6.11.

### 3.2.1 NRF24L01+ wiring

The NRF module is an 8-pin female header (2×4). Pin 1 of the header
is closest to the silk-printed "GND" marker and the antenna is on
the opposite end.

| NRF module pin | STM32 pin | Signal | Note |
|----------------|-----------|--------|------|
| VCC (usually pin 2) | 3V3 (Nucleo "3V3") | Power | **Must be 3.3 V** — check with multimeter before plugging the NRF |
| GND (pin 1) | GND (Nucleo "GND") | Ground | Any GND on the Nucleo works |
| CSN (pin 4) | **PB7** | SPI chip select, active-low | |
| SCK (pin 5) | **PB3** | SPI clock, 5 MHz | Also shared with JTDO — SWD still works |
| MOSI (pin 6) | **PB5** | SPI data to NRF | |
| MISO (pin 7) | **PB4** | SPI data from NRF | **Enable internal pull-up** — the pin shares with NJTRST and floats otherwise |
| CE (pin 3) | **PA8** | NRF chip enable | Initialises LOW (radio standby) |
| IRQ (pin 8) | **PB6** | Active-low interrupt from NRF | Not required at run time but connect it anyway — the firmware reads it in test modes |

**Critical: place a 10 µF tantalum or electrolytic cap + a 100 nF
ceramic cap between NRF VCC and GND, as close to the module pins as
possible.** The NRF pulls fast current spikes during TX and without
these caps you'll see intermittent radio failures that look like
"packet loss" but are actually brownouts.

### 3.2.2 EMG front-end wiring (summary)

Your EMG amplifier's 8 outputs go to **PA0–PA7** on the Nucleo. The
reference voltage of the amplifier stage should be set to **~1.65 V**
(half of VDDA = 3.3 V) so the amplified EMG signal swings above and
below that midpoint. The STM32's ADC reads 0–3.3 V → 0–4095, so a
1.65 V DC bias lets both positive and negative muscle activations
show up as a deviation from ADC count 2048.

| STM32 pin | Nucleo silk | ADC channel | Notes |
|-----------|-------------|-------------|-------|
| PA0 | A0 | IN5 | "Fast" channel (datasheet table 64) |
| PA1 | A1 | IN6 | "Fast" channel |
| PA2 | A3 | IN7 | Slow |
| PA3 | A4 | IN8 | Slow |
| PA4 | A5 | IN9 | Slow |
| PA5 | A6 | IN10 | Slow |
| PA6 | A7 | IN11 | Slow (new in v2.0) |
| PA7 | D9? | IN12 | Slow (new in v2.0) — check silkscreen |
| PB0 | D3 | IN15 | Battery monitor (divider, 100k/100k) |

**Note on PA6/PA7:** These two channels were added in v2.0. The ADC
scan covers all 9 ranks (8 EMG on PA0–PA7 + battery on PB0) and the
packet codec transmits all 8 EMG channels. Channels 6 and 7 carry
real amplifier output — the "garbage until the scan is widened"
caveat from v1 of this guide no longer applies.

### 3.2.3 Battery monitor (if used)

If you're running on battery, a resistor divider takes the pack
voltage down to a range the STM32's ADC can measure.

```
     BATT+ ────┬──── [100 kΩ] ──── PB0 ──── [100 kΩ] ──── GND
               │                    │
               │                    └──── 12-bit ADC input
               ▼
              ... to board's 3.3 V regulator ...
```

For a 2S Li-ion pack (7.4 V nominal, 8.4 V full, 6.0 V depleted):
the divider gives ~3.7 V at PB0 when the battery is full — too high
for the ADC. **Use a 10:1 divider (say 180 kΩ + 20 kΩ) instead for
a 2S battery, or a simple 2:1 divider for a single Li-ion (3.7 V
nominal).** Update `BATT_LOW_THRESHOLD` in `bsau_app.h` to match
your chosen divider.

### 3.2.4 Debug interface

The Nucleo's on-board ST-LINK provides both **SWD flashing** and a
**virtual COM port** through the single Micro-B USB connection. You
don't need to wire anything extra — just plug in the USB cable to
your laptop.

Upon plugging in:

-   Linux: a `/dev/ttyACM0` (or ACM1, ACM2, ...) appears, plus an
    ST-LINK mass-storage device called `NODE_L432KC`.
-   macOS: `/dev/tty.usbmodem<serial>` appears.
-   Windows: a new COM port appears in Device Manager.

That virtual COM port IS USART1 (PA9 TX / PA10 RX) internally. So
when the firmware emits `LOG(...)` messages or DATASET CSV, that's
where they come out. Nothing to wire.

### 3.2.5 Status LED

The Nucleo already has a user LED (LD3) wired to **PA15**. The
firmware blinks it at 1 kHz in RELEASE/DEBUG/DATASET mode (one
toggle per packet). At 1 kHz it looks to the eye like a steady dim
glow, not a blink — that's correct. If you see real blinking
(visible on/off), the loop is running at a much lower rate than it
should; go to Section 5.4 testing.

## 3.3 CPCU — wiring the Raspberry Pi 5

The Pi 5 uses the standard 40-pin GPIO header. Pin 1 is the corner
closest to the microSD card slot.

### 3.3.1 NRF24L01+ on Pi (SPI0)

| NRF module pin | Pi GPIO (BCM) | Pi physical pin | Notes |
|----------------|----------------|------------------|-------|
| VCC | 3V3 | Pin 1 or 17 | **Must be 3.3 V!** 5 V fries the NRF |
| GND | GND | Pin 6, 9, 14, 20, 25, 30, 34, 39 | Any GND |
| CSN | GPIO 8 (CE0) | Pin 24 | SPI0 chip select |
| SCK | GPIO 11 (SCLK) | Pin 23 | SPI0 clock, 8 MHz |
| MOSI | GPIO 10 (MOSI) | Pin 19 | |
| MISO | GPIO 9 (MISO) | Pin 21 | |
| CE | **GPIO 25** | Pin 22 | Held HIGH in RX mode |
| IRQ | GPIO 24 | Pin 18 | Not used (busy-poll instead) |

Same decoupling caps (10 µF + 100 nF) as on the BSAU side. Same
rule.

### 3.3.2 PCA9685 on Pi (I²C1)

The PCA9685 breakout has 2 connectors: a 6-pin I²C header for
control signals, and a 2-pin terminal block for servo power.

Control side:

| PCA9685 pin | Pi GPIO (BCM) | Pi physical pin |
|-------------|----------------|------------------|
| VCC | 3V3 | Pin 1 (same 3V3 rail as NRF is fine) |
| GND | GND | Pin 14 (any GND) |
| SDA | GPIO 2 (SDA1) | Pin 3 |
| SCL | GPIO 3 (SCL1) | Pin 5 |
| OE  | — (leave floating or pull LOW) | — |

**I²C pull-ups:** Most PCA9685 breakouts (Adafruit, generic AliExpress
clones) have 4.7 kΩ pull-ups already populated on SDA and SCL. Verify
by looking at the board silkscreen or the back. If missing, add
4.7 kΩ from SDA to 3.3 V and SCL to 3.3 V externally. Without them
you get intermittent `i2c-bcm2835` errors.

**Address:** the PCA9685's address is `0x40` by default. If you need
a different address (e.g. to cohabit with another I²C device),
short the A0–A5 solder jumpers on the back — each adds a bit to the
address. The code expects `0x40`; changing it means editing
`cpcu_pca9685.h` too.

Servo-power side:

| PCA9685 terminal | Connect to |
|------------------|------------|
| V+ | **Positive of your external 6 V servo PSU** |
| GND | **Common GND** — must be tied to the Pi's GND *and* the servo PSU's ground |

If you forget the common GND the PWM signals are referenced to
nothing on the servo side and the motors twitch randomly or not at
all.

### 3.3.3 Servo wiring

Each servo has a 3-wire cable: brown/black = GND, red = V+, yellow/
orange = signal. Plug the whole 3-pin connector directly onto one of
PCA9685's 16 output channels (matching the orientation printed on
the breakout).

Channel mapping (from `cpcu_pca9685.h` v1.1). The 6 servos are wired to
**non-contiguous** PCA9685 output terminals — the firmware uses a
logical-to-physical translation table internally, so callers (and the
gesture-mapping code in `cpcu_dsp.py`) just refer to the logical index
S0..S5:

| Logical | Function | PCA terminal | Servo type | Pulse range |
|---------|----------|--------------|------------|-------------|
| S0 | Base    | 0  | MG995 |  498–2500 µs |
| S1 | Upper   | 1  | MG995 | 1074–1953 µs |
| S2 | Last    | 11 | MG995 | 1074–1953 µs |
| S3 | Joint-1 | 8  | SG90  | 1001–2002 µs |
| S4 | Joint-2 | 5  | SG90  | 1001–2002 µs |
| S5 | Gripper | 4  | SG90  |  976–1733 µs |

When you press `↑`/`↓` in `pca_testbench` to select "S2 Last", the
driver writes to **PCA9685 channel 11**, not channel 2. Likewise S3
goes to channel 8, S4 to channel 5, S5 to channel 4. If you reroute
cables, edit the `PCA_SERVO_CHANNEL` macro in `cpcu_pca9685.h` to
match — that single line is the source of truth.

The PCA9685 handles the PWM frequency (50 Hz = 20 ms period) and
pulse width independently per channel; it has its own 25 MHz
oscillator, so the Pi's CPU load doesn't affect servo jitter.

### 3.3.4 Power architecture

```
     USB-C PSU (5 V / 5 A)                   6 V benchtop PSU (3 A)
           │                                         │
           ▼                                         ▼
     ┌─────────┐     3V3 rail       ┌──────────────────────────┐
     │  Pi 5   │ ───────────────────▶│  PCA9685 VCC (logic)     │
     │         │                     └──────────────────────────┘
     │  GND ───┼─── common GND ──────────▶ PCA9685 GND
     └─────────┘
                              V+ from 6V PSU ─────▶ PCA9685 V+
                                                           │
                                                           ▼
                                                       6 servos
```

## 3.4 Pre-power checks (before you plug anything in)

Go through this checklist with a multimeter BEFORE you connect USB or
turn on a PSU. It takes five minutes and saves days of debugging.

**BSAU side:**

1.  Multimeter to continuity mode. Probe:
    -   NRF VCC pin ↔ Nucleo 3V3 silk — should beep.
    -   NRF GND pin ↔ Nucleo GND silk — should beep.
    -   NRF CSN ↔ Nucleo PB7 silk — should beep.
    -   (repeat for SCK/MOSI/MISO/CE/IRQ).
2.  Multimeter to resistance mode. Probe NRF VCC ↔ GND. You should
    read **megaohms** (>1 MΩ). **A few ohms means a short — DO NOT
    POWER.**
3.  Visual: look at the NRF's 8-pin header. No bent pins, no solder
    bridges, no pins in the wrong socket.

**CPCU side:**

1.  Multimeter continuity:
    -   NRF VCC ↔ Pi pin 1 or 17 (3V3) — should beep.
    -   NRF GND ↔ Pi GND — should beep.
    -   NRF SCK ↔ Pi pin 23 — should beep.
    -   (repeat for the rest).
    -   PCA9685 SDA ↔ Pi pin 3 — should beep.
    -   PCA9685 SCL ↔ Pi pin 5 — should beep.
    -   PCA9685 V+ terminal ↔ servo PSU positive — should beep.
    -   PCA9685 GND terminal ↔ Pi GND — should beep.
    -   PCA9685 GND terminal ↔ servo PSU negative — **must beep**
        (common ground).
2.  Multimeter resistance:
    -   NRF VCC ↔ GND >1 MΩ.
    -   PCA9685 V+ ↔ GND >1 MΩ.
    -   Pi's 3V3 ↔ GND (only with Pi unpowered) several hundred kΩ.
3.  Verify the servo PSU output voltage **before** connecting it to
    PCA9685. Set to 6.0 V, turn it on, measure with multimeter, turn
    it off, then plug in. 7.4 V pack from a 2S lipo is also OK for
    most servos but will make SG90s run hot.

## 3.5 First power-on

**Order matters** because mis-sequencing can brown out the 3.3 V rail
and make peripherals enumerate in a broken state.

BSAU:

1.  Plug USB cable into the Nucleo.
2.  Plug the other end into your laptop.
3.  The red LD1 power LED on the Nucleo should light immediately.
4.  LD3 (user LED) stays dark until firmware runs. Flashing fresh
    firmware comes in Part 4.

CPCU:

1.  Insert the flashed microSD card into the Pi (see §4.3 for
    flashing).
2.  Plug in Ethernet or confirm Wi-Fi settings are in place.
3.  Plug in the USB-C power supply to the Pi.
4.  Pi's red power LED lights immediately; green ACT LED blinks a
    few times as it boots.
5.  **Only after** the Pi finishes booting (green LED settled), turn
    on the servo PSU. If you turn them on simultaneously the
    inrush from the servos' capacitors can trip the PSU's
    over-current and the servos end up un-initialised.

---

# Part 4 — Software Setup

## 4.1 Your laptop (dev workstation)

**Linux (Debian/Ubuntu):**

```bash
sudo apt update
sudo apt install -y git build-essential cmake libncurses-dev \
                    python3 python3-pip openssh-client picocom
pip install --user pyserial matplotlib pandas
```

**macOS:**

```bash
# If you don't have Homebrew: https://brew.sh/
brew install git cmake python3 screen
pip3 install pyserial matplotlib pandas
```

**Windows:** install Git for Windows, PuTTY, and Python 3 from python.org.
Open a Git Bash terminal for everything that follows; `pip install
pyserial matplotlib pandas` works there.

## 4.2 BSAU toolchain (STM32CubeIDE)

1.  Download STM32CubeIDE from
    `https://www.st.com/en/development-tools/stm32cubeide.html`
    (free, requires a free ST account). Any recent version (≥
    1.13.0) works.

2.  Install it with all default options. On Linux the installer asks
    for a sudo password at one point to install udev rules for the
    ST-LINK USB devices — answer yes; without those rules you have
    to use `sudo` every time you flash.

3.  Launch it. Pick any workspace folder (default is fine).

4.  Import the BSAU project:
    -   `File → Import → General → Existing Projects into Workspace`
    -   Click `Next`, then `Browse`, navigate to the `bsau_v2/`
        directory of the cloned repo, select it, and hit `Finish`.

5.  Right-click the project in the Project Explorer → `Properties
    → C/C++ Build → Environment`. Verify `PATH` includes the ARM GCC
    toolchain (CubeIDE ships with one). Usually nothing to change.

6.  Hit **Ctrl-B** (or `Project → Build All`). First build takes
    ~30 s; expected: **zero errors**.

    -   Warnings about `assert_param` are harmless.
    -   "Unresolved include" means CubeIDE didn't pick up the HAL
        source location. Fix via
        `Properties → C/C++ General → Paths and Symbols → Includes`.

7.  Connect the Nucleo via USB if you haven't yet.

8.  First flash: click the green bug icon (`Run → Debug`). CubeIDE
    builds, flashes, and drops you at `main()` with the debugger
    attached. Press **F8** (Resume) to run.

9.  Subsequent flashes: click the green play icon (`Run → Run`) —
    faster, no debugger attach.

**Verification of toolchain:** after the first successful flash, the
LD3 LED on the Nucleo starts blinking (actually at 1 kHz, visually
it looks like a steady dim glow in RELEASE/DEBUG/DATASET modes). If
it's fully off, the flash didn't take — check the console.

## 4.3 Raspberry Pi OS bring-up

### 4.3.1 Flash the SD card

1.  Install **Raspberry Pi Imager** from rpi.org.
2.  Insert your microSD card into your laptop.
3.  Launch Raspberry Pi Imager.
4.  Choose OS: `Raspberry Pi OS (64-bit)` — the "Lite" variant is
    fine and saves SD space.
5.  **Before hitting "Write"**, click the gear icon (or press
    Ctrl+Shift+X) to edit the pre-install settings:
    -   Set hostname (e.g., `cpcu`).
    -   Enable SSH with password auth (or, better, paste in your
        public key).
    -   Set username + password (this project's docs assume `pi`).
    -   Configure Wi-Fi if you're not using Ethernet.
    -   Set locale/timezone.
6.  Click `Save`, then `Write`. ~5 min.

### 4.3.2 First boot

1.  Insert the microSD into the Pi. Plug in Ethernet and power.
2.  Wait ~60 s for first boot (cloud-init extends the filesystem to
    fill the SD card, runs your pre-install settings).
3.  SSH from your laptop:
    ```bash
    ssh pi@cpcu.local
    # or if mDNS doesn't work, find the Pi's IP with `nmap` or your
    # router's DHCP table and use the IP directly.
    ```
4.  Update everything:
    ```bash
    sudo apt update && sudo apt -y full-upgrade
    sudo reboot
    ```
5.  SSH back in after reboot.

### 4.3.3 Run setup_pi.sh

```bash
sudo apt install -y git
git clone <your-repo-url> ~/cpcu_v2
cd ~/cpcu_v2
./setup_pi.sh
sudo reboot
```

**What `setup_pi.sh` actually does** (read it before running — it
makes system-level changes):

1.  Installs build toolchain: `gcc`, `cmake`, `libncurses-dev`.
2.  Installs Python deps: `numpy`, `scipy`, `scikit-learn`, `joblib`.
3.  Enables SPI and I²C in `/boot/firmware/config.txt`.
4.  Appends `isolcpus=1,2,3 nohz_full=1,2,3 rcu_nocbs=1,2,3` to
    `/boot/firmware/cmdline.txt` — reserves cores 1-3 for our code.
5.  Locks CPU governor to `performance` and sets ARM clock to
    2.8 GHz.
6.  Creates `/opt/cpcu/{bin,scripts,models,test}` and `/var/log/cpcu/`.

### 4.3.4 Verify the Pi setup after reboot

SSH back in after the reboot and run each of these — **every one
must produce the expected output**:

```bash
cat /sys/devices/system/cpu/isolated
# Expected: "1-3"  — if empty, isolcpus didn't apply. Re-run setup_pi.sh.
```

```bash
ls /dev/spidev0.0
# Expected: /dev/spidev0.0  — if "No such file", SPI isn't enabled.
# Fix: sudo raspi-config → Interface Options → SPI → Enable.
```

```bash
ls /dev/i2c-1
# Expected: /dev/i2c-1  — if missing, I²C isn't enabled. Same raspi-config menu.
```

```bash
vcgencmd measure_clock arm
# Expected: frequency(48)=2800000000  (2.8 GHz)
# If lower (say 1.5 GHz), the overclock didn't stick — the active cooler
# might not be seated or the PSU is under-spec.
```

```bash
vcgencmd measure_temp
# Expected: < 60 °C at idle; anything above 70 °C at idle means cooling is
# inadequate and sustained 2.8 GHz will throttle.
```

```bash
python3 -c "import numpy, scipy, joblib, sklearn; print('OK')"
# Expected: "OK". Any ModuleNotFoundError means a pip install failed.
```

```bash
i2cdetect -y 1
# Expected: the PCA9685 shows as "40" at row 40 column 0 (once it's wired).
# If it shows "--" everywhere but the row/col labels, the PCA9685 is
# unpowered, the pull-ups are missing, or wiring is wrong.
```

## 4.4 Cloning the project

If you haven't already in §4.3.3, on both the laptop and the Pi:

```bash
git clone <your-repo-url>
```

Directory layout after clone:

```
prosthetic_hand/
├── bsau_v2/          ← STM32 firmware (opened in CubeIDE on laptop)
└── cpcu_v2/          ← Pi codebase (built on the Pi)
```

Both projects share the source of truth for `wireless_packet.h` —
do not modify one without the other, or BSAU and CPCU will decode
packets differently and fall silently out of sync.

---

# Part 5 — Testing (in the correct order)

## 5.1 Why bottom-up matters

Tests are layered by dependency. If a lower layer is broken, a test
on a higher layer produces meaningless numbers — you'll chase a
ghost for hours.

```
    Phase 0  Host-only software tests      (no hardware at all)
    Phase 1  BSAU standalone               (STM32 + NRF, no Pi)
    Phase 2  CPCU standalone               (Pi + peripherals, no STM32)
    Phase 3  Integration                   (both boards)
    Phase 4  Qualification                 (endurance, fault recovery)
```

**Rule:** if Phase N fails, **stop** and fix. Do not run Phase N+1
and try to interpret its numbers.

## 5.2 The full test sequence at a glance

| # | Phase | Runs on | Needs | Duration |
|---|-------|---------|-------|----------|
| 1 | Host software tests | Laptop | Nothing | 2 min |
| 2 | CPCU `--demo` smoke | Laptop | Nothing | 2 min |
| 3 | BSAU boot sanity | STM32 alone | USB cable | 1 min |
| 4 | BSAU codec round-trip (`TEST_PKT_LOG`) | STM32 alone | USB cable | 1 min |
| 5 | BSAU ADC check (`TEST_ADC_CSV`) | STM32 + function gen | Oscilloscope optional | 5 min |
| 6 | BSAU DFT check (`TEST_DFT_LOG`) | STM32 + function gen | — | 2 min |
| 7 | BSAU NRF self-test (`TEST_NRF_LOG`) | STM32 | USB cable | 2 min |
| 8 | Pi hardware checks | Pi alone | PCA9685 wired | 2 min |
| 9 | Pi servo calibration (`pca_testbench`) | Pi + servos + 6 V | — | 10 min |
| 10 | Safety FSM bench | Pi | Nothing | 1 min |
| 11 | First packet (RELEASE) | Both | Everything | 2 min |
| 12 | Signal integrity (`signal_testbench`) | Both + function gen | — | 10 min |
| 13 | Safety timeout | Both | — | 2 min |
| 14 | 1-hour endurance | Both | — | 60 min |

## 5.3 Software-only tests (no hardware)

These run on your laptop (or any machine with a C compiler and
Python 3). Do these before you touch a soldering iron.

### 5.3.1 CPCU Phase 1 — codec and DSP

```bash
cd ~/cpcu_v2
mkdir -p build && cd build
cmake ..
make -j4
```

Expected: build produces `cpcu_io`, `cpcu_kernel`, `cpcu_tui`,
`pca_testbench`, `signal_testbench`, `test_codec`, `safety_testbench`
with no errors.

```bash
./test_codec
```

**Expected output:**
```
RESULTS: 7 PASS, 0 FAIL
```

What this actually proves: the `WL_Pack`/`WL_Unpack` 12-bit codec is
bit-exact for every ADC value from 0 to 4095; the ring buffer works
under concurrent push/pop; the sequence gap detector handles wrap-
around at `seq 0xFFFF → 0x0000`. **If this fails, nothing else
works** — the BSAU's packets will decode to garbage on the Pi and
there's no ground truth to cross-check against.

```bash
python3 ../test/test_dsp_pipeline.py
```

**Expected output:** all assertions PASS, ending in `OK`.

This checks that the Python DSP pipeline produces the features the
model expects (column order, filter response, RMS values on known
inputs). **If this fails and you haven't touched the code**, your
`sklearn`/`numpy` version is different from what the `.pkl` was
trained on. `pip install 'scikit-learn==<original_version>'` fixes
it.

### 5.3.2 Safety FSM bench

```bash
./safety_testbench
```

Exercises the safety finite-state machine with synthetic inputs. 7
test groups, 31 assertions, all must PASS. Covers:

1.  Happy path — FSM stays `RUNNING` forever.
2.  Radio-loss timeout — `RUNNING → DEGRADED` at 750 ms, `DEGRADED →
    SAFE` at 1500 ms more.
3.  Low battery trip — `vbat_raw` encoding <2.7 V forces `SAFE`.
4.  Seq gap storm — loss >5% over 1000 packets forces `SAFE`.
5.  Ring overflow — >100 drops forces `SAFE`.
6.  I²C error streak — 5 consecutive failures force `SAFE`, single
    success recovers.
7.  State-transition graph — no illegal edges under any input.

### 5.3.3 TUI demo (read-only, no shared memory)

```bash
./cpcu_tui --demo
```

Press `1`, `2`, `3`, `4`, `5`, `6`, `7` in turn and verify each page
renders. Press `q` to quit. You should see:

-   Page 1 (Overview): rolling EMG bars, gesture name updating,
    colour-coded HEALTH banner at the top.
-   Page 2 (Radio): packet counter ticking at ~1000/s,
    last-packet hex dump updating.
-   Page 3 (DSP): inference count ticking, per-class confidence
    bars animating.
-   Page 4 (Waveforms): 8 animated channels. `UP`/`DOWN` selects;
    `TAB` toggles grid ↔ single-channel detail.
-   Page 5 (Health): 10 traffic-light rows.
-   Page 6 (Dataset): dataset collection UI. Label picker cycles
    with `←`/`→`. `s`/SPACE and `r` keys are intercepted; capture
    works in demo mode against the synthetic packet stream.
-   Page 7 (Config): static spec sheet *(moved to last tab in v3.4
    so live-data pages occupy the lowest keys)*.

Try the fault injection:
-   `F` injects radio freeze → HEALTH banner turns yellow then red;
    ~2.25 s later state flips to `SAFE`.
-   `B` injects low battery → battery pill goes red.
-   `G` injects seq-gap storm; `O` injects ring overflow (auto-clears
    after ~5 s once the burst stops, v2.3 recovery); `I` injects
    I²C failure.
-   `R` resets everything (faults cleared, counters zeroed).

**What this validates:** the TUI code compiles, runs, reflows on
terminal resize, and reacts to fault-injection keys. It doesn't
validate real hardware — for that, you still need the boards.

## 5.4 BSAU standalone tests (just the STM32, no Pi)

These tests exercise the BSAU in isolation. All you need is the
Nucleo and a USB cable.

### 5.4.1 Phase 3 — Boot sanity

1.  In `bsau_config.h`, confirm exactly one `BSAU_MODE_*` macro is
    uncommented. For this test pick `BSAU_MODE_DEBUG`.
2.  In CubeIDE: Build All (Ctrl+B), then Run (green play).
3.  Open a serial terminal on the Nucleo's virtual COM port at
    **921600 8N1**:
    ```bash
    # Linux:
    picocom -b 921600 /dev/ttyACM0

    # macOS:
    screen /dev/tty.usbmodem<serial> 921600

    # Windows (PuTTY): COM<n>, 921600 baud, 8N1, no flow control.
    ```
4.  Press the NRST button on the Nucleo. Watch the terminal.

**Expected output:**

```
[BSAU - APP ]: BSAU_Init         [RUN ]
[BSAU - APP ]: BSAU_Init         [OK  ] NVIC priority group verified
[BSAU - APP ]: TIM2_Start        [OK  ] 1 MHz free-running counter live
[BSAU - NRF ]: NRF_Init          [RUN ] ch=76 (POR wait 200ms done)
[BSAU - NRF ]: NRF_Init          [OK  ]
[BSAU - ADC ]: BSAU_ADC_Init     [RUN ] Calibrating ADC1...
[BSAU - ADC ]: BSAU_ADC_Init     [OK  ] Calibration complete
[BSAU - ADC ]: BSAU_ADC_Init     [OK  ] Pipeline running (TIM6 trig, DMA circ)
[BSAU - APP ]: BSAU_Init         [OK  ] Pipeline live
[BSAU - APP ]: BSAU_Run          [INFO] seq=67 batt=1945 lvl=0 retry=0 loss=15 ok=329 lost=0 drop=0
```

**PASS criteria:** every line ends with `[OK  ]`, no `[FAIL]`.
One `[WARN ]` on `NRF_Init` followed by an `[OK  ]` retry is
tolerable (marginal POR timing on a dead-cold board).

**What can go wrong:**

| Symptom | Cause | Fix |
|---------|-------|-----|
| Nothing at all | Baud is wrong / wrong port | 921600 8N1, `dmesg | tail` to confirm port |
| Garbled characters | Clock wrong (MSI instead of HSI/PLL) | Verify `SystemClock_Config()` in main.c runs PLL |
| Stops at NVIC line | TIM2 clock gate off | CubeMX → TIM2 peripheral clock enabled |
| `NRF_Init [FAIL]` after 2 retries | NRF wiring bad / dead chip | Re-seat cable, swap NRF module, re-do Section 3.4 |
| `ADC calibration FAIL` | VDDA ≠ 3.3 V or ADC clock wrong | Multimeter on VDDA; CubeMX ADC clock = Sync /1 |

### 5.4.2 Phase 4 — Codec round-trip (`TEST_PKT_LOG`)

Edit `bsau_config.h`:

```c
// Comment out BSAU_MODE_DEBUG, uncomment:
#define BSAU_MODE_TEST_PKT_LOG
```

Build, flash, reset. Watch the serial terminal.

**Expected output (abridged):**

```
[BSAU - TEST]: PacketVerify       [RUN ] === WL CODEC ROUND-TRIP START ===
[BSAU - TEST]: PKT_Ramp           [RUN ] ramp 0x100..0x600, seq=0x55, flags=0x03
[BSAU - TEST]: PKT_Seq            [PASS] got=0x55 exp=0x55
[BSAU - TEST]: PKT_Flags          [PASS] got=0x03 exp=0x03
[BSAU - TEST]: PKT_Samples        [PASS] All 16 samples match
...
[BSAU - TEST]: PacketVerify       [PASS] === WL CODEC: 23 PASS, 0 FAIL ===
[BSAU - TEST]: BSAU_Test_Run      [INFO] Tests complete. Idling.
```

**PASS criteria:** final summary shows `N PASS, 0 FAIL`; LD3 starts
a slow 1 Hz blink (the idle indicator).

**What it proves:** `WL_Pack` packs 12-bit ADC values into the
32-byte wireless payload correctly, `WL_Unpack` recovers them bit-
exact, every field round-trips. If this fails, don't bother
flashing RELEASE.

### 5.4.3 Phase 5 — ADC pipeline (`TEST_ADC_CSV`)

**What you need:** a function generator or a known-amplitude signal
source. If you have nothing, you can use a short wire to touch the
PA0 pin and watch the noise spike.

Edit `bsau_config.h`:

```c
#define BSAU_MODE_TEST_ADC_CSV
```

Build, flash, reset.

This mode streams the raw ADC readings as a binary packet every
500 µs. Visualise with SerialPlot:

1.  Install SerialPlot: `https://github.com/hyOzd/serialplot`
2.  File → Settings → open serial port at 921600 8N1.
3.  Data format: `Binary`; channel layout: 9 channels × `int16 LE`;
    frame start bytes `0xAA 0x55` (or whatever the firmware uses —
    check `bsau_test.c` for the sync bytes). Sample rate: 2000 Hz.
4.  Hit "Connect."

**Expected:** 9 rolling traces (8 EMG + 1 battery). Touching PA0
with a finger should make ch0 spike and only ch0. Each channel
should move independently.

**What can go wrong:**
-   All channels move identically → the ADC scan has collapsed to
    one channel. Check CubeMX ADC rank sequence; all ranks pointing
    to the same IN number is the classic sign.
-   Battery channel jumps around at random → PB0 is floating or the
    battery divider is wrong.

### 5.4.4 Phase 6 — Goertzel DFT (`TEST_DFT_LOG`)

**What you need:** a function generator that can produce a clean
sine into PA0.

Edit `bsau_config.h`:

```c
#define BSAU_MODE_TEST_DFT_LOG
```

Build, flash. Feed PA0 a **200 Hz sine, 0.5 Vpp, 1.65 V DC offset**.

Watch the UART:

```
[BSAU - TEST]: DFT_ch0   [INFO] peak=200Hz mag=1025   (200 +- 3.9 Hz = OK)
[BSAU - TEST]: DFT_ch0   [INFO] peak=200Hz mag=1031
...
```

**PASS criteria:** reported peak is within one bin (±3.9 Hz at
512-point block, 2000 Hz sample rate) of 200 Hz.

**If the peak says 180 Hz or 220 Hz:** the ADC isn't actually
running at 2000 Hz. The most common cause is a wrong TIM6 prescaler;
verify `tim.c` has `TIM6->PSC` and `TIM6->ARR` giving
`80_000_000 / ((PSC+1) * (ARR+1)) = 2000`.

### 5.4.5 Phase 7 — NRF self-test (`TEST_NRF_LOG`)

Edit `bsau_config.h`:

```c
#define BSAU_MODE_TEST_NRF_LOG
```

Build, flash, reset. Watch the UART.

This runs **three interleaved tests forever**:
-   TX ping with no receiver — expected to MAX_RT every time
    (because nothing's listening), but `PLOS_CNT` and `ARC_CNT`
    should increment as expected.
-   Health check every 20 pings: SPI loopback + register audit +
    FIFO exercise. Non-destructive.
-   Stress cycle every 100 pings: power-cycle + re-init. Invasive;
    verifies the init sequence is idempotent.

**Expected output:**

```
[BSAU - TEST]: NRF_SelfTest   [PASS] Self-test suite passed
[BSAU - TEST]: NRF_TxPing     [FAIL] iter=1 PLOS=1 ARC=15
[BSAU - TEST]: NRF_TxPing     [FAIL] iter=2 PLOS=2 ARC=15
[BSAU - TEST]: NRF_Health     [PASS] registers + FIFO OK
...
[BSAU - TEST]: NRF_Summary    [INFO] tx_ok=0 tx_fail=49 rate=0% health=2/2 t=7s
```

**PASS criteria:**
-   `NRF_SelfTest [PASS]` at boot
-   `NRF_Health [PASS]` every 3 s
-   `NRF_Stress [PASS]` every 15 s
-   `ARC=15` on every `NRF_TxPing [FAIL]` — yes, FAIL is correct
    here (no receiver), the point is that retries *happened*.

**What can go wrong:**
| Symptom | Cause | Fix |
|---------|-------|-----|
| `NRF_SelfTest [FAIL]` at boot | SPI wiring bad | Redo §3.2.1 checklist |
| `NRF_SelfTest [PASS]` but `ARC = 0` | Radio stuck in RX | Check that `CONFIG` register writes include `PRIM_RX=0` |
| `PLOS` doesn't increment | CSN never pulled low | Bad SPI transfer — check driver |

## 5.5 CPCU standalone tests (just the Pi, no STM32)

### 5.5.1 Phase 8 — Pi hardware automated checks

SSH to the Pi and run:

```bash
cd ~/cpcu_v2
./run_tests.sh 3
```

This runs a script that checks:
-   `cat /sys/devices/system/cpu/isolated` = "1-3"
-   `/dev/spidev0.0` exists
-   `/dev/i2c-1` exists
-   `i2cdetect -y 1` sees 0x40
-   PCA9685 `PRESCALE` register readable
-   PCA9685 init smoke-test succeeds
-   CPU clock >= 2.8 GHz
-   CPU temp < 80 °C

**Expected:** every line ends `[PASS]`. Any `[FAIL]` is explained by
the adjacent message; usually a peripheral that isn't enabled.

### 5.5.2 Phase 9 — PCA9685 servo calibration

**Connect servos first.** Turn on the 6 V PSU, confirm common
ground is wired. Then:

```bash
/opt/cpcu/bin/pca_testbench
# or from the build directory:
./pca_testbench
```

A TUI opens. Controls:

| Key | Action |
|-----|--------|
| ↑/↓ | Select servo |
| ←/→ | -/+ 10 µs pulse width (1 PCA9685 tick = 4.88 µs; 5 µs was on the boundary so half the keypresses produced no change) |
| PgUp/PgDn | -/+ 50 µs |
| m / M | Jump to min / max |
| n / N | Neutral selected / all |
| 0 | Kill PWM (LED_OFF) on selected channel |
| s | Toggle slew smoother on/off |
| A | Write all 6 channels in one I²C transaction |
| r | Read back MODE1/MODE2/PRESCALE registers |
| q | Quit |

**What to verify, per servo (0 through 5):**

1.  Press `n` (neutral) — servo moves to mid-travel (~1500 µs).
2.  Press `m` (min) — servo swings to minimum (~1000 µs).
3.  Press `M` (max) — servo swings to maximum (~2000 µs).
4.  No mechanical binding at either extreme; no grinding sound.
5.  Centre command physically centres the mechanism (for a finger,
    mid-flex; for the wrist, neutral axis).
6.  Press `s` to toggle the slew smoother on. Press `M` — servo
    should slew smoothly over ~0.5 s instead of snapping.
7.  Press `r` — check MODE1 is `0x21` (AI=1, SLEEP=0).

**If a servo binds before reaching min or max**, tighten its
mechanical travel with `--min A,B,C,D,E,F` and `--max A,B,C,D,E,F`
flags on launch, measuring your new limits and then updating
`cpcu_pca9685.h` permanently.

**If a servo doesn't move at all** but the TUI says it sent the
command:
-   Check the 6 V rail at the PCA terminal block with a multimeter.
-   Check that OE is floating or LOW (not HIGH).
-   `r` readback of MODE2: SLEEP bit must be 0.
-   Swap the servo to a different channel — if it works there, the
    original PCA channel is dead.

Press `q` to quit. The testbench neutralises all servos then calls
`PCA_AllOff` (clears every PWM channel). Servos go limp — that's
expected.

## 5.6 Integration tests (both boards)

Now wire both sides up. The BSAU transmits in RELEASE mode; the
CPCU runs the full pipeline.

### 5.6.1 Phase 11 — First packet

On the BSAU: flash `BSAU_MODE_RELEASE`.

On the Pi:

```bash
# Terminal 1
sudo systemctl start cpcu
journalctl -u cpcu -f
```

**Expected within 2 seconds:**

```
[IO]  INFO  === CPCU I/O Controller (Core 3) v2.2 ===
[IO]  INFO  Ready (NRF=OK PCA=OK). Entering loop.
[IO]  INFO  pkts=1000 gaps=0 ring=5 state=RUNNING fault=OK nrf_sr=0x0E motion=IDLE
```

Every second after that, a new stats line. `pkts` should grow by
~1000 per line (the link runs at 1 kHz).

Terminal 2 (open TUI):

```bash
/opt/cpcu/bin/cpcu_tui
```

Page 1: `Pkts` climbs, `Rate: 1000 /s`, `Loss: 0.00 %`, `Battery`
reads a value that matches what the multimeter shows across BSAU
battery terminals within ±30 mV.

**Failure modes:**

| Symptom | Cause | Fix |
|---------|-------|-----|
| `Rate: 0 /s`, no packets ever | Address mismatch BSAU↔CPCU, or channel mismatch | Both must use `NRF_CHANNEL = 76` and address `{0xE7, 0xE7, 0xE7, 0xE7, 0xE7}` |
| Rate flaps 0↔1000 | 2.4 GHz interference | Move away from Wi-Fi router; channels 72–76 are the sweet spot |
| Rate = 1000 but `Loss = 50%` | Marginal RF (range, antenna) | Bring boards within 1 m; re-test |
| `state = SAFE` immediately | Safety FSM tripped on boot | Check `log_io.csv` for FAULT message |

### 5.6.2 Phase 12 — Signal integrity

Connect a function generator: **100 Hz sine, 0.5 V amplitude, 1.65 V DC
offset, applied to all 8 EMG inputs** (tie them together for the
test).

```bash
/opt/cpcu/bin/signal_testbench
```

Press `TAB` for the 2-column 8-channel grid view.

**Per-channel pass criteria:**
-   Clean sinusoid in the rolling plot (no flat-top clipping).
-   Goertzel reports dominant frequency = **100 Hz**.
-   `Vpp ≈ 1.0 V` (twice the amplitude).
-   `SNR > 20 dB`.
-   `DC Offset ≈ 1.65 V`.
-   `ADC min/max` span stays constant over 60 s (not thermally
    drifting).

If the waveform on screen does not resemble the function-generator
signal at all, the problem is **before** DSP — it's in the physical
signal chain (electrode → amp → ADC → codec → radio → ring → TUI).
Start bisecting: does the BSAU UART in TEST_ADC_CSV mode show the
sine? If yes, the radio side is the problem. If no, the amp or ADC
is the problem.

### 5.6.3 Phase 13 — Safety timeout

With the system running and servos moving in response to gestures:

1.  Power off the BSAU (unplug its USB cable).
2.  Watch Page 1 on `cpcu_tui`:
    -   `pkt_rate` drops to 0.
    -   Within ~750 ms: `state = DEGRADED`.
    -   Servos snap to neutral (1500 µs on all channels) via
        `SMOOTH_Snap`.
    -   Within 1500 ms more: `state = SAFE`; `PCA_AllOff` fires,
        servos go limp.

3.  Power the BSAU back on:
    -   CPCU sees packets, `state` cycles `INIT → RUNNING`.
    -   Servos re-engage and take their commanded positions.

**PASS criteria:** state transitions fire at the correct times,
servos actually neutralise and go limp, recovery works.

## 5.7 Qualification — endurance and recovery

### 5.7.1 Phase 14 — 1-hour endurance

```bash
sudo systemctl start cpcu
# Let it run for 60 minutes with BSAU transmitting.
# Glance at `cpcu_tui` page 1 periodically.
```

**PASS criteria after 60 min:**
-   `Pkts` ≈ 3,600,000 (1 kHz × 3600 s, ±0.5 %).
-   `Seq gaps` < 360 (< 0.01 %).
-   `Ring overflows` = 0.
-   CPU temp < 75 °C (active cooler should keep it there).
-   No `respawning` events in `journalctl -u cpcu`.
-   Python RSS memory stable (no leak):
    ```bash
    ps -o rss= -p $(pidof python3)
    # Check again 30 min later — should not have grown more than a few MB.
    ```

### 5.7.2 Process recovery test

```bash
sudo kill -9 $(pidof cpcu_io)
```

Watch the TUI:
-   `cpcu_kernel` detects the death within 2 s.
-   Respawns `cpcu_io`.
-   During the gap, safety FSM enters `SAFE` (radio watchdog and
    process-lock release both trip it). Servos go limp.
-   After respawn, state returns to `RUNNING`. Total recovery
    <5 s.

---

# Part 6 — Running the System (daily operation)

## 6.1 Pre-flight checklist

Before every session:

- [ ] BSAU's USB cable is plugged into a laptop or wall USB charger
  (the Nucleo powers from USB).
- [ ] BSAU's LD1 red LED is lit (board has power).
- [ ] Electrodes have fresh gel (if wet Ag/AgCl type) or are cleaned
  with isopropyl (if dry).
- [ ] Pi's 5 V PSU is plugged in and the Pi is fully booted (green
  ACT LED settled, not blinking constantly).
- [ ] Servo PSU is plugged in and set to 6.0 V.
- [ ] Common ground between Pi GND and servo PSU GND is in place.
- [ ] SSH from your laptop to the Pi works.

## 6.2 Typical startup sequence

Power-on order matters:

1.  **BSAU first.** Plug in USB; LD3 starts its steady dim glow
    (1 kHz blink).
2.  **Pi next.** Plug in USB-C. Wait for full boot (green ACT LED
    quiets down).
3.  **Servo PSU last.** Turn on the 6 V rail.

Then either start in production mode (systemd) or development mode
(manual).

## 6.3 Production run (RELEASE mode, systemd)

**On BSAU:** flash with `BSAU_MODE_RELEASE`. LD3 blinks (1 kHz glow).

**On Pi:**

```bash
ssh pi@cpcu.local
sudo systemctl start cpcu          # Start now
sudo systemctl enable cpcu         # Auto-start on every boot
sudo systemctl status cpcu         # Verify it's active (running)
journalctl -u cpcu -f              # Live log tail
```

In a second terminal:

```bash
ssh -t pi@cpcu.local /opt/cpcu/bin/cpcu_tui
```

To stop:

```bash
sudo systemctl stop cpcu
# Clean shutdown: servos driven to neutral → wait 300 ms → PCA_AllOff → NRF
# powered down → log files flushed → processes exit.
```

## 6.4 Development run (DEBUG mode, manual)

Use this when bringing up new hardware, modifying the firmware, or
tracking down issues — because you can see everything on the
console.

**On BSAU:** flash with `BSAU_MODE_DEBUG`. LD3 still glows at 1 kHz.

**On Pi, Terminal 1 (kernel + log):**

```bash
ssh -t pi@cpcu.local 'cd /opt/cpcu/bin && ./cpcu_kernel --log --debug'
```

This spawns `cpcu_io` and `cpcu_dsp.py` as child processes and
watches them. Logs go to both stderr and per-module CSV files in
`/var/log/cpcu/`. Hit Ctrl+C in this terminal to stop everything
cleanly.

**On Pi, Terminal 2 (TUI):**

```bash
ssh -t pi@cpcu.local '/opt/cpcu/bin/cpcu_tui'
```

**On laptop, Terminal 3 (BSAU UART in DEBUG):**

```bash
picocom -b 921600 /dev/ttyACM0
```

You'll see periodic status lines from BSAU every ~330 ms:

```
[BSAU - APP ]: BSAU_Run   [INFO] seq=234 batt=1945 lvl=0 retry=0 loss=0 ok=658 lost=0 drop=0
```

Field meanings:

| Field | Meaning | Healthy value |
|-------|---------|---------------|
| `seq` | Current packet counter (8-bit, wraps 256) | Always incrementing |
| `batt` | Raw battery ADC reading | >1861 (≈3.0 V with 2:1 divider) |
| `lvl` | Battery level: 0=OK, 1=LOW, 2=CRIT, 3=CHARG | 0 |
| `retry` | NRF ARC_CNT from previous TX | 0 most of the time |
| `loss` | NRF PLOS_CNT from previous TX | 0 most of the time |
| `ok` | Packets TX'd since boot | Monotonic |
| `lost` | TX packets that hit MAX_RT | Stays 0 indoors within 5 m |
| `drop` | ADC ISR overruns | **MUST stay 0** — non-zero is a bug |

## 6.5 Monitoring while it's running

### 6.5.1 Use tmux (one SSH connection, multiple panes)

```bash
ssh pi@cpcu.local
tmux new -s cpcu
# Inside tmux:
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

# Pane 0 (top-left):     journalctl -u cpcu -f
# Pane 1 (bottom-left):  watch -n 2 "vcgencmd measure_temp; ps -eo pid,comm,psr,pri | grep cpcu"
# Pane 2 (top-right):    /opt/cpcu/bin/cpcu_tui
# Pane 3 (bottom-right): /opt/cpcu/bin/pca_testbench   (if doing servo work)

# Navigate: Ctrl+b then arrow keys
# Detach:   Ctrl+b d
# Reattach: tmux attach -t cpcu
```

Why tmux over four SSH sessions: one TCP stream, survives Wi-Fi
drops, session persists across `ssh` disconnect/reconnect.

### 6.5.2 TUI pages overview

-   **Page 1 (Overview):** if one thing is wrong, this tells you
    which layer (radio / io / ipc / batt / dsp / fsm) before you
    have to dig.
-   **Page 2 (Radio):** raw NRF internals. Packet rate, loss rate,
    ring fill, last packet hex dump. Use when packets arrive but
    data is weird.
-   **Page 3 (DSP):** DSP windows/s, inference rate, active gesture,
    per-class softmax, servo command values. Use when radio is
    fine but prosthesis doesn't move as expected.
-   **Page 4 (Waveforms):** 8-channel rolling scope. Use when a
    specific channel looks suspicious. `UP`/`DOWN` picks, `TAB`
    toggles detail.
-   **Page 5 (Health):** 10-row traffic light. Put this on a
    second monitor during testing.
-   **Page 6 (Dataset):** capture UI, details in Part 7.
-   **Page 7 (Config):** static spec sheet *(moved to the end of the
    tab order in v3.4 so live-data pages get the lowest keys)*.

### 6.5.3 Log files

Three sources of logs:

1.  `journalctl -u cpcu -f` — full aggregate, timestamps, process.
2.  `/var/log/cpcu/log_*.csv` — per-module CSV (kern, wdg, io, nrf,
    pca). These are flushed every line so `tail -f` works.
3.  BSAU serial console in DEBUG mode — periodic status lines.

Useful `awk`-fu on the CSV logs:

```bash
# Everything that hit WARN or above:
awk -F, '$4=="WARN"||$4=="ERROR"' /var/log/cpcu/log_*.csv | column -s, -t

# NRF events in the last hour:
tail -n 10000 /var/log/cpcu/log_nrf.csv | awk -F, '$1 > ('$(date +%s)' - 3600)'
```

## 6.6 Clean shutdown

`sudo systemctl stop cpcu` runs the same sequence as Ctrl+C in
`cpcu_kernel`:

1.  Signal sent to `cpcu_io`.
2.  All servos driven to neutral pulse (1500 µs).
3.  Wait 300 ms — servos physically settle.
4.  `PCA_AllOff` — clears every PWM channel. Servos go limp (no
    torque holding).
5.  NRF powered down.
6.  I²C and SPI handles released.
7.  CSV log files flushed and closed.
8.  Python processes receive SIGTERM and exit.

The whole sequence completes in <1 s. If you SSH over an unstable
link and the connection drops mid-shutdown, the signal is already
queued to PID 1 and it still completes — but the TUI won't show
the finish. Reconnect to verify with `systemctl status cpcu`.

---

# Part 7 — Dataset Collection (v2.1)

This is how the DSP/AI team collects training data. It's the main
reason v2.1 exists.

## 7.1 Why two captures

See Part 1.5 for the full rationale. Summary:

-   **BSAU-side** (the UART stream on your laptop) gives the team a
    **golden reference** — what the ADC actually read, with no
    radio in the loop.
-   **CPCU-side** (the TUI dataset page) gives the team the
    **filtered output** — same data after the Butterworth 20-450 Hz
    bandpass + 50 Hz notch that production inference uses.

The two files from the same contraction, stacked, tell you
everything: (a) the raw muscle signal, (b) what the pipeline's
filtering does to it, and (c) — by comparing row counts — how
many packets were lost over the air.

## 7.2 Capturing BSAU-side (UART)

### 7.2.1 Flash DATASET mode

```c
// bsau_config.h — comment out whatever's active, then:
#define BSAU_MODE_DATASET
```

Build, flash, reset. No visible change on the board (LD3 still
glows at 1 kHz).

### 7.2.2 Run the collector

On your laptop, in a terminal with the repo cloned:

```bash
pip install pyserial           # once per machine
# optional: pip install matplotlib   # for --live view

cd /path/to/repo
python3 bsau_dataset_collector.py \
    --port /dev/ttyACM0 \
    --label REST \
    --output ./datasets
```

On Windows use `--port COM5` (or whatever the Nucleo enumerated as).

**What the script does:**
1.  Opens the serial port at 921600 8N1.
2.  Discards any partial first line (the firmware has been
    streaming since it booted, so the first line is always mid-
    packet garbage).
3.  Scans `./datasets/` for existing `REST_*.csv` files and picks
    the next free index (`REST_0.csv` on a clean run, `REST_1.csv`
    on the second, etc.).
4.  Prints `[collector] Writing to ./datasets/REST_0.csv`.
5.  Starts writing one line per millisecond to the file.
6.  Every second, prints a status line to stderr:
    ```
    [collector] lines=12043  rate= 1000.3/s  bad=0
    ```
7.  Runs until you press **Ctrl+C**. On SIGINT the script flushes
    the file, closes it, closes the port, and prints a summary.

**Typical workflow:**

```bash
# For each gesture in the training set, 10-30s per capture:
python3 bsau_dataset_collector.py --port /dev/ttyACM0 --label REST
# Subject rests. Press Ctrl+C.

python3 bsau_dataset_collector.py --port /dev/ttyACM0 --label H_OPN
# Subject opens hand. Press Ctrl+C.

python3 bsau_dataset_collector.py --port /dev/ttyACM0 --label H_HRD
# Subject makes a hard fist. Ctrl+C.

# ... etc for every gesture.
```

**Live view:** add `--live` and you get an 8-channel matplotlib
rolling scope. Slows the collection ~5 % due to the render load,
but very useful for spotting "electrode about to fall off" in real
time.

**PASS criteria per capture (from the summary printed at end):**
-   `rate ≈ 1000 lines/s`
-   `bad lines = 0` (or <0.01 % if a USB hub is in the path)
-   The resulting file opens cleanly in pandas:
    ```python
    import pandas as pd
    df = pd.read_csv('./datasets/REST_0.csv', header=None)
    print(df.shape)    # should be (N, 8), N ~= 1000 * duration_seconds
    print(df.describe())
    ```

### 7.2.3 Sanity-check on the wire

If you're not sure the BSAU is actually streaming, you can peek at
the raw bytes without any script:

```bash
# Linux: configure the port
stty -F /dev/ttyACM0 921600 cs8 -cstopb -parenb -icanon -echo
head -c 500 /dev/ttyACM0
# Expect CSV lines like: 2048,2049,2047,2048,2050,2047,2051,2048\r\n
```

## 7.3 Capturing CPCU-side (TUI Page 6)

With BSAU still in DATASET mode (same radio, same packets, no
reconfiguration needed):

```bash
ssh -t pi@cpcu.local /opt/cpcu/bin/cpcu_tui
```

Press `6` to jump to the DATASET page (was `7` before v3.4 — see the
v3.4 page-order change in `CPCU_ARCHITECTURE.md` §8.1).

**Page 6 layout:**

```
State: IDLE              Label: [0] REST              Mode: FILTERED
Samples: 0               Elapsed: 0.000s              Gaps: 0   Missed: 0
Out dir: ./datasets/
File: (none yet)

Labels (<-/-> to cycle, s:start/stop, t:toggle raw, r:cancel):
 REST   H.SLO   H.HRD   H.OPN   A.BND<   A.BND=   A.BND>   A.SLO   A.FST   BICEP

Live waveforms (raw ADC, 512 samples @ 2 kHz = 256 ms window):
  ch0 ...     ch4 ...
  ch1 ...     ch5 ...
  ...
```

### 7.3.1 Key bindings on Page 6

| Key | Action |
|-----|--------|
| `←`/`→` | Cycle through labels (disabled while collecting) |
| `s` / SPACE | Start / stop collection |
| `r` | Cancel current capture and **delete** the partial file |
| `t` | Toggle RAW-only ↔ FILTERED-only output (default FILTERED) |
| `1`-`7` | Switch pages (works while collecting too) |
| `q` | Quit TUI (auto-stops and saves any active capture) |

### 7.3.2 A typical capture

1.  Press `←` or `→` until the label you want is highlighted
    (reverse video). For the example, stop on `REST`.
2.  Press `s` (or SPACE). State line turns red with `● COLLECTING`;
    the filename `./datasets/REST_0.csv` (or next free N) appears.
3.  Subject holds the gesture. Watch the live waveforms update;
    `Samples:` and `Elapsed:` climb.
4.  After 10-30 s, press `s` again. State shows `✓ SAVED` and
    reports the sample count + gap count.
5.  Press `←`/`→` to change label, then repeat from step 2.

### 7.3.3 FILTERED vs RAW

-   **FILTERED** (default): each row is 8 **float** channels, volts
    (DC-centred), Butterworth 20-450 Hz bandpassed + 50 Hz notch
    applied. This is what `cpcu_dsp.py` feeds the classifier.
-   **RAW**: each row is 8 **integer** channels, 0-4095 directly
    from the ADC. Identical byte-for-byte to what the BSAU UART
    emits (modulo radio packet loss) — useful for validating the
    wireless path is not corrupting data.

Toggle with `t` **before** starting a capture. Mode change mid-
capture is refused.

### 7.3.4 Status fields

-   **Samples:** rows written so far (1 row = 1 ADC scan = 0.5 ms
    of signal in filtered mode, since both scans/packet are
    written).
-   **Elapsed:** wall-clock time since `s` press.
-   **Gaps:** over-the-air packet loss detected by sequence-number
    discontinuity. 0 is ideal; a few per 30 s is normal; hundreds
    means bad RF — move closer or change channel.
-   **Missed:** ring buffer lapped the TUI reader (TUI reads at
    10 Hz; ring holds 1024 entries = ~0.5 s of packets). Should
    stay 0. If non-zero, the TUI is CPU-starved — close the
    browser, kill extraneous processes.

## 7.4 Verifying a capture pair

After a session you'll have matching pairs like:

```
laptop: ./datasets/REST_0.csv       # 8 int cols,   ~1000 rows/s
Pi:     ~/datasets/REST_0.csv       # 8 float cols, ~2000 rows/s (filtered)
```

Copy the Pi file to the laptop:

```bash
scp pi@cpcu.local:~/datasets/REST_0.csv ./cpcu_REST_0.csv
```

Then in Python:

```python
import pandas as pd

bsau = pd.read_csv('./datasets/REST_0.csv', header=None)     # (N, 8), ints
cpcu = pd.read_csv('./cpcu_REST_0.csv',     header=None)     # (2M, 8), floats

print(f"BSAU: {bsau.shape}, 1000 Hz → {bsau.shape[0]/1000:.2f} s")
print(f"CPCU: {cpcu.shape}, 2000 Hz → {cpcu.shape[0]/2000:.2f} s")

# Expected: durations match within a few hundred ms.
# cpcu rows ≈ 2 × bsau rows (CPCU logs both scans per packet).
# Any shortfall is over-the-air packet loss (see the Gaps counter).
```

---

# Part 8 — Troubleshooting

## 8.1 Symptom to cause, BSAU side

| Symptom | Probable cause | Fix |
|---------|----------------|-----|
| No `/dev/ttyACM0` when Nucleo is plugged in | USB cable is charge-only, not data | Swap cable to a known data-grade one |
| LD1 off (no power LED) | USB hub or port not powering the Nucleo | Try plugging directly into the laptop |
| LD1 on but LD3 dark | Firmware didn't boot | `picocom -b 921600 /dev/ttyACM0`, press NRST, read the banner |
| LD3 off in RELEASE/DEBUG/DATASET | `main()` didn't reach `BSAU_Run()` | Check UART for FAULT or Error_Handler trap |
| `NRF_Init [FAIL]` | Bad SPI wiring, NRF unpowered, or dead chip | **v2.4: no longer fatal** — board boots with `g_nrf_alive = false` and BSAU_Run retries every 500 packets. If `Health [OK] Recovered` never prints, re-do §3.4 multimeter checks and swap the NRF module. See `BSAU_RUN_GUIDE.md` §8 for the full recovery flow. |
| `drop` counter climbing | ADC ISR couldn't finish before the next scan | Check that no LOG is being printed in a tight loop; in DATASET, raise `BSAU_DATASET_CSV_DECIMATION` |
| Garbled UART output | Baud wrong on the laptop side | Must be 921600 8N1 |
| CPCU stopped seeing packets after switching to DATASET | UART TX blocking the main loop | Verify USART1_TX DMA is enabled; verify no stray LOG call added |
| `retry` or `loss` counters spiking | 2.4 GHz interference | Move away from Wi-Fi router; temporary: `NRF_CHANNEL = 80` |
| First packet lost every boot | NRF POR wasn't done when `NRF_Init` ran | Raise `NRF_POR_DELAY_MS` to 300 |
| Board boots with `radio OFFLINE` log line | NRF wasn't reachable at boot — **non-fatal in v2.4** | Wait one health-check interval (500 packets ≈ 0.5 s) for auto-recovery; if it persists, treat as `NRF_Init [FAIL]` row above |

## 8.2 Symptom to cause, CPCU side

| Symptom | Probable cause | Fix |
|---------|----------------|-----|
| `cpcu_kernel` won't start | `/dev/shm/cpcu_ipc` stale from previous run | `rm /dev/shm/cpcu_ipc; sudo systemctl restart cpcu` |
| "NRF init failed" in log | SPI wiring / 5 V on the NRF / wrong channel — **cpcu_io retries every 3 s** | Multimeter check; confirm `/dev/spidev0.0` exists. The cpcu_io re-init path drains FIFOs, power-cycles via PWR_DOWN, and runs `NRF_Init` again on a 3 s cadence — see `cpcu_io.c` step 6. |
| "PCA init failed" | I²C wiring / missing pull-ups / no 3.3 V to PCA | `i2cdetect -y 1` → expect `40`; if `--`, check wires and pull-ups |
| "Model not found" | `.pkl` file not in `/opt/cpcu/models/` | `scp emg_rf_model.pkl pi@cpcu.local:/opt/cpcu/models/` |
| TUI shows `Rate: 0 /s` | BSAU not transmitting, or wrong NRF channel/address | Section 5.6.1 |
| Page 4 flat lines | ADC inputs at 0 V or railed | EMG amplifier issue — check BSAU side |
| Python DSP keeps dying | `sklearn` version mismatch with the model | Run `python3 /opt/cpcu/scripts/cpcu_dsp.py` manually to see error |
| `cpcu_io` not on core 3 | `isolcpus` didn't apply | `cat /sys/devices/system/cpu/isolated` → if empty, re-run `setup_pi.sh` |
| CPU temp climbing past 80 °C | Active cooler not seated | Reseat the cooler; apply thermal paste; verify PSU is ≥5 A |

## 8.3 Symptom to cause, integration

| Symptom | Probable cause | Fix |
|---------|----------------|-----|
| BSAU TX OK, CPCU `Rate: 0 /s` | Address or channel mismatch between the two sides | Both must define `NRF_CHANNEL = 76` and address `{0xE7,0xE7,0xE7,0xE7,0xE7}` |
| High packet loss (>1 %) in the lab | 2.4 GHz WiFi overlap / antenna fouling | Move away from router; slip BSAU out of any metal case |
| Servos don't move despite `state = RUNNING` | `gesture` stuck at REST, or 6 V PSU off | Page 3: check gesture + confidence. Multimeter on 6 V rail |
| Servos twitch on startup then freeze | 6 V PSU under-sized | Needs ≥3 A peak. Try a bench PSU |
| Large `tx_retry` but `tx_loss` = 0 | Marginal RF, retries saving the link | Acceptable transient; if sustained, investigate range/antenna |
| Big `seq_gaps` visible on CPCU page 2 | BSAU `drop` is non-zero (ADC overrun) OR radio is dropping packets | Cross-reference BSAU DEBUG log `drop` field and CPCU `pkt_loss` |

---

# Part 9 — Reference

## 9.1 Command cheatsheet

**BSAU — on your laptop:**

```bash
# Pick mode by editing bsau_config.h, then build/flash in CubeIDE.

# Watch the UART (DEBUG or DATASET modes):
picocom -b 921600 /dev/ttyACM0                       # Linux
screen /dev/tty.usbmodem<serial> 921600              # macOS
# Windows: PuTTY, COMn, 921600, 8N1

# Dataset collection:
python3 bsau_dataset_collector.py --port /dev/ttyACM0 --label REST
python3 bsau_dataset_collector.py --port /dev/ttyACM0 --label H_OPN --output ./my_data
python3 bsau_dataset_collector.py --port /dev/ttyACM0 --label REST --live

# Raw UART sniff (if you want to see the bytes):
stty -F /dev/ttyACM0 921600 cs8 -cstopb -parenb -icanon -echo
cat /dev/ttyACM0

# Quick-plot a captured file:
python3 -c "
import pandas as pd, matplotlib.pyplot as plt
d = pd.read_csv('datasets/REST_0.csv', header=None)
d.plot(subplots=True, figsize=(10,8))
plt.show()"
```

**CPCU — on the Pi (usually via SSH):**

```bash
# Build
cd ~/cpcu_v2 && mkdir -p build && cd build && cmake .. && make -j4
cmake --install build

# Systemd control
sudo systemctl start|stop|restart|status|enable|disable cpcu
journalctl -u cpcu -f

# Dev mode manual launch
/opt/cpcu/bin/cpcu_kernel --log --debug       # terminal 1
/opt/cpcu/bin/cpcu_tui                              # terminal 2

# Tests
./run_tests.sh 1 2 3                                # automated
./run_tests.sh pca                                  # servo calibration TUI
./run_tests.sh signal                               # signal integrity TUI (live)
./run_tests.sh signal-demo                          # signal integrity (synthetic)
./cpcu_tui --demo                                   # TUI only, synthetic data

# Hardware verification
i2cdetect -y 1
vcgencmd measure_temp
vcgencmd measure_clock arm
cat /sys/devices/system/cpu/isolated

# Process health
ps -eo pid,comm,psr,pri | grep cpcu
chrt -p $(pidof cpcu_io)                            # Expect SCHED_FIFO 90
grep VmLck /proc/$(pidof cpcu_io)/status            # Expect non-zero

# Logs
ls /var/log/cpcu/
tail -f /var/log/cpcu/log_io.csv
awk -F, '$4=="WARN"||$4=="ERROR"' /var/log/cpcu/log_*.csv | column -s, -t
```

## 9.2 Where to go next

If you want to understand **why** the design is the way it is:

-   `BSAU_ARCHITECTURE.md` — every firmware-design decision with
    reasoning. ADC pipeline, DMA choice, NRF SPI timing, clock
    tree, packet format, battery monitoring.
-   `CPCU_ARCHITECTURE.md` — Pi-side design. Core isolation, IPC
    layout, SPSC ring, seqlock motor commands, DSP pipeline,
    safety FSM.

If you want **exact numeric test thresholds** (e.g. "what's the
exact acceptable drop rate at 5 m?", "what are the expected NRF
register values?"):

-   `BSAU_TEST_GUIDE.md` **Part B** — every TB-XXX with preconditions,
    exact expected output, pass criteria, and failure analysis tables.
-   `CPCU_TEST_GUIDE.md` — per-phase details for the Pi-side tests.

If you want to **set up a new team member**:

-   Start them on this document, then move to `BSAU_RUN_GUIDE.md`
    (BSAU operational detail) and `CPCU_RUN_GUIDE.md` (CPCU
    operational detail) for the deep dive.

If you want to **tweak the DSP or the ML model**:

-   The feature extraction and model loading are all in
    `cpcu_dsp.py`. `test/test_dsp_pipeline.py` is the safety net —
    run it after any change.
-   The filter coefficients also live baked into `cpcu_tui.c`
    (dataset page). **If you regenerate them in Python, you MUST
    update the C arrays in the same commit**, or the dataset page
    will produce files that drift away from training-time features.

If you **broke something**:

-   Part 8 of this document.
-   The per-phase "What can go wrong" tables in Part 5.
-   `BSAU_TEST_GUIDE.md` and `CPCU_TEST_GUIDE.md` for the full
    test matrix.

---

**Good luck. Take it slow, test bottom-up, and when in doubt:
multimeter first, then code.**
