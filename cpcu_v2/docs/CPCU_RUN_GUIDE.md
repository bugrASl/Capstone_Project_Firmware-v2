# CPCU Run Guide — v3.4

**Author:** bugrASl
**Date:** April 2026
**Audience:** anyone on the team — from first-time new members to the original
authors. No prior CPCU knowledge assumed. Every step explains both *what* and
*why*.

This guide takes you from a fresh Raspberry Pi to a running prosthetic hand
controller. Every command is copy-pasteable. Every step has a verification
check and a troubleshooting note.

If you are completely new to this project, start by reading **Section 0**
("What is CPCU?") and the **Glossary at the end** — come back here for the
commands.

---

## Table of Contents

0.  [What is CPCU? (big picture)](#0-what-is-cpcu-big-picture)
1.  [Prerequisites](#1-prerequisites)
2.  [Fresh Pi setup (one-time)](#2-fresh-pi-setup-one-time)
3.  [Build](#3-build)
4.  [Test (before deploying anything)](#4-test-before-deploying-anything)
5.  [Install](#5-install)
6.  [Run](#6-run)
7.  [Monitor (live, while running)](#7-monitor-live-while-running)
8.  [Terminal theme](#8-terminal-theme)
9.  [Troubleshooting](#9-troubleshooting)
10. [Commands reference cheatsheet](#10-commands-reference-cheatsheet)
11. [Glossary](#11-glossary)

---

## 0. What is CPCU? (big picture)

**CPCU** = **Central Processing & Control Unit**. It is the box that sits on
the prosthetic forearm, receives muscle-signal packets from the BSAU, decides
which hand gesture the user is making, and drives the servo motors in the
fingers.

The whole system looks like this:

```
   User's muscles                        Prosthesis
   ──────────────                        ──────────
                                          ┌────────────────────────────────┐
   ┌──────┐  electrodes  ┌────┐           │   ┌────────┐   ┌─────────┐     │
   │ EMG  │─────────────▶│BSAU│ ─ 2.4GHz ─┼──▶│  CPCU  │──▶│ servos  │     │
   └──────┘              └────┘   radio   │   │ (Pi)   │   │ (PCA)   │     │
                                          │   └────────┘   └─────────┘     │
                                          └────────────────────────────────┘
```

-   **BSAU** (Biosignal Sampling & Acquisition Unit): STM32-based board worn
    on the upper arm. Reads 8 EMG electrodes with a 12-bit ADC, packs 2
    samples-per-channel into a 32-byte payload, and fires 1000 packets per
    second over an NRF24L01+ radio.
-   **CPCU** (this project): a Raspberry Pi 5 inside the forearm. It receives
    the packets, runs DSP + a machine-learning classifier, and commands 6
    servos to open/close fingers and rotate the wrist.

Inside the Pi, the work is split across cores:

```
   Core 0  ── cpcu_kernel     (supervisor, spawns & watchdogs everyone)
   Core 1  ── cpcu_dsp.py     (Python: band-pass filter + RMS + ML inference)
   Core 2  ── cpcu_ipc_bridge (optional; Python helper)
   Core 3  ── cpcu_io         (C, real-time: talks to NRF + PCA9685)

   Cores 1,2,3 are *isolated* from the Linux scheduler (isolcpus=1,2,3)
   so Linux never steals them for unrelated tasks.
```

All processes share data through **one shared-memory region** at
`/dev/shm/cpcu_ipc`:

```
   cpcu_io  ──(sensor ring buffer)──▶  cpcu_dsp.py
                                            │
                                            │ ML inference result
                                            ▼
   cpcu_io  ◀──(motor-command seqlock)──  cpcu_dsp.py
       │
       ▼
   PCA9685 ─▶ servos
```

You mostly interact with this system through **`cpcu_tui`**, an ncurses
dashboard that shows what every layer is doing in real time. See §7.3 for what
each page means.

---

## 1. Prerequisites

### 1.1 Hardware

```
[x] Raspberry Pi 5 (any RAM, 1 GB minimum)
[x] Active cooler (heatsink + fan — mandatory for 2.8 GHz OC)
[x] USB-C power supply (5V / 5A, 27W PD recommended)
[x] MicroSD card (16 GB minimum, Class 10)
[x] NRF24L01+ module (connected to SPI0 + GPIO 25 for CE)
[x] PCA9685 servo driver board (connected to I2C1 at 0x40)
[x] Servo motors (up to 6: MG995 Base/Upper/Last + SG90 Joint-1/Joint-2/Gripper)
[x] Separate servo power supply (6V / 3A minimum, common GND with Pi)
[x] BSAU board (NUCLEO-L432KC + NRF24L01+ transmitter)
[x] Ethernet or Wi-Fi (for SSH access)
```

> **Why a separate servo PSU?** Servos draw 1–2 A of transient current when
> they stall or change direction fast. That spike would crash the Pi if they
> shared the 5 V rail. Share only GND between the two supplies.

### 1.2 Software (on your laptop)

```
[x] SSH client (terminal on Linux/Mac, PuTTY on Windows)
[x] Git
[x] Trained ML model file: emg_rf_model.pkl
[x] Terminal emulator with 3024 Night theme (recommended: Kitty)
```

---

## 2. Fresh Pi setup (one-time)

```bash
git clone <your-repo-url> ~/prosthetic_hand
cd ~/prosthetic_hand/cpcu_v2
./setup_pi.sh                       # self-elevates via sudo (one prompt)
sudo reboot                         # the only sudo command you'll type today
```

**What `setup_pi.sh` does (v2.3, sudo-wrapped internally):**

1.  Self-elevates via `exec sudo "$0" "$@"` so you only see one
    password prompt regardless of how many privileged steps it runs.
2.  Installs the build toolchain (`gcc`, `cmake`, `libncurses-dev`,
    `tmux`) and Python deps (`numpy`, `scipy`, `scikit-learn`,
    `joblib`).
3.  Enables SPI and I²C in `/boot/firmware/config.txt`.
4.  Appends `isolcpus=1,2,3 nohz_full=1,2,3 rcu_nocbs=1,2,3` to
    `/boot/firmware/cmdline.txt` so the Linux scheduler leaves those
    cores alone.
5.  Creates `spi`, `i2c`, `gpio` groups (idempotent), drops a udev
    rule at `/etc/udev/rules.d/90-cpcu.rules` granting group access
    to `/dev/spidev*`, `/dev/i2c-*`, `/dev/gpiochip*`, and adds your
    real user (the one that ran the script — picked up via
    `$SUDO_USER`) to all three groups.
6.  Creates `/opt/cpcu/{bin,scripts,models,test}` and `/var/log/cpcu/`
    and **chowns them to your user** so subsequent `cmake --install`
    and log-tailing work without sudo.

Verify after reboot:

```bash
cat /sys/devices/system/cpu/isolated        # Expected: 1-3
ls /dev/spidev0.0                           # Expected: exists
ls /dev/i2c-1                               # Expected: exists
groups                                      # Expected: ... spi i2c gpio ...
vcgencmd measure_clock arm                  # Expected: ~2800000000
python3 -c "import numpy, scipy, joblib, sklearn; print('OK')"
```

**What can go wrong:**

-   `isolated` prints `""` (empty): `cmdline.txt` was not edited → re-run
    `./setup_pi.sh` and reboot. **All RT guarantees depend on this.**
-   `/dev/spidev0.0` missing: SPI still disabled →
    `sudo raspi-config` → Interface Options → SPI. *(One of the rare
    places a manual sudo is needed; raspi-config has no non-sudo
    equivalent.)*
-   `/dev/i2c-1` missing: I²C still disabled → same place, I²C.
-   `groups` doesn't show `spi`/`i2c`/`gpio`: log out and back in
    (group membership is set at login).

---

## 3. Build

```bash
cd ~/prosthetic_hand/cpcu_v2
cmake -S . -B build                 # configure (out-of-tree)
cmake --build build -j4             # compile all 7 binaries
```

Verify:

```bash
ls build/cpcu_io build/cpcu_kernel build/cpcu_tui \
   build/test_codec build/safety_testbench \
   build/pca_testbench build/signal_testbench
# All seven binaries should exist.
```

**What each binary does:**

| Binary             | Runs on     | Purpose                                          |
|--------------------|-------------|--------------------------------------------------|
| `cpcu_kernel`      | Core 0      | Supervisor. Spawns `cpcu_io` + `cpcu_dsp.py`. Watchdogs them. Owns `/var/lock/cpcu.lock`. |
| `cpcu_io`          | Core 3 (RT) | Reads NRF packets → sensor ring. Reads motor seqlock → writes PCA9685. |
| `cpcu_tui`         | any         | ncurses dashboard. 7 pages (v3.4 split build), read-only, safe to attach/detach live. |
| `test_codec`       | any         | Unit test: pack/unpack + ring-buffer round-trip. No hardware. **7 PASS.** |
| `safety_testbench` | any         | Automated safety-FSM harness. **33 PASS (v2.3).** |
| `pca_testbench`    | Pi (I2C)    | Interactive servo calibration. Standalone — does not talk to IPC. |
| `signal_testbench` | any         | End-to-end signal integrity TUI. Uses IPC when live, synthetic in `--demo`. |

Build a single target:

```bash
cmake --build build --target cpcu_tui           # Just the TUI
cmake --build build --target pca_testbench      # Just servo testbench
cmake --build build --target signal_testbench   # Just signal testbench
```

**What can go wrong:**

-   `Curses not found`: re-run `./setup_pi.sh` (it installs `libncurses-dev`).
-   `cmake: command not found`: re-run `./setup_pi.sh` (it installs `cmake`).
-   `add_executable references unknown source...`: you're invoking `cmake`
    from the wrong directory. The `CMakeLists.txt` lives at
    `cpcu_v2/CMakeLists.txt` and references `src/file.c`, `test/file.c`,
    `include/`. Use `cmake -S . -B build` from `cpcu_v2/`, not from a
    parent or `bsau_v2/`.

---

## 4. Test (before deploying anything)

Do not skip this step. Tests catch 90 % of problems before they reach
hardware.

### 4.1 Phase 1 — no hardware (runs on any machine)

```bash
./run_tests.sh 1
```

Three test groups run, all expected green:

| Group               | Source                          | Pass count |
|---------------------|---------------------------------|------------|
| TB-CODEC            | `test/test_codec.c`             | **7 PASS** |
| TB-SAF (FSM)        | `test/safety_testbench.c`       | **33 PASS (v2.3)** |
| TB-DSP (pipeline)   | `test/test_dsp_pipeline.py`     | **65 PASS (v2.3)** |

Or manually:

```bash
build/test_codec
build/safety_testbench
python3 test/test_dsp_pipeline.py
```

**What this proves:** codec packing/unpacking is bit-exact, ring buffer is
lock-free-safe under concurrent push/pop, the safety FSM transitions
correctly through every fault path *and* recovery path (radio,
battery, DSP stall, I²C, ring overflow with v2.3 delta-recovery,
thermal), and the Python DSP produces the expected feature outputs
on known analytical inputs. If any of this fails, **stop** —
nothing downstream will work.

### 4.2 Demo mode — preview the TUIs without hardware

```bash
build/cpcu_tui --demo
# Press 1/2/3/4/5/6/7 to see all pages.
#   1=Overview 2=Radio/IO 3=DSP/AI 4=Waves 5=Health 6=Dataset 7=Config
# Try w/[/] to cycle waveforms and F/B/G/O/I/R for fault injection.
# Page 6 (Dataset): LEFT/RIGHT to pick a label, s/SPACE to start/stop,
#                   t to toggle RAW/FILTERED, r to cancel a capture.
# Press q to quit.

build/signal_testbench --demo
# See synthetic sine waveforms + Goertzel analysis. Press q to quit.
```

These run on your laptop — no Pi, no shared memory, no peripherals. They are
useful for screenshots, docs, and learning the key bindings.

### 4.3 Phase 3 — Pi hardware (after install)

```bash
./run_tests.sh 3                        # Automated hardware checks
./run_tests.sh pca                      # Interactive servo calibration
```

See [CPCU_TEST_GUIDE.md](CPCU_TEST_GUIDE.md) for the full test matrix —
it explains what each test actually checks and how to interpret the output.

---

## 5. Install

```bash
cmake --install build               # No sudo: /opt/cpcu is owned by you
                                    # (setup_pi.sh chowned it).
./scripts/launch.sh grant-caps      # Self-elevates: setcap CAP_SYS_NICE +
                                    # CAP_IPC_LOCK on cpcu_io and cpcu_kernel
                                    # so they can take SCHED_FIFO and
                                    # mlockall without running as root.
```

This copies:

-   C binaries → `/opt/cpcu/bin/`
-   Python scripts → `/opt/cpcu/scripts/`
-   Launch script → `/opt/cpcu/scripts/launch.sh`

The systemd unit file is **not** copied at install time — it's
generated and installed in one step by
`./scripts/launch.sh install-service` (see §6.2 below).

Deploy the ML model **separately** — it is not in the repo:

```bash
scp emg_rf_model.pkl pi@<pi-ip>:/opt/cpcu/models/
```

The log directory (`/var/log/cpcu`) was created and chowned to your
user by `setup_pi.sh`.

---

## 6. Run

There are two ways to run CPCU: **interactive** (you launch it by hand
in a tmux session and watch everything live — best for hardware bring-up
and debugging) and **production** (managed by systemd, auto-starts at
boot — for actual deployment).

Either way, the only sudo prompts you'll see come from `systemctl`
itself when you ask it to start/stop a service; the script wrappers
have all the privilege handling done for you.

### 6.1 Interactive (one tmux session, recommended for development)

```bash
./scripts/launch.sh tui              # tmux: KERNEL window + TUI window
                                     #   Ctrl-b 0/1   switch windows
                                     #   Ctrl-b d     detach (keeps running)

./scripts/launch.sh attach           # later: re-attach to that session
./scripts/launch.sh stop             # later: shut everything down
```

Modes available (all sudo-free):

| Mode | What it spawns |
|---|---|
| `./scripts/launch.sh kernel`   | `cpcu_kernel --log` in foreground (no tmux) |
| `./scripts/launch.sh tui`      | tmux: KERNEL + TUI windows |
| `./scripts/launch.sh signal`   | tmux: KERNEL + SIGNAL (signal_testbench) |
| `./scripts/launch.sh collect`  | tmux: KERNEL + TUI, capture-workflow reminder banner |
| `./scripts/launch.sh pca`      | `pca_testbench` only (no kernel, no tmux) |
| `./scripts/launch.sh menu`     | Interactive picker (default if launched on a TTY) |
| `./scripts/launch.sh attach`   | Re-attach to the running cpcu tmux session |
| `./scripts/launch.sh stop`     | Kill the running cpcu tmux session and all children |

Inside tmux:

```
Ctrl-b 0          switch to KERNEL window
Ctrl-b 1          switch to TUI / SIGNAL / etc. window
Ctrl-b w          interactive window picker
Ctrl-b d          detach (everything keeps running in the background)
Ctrl-b ?          tmux help
```

### 6.2 Production (systemd, auto-start at boot)

Generate and install the unit file (one-shot, after a fresh install):

```bash
./scripts/launch.sh install-service     # self-elevates via sudo
                                        #   - writes /etc/systemd/system/cpcu.service
                                        #     (User=$REAL_USER, AmbientCapabilities=
                                        #      CAP_SYS_NICE CAP_IPC_LOCK)
                                        #   - applies setcap to cpcu_io + cpcu_kernel
                                        #   - systemctl daemon-reload + enable
```

The unit runs as **your** user, not root, and is granted the two
capabilities `cpcu_io` actually needs (`CAP_SYS_NICE` for `SCHED_FIFO`,
`CAP_IPC_LOCK` for `mlockall`). Operate it with the standard
systemctl commands:

```bash
sudo systemctl start cpcu       # Start now (sudo here is unavoidable —
                                # systemctl requires it to mutate state)
sudo systemctl stop cpcu        # Stop
sudo systemctl status cpcu      # Status
sudo systemctl restart cpcu     # Restart after edits

journalctl -u cpcu -f           # Live logs (no sudo needed for read-only)
```

Systemd invokes `/opt/cpcu/scripts/launch.sh kernel`, which calls
`./cpcu_kernel --log`. The `--log` flag is what turns on **per-module
CSV logging** to `/var/log/cpcu/log_*.csv`. See §7.5.

### 6.3 Stop

```bash
./scripts/launch.sh stop        # tmux session: kills KERNEL + TUI + everything
sudo systemctl stop cpcu        # Or systemd path
                                # Or Ctrl+C in the kernel window — traps SIGINT.
```

**What "clean shutdown" means** (shown in the logs): cpcu_io first
drives all servos to their neutral pulse width, waits 300 ms so they
physically settle, then calls `PCA_AllOff` (clears every PWM channel)
so the servos go limp instead of holding torque. Finally the NRF is
powered down, the I²C/SPI handles are released, and the log files are
flushed.

---

## 7. Monitor (live, while running)

### 7.1 tmux — recommended (single SSH connection)

```bash
ssh pi@<pi-ip>
tmux new -s cpcu

# Split into 4 panes:
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

# Navigate: Ctrl+b then arrow keys.
# Pane 0 (top-left):     journalctl -u cpcu -f
# Pane 1 (bottom-left):  watch -n 2 "vcgencmd measure_temp; ps -eo pid,comm,psr,pri | grep cpcu"
# Pane 2 (top-right):    /opt/cpcu/bin/cpcu_tui
# Pane 3 (bottom-right): /opt/cpcu/bin/pca_testbench
#
# Detach:    Ctrl+b d
# Reattach:  tmux attach -t cpcu
```

**Why tmux over 4 SSH connections:** one TCP stream, survives reconnection,
session state persists across Wi-Fi drops.

### 7.2 Hyprland workspaces — alternative (multiple SSH)

```bash
# Terminal 1 → workspace 1 (SUPER+SHIFT+1)
ssh -t pi@<pi-ip> '/opt/cpcu/bin/cpcu_tui'

# Terminal 2 → workspace 2 (SUPER+SHIFT+2)
ssh -t pi@<pi-ip> 'journalctl -u cpcu -f'

# Terminal 3 → workspace 3 (SUPER+SHIFT+3)
ssh -t pi@<pi-ip> 'watch -n 2 "vcgencmd measure_temp; echo ---; ps -eo pid,comm,psr,pri | grep cpcu; echo ---; cat /sys/devices/system/cpu/isolated"'
```

Switch workspaces: SUPER+1, SUPER+2, SUPER+3.

### 7.3 TUI pages (`cpcu_tui`) — what each one means

Press number keys `1`-`6` to switch pages while the TUI is running.

**Page 1 — Overview**
The "what is the whole system doing right now" page. Starts with a rolled-up
**HEALTH banner** showing green/yellow/red pills for `radio / io / ipc /
batt / dsp / fsm`, plus an overall verdict (`NOMINAL` / `WARNING` /
`DEGRADED`) at the top right. Below that: system state (INIT/RUNNING/SAFE),
radio link summary, all 8 EMG bar-graphs as % of full-scale ADC, all 6
servo target pulse-widths with min/neutral/max sliders, battery pack
voltage + raw ADC + divider-aware level (OK/LOW/CRIT/CHARG), a DSP
PIPELINE mini-summary, and the current ML gesture name + confidence.
**If something is off, start here — the HEALTH banner tells you which
layer is broken before you have to dig.**

**Page 2 — Radio / IO (aka cpcu_io internals)**
NRF24L01+ init status, channel (with GHz), address, SPI clock, IO
heartbeat age (proves the RT loop on core 3 is alive), SAFE entries
counter (how many times the FSM tripped since boot), battery pack
voltage. Packet statistics: total RX, rate (pkt/s), max poll (µs, worst
SPI read time), dropped packets (ring was full), seq gaps (missed
sequence numbers), loss rate over last 1 k packets, ring-fill **bar
graph**, retry count on the last packet. Last packet raw field dump
(`seq`/`flags`/`retry`/`loss`/`ts`/`rx_age`/`vbat`), decoded **BSAU
flags** banner (`CLIP` / `ELEC` / `OVRN` / `TX_SAT` / `CAL` / `FIRST`,
colour-coded by severity), and an 8-channel bar graph of the latest
payload. **Use this when packets are arriving but data looks wrong.**

**Page 3 — DSP / AI**
DSP-ready flag, number of DSP windows processed (400-sample FFTs) with
per-second rate, number of inferences with per-second rate, max DSP
latency (worst batch), ring fill, DSP stride (200 samples = 50 %
overlap), underflow count, DSP export rate (Hz — how often Python
publishes), motor cmd count + rate + age (ms since last DSP→IO write;
if this climbs above 100 ms, Python stalled). Active gesture banner
with confidence %, last inference time µs, and export-sequence counter
ticking. Per-class softmax confidence bars (10 classes, active one
highlighted magenta). Per-channel filtered RMS (bar = % of 0.5 V full-
scale, plus absolute value in V). Six smoothed servo pulse-width
sliders with the µs value displayed. **Use this when the radio is good
but the prosthesis doesn't move as expected.**

**Page 4 — Waveforms**
Live 8-channel rolling waveforms drawn with a **line-trace** renderer
(`'` `` ` `` `-` `.` `,` as fill glyphs, `/` and `\` as connectors)
giving 5× vertical sub-sampling per row. Top banner: BSAU flags + glyph
legend. Per channel: frequency (Hz from zero-crossing rate), Vpp, Vrms,
and a red `CLIP` indicator when the ADC hits rails (min ≤ 40 or
max ≥ 4055). UP/DOWN selects a channel, TAB switches between grid
view and zoomed single-channel detail. Single-channel detail adds DC
offset and a time-axis scale at the bottom. **Use this when a specific
electrode looks suspicious.** Page 4 peeks at the ring buffer **read-
only** — it does **not** consume entries, so it is safe to run
alongside `cpcu_dsp.py`.

**Page 5 — Health (traffic-light dashboard)**
Rolls up Page 1's banner into ten full-detail rows, one per subsystem.
Each row is `[  OK  ]` (green) / `[ WARN ]` (yellow) / `[FAULT ]` (red)
with a one-line explanation ("why"). Subsystems covered: Safety FSM,
Radio (nRF), IO loop, IPC ring, Pkt integrity, Battery, DSP pipeline,
ML export, BSAU sensor (from decoded packet flags), SAFE trips. Top
banner tallies `N OK | N WARN | N FAULT` and shows the overall verdict.
This is the page to put on a second monitor during hardware testing —
glance at it once a minute and you'll catch any regression immediately.

**Page 6 — Dataset (interactive CSV capture, v2.1)**
Capture 8-channel EMG recordings labelled by gesture, byte-compatible
with `bsau_dataset_collector.py` (RAW mode) or matching the voltage-
domain output of `cpcu_dsp.py` (FILTERED mode). Top banner shows the
current label, mode, capture state (IDLE / COLLECTING / SAVED /
CANCELLED), filename being written, and live counters: samples,
elapsed seconds, missed-due-to-overflow count, sequence gaps. Below
is a label picker (10 classes, current one highlighted) and the
RAW/FILTERED toggle indicator. The capture state machine drains the
ring on every TUI tick regardless of which page is rendered, so
flipping pages mid-capture does **not** lose samples. Output files
land in `./datasets/` with the naming convention
`<MODE>_<LABEL>_<NN>.csv` (auto-incremented). **Use this for ML
dataset collection — the DSP/AI team can stream-collect labelled
recordings from any laptop on the network via the TUI over SSH.**

**Page 7 — Config (spec sheet, static)**
Compile-time + hardware reference — what system you're actually looking
at. BSAU: STM32L432KC Cortex-M4F, 8 EMG channels through Soldered INA333
front-ends, 12-bit ADC, 2 kHz sample rate, 2 samples/packet at 1000
pkt/s, 2S Li-ion pack with 2:1 resistor divider. CPCU: Raspberry Pi
4B/5 on Raspberry Pi OS 64-bit, `isolcpus=1,2,3`, core 0 supervisor,
cores 1–2 Python DSP + ML, core 3 `cpcu_io` with `SCHED_FIFO` priority
80 + `mlockall`. Wireless: nRF24L01+ on channel 76 (2.476 GHz),
5-byte address `E7:E7:E7:E7:E7`, SPI 8 MHz, 32-byte fixed payload,
auto-ACK up to 3 retries. IPC: `/dev/shm/cpcu_ipc` = 66 240 B,
1024-entry SPSC lock-free ring (64 B/entry), seqlock-protected motor
command block, 256 B DSP export block. Motor: PCA9685 over I²C at
400 kHz driving 50 Hz PWM, 6 × SG90 servos (1.0–2.0 ms pulse),
2000 µs/s slew limit (1200 µs/s on the gripper), servos park at
neutral 1500 µs on SAFE entry. Safety thresholds: radio-silent
750 / 1500 ms (degraded / SAFE), Vbatt 3.0 V warn / 2.7 V critical.
DSP / ML: 400-sample FFT windows with 50 % stride overlap, features =
MAV + WL + ZC + SSC + RMS + spectral, RandomForest classifier with
10 classes. Build: TUI version string, compiler, ISO C standard, build
date/time. *(v3.4: moved from page 5 to the end of the tab order so
live data pages occupy the lowest keys.)*

---

**Keys available on every page**

| Key        | Action                                                          |
|------------|-----------------------------------------------------------------|
| `1`..`7`   | Switch page                                                     |
| `q` `Q`    | Quit                                                            |
| `UP`/`DN`  | Select channel (Page 4 only)                                    |
| `TAB`      | Toggle grid ↔ single-channel detail (Page 4 only)               |
| `← / →`    | Cycle gesture label (Page 6 only, when not capturing)           |
| `s` `SPACE`| Start / stop capture (Page 6 only)                              |
| `t` `T`    | Toggle RAW ↔ FILTERED capture (Page 6 only, when idle)          |
| `r`        | Cancel + delete in-progress capture (Page 6 only)               |

**Demo-mode-only hotkeys** (`cpcu_tui --demo`)

| Key       | Action                                                           |
|-----------|------------------------------------------------------------------|
| `w` `W`   | Cycle waveform: SINE → SQUARE → TRI → SAW → NOISE → EMG → ECG → CHIRP |
| `[`       | Halve frequency (floor 10 Hz)                                    |
| `]`       | Double frequency (ceiling 1000 Hz)                               |
| `F`       | Inject fault: radio freeze (triggers DEGRADED → SAFE after 2.25 s) |
| `B`       | Inject fault: low battery (triggers SAFE on `VBAT_CRITICAL`)     |
| `G`       | Inject fault: sequence-gap storm                                 |
| `O`       | Inject fault: ring overflow (auto-clears once burst ends, v2.3)  |
| `I`       | Inject fault: I²C error streak                                   |
| `R`       | Reset — clears all injected faults **and** zeros every counter (non-Dataset pages) |

When a fault is active, a red `[INJ:RADIO_FREEZE]` / `[INJ:BATT_LOW]` /
etc. banner appears at the bottom-right. When no fault is active, that
slot shows the current waveform tag (cyan): `[SINE 100Hz]`,
`[SQUARE 200Hz]`, `[CHIRP 400Hz]`, etc.

`R` is the master reset: not only does it clear `demo_fault_mask` and set
state back to RUNNING, it also zeros every accumulated IPC diag counter
(packets received, dropped, seq gaps, overflows, SAFE entries, DSP
batches, inferences, max latency, underflows) and resets ready flags.
After pressing `R` the TUI snaps back to a clean-boot look as if you had
just launched it.

In `--demo` mode a `[DEMO]` marker appears in the header bar and the ring
buffer is fed with 100 synthetic sensor packets per frame (so every page
has live-looking data even without hardware). The synthetic packets
exercise the full pipeline — codec, ring push/pop, atomics, seqlock
motor-command writes — so demo mode is a legitimate smoke test, not a
stub.

The layout is **fully dynamic**: resize the terminal and the TUI reflows
on the next frame. Minimum size is 60×18.

### 7.4 Standalone test tools

These run independently of the main CPCU pipeline:

```bash
# PCA9685 servo calibration (direct I2C, no kernel needed)
/opt/cpcu/bin/pca_testbench
#   Controls: arrows, PgUp/PgDn, m/M min/max, n/N neutral, 0 kill,
#             's' toggle slew smoother, 'A' write all channels,
#             'r' read back MODE1/MODE2/PRESCALE registers, 'q' quit.
#   Flags:    --min A,B,C,D,E,F  / --max A,B,C,D,E,F  / --smooth
#             (override default pulse limits and start with slew on)

# Signal integrity (needs kernel + cpcu_io + BSAU transmitting)
/opt/cpcu/bin/signal_testbench
#   Shows: 8-ch raw-ADC waveform + Goertzel freq analysis + Vpp + SNR.
#   Notice: this TUI plots the RAW ADC stream off the ring buffer — it
#           does NOT apply any DSP. For DSP-output waveforms use cpcu_tui
#           Page 3/4.
#   Controls: UP/DOWN select channel, TAB toggle all-channel grid vs
#             single-channel detail, q quit.

# Signal integrity with synthetic data (no hardware needed)
/opt/cpcu/bin/signal_testbench --demo
#   Same TUI, but fed with synthetic waveforms so you can see what each
#   waveform type looks like through the full codec + ring + render chain
#   without touching the radio.
#   Demo-only hotkeys (same as cpcu_tui --demo):
#     w / W   cycle SINE → SQUARE → TRI → SAW → NOISE → EMG → ECG → CHIRP
#     [       halve frequency (floor 10 Hz)
#     ]       double frequency (ceiling 1000 Hz)
#   Header bar shows `[DEMO <WAVE> <FREQ>Hz]` so you always know what
#   the signal generator is producing.
#   Safety testbench (automated, Phase 1):
/opt/cpcu/bin/safety_testbench
#   Exercises the safety FSM without hardware: radio-loss timeout,
#   low-battery trip, sequence-gap storm, ring overflow, I2C streak.
#   Runs 7 test groups / 33 checks (v2.3), prints PASS/FAIL, exits non-zero
#   if anything fails. Also registered in CTest (`ctest -R safety_fsm`).
#   For the interactive version, use `cpcu_tui --demo` and press F/B/G/
#   O/I to inject faults, R to reset.
```

### 7.5 Logs — journal and per-module CSV

**Systemd journal:**

```bash
journalctl -u cpcu -f                       # All logs
journalctl -u cpcu -f | grep "\[IO\]"       # cpcu_io only
journalctl -u cpcu -f | grep "\[DSP\]"      # Python DSP only
journalctl -u cpcu -f | grep "\[KERN\]"     # Supervisor only
journalctl -u cpcu -f | grep "\[NRF\]"      # Radio events
journalctl -u cpcu -f | grep "\[WDG\]"      # Watchdog events
```

**Per-module CSV files** (enabled by the `--log` flag, which `launch.sh`
already passes):

```bash
ls /var/log/cpcu/
#   log_kern.csv    log_wdg.csv    log_io.csv    log_nrf.csv    log_pca.csv
#
# Format: one row per log line, CSV.
#   timestamp_s,timestamp_us,proc,level,"message"
#
# Example — see every NRF recovery event:
awk -F, '$4=="WARN"||$4=="ERROR"' /var/log/cpcu/log_nrf.csv | column -s, -t
#
# Example — plot log_io.csv in Python:
python3 -c "import pandas as pd;
d=pd.read_csv('/var/log/cpcu/log_io.csv');
print(d.groupby('level').size())"
```

CSVs are `fflush`-ed after every write, so tailing them while the system runs
is safe: `tail -f /var/log/cpcu/log_io.csv`.

### 7.6 Process health checks

```bash
ps -eo pid,comm,psr,pri | grep cpcu         # Core assignment
chrt -p $(pidof cpcu_io)                     # RT priority (expect SCHED_FIFO 90)
grep VmLck /proc/$(pidof cpcu_io)/status     # Memory lock (expect non-zero)
ls -la /dev/shm/cpcu_ipc                     # Shared memory (expect 66240 bytes)
```

Expected output shape:

```
cpcu_kernel     0  20     <- Core 0, normal prio
cpcu_io         3   0     <- Core 3, RT (chrt shows policy/prio)
python3         1  20     <- Core 1, cpcu_dsp.py
```

**What can go wrong:**

-   `cpcu_io` on core 0 instead of core 3 → `isolcpus=1,2,3` did not apply
    (see §2).
-   `chrt` shows `SCHED_OTHER` → cpcu_kernel didn't run as root, or
    `CAP_SYS_NICE` is missing.

---

## 8. Terminal theme

For consistent TUI appearance across the team, set your terminal to the
**3024 Night** theme. Our TUIs deliberately use only standard ANSI colors
(GREEN=good, RED=bad, CYAN=info, MAGENTA=highlights) so that the terminal's
theme controls the actual rendered palette. We do not hard-code RGB anywhere.

For Kitty:

```bash
kitty +kitten themes 3024 Night
```

No code changes needed — ncurses reads the terminal's palette automatically.

---

## 9. Troubleshooting

### "NRF init failed"

Check SPI wiring (GPIO 8/9/10/11 + CE on GPIO 25), verify 3.3 V power (**not
5 V — NRF is 3.3V-only and 5V will fry it**), check `ls /dev/spidev0.0`.

Also check `/var/log/cpcu/log_nrf.csv` — v2.2 reads back the `CONFIG`,
`EN_RXADDR`, and `RF_CH` registers at init time. If those values don't match
what was written (0x0F, 0x03, 76), the SPI link has noise or a wiring fault.

### "PCA init failed"

Check I²C wiring (SDA GPIO 2, SCL GPIO 3), run `i2cdetect -y 1` and
confirm address `0x40`, verify 3.3 V power and that the board has its 4.7 kΩ
pull-ups populated.

### "Model not found"

`cp emg_rf_model.pkl /opt/cpcu/models/` — the model is never in git.

### "IPC magic mismatch"

Start `cpcu_kernel` first — it creates the shared-memory region. TUIs and
testbenches refuse to attach to stale shared memory from an earlier version,
which is what this error means. Fix: `rm /dev/shm/cpcu_ipc` then restart.

### Servos don't move

Walk down the pipeline **in order** and stop at the first thing that looks
wrong:

1.  Page 1: is `state == RUNNING`? If `SAFE` or `FAULT`, read the last FAULT
    event in `log_io.csv`.
2.  Page 3: is DSP inferences/sec ticking up? If flat at 0, `cpcu_dsp.py`
    isn't running (see next).
3.  Page 3: is the gesture ≠ `REST`? If always REST, the ML model may be
    mis-loaded or the EMG signal is too weak.
4.  `i2cdetect -y 1` shows 0x40? If not, PCA9685 is unpowered or not
    wired.
5.  Is the **servo power supply** actually on? Servos draw from the 6V rail,
    not from the Pi.

### Python DSP keeps dying

Run it manually to see the error directly instead of through journald:

```bash
cd /opt/cpcu/scripts && python3 cpcu_dsp.py
```

Most common causes: `sklearn` version mismatch (the `.pkl` was trained on a
different version), missing `numpy`/`scipy`, or the model file is missing /
wrong path.

### Everything seems fine but Page 4 shows flat lines

The ring buffer is getting samples, but they are all the same value. Almost
always: BSAU ADC input is at 0 V or at full rail (3.3 V). Check electrode
contact and the BSAU's amplifier stage.

---

## 10. Commands reference cheatsheet

```bash
# Build
cd ~/cpcu_v2/build && cmake .. && make -j4
cmake --install build

# Systemd
sudo systemctl start|stop|restart|status|enable|disable cpcu
journalctl -u cpcu -f

# Tests
./run_tests.sh 1 2 3           # All automated
./run_tests.sh pca             # Servo calibration TUI
./run_tests.sh signal          # Signal integrity TUI (live)
./run_tests.sh signal-demo     # Signal integrity TUI (synthetic)
./cpcu_tui --demo              # Full TUI demo (no hardware)

# Hardware
i2cdetect -y 1
vcgencmd measure_temp
vcgencmd measure_clock arm
cat /sys/devices/system/cpu/isolated

# Per-module logs (v2.2)
ls /var/log/cpcu/
tail -f /var/log/cpcu/log_io.csv
awk -F, '$4=="ERROR"' /var/log/cpcu/log_*.csv
```

---

## 11. Glossary

Plain-language definitions of every term in this guide (and in the code).

-   **ADC** — Analog-to-Digital Converter. The BSAU's STM32 chip has one; it
    turns the electrode voltage into a 12-bit number (0–4095) many times a
    second.
-   **BSAU** — Biosignal Sampling & Acquisition Unit. The arm-worn board that
    reads the muscle electrodes and transmits packets over the NRF radio.
-   **BSAU flags (CLIP / ELEC / OVRN / TX_SAT / CAL / FIRST)** — Six status
    bits the BSAU sets in each packet header to report health of the sample
    itself. `CLIP` = at least one channel's ADC hit the rail (electrode
    voltage outside the input range → amp is saturating). `ELEC` = electrode
    off (sudden open-circuit detection, impedance spike). `OVRN` = ADC
    overrun (BSAU couldn't service the DMA buffer in time). `TX_SAT` =
    nRF TX FIFO saturated (prior packet didn't leave the radio — link is
    flapping). `CAL` = calibration frame (first few packets after boot, not
    user data). `FIRST` = session-first packet (BSAU rebooted). Decoded and
    colour-coded on `cpcu_tui` Pages 2 and 4 and on Page 5's Health
    dashboard.
-   **Core isolation / isolcpus** — A Linux kernel command-line option that
    tells Linux "don't put general-purpose tasks on these cores." We set
    `isolcpus=1,2,3` so cores 1–3 are reserved for our code and nothing else
    gets scheduled there.
-   **CPCU** — Central Processing & Control Unit (this project). The Pi-side
    box that receives packets, does DSP + ML, and drives the servos.
-   **Codec** — Here it means the **packing/unpacking** code
    (`wireless_packet.c`) that turns a 32-byte radio payload into an
    8-channel sample struct and back. Not to be confused with audio codecs.
-   **CRC** — Cyclic Redundancy Check. A short checksum the NRF24L01+
    auto-computes over each packet so corrupted packets can be dropped.
-   **DMA** — Direct Memory Access. Hardware moving data without CPU
    involvement. Used on the BSAU to shuffle ADC samples into RAM.
-   **DSP** — Digital Signal Processing. Our Python `cpcu_dsp.py` does three
    things: band-pass filter 20–450 Hz, sliding RMS window, and ML inference.
-   **EMG** — Electromyography. Measuring the tiny voltages produced by
    muscle contractions. Typically 0.05–2 mV — tiny, so the BSAU amplifies
    before sampling.
-   **Goertzel** — An algorithm for finding the magnitude at **one**
    frequency in a signal. Cheaper than a full FFT when you only want a few
    bins. `signal_testbench` uses it to detect the function-generator tone.
-   **I²C / I2C** — "Inter-Integrated Circuit", a 2-wire bus (SDA + SCL). We
    use it at 400 kHz to talk to the PCA9685 at address `0x40`.
-   **IPC** — Inter-Process Communication. Our IPC is a POSIX shared-memory
    region at `/dev/shm/cpcu_ipc` that contains a sensor ring buffer + motor
    seqlock.
-   **MCU** — Microcontroller Unit. The BSAU's STM32 is an MCU.
-   **NRF / NRF24L01+** — Nordic Semiconductor's 2.4 GHz radio. We run it at
    channel 76, 2 Mbps, 32-byte fixed payload, 5-byte address, auto-ACK off
    to save latency.
-   **PCA9685** — NXP's 16-channel 12-bit PWM driver IC. We use 6 channels
    for servos, driven over I²C at 50 Hz PWM (20 ms period).
-   **PSU** — Power Supply Unit. Servos need their own 6 V PSU.
-   **PWM** — Pulse-Width Modulation. How servos are commanded: the pulse
    width (typically 1000–2000 µs) sets the servo angle.
-   **RT / SCHED_FIFO** — Linux's real-time scheduling policy. A
    `SCHED_FIFO`-priority-90 task preempts almost everything else.
    `cpcu_io` runs under this.
-   **Seqlock** — A lock-free synchronization primitive: writer bumps a
    sequence counter before and after writing; reader retries if the two
    observed counters differ. Used for the single motor-command struct
    (tiny, frequently updated).
-   **Slew / slew rate** — How fast a value is allowed to change. Our
    `cpcu_smooth` limits servos to ~2000 µs/s by default (the gripper
    overrides to 1200 µs/s for a gentler grab).
-   **SPI** — Serial Peripheral Interface, a 4-wire bus (MISO, MOSI, SCK,
    CS). We talk to the NRF at 8 MHz, mode 0, MSB-first.
-   **SPSC / ring buffer** — Single-Producer Single-Consumer lock-free
    queue. Our sensor ring is SPSC: `cpcu_io` is the only producer, `cpcu_
    dsp.py` is the only consumer.
-   **Systemd** — Linux's service manager. Our `cpcu.service` unit ensures
    CPCU starts at boot and restarts on crash.
-   **Vpp** — Peak-to-peak voltage. `max - min` of a waveform. Shown per
    channel in `signal_testbench` — handy for checking that a function
    generator is putting out what you think it is.
-   **Vrms** — Root-mean-square voltage: `sqrt( mean((x − mean(x))²) )`,
    computed over the rolling sample buffer after removing the DC offset.
    The "energy" of the signal in a single number. Shown per channel on
    `cpcu_tui` Page 4.
-   **Zero-crossing rate (ZCR) / ZC** — Number of times the signal crosses
    its mean per unit time. Gives a quick frequency estimate for periodic
    signals without a full FFT. Page 4 uses ZCR (with 1 % hysteresis) to
    label each mini-plot with a `Hz` reading. Zero for pure noise or DC.
-   **Health traffic-light** — Page 5 of `cpcu_tui`. Ten subsystem rows,
    each `[OK]` / `[WARN]` / `[FAULT]` with a one-line "why" explanation.
    Overall verdict at the top: `NOMINAL` / `WARNING` / `DEGRADED`. A
    compact six-pill version of the same banner is rolled up to the top
    of Page 1 Overview.
-   **SAFE entries** — Counter in `IPC_Diagnostics` incremented every time
    the safety FSM transitions into the `SAFE` state (radio loss, low
    battery, ring overflow, I²C streak, whatever tripped it). A healthy
    long-running session reads `0`; anything non-zero means the system
    recovered at least once and you should look at the logs to see why.
-   **IO heartbeat** — Monotonic-clock timestamp `cpcu_io` writes to
    `ipc->ctrl->io_heartbeat_us` at the top of every main-loop iteration.
    The TUI computes `now − heartbeat` and displays it as "age in ms." If
    this climbs above 20 ms the RT loop is jittery; above 100 ms it's
    stalled (bad priority, bus contention, or `cpcu_io` crashed).
-   **Line-trace renderer** — The Page-4 waveform drawing algorithm.
    Instead of stacking block characters (the old 8-level Unicode
    approach), each column gets a single glyph chosen from `'` `` ` ``
    `-` `.` `,` by sub-row position (5 sub-cells per row), and vertical
    gaps between adjacent samples are filled with `/` / `\` connectors.
    Gives 5× more effective vertical resolution than the row-count
    suggests and renders correctly on terminals that don't support
    Unicode block glyphs.
-   **Demo waveform selector** — In `--demo` mode, eight built-in
    signal generators (sine, square, triangle, sawtooth, noise, EMG
    burst, ECG, chirp) can be selected live with `w` / `W` and
    frequency scaled with `[` / `]`. Shared header `demo_signals.h`
    provides the `demo_gen(wave, t, freq, phase_off) → voltage`
    dispatch used by both `cpcu_tui` and `signal_testbench`.

---
