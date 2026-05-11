# InfiniTech Prosthetic Hand — User Guide

**You only need to know one command:** `./launch.sh`.

Everything you do with this system — setting it up, building it, testing
it, calibrating it, running it day-to-day, stopping it, recovering when
something goes wrong — happens through `./launch.sh <command>`. You do
not need to know Linux, tmux, Raspberry Pi internals, build systems, or
where files live on disk. The system handles those details for you.

If you ever forget what's available, type:

```
./launch.sh help
```

That's the most important command in this guide. The rest of this
document walks you through the order of operations, from an unboxed
set of boards to a live, calibrated prosthetic hand.

---

## Table of contents

1. [What this system is](#1-what-this-system-is)
2. [What you need on the bench](#2-what-you-need-on-the-bench)
3. [Hardware: wiring the boards](#3-hardware-wiring-the-boards)
4. [First-time setup (one command)](#4-first-time-setup-one-command)
5. [Build the software (one command)](#5-build-the-software-one-command)
6. [Verify everything is ready](#6-verify-everything-is-ready)
7. [Software-only tests](#7-software-only-tests)
8. [Flashing the BSAU board (laptop side)](#8-flashing-the-bsau-board-laptop-side)
9. [Hardware tests](#9-hardware-tests)
10. [Live integration test](#10-live-integration-test)
11. [Calibration](#11-calibration)
12. [Running the system day-to-day](#12-running-the-system-day-to-day)
13. [Auto-start at boot](#13-auto-start-at-boot)
14. [Stopping, recovering, troubleshooting](#14-stopping-recovering-troubleshooting)
15. [Quick reference card](#15-quick-reference-card)

---

## 1. What this system is

The prosthetic hand has two electronic boards:

- **BSAU** ("Bio-Signal Acquisition Unit") — a small STM32 board that
  reads electrical signals from your forearm muscles and transmits
  them wirelessly. It clips onto the forearm cuff.
- **CPCU** ("Central Processing and Control Unit") — a Raspberry Pi 5
  inside the prosthetic. It receives the wireless signals, runs them
  through a machine-learning classifier, decides which gesture you're
  making, and drives the six servo motors that move the fingers and
  joints.

You'll do most of your work through the CPCU. The BSAU is mostly
hands-off: you flash its firmware once and forget about it. **All
`./launch.sh` commands run on the CPCU**, not the BSAU.

System flow:

```
   Your forearm muscles
          │
          ▼
   [BSAU] sample 8 EMG channels at 2 kHz
          │       (wireless link, 1000 packets/second)
          ▼
   [CPCU] filter → classify → control 6 servos
          │
          ▼
     Prosthetic hand moves
```

The CPCU has a built-in **safety system** that watches for problems:
if it loses the wireless signal, the battery gets too low, or the
servos overheat, it stops the motors and parks them safely. You'll
see the safety system in action during testing.

---

## 2. What you need on the bench

Hardware (you should already have most of this from the prior phases
of the project):

- **NUCLEO-L432KC** development board — this is the BSAU.
- **Raspberry Pi 5** with active cooler and 32 GB+ microSD card —
  this is the CPCU.
- **Two NRF24L01+ modules** (one per board) — wireless transceivers.
- **PCA9685** servo driver breakout board.
- **Six servos** wired into the prosthetic arm assembly (3 MG995, 3 SG90).
- **USB-C 5 V / 5 A power supply** for the Pi.
- **Separate 6 V, 3 A or higher power supply** for the servo motors.
  (The servo supply must be **separate** from the Pi supply — running
  servos off the Pi's 5 V rail will brown out the Pi when motors stall.)
- A laptop with internet, a Micro-USB cable for the Nucleo, USB-C cable
  for the Pi, multimeter, basic jumper wires.

Software (one-time installs on your laptop):

- **STM32CubeIDE** — for flashing the BSAU. Free download from
  [st.com/en/development-tools/stm32cubeide.html](https://www.st.com/en/development-tools/stm32cubeide.html).
- **A serial terminal** at 921600 baud — built-in `screen` command on
  macOS/Linux works, on Windows install PuTTY.
- An **SSH client** to connect to the Pi remotely (built-in on
  macOS/Linux as the `ssh` command, on Windows use PuTTY or
  Windows Terminal).

You won't need anything else on the laptop. Everything else lives on
the Pi and is handled by `./launch.sh`.

---

## 3. Hardware: wiring the boards

Read this section once, do the wiring, never touch it again. The
wiring is unchanged from earlier project phases, but `./launch.sh`
won't help you with physical wiring — that's the one thing software
can't do for you.

### 3.1 Safety rules (read before plugging anything in)

1. **The NRF24L01+ modules run on 3.3 V, NOT 5 V.** Connecting them
   to 5 V destroys them instantly. The NRF module has "VCC" printed
   next to one of its corner pins; that pin must connect to a **3.3 V**
   pin on the Nucleo or Pi.
2. **Never run the servo motors off the Raspberry Pi's USB power.**
   Servos draw bursts of 1-2 amps during motion. The Pi can't supply
   that and will reboot. Always use the separate 6 V servo supply.
3. **The two power supplies share ground only.** The Pi's 5 V rail
   and the servos' 6 V rail must NOT be connected to each other,
   but their ground (negative) terminals must be tied together.
   Without the common ground the servos won't move.
4. **Power on order:** plug in USB cables first, then turn on the
   servo supply last. Powering everything simultaneously can cause
   the servos' inrush current to glitch the Pi while it's booting.

### 3.2 BSAU side wiring (the Nucleo)

The Nucleo has pins labeled with both Arduino-style names (`A0`,
`D5`, etc.) and STM32 names (`PA0`, `PB6`, etc.). **Always use the
STM32 names** in the table below — that's what the firmware expects.

#### NRF24L01+ wireless module → Nucleo

| NRF pin | Connect to Nucleo pin | Why |
|---|---|---|
| VCC | 3V3 | **Power. MUST be 3.3 V, not 5 V.** |
| GND | GND | Ground |
| CSN | PB7 | Chip select |
| SCK | PB3 | Clock |
| MOSI | PB5 | Data out |
| MISO | PB4 | Data in |
| CE | PA8 | Chip enable |
| IRQ | PB6 | Interrupt (connect even though firmware doesn't use it) |

**Crucial:** solder a 10 µF and 100 nF capacitor in parallel between
the NRF's VCC and GND pins, as close to the module as possible.
Without these, you'll get phantom packet loss that's actually
power-supply brownouts.

#### EMG inputs → Nucleo (8 channels)

| EMG channel | Connect to Nucleo pin |
|---|---|
| ch 0 | PA0 |
| ch 1 | PA1 |
| ch 2 | PA2 |
| ch 3 | PA3 |
| ch 4 | PA4 |
| ch 5 | PA5 |
| ch 6 | PA6 |
| ch 7 | PA7 |
| Battery monitor (post-divider) | PB0 |

Your EMG amplifier output should sit at about **1.65 V** when the
muscle is at rest. Muscle activity moves the signal up and down from
that midpoint.

### 3.3 CPCU side wiring (the Raspberry Pi 5)

The Pi has a 40-pin header along one edge. Pin 1 is the corner
closest to the microSD card slot.

#### NRF24L01+ → Pi

| NRF pin | Pi physical pin | Why |
|---|---|---|
| VCC | 1 or 17 (3.3 V) | **Power. MUST be 3.3 V, not 5 V.** |
| GND | 6, 9, 14, 20, 25, 30, 34, or 39 | Any GND pin works |
| CSN | 24 | Chip select |
| SCK | 23 | Clock |
| MOSI | 19 | Data out |
| MISO | 21 | Data in |
| CE | 22 | Chip enable |
| IRQ | 18 | Interrupt |

Same 10 µF + 100 nF capacitors as the BSAU side.

#### PCA9685 (servo driver) → Pi

The PCA9685 has two connector areas: a 6-pin row for control signals
(connects to the Pi) and a 2-pin terminal block for servo power
(connects to the 6 V supply).

| PCA9685 pin | Pi physical pin | Why |
|---|---|---|
| VCC | 1 (3.3 V — same rail as the NRF is fine) | Logic power |
| GND | 14 (or any GND) | Ground |
| SDA | 3 | Data line |
| SCL | 5 | Clock line |
| OE | leave unconnected | Output enable (auto-on) |

The PCA9685's I²C address is `0x40` by default. Most breakouts
already have the I²C pull-up resistors populated. If you bought a
generic clone and find it's not detected later, check the back of
the board for missing 4.7 kΩ pull-ups.

#### PCA9685 servo power side

| PCA terminal | Connect to |
|---|---|
| V+ | Positive (+) of the 6 V servo supply |
| GND | **Both** the 6 V supply's negative AND the Pi's GND |

The shared ground between the Pi and the servo supply is mandatory.
Without it, the servo PWM signals are referenced to nothing on the
servo side, and the motors will twitch randomly or not move at all.

#### Servo cables → PCA9685 outputs

Each servo has a 3-wire cable. Plug each one onto a numbered output
channel on the PCA9685. Match servos to channels per this table —
the firmware expects this exact mapping:

| Servo | Function | PCA9685 channel | Type |
|---|---|---|---|
| S0 | Base rotation | **0** | MG995 |
| S1 | Upper arm | **1** | MG995 |
| S2 | Last joint | **11** | MG995 |
| S3 | Joint 1 (finger) | **8** | SG90 |
| S4 | Joint 2 (finger) | **5** | SG90 |
| S5 | Gripper | **4** | SG90 |

The channel numbers are non-sequential — that's intentional, it
matches how the cables route through the prosthetic. If you absolutely
must rewire to different channels, the firmware has one place that
defines the mapping; tell us and we'll update the constant.

### 3.4 Multimeter checks before powering up

Before plugging in any USB cable or turning on the servo supply,
check the wiring with a multimeter (5 minutes now saves hours later):

**Continuity (the meter should beep):**

```
NRF VCC pin  ↔ Nucleo 3V3 silk     (BSAU side)
NRF GND pin  ↔ Nucleo GND          (BSAU side)
NRF VCC pin  ↔ Pi pin 1 or 17      (CPCU side)
NRF GND pin  ↔ any Pi GND pin      (CPCU side)
PCA SDA      ↔ Pi pin 3
PCA SCL      ↔ Pi pin 5
PCA V+       ↔ servo supply (+)
PCA GND      ↔ servo supply (–)
PCA GND      ↔ Pi GND               ← THIS IS THE COMMON GROUND, MUST BEEP
```

**Resistance (set the meter to ohms, with everything UNPOWERED):**

```
NRF VCC ↔ NRF GND   →  > 1 MΩ      (a few ohms = short, DO NOT POWER)
PCA V+  ↔ PCA GND   →  > 1 MΩ
```

**Servo supply voltage:** turn on the 6 V supply *with no load
connected*, measure across its terminals with a multimeter, confirm
it reads 6.0 V (or anywhere in the 6.0-6.5 V range — most servos
tolerate up to ~7 V). Turn it off, then plug into the PCA9685.

### 3.5 First power-on (order matters)

Once wiring is verified:

1. Plug the **Nucleo's USB cable** into your laptop. The red power
   LED on the Nucleo lights immediately.
2. Plug the **Pi's USB-C cable** into its 5 V supply. Pi's red LED is
   solid, the green LED blinks during boot, then settles after about
   30 seconds.
3. **After the Pi has finished booting**, turn on the 6 V servo
   supply. (Doing this before the Pi finishes booting can cause
   spurious servo motion.)

You're now ready for software setup.

---

## 4. First-time setup (one command)

You only run this once per Raspberry Pi.

### 4.1 Get on the Pi

From your laptop, open a terminal and connect to the Pi:

```
ssh <your-username>@<pi-address>.local
```

Replace `<your-username>` with your Pi's user account name (you set
this when flashing Raspberry Pi OS to the SD card) and `<pi-address>`
with whatever hostname or IP you assigned.

If you don't know the Pi's address, check your home router's admin
page for connected devices, or run `arp -a` from your laptop after
the Pi has booted.

### 4.2 Get the project onto the Pi

The first time only, install `git` and clone the project:

```
sudo apt install -y git
git clone <project-repo-url> ~/prosthetic_hand
cd ~/prosthetic_hand/cpcu_v2
```

Replace `<project-repo-url>` with the actual repository URL your
team uses. After this command finishes, you'll have a folder called
`cpcu_v2` and you'll be inside it.

This is the only directory you'll ever need to work from. Every
`./launch.sh` command is run from here.

### 4.3 Run setup

```
./launch.sh setup
```

That's the entire setup process. The command will:

- Tell you exactly what it's about to do.
- Ask for your password once (one prompt, even though it does many
  privileged things internally).
- Install the build tools and Python libraries it needs.
- Configure the Pi for real-time use (reserves 3 of the 4 CPU cores
  for the prosthetic-hand control loop).
- Enable SPI and I²C (the protocols used for the wireless and servo
  drivers).
- Add your user account to the groups that can talk to those
  protocols.
- Create a system folder at `/opt/cpcu` owned by your user.

When it finishes, one of two things happens:

**Case A — no reboot needed.** You see:

```
[LAUNCH] OK: Setup complete. No reboot needed.
[LAUNCH] Next step: ./launch.sh build
```

You can move directly to section 5.

**Case B — reboot required.** You see:

```
[LAUNCH] WARN: ════════════════════════════════════════════════════════════
[LAUNCH] WARN:   REBOOT REQUIRED
[LAUNCH] WARN:
[LAUNCH] WARN:   Setup modified /boot/firmware/config.txt and/or
[LAUNCH] WARN:   cmdline.txt. The Pi must reboot for those to take effect.
[LAUNCH] WARN: ════════════════════════════════════════════════════════════

  Reboot now? [y/N]:
```

Type `y` and Enter. The Pi will reboot in 3 seconds. After it's back
up (about a minute), reconnect:

```
ssh <your-username>@<pi-address>.local
cd ~/prosthetic_hand/cpcu_v2
```

### 4.4 You will not see this command again

`./launch.sh setup` is a one-time thing. You don't run it on a daily
basis. You only run it again if you wipe the SD card and start fresh
on a new Pi.

If you ever wonder whether setup completed correctly, run:

```
./launch.sh check
```

This will report what's missing or wrong (more on this command in
section 6).

---

## 5. Build the software (one command)

```
./launch.sh build
```

That's it. This compiles all the program code, installs it to its
operating location, and applies the special permissions the system
needs to run reliably (real-time scheduling and memory locking).

You'll see one password prompt near the end (for the permission
step). Total time is about 30-60 seconds on a Pi 5.

When it finishes:

```
[LAUNCH] OK: Build complete. Run './launch.sh check' to verify, or './launch.sh tui' to start.
```

You'll re-run `./launch.sh build` whenever:

- You pull new project source code (`git pull`).
- You change a safety threshold via `./launch.sh configure` (the
  system will prompt you to rebuild — see section 11.3).

You **don't** need to re-run it for:

- Calibration changes (servo limits, gestures) — those are runtime
  changes, no rebuild needed.
- Daily startup — once built, you just `./launch.sh tui` to run.

---

## 6. Verify everything is ready

Before you launch the live system for the first time — and any time
something seems wrong — run:

```
./launch.sh check
```

This prints a one-line status for every requirement and tells you
whether the system can launch. Example output if everything is fine:

```
[LAUNCH] Standalone pre-flight...
[LAUNCH] Isolated cores: 1-3
[LAUNCH] OK: All checks passed — system is ready to launch
```

Example output with one warning (model not deployed yet — that's
fine for now):

```
[LAUNCH] Standalone pre-flight...
[LAUNCH] WARN: ML model not in /opt/cpcu/models — DSP will run feature-only
[LAUNCH] WARN: 1 warning(s) — system can start with degraded behavior
```

Example output with a fatal problem:

```
[LAUNCH] ERROR: cpcu_kernel missing CAP_SYS_NICE — run './launch.sh grant-caps'
[LAUNCH] ERROR: 1 fatal issue(s), 0 warning(s) — system will NOT start
```

The error messages always tell you the exact `./launch.sh` command
to fix the problem. Read them and run what they suggest.

### About the ML model

The classifier is a pre-trained file (`hmi_svm_model_200hz.joblib`
plus a companion scaler file) that converts your muscle signals
into gestures. **It is NOT included with the source code** — your
team trains it separately.

If you don't have the model file yet, that's perfectly fine: the
system runs in **feature-only mode**. The kernel comes up, the
servos are commanded to stay neutral, the dashboards work, the
safety system works — you just don't get gesture classification.
This means you can verify everything in this guide except gesture
recognition (which is one specific test, TS-04) before training a
model.

When you do have the model, copy it to the Pi:

```
# From your laptop:
scp hmi_svm_model_200hz.joblib <user>@<pi>.local:/opt/cpcu/models/
scp hmi_scaler_200hz.joblib    <user>@<pi>.local:/opt/cpcu/models/
```

Or use `./launch.sh stop` and a fresh `./launch.sh tui` to pick up
new model files.

---

## 7. Software-only tests

Before testing anything that involves hardware, verify the software
is internally consistent. These tests don't need any wires, the
wireless link, or the servos — they just check that the math, the
filters, the safety logic, and the data structures all work.

```
./launch.sh test-sw
```

This runs **233 tests** across 7 areas:

- 7 wireless packet codec tests
- 38 safety state-machine tests
- 28 servo motion smoother tests
- 86 signal-processing pipeline tests
- 43 configuration loader tests
- 24 in-system editor tests
- 7 JSON serializer tests

You should see, at the end:

```
RESULTS: 233 PASS, 0 FAIL
```

If any test fails, **stop here**. Do not move to hardware tests
until the software passes — a software failure means none of the
hardware test results will be trustworthy.

---

## 8. Flashing the BSAU board (laptop side)

This is the one thing `./launch.sh` cannot help with — flashing the
BSAU runs on your laptop, not the Pi. You'll do this once, then
forget about it.

### 8.1 Open the project in STM32CubeIDE

On your laptop:

1. Launch STM32CubeIDE (any version 1.13 or newer).
2. Pick a workspace folder when prompted (the default is fine).
3. From the menu: **File → Import → General → Existing Projects
   into Workspace**, click **Next**.
4. Click **Browse**, navigate to the cloned project folder, and
   select the `bsau_v2` directory inside it. Click **Finish**.
5. The project appears in the Project Explorer on the left.

### 8.2 Choose the firmware mode

The BSAU has several firmware "modes" you can build (release mode
for normal use, debug modes for verifying the radio, the ADC, the
codec). For each mode, you edit one file, build, and flash.

Open the file `bsau_v2/Core/Inc/bsau_config.h` in CubeIDE. Near the
top you'll see a block of `#define` lines like:

```c
//  ─── Choose ONE mode ───
#define BSAU_MODE_DEBUG
//  #define BSAU_MODE_RELEASE
//  #define BSAU_MODE_TEST_PKT_LOG
//  #define BSAU_MODE_TEST_NRF_LOG
//  #define BSAU_MODE_TEST_ADC_CSV
//  #define BSAU_MODE_TEST_DFT_LOG
```

Exactly one of these must be uncommented (no `//` in front).
The others must be commented out.

For verifying the system, work through the modes in this order:

| Mode | What it tests | What you watch |
|---|---|---|
| `BSAU_MODE_DEBUG` | Boot, NRF init, ADC pipeline | UART startup messages |
| `BSAU_MODE_TEST_PKT_LOG` | Wireless packet codec (one-time test) | "23 PASS, 0 FAIL" then LED blinks slowly |
| `BSAU_MODE_TEST_NRF_LOG` | NRF radio self-test | "NRF SELF-TEST: 6 PASS" |
| `BSAU_MODE_TEST_ADC_CSV` | ADC pipeline (needs function generator) | Streamed CSV in SerialPlot |
| `BSAU_MODE_TEST_DFT_LOG` | Frequency analysis (needs function gen) | "peak=200Hz" matches input |
| `BSAU_MODE_RELEASE` | Production mode — switch to this last | Wireless packets at 1000/sec |

For each mode:

1. Edit `bsau_config.h`, comment out the previous mode, uncomment
   the new mode, save.
2. In CubeIDE menu: **Project → Build All** (or press Ctrl+B).
3. Click the green **Run** arrow. CubeIDE flashes the BSAU over
   the USB cable.
4. Open a serial terminal on the Nucleo's USB port at **921600
   baud, 8 data bits, no parity, 1 stop bit** (`921600 8N1`):
   - **Linux**: `screen /dev/ttyACM0 921600` (Ctrl+A then `\` to quit)
   - **macOS**: `screen /dev/tty.usbmodem* 921600`
   - **Windows**: open PuTTY, choose "Serial", set the COM port and
     baud rate, click Open.
5. Press the small black NRST button on the Nucleo to reset.
6. Watch the messages. Each test mode has a clear PASS/FAIL line.

### 8.3 Done with BSAU

After verifying each test mode, switch to `BSAU_MODE_RELEASE`,
build, and flash one final time. The BSAU's user LED will appear as
a steady dim glow (it's actually blinking at 1 kHz — too fast to see
as a blink). The BSAU is now ready to talk to the CPCU.

You won't touch the BSAU again unless you change the firmware. From
here on, all work is on the Pi via `./launch.sh`.

---

## 9. Hardware tests

Back to the Pi. These tests verify the wireless, the servo driver,
and the I²C bus — everything physical on the CPCU side.

### 9.1 Probe the Pi's hardware

```
./launch.sh test-hw
```

This runs the Phase 1 software tests (which you already passed),
then adds:

- IPC bridge validation (verifies C and Python agree on data
  layouts).
- Hardware probes: SPI port, I²C port, PCA9685 detection at
  address `0x40`, real-time core isolation, CPU temperature, CPU
  frequency.

Pass criteria: every line shows `[PASS]` and the final summary says
`0 FAIL`. If anything fails, the message tells you what to fix —
usually it's a wiring problem from section 3.

### 9.2 Servo motion check

```
./launch.sh test-pca
```

This launches an interactive servo testbench. You'll see a
text-based interface listing the 6 servos with their current pulse
widths. The kernel is **not** running here — this tool talks
directly to the servo driver.

Inside the testbench:

| Key | Effect |
|---|---|
| ↑ / ↓ | Select a servo (S0 through S5) |
| ← / → | Jog the selected servo by ±10 µs (small motion) |
| PgUp / PgDn | Jog by ±50 µs (larger motion) |
| `m` | Jog to the configured minimum |
| `M` | Jog to the configured maximum |
| `n` | Return to neutral (1500 µs) |
| `q` | Quit |

What to verify:
- All 6 servos respond when you select them and jog.
- Each servo moves the correct mechanical part of the prosthetic
  (S0 rotates the base, S5 closes the gripper, etc.).
- Hitting `m` and `M` produces a controlled sweep, not a sudden
  jerk that hits a hard mechanical stop.

If a servo doesn't move: cable, channel mapping (recheck section
3.3), or the 6 V supply is current-limited. Try one servo at a
time if you suspect the supply.

When you're done, press `q`.

### 9.3 Dashboard dry-run (no hardware)

```
./launch.sh test-signal-demo
```

This launches the signal-integrity dashboard with synthetic data
(no real wireless link, no BSAU needed). It's a good sanity check
that the dashboard renders correctly on your terminal before you
plug in real hardware.

You'll see 8 channels of synthetic 100 Hz sine waves rolling across
the screen. Press `q` to quit.

---

## 10. Live integration test

Now both boards are talking. This is the single most important
test: it proves the BSAU samples correctly, the wireless link is
clean, the CPCU receives without packet loss, and the safety
system reacts properly when something goes wrong.

### 10.1 Setup

1. BSAU is in `BSAU_MODE_RELEASE` from section 8.3, plugged in
   via USB to your laptop. The user LED appears as a steady dim
   glow.
2. The CPCU has servos powered (turn on the 6 V supply if it
   isn't already on).
3. Connect a function generator to the BSAU's PA0 pin set to:
   - **100 Hz sine wave**
   - **0.6 V amplitude (1.2 V peak-to-peak)**
   - **1.65 V DC offset**

   If you have one signal source, you can split it across PA1-PA7
   too (so all 8 channels see the same input) — useful for
   checking inter-channel consistency.

### 10.2 Launch the live signal test

```
./launch.sh test-signal
```

This brings up the kernel and the signal-integrity dashboard
simultaneously inside a single window manager (called tmux).
You'll see the dashboard with 8 channels plotted live.

**Important tmux key** (the only one you'll ever need to know):

- **`Ctrl-b` followed by `d`** — detach. The dashboard goes away
  but the kernel keeps running in the background. Use this if you
  want to do other things while the system runs.
- **`Ctrl-b` followed by `1`** — switch to the SHELL window. You can
  type `./launch.sh stop` there without detaching.
- After detaching, run `./launch.sh attach` to come back.
- Run `./launch.sh stop` when you're completely done.

Inside the dashboard, press **TAB** to toggle between
single-channel detail view and all-channel view.

Pass criteria, per channel:

| Metric | Target |
|---|---|
| Visible waveform | Clean sinusoid, no clipping |
| Dominant frequency | 100 Hz |
| Peak-to-peak | About 1.2 V |
| Signal-to-noise | Above 20 dB |
| DC offset | About 1.65 V |

At the top of the dashboard, the radio statistics row should show:

| Metric | Target |
|---|---|
| Packet rate | About 1000 per second |
| Sequence gaps | Less than 10 over a minute |
| Loss rate | Less than 0.1 % |

If you see no packets at all: BSAU is silent (verify section 8.3),
the NRF link is broken (recheck wiring from section 3.3), or there's
RF interference. If you see packets but the waveform is wrong: the
problem is in the BSAU's analog front-end (recheck section 8 with
the ADC and DFT test modes).

### 10.3 Test the safety system

While the system is still running from 10.2, test what happens when
something goes wrong. With the dashboard visible:

**Unplug the BSAU's USB cable.**

Watch the dashboard. Within about ¾ of a second:
1. The state indicator switches from `RUNNING` to `DEGRADED`.
2. The servos snap to their neutral positions (you'll hear them
   move briefly).
3. After about 1.5 seconds total, the state switches to `SAFE` and
   the servo outputs are turned off — the servos go limp.

**Plug the BSAU's USB back in.**

The BSAU re-initializes within about 1 second. The state should
recover to `RUNNING` automatically.

This proves the watchdog works. If the state doesn't change after
unplugging, the safety system isn't firing — stop and investigate
before relying on it.

When you're done, press `q` inside the dashboard, then run
`./launch.sh stop` to fully shut down.

---

## 11. Calibration

There are three layers of tunable values, in increasing rarity of use:

| Layer | What it controls | Tool |
|---|---|---|
| Per-servo limits and bias | Mechanical end-stops, neutral offsets | `./launch.sh test-pca` (save with `s`) |
| Runtime tunables | DSP thresholds, smoother knobs, grip pressure | TUI editor: `./launch.sh tui`, then press `e` on Config page |
| Compile-time safety thresholds | Watchdog timeouts, battery cutoffs | `./launch.sh configure` |

### 11.1 Per-servo limits (every time the prosthetic is reassembled)

Stop the live system if it's running:

```
./launch.sh stop
```

Then launch the calibration tool:

```
./launch.sh test-pca
```

For each of the 6 servos:

1. Press ↑ or ↓ to select the servo.
2. Jog it (← / →) gently into the safe minimum mechanical position
   — just before the servo whines or stalls against a hard stop.
   Press `[`. The MIN value updates.
3. Jog into the safe maximum position. Press `]`.
4. Jog to the centred resting position. Press `b` to set bias.
5. Repeat for all 6 servos.
6. Press `s` to save. Your changes write to a file on disk;
   the next time the live system runs, it picks them up.
7. Press `q` to exit.

### 11.2 Runtime tunables (during a tuning session)

Start the live system normally:

```
./launch.sh tui
```

You'll see the dashboard. The pages are accessed by number keys 1-7:

| Key | Page | Use |
|---|---|---|
| `1` | Overview | System state at a glance |
| `2` | Radio / IO | Wireless link statistics |
| `3` | DSP / AI | Current gesture, classifier confidence |
| `4` | Waves | Live waveform plots |
| `5` | Health | All-systems traffic-light status |
| `6` | Dataset | Record EMG to .csv files for training |
| `7` | Config | Read settings + edit mode |

Press `7` to go to the Config page, then press `e` to enter **edit
mode**. The status bar will say "PARKING ARM..." for a moment
(the safety system parks all servos at neutral so you don't tune
mid-motion), then switch to "EDITING — arm parked".

Use arrow keys to navigate the editable fields. Press Enter on a
field to begin typing a new value, Enter again to commit. Press
`Esc` to cancel a partial edit. When you're done with one or more
fields:

- **Ctrl+S** — saves all changes and reloads the live system in
  about 20 milliseconds.
- **`r`** — reverts all unsaved edits.
- **`e`** — exits edit mode and resumes normal motion.

Typical iteration loop: enter edit mode, change one value, Ctrl+S,
exit, test the prosthetic on your forearm, note what feels off,
repeat. Each iteration is about 5 seconds of arm-parked time.

### 11.3 Safety thresholds (rare — once or twice ever)

These are the watchdog timeouts, low-battery cutoffs, thermal
limits, and the radio channel. Changing them requires recompiling
the program, so the system asks for confirmation.

```
./launch.sh configure
```

With no arguments this is an interactive walkthrough — it shows
you each threshold one at a time, the current value, the
acceptable range, and lets you change it or skip.

Direct flags work too:

```
./launch.sh configure --show                       # list everything
./launch.sh configure --diff                       # only what's been changed
./launch.sh configure --reset                      # restore all to defaults
./launch.sh configure --radio-timeout 1000         # set one value
./launch.sh configure --vbat-low 3.1 --thermal-warn 70   # multiple
```

The available threshold names:

| Flag | What it controls |
|---|---|
| `--radio-timeout` | How many ms of wireless silence before declaring DEGRADED |
| `--radio-safe` | How long to stay in DEGRADED before going SAFE |
| `--boot-grace` | Time after boot before the radio watchdog arms |
| `--vbat-low` | Battery LOW threshold (volts) |
| `--vbat-crit` | Battery CRITICAL threshold (volts) |
| `--thermal-warn` | Temperature WARN level (°C) |
| `--thermal-crit` | Temperature CRITICAL level (°C) |
| `--i2c-max` | I²C errors before marking the bus failed |
| `--ring-overflow` | Internal buffer overflows allowed |
| `--nrf-channel` | Wireless channel number (0-125) |

After any change, you'll see:

```
[LAUNCH] WARN: ════════════════════════════════════════════════════════════
[LAUNCH] WARN:   REBUILD REQUIRED
[LAUNCH] WARN: ════════════════════════════════════════════════════════════

  Rebuild now? [Y/n]:
```

Type `y` (or just Enter, since `Y` is the default) and the system
rebuilds itself in about 30 seconds.

For runtime tunables (servo limits, gesture velocity, smoother
parameters, gripper pressure) — DON'T use `./launch.sh configure`,
that's only for safety thresholds. Use `./launch.sh tui` and the
edit mode from section 11.2 instead.

---

## 12. Running the system day-to-day

For routine use, you have three ways to run:

### 12.1 With the dashboard (recommended for development)

```
./launch.sh tui
```

Opens the dashboard alongside the running kernel. You see live
status, wave plots, classification results.

To leave the dashboard but keep the system running: **Ctrl-b** then
**d** (detach). To switch to a shell prompt: **Ctrl-b** then **1**.
To come back: `./launch.sh attach`. To shut
everything down: `./launch.sh stop`.

### 12.2 Via web browser (for demos, multiple viewers)

```
./launch.sh ws
```

Starts a web server on the Pi. From any laptop, phone, or tablet on
the same network, point a browser at:

```
http://<pi-address>:8765
```

The dashboard runs in the browser. Multiple people can watch
simultaneously. Press Ctrl+C in the Pi terminal to stop the web
server.

Note: by default the web dashboard is open to your whole local
network. Anyone on the same Wi-Fi can view (read-only — they can't
control). For private use only, run:

```
./launch.sh ws --bind ws://127.0.0.1:8765
```

This restricts access to the Pi itself.

### 12.3 With the menu (for a guided choice)

If you forget which command you want, run with no argument:

```
./launch.sh
```

A menu appears letting you pick.

### 12.4 Running without the dashboard

For demos where the dashboard isn't useful, run just the kernel:

```
./launch.sh kernel
```

This runs forever (Ctrl+C to stop) without any user interface. The
servos respond to muscle signals, the safety system is active,
logs go to a file. This is the same mode used by the auto-start
service (next section).

---

## 13. Auto-start at boot

If you want the prosthetic to come up automatically every time the
Pi powers on (so you don't need to SSH in and type a command):

```
./launch.sh install-service
```

You'll see one password prompt. After it finishes, the kernel
starts at every boot.

To control the service:

```
sudo systemctl start cpcu      # start now
sudo systemctl stop cpcu       # stop now
sudo systemctl status cpcu     # check what state it's in
journalctl -u cpcu -f          # watch live logs
```

If you also want the web dashboard to start at boot:

```
./launch.sh install-ws-service
```

The dashboard becomes available at `http://<pi-address>:8765`
within a few seconds of every boot.

To remove the auto-start later:

```
sudo systemctl disable cpcu
sudo systemctl disable cpcu_ws    # if you also installed it
```

---

## 14. Stopping, recovering, troubleshooting

### 14.1 Stop the system

```
./launch.sh stop
```

This kills the running session and all its child processes
cleanly. If you installed it as a service:

```
sudo systemctl stop cpcu
sudo systemctl stop cpcu_ws
```

### 14.2 Something seems wrong — what's the diagnostic command?

```
./launch.sh check
```

This reports every requirement — is the build present, are
capabilities applied, is the wireless module detected, is the
servo driver detected, etc. — and tells you the exact command to
fix each problem.

### 14.3 Common situations and what to do

**The servos went limp during a demo.**
The safety system tripped to SAFE. Most common cause: BSAU lost
power or the wireless link dropped. Check the BSAU is plugged in,
look at the dashboard's Health page (key `5` in the TUI) for the
specific reason. Once the cause is removed, the system recovers
within about 1 second.

**The gripper is pinned at the floor and a "stalls" counter is
incrementing.**
The soft-grip protection is firing. Open the gripper manually with
your other hand. If it keeps happening on a soft object, the
"firm grip" pulse width is set too low — go to `./launch.sh tui`,
press `7` then `e`, raise `grip_firm_us` by about 50, Ctrl+S, exit
edit mode.

**Servos are jittering when supposedly held still.**
The smoother's deadband is too tight for that servo. In the TUI
editor (Config page, edit mode), increase `smoother_deadband_us`
for the affected servo. Try increments of 5 µs.

**The dashboard shows packets arriving but the gesture is always
"rest".**
Either the model isn't deployed (run `./launch.sh check` and look
for the model warning), or the SVM confidence is below the
threshold (try lowering `confidence_threshold` in the TUI editor).

**`./launch.sh tui` says "tmux not installed".**
Re-run `./launch.sh setup` — that installs tmux automatically.

**The dashboard window is frozen.**
The dashboard process or the kernel died. Run `./launch.sh stop`
to clean up, then `./launch.sh check` to find what's wrong, then
`./launch.sh tui` again.

### 14.4 Logs

Logs are written to `/var/log/cpcu/`. You don't need to look at
them yourself in normal operation; if you do need to:

```
journalctl -u cpcu -f                      # live tail (if running as service)
tail -f /var/log/cpcu/cpcu.log             # live tail (if running via launch.sh)
```

When something is mysteriously failing, sharing the output of
`./launch.sh check` and the last 50 lines of the log file is
usually enough to diagnose.

### 14.5 Hard reset (if everything is hung)

If `./launch.sh stop` doesn't work and the system is wedged, the
nuclear option:

```
sudo reboot
```

This always works. After about a minute the Pi is back, and you
can `./launch.sh tui` (or whichever) to pick up.

---

## 15. Quick reference card

The minimum you need to remember is this, in order:

```
# === First time on a new Pi ===
ssh <user>@<pi>.local
git clone <repo-url> ~/prosthetic_hand
cd ~/prosthetic_hand/cpcu_v2
./launch.sh setup                # one password prompt; reboot if asked
./launch.sh build                # one password prompt
./launch.sh check                # everything green?

# === Verification (skip if already passed) ===
./launch.sh test-sw              # 233 PASS expected
./launch.sh test-hw              # all hardware checks
# (flash BSAU on laptop in CubeIDE, see section 8)
./launch.sh test-signal          # live, with function generator on PA0

# === Calibration ===
./launch.sh test-pca             # per-servo limits, save with 's', quit with 'q'
./launch.sh tui                  # press 7, then 'e' for runtime tunables
./launch.sh configure --show     # safety thresholds (rare)

# === Daily operation ===
./launch.sh tui                  # interactive dashboard
./launch.sh ws                   # browser dashboard, http://<pi>:8765
./launch.sh stop                 # stop everything
./launch.sh check                # diagnose anything wrong

# === Boot-time auto-start (optional) ===
./launch.sh install-service      # kernel auto-starts at boot
./launch.sh install-ws-service   # web dashboard too

# === Help ===
./launch.sh help                 # full command list
./launch.sh help <command>       # detail on a specific command
```

That's the entire user-facing surface. There is no other command,
no other tool, and no other thing you need to know to operate the
prosthetic hand system end-to-end.

---

## See also

For when you want to dig deeper than this guide:

- **[`ARCHITECTURE.md`](ARCHITECTURE.md)** — system architecture: how the CPU cores
  divide labor, how shared memory is laid out, how the safety FSM works.
  Read this if you're modifying the C source.
- **[`CONFIGURATION.md`](CONFIGURATION.md)** — every tunable in the system, with
  defaults, ranges, and rationale. Read this when `./launch.sh configure --show`
  isn't enough detail.
- **[`TESTING.md`](TESTING.md)** — the test plan in full: what each of the 233
  software tests covers, what the hardware tests prove, how to add a new test.
- **[`BOOT_AND_SYNC.md`](BOOT_AND_SYNC.md)** — the cold-start grace period that
  prevents spurious safety trips when CPCU boots before BSAU.
- **[`TUI_EDITOR.md`](TUI_EDITOR.md)** — the live-editing dashboard (page 7,
  press `e`) and the underlying edit-mode handshake protocol.
- **[`WEB_DASHBOARD.md`](WEB_DASHBOARD.md)** — the browser-based read-only
  multi-viewer dashboard (`./launch.sh ws`).
- **Topic deep-dives:** [`JITTER_MITIGATION.md`](JITTER_MITIGATION.md),
  [`VELOCITY_MODE.md`](VELOCITY_MODE.md),
  [`SOFT_GRIP.md`](SOFT_GRIP.md) — feature-specific docs for the
  smoother, the gesture velocity stack, and the gripper protection.

---

**Project:** InfiniTech Prosthetic Hand
**Document version:** v2.7 (April 2026)
**System version:** CPCU v2.7 / launch.sh v2.7


---

# Appendix: TUI Live Editor — In-System Runtime Tuning

> **Merged from:** `TUI_EDITOR.md` (v2.3.8).
> Covers the TUI CONFIG page editor, field model, edit-mode handshake,
> save protocol, and what's not editable.

## TL;DR

Press `e` on the TUI's CONFIG page (page 7) to enter edit mode. The
arm parks at neutral via the v2.3.4 handshake. The CONFIG page
switches from spec-sheet view to a **navigable spreadsheet editor**
covering 13 runtime.json fields:

- 6 per-servo arrays (servo_min/max/bias, smoother vel/accel/deadband)
- 7 scalar fields (DSP thresholds, grip levels)

**Keys in edit mode:**

| Key | Effect |
|---|---|
| Arrows | Move cursor (row = field, col = servo for arrays) |
| Enter | Begin numeric entry on the selected cell |
| Digits / `-` / Backspace | Type the new value (`-` only for `servo_bias_us`) |
| Enter | Commit entered value (clamped to range, marked dirty) |
| Esc | Cancel in-flight entry, restore prior value |
| `r` | Revert all dirty cells to disk values |
| Ctrl+S | Save dirty cells to runtime.json + SIGHUP cpcu_kernel |
| `e` | Exit edit mode (resumes normal operation) |

Saves use the v2.3.6 `CFG_PatchFile` API — surgical edit, only
dirty fields rewritten, other JSON keys preserved byte-for-byte
(including `gesture_velocity` which the C parser doesn't know
about). The kernel re-parses on SIGHUP and republishes
`IPC_RuntimeConfig` within ~20 ms; cpcu_io's smoother re-applies
on `config_seq` change automatically.

---

## 1. Why a TUI editor

You can already edit `runtime.json` from any text editor and
`kill -HUP $(pgrep cpcu_kernel)` to reload. So why a TUI editor?

**Bench-discovered values can be saved from the bench tool, but the
live system isn't the bench.** When you're wearing the prosthetic
and notice the gripper is too aggressive, you want to lower
`grip_firm_us` *now*, not stop the daemon and run pca_testbench.
The TUI editor lets you tune while the system is running, with the
arm safely parked.

**Edit-mode handshake guarantees safety.** v2.3.4 ensures the arm
isn't moving while you edit. SAFE has priority — if anything goes
wrong during edit, the arm snaps to neutral and the editor stays
disabled until SAFE clears.

**Single source of truth for editable fields.** A declarative table
in `cpcu_tui_editor.c` lists every field, its range, and its kind.
Adding a new editable field is one row.

---

## 2. The field model

Each row in `g_ed_fields[]` is an `ED_Field`:

```c
typedef struct {
    const char    *json_key;        // "servo_min_us"
    const char    *display_name;    // "servo_min_us"
    const char    *units;           // "us", "us/s", "%", ""
    ED_FieldKind   kind;            // U16, I16, U8
    int            count;           // 1 (scalar) or 6 (per-servo)
    int            range_min, range_max;
    int            draft[6];        // current edit state
    int            disk[6];         // last-saved baseline
    bool           dirty[6];        // draft != disk
} ED_Field;
```

`draft` is what the editor displays. `disk` is the source of truth
on file. `dirty[i]` is set whenever `draft[i] != disk[i]`, cleared
on save or revert.

The kind only affects entry rules and save-time casting:
- `ED_KIND_U16` accepts digits 0-9, range typically 100..50000.
- `ED_KIND_I16` accepts `-` followed by digits, range ±100 (only
  used for `servo_bias_us`).
- `ED_KIND_U8` accepts digits, range 0..100 (percentages) or 1..20
  (`hysteresis_votes`).

---

## 3. The two-mode state machine

```
       ┌─────────────────┐
       │     NAV mode    │
       │ (cursor moves)  │◄─────────┐
       └─────────────────┘          │
        Enter│                Esc/Enter
             │                      │
             ▼                      │
       ┌─────────────────┐          │
       │   ENTRY mode    │──────────┘
       │ (typing digits) │
       └─────────────────┘
```

**NAV mode** (default on entering the editor): arrows move the
cursor, no values change.

**ENTRY mode** (after pressing Enter on a cell): digits/backspace/
sign accumulate in a buffer shown next to the cell. Enter commits
to draft (with range clamping); Esc cancels and restores prior draft.

This nesting matters: the global `e` key for exiting edit mode does
NOT take effect inside ENTRY — you'd never accidentally exit edit
mode mid-typing. The only way out of ENTRY is Enter or Esc.

The cursor is `(row, col)` indexing into `g_ed_fields[]`. For scalar
fields (count=1), col is forced to 0. Switching rows with up/down
clamps col to the new row's count.

---

## 4. Edit-mode handshake protocol (v2.3.4)

The naive alternative would be: TUI just sets a flag, dsp+io
immediately stop their normal work. But that's bad in two ways:

**Mid-motion freeze is dangerous.** If dsp publishes a motor command
and io is mid-trajectory toward it, freezing in place leaves the arm
in a transient pose — possibly mid-air with the gripper closing on
something. Better to walk to a known-safe pose (neutral) before
declaring "ok to edit".

**The user needs feedback.** If the editor activates *immediately*
on `e` while the arm is still moving for ~1 second, the user starts
typing values that get applied to a moving arm. They lose the
mental model of "static state I'm tweaking."

So the handshake has two phases:
1. **PARKING**: request raised, walking to neutral. UI shows yellow banner, edits blocked.
2. **EDITING**: settled at neutral, fully parked. UI shows green banner, edits allowed.

The transition between them is data-driven (`SMOOTH_AllSettled()`),
not time-based.

---



### Wire-level protocol

Three new atomic bytes plus one timestamp in `IPC_ControlBlock`
(reserve region of cache line 0 — no layout change, IPC_VERSION
bumped 0x0203 → 0x0204):

```c
_Atomic uint8_t     edit_mode_request;      // TUI -> world
_Atomic uint8_t     edit_mode_active;       // io  -> TUI
_Atomic uint8_t     edit_mode_dsp_ack;      // dsp -> TUI
_Atomic uint64_t    edit_mode_request_us;   // TUI stamps on raise
```

### Single-writer-per-byte rule

Each byte has exactly one writer. Multiple readers are fine.

| Byte | Writer | Readers |
|---|---|---|
| `edit_mode_request` | TUI | cpcu_io, cpcu_dsp.py |
| `edit_mode_active` | cpcu_io | TUI |
| `edit_mode_dsp_ack` | cpcu_dsp.py | TUI |
| `edit_mode_request_us` | TUI (stamps on raise) | TUI (reads for timeout calc) |

Single-writer-per-byte means we don't need a seqlock — atomic loads
and stores at byte granularity are sufficient. The four bytes don't
need to be a coherent snapshot, only individually atomic.

### Sequence — entering edit mode

```
t=0      TUI:    edit_mode_request := 1
                 edit_mode_request_us := now_us
                 (banner switches to "[PARKING ARM...]" yellow)

t=20ms   io:     observes request=1
                 SMOOTH_SetAllTargets(neutral)
                 (smoother begins trapezoidal walk to 1500us)
                 motor_cmd from dsp ignored (sticky-park)

t=80ms   dsp:    observes request=1
                 commits current_state := "rest"
                 hysteresis_count := 0
                 stops calling write_motor_cmd
                 edit_mode_dsp_ack := 1

t=300ms  io:     SMOOTH_AllSettled() -> true (depending on starting pose)
                 edit_mode_active := 1

t=320ms  TUI:    next render observes active=1
                 banner switches to "[EDITING — arm parked]" green
                 editor unlocks
```

### Sequence — exiting edit mode

```
t=0      TUI:    edit_mode_request := 0
                 (banner switches to "[LOCKED]" dim)

t=20ms   io:     observes request=0
                 edit_mode_active := 0
                 normal motor_cmd processing resumes
                 (smoother walks back toward whatever dsp publishes;
                  often that's still neutral if user hasn't started
                  any gesture yet, so motion is minimal)

t=80ms   dsp:    observes request=0
                 edit_mode_dsp_ack := 0
                 resumes write_motor_cmd
```

### Sequence — fault during edit mode

```
t=0      User in edit mode, banner green.

t=X      Some safety fault triggers (radio drop, battery, etc.).
         FSM transitions RUNNING → SAFE.

t=X+ε    io:     SAFETY_CheckSystem() returns false.
                 SAFE-snap branch fires:
                 SMOOTH_Snap to neutral.
                 PCA_SetAllNeutral.
                 edit_mode_active := 0  (forced clear)

t=X+1tick TUI:   observes active=0 + system_state=SAFE.
                 banner switches based on edit_req still being set:
                   if request=1 still: "[PARKING ARM...]" yellow
                   even though arm is already at neutral, the FSM
                   forced our hand and the user should explicitly
                   re-press 'e' to confirm intent.
```

The user experience: a fault forces the editor closed even if you
were mid-edit. You re-press `e` after the system recovers.

---



## 5. DSP UNRESPONSIVE timeout

If the user presses `e` and `cpcu_dsp.py` is hung (crashed silently,
deadlocked in scipy, whatever), then `edit_mode_dsp_ack` never goes
to 1. The TUI's banner watches for this:

```c
if(edit_req && !edit_active && elapsed_ms > 500 && !edit_dsp_ack)
    banner = "[DSP UNRESPONSIVE]"  // red
```

At 500 ms the TUI flips the banner red. The user sees "the DSP
isn't acknowledging" and can investigate — usually `tail -f
/var/log/cpcu/log_DSP.csv` or `pgrep -af cpcu_dsp.py`.

**Note that `edit_mode_active` does NOT depend on `edit_mode_dsp_ack`.**
cpcu_io's view of "ready to edit" is purely about whether the smoother
has settled — that's the safety-relevant condition. dsp's ack is
diagnostic. If dsp is dead but io is healthy, you can technically
still edit (the arm is parked), you just won't have inference
running. The banner makes that visible without blocking the editor.

---



---

## 6. The save protocol

`Ctrl+S` triggers `ed_save()`:

1. Walk `g_ed_fields[]`, find every field with at least one dirty cell.
2. Build a `CFG_PatchEntry` per dirty field. Each entry rewrites the
   *whole array* — partial updates aren't supported by the patcher,
   but rewriting the whole array with the in-memory draft (which has
   non-dirty cells preserved at their disk values) is equivalent.
3. Resolve the target file: prefer `/opt/cpcu/config.json` if writable,
   fall back to `config/runtime.json`. Fail loudly if neither.
4. Call `CFG_PatchFile()` — surgical text-level edit, atomic via
   tmpfile + `rename(2)`. On failure, leave drafts dirty so the user
   can retry.
5. On success, promote each draft cell to disk and clear dirty.
6. Read `kernel_pid` from `IPC_ControlBlock` and `kill(pid, SIGHUP)`.
   Status line confirms.
7. Within ~20 ms, cpcu_kernel re-parses runtime.json and republishes
   `IPC_RuntimeConfig` with bumped `config_seq`. cpcu_io notices the
   bump on its next servo tick and re-applies smoother values.

**What if kernel_pid is 0?** It would be 0 only if the kernel hasn't
finished startup yet (it publishes the pid right after `IPC_Create`).
The save still succeeds — the file is written — but the running
system won't reload until you manually run `kill -HUP $(pgrep
cpcu_kernel)`. The status line tells you that's needed.

**What if the file is non-writable?** The status line says "SAVE
FAILED: no writable runtime.json found". Drafts stay dirty.

---

## 7. The `kernel_pid` field in IPC

A new field in `IPC_ControlBlock`:

```c
_Atomic uint32_t    kernel_pid;
```

Allocated from the existing `_reserved0[12]` pool — consumed 4
bytes, 8 remain. cpcu_kernel writes it once at startup:

```c
atomic_store(&ipc.ctrl->kernel_pid, (uint32_t)getpid());
```

No other writer. The TUI reads it on Ctrl+S to send SIGHUP. The
DSP doesn't need it. cpcu_io doesn't need it.

`IPC_VERSION` was bumped from `0x0204` to `0x0205` to mark the new
contract. The byte-level layout is unchanged (the field consumed
reserved space), but a tool that strictly checks version will
require a rebuild — correct behavior for a contract change.

---

## 8. What's NOT editable (and why)

The editor surfaces a **curated subset** of runtime.json. Several
fields are intentionally absent:

### `gesture_velocity`

A string-keyed nested object — not array of int16 like everything
else, so the patcher can't write it directly. More importantly,
**dsp loads `gesture_velocity` once at startup** and never re-reads
it. Even if we patched it, the running dsp wouldn't see the change.
Editing it requires `kill -HUP cpcu_kernel` AND restarting dsp.
Better to do this from your editor + restart cycle.

### `servo_min_us` / `servo_max_us` (debatable)

Mechanical limits feel like they belong in the bench tool — you
need to *physically watch the servos* to find their real limits,
which is what `pca_testbench` is for. The TUI editor exposes them
anyway for emergencies (e.g., "the servo is grinding at the lower
limit and I can't stop and run pca_testbench right now"). Use with
caution; pca_testbench is the proper home.

### `schema_version`

Not user-editable. Bumping it requires a coordinated update across
the C parser, the dsp loader, and the JSON file. Not a runtime knob.

### Compile-time things

Anything that's a `#define` rather than a runtime field — RADIO_
TIMEOUT_MS, NRF channel, IPC layout sizes. These need a recompile.
See [`CONFIGURATION.md`](CONFIGURATION.md).

---

## 9. Visual layout

```
Edit mode: [EDITING - arm parked]   Press 'e' to exit, Ctrl+S to save

FIELD                       S0      S1      S2      S3      S4      S5    UNITS
──────────────────────────────────────────────────────────────────────────────
servo_min_us                498   1074   1074   1001   1001    976    us
servo_max_us               2500   1953   1953   2002   2002   1733    us
servo_bias_us                 0      0      0      0      0      0    us
smooth_velocity            2000   2000   2000   2000   2000   1200    us/s
smooth_accel               8000   8000   8000   8000   8000   8000    us/s2
smooth_deadband              10     10     10     10     10     10    us
interp_floor_pct             40                                       %
interp_ceil_pct              85                                       %
hysteresis_votes              3
grip_open_us               1700                                       us
grip_touch_us              1200                                       us
grip_firm_us               1100                                       us
grip_stall_recover         2000                                       ms
──────────────────────────────────────────────────────────────────────────────
NAV    arrows=move  Enter=edit  r=revert all  Ctrl+S=save  dirty=0
```

**Highlighted cell** = current cursor. **Reverse video** marks the
cell when in NAV mode. **Bold yellow + asterisk** marks dirty cells
(unsaved edits). **Cyan** is the default cell color.

The status line at the bottom rotates between:
- mode/cursor info ("NAV ..." or "ENTRY ...")
- save/load result ("saved 3 patches to ... -- SIGHUP'd kernel pid 1234")
- dirty warning ("* 5 unsaved changes — press Ctrl+S to commit, r to revert *")

---

## 10. Interaction with safety

**SAFE forces exit.** If the safety FSM trips while editing,
`edit_mode_active` gets cleared by cpcu_io (priority over the
handshake). The TUI's renderer notices `edit_active=0` and falls
back to the spec-sheet view. **Drafts are preserved in memory** —
they don't write to disk and they're not discarded. When you next
re-enter edit mode (after SAFE recovers), you can resume editing
or save what you had.

This means a SAFE event during a long edit session doesn't lose
your work. But: if `runtime.json` was reloaded by the kernel during
the SAFE event (e.g., another tool patched it), your drafts are
now stale — they'd be saved against new disk baselines. The next
ED_Init refreshes baselines, so the dirty flags get recomputed
correctly on next edit-mode entry. **Drafts that match the new
disk values stop being dirty automatically; drafts that differ
stay dirty and ready for save.**

**Edit mode entry is gated.** You can only enter edit mode (press
`e`) when `system_state == RUNNING`. In SAFE state, `e` does
nothing visible. This prevents tuning while faulted.

---

## 11. Testing

`test/editor_testbench.c` runs five test groups:

| Group | What |
|---|---|
| TB-ED01 | `ED_Init()` loads disk values and clears dirty |
| TB-ED02 | NAV-mode arrow keys move the cursor between rows/cols |
| TB-ED03 | Out-of-range entry gets clamped (both ends) |
| TB-ED04 | Esc cancels in-flight entry without dirtying |
| TB-ED05 | Ctrl+S round-trip via `CFG_PatchFile` — value persists, untouched cells preserved |

Render is NOT unit-tested — it would require an ncurses pty fixture
and ANSI-escape parsing for ~150 lines of `mvprintw` calls. Verified
visually on hardware (and on the host with `cpcu_tui --demo` once
the editor's lazy-init path is exercised).

`test_dsp_pipeline.py`'s `gesture_velocity` parsing tests still pass
unchanged — the editor doesn't touch that field, the patcher
preserves it.

---

## 12. Operating procedure

### First-time tuning session

```bash
sudo systemctl start cpcu              # bring up the daemon
./cpcu_tui                             # run as your user
# Press '7' to switch to CONFIG page.
# Press 'e' to enter edit mode.
# Banner says "PARKING ARM..." then "EDITING - arm parked".
# Use arrows to navigate, Enter to edit a cell.
# Ctrl+S to save when you're done.
# Press 'e' again to exit edit mode and resume normal operation.
```

### Iterative tuning (the typical workflow)

```
1. Enter edit mode (arm parks).
2. Adjust one or two cells.
3. Ctrl+S — kernel reloads, smoother re-applies.
4. Exit edit mode — system resumes with new values.
5. Wear and test. Note what feels off.
6. Re-enter edit mode. Repeat.
```

Each iteration is ~5 seconds of arm-parked time. The `config_seq`
mechanism makes the smoother changes feel immediate on resume.

### Recovery from a bad save

If you save values that make the system unusable (e.g., velocity
1, deadband 50), here's how to recover without rebooting:

```bash
# In another terminal:
$EDITOR cpcu_v2/config/runtime.json    # restore by hand
kill -HUP $(pgrep cpcu_kernel)         # kernel re-parses
```

Or use pca_testbench's `L` (reload) key after stopping the daemon.

---

## 13. See also

- [`TUI_EDITOR.md`](TUI_EDITOR.md) §4 — the v2.3.4 handshake protocol
  the editor sits on top of. Explains banner states, DSP UNRESPONSIVE
  timeout, SAFE-has-priority semantics.
- [`CONFIGURATION.md`](CONFIGURATION.md) — schema for every field
  the editor surfaces. §10 covers pca_testbench round-trip (the same
  `CFG_PatchFile` infrastructure).
- [`SOFT_GRIP.md`](SOFT_GRIP.md) — `grip_firm_us` and friends are
  the most commonly tuned values, and the editor's main use case.
- [`VELOCITY_MODE.md`](VELOCITY_MODE.md) — explains why
  `gesture_velocity` isn't editable from the TUI.
- [`ARCHITECTURE.md`](ARCHITECTURE.md) §3.3 — core
  allocation. The editor lives entirely on Core 0 (with the rest of
  the TUI).
- [`cpcu_v2/include/cpcu_tui_editor.h`](../include/cpcu_tui_editor.h) —
  full editor API.
- [`cpcu_v2/src/cpcu_tui_editor.c`](../src/cpcu_tui_editor.c) —
  state machine + render + save logic.
- [`cpcu_v2/test/editor_testbench.c`](../test/editor_testbench.c) —
  TB-ED01..ED05 unit tests.
