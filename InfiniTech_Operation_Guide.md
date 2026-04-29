# InfiniTech — Step-by-Step Operation Guide

**Author:** bugrASl
**Date:** April 2026
**Audience:** the operator (you) doing the full bring-up sequence.
**Scope:** every command you'll run, in the exact order, from
unboxed boards to a live system. Tests first → individual
calibrations → final launch.

The flow is:

> **Phase 0** — Hardware, wiring, pre-power checks
> **Phase 1** — One-time Pi setup + reboot
> **Phase 2** — Build, install, RT capabilities, ML model deploy
> **Phase 3** — Phase-1 software tests on the Pi (no hardware needed)
> **Phase 4** — BSAU standalone tests (just the STM32 + USB)
> **Phase 5** — CPCU hardware tests (Pi peripherals, no BSAU yet)
> **Phase 6** — IPC validation (kernel running)
> **Phase 7** — Live integration test (BSAU + CPCU, sine wave end-to-end)
> **Phase 8** — Calibrations (servos, runtime tunables, safety thresholds)
> **Phase 9** — Final launch (TUI mode, dashboard, or systemd)
> **Phase 10** — Stop / recover / iterate

If a step fails, stop and fix it before moving on. Tests are
ordered bottom-up because a failure at layer N invalidates every
number measured at layer N+1.

---

## Phase 0 — Hardware and wiring (do this once, then never touch)

### 0.1 What you need on the bench

- NUCLEO-L432KC (BSAU board)
- Raspberry Pi 5 with active cooler + 32 GB+ microSD card
- 2× NRF24L01+ modules (one per side)
- PCA9685 16-channel servo driver breakout
- 6 servos already wired into the prosthetic arm: 3× MG995 (S0–S2), 3× SG90 (S3–S5)
- USB-C 5 V / 5 A PSU for the Pi
- Separate 6 V / ≥ 3 A bench PSU for the servo rail
- Micro-USB cable (Nucleo) + USB-C cable (Pi)
- Multimeter
- Two GND-only jumpers between the Pi and the servo PSU (common ground is mandatory)

### 0.2 BSAU side wiring (NUCLEO-L432KC)

The Nucleo silkscreen has both Arduino labels (`D0`, `A0`) and
STM32 pin names (`PA0`, `PB6`). **Always wire to the STM32 pin
name** — that's what the firmware references.

#### 0.2.1 NRF24L01+ on the Nucleo

| NRF pin | Nucleo pin | Function |
|---|---|---|
| VCC | 3V3 (silk) | Power — **must be 3.3 V, NOT 5 V** |
| GND | GND (silk) | Ground |
| CSN | PB7 | SPI chip select, active-low |
| SCK | PB3 | SPI clock, 5 MHz |
| MOSI | PB5 | SPI data to NRF |
| MISO | PB4 | SPI data from NRF |
| CE | PA8 | NRF chip enable |
| IRQ | PB6 | Interrupt (busy-polled by firmware in v2; connect anyway) |

**Decoupling caps directly across NRF VCC/GND:** 10 µF tantalum +
100 nF ceramic, as close to the module pins as possible. Without
these you get phantom packet loss that's actually brownouts.

#### 0.2.2 EMG front-end inputs (8 channels)

| STM32 pin | ADC channel | Purpose |
|---|---|---|
| PA0 | IN5 | EMG ch 0 |
| PA1 | IN6 | EMG ch 1 |
| PA2 | IN7 | EMG ch 2 |
| PA3 | IN8 | EMG ch 3 |
| PA4 | IN9 | EMG ch 4 |
| PA5 | IN10 | EMG ch 5 |
| PA6 | IN11 | EMG ch 6 |
| PA7 | IN12 | EMG ch 7 |
| PB0 | IN15 | Battery monitor (post-divider) |

The amplifier reference voltage should be ~1.65 V (mid-rail) so
both polarities of muscle activation appear as deviations from
ADC count 2048.

### 0.3 CPCU side wiring (Raspberry Pi 5)

#### 0.3.1 NRF24L01+ on the Pi (SPI0)

| NRF pin | Pi physical pin | BCM GPIO |
|---|---|---|
| VCC | 1 or 17 | 3V3 — **3.3 V, NOT 5 V** |
| GND | 6, 9, 14, 20, 25, 30, 34, or 39 | GND |
| CSN | 24 | GPIO 8 (SPI0 CE0) |
| SCK | 23 | GPIO 11 (SPI0 SCLK), 8 MHz |
| MOSI | 19 | GPIO 10 (SPI0 MOSI) |
| MISO | 21 | GPIO 9 (SPI0 MISO) |
| CE | 22 | GPIO 25 |
| IRQ | 18 | GPIO 24 (busy-polled) |

Same decoupling caps as the BSAU side.

#### 0.3.2 PCA9685 on the Pi (I²C1)

| PCA pin | Pi physical pin | BCM GPIO |
|---|---|---|
| VCC | 1 (3V3 — same rail as NRF is fine) | 3V3 |
| GND | 14 (or any GND) | GND |
| SDA | 3 | GPIO 2 (SDA1) |
| SCL | 5 | GPIO 3 (SCL1) |
| OE | leave floating or tie to GND | — |

Most PCA9685 breakouts have the 4.7 kΩ I²C pull-ups already
populated. If yours doesn't (rare), add 4.7 kΩ from SDA to 3.3 V
and 4.7 kΩ from SCL to 3.3 V externally.

PCA address is `0x40` by default (matches the firmware).

#### 0.3.3 Servo power side of the PCA9685

| PCA terminal | Connect to |
|---|---|
| V+ | Positive of the **6 V bench PSU** (NOT the Pi's 5 V) |
| GND | **Both** Pi GND **and** the 6 V PSU's negative — common ground is mandatory |

Two physically-separate PSUs, one shared ground. If you forget the
common ground the PWM is referenced to nothing on the servo side
and you get random twitching or no motion.

#### 0.3.4 Servo cables

Each servo has a 3-wire connector (brown/black GND, red V+,
yellow/orange signal). Plug each onto a PCA9685 output channel.
The 6 servos are wired to **non-contiguous** PCA channels:

| Logical name | PCA terminal | Servo type | Pulse range |
|---|---|---|---|
| S0 Base | 0 | MG995 | 498–2500 µs |
| S1 Upper | 1 | MG995 | 1074–1953 µs |
| S2 Last | 11 | MG995 | 1074–1953 µs |
| S3 Joint-1 | 8 | SG90 | 1001–2002 µs |
| S4 Joint-2 | 5 | SG90 | 1001–2002 µs |
| S5 Gripper | 4 | SG90 | 976–1733 µs |

The firmware translates logical S0–S5 into the right physical PCA
channel via `PCA_SERVO_CHANNEL` in `cpcu_pca9685.h`. If you reroute
cables, that single macro is the source of truth — edit it.

### 0.4 Pre-power continuity + resistance checks

Before you plug **anything** into power, with multimeter in
continuity / resistance mode:

**BSAU side:**

```
Continuity (should beep):
  NRF VCC  ↔ Nucleo 3V3 silk
  NRF GND  ↔ Nucleo GND  silk
  NRF CSN  ↔ PB7
  NRF SCK  ↔ PB3
  NRF MOSI ↔ PB5
  NRF MISO ↔ PB4
  NRF CE   ↔ PA8
  NRF IRQ  ↔ PB6

Resistance (DO NOT POWER if you read a few ohms — that's a short):
  NRF VCC ↔ GND      → > 1 MΩ
```

**CPCU side:**

```
Continuity (should beep):
  NRF VCC      ↔ Pi pin 1 or 17 (3V3)
  NRF GND      ↔ Pi GND
  NRF CSN      ↔ Pi pin 24
  NRF SCK      ↔ Pi pin 23
  NRF MOSI     ↔ Pi pin 19
  NRF MISO     ↔ Pi pin 21
  NRF CE       ↔ Pi pin 22
  NRF IRQ      ↔ Pi pin 18

  PCA VCC      ↔ Pi pin 1 (3V3)
  PCA GND      ↔ Pi GND
  PCA SDA      ↔ Pi pin 3
  PCA SCL      ↔ Pi pin 5
  PCA V+       ↔ servo PSU positive
  PCA GND term ↔ servo PSU negative   ← common ground, MUST beep

Resistance:
  NRF VCC ↔ GND       → > 1 MΩ
  PCA V+  ↔ GND       → > 1 MΩ
  Pi 3V3  ↔ GND       → several hundred kΩ (Pi unpowered)
```

Then verify the servo PSU output voltage **before** connecting it:
set to 6.0 V, turn on, measure with the DMM, turn off, plug into
the PCA V+ terminal.

### 0.5 First power-on (order matters)

1. Plug the **Nucleo USB** into your laptop. The red LD1 power
   LED lights immediately; LD3 stays dark until firmware runs.
2. Plug the **Pi USB-C** into its 5 V PSU. Red LED solid, green
   LED blinks during boot, settles when boot finishes.
3. **Only after the Pi has fully booted** (green LED settled),
   turn on the **6 V servo PSU**. Powering the servo rail at the
   same time as the Pi can brown out the Pi from the servos'
   inrush capacitors.

Hardware bring-up is now complete. Everything below is software.

---

## Phase 1 — One-time Pi setup

You'll do this exactly once per fresh Pi. The script self-elevates
to root via sudo for the privileged steps, so the only sudo prompt
you see is the one from sudo asking for your password.

### 1.1 SSH into the Pi

From your laptop:

```bash
ssh <your-pi-username>@<pi-ip-or-hostname>.local
```

If you don't know the IP, check your router's DHCP table or run
`arp -a | grep <Pi-MAC>`.

### 1.2 Update the OS and install git

```bash
sudo apt update && sudo apt -y full-upgrade
sudo apt install -y git
sudo reboot
```

SSH back in after the reboot.

### 1.3 Clone the repo

```bash
git clone <your-repo-url> ~/prosthetic_hand
cd ~/prosthetic_hand/cpcu_v2
```

### 1.4 Run setup_pi.sh

```bash
./setup_pi.sh
```

You'll see one sudo password prompt as the script self-elevates.
The script:

1. Installs the build toolchain and Python deps (`gcc`, `cmake`,
   `libncurses-dev`, `tmux`, `numpy`, `scipy`, `scikit-learn`,
   `joblib`).
2. Enables SPI and I²C in `/boot/firmware/config.txt`,
   sets I²C to 400 kHz fast mode, and disables Bluetooth (frees
   up kernel interrupts).
3. Appends `isolcpus=1,2,3 nohz_full=1,2,3 rcu_nocbs=1,2,3` to
   `/boot/firmware/cmdline.txt`. **All real-time guarantees in
   the system depend on this line.** Without it, cores 1–3 get
   load-balanced by the Linux scheduler and your latency budget
   evaporates.
4. Creates the `spi`, `i2c`, `gpio` groups (idempotent), drops
   a udev rule at `/etc/udev/rules.d/90-cpcu.rules`, and adds
   your real user to all three groups.
5. Creates `/opt/cpcu/{bin,scripts,models,test}` and
   `/var/log/cpcu/` and chowns them to your user — so you can
   `cmake --install` and tail logs without sudo.
6. Symlinks `/opt/cpcu/config.json → cpcu_v2/config/runtime.json`.

When it finishes, it prints **\*\*\* REBOOT REQUIRED \*\*\***.

### 1.5 Reboot

```bash
sudo reboot
```

This is the only sudo command you'll type today aside from the one
inside `setup_pi.sh`.

### 1.6 Verify the setup after reboot

SSH back in and check each one. **Every line below must produce
the expected output.** If one fails, do not proceed — re-run
`setup_pi.sh` and reboot first.

```bash
cat /sys/devices/system/cpu/isolated
```
Expected: `1-3`. Empty means `isolcpus` didn't apply — re-run
`setup_pi.sh` and reboot.

```bash
ls /dev/spidev0.0
```
Expected: the file exists. If `No such file`, run `sudo
raspi-config` → Interface Options → SPI → Enable, reboot.

```bash
ls /dev/i2c-1
```
Expected: the file exists. Same fix via raspi-config → I²C.

```bash
groups
```
Expected: includes `spi i2c gpio`. Log out and back in if missing
(group membership is set at login).

```bash
vcgencmd measure_clock arm
```
Expected: `frequency(48)=2800000000` (2.8 GHz overclock). If
lower, the active cooler is loose or the PSU is under-spec.

```bash
vcgencmd measure_temp
```
Expected: < 60 °C at idle. Above 70 °C means cooling is inadequate
and the Pi will throttle under load.

```bash
python3 -c "import numpy, scipy, joblib, sklearn; print('OK')"
```
Expected: `OK`. Any `ModuleNotFoundError` means a pip install failed.

```bash
i2cdetect -y 1
```
Expected: `40` shows up at row 4, column 0 (= I²C address 0x40).
The PCA9685 is wired and powered. If it shows `--` everywhere,
either PCA isn't powered, the I²C lines are wrong, or the pull-ups
are missing.

---

## Phase 2 — Build, install, RT capabilities, ML model

> **v2.6 shortcut:** All three of the build/install/grant-caps steps
> below are wrapped into a single command:
>
> ```bash
> ./scripts/launch.sh build
> ```
>
> This does cmake configure (lazy — only re-runs if needed) +
> build + install + grant-caps in one shot. Use this for every
> rebuild after the first one. The expanded steps below show what
> it does internally; first-time bring-up is identical either way.

### 2.1 Build the CPCU binaries

```bash
cd ~/prosthetic_hand/cpcu_v2
cmake -S . -B build
cmake --build build -j4
```

This produces 7+ binaries in `build/`. Verify:

```bash
ls build/cpcu_io build/cpcu_kernel build/cpcu_tui \
   build/test_codec build/safety_testbench \
   build/pca_testbench build/signal_testbench
```
All should exist.

### 2.2 Install to /opt/cpcu

```bash
cmake --install build
```

No sudo required — `setup_pi.sh` already chowned `/opt/cpcu` to
your user. This copies binaries to `/opt/cpcu/bin/` and python
scripts to `/opt/cpcu/scripts/`.

### 2.3 Grant RT capabilities to the binaries

```bash
./scripts/launch.sh grant-caps
```

You'll see one sudo password prompt as the script self-elevates.
This runs `setcap` to attach `CAP_SYS_NICE` (for `SCHED_FIFO`) and
`CAP_IPC_LOCK` (for `mlockall`) to `cpcu_io` and `cpcu_kernel` so
they take RT priority and lock pages without needing root at
runtime.

Verify:

```bash
getcap build/cpcu_kernel build/cpcu_io
```
Expected output:
```
build/cpcu_kernel = cap_ipc_lock,cap_sys_nice+ep
build/cpcu_io     = cap_ipc_lock,cap_sys_nice+ep
```

**You must re-run `grant-caps` after every clean rebuild.** A new
ELF file means a new inode, which means the caps are gone.

### 2.4 Deploy the ML model

The `.pkl` is not in the repo. Copy it from your laptop:

```bash
# On your laptop:
scp /path/to/hmi_svm_model_200hz.joblib pi@<pi-ip>:/opt/cpcu/models/
scp /path/to/hmi_scaler_200hz.joblib   pi@<pi-ip>:/opt/cpcu/models/
```

`/opt/cpcu/models/` is owned by you, so no sudo on either end.

> **Sidebar — running without the model**
>
> If you don't have a trained model yet (or you're verifying the
> system before training one), **`cpcu_dsp.py` has a built-in
> "feature-only mode"** that activates automatically when the model
> files are missing. The DSP loads, drains the ring buffer at full
> rate, computes features for the dashboard's spectrum view, and
> emits no classified gestures. The kernel runs cleanly.
>
> What this means for the bring-up:
>
> | Phase | Works without model? | Notes |
> |---|---|---|
> | 0–7 | ✅ Yes, fully | None of these touch classification |
> | 8.1 (servo calibration) | ✅ Yes | `pca_testbench` is direct I²C |
> | 8.2 (TUI runtime editor) | ✅ Yes | Kernel runs in feature-only mode |
> | 8.3 (configure.sh) | ✅ Yes | |
> | 9 (final launch) | ⚠️ Runs but no gestures | Servos hold neutral, dashboard renders, classification just shows "rest" forever |
> | TS-04 (gesture accuracy test sheet) | ❌ Defer | This is the only test that genuinely needs the model |
>
> So you can do **everything in this guide except meaningful Phase
> 9 demoing and TS-04** before having a model. When the kernel
> starts you'll see this in the log:
>
> ```
> [DSP] /opt/cpcu/models/hmi_svm_model_200hz.joblib or scaler missing -- feature-only mode
> ```
>
> That's the system telling you the situation, not a failure. When
> you later `scp` the real model files into place, send `kill -HUP
> $(pgrep -f cpcu_kernel)` (or restart the kernel) and the DSP
> picks them up — no rebuild, no reinstall.

---

## Phase 3 — Phase-1 software tests on the Pi (no hardware needed)

These tests don't need NRF, PCA, BSAU, or anything physical. They
verify the codec, the safety FSM, the smoother, the DSP pipeline,
the runtime config loader, the TUI editor, and the JSON serializer.
**If any of these fail, nothing downstream will work** — fix Phase
1 first.

```bash
chmod +x run_tests.sh
./run_tests.sh 1
```

> **v2.6 shortcut:**
>
> ```bash
> ./scripts/launch.sh test
> ```
>
> Equivalent to the above; just shorter to remember.

What you should see at the end:

```
RESULTS: 233 PASS, 0 FAIL
```

Breakdown:
- TB-C100..C106 codec: 7 PASS
- TB-SAF01..SAF09 safety FSM: 38 PASS
- Smoother: 28 PASS
- Runtime config loader: 43 PASS
- TUI editor: 24 PASS
- JSON serializer: 7 PASS
- DSP pipeline (Python): 86 PASS

If any group fails, do **not** proceed to Phase 4. Look at the
specific testbench output for the line that says `[FAIL]`.

---

## Phase 4 — BSAU standalone tests

These exercise the BSAU in isolation. You only need the Nucleo,
a USB cable, and a laptop with STM32CubeIDE installed.

### 4.1 Open the project in STM32CubeIDE

On your laptop:

1. Launch STM32CubeIDE (≥ 1.13.0).
2. `File → Import → General → Existing Projects into Workspace`.
3. Browse to the `bsau_v2/` directory of the cloned repo.
4. Hit `Finish`.

### 4.2 Phase 4.1 — Boot sanity (BSAU_MODE_DEBUG)

In `bsau_v2/Core/Inc/bsau_config.h`, confirm exactly one
`BSAU_MODE_*` macro is uncommented. For this test, pick:

```c
#define BSAU_MODE_DEBUG
```

Comment out any other `BSAU_MODE_*` defines.

In CubeIDE: **Build All** (Ctrl+B), then **Run** (green play). The
build flashes via the on-board ST-LINK over USB.

Open a serial terminal at **921600 8N1** on the Nucleo's virtual
COM port:

```bash
# Linux:
picocom -b 921600 /dev/ttyACM0

# macOS:
screen /dev/tty.usbmodem<serial> 921600

# Windows: PuTTY, COM<n>, 921600 baud, 8N1, no flow control.
```

Press the **NRST button** on the Nucleo. You should see:

```
[BSAU - APP ]: BSAU_Init        [RUN ]
[BSAU - APP ]: BSAU_Init        [OK  ] NVIC priority group verified
[BSAU - APP ]: TIM2_Start       [OK  ] 1 MHz free-running counter live
[BSAU - NRF ]: NRF_Init         [RUN ] ch=76 (POR wait 200ms done)
[BSAU - NRF ]: NRF_Init         [OK  ]
[BSAU - ADC ]: BSAU_ADC_Init    [RUN ] Calibrating ADC1...
[BSAU - ADC ]: BSAU_ADC_Init    [OK  ] Calibration complete
[BSAU - ADC ]: BSAU_ADC_Init    [OK  ] Pipeline running (TIM6 trig, DMA circ)
[BSAU - APP ]: BSAU_Init        [OK  ] Pipeline live
[BSAU - APP ]: BSAU_Run         [INFO] seq=67 batt=1945 lvl=0 retry=0 loss=15 ...
```

**PASS criteria:** every line ends with `[OK]`, no `[FAIL]`.

### 4.3 Phase 4.2 — Codec round-trip (BSAU_MODE_TEST_PKT_LOG)

Edit `bsau_config.h`:

```c
// #define BSAU_MODE_DEBUG
#define BSAU_MODE_TEST_PKT_LOG
```

Build, flash, reset. Watch the terminal:

```
[BSAU - TEST]: PacketVerify       [RUN ] === WL CODEC ROUND-TRIP START ===
[BSAU - TEST]: PKT_Ramp           [RUN ] ramp 0x100..0x600, seq=0x55, flags=0x03
...
[BSAU - TEST]: PacketVerify       [PASS] === WL CODEC: 23 PASS, 0 FAIL ===
```

**PASS:** `N PASS, 0 FAIL` line + LD3 starts a slow 1 Hz blink.

If this fails, do not flash RELEASE — the wire format is wrong.

### 4.4 Phase 4.3 — NRF self-test (BSAU_MODE_TEST_NRF_LOG)

Edit `bsau_config.h`:

```c
#define BSAU_MODE_TEST_NRF_LOG
```

Build, flash, reset. Watch the terminal — the firmware runs the
TB-105/TB-106 NRF24L01+ register and FIFO self-test.

**PASS:** `=== NRF SELF-TEST: 6 PASS, 0 FAIL ===` line.

The subsequent `TX_Ping` lines will all show `MAX_RT` because no
receiver is on yet — this is expected and validates the radio
state machine handles the no-ACK case correctly. Watch for
`[INFO] Periodic SPI/REGS/FIFO check at iter=20` and confirm it
logs success. That's the stress-loop health check.

### 4.5 Phase 4.4 — ADC pipeline (BSAU_MODE_TEST_ADC_CSV)

Optional but recommended if you have a function generator.

Edit `bsau_config.h`:

```c
#define BSAU_MODE_TEST_ADC_CSV
```

Build, flash, reset. The firmware streams 9 channels of raw ADC
counts as a binary CSV stream at 921600 baud. Visualise with
SerialPlot:

1. Install SerialPlot from `https://github.com/hyOzd/serialplot`.
2. File → Settings → open serial port at 921600 8N1.
3. Data format: Binary, 9 channels of int16 LE, the sync bytes
   used in `bsau_test.c`. Sample rate 2000 Hz.
4. Hit Connect.

**Expected:** 9 rolling traces (8 EMG + 1 battery). Touching PA0
with your fingertip should make ch 0 — and only ch 0 — spike.

If all 8 channels move together, the ADC scan has collapsed to one
input — check the CubeMX ADC rank sequence.

### 4.6 Phase 4.5 — Goertzel DFT (BSAU_MODE_TEST_DFT_LOG)

Edit `bsau_config.h`:

```c
#define BSAU_MODE_TEST_DFT_LOG
```

Build, flash. Feed PA0 a **200 Hz sine, 0.5 V_pp, 1.65 V DC offset**
from the function generator. Watch the UART:

```
[BSAU - TEST]: DFT_ch0   [INFO] peak=200Hz mag=1025
```

**PASS:** reported peak within ±3.9 Hz of the input. If the report
is 180 or 220 Hz, the ADC isn't running at 2000 Hz — check
`tim.c`'s TIM6 prescaler.

### 4.7 Switch BSAU to RELEASE for the rest of the guide

Edit `bsau_config.h`:

```c
#define BSAU_MODE_RELEASE
```

Build, flash. The BSAU now transmits real packets at 1000 pkt/s.

---

## Phase 5 — CPCU hardware verification (Pi peripherals)

Back on the Pi:

```bash
cd ~/prosthetic_hand/cpcu_v2
./run_tests.sh 3
```

Phase 3 tests:

- Core isolation (`/sys/devices/system/cpu/isolated == "1-3"`)
- `/dev/spidev0.0` exists
- `/dev/i2c-1` exists
- PCA9685 detected at I²C address 0x40
- PCA9685 prescaler register read-back works
- pca_testbench dry-run smoke test (init + neutral-position write)
- CPU frequency ≥ 2.8 GHz
- CPU temperature < 80 °C

**PASS:** all rows show `[PASS]`. Any `[FAIL]` here points at a
wiring problem from Phase 0 or a missed step in Phase 1.

### 5.1 Servo motion sanity check (interactive)

```bash
./run_tests.sh pca
```

This launches `pca_testbench`. The TUI lists the 6 servos with
their current commanded pulse and the corresponding angle. Verify
that:

1. All 6 servos respond when you select them with `↑`/`↓` and jog
   with `←`/`→` (10 µs steps) or `PgUp`/`PgDn` (50 µs steps).
2. Pressing `m` jogs to the configured MIN, `M` to MAX. The arm
   moves in a controlled sweep.
3. Pressing `q` exits. The servos hold the last commanded
   position; the kernel isn't running so they just stay there.

If a servo doesn't move at all: cable, channel mapping (verify
against the table in 0.3.4), or PSU output current limit.

### 5.2 Signal testbench dry-run

```bash
./run_tests.sh signal-demo
```

This brings up the signal-integrity TUI fed with synthetic 100 Hz
sine waves on all 8 channels. No hardware involved. **PASS:** the
TUI renders cleanly, all 8 channels show the sine wave, Goertzel
reports 100 Hz on each.

Press `q` to quit.

---

## Phase 6 — IPC validation (kernel running, no peripherals stress)

```bash
./run_tests.sh 1 2
```

This re-runs Phase 1 and adds Phase 2 (IPC bridge offset
validation). Phase 2 needs `cpcu_kernel` running so
`/dev/shm/cpcu_ipc` exists; the test harness starts the kernel
itself, runs the Python IPC bridge tests, then stops it.

**PASS:** all Phase 1 233 PASS + the `TB-IPC` row passes.

---

## Phase 7 — Live integration test (BSAU + CPCU together)

This is the first time both boards are talking. It's the
definitive end-to-end signal-integrity test.

### 7.1 Setup

1. BSAU is in **BSAU_MODE_RELEASE** (you switched to it at the
   end of Phase 4). Power it via USB; the user LED LD3 lights at
   1 kHz (looks like steady dim glow — that's normal at 1 kHz).
2. Connect a function generator to BSAU's **PA0** with a 100 Hz
   sine, ~0.6 V amplitude, **1.65 V DC offset**. (Use a T-piece
   or a star-junction breadboard wire to feed the same signal to
   PA1–PA7 if you want all 8 channels exercised.)
3. CPCU side already has the PCA9685 + servos powered (Phase 0.5).

### 7.2 Launch the live signal testbench

```bash
cd ~/prosthetic_hand/cpcu_v2
./run_tests.sh signal
```

This brings up `cpcu_kernel` (which spawns `cpcu_io`) plus the
signal-integrity TUI in one go. The TUI plots the 8-channel raw
ADC data straight off the IPC ring buffer — no DSP, no filtering.

Press **TAB** for the all-channel view (Vpp, dominant frequency,
DC offset, SNR per channel).

**PASS criteria, per channel:**

- Clean sinusoid visible in the rolling plot.
- Dominant frequency = 100 Hz (Goertzel report).
- Vpp ≈ 1.2 V (twice the function generator's amplitude).
- SNR > 20 dB.
- DC offset ≈ 1.65 V.

The radio status row at the top should show:

- packet rate ≈ 1000 pkt/s
- seq gaps < 10 over 1 minute
- loss < 0.1 %

Press `q` to quit.

If you see no packets at all: BSAU is silent, NRF link is broken,
or the channel/address don't match. Verify the BSAU test from
Phase 4.4 still passes; on the CPCU side, look at
`/var/log/cpcu/cpcu.log` for `NRF_Init` errors.

If you see packets but the waveforms look wrong: signal-chain
problem upstream of the radio (check Phase 4.5 first).

### 7.3 Safety FSM smoke test

While the system is still running from 7.2, in another SSH session:

```bash
sudo systemctl stop cpcu 2>/dev/null  # in case you started it earlier; ignore if no service
# Now the test harness's kernel is the only one running.
```

Power off the BSAU (unplug USB). Watch the signal_testbench TUI:

1. Within ~750 ms: state transitions from `RUNNING` → `DEGRADED`.
2. Servos snap to neutral (1500 µs) — **you'll hear them move
   briefly**.
3. After ~1500 ms total: state transitions to `SAFE`; PCA outputs
   are turned off and servos go limp.
4. Plug BSAU back in, the firmware re-inits, and within ~1 second
   you should see state recover to `RUNNING` automatically.

If transitions don't happen, the watchdog isn't firing — check
`SAFETY_RADIO_TIMEOUT_MS` in `cpcu_safety.h` and the kernel log
for safety-FSM messages.

Press `q` to quit the testbench (or just close the SSH session).

---

## Phase 8 — Calibrations

Three layers, in order of how often you'll touch them:

1. **Per-servo limits and bias** (most common — every time the
   mechanical assembly changes, or you swap a servo).
2. **Runtime tunables** (DSP thresholds, gesture velocities,
   smoother knobs — you'll touch these during a tuning session).
3. **Compile-time safety thresholds** (radio timeout, vbat
   cutoffs, thermal limits — these change rarely, behind a code
   review).

### 8.1 Per-servo limits via pca_testbench round-trip

`pca_testbench` is the bench tool that lets you physically jog
each servo and write the discovered values back to
`runtime.json`. The kernel must be **stopped** so the bench tool
can own I²C exclusively.

```bash
# Stop the kernel if it's running:
./scripts/launch.sh stop 2>/dev/null         # if started via launch.sh tmux
# or
sudo systemctl stop cpcu 2>/dev/null         # if installed as a service

# Launch the bench tool in config-loaded mode:
./build/pca_testbench --config config/runtime.json
```

Note: `pca_testbench` is launched with `sudo` only if `/dev/i2c-1`
isn't reachable through the `i2c` group — but `setup_pi.sh` set up
the group membership, so a plain `./build/pca_testbench` works.

Inside the TUI:

| Key | Action |
|---|---|
| `↑`/`↓` | Select servo |
| `←`/`→` | Jog ±10 µs |
| `PgUp` / `PgDn` | Jog ±50 µs |
| `m` / `M` | Jog to current MIN / MAX |
| `[` | Set the current jog AS MIN for the selected servo |
| `]` | Set the current jog AS MAX |
| `b` | Set the current deviation from neutral AS BIAS |
| `B` | Clear bias for the selected servo |
| `v` | Cycle smoother VELOCITY preset |
| `a` | Cycle smoother ACCELERATION preset |
| `d` | Cycle smoother DEADBAND preset |
| `s` | Save changes to `runtime.json` |
| `q` | Quit (without saving — confirms first if dirty) |

**Workflow per servo:**

1. Select the servo with `↑`/`↓`.
2. Jog into the safe minimum mechanical position (just before the
   servo whines or stalls). Press `[`. Watch the MIN value on
   screen update.
3. Jog into the safe maximum. Press `]`.
4. Jog into the centred neutral position. Press `b` to set bias.
5. Repeat for all 6 servos.
6. Press `s` to save to `runtime.json`. The file is updated using
   `CFG_PatchFile` — only the fields you changed are rewritten;
   the rest of the JSON (including `gesture_velocity`, which the
   C parser doesn't know about) is preserved byte-for-byte.

When the kernel is restarted, it'll read the new values and
republish via the IPC seqlock; cpcu_io applies them on the next
servo tick.

### 8.2 Runtime tunables (DSP thresholds, smoother, grip levels)

Two ways:

**Option A — TUI editor (preferred when the system is running).**

```bash
./scripts/launch.sh tui          # bring up kernel + TUI in tmux
```

Inside the TUI:

1. Press `7` to switch to the CONFIG page.
2. Press `e` to enter edit mode. The arm parks at neutral via
   the v2.3.4 handshake (banner says "PARKING ARM..." then
   "EDITING — arm parked").
3. Use arrow keys to navigate the 13 editable runtime fields:
   per-servo arrays (min/max/bias, smoother vel/accel/deadband)
   and 7 scalar fields (DSP thresholds, grip levels).
4. Press `Enter` on a cell to begin numeric entry; type the new
   value; `Enter` again to commit. `Esc` cancels in-flight entry.
5. `r` reverts dirty cells to disk values.
6. `Ctrl+S` saves dirty cells to `runtime.json` and SIGHUPs the
   kernel for live reload (~20 ms latency).
7. Press `e` again to exit edit mode and resume normal operation.

If `Ctrl+S` doesn't work cleanly (rare — probably means another
tool patched the file in between), the editor preserves your
drafts in memory; re-enter edit mode and try again.

**Option B — edit the JSON file directly (when the system is up
and you don't want to use the TUI).**

```bash
# In another shell, while cpcu_kernel is running:
$EDITOR cpcu_v2/config/runtime.json
kill -HUP $(pgrep -f cpcu_kernel)        # reload, ~20 ms
tail -f /var/log/cpcu/log_KERN.csv       # confirm: "runtime config loaded ..."
```

### 8.3 Compile-time safety thresholds (rare)

These are `#define`s in `cpcu_safety.h` (radio timeout, vbat
thresholds, thermal limits, ring-overflow limit) plus the BSAU's
NRF channel in `bsau_app.h`. Changing any of them requires a
rebuild.

```bash
cd ~/prosthetic_hand/cpcu_v2

# Inspect:
./configure.sh --show          # all values
./configure.sh --diff          # only modifications

# Set one or more:
./configure.sh --radio-timeout 1000
./configure.sh --vbat-low 3.1 --thermal-warn 70

# Restore defaults:
./configure.sh --reset

# Interactive walkthrough:
./configure.sh
```

Each edit prints a **REBUILD REQUIRED** banner. Apply with:

```bash
cmake --build build
cmake --install build
./scripts/launch.sh grant-caps    # caps were lost when the binary was rewritten
```

If the change touched the BSAU channel: re-flash the BSAU too
(open in CubeIDE, build, run).

`runtime.json` and `configure.sh` are intentionally separate. The
former is for tuning that you'd revert on a whim; the latter is
for safety envelope changes that warrant a code review.

---

## Phase 9 — Final launch (the actual operating mode)

You have three runtime modes. Pick the one that matches what
you're doing right now.

> **Pre-flight tip (v2.6):** Run this first to confirm the
> environment is ready:
>
> ```bash
> ./scripts/launch.sh check
> ```
>
> This verifies binaries are installed, capabilities are applied,
> isolcpus is set, SPI/I²C are up, the model is present (warns if
> not — feature-only mode is supported), config symlink exists,
> and group membership is correct. Exits 0 if you can launch, 1
> if there's a fatal-class issue. Doesn't actually start anything.
> Useful right before a demo to avoid surprises.

### 9.1 Interactive mode — tmux with kernel + TUI

Best for development, demos, and debugging. Everything visible
in one terminal.

```bash
cd ~/prosthetic_hand/cpcu_v2
./scripts/launch.sh tui
```

This creates a tmux session with two windows:

- Window 0 (`KERNEL`): `cpcu_kernel --log` + the spawned
  `cpcu_io` and `cpcu_dsp.py`. Their stdout is here.
- Window 1 (`TUI`): the ncurses dashboard with 7 pages.

Tmux keys (default prefix is `Ctrl-b`):

| Key sequence | Action |
|---|---|
| `Ctrl-b 0` | Switch to KERNEL window |
| `Ctrl-b 1` | Switch to TUI window |
| `Ctrl-b w` | Interactive window picker |
| `Ctrl-b d` | Detach (everything keeps running in background) |
| `Ctrl-b ?` | tmux help |

TUI pages (press the digit key):

| Page | What |
|---|---|
| `1` | Overview — system state, packet rate, loss, uptime |
| `2` | Radio / IO — NRF + I²C + IPC ring deep-dive |
| `3` | DSP / AI — current gesture, confidence, inference time |
| `4` | Waves — 8-channel rolling raw + filtered + RMS plot |
| `5` | Health — 10-row traffic-light system rollup |
| `6` | Dataset — interactive CSV capture (RAW or FILTERED) |
| `7` | Config — spec sheet + runtime editor entry (`e`) |

Press `q` inside the TUI to quit the TUI only — kernel keeps running.

### 9.2 Re-attach later

After detaching (`Ctrl-b d`):

```bash
./scripts/launch.sh attach          # come back to the running session
./scripts/launch.sh stop            # kill the session and all children
```

### 9.3 Web dashboard (multi-viewer, browser-based)

Useful when you have committee members watching from their own
laptops. Read-only, multi-viewer, no shared-IP-needed:

```bash
./scripts/launch.sh ws              # foreground; bind 0.0.0.0:8765
```

Then on any laptop on the same LAN:

```
http://<pi-ip>:8765
```

Or if you set the Pi's hostname to `cpcu` (`sudo hostnamectl
set-hostname cpcu` once) and Avahi is up:

```
http://cpcu.local:8765
```

The dashboard has 4 tabs (Overview, Waves, Spectrum, Tools). It
streams from `/dev/shm/cpcu_ipc` over WebSocket, so it can run
alongside the TUI without contention.

**Warning:** the WS bridge binds 0.0.0.0 by default — anyone on
the LAN can view. The script prints a loud warning at startup.

### 9.4 Production mode — systemd auto-start

For when the system should come up at boot without you typing
anything:

```bash
./scripts/launch.sh install-service
```

You'll see one sudo prompt. The script:

1. Generates `/etc/systemd/system/cpcu.service` with
   `User=<your-user>`, `AmbientCapabilities=CAP_SYS_NICE
   CAP_IPC_LOCK`.
2. Re-applies `setcap` to the installed binaries.
3. `systemctl daemon-reload` + `enable` — the service starts at
   the next boot.

To start / stop / inspect:

```bash
sudo systemctl start cpcu
sudo systemctl stop cpcu
sudo systemctl status cpcu
journalctl -u cpcu -f                   # live tail of kernel logs
```

For the dashboard alongside systemd:

```bash
./scripts/launch.sh install-ws-service  # cpcu_ws service
sudo systemctl start cpcu_ws
```

---

## Phase 10 — Stop, recover, iterate

### 10.1 Stop everything

If launched via tmux (`./scripts/launch.sh tui`):

```bash
./scripts/launch.sh stop
```

If installed as a service:

```bash
sudo systemctl stop cpcu
sudo systemctl stop cpcu_ws         # if installed
```

If something is hung:

```bash
pkill -9 cpcu_kernel cpcu_io cpcu_tui cpcu_ws
pkill -9 -f cpcu_dsp.py
rm -f /dev/shm/cpcu_ipc /var/lock/cpcu.lock
```

(The lock and shm region get re-created on next launch.)

### 10.2 Reload runtime config without restarting

```bash
$EDITOR cpcu_v2/config/runtime.json
kill -HUP $(pgrep -f cpcu_kernel)
```

The kernel re-parses, validates, and republishes via the IPC
seqlock within ~20 ms. cpcu_io's smoother re-applies on the
`config_seq` change automatically.

If the new config is invalid (out-of-range value, missing required
key, etc.), the kernel **refuses to apply it** and keeps the old
config running. Watch for the `[ERROR] [KERN] config load failed`
line in `/var/log/cpcu/log_KERN.csv` — it'll point at the offending
key.

### 10.3 Iterate during a calibration session

Typical loop:

```
1. Enter edit mode in TUI (press 'e' on CONFIG page) — arm parks.
2. Adjust one or two cells.
3. Ctrl+S — kernel reloads, smoother re-applies.
4. Press 'e' again — exit edit mode, system resumes.
5. Wear the prosthetic, test the gesture you tuned.
6. Note what feels off; go back to step 1.
```

Each iteration is ~5 seconds of arm-parked time.

### 10.4 Logs

```bash
# Live tail of everything:
journalctl -u cpcu -f

# Persistent CSV logs:
ls /var/log/cpcu/
tail -f /var/log/cpcu/log_KERN.csv     # kernel + supervisor
tail -f /var/log/cpcu/log_IO.csv       # io loop, ring stats
tail -f /var/log/cpcu/log_DSP.csv      # python DSP + ML
```

### 10.5 Recovery checklist if something goes wrong mid-demo

1. **Servos go limp** → safety FSM tripped to SAFE. Look at the
   TUI's Health page (key `5`) for the [FAULT] row and the "why"
   line. Most common: BSAU power loss. Plug BSAU back in, wait
   for `RUNNING` recovery (~1 s).
2. **Gripper pinned + counter incrementing** → soft-grip stall
   watchdog firing. Open the gripper manually, the system
   recovers automatically. If it keeps happening on a soft
   object, raise `grip_firm_us` via the TUI editor.
3. **Dashboard frozen / can't connect** → tmux session ended,
   or `cpcu_ws` died. `./scripts/launch.sh attach` to check; if
   ended, `./scripts/launch.sh tui` again.
4. **Servos jittering at hold** → smoother deadband too tight
   or velocity too high. TUI editor → CONFIG page → adjust
   `smoother_deadband_us` for the offending servo.

---

## Quick-reference: order of commands for a clean bring-up

```bash
# === Phase 1: one-time Pi setup ===
ssh <user>@<pi>.local
sudo apt update && sudo apt -y full-upgrade
sudo apt install -y git
sudo reboot
ssh <user>@<pi>.local
git clone <repo-url> ~/prosthetic_hand
cd ~/prosthetic_hand/cpcu_v2
./setup_pi.sh
sudo reboot
ssh <user>@<pi>.local

# === Phase 1.6: verify after reboot ===
cat /sys/devices/system/cpu/isolated      # → "1-3"
ls /dev/spidev0.0 /dev/i2c-1
groups                                    # includes spi i2c gpio
vcgencmd measure_clock arm                # ≥ 2.8 GHz
i2cdetect -y 1                            # PCA at 0x40

# === Phase 2: build, install, RT caps, ML model ===
cd ~/prosthetic_hand/cpcu_v2
./scripts/launch.sh build                 # v2.6: configure + build + install + grant-caps
# (laptop side:)  scp hmi_svm_model_200hz.joblib hmi_scaler_200hz.joblib pi@<pi>:/opt/cpcu/models/
# (model is OPTIONAL — DSP runs in feature-only mode without it)

# === Phase 3: software-only tests ===
./scripts/launch.sh test                  # v2.6 shortcut for: ./run_tests.sh 1
                                          # → 233 PASS

# === Phase 4: BSAU standalone (laptop, CubeIDE) ===
# Edit bsau_v2/Core/Inc/bsau_config.h, pick a profile, build, flash, observe UART.
# Sequence: DEBUG → TEST_PKT_LOG → TEST_NRF_LOG → TEST_ADC_CSV → TEST_DFT_LOG → RELEASE.

# === Phase 5: CPCU hardware ===
./run_tests.sh 3
./run_tests.sh pca                        # interactive servo motion check; q to exit
./run_tests.sh signal-demo                # interactive TUI dry-run; q to exit

# === Phase 6: IPC validation ===
./run_tests.sh 1 2

# === Phase 7: live integration (BSAU + CPCU) ===
# BSAU on USB in BSAU_MODE_RELEASE, function gen on PA0.
./run_tests.sh signal                     # check waveform + radio + safety; q to exit

# === Phase 8: calibrations ===
./build/pca_testbench --config config/runtime.json   # per-servo limits + bias
./scripts/launch.sh tui                              # then 'e' on CONFIG page for runtime editor
./configure.sh --diff                                # inspect compile-time tunables
# (rebuild if you changed configure.sh values)

# === Phase 9: final launch ===
./scripts/launch.sh check                 # v2.6: pre-flight (optional but recommended)
./scripts/launch.sh tui                   # interactive (kernel + TUI in tmux)
# OR
./scripts/launch.sh ws                    # browser dashboard at :8765
# OR
./scripts/launch.sh install-service       # systemd, auto-start at boot
sudo systemctl start cpcu

# === Phase 10: stop ===
./scripts/launch.sh stop                  # if started via tmux
sudo systemctl stop cpcu                  # if started via systemd
```

That is the full sequence. Start at Phase 0, end at Phase 9, and
the system is live.
