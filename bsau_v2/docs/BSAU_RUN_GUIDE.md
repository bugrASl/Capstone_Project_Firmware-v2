# BSAU Run Guide — v2.4 (NRF non-fatal init)

**Author:** bugrASl
**Date:** April 2026
**Audience:** anyone on the team — from first-time new members to the original
authors. No prior BSAU knowledge assumed. Every step explains both *what* and
*why*.

This guide takes you from a fresh NUCLEO-L432KC board and a clean IDE to a
running BSAU transmitter. Every command is copy-pasteable. Every step has a
verification check and a troubleshooting note.

If you are completely new to the project, start by skimming `BSAU_ARCHITECTURE.md`
§1 (System Overview) — come back here for the hands-on steps.

---

## Table of Contents

0.  [What is BSAU?](#0-what-is-bsau)
1.  [Prerequisites](#1-prerequisites)
2.  [Fresh board setup (one-time)](#2-fresh-board-setup-one-time)
3.  [Build modes — which one do I want?](#3-build-modes--which-one-do-i-want)
4.  [Build and flash](#4-build-and-flash)
5.  [Run in RELEASE / DEBUG mode](#5-run-in-release--debug-mode)
6.  [Run in DATASET mode (dual-path collection)](#6-run-in-dataset-mode-dual-path-collection)
7.  [Monitor the UART stream](#7-monitor-the-uart-stream)
8.  [Troubleshooting](#8-troubleshooting)
9.  [Command reference cheatsheet](#9-command-reference-cheatsheet)

---

## 0. What is BSAU?

**BSAU** = **Bio-Signal Acquisition Unit**. It is the small battery-powered
board the user wears on the upper arm. It:

1. Reads 8 EMG electrode channels (plus a battery monitor) through the
   STM32L432KC's 12-bit ADC at 2 kHz per channel, with 32× hardware
   oversampling for ~14.5 ENOB.
2. Packs 2 consecutive scans into a 32-byte wireless packet.
3. Fires the packet over a 2 Mbps NRF24L01+ link to the CPCU (Raspberry Pi
   inside the prosthetic forearm) at 1000 packets/s.
4. In DATASET mode: **also** streams the raw samples as plain CSV over
   the USB-serial debug UART at 921600 baud — same data the radio sends,
   pre-filtering, so the DSP/AI team can train against ground truth.

A block view:

```
                    ┌─────────────────────────────────────┐
                    │             BSAU board              │
                    │                                     │
   8× EMG electrodes│  PA0-PA7 ───► ADC+DMA (2 kHz × 32×) │
     ─────────────▶ │                      │              │
                    │                      ▼              │
                    │               WL_Packet build       │
                    │                │           │        │
    2.4 GHz radio   │                ▼           ▼        │
    ◄────────────── │              NRF TX      UART CSV   │
                    │              (always)    (DATASET   │
                    │                           mode only)│
                    └─────────────────────────────────────┘
                          ▲                       ▲
                          │                       │
                     to CPCU                to laptop (serial)
```

The two streams are produced *simultaneously*; you never have to pick one.

---

## 1. Prerequisites

### Hardware

-   **NUCLEO-L432KC** development board (the Arduino Nano-sized one)
-   **NRF24L01+** module wired as described in `BSAU_ARCHITECTURE.md` §2.2
-   USB-A-to-Micro-B cable (the NUCLEO's built-in ST-LINK provides both power
    and UART)
-   For radio testing: a matching receiver (CPCU, or another NUCLEO running
    an RX sketch)

### Software

-   **STM32CubeIDE** ≥ 1.13.0 (free from st.com)
    -   Bundles the ARM GCC toolchain, ST-LINK GDB server, and CubeMX.
    -   If you prefer the command line: `arm-none-eabi-gcc` and
        `st-flash`/`openocd` from your package manager.
-   **A serial terminal** that can handle 921600 8N1. Known-good choices:
    -   `minicom -D /dev/ttyACM0 -b 921600` (Linux)
    -   `screen /dev/ttyACM0 921600` (Linux / macOS)
    -   PuTTY (Windows) — make sure to set the baud explicitly; the default
        is 9600 and will show nothing but garbage
    -   SerialPlot for binary waveform plotting (TEST_ADC_CSV mode)
-   **Python 3.10+** (only for the dataset collector script) with
    `pyserial`:
    ```bash
    pip install pyserial
    ```

---

## 2. Fresh board setup (one-time)

1.  Connect the NUCLEO to your laptop via USB. Two things happen:
    -   ST-LINK enumerates as a mass-storage device (`NODE_L432KC`)
    -   A virtual COM port appears:
        -   Linux: `/dev/ttyACM0` (run `dmesg | tail` to confirm)
        -   macOS: `/dev/tty.usbmodem*`
        -   Windows: a new COM port in Device Manager (say `COM5`)
2.  Open STM32CubeIDE and import the project:
    `File → Import → Existing Projects into Workspace`, point at the
    directory holding `main.c` / `bsau_app.c` etc.
3.  The first build will pull in the HAL and CMSIS sources from the ST
    package the project was generated against. If you get "unresolved
    includes" errors, right-click the project → *Properties → C/C++ General
    → Paths and Symbols* and confirm `Drivers/STM32L4xx_HAL_Driver/Inc`
    and `Drivers/CMSIS/Device/ST/STM32L4xx/Include` are listed.

**Verification:** `Project → Build All` should finish with zero errors.
Warnings about `assert_param` are harmless (those assertions are disabled
unless `USE_FULL_ASSERT` is defined in `stm32l4xx_hal_conf.h`).

---

## 3. Build modes — which one do I want?

`bsau_config.h` has exactly one `#define BSAU_MODE_*` active. Pick based
on what you're doing:

| Your goal | Mode to use | What it does |
|-----------|-------------|--------------|
| Ship a board to the user | `BSAU_MODE_RELEASE` | Transmits to CPCU, zero debug overhead, battery max |
| Developing with live `printf` | `BSAU_MODE_DEBUG` | Same as RELEASE + periodic LOG lines + decimated CSV |
| Verify ADC + filter design on the bench | `BSAU_MODE_TEST_ADC_CSV` | Binary stream for SerialPlot, radio OFF |
| Verify the 12-bit codec round-trips correctly | `BSAU_MODE_TEST_PKT_LOG` | Runs codec tests once, logs PASS/FAIL, idles |
| Measure ADC dropped-packet rate under UART load | `BSAU_MODE_TEST_CSV` | ASCII CSV with a drop counter column, radio OFF |
| Feed a sine into ch0 and confirm the frequency | `BSAU_MODE_TEST_DFT_LOG` | Goertzel DFT, 5 bins, radio OFF |
| Self-test the NRF and watch link stats | `BSAU_MODE_TEST_NRF_LOG` | Full self-test suite + TX stress loop |
| **Collect a training dataset** | **`BSAU_MODE_DATASET`** | **Normal TX to CPCU AND channels-only UART CSV at 921600 baud** |

All test modes that disable the radio say so explicitly in the table in
`BSAU_ARCHITECTURE.md` §12. Don't try to test radio health in a mode
that has the radio turned off.

Open `bsau_config.h`, comment out whatever's active, uncomment the mode
you want, **save and rebuild**. The compiler will emit a clear error
like `#error "Define exactly one BSAU_MODE_*."` if you leave two uncommented
or none uncommented.

---

## 4. Build and flash

From STM32CubeIDE:

1.  Save `bsau_config.h`.
2.  `Project → Build All` (or Ctrl+B). Expected: zero errors.
3.  Click the green bug/play icon (`Run → Debug`) the first time to push
    the binary; subsequent flashes can use the plain "play" button.
4.  When the debugger is attached, hit "Resume" (F8) to start the firmware.

From the command line (if you've built your own toolchain setup):

```bash
# One-off: build
make clean && make

# Flash (OpenOCD target for L432KC)
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
        -c "program build/bsau.elf verify reset exit"
```

**Verification:** the blue LD3 LED (PA11) starts blinking at 1 Hz once the
firmware is running. No blinking = the board didn't boot; hit the black
reset button and check your UART terminal for a panic line.

---

## 5. Run in RELEASE / DEBUG mode

In RELEASE the UART is silent. You confirm the board is running by
whether the CPCU sees packets (check `cpcu_tui` page 2, pkt_rate should be
~1000 Hz).

In DEBUG the UART prints a structured status line every ~330 ms:

```
[BSAU - APP ]: BSAU_Run           [INFO] seq=234 batt=1945 lvl=0 retry=0 loss=0 ok=658 lost=0 drop=0
[BSAU - APP ]: BSAU_Run           [INFO] seq=207 batt=1946 lvl=0 retry=0 loss=0 ok=987 lost=0 drop=0
```

What each field means:

| Field | Meaning | Healthy value |
|-------|---------|---------------|
| `seq` | Current packet sequence (wraps every 256) | Incrementing |
| `batt` | 12-bit battery ADC reading | > 1861 (≈ 3.0 V) |
| `lvl` | 2-bit battery level: 0 OK, 1 LOW, 2 CRIT | 0 |
| `retry` | NRF ARC_CNT from previous TX | 0 most of the time |
| `loss` | NRF PLOS_CNT from previous TX | 0 most of the time |
| `ok` | Packets transmitted (cumulative) | Monotonic |
| `lost` | Packets that hit MAX_RT (cumulative) | Stays 0 indoors within 5 m |
| `drop` | ADC ISR overruns (cumulative) | **Must stay 0**. Non-zero = bug. |

If any of these looks wrong, §8 Troubleshooting has the answers.

---

## 6. Run in DATASET mode (dual-path collection)

This is the mode the DSP/AI team uses when building training sets.

### 6.1 Prepare

1.  Power the board via the USB cable (the UART we want is the same port).
2.  Boot the CPCU (`systemctl start cpcu` on the Pi, or manually run
    `cpcu_kernel`). See `CPCU_RUN_GUIDE.md` §6 if it's your first time.
3.  Ensure the CPCU is actually receiving packets — open `cpcu_tui` on
    the Pi and page 2 should show `pkt_rate ≈ 1000 Hz`. If it isn't,
    fix that first; DATASET mode doesn't change anything about the radio
    link.

### 6.2 Flash BSAU with DATASET mode

```
// bsau_config.h
#define BSAU_MODE_DATASET
```

Build, flash, reset. No visible behaviour change on the board itself.

### 6.3 Collect on the PC side (BSAU UART)

In one terminal, run the collector:

```bash
python3 bsau_dataset_collector.py --port /dev/ttyACM0 --label REST
```

It will:

1.  Scan `./datasets/` and pick the next available `REST_N.csv`.
2.  Open the UART and discard any partial first line so you get a clean
    start.
3.  Start streaming sample lines to the file immediately. Every second
    it prints a one-line status to stderr:
    `[collector] lines=14203 rate=1000.3/s bad=0`
4.  Press **Ctrl+C** when the contraction is done. The script flushes
    the file, closes the port, prints a final summary (duration, line
    count, bad-line count, throughput) and exits cleanly.

Optionally add `--live` to get an 8-channel matplotlib rolling scope
(requires `pip install matplotlib`; slows capture ~5-10 %).

Change `--label` between runs. The script always picks the next number,
so `REST_0`, `REST_1`, ..., `H_OPN_0`, etc., never overwriting.

### 6.4 Collect on the CPCU side (TUI)

In a second terminal (SSH to the Pi):

```bash
cpcu_tui
```

Press `7` to jump to the DATASET page. What you see:

```
State: IDLE              Label: [0] REST              Mode: RAW + filtered
Samples: 0               Elapsed: 0.000s              Gaps: 0    Missed: 0
Out dir: ./datasets/
File: (none yet)

Labels (←/→ to cycle, s:start/stop, t:toggle raw, r:cancel):
 REST   H.SLO   H.HRD   H.OPN   A.BND<   A.BND=   A.BND>   A.SLO   A.FST   BICEP

Live waveforms (raw ADC, 512 samples @ 2 kHz = 256 ms window):
 ch0  0.12Vpp 0.045Vrms    ch4  0.08Vpp 0.029Vrms
   [waveform]                   [waveform]
 ch1  0.11Vpp 0.042Vrms    ch5  0.09Vpp 0.033Vrms
   ...
```

Use `←` / `→` to cycle labels, press `t` to pick RAW-only or FILTERED-only
output (default filtered, which matches what `cpcu_dsp.py` does in prod).

Press `s` (or SPACE) to start. The state turns red with `● COLLECTING`
and `Samples:` starts counting. Press `s` again to stop and save. To
throw away a bad take, press `r` while collecting — the file is deleted.

Files land in `./datasets/` under the working directory where you
launched `cpcu_tui`. Filenames match the BSAU-side collector
(`REST_0.csv`, `H_OPN_2.csv`, ...) but the content is different: BSAU
has raw 12-bit ints, CPCU has either raw 12-bit ints (`t` toggled on)
or filtered volts.

### 6.5 Cross-referencing the two files

Same contraction → two files:

-   `./datasets/REST_0.csv` on the PC (BSAU UART, 1000 rows/s, 8 int cols)
-   `./datasets/REST_0.csv` on the Pi (CPCU TUI, 2000 rows/s, 8 float cols
    if filtered / 8 int cols if raw)

The CPCU file is 2× the length of the BSAU file in rows — it includes
both scans of every packet. Any scan that appears in BSAU but not in
CPCU was lost over the air; the `Gaps:` counter on the TUI dataset page
tells you how many over the whole capture.

---

## 7. Monitor the UART stream

Even without the collector script, raw UART inspection is useful for
debugging. Examples assume Linux; adjust `/dev/ttyACM0` to your platform.

### Raw byte check

```bash
stty -F /dev/ttyACM0 921600 cs8 -cstopb -parenb -icanon -echo
head -c 500 /dev/ttyACM0
```

You should see CSV lines with 7 commas and a CRLF between them, e.g.
`2048,2049,2047,2048,2050,2047,2051,2048`.

### Sample rate check

```bash
timeout 5 cat /dev/ttyACM0 | wc -l
# Expect ~5000 lines (1000 lines/s × 5 s) in DATASET mode
```

### Noise floor check (electrodes disconnected)

Connect the collector with `--label NOISE`, record 10 seconds. Open the
file with a hex editor and skim column by column — every column should
hover in a narrow band around mid-scale (~2048 ± 20). A column that
wanders far from 2048 has a floating input pin — stop and fix before
gathering real data.

---

## 8. Troubleshooting

### Board doesn't enumerate / no `/dev/ttyACM0`

-   Bad USB cable. Swap it. Micro-B cables that charge but don't do data
    are common.
-   ST-LINK firmware outdated. In CubeIDE: `Help → ST-LINK Upgrade`.
-   Wrong USB port — some hubs don't pass through the ST-LINK
    enumeration. Plug directly into the laptop.

### No output on serial

-   Baud is wrong. **DATASET runs at 921600**, not 115200.
-   Stop bits / parity mismatched. Always 8N1.
-   You're in a mode with LOG *and* CSV both off (e.g. RELEASE). The UART
    is genuinely silent then — switch to DEBUG or DATASET.
-   The NUCLEO's ST-LINK re-enumerates on every reset. If your terminal
    was opened before you flashed, reopen it after.

### Garbled output (every second character is weird)

-   Baud is off by one of the common wrong-guesses. Try 115200 instead of
    921600. If characters are now readable but very few — you found the
    issue; switch back to 921600 on the terminal side.

### CPCU stops seeing packets after switching to DATASET

-   Inspect the board's LED. If it stopped blinking, the UART transmit is
    blocking the main loop (bug in `log.h`'s `HAL_UART_Transmit` timeout,
    or a runaway LOG_CSV line).
-   Check UART line length in your CSV. If it's >42 chars you introduced
    extra columns somewhere and the budget is blown.
-   Confirm USART1_TX DMA (DMA1 Ch 4) is actually enabled in CubeMX.
    Without DMA, 1000 × 41-byte lines/s will saturate polling UART and
    starve `NRF_Transmit`.

### `drop` counter in LOG output is non-zero

-   The ADC ISR (priority 0) lapped the main loop consumer. This means a
    LOG call took > 500 µs or `NRF_Transmit` hit a worst-case retransmit
    storm.
-   In DEBUG mode, raise `LOG_CSV_DEBUG_INTERVAL` (fewer lines = less UART
    pressure).
-   In DATASET mode, raise `BSAU_DATASET_CSV_DECIMATION` in
    `bsau_config.h` (default 1 emits every packet; 2 = every other;
    5 = 200 Hz CSV).
-   If the drop counter still climbs at decimation 5, something else is
    wrong — see `BSAU_TEST_GUIDE.md` Phase 2 to isolate.

### `retry` or `loss` counters spike

-   Wi-Fi interference. `NRF_CHANNEL = 76` (2476 MHz) is above Wi-Fi
    Channel 11, but some 5 GHz routers still have 2.4 GHz spillover. Move
    further from the router or switch the NRF channel (change
    `NRF_CHANNEL` on both BSAU and CPCU and rebuild both).
-   Antenna problems. The board antenna is meant to face away from metal
    — slip the board out of any metal enclosure while debugging.
-   Distance. Line-of-sight limit at 2 Mbps / 0 dBm with a PCB antenna is
    ~19 m indoors (§6.10). Beyond that, retries climb.

### First packet lost every time

-   Normal — the `FIRST_PACKET` flag is designed to let CPCU resynchronise,
    and the first packet may fire before the NRF has fully settled. As long
    as the CPCU sees it within 50 ms, everything continues.
-   If the CPCU reports `seq_gaps > 0` on *every* boot, check `NRF_POR_DELAY_MS`
    — it should be ≥ 200 ms. Lower values can start NRF_Init before the
    chip's POR is done.

### NRF init failed at boot but the board is still running (v2.4)

This is the new non-fatal behaviour — **not a bug**. Symptoms:

```
[BSAU - NRF ]: NRF_Init           [WARN] Attempt 1 failed (err=-3), ...
[BSAU - NRF ]: NRF_Init           [WARN] Attempt 2 failed (err=-3), ...
[BSAU - NRF ]: NRF_Init           [WARN] All 2 attempts failed (err=-3) — booting
                                          without radio, BSAU_Run will retry every
                                          500 packets
[BSAU - APP ]: BSAU_Init          [OK  ] Pipeline live (radio OFFLINE)
```

In v2.3 and earlier this would have called `Error_Handler()` (i.e. hard
lock the board with `__disable_irq() + while(1)`). In v2.4 the system
boots, ADC + UART come up, and the periodic health check inside
`BSAU_Run` retries the chip every `NRF_HEALTH_CHECK_INTERVAL = 500`
packets (~500 ms at 1 kHz). When the chip finally answers you'll see:

```
[BSAU - NRF ]: Health             [OK  ] Recovered
```

Common causes that this catches automatically:

-   Radio rail brownout at boot (battery sagging on motor inrush) —
    chip will come up once the rail recovers.
-   User pulled the NRF module mid-run and re-inserted it.
-   SPI bus glitch from a long jumper-wire setup that resolves itself
    once you settle the cabling.
-   POR delay edge case where the chip wasn't *quite* ready in 200 ms.

If `Health [FAIL]` keeps repeating without ever printing
`Health [OK ] Recovered`, then the chip really is dead — check Phase 5
NRF self-test (`BSAU_TEST_GUIDE.md §7`) and the wiring; this is the
same situation that the v2.3 hard-lock used to indicate, just no longer
silently behind a wedged main loop.

In DATASET mode specifically, even with the radio offline the UART
CSV stream keeps flowing because the CSV emit happens **before** the
TX gate (per the v2.2 fix); so the DSP/AI team can keep collecting
ground-truth labels via UART while you debug the radio side. This
is the main reason the init is no longer fatal.

---

## 9. Command reference cheatsheet

```bash
# -- BSAU-side build / flash --
# (done in STM32CubeIDE; click through the Build / Run buttons)

# -- PC-side UART capture --
python3 bsau_dataset_collector.py --port /dev/ttyACM0 --label REST
python3 bsau_dataset_collector.py --port /dev/ttyACM0 --label H_OPN --output-dir ./my_data

# -- Raw UART sniff (no collector) --
stty -F /dev/ttyACM0 921600 cs8 -cstopb -parenb -icanon -echo
cat /dev/ttyACM0

# -- Windows equivalent of the above --
mode COM5 BAUD=921600 PARITY=N DATA=8 STOP=1
python -m serial.tools.miniterm COM5 921600

# -- CPCU-side TUI dataset page --
cpcu_tui                # then press '7'
cpcu_tui --demo         # dataset page disabled, but you can preview the layout

# -- Count samples in a collected file --
wc -l ./datasets/REST_0.csv

# -- Quick-look the waveform --
python3 -c "import pandas as pd, matplotlib.pyplot as plt; \
            d=pd.read_csv('datasets/REST_0.csv', header=None); \
            d.plot(); plt.show()"
```

---

**Done.** For the deeper "why" on the ADC / DMA / radio design see
`BSAU_ARCHITECTURE.md`. For running the BSAU-side test suite see
`BSAU_TEST_GUIDE.md`.
