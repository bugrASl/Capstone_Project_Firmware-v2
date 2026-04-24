# BSAU Test Guide — v2.1

**Author:** bugrASl
**Date:** April 2026
**Audience:** anyone who needs to verify that a BSAU board actually works —
from a brand-new team member running their first `BSAU_MODE_TEST_PKT_LOG`
build to a seasoned engineer chasing a regression the night before a demo.

This document is in two parts:

- **Part A — Walkthrough.** What to run in what order, what a green result
  looks like, what each failure actually points at.
- **Part B — Test Reference.** Every test (TB-100 through TB-309) with
  exact preconditions, expected output, pass criteria, failure analysis,
  and wire-level bit patterns.

Most of the time you only need Part A. Jump to the relevant TB-XXX in
Part B when you need the exact-bits detail. If a step says "PASS
criterion: …" and you see something *different*, check the **What it
means when it fails** note right under it — don't guess.

---

## Table of Contents

**Part A — Walkthrough**
1.  [Test philosophy (why this order)](#1-test-philosophy-why-this-order)
2.  [Quick start](#2-quick-start)
3.  [Phase 1 — Build sanity](#3-phase-1--build-sanity)
4.  [Phase 2 — Codec round-trip (`TEST_PKT_LOG`)](#4-phase-2--codec-round-trip-test_pkt_log)
5.  [Phase 3 — ADC pipeline (`TEST_ADC_CSV` / `TEST_CSV`)](#5-phase-3--adc-pipeline-test_adc_csv--test_csv)
6.  [Phase 4 — Goertzel DFT verify (`TEST_DFT_LOG`)](#6-phase-4--goertzel-dft-verify-test_dft_log)
7.  [Phase 5 — NRF self-test (`TEST_NRF_LOG`)](#7-phase-5--nrf-self-test-test_nrf_log)
8.  [Phase 6 — Integration with CPCU](#8-phase-6--integration-with-cpcu)
9.  [DATASET mode smoke test](#9-dataset-mode-smoke-test)
10. [Regression policy — what to re-run after a change](#10-regression-policy--what-to-re-run-after-a-change)
11. [Equipment checklist](#11-equipment-checklist)

**Part B — Test Reference**
- [B.1 Tier 1 — BSAU Standalone](#b1-tier-1--bsau-standalone) (TB-100 … TB-108)
- [B.2 Tier 2 — CPCU Standalone](#b2-tier-2--cpcu-standalone) (TB-200 … TB-206)
- [B.3 Tier 3 — End-to-End Integration](#b3-tier-3--end-to-end-integration) (TB-300 … TB-309)
- [B.4 Test equipment summary (per test)](#b4-test-equipment-summary-per-test)
- [B.5 Test execution order (board bring-up)](#b5-test-execution-order-board-bring-up)
- [B.6 Regression policy — exhaustive (by changed file)](#b6-regression-policy--exhaustive-by-changed-file)

---

## 1. Test philosophy (why this order)

Tests are layered by dependency:

```
  Phase 1    Build sanity         (nothing runs, does it at least compile?)
  Phase 2    Codec round-trip     (no peripherals, pure C memory test)
  Phase 3    ADC pipeline         (MCU only — ADC, DMA, TIM6)
  Phase 4    Goertzel DFT         (ADC + math — verifies chain end-to-end)
  Phase 5    NRF self-test        (SPI + radio — no CPCU needed)
  Phase 6    Integration          (radio with real CPCU at the other end)
  Phase 7    DATASET smoke        (new v2.1 dual-path, depends on 5+6)
```

**Always bottom-up.** If Phase 2 fails, Phase 6 lies — the CPCU might be
seeing garbage that looks structurally correct because the codec is broken
in the same direction on both ends. Fix lower phases first.

A single test mode is just "uncomment one `#define` in `bsau_config.h`,
rebuild, flash, read UART". You don't need any special runner, and there's
no `run_tests.sh` equivalent like the CPCU has — the compiler and ST-LINK
are all the infrastructure you need.

---

## 2. Quick start

You're holding a board that someone says "should work". To confirm in
under five minutes:

```
1.  Flash BSAU_MODE_TEST_PKT_LOG, reset, read UART.
    PASS: "WL CODEC: 24 PASS, 0 FAIL"

2.  Flash BSAU_MODE_TEST_NRF_LOG, reset, read UART.
    PASS: "Self-test suite passed" in the first 2 seconds.

3.  Flash BSAU_MODE_RELEASE, power on, check CPCU TUI page 2.
    PASS: pkt_rate ~= 1000 Hz, loss_rate < 0.1%.
```

If all three pass, the board is ready for production use. If any fails,
go to the matching Phase below.

---

## 3. Phase 1 — Build sanity

**What it validates:** the toolchain is set up, the HAL + CMSIS sources
are linked, `bsau_config.h` has exactly one active mode, and `log.h`'s
mutual-exclusion checks all evaluate cleanly.

**How:** open `bsau_config.h`, make sure *exactly one* mode is uncommented,
then `Project → Build All` in STM32CubeIDE. That's it.

**PASS criterion:**

```
**** Build Finished. 0 errors, 0 warnings. ****
```

Or up to a handful of "unused variable" warnings — those are harmless.

**What it means when it fails:**

| Compiler says… | What's wrong | Fix |
|----------------|--------------|-----|
| `#error "Define exactly one BSAU_MODE_*"` | Two uncommented, or none uncommented | Edit `bsau_config.h`, leave exactly one active |
| `#error "Define LOG_BOARD_CPCU or LOG_BOARD_BSAU"` | `LOG_BOARD_BSAU` got deleted | Add `#define LOG_BOARD_BSAU` near the top of `bsau_config.h` |
| `undefined reference to 'HAL_...'` | HAL source files not added to project | *Project → Properties → C/C++ General → Paths and Symbols*, add `Drivers/STM32L4xx_HAL_Driver/Src` |
| `undefined reference to 'NRF_GetTxStats'` | `nrf24l01.c` not in build | Same menu, add `Core/Src` |
| `'pkt.reserved' has no member named 'reserved'` | `bsau_app.c` is from before the v2.1 rename | Re-pull `bsau_app.c` — `reserved` was removed (it's `tx_retry`, `pkt_loss`, `timestamp`, `vbat_raw` now) |

---

## 4. Phase 2 — Codec round-trip (`TEST_PKT_LOG`)

**What it validates:** `WL_Pack()` / `WL_Unpack()` in `wireless_packet.c`.
No peripherals involved — this is a pure software test of the 12-bit
packing logic. If this fails, every other phase is compromised.

**How:**

```
bsau_config.h:  #define BSAU_MODE_TEST_PKT_LOG
```

Build, flash. Open UART at 921600 8N1.

**PASS criterion:** the last line printed is

```
[BSAU - TEST]: PacketVerify       [PASS] === WL CODEC: 24 PASS, 0 FAIL ===
```

and then the LED blinks forever at 1 Hz.

**What's being tested, in plain English:**

| Sub-test | What it catches |
|----------|-----------------|
| `PKT_Ramp` | Basic 12-bit packing across all 8 channels × 2 samples |
| `PKT_Boundary` | The 0x000 and 0xFFF edge cases (is one of them being lost to sign extension?) |
| `PKT_SeqWrap` | The 8-bit `seq` field wraps cleanly at 256 |
| `PKT_Alternating` | Alternating 0x555 / 0xAAA catches nibble-swap bugs |
| `PKT_Overflow` | Values > 12 bits correctly masked, not truncated high nibble |
| `PKT_RawBytes` | Hand-computed exact bytes at known offsets (catches the rare "it ALL shifted by one byte" regressions) |

**What it means when it fails:**

-   One specific channel wrong → the `WL_OFF_SAMPLE(s)` macro in
    `wireless_packet.h` is wrong.
-   One specific bit wrong on every value → the 12-bit packing macros
    (`byte[1] = A_hi | (B_lo << 4)`) were touched.
-   Raw bytes wrong but everything else passes → endian confusion in
    a new helper.

---

## 5. Phase 3 — ADC pipeline (`TEST_ADC_CSV` / `TEST_CSV`)

Two related tests — `TEST_ADC_CSV` for waveform inspection, `TEST_CSV`
for long-run quality statistics. Run them in that order.

### 5.1 `TEST_ADC_CSV` — watch the waveforms

**What it validates:** ADC calibration, DMA circular linkage, TIM6 trigger
routing, channel ordering — the whole front half of the pipeline, without
the radio.

**How:**

```
bsau_config.h:  #define BSAU_MODE_TEST_ADC_CSV
```

Build, flash. Open SerialPlot:

-   Port: the BSAU's COM
-   Baud: 921600
-   Binary mode, little-endian
-   Frame layout: 2 sync bytes + 9 × uint16 (8 EMG + 1 battery)

**PASS criterion:** all 8 channels should show noise around mid-scale
(2048) when electrodes are disconnected. Touch a finger to a channel
input pin and that channel's trace should wiggle visibly.

**What it means when it fails:**

| Symptom | Cause |
|---------|-------|
| All channels stuck at 0 or 0xFFF | ADC not converting — check TIM6 TRGO wiring in `MX_TIM6_Init` |
| Channel trace is a 90°-shifted copy of the adjacent channel | DMA stride wrong or `ADC_DMA_CHANNELS` doesn't match the CubeMX scan length |
| Everything looks right but channels 6 and 7 are garbage | **Known issue** — ADC_DMA_CHANNELS = 7 but the packet has 8. See the TODO at top of `bsau_app.c`. Ignore ch6/ch7 until the CubeMX ADC scan is widened to 9 ranks. |
| Periodic spikes every ~500 µs | An ISR is running too long and corrupting the DMA buffer. Check `NVIC` priorities in `BSAU_ARCHITECTURE.md` §5 |

### 5.2 `TEST_CSV` — drop-counter stress

**What it validates:** the `drop` counter in the CSV stays at 0 even
under UART load. Confirms the main-loop consumer keeps up with the ADC
ISR producer.

**How:**

```
bsau_config.h:  #define BSAU_MODE_TEST_CSV
```

Flash, then on the host:

```bash
python3 - << 'EOF'
import serial
ser = serial.Serial('/dev/ttyACM0', 921600, timeout=1)
last_drop = 0
for _ in range(10000):   # ~ 10 seconds
    line = ser.readline().decode().strip()
    if not line: continue
    cols = line.split(',')
    drop = int(cols[-1])
    if drop != last_drop:
        print(f"DROP INCREASED: {last_drop} -> {drop}")
        last_drop = drop
print(f"Final drop counter: {last_drop}")
EOF
```

**PASS criterion:** `Final drop counter: 0`.

**What it means when it fails:** the CPU is failing to consume an ADC
scan before the next one arrives — either UART is blocking too long
(missing DMA?) or a LOG call is taking >500 µs.

---

## 6. Phase 4 — Goertzel DFT verify (`TEST_DFT_LOG`)

**What it validates:** inject a known sine into channel 0 and confirm
BSAU's DFT sees the right bin as dominant. Catches scale/axis/sample-rate
mistakes that Phase 3 can miss.

**How:**

1.  Function generator → channel-0 input. 1.0 Vpp, DC offset 1.65 V,
    frequency **100 Hz** (one of the five Goertzel bins).
2.  `bsau_config.h`: `#define BSAU_MODE_TEST_DFT_LOG`.
3.  Build, flash, open UART.

**Expected lines:**

```
[BSAU - TEST]: DFT_Verify         [INFO] blk=4 |  50Hz=31 100Hz=2180 200Hz=44 350Hz=39 500Hz=31 | peak=100Hz conc=91%
[BSAU - TEST]: DFT_Verify         [INFO] blk=5 |  50Hz=28 100Hz=2165 200Hz=42 350Hz=40 500Hz=33 | peak=100Hz conc=92%
```

**PASS criterion:** `peak=100Hz` and `conc > 80%` for at least 10
consecutive blocks.

**What it means when it fails:**

| Symptom | Cause |
|---------|-------|
| `peak=50Hz` always | Function generator set to 50 Hz, or 50 Hz mains interference swamping the test signal. Try 150 Hz. |
| Peak hops randomly | Input is clipping. Reduce amplitude. |
| All bins tiny numbers | No signal reaching ADC. Check probe contact. |
| Peak off by factor of 2 | Sample rate wrong. Verify `GOERTZEL_FS_HZ == 2000.0f` in `bsau_app.h` matches the actual TIM6 rate. |

---

## 7. Phase 5 — NRF self-test (`TEST_NRF_LOG`)

**What it validates:** every NRF register reads back the programmed value,
SPI loopback is clean, FIFO behaves, TX state machine transitions correctly.
Does NOT need a receiver — the board talks only to the local NRF chip.

**How:**

```
bsau_config.h:  #define BSAU_MODE_TEST_NRF_LOG
```

Build, flash, open UART. The first burst runs the full self-test suite,
after which the board enters a repeating test loop (TX ping every 150 ms,
periodic health check, periodic full power-cycle stress test).

**PASS criterion (first 2 seconds of output):**

```
[BSAU - TEST]: NRF_Test_Init      [OK  ] Radio ready on CH76
[BSAU - TEST]: NRF_SelfTest       [PASS] Self-test suite passed
```

**PASS criterion (after 30 seconds):**

```
[BSAU - TEST]: NRF_Summary        [INFO] tx_ok=180 tx_fail=2 rate=98% health=2/2 t=28s
```

`rate ≥ 95%` with no receiver present is normal — the TX sees MAX_RT for
every ping because nobody ACKs. If you have a receiver powered on the
same channel, rate should climb to 100%.

**What it means when it fails:**

| Symptom | Cause |
|---------|-------|
| `NRF_Init failed after retry` | Wiring problem. Double-check PB3/PB4/PB5 (SPI), PB6 (CSN), PA8 (CE). |
| `SPI+REGs` FAIL | SPI clock too fast for your wiring. `SPI_PRESCALER_16` in `spi.c` gives 5 MHz — reduce to /32 if traces are long. |
| Power cycle test keeps FAILing | VDD droops on CE pulse. Add a 10 µF bulk cap near the NRF VCC pin. |
| All test phases pass at start but degrade over time | Thermal issue. Check the NRF module's onboard regulator isn't overheating under load. |

---

## 8. Phase 6 — Integration with CPCU

**Preconditions:** Phases 1–5 all pass on BSAU. The CPCU has passed
its own Phase 3 (see `CPCU_TEST_GUIDE.md`). Both devices are within
~5 m line-of-sight.

**How:**

1.  CPCU: `systemctl start cpcu`, then `cpcu_tui`.
2.  BSAU: `#define BSAU_MODE_RELEASE`, flash, power on.
3.  Watch `cpcu_tui` page 2 (`2`).

**PASS criteria:**

| Metric | Expected | Notes |
|--------|----------|-------|
| `pkt_rate` | 990–1000 Hz | Below 990: retransmits are happening; check `tx_retry` on BSAU UART in DEBUG mode |
| `loss_rate` | < 0.1% | |
| `ring overflows` | 0 | Non-zero = the Python DSP is too slow |
| `seq_gaps` | 0 after ~5 s warmup | First packet may count as a gap; that's fine |
| Battery `lvl` | 0 (OK) | If 1 or 2, the battery on BSAU is dying |

**Endurance check:** let it run for 1 hour. Come back. All of the above
metrics should still hold; `seq_gaps` can be up to ~10 (one gap per
~400 000 packets is normal for 2 Mbps ISM with no error correction).

**What it means when it fails:**

-   `pkt_rate` way below 1000 (say 600–800) → BSAU radio is retransmitting
    constantly. Usually a 2.4 GHz Wi-Fi collision (§8 in Run Guide)
    or an antenna placement problem.
-   `pkt_rate` exactly at 500 or 333 → somebody changed the CubeMX TIM6
    ARR. Verify 39999 in `MX_TIM6_Init`.
-   `ring_overflows > 0` but everything else fine → the Python DSP isn't
    consuming fast enough. This is a CPCU problem, not BSAU; see CPCU Test Guide.

---

## 9. DATASET mode smoke test

**Preconditions:** Phase 8 (Integration) passing. The DATASET mode
shares every critical code path with RELEASE, so if integration is
healthy the DATASET addition is almost certainly healthy too — this
section is for *confirming* that the new UART emit doesn't disturb
the radio path.

**How:**

1.  BSAU: `#define BSAU_MODE_DATASET`, flash.
2.  On the PC: `python3 bsau_dataset_collector.py --port /dev/ttyACM0 --label SMOKE`
    in one terminal; leave it collecting.
3.  On the Pi: `cpcu_tui` in another terminal; watch page 2.

**PASS criteria:**

| Check | Expected |
|-------|----------|
| `pkt_rate` on CPCU | Still 990–1000 Hz (same as RELEASE) |
| `loss_rate` on CPCU | Still < 0.1% |
| `drop` counter on BSAU | Not visible (no LOG), but `BSAU_ADC_GetBattery` returning sensible values across captures suggests the ADC ISR is on-time |
| Collector script `gaps` column after 60 s | 0 or 1 — a single gap at startup is the UART framing first-line warmup |
| Collector file line count after 10 s | ~10 000 lines |
| `head -1 datasets/SMOKE_0.csv` | A row of 8 integers in 0–4095, no header |

**What it means when it fails:**

-   `pkt_rate` drops versus RELEASE → UART DMA isn't running and the
    `HAL_UART_Transmit` is blocking the main loop. Verify DMA1 Channel 4
    is configured as USART1_TX in CubeMX.
-   Collector file has fewer than ~10 × elapsed_seconds lines → the board
    is emitting at <1000 Hz. Either the decimation constant is > 1 or the
    ADC isn't triggering at 2 kHz.
-   `drop` counter climbing (seen by switching briefly to DEBUG mode) →
    the UART line is taking too long. Raise `BSAU_DATASET_CSV_DECIMATION`
    to 2 and retry.

---

## 10. Regression policy — what to re-run after a change

| Changed… | Re-run |
|----------|--------|
| `wireless_packet.c` / `.h` | Phase 2. If it touches `WL_SAMPLES_PER_PACKET` or `WL_NUM_CHANNELS`, also Phase 3 + Phase 6. |
| `bsau_adc.c` / ADC CubeMX settings | Phases 3 + 4. |
| `nrf24l01.c` / SPI CubeMX settings | Phase 5 + Phase 6. |
| `bsau_app.c` (main loop) | Phases 6 + 9 (the one most likely to slip in a regression). |
| `log.h` | Nothing automatic, but eyeball the first few seconds of UART in DEBUG and DATASET to confirm neither is silent or garbled. |
| CubeMX clock tree | **Everything.** Clock changes touch every peripheral timing budget. |

---

## 11. Equipment checklist

| Item | Used for |
|------|----------|
| NUCLEO-L432KC board, flashed via ST-LINK | Target |
| USB-A-to-Micro-B cable (data, not charge-only) | Power + UART |
| Serial terminal at 921600 baud | UART output reading |
| Function generator (or sine-gen smartphone app into an audio jack) | Phase 4 DFT check |
| Oscilloscope (1–2 ch, ≥ 10 MHz) | Only if debugging SPI timing or clocks |
| A second NUCLEO or a CPCU | Phase 6 integration |
| Python 3 + `pyserial` | Phases 5 consumption script, collector script |
| SerialPlot (or equivalent) | Phase 3 waveform inspection |

---

**End of walkthrough. For the exact per-test specifications, continue to
[Part B — Test Reference](#part-b--test-reference) below. For standard
operations once everything is green, see `BSAU_RUN_GUIDE.md`.**

---

# Part B — Test Reference

Every test has a stable ID (TB-XXX). The walkthrough in Part A refers
to these IDs so Part B can stay authoritative for the pass/fail
details, exact bit patterns, and failure-analysis tables.

Convention for every TB entry:

- **Preconditions** — hardware and software state required.
- **Procedure** — step-by-step execution.
- **Implementation** — code snippets, build mode, or CubeMX config.
- **Expected output** — exact UART lines or metric values.
- **Pass criteria** — quantitative thresholds.
- **Failure analysis** — what each failure mode means and how to fix
  it.

```
Tier 1 — BSAU Standalone      (NUCLEO + UART, no radio receiver)
Tier 2 — CPCU Standalone      (Pi, synthetic packets, no BSAU)
Tier 3 — End-to-End           (both boards, within radio range)
```

---

## B.1 Tier 1 — BSAU Standalone

### B.1.1 TB-100: ADC Calibration and Startup

| Field | Value |
|-------|-------|
| Build mode | Any (runs during `BSAU_Init` / `BSAU_Test_Init`) |
| Preconditions | Board powered via USB, UART terminal at 921600 8N1 |
| Duration | < 2 seconds (automatic) |

**Procedure:** Power cycle the board. Observe UART output.

**Expected output:**

```
[BSAU - ADC ]: BSAU_ADC_Init      [RUN ] Calibrating ADC1...
[BSAU - ADC ]: BSAU_ADC_Init      [OK  ] Calibration complete
[BSAU - ADC ]: BSAU_ADC_Init      [OK  ] Pipeline running (TIM6 trig, DMA circ)
```

**Pass criteria:** Both `[OK  ]` lines appear. No `[FAIL]` lines.

**Failure analysis:**

| Symptom | Cause | Fix |
|---------|-------|-----|
| "Calibrating" never completes | ADC clock not running | CubeMX: ADC clock source = Synchronous /1 |
| `[FAIL] Calibration` | ADC hardware error | Check VDDA (must be 3.3 V). Check ADC_CR ADCAL bit. |
| `[FAIL] Pipeline` | DMA not started or TIM6 not triggering | `MX_DMA_Init()` must run BEFORE `MX_ADC1_Init()`. TIM6 TRGO = Update Event. |
| No UART at all | Wrong baud or UART not init | 921600 8N1. Verify `MX_USART1_UART_Init()`. |

**Validates:** ADC internal calibration, DMA circular buffer linkage,
TIM6 trigger routing.

---

### B.1.2 TB-101: ADC Channel Liveness (Binary Stream)

| Field | Value |
|-------|-------|
| Build mode | `BSAU_MODE_TEST_ADC_CSV` |
| Preconditions | All 8 EMG inputs connected (or floating for noise-floor test) |
| Tool | SerialPlot (binary mode) or custom Python script |
| Duration | 10 seconds minimum |

**Procedure:**
1. Flash with `BSAU_MODE_TEST_ADC_CSV`.
2. Open SerialPlot. Configure: binary frame, little-endian, 2-byte
   sync header + 9 × uint16.
3. Or capture raw binary with Python:

```python
import serial, struct
ser = serial.Serial('/dev/ttyACM0', 921600, timeout=1)
FRAME_SIZE = 2 + 9*2       # sync + 9 × uint16

for _ in range(2000):       # 1 second at 2 kHz
    frame = ser.read(FRAME_SIZE)
    if len(frame) < FRAME_SIZE:
        continue
    seq = struct.unpack_from('<H', frame, 0)[0]
    channels = struct.unpack_from('<9H', frame, 2)
    emg = channels[:8]
    batt = channels[8]
    print(f"seq={seq:5d}  ch0={emg[0]:4d} ch1={emg[1]:4d} ... batt={batt:4d}")
```

**Pass criteria:**
- Sync counter increments by 1 each frame (no gaps).
- All 8 EMG channels show non-zero values (not stuck at 0x000 or 0xFFF).
- Battery channel reads ~4095 if USB powered (both sides of the 100k/100k
  divider sit at VDD).
- If a function generator is connected, the waveform is visible at the
  correct frequency on the correct channel only.

**Failure analysis:**

| Symptom | Cause | Fix |
|---------|-------|-----|
| One channel always 0 | Pin not configured as analog | CubeMX pin mode must be `ADC_INx` |
| One channel always 4095 | Input shorted to VDDA | Check soldering, trace continuity |
| Sync gaps | UART can't keep up (blocking TX) | Expected — gaps = dropped frames, not lost scans |
| All channels identical | ADC scan rank order wrong | Verify CubeMX rank assignments match pin table |
| Battery reads 0 | PB0 not in scan, divider disconnected | Rank 9 = PB0/IN15 |

---

### B.1.3 TB-102: ASCII CSV with Drop Counter

| Field | Value |
|-------|-------|
| Build mode | `BSAU_MODE_TEST_CSV` |
| Preconditions | UART terminal or file capture |
| Duration | 30 seconds |

**Expected output (first 3 lines):**

```
seq,s0c0,s0c1,s0c2,s0c3,s0c4,s0c5,s0c6,s0c7,batt,drop
0,2048,2050,2045,2055,2040,2060,2038,2062,4095,0
1,2049,2051,2046,2056,2041,2061,2039,2063,4095,0
```

**Pass criteria:**
- Header row has 8 EMG columns (s0c0 through s0c7) + batt + drop.
- `seq` increments monotonically.
- `drop` column reveals DMA overwrites (expected > 0 in CSV mode — the
  blocking UART transmit is slower than the 2 kHz ADC trigger).
- All 8 EMG columns show varying values.

**Failure analysis:**

| Symptom | Cause | Fix |
|---------|-------|-----|
| Only 6 EMG columns | Old `CONSIDERED_CHANNELS` config | Set `CONSIDERED_CHANNELS_COUNT = 8` in `bsau_app.h` |
| All EMG columns identical | `g_considered_ch[]` indices wrong | Verify channel index array matches rank order |
| `drop` climbs fast (> 10/s) | UART blocking too long | Expected — `snprintf` + UART TX takes ~200 µs per line |

---

### B.1.4 TB-103: Goertzel DFT Frequency Verification

| Field | Value |
|-------|-------|
| Build mode | `BSAU_MODE_TEST_DFT_LOG` |
| Equipment | Function generator → PA0 (ch0), sine, 1.0 Vpp, 1.65 V DC offset |
| Test frequencies | 50, 100, 200, 500 Hz |

**Procedure per frequency:**
1. Set function generator to target frequency.
2. Power cycle the board, wait for 5 Goertzel blocks.
3. Record dominant frequency.

**Expected output (example: 100 Hz input):**

```
[BSAU - TEST]: DFT_Init           [OK  ] DFT configured: 8 bins, block=128, fs=2000
[BSAU - TEST]: DFT_Block          [INFO] block=0 dominant=100Hz mag=24537.2 ratio=42.1
[BSAU - TEST]: DFT_Block          [INFO] block=1 dominant=100Hz mag=24412.8 ratio=41.8
```

**Pass criteria:**
- Dominant frequency matches input within ±(Fs / block_size) Hz.
- At block_size = 128, Fs = 2000: Δf = 15.625 Hz → max error = ±15.625 Hz.
- Dominant bin magnitude > 10× (20 dB) above next highest bin
  (`ratio > 10.0`).

**Mathematical basis:**

```
Goertzel bin resolution = Fs / block_size
For Fs=2000, block_size=128: Δf = 15.625 Hz

50 Hz  → k = round(128 × 50/2000) = 3  → f_bin = 46.875 Hz
100 Hz → k = round(6.4)           = 6  → f_bin = 93.75 Hz
200 Hz → k = round(12.8)          = 13 → f_bin = 203.125 Hz
500 Hz → k = round(32.0)          = 32 → f_bin = 500.0 Hz (exact)
```

**Failure analysis:**

| Symptom | Cause | Fix |
|---------|-------|-----|
| Wrong dominant frequency | ADC sample rate ≠ 2 kHz | Check TIM6: PSC = 0, ARR = 39999 |
| Low magnitude, no clear peak | Signal amplitude too low, or DC offset wrong | Ensure signal stays within 0–3.3 V |
| Dominant frequency jitters between bins | Signal frequency falls between two bins | Normal — reduce block_size for wider bins, or increase it for finer resolution |

---

### B.1.5 TB-104: Wireless Packet Codec Round-Trip

| Field | Value |
|-------|-------|
| Build mode | `BSAU_MODE_TEST_PKT_LOG` |
| Preconditions | UART terminal only (no radio, no ADC) |
| Duration | < 1 second (runs once at startup) |

**Sub-tests:**

| ID | Name | Test vector | Expected |
|----|------|-------------|----------|
| TB-104a | PKT_Ramp | `ch[c] = 0x100×(c+1)`, 8ch, 2 scans | Exact round-trip |
| TB-104b | PKT_Boundary | All 0x000, then all 0xFFF | Exact round-trip |
| TB-104c | PKT_SeqWrap | seq=0x00 and seq=0xFF | Exact round-trip |
| TB-104d | PKT_Alternating | 0xAAA/0x555 alternating | Exact round-trip |
| TB-104e | PKT_Overflow | 0x1ABC → 0x0ABC, 0xFFFF → 0x0FFF | Masked to 12-bit |
| TB-104f | PKT_RawBytes | Known input, verify wire bytes | Byte-exact match |
| TB-104g | PKT_Metadata | tx_retry=5, pkt_loss=3, ts=0x1234, vbat=0x0ABC | Exact round-trip |
| TB-104h | PKT_Flags | All flag bits + BATT_LVL combinations | Preserved exactly |
| TB-104i | PKT_StaticAssert | (compile-time) | Layout = 32 bytes |

**TB-104f raw byte verification (v2.1 layout):**

```
Input:
  seq=0x42, flags=0x81, tx_retry=0x03, pkt_loss=0x02,
  timestamp=0x1234, vbat_raw=0x0ABC,
  samples[0].ch[0]=0x123, samples[0].ch[1]=0x456

Expected wire bytes:
  raw[0]  = 0x42   (seq)
  raw[1]  = 0x81   (flags: FIRST_PACKET | BATT_LOW)
  raw[2]  = 0x03   (tx_retry)
  raw[3]  = 0x02   (pkt_loss)
  raw[4]  = 0x34   (timestamp low byte: 0x1234 & 0xFF)
  raw[5]  = 0x12   (timestamp high byte: 0x1234 >> 8)
  raw[6]  = 0xAB   (vbat_raw >> 4)
  raw[7]  = 0xC0   ((vbat_raw & 0x0F) << 4)
  raw[8]  = 0x23   (ch0 low byte)
  raw[9]  = 0x61   (ch0 high nibble | ch1 low nibble)
  raw[10] = 0x45   (ch1 high byte)

Derivation for raw[9]:
  ch0 high nibble = (0x123 >> 8) & 0x0F = 0x01
  ch1 low nibble  = (0x456 & 0x0F) << 4 = 0x60
  raw[9] = 0x01 | 0x60 = 0x61  ✓
```

**Expected output (all pass):**

```
[BSAU - TEST]: PacketVerify       [RUN ] === WL CODEC ROUND-TRIP START ===
[BSAU - TEST]: PKT_Ramp           [PASS] 8ch × 2scan ascending OK
[BSAU - TEST]: PKT_Boundary       [PASS] all-zero OK
[BSAU - TEST]: PKT_Boundary       [PASS] all-FFF OK
[BSAU - TEST]: PKT_SeqWrap        [PASS] seq=0x00 OK
[BSAU - TEST]: PKT_SeqWrap        [PASS] seq=0xFF OK
[BSAU - TEST]: PKT_AltData        [PASS] 0xAAA/0x555 OK
[BSAU - TEST]: PKT_Ovf_0          [PASS] 0x1ABC→0xABC
[BSAU - TEST]: PKT_Ovf_1          [PASS] 0xFFFF→0xFFF
[BSAU - TEST]: PKT_Ovf_2          [PASS] 0x2000→0x000
[BSAU - TEST]: PKT_Raw_Seq        [PASS] raw[0]=0x42
[BSAU - TEST]: PKT_Raw_Flags      [PASS] raw[1]=0x81
... (remaining raw byte checks) ...
[BSAU - TEST]: META_seq           [PASS] seq=0xFE
[BSAU - TEST]: META_flags         [PASS] flags=0xFF
[BSAU - TEST]: META_retry         [PASS] retry=5
[BSAU - TEST]: META_loss          [PASS] loss=3
[BSAU - TEST]: META_ts            [PASS] ts=0xABCD
[BSAU - TEST]: META_vbat          [PASS] vbat=0x0F0F
[BSAU - TEST]: PacketVerify       [PASS] === WL CODEC: N PASS, 0 FAIL ===
```

**Failure analysis:**

| Symptom | Cause | Fix |
|---------|-------|-----|
| ch0/ch1 swapped | Nibble order wrong in pack/unpack | Check byte[1] split: high nibble of A goes in low nibble of byte[1] |
| vbat_raw always 0 | `VBAT_ENCODE`/`DECODE` macros wrong | encode = `(val>>4, (val&0xF)<<4)`; decode = `(b0<<4) | (b1>>4)` |
| timestamp endian wrong | BE write, LE read (or vice versa) | Both must use LE: `out[4] = ts & 0xFF, out[5] = ts >> 8` |
| Static assert fails at compile | Layout constant changed | Verify: 8 + 2 × 12 = 32. `WL_OFF_SAMPLES = 8`, `WL_BYTES_PER_SAMPLE = 12` |

---

### B.1.6 TB-105: NRF24L01 Hardware Self-Test Suite

| Field | Value |
|-------|-------|
| Build mode | `BSAU_MODE_TEST_NRF_LOG` |
| Preconditions | NRF24L01+ soldered/connected, SPI1 pins verified |
| Duration | < 3 seconds |

**Sub-tests (run by `NRF_Test_All()`):**

| ID | Name | Method | Catches |
|----|------|--------|---------|
| TB-105a | SPI Loopback | Write 0x03 to SETUP_AW, read back, compare | Wiring faults, SPI clock polarity, dead chip |
| TB-105b | Register Audit | Read every init'd register, compare to expected | Init-order bugs, register write failures |
| TB-105c | Address Verify | Read 5-byte pipe 0 address | Multi-byte SPI transfer issues |
| TB-105d | FIFO Exercise | empty → write payload → non-empty → flush → empty | FIFO controller faults |
| TB-105e | Power Cycle | power-down → PWR_UP=0 → power up → PWR_UP=1 | CONFIG write failures, oscillator startup |
| TB-105f | TX State Machine | Transmit test packet, check TX_DS or MAX_RT | RF synthesiser, PLL, CE pulse timing |

**TB-105b: Register Audit expected values (v2.1 config):**

| Register | Expected | Mask | Verification |
|----------|----------|------|--------------|
| SETUP_AW | 0x03 | 0x03 | 5-byte address width |
| RF_CH | 0x4C | 0x7F | Channel 76 |
| RF_SETUP | **0x0F** | 0x2E | **2 Mbps, 0 dBm** (v1 was 0x26 for 250 kbps) |
| EN_AA | 0x01 | 0x3F | Auto-ACK pipe 0 |
| EN_RXADDR | 0x01 | 0x3F | Pipe 0 enabled |
| RX_PW_P0 | 0x20 | 0x3F | 32-byte payload |
| SETUP_RETR | **0x1F** | 0xFF | **ARD=500 µs, ARC=15** (v1 was 0x5F for ARD=1500 µs) |
| CONFIG | 0x4E | 0x7F | CRC2, PWR_UP, TX, MASK_RX_DR |

**Expected output:**

```
[BSAU - NRF ]: Test_SPI           [PASS] SETUP_AW write/read OK
[BSAU - NRF ]: Test_Regs          [PASS] RF_CH=0x4C (exp 0x4C)
[BSAU - NRF ]: Test_Regs          [PASS] RF_SETUP=0x0F (exp 0x0F)
[BSAU - NRF ]: Test_Regs          [PASS] SETUP_RETR=0x1F (exp 0x1F)
[BSAU - NRF ]: Test_Regs          [PASS] CONFIG=0x4E (exp 0x4E)
[BSAU - NRF ]: Test_Addr          [PASS] P0=E7E7E7E7E7 TX=E7E7E7E7E7
[BSAU - NRF ]: Test_FIFO          [PASS] Empty→Write→NonEmpty→Flush→Empty
[BSAU - NRF ]: Test_Power         [PASS] PowerDown/Up cycle OK
[BSAU - NRF ]: Test_TX            [PASS] State machine responded (MAX_RT expected)
[BSAU - NRF ]: Test_All           [PASS] 6/6 tests passed
```

**Failure analysis:**

| Symptom | Cause | Fix |
|---------|-------|-----|
| SPI loopback FAIL | SCK/MOSI/MISO wiring wrong | Swap MISO/MOSI, check AF5 config |
| RF_SETUP=0x26 (not 0x0F) | Old v1 NRF_Init code | Update NRF_Init: `RF_DR_HIGH | RF_PWR_3` |
| SETUP_RETR=0x5F (not 0x1F) | Old v1 ARD setting | Update NRF_Init: write 0x1F |
| Test_TX TIMEOUT | CE pulse too short or NRF unpowered | Check PA8 GPIO output, NRF VCC=3.3 V, decoupling caps |
| Address mismatch | Multi-byte SPI transfer order | NRF uses LSByte-first for addresses |

---

### B.1.7 TB-106: NRF TX Stress Loop

| Field | Value |
|-------|-------|
| Build mode | `BSAU_MODE_TEST_NRF_LOG` |
| Preconditions | NRF connected, optionally CPCU receiving |
| Duration | 5 minutes minimum |

**Procedure:**
After TB-105, the NRF test mode enters a continuous loop with three
interleaved phases:

```
Every iteration (150 ms interval):
  Phase A: TX ping      — send test payload, log PLOS/ARC
Every 20 iterations (~3 s):
  Phase B: Health check — re-run SPI loopback + register audit + FIFO
Every 100 iterations (~15 s):
  Phase C: Stress cycle — power-cycle radio, re-init, re-verify
```

**Expected output (no receiver — all TX pings report MAX_RT):**

```
[BSAU - TEST]: NRF_TxPing         [FAIL] iter=0 PLOS=1 ARC=15
[BSAU - TEST]: NRF_TxPing         [FAIL] iter=1 PLOS=2 ARC=15
[BSAU - TEST]: NRF_HealthCheck    [PASS] SPI+Regs+FIFO OK (iter=20)
[BSAU - TEST]: NRF_Summary        [INFO] tx_ok=0 tx_fail=50 health=20/0
```

**Expected output (with CPCU receiver):**

```
[BSAU - TEST]: NRF_TxPing         [PASS] iter=0 PLOS=0 ARC=0
[BSAU - TEST]: NRF_Summary        [INFO] tx_ok=49 tx_fail=1 health=20/0
```

**Pass criteria:**
- `health_fail = 0` over the entire run (SPI/register integrity holds).
- No TIMEOUT errors (state machine always responds).
- With receiver: `tx_ok >> tx_fail`, ARC mostly 0.

---

### B.1.8 TB-107: DEBUG Mode Full Pipeline

| Field | Value |
|-------|-------|
| Build mode | `BSAU_MODE_DEBUG` |
| Preconditions | Full hardware: NRF + ADC inputs |
| Duration | 60 seconds |

**Expected init output:**

```
[BSAU - APP ]: BSAU_Init          [RUN ]
[BSAU - APP ]: BSAU_Init          [OK  ] NVIC priority group verified
[BSAU - NRF ]: NRF_Init           [RUN ] ch=76 (POR wait 200ms done)
[BSAU - NRF ]: NRF_Init           [OK  ]
[BSAU - ADC ]: BSAU_ADC_Init      [RUN ] Calibrating ADC1...
[BSAU - ADC ]: BSAU_ADC_Init      [OK  ] Calibration complete
[BSAU - ADC ]: BSAU_ADC_Init      [OK  ] Pipeline running (TIM6 trig, DMA circ)
[BSAU - APP ]: BSAU_Init          [OK  ] Pipeline live
[BSAU - APP ]: BSAU_Run           [RUN ] Entering main loop
```

**Expected runtime output (periodic status every `LOG_STATS_INTERVAL`):**

```
[BSAU - APP ]: Stats              [INFO] seq=1000 batt=4095 ok=1000 lost=0 drop=329
```

**Pass criteria:**
- `lost=0` (with CPCU receiver present).
- `ok` increments at ~1000/s.
- `drop` increments (expected in DEBUG mode — blocking UART overhead
  causes DMA snapshot overwrites).
- `batt` reads a reasonable value.

---

### B.1.9 TB-108: RELEASE Mode Power Measurement

| Field | Value |
|-------|-------|
| Build mode | `BSAU_MODE_RELEASE` |
| Equipment | Multimeter in series on VDD supply, or INA219 breakout |
| Duration | 60 seconds of stable measurement |

**Procedure:**
1. Flash RELEASE mode.
2. Verify: absolutely no UART output (LOG compiles to `((void)0)`).
3. Measure current on 3.0 V supply.

**Pass criteria:**

```
I_total = 7.58 mA ± 30%   →   acceptable range: 5.3 – 10.0 mA

Breakdown if measured with/without NRF:
  MCU only (NRF disconnected):  2.5 – 4.0 mA
  Full system:                  6.0 – 10.0 mA
```

**Failure analysis:**

| Measured | Cause | Fix |
|----------|-------|-----|
| > 12 mA | NRF stuck in continuous TX | Verify RF_SETUP=0x0F, check T_ESB < T_packet |
| > 15 mA | Looks like v1 power budget | Check SETUP_RETR=0x1F, RF_SETUP != 0x26 |
| < 3 mA (no NRF) | MCU in unexpected low-power state | Verify SYSCLK=80 MHz (not MSI 4 MHz) |
| ~3.5 mA (NRF connected) | NRF in power-down or not transmitting | Check NRF_Init succeeds, CONFIG PWR_UP=1 |

---

## B.2 Tier 2 — CPCU Standalone

These run on the Raspberry Pi (or any Linux GCC host) without BSAU
hardware. They validate the CPCU software pipeline using synthetic
data.

### B.2.1 TB-200: Packet Codec Cross-Platform Verification

| Field | Value |
|-------|-------|
| Platform | Raspberry Pi (or any Linux GCC host) |
| Source | `wireless_packet.c` + `test_codec.c` |

**Implementation (excerpt):**

```c
// Compile: gcc -o test_codec test_codec.c wireless_packet.c
void test_ramp(void) {
    WL_Packet tx, rx;
    uint8_t raw[WL_PAYLOAD_SIZE];
    memset(&tx, 0, sizeof(tx));
    tx.seq       = 0x42;
    tx.tx_retry  = 5;
    tx.pkt_loss  = 3;
    tx.timestamp = 0xABCD;
    tx.vbat_raw  = 0x0F0F;

    for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
        for (int c = 0; c < WL_NUM_CHANNELS; c++)
            tx.samples[s].ch[c] = 0x100 * (c + 1) + s;

    WL_Pack(&tx, raw);
    WL_Unpack(raw, &rx);

    assert(rx.seq == tx.seq);
    assert(rx.tx_retry == tx.tx_retry);
    assert(rx.pkt_loss == tx.pkt_loss);
    assert(rx.timestamp == tx.timestamp);
    assert(rx.vbat_raw == tx.vbat_raw);
    for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
        for (int c = 0; c < WL_NUM_CHANNELS; c++)
            assert(rx.samples[s].ch[c] == tx.samples[s].ch[c]);
    printf("[PASS] test_ramp\n");
}

void test_vbat_exhaustive(void) {
    for (uint16_t v = 0; v < 4096; v++) {
        uint8_t b[2];
        WL_VBAT_ENCODE(b, v);
        assert(WL_VBAT_DECODE(b) == v);
    }
    printf("[PASS] test_vbat_exhaustive (4096 values)\n");
}
```

**Pass criteria:** All asserts pass. Zero tolerance for codec mismatch
between BSAU (Cortex-M4, little-endian) and CPCU (Cortex-A76, little-
endian).

---

### B.2.2 TB-201: Sequence Gap Detector

```python
def detect_gaps(seq_list):
    gaps = []
    expected = seq_list[0]
    for i, s in enumerate(seq_list):
        if s != expected:
            gap = (s - expected) & 0xFF
            gaps.append((i, gap))
        expected = (s + 1) & 0xFF
    return gaps

assert detect_gaps([0,1,2,3,4]) == []                      # No gaps
assert detect_gaps([0,1,3,4,5]) == [(2, 2)]                # 1 lost at pos 2
assert detect_gaps([0,1,5,6,7]) == [(2, 4)]                # 3 lost at pos 2
assert detect_gaps([253,254,1,2]) == [(2, 3)]              # Wrap-crossing gap
assert detect_gaps(list(range(0,256)) + [0,1]) == []       # Full wrap, no gaps
print("[PASS] TB-201: Sequence gap detector")
```

---

### B.2.3 TB-202: Timestamp Jitter Analyzer

```python
import numpy as np

def analyze_jitter(timestamps, nominal_dt=1000):
    dt = np.diff(timestamps.astype(np.int32))
    dt = dt & 0xFFFF
    dt = np.where(dt > 32768, dt - 65536, dt)
    return {
        'mean_dt': np.mean(dt),
        'std_dt':  np.std(dt),
        'min_dt':  np.min(dt),
        'max_dt':  np.max(dt),
        'outliers': np.sum(np.abs(dt - nominal_dt) > 500),
    }

# Synthetic: 10 000 packets at exactly 1000 µs apart
ts = np.arange(0, 10_000_000, 1000, dtype=np.uint16)
stats = analyze_jitter(ts)
assert abs(stats['mean_dt'] - 1000) < 1
assert stats['std_dt'] < 1
assert stats['outliers'] == 0

# Inject a single 5 ms stall at packet 501
ts_j = ts.copy()
ts_j[501:] += 5000
stats_j = analyze_jitter(ts_j)
assert stats_j['max_dt'] > 4000
assert stats_j['outliers'] >= 1
print("[PASS] TB-202: Timestamp jitter analyzer")
```

---

### B.2.4 TB-203: Timestamp Wrap Handling

```python
def compute_dt(ts_prev, ts_curr):
    return (ts_curr - ts_prev) & 0xFFFF

assert compute_dt(65530, 994) == 1000     # Normal wrap
assert compute_dt(65535, 999) == 1000     # Wrap at boundary
assert compute_dt(0, 1000)     == 1000    # No wrap
assert compute_dt(1000, 2000)  == 1000    # No wrap
print("[PASS] TB-203: Timestamp wrap handling")
```

---

### B.2.5 TB-204: Battery Voltage Reconstruction

```python
def reconstruct_vbatt(adc_raw, vdda=3.3):
    v_adc = adc_raw * vdda / 4095
    return v_adc * 2     # 100k/100k divider

tests = [
    (4095, 6.600, 'OK'),
    (2048, 3.300, 'OK'),
    (1861, 2.998, 'LOW'),
    (1675, 2.699, 'CRITICAL'),
    (0,    0.000, 'CRITICAL'),
]
for adc, expected_v, expected_level in tests:
    v = reconstruct_vbatt(adc)
    assert abs(v - expected_v) < 0.01

print("[PASS] TB-204: Battery voltage reconstruction")
```

---

### B.2.6 TB-205: vbat_raw Encode/Decode (Exhaustive)

Covered by the `test_vbat_exhaustive()` check in TB-200. All 4096
12-bit values must round-trip exactly through `WL_VBAT_ENCODE` /
`WL_VBAT_DECODE`.

---

### B.2.7 TB-206: Link Quality State Machine

```python
class LinkQuality:
    GOOD, DEGRADED, POOR = 0, 1, 2
    def __init__(self, window=100):
        self.window, self.retry_buf, self.loss_count, self.state = \
            window, [], 0, self.GOOD
    def update(self, tx_retry, seq_gap):
        self.retry_buf.append(tx_retry)
        if len(self.retry_buf) > self.window:
            self.retry_buf.pop(0)
        self.loss_count += max(0, seq_gap - 1)
        avg = sum(self.retry_buf) / len(self.retry_buf)
        if self.loss_count > 10 or avg > 5:       self.state = self.POOR
        elif avg > 1.0:                            self.state = self.DEGRADED
        elif avg < 0.5 and self.loss_count == 0:   self.state = self.GOOD

lq = LinkQuality()
for _ in range(1000):  lq.update(0, 1);  # clean → GOOD
assert lq.state == LinkQuality.GOOD
for _ in range(200):   lq.update(3, 1);  # retries climb → DEGRADED
assert lq.state == LinkQuality.DEGRADED
for _ in range(50):    lq.update(10, 3); # losses → POOR
assert lq.state == LinkQuality.POOR
lq.loss_count = 0
for _ in range(200):   lq.update(0, 1);  # recovery → GOOD
assert lq.state == LinkQuality.GOOD
print("[PASS] TB-206: Link quality state machine")
```

---

## B.3 Tier 3 — End-to-End Integration

Both boards powered, within radio range. CPCU receiver running on
Core 3.

### B.3.1 TB-300: First Packet Reception

| Field | Value |
|-------|-------|
| BSAU mode | `BSAU_MODE_RELEASE` or `BSAU_MODE_DEBUG` |
| Distance | 1 metre, line-of-sight |

**Pass criteria:**
- Packet received within 500 ms of BSAU power-on.
- `WL_FLAG_FIRST_PACKET` (bit 7) is set.
- Packet unpacks without error.

```python
timeout = time.time() + 2.0
while time.time() < timeout:
    if nrf_data_available():
        pkt = wl_unpack(nrf_read_payload())
        assert pkt.flags & 0x80, "FIRST_PACKET not set"
        print(f"[PASS] TB-300: seq={pkt.seq}")
        break
else:
    print("[FAIL] TB-300: no packet within 2 s")
```

---

### B.3.2 TB-301: Sustained Throughput (60 seconds)

```
Expected in 60 s:  60 000 packets
Packet loss:       < 0.1 %   (< 60 lost)
Mean tx_retry:     < 0.5
Throughput:        > 990 pkt/s average
```

---

### B.3.3 TB-302: Packet Integrity (Zero Corruption)

On every received packet, verify all fields stay in valid ranges:

```python
violations = 0
for pkt in all_received_packets:
    for s in range(2):
        for c in range(8):
            if pkt.samples[s].ch[c] > 4095: violations += 1
    if pkt.vbat_raw  > 4095: violations += 1
    if pkt.tx_retry  > 15:   violations += 1
    if pkt.pkt_loss  > 15:   violations += 1
assert violations == 0, f"{violations} field violations"
```

**Pass criteria:** Zero violations across 60 000 packets.

---

### B.3.4 TB-303: Timestamp Continuity

```python
dts = [(p[i].timestamp - p[i-1].timestamp) & 0xFFFF
       for i in range(1, len(p))]
assert abs(np.mean(dts) - 1000) < 10     # µs
assert np.std(dts) < 50                   # µs
assert np.max(dts) < 15000                # µs
```

**Pass criteria:** mean ≈ 1000 µs, stdev < 50 µs, max < 15 ms.

---

### B.3.5 TB-304: Battery Telemetry Accuracy

Supply the BSAU through a divider fed by a bench PSU set to known
voltages. Record `vbat_raw` on the CPCU side.

**Pass criteria:**

```
|V_reconstructed − V_measured| < 50 mV at each test point.
At 3.0 V: flags show WL_BATT_LOW.
At 2.7 V: flags show WL_BATT_CRIT.
```

---

### B.3.6 TB-305: Link Degradation Response

| Distance | Expected tx_retry | Expected loss | Expected state |
|----------|-------------------|---------------|----------------|
| 1 m | ~0 | 0 % | GOOD |
| 3 m | 0–0.5 | < 0.1 % | GOOD |
| 5 m | 0.5–2 | < 1 % | GOOD–DEGRADED |
| 10 m | 2–5 | 1–5 % | DEGRADED |
| 3 m + body obstruction | 1–3 | 0.5–2 % | DEGRADED |

**Pass criteria:** `tx_retry` increases monotonically with distance.
Loss rate < 5 % at 10 m.

---

### B.3.7 TB-306: EMG Signal Fidelity

Function generator: 100 Hz sine, 1 Vpp, 1.65 V offset → PA0.

```python
ch0 = np.array([p.samples[s].ch[0] for p in pkts for s in range(2)])
fft = np.abs(np.fft.rfft(ch0 - ch0.mean()))
freqs = np.fft.rfftfreq(len(ch0), d=1/2000)
peak = freqs[np.argmax(fft[1:]) + 1]
snr  = 20 * np.log10(fft.max() / np.median(fft[1:]))
assert abs(peak - 100) < 2
assert snr > 20
```

**Pass criteria:** Peak at 100 ± 2 Hz, SNR > 20 dB above noise floor.
Repeat at 50, 200, 500, 900 Hz.

---

### B.3.8 TB-307: 8-Channel Isolation (Crosstalk)

Signal on PA0, every other channel grounded through 1 kΩ to 1.65 V.

**Pass criteria:** > 40 dB isolation from ch0 to each of ch1-ch7.

---

### B.3.9 TB-308: Stress Test (1 Hour Continuous)

```
Duration:           3600 s
Expected packets:   3 600 000
Loss rate:          < 0.1 %
Max single gap:     < 20 consecutive packets
ADC_OVRN flags:     0
TX_SAT flags:       0
Max timestamp dt:   < 20 ms
vbat_raw drift:     < 50 counts (USB powered)
Unexpected FIRST_PACKET after initial: 0 (no spurious resets)
```

---

### B.3.10 TB-309: Power-On Sequence Robustness

Power-cycle the BSAU 20 times with CPCU listening (2 s off between
cycles). Each cycle:

- First packet within 500 ms.
- `FIRST_PACKET` flag set.
- No permanent link failure.
- CPCU never requires a restart.

---

## B.4 Test equipment summary (per test)

| Equipment | Tests | Purpose |
|-----------|-------|---------|
| UART terminal (921600) | TB-100 – TB-108 | LOG capture |
| SerialPlot | TB-101 | Binary ADC stream |
| Function generator | TB-103, TB-306, TB-307 | Known test signals |
| Oscilloscope | TB-306 | Reference waveform |
| Multimeter / INA219 | TB-108, TB-304 | Current / voltage |
| Bench power supply | TB-304 | Controlled voltage |
| CPCU (Pi + NRF) | TB-300 – TB-309 | Radio receive end |

---

## B.5 Test execution order (board bring-up)

```
Phase A — Silicon alive (no radio):
  TB-100  ADC Calibration        → confirms ADC, DMA, TIM6
  TB-101  ADC Channel Liveness   → all 9 physical channels readable
  TB-102  ASCII CSV              → UART + data formatting
  TB-104  Packet Codec           → WL_Pack/Unpack logic

Phase B — Radio alive (no receiver):
  TB-105  NRF Self-Test Suite    → SPI chain and NRF registers
  TB-106  NRF TX Stress Loop     → RF state machine stability

Phase C — Full BSAU pipeline:
  TB-103  DFT Frequency Verify   → 2 kHz sample rate accuracy
  TB-107  DEBUG Mode Pipeline    → end-to-end BSAU data path
  TB-108  RELEASE Power Measure  → power budget matches spec

Phase D — CPCU software (no BSAU hardware):
  TB-200  Codec Cross-Platform   → Pack/Unpack identical on CPCU
  TB-201  Sequence Gap Detector
  TB-202  Timestamp Jitter
  TB-203  Timestamp Wrap
  TB-204  Battery Voltage
  TB-205  vbat_raw Exhaustive
  TB-206  Link Quality FSM

Phase E — Integration (both boards):
  TB-300  First Packet
  TB-301  Sustained Throughput
  TB-302  Packet Integrity
  TB-303  Timestamp Continuity
  TB-304  Battery Telemetry
  TB-305  Link Degradation
  TB-306  EMG Signal Fidelity
  TB-307  8-Channel Isolation
  TB-308  Stress Test (1 hour)
  TB-309  Power-On Robustness
```

---

## B.6 Regression policy — exhaustive (by changed file)

| Trigger | Required tests | Time |
|---------|----------------|------|
| Any code change | TB-104 (codec), TB-105 (NRF regs) | 5 s |
| `wireless_packet.c/h` change | TB-104 + TB-200 | 10 s |
| `nrf24l01.c/h` change | TB-105 + TB-106 (5 min stress) | 5 min |
| ADC config change | TB-100 + TB-101 + TB-103 | 2 min |
| Pre-release candidate | All Tier 1 + Tier 2 + TB-301/302/303 | 10 min |
| Release sign-off | All tiers including TB-308 | 1.5 h |
