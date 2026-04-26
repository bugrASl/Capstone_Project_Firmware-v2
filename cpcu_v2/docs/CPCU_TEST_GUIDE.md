# CPCU Test Guide — v3.4

**Author:** bugrASl
**Date:** April 2026
**Audience:** anyone who needs to verify that some part of CPCU actually
works — from a brand-new team member running their first `./test_codec` to a
seasoned engineer debugging a regression before a demo.

If a step in this guide says "PASS criterion: …", that is what a green test
looks like. If you see something *different* from what's described, jump to
the **What it means when it fails** note right under that step — don't guess.

---

## Table of Contents

1.  [Test philosophy (why we test in this order)](#1-test-philosophy-why-we-test-in-this-order)
2.  [Quick start](#2-quick-start)
3.  [Phase 1 — pure software (no hardware)](#3-phase-1--pure-software-no-hardware)
4.  [Phase 2 — IPC bridge validation](#4-phase-2--ipc-bridge-validation)
5.  [Phase 3 — Pi hardware verification](#5-phase-3--pi-hardware-verification)
6.  [Phase 4 — integration (both boards)](#6-phase-4--integration-both-boards)
7.  [Phase 5 — qualification (endurance + recovery)](#7-phase-5--qualification-endurance--recovery)
8.  [Test equipment](#8-test-equipment)
9.  [Regression policy (what to re-run after a change)](#9-regression-policy-what-to-re-run-after-a-change)
10. [Interpreting testbench numbers (reference)](#10-interpreting-testbench-numbers-reference)
11. [Glossary cross-reference](#11-glossary-cross-reference)

---

## 1. Test philosophy (why we test in this order)

Tests are organised in two axes:

-   **Automated vs interactive.** Automated tests script a known input and
    check the output against expected values — they either PASS or FAIL and
    can run in CI. Interactive tests open an ncurses TUI and ask a human to
    visually confirm something (e.g. "did the servo actually move?").
-   **Dependency layer.** Phase 1 needs nothing. Phase 2 needs the kernel
    running (which creates shared memory). Phase 3 needs the Pi's real
    peripherals. Phases 4–5 need the BSAU transmitting too.

**Always run tests bottom-up.** If Phase 1 fails, nothing downstream will
work — the codec is broken, or the ring buffer has a race condition, and any
Phase 4 number you read afterwards is lying to you. Fix Phase 1 first.

```
AUTOMATED (run_tests.sh 1 2 3):
  Phase 1:  Pure software — no hardware, no shared memory, any Linux/Mac
  Phase 2:  IPC validation — needs cpcu_kernel running
  Phase 3:  Pi hardware — needs SPI, I2C, core isolation, PCA9685

INTERACTIVE (run_tests.sh <n>):
  pca:          PCA9685 servo calibration TUI (direct I2C, no kernel)
  signal:       End-to-end signal integrity TUI (needs kernel + BSAU)
  signal-demo:  Signal integrity TUI with synthetic data (no hardware)

BUILT-IN DEMO (no hardware, no shared memory):
  ./cpcu_tui --demo              All 4 TUI pages with synthetic data
  ./signal_testbench --demo      Waveform + Goertzel with synthetic sine
  ./pca_testbench                Auto dry-run if PCA9685 not detected
```

**What "demo mode" is for:** demo mode fakes a full sensor ring (100 packets
per frame at 10 Hz = 1000 pkt/s, the real rate) so the TUIs have live-looking
data with no hardware attached. Use it for learning the key bindings,
documenting the UI, or proving that a TUI change didn't break rendering,
*without* standing up the whole pipeline.

---

## 2. Quick start

```bash
chmod +x run_tests.sh

# Automated — run in order
./run_tests.sh 1            # Phase 1: software tests (any machine)
./run_tests.sh 1 2          # Phase 1 + 2: software + IPC
./run_tests.sh 1 2 3        # All automated phases (Pi with hardware)

# Interactive — launch TUI tools
./run_tests.sh pca          # Servo calibration (needs I2C)
./run_tests.sh signal       # Signal integrity (needs kernel + BSAU)
./run_tests.sh signal-demo  # Signal integrity (synthetic, anywhere)

# Demo — preview TUIs without any hardware
./cpcu_tui --demo           # Full 4-page dashboard
./signal_testbench --demo   # Waveform analysis
```

---

## 3. Phase 1 — pure software (no hardware)

These tests run on any machine with GCC and Python 3. No Pi, no shared memory,
no peripherals. If any of these fail, **nothing downstream will work** — the
failure is telling you that a fundamental assumption about how data moves
through the system is wrong.

### 3.1 TB-C100..C106 — packet codec round-trip

```bash
cd build
make test_codec
./test_codec
```

**What it checks, concretely:**

-   Metadata (`seq`, `vbat_raw`) round-trips unchanged through pack/unpack.
-   12-bit packing is exact for all 8 channels at all 4096 possible values.
-   Boundary values `0x000` and `0xFFF` don't overflow into neighbouring
    fields.
-   Exhaustive `vbat_raw` sweep (4096 values) — catches off-by-one in the
    bit-shift logic.
-   Sequence gap detector handles wrap-around (`seq 0xFFFF → 0x0000`).
-   Battery voltage reconstruction:
    `V = raw / 4095 * Vref * DIVIDER_RATIO`.

**Pass criterion:** `RESULTS: 7 PASS, 0 FAIL`.

**What it means when it fails:**

-   Any 12-bit packing FAIL → `wireless_packet.c` bit-shift arithmetic is
    wrong; radio payloads will decode to garbage. Do not proceed.
-   vbat_raw FAIL at a specific value → check `WL_Pack`/`WL_Unpack` for that
    bit position.
-   Sequence wrap FAIL → `io_seq_gaps` counter on Page 2 will be meaningless.

### 3.2 TB-DSP — DSP pipeline validation

```bash
python3 test/test_dsp_pipeline.py
```

**What it checks:**

-   Feature extraction on a known sine wave produces the expected MAV, RMS,
    zero-crossing rate, and waveform-length values.
-   Feature column order matches the order the model was trained on — a
    silent swap here causes "wrong gestures on unchanged hardware" bugs.
-   Causal band-pass (the one we run live) matches the offline (zero-phase)
    filter within 30 % — this is the bar below which we'd need to retrain.
-   Gesture-to-servo map has an entry for every gesture the model can emit
    (REST, fist, pinch, point, thumbs-up, wrist-rotate, open).
-   Noise gate attenuates sub-threshold inputs.
-   Model file loads without version errors.

**Pass criterion:** all assertions pass.

**What it means when it fails:**

-   "Causal vs offline > 30 % error" → retrain the model using the same
    causal-filter pipeline we deploy. The model has learned features that
    don't exist at runtime.
-   "Feature columns mismatch" → someone reordered the feature vector;
    re-export the model or fix `cpcu_dsp.py`.
-   "Model file load error" → scikit-learn or numpy version mismatch.

### 3.3 TB-SAFETY — safety FSM (automated, no hardware)

```bash
./safety_testbench
# or (runs as part of the full Phase 1 sweep):
ctest -R safety_fsm --output-on-failure
```

Exercises the safety finite-state-machine without any hardware. The
binary drives the FSM through its state-transition matrix, injecting
each failure mode and verifying the expected response.

**Seven test groups, 31 individual checks, all must PASS:**

1.  **Happy path** — no faults; FSM stays `RUNNING` forever, counters
    are monotonic, no spurious SAFE entries.
2.  **Radio-loss timeout** — simulate silent radio for varying durations.
    FSM must go `RUNNING → DEGRADED` at 750 ms and `DEGRADED → SAFE`
    at 1500 ms in degraded (= 2250 ms total). Recovery: after packets
    resume, `DEGRADED → RUNNING` after 10 consecutive good packets.
3.  **Low battery** — drive `vbat_raw` to encode 2.58 V (below
    `SAFETY_VBAT_CRITICAL_V = 2.7 V`). FSM must trip to `SAFE` within
    one tick and set `err_flags |= ERR_BATTERY`.
4.  **Seq gap storm** — inject 2-packet gap bursts until
    `link.loss_rate > 0.05` over the 1 k-packet window. FSM trips
    `SAFE`. Tolerance test: single-packet gaps (gap ≤ 1) do **not**
    trip — absorbed by `SAFETY_SeqGap`.
5.  **Ring overflow** — push `io_ring_overflows` to ≥ 150 (delta of 150
    above the baseline 0, threshold 100). FSM trips `SAFE`. *(This is
    one of the 31 testbench checks and continues to pass against the
    v2.3 logic because the fault threshold is now applied to the
    delta-since-baseline, which equals the cumulative count when
    starting from 0.)* **Suggested v2.3 follow-up test (manual):**
    keep the counter stable for `SAFETY_RING_RECOVER_MS = 5000 ms`
    and verify `ring.faulted` clears and re-baselines, so a subsequent
    +101 burst re-triggers cleanly. This proves the pre-v2.3 latching
    bug — where a single historical burst kept the ring fault asserted
    forever — is gone.
6.  **I²C error streak** — simulate 5 consecutive I²C failures. FSM
    trips `SAFE`. Recovery: one successful I²C transaction clears
    both the counter **and** the fault flag.
7.  **State-transition graph** — drive the FSM through every legal
    edge (`INIT → RUNNING → DEGRADED → RECOVERING → RUNNING → SAFE`)
    and verify no illegal transitions occur under any input.

**Pass criteria:** stdout shows `[PASS]` for all 33 checks (was 31 pre-v2.3
— TB-SAF02 was extended with `e/f/g` for the v2.2 SAFE-recovery path that
the original test couldn't see), exit code is 0. On fail the binary prints
which group and which check, with expected vs actual state.

**Interactive version:** to exercise the same faults live with visual
feedback, run `./cpcu_tui --demo` and use `F` / `B` / `G` / `O` / `I`
to inject faults, `R` to reset. Each fault triggers a red
`[INJ:RADIO_FREEZE]` / `[INJ:BATT_LOW]` / etc. banner in the bottom
footer; Page 5 Health goes red on the affected subsystem row; overall
verdict goes `DEGRADED`; Page 1's state row flips to `SAFE` on the
expected threshold.

```bash
./run_tests.sh safety-demo
# Shortcut: launches cpcu_tui --demo after printing a quick reminder
# of the fault-injection hotkeys.
```

### 3.4 TUI demo validation (visual, no hardware)

```bash
./cpcu_tui --demo
```

Press `1`/`2`/`3`/`4`/`5`/`6`/`7` to verify all pages render correctly. Press
`q` to quit. You can also exercise the demo waveform selector (`w`, `[`, `]`)
and the fault-injection keys (`F`, `B`, `G`, `O`, `I`, `R`) without
touching any hardware.

This validates that the TUI code compiles, runs, and that the demo-mode
packet generator feeds the ring buffer fast enough to keep pages 3 and 4
live.

**Pass criteria:**

-   Splash screen appears, disappears on keypress (or after ~3 s).
-   Page 1 shows a moving EMG bar animation, a rotating gesture name, and
    a `HEALTH` rolled-up banner at the top with six coloured pills and
    an overall verdict (`NOMINAL` / `WARNING` / `DEGRADED`).
-   Page 2 shows non-zero `pkts/s` (target ~1000), a fresh last-packet
    hex dump each second, and a decoded BSAU-flags banner (`OK` in
    clean demo).
-   Page 3 shows non-zero `Inferences`/s and `Motor cmds`/s rates ticking.
    Cmd age stays below 100 ms.
-   Page 4 shows 8 rolling waveforms that visibly update. Per-channel
    Hz / Vpp / Vrms values populate. With `w` you can cycle through
    eight waveforms; each renders recognisably (square has flat top/
    bottom plateaus, triangle has diagonal ramps, noise is scattered
    dots with no pattern, ECG shows sharp periodic R-spikes, etc.).
-   Page 5 (HEALTH) shows ten subsystem rows each with a `[OK]` /
    `[WARN]` / `[FAULT]` pill, a detail column explaining the check,
    and a summary line at the top tallying `N OK | N WARN | N FAULT`.
-   Page 6 (DATASET) shows the label picker, RAW/FILTERED toggle, and
    capture-state banner. With LEFT/RIGHT you can cycle the label;
    `s`/SPACE starts a capture (synthetic-packet path is identical to
    the real-packet path, so the file produced under `--demo` is
    byte-format-valid for the DSP/AI team's loader); `r` cancels and
    deletes the partial file. Capture continues even if you switch to
    another page.
-   Page 7 (CONFIG) renders four static spec sections (BSAU/CPCU,
    wireless/IPC, motor/DSP, build info) — nothing changes over time
    because this is a compile-time reference page. *(v3.4: this page
    moved from key 5 to key 7.)*
-   Resizing the terminal window reflows the layout on the next frame.
-   Pressing `F` (inject radio freeze) causes the Health banner on
    Page 1 to turn yellow then red, verdict goes `WARNING` → `DEGRADED`,
    and after ~2.25 s the state row flips to `SAFE`.
-   Pressing `O` (inject ring overflow) trips `[FAULT]` on the IPC ring
    row of Page 5, then **clears automatically** ~5 s after the burst
    stops — this is the v2.3 recovery path; if Page 5 stays red
    indefinitely you're looking at the pre-v2.3 latching bug.
-   Pressing `R` immediately clears all injected faults **and** zeros
    every accumulated counter (seq gaps, inferences, SAFE entries,
    batches, max latency, underflows, drops, overflows). Every page
    snaps back to a clean-boot look. If after pressing `R` the
    counters still show non-zero values that were there before, you're
    running an old build — rebuild.

**What it means when it fails:** demo-mode pumps 100 packets per tick; if
the waveforms look flat, the ring-buffer push path is broken. If the
Health banner stays grey or shows `WARN` on a subsystem that should be
OK, inspect the corresponding `atomic_load` in `draw_page_overview`'s
HEALTH block — the threshold comparison is probably off.

```bash
./signal_testbench --demo
```

**Pass criteria:** 8 channels all show a clean ~100 Hz waveform (or
whatever the current demo wave is — header bar shows
`[DEMO <WAVE> <FREQ>Hz]`). Goertzel detects the dominant at the expected
frequency in every channel. SNR > 20 dB on sine/square/triangle,
< 10 dB on noise (by design — noise is supposed to be unclassifiable
as a tone). Press `w` to cycle waveforms and verify each one renders
as expected through the line-trace grid. TAB toggles between 2-column
8-mini-plot grid and a full-screen single-channel detail view.

**If the 8 mini-plots look cramped or only fill the top half of the
terminal**: you're running an older build that hardcoded `g_mini_h=4`.
Rebuild with the current `signal_testbench.c` — layout now scales
vertically with `g_term_h`.

### 3.5 Note on `signal_testbench` (what it is, what it isn't)

`signal_testbench` plots the **raw ADC stream** straight off the IPC ring.
It is a physical-layer test: function-gen → BSAU ADC → NRF TX → NRF RX →
codec → IPC ring → this TUI.

It does **not** apply the DSP (no band-pass, no envelope, no RMS smoothing,
no classifier). For the DSP output, look at `cpcu_tui` Page 3 and Page 4.

Why this matters: if `signal_testbench` shows a clean sine but `cpcu_tui`
Page 4 looks noisy, the problem is in Python DSP — not in the radio link.

---

## 4. Phase 2 — IPC bridge validation

Needs `cpcu_kernel` running to create `/dev/shm/cpcu_ipc`.

### 4.1 Start kernel temporarily

```bash
cd build
./cpcu_kernel &
KERNEL_PID=$!
sleep 2
```

### 4.2 TB-IPC — IPC bridge offset validation

```bash
python3 test/test_ipc_bridge.py
```

**What it checks:**

-   Struct sizes match: `ControlBlock = 192`, `SensorEntry = 64`,
    `MotorCommand = 128`. These are the sizes the Python bridge assumes
    when it `mmap`s shared memory — if C and Python disagree, Python will
    read garbage without any compile-time warning.
-   Section offsets are sequential (no padding surprises between
    `ControlBlock`, the diagnostics block, the sensor ring, and the motor
    seqlock region).
-   Hot fields (`head`, `tail`) are on separate cache lines — so the
    producer and consumer don't ping-pong the same cacheline.
-   Magic number and version fields are readable at the expected offset.
-   DSP-ready flag round-trip works.
-   Motor-command write goes through the seqlock correctly (reader sees
    either the old or the new value, never a torn half-half).
-   DSP export write works.
-   Ring buffer `pop` on an empty ring returns empty (doesn't block, doesn't
    segfault).

**Pass criterion:** all PASS.

**What it means when it fails:** the Python bridge and C header are out of
sync. Either a C struct has a new field and Python doesn't know, or a member
was reordered. Regenerate the Python bridge constants from the C header.

### 4.3 Cleanup

```bash
kill $KERNEL_PID
wait $KERNEL_PID 2>/dev/null
```

---

## 5. Phase 3 — Pi hardware verification

Needs a Pi that has already been through `setup_pi.sh`, rebooted, and has
the PCA9685 + NRF24L01+ wired.

### 5.1 Automated checks

```bash
./run_tests.sh 3
```

**What it checks, one at a time:**

-   `isolcpus == "1-3"` — without this, `cpcu_io` can't run with RT
    guarantees.
-   `/dev/spidev0.0` exists — NRF24L01+ can't be reached without it.
-   `/dev/i2c-1` exists — PCA9685 can't be reached without it.
-   `i2cdetect -y 1` sees `0x40` — the PCA9685 is physically there and
    responding.
-   PCA9685 `PRESCALE` register is readable — the I²C read path works, not
    just writes.
-   PCA9685 init smoke-test succeeds — configures MODE1/MODE2 and the
    prescaler without error (skipped if `pca_testbench` isn't built).
-   CPU clock ≥ 2.8 GHz — the overclock actually stuck.
-   CPU temp < 80 °C — thermal headroom is healthy.

### 5.2 Interactive servo test

```bash
./run_tests.sh pca
```

**Controls:**

```
  UP/DOWN                select servo
  LEFT/RIGHT             +/- 5 us
  PgUp / PgDn            +/- 50 us
  m / M                  jump to min / max
  n                      neutral selected servo
  N                      neutral all
  0                      kill PWM (LED_OFF on selected)
  s                      toggle slew smoother on/off
  A                      write all channels at once (PCA_SetAllServos)
  r                      read back MODE1/MODE2/PRESCALE live
  q                      quit
```

**Command-line flags:**

-   `--min A,B,C,D,E,F` / `--max A,B,C,D,E,F` — override the default per-
    channel pulse-width limits (useful when calibrating a new servo that
    mechanically binds before reaching the library default).
-   `--smooth` — start with the slew-rate smoother already on.

The testbench talks **directly** to PCA9685 over I²C — no kernel, no IPC, no
DSP. If PCA9685 is not detected it runs in dry-run mode: the TUI works, no
hardware writes happen. This lets you practice the UI on a plain laptop.

**Verify:**

-   Each servo moves to min/max without mechanical binding.
-   Neutral command centres the servo physically.
-   The TUI's slider position reflects the actual servo pose.
-   With `s` (smoother on), setting a new target makes the servo slew at a
    steady rate rather than snapping.
-   `r` shows sane MODE1/MODE2 values (AI bit on, SLEEP off, INVRT
    typically off for active-high LEDs).

**What it means when it fails:**

-   Servo doesn't move but TUI says it sent the command → check the 6V rail
    with a multimeter, and check that the PCA9685's OE pin is not pulled
    high (OE-high disables all outputs).
-   Servo moves but min/max values differ from library defaults → pass
    `--min`/`--max` with the measured limits and commit them to
    `cpcu_pca9685.h`.

### 5.3 Shutdown-and-cleanup check

Press `q` in `pca_testbench`. The log should show: all channels neutralised,
300 ms wait, then `PCA_AllOff` (drives every channel to `LED_OFF`). This is
the same sequence `cpcu_io` runs on clean shutdown, so passing here gives
you confidence the main binary will release the servos on SIGINT too.

---

## 6. Phase 4 — integration (both boards)

Needs BSAU powered and transmitting, within radio range of the Pi's NRF.

### 6.1 TB-INT01 — first packet

```bash
sudo systemctl start cpcu
journalctl -u cpcu -f
```

**Expected within 2 seconds:**

```
[IO]  INFO  === CPCU I/O Controller (Core 3) v2.2 ===
[IO]  INFO  Ready (NRF=OK PCA=OK). Entering loop.
[IO]  INFO  pkts=1000 gaps=0 ring=5 state=RUNNING fault=OK nrf_sr=0x0E motion=IDLE
```

**What the telemetry fields mean:**

-   `pkts` — total packets received by `cpcu_io` since start.
-   `gaps` — detected sequence-number discontinuities. > 10/minute is
    suspicious.
-   `ring` — how many sensor entries are sitting unread in the ring buffer.
    A few is fine; close to the ring size (1024) means the DSP side is
    falling behind.
-   `state` — safety FSM: `INIT`/`RUNNING`/`DEGRADED`/`SAFE`/`FAULT`.
-   `fault` — last fault code; `OK` means no fault.
-   `nrf_sr` — live `STATUS` register read of the NRF. `0x0E` is the
    post-power-up idle state (no RX/TX/MAX_RT flags).
-   `motion` — `IDLE` if all smoothers are settled, `MOVING` if any servo
    is still slewing to its target.

### 6.2 Signal integrity verification

Connect a function generator (100 Hz sine, ~0.6 V amplitude, 1.65 V DC
offset) to all 8 EMG inputs:

```bash
./run_tests.sh signal
```

Press TAB for all-channel view.

**Pass criteria, per channel:**

-   Clean sinusoid visible in the rolling plot (no flat-top clipping, no
    railing).
-   Dominant frequency = 100 Hz (Goertzel report).
-   `Vpp ≈ 1.2 V` (twice the generator amplitude).
-   `SNR > 20 dB` (clear signal, little noise).
-   `DC Offset ≈ 1.65 V` (mid-rail, as set).
-   `ADC min/max` span stays constant — if it creeps, the BSAU's analog
    front-end may be drifting thermally.

**Note — this is the RAW ADC stream.** No filtering. A 100 Hz sine on the
function generator should produce a nearly-identical sine on screen (apart
from ADC quantisation). If the waveform looks different from the input, the
signal chain — not DSP — is the problem.

### 6.3 Live TUI monitoring

```bash
./cpcu_tui
```

Press `1` for overview, `2` for radio deep-dive, `3` for DSP/AI detail, `4`
for waveforms.

**Page 2 (Radio) pass criteria:**

-   packet rate ~1000/s,
-   seq gaps < 10/min,
-   loss < 0.01 %.

**Page 3 (DSP/AI) pass criteria:**

-   gestures change with muscle activity (doing a fist should produce the
    `fist` class),
-   confidence on the active class > 65 %,
-   `REST` is returned when the forearm is relaxed.

**Page 4 (Waveforms) pass criteria:**

-   waveforms update at 10 Hz,
-   Vpp numbers look plausible for real EMG (typically tens to hundreds of
    mV at the ADC after gain — not railing at 3.3 V).

### 6.4 TB-INT05 — safety timeout

Power off the BSAU during operation.

**Expected sequence (visible in Page 1 and in `log_io.csv`):**

1.  `state = RUNNING` with packet rate dropping.
2.  Within ~750 ms: `state -> DEGRADED`.
3.  Servos snap to neutral (1500 µs) via `SMOOTH_Snap`.
4.  After ~1500 ms total: `state -> SAFE`; `PCA_AllOff` is called so servos
    go limp.
5.  When BSAU comes back: `state -> INIT -> RUNNING` automatically.

**What it means when it fails:**

-   No transition to `SAFE` → watchdog in `cpcu_io` isn't firing; check
    `last_packet_time` handling.
-   Servos don't neutralise → `SMOOTH_Snap` didn't run, or PCA writes are
    erroring out.

---

## 7. Phase 5 — qualification (endurance + recovery)

### 7.1 One-hour endurance

```bash
sudo systemctl start cpcu
# Run for 1 hour with BSAU transmitting
# Monitor: ./cpcu_tui (Page 1 overview)
```

**Pass criteria:**

-   total pkts ≈ 3,600,000 (1 kHz × 3600 s),
-   seq gaps < 0.01 % of total,
-   ring overflows = 0,
-   CPU temp stable below 75 °C,
-   no process restarts (kernel log shows no `respawning` events),
-   Python process RSS memory stable (no leak).

### 7.2 Process recovery

```bash
sudo kill -9 $(pidof cpcu_io)
```

**Pass criteria:**

-   `cpcu_kernel` detects the death within 2 s,
-   respawns `cpcu_io`,
-   radio re-establishes within 1 s,
-   total recovery time < 5 s,
-   servos are driven to neutral during the gap (safety FSM enters `SAFE`
    while `cpcu_io` is dead because the watchdog file lock is released, then
    `DEGRADED`/`RUNNING` when the new `cpcu_io` re-claims it).

---

## 8. Test equipment

| Equipment              | Tests                     | Purpose                       |
|------------------------|---------------------------|-------------------------------|
| Any PC                 | Phase 1, demo modes       | Software validation           |
| Pi + PCA9685           | Phase 3, `pca_testbench`  | Servo calibration             |
| Pi + PCA + NRF         | Phase 3 full              | Hardware validation           |
| Function generator     | `signal_testbench`        | Known test signals            |
| BSAU + electrodes      | Phase 4–5                 | Live EMG integration          |
| Oscilloscope (optional)| Debug physical layer      | See PWM on scope, check NRF SPI traffic |

---

## 9. Regression policy (what to re-run after a change)

| Changed file                | Required re-tests                       |
|-----------------------------|-----------------------------------------|
| `wireless_packet.c/h`       | TB-C100 + BSAU TB-104                   |
| `nrf24l01_linux.c/h`        | TB-HW06, TB-INT01                       |
| `cpcu_ipc.c/h`              | TB-IPC, TB-INT02                        |
| `cpcu_safety.c/h`           | TB-INT05, TB-QUAL03                     |
| `cpcu_pca9685.c/h`          | Phase 3, `pca_testbench`                |
| `cpcu_smooth.c/h`           | `pca_testbench`, TB-INT04               |
| `cpcu_log.c/h`              | Every binary once (--log smoke) + tail `/var/log/cpcu/log_*.csv` for 30 s |
| `cpcu_dsp.py`               | TB-DSP, TB-INT03                        |
| `cpcu_tui.c`                | `--demo` validation (all 4 pages, resize test) |
| `signal_testbench.c`        | `--demo` validation + live run with function gen |
| `pca_testbench.c`           | Phase 3 interactive, verify new flags (`--min`, `--max`, `--smooth`) |
| `cpcu_io.c`                 | All of Phase 3 + 4                      |
| `cpcu_kernel.c`             | TB-QUAL03 (kill -9 recovery)            |

---

## 10. Interpreting testbench numbers (reference)

Below is a quick reference for the numbers the testbenches display, so you
can tell at a glance whether something is within tolerance.

**`test_codec`:**
-   All checks PASS or the codec is broken; there is no "degraded" mode.

**`signal_testbench` on a 100 Hz, 0.5 V-amplitude, 1.65 V-DC sine:**
-   Dominant freq: 100 Hz (exact, since 100 is in the Goertzel bin list).
-   Vpp: ~1.00 V (2 × amplitude).
-   DC Offset: ~1.65 V.
-   RMS of the sine (not shown directly): 0.5 × 0.707 ≈ 0.354 V. If you
    compute it manually over the buffer, that is the expected value.
-   SNR: > 20 dB (should be 30 dB+ on a clean bench setup).

**`cpcu_tui` Page 2 telemetry, stable state:**
-   pkts/s: 990–1010 (radio has tiny jitter, ±1 %).
-   seq gaps: 0–2 per minute.
-   ring fill: 0–20 of 1024 (DSP keeps up).
-   `nrf_sr`: `0x0E` or `0x0F` (idle/RX-ready).

**PCA9685 readback (`r` key in `pca_testbench`):**
-   MODE1: `0x21` means AI=1, SLEEP=0, RESTART=0 — the normal running
    config.
-   MODE2: typically `0x04` (OUTDRV=1, totem-pole; INVRT=0).
-   PRESCALE: `0x79` (121) for 50 Hz at 25 MHz internal osc.

---

## 11. Glossary cross-reference

Every unfamiliar term used here (ADC, NRF24L01+, PCA9685, SPI, I²C, SPSC,
seqlock, SCHED_FIFO, isolcpus, DSP, RMS, Goertzel, Vpp, MAV, BSAU, CPCU) is
defined in the [CPCU_RUN_GUIDE.md glossary](CPCU_RUN_GUIDE.md#11-glossary).
