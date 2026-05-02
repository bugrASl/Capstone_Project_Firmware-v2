# Configuration Reference

**Author:** bugrASl
**Date:** April 2026
**Audience:** Anyone changing a value in this project, anyone debugging
"why doesn't my edit take effect?", anyone wiring a new tunable.

This is the single reference for every configuration option in
CPCU + BSAU. It covers the **runtime/compile-time split**, the
**runtime.json schema**, the **`./launch.sh configure` workflow** for
safety thresholds, the **bias-then-clamp** safety guarantee, and a
file-by-file reference of every tunable in the system.

> Originally split across `RUNTIME_CONFIG.md` (architecture) and
> `CPCU_CONFIGURATION.md` (per-knob reference). Merged into one doc in
> v2.7 because the two-doc split forced too many cross-references.

---

## Table of contents

- [TL;DR](#tldr)
- [Part 1 — Architecture](#part-1--architecture)
  - [1. The runtime/compile-time decision](#1-the-runtimecompile-time-decision)
  - [2. The runtime side — `runtime.json`](#2-the-runtime-side--runtimejson)
  - [3. The compile-time side — `./launch.sh configure`](#3-the-compile-time-side--launchsh-configure)
  - [4. Why bias-then-clamp matters](#4-why-bias-then-clamp-matters)
  - [5. SeqLock pattern (for IPC tinkering)](#5-seqlock-pattern-for-ipc-tinkering)
  - [6. Core allocation](#6-core-allocation)
  - [7. Testing](#7-testing)
  - [8. Operating procedure](#8-operating-procedure)
- [Part 2 — Per-file reference](#part-2--per-file-reference)
  - [9. `cpcu_safety.h` — safety thresholds](#9-cpcu_safetyh--safety-thresholds)
  - [10. `cpcu_smooth.h` — servo motion profile](#10-cpcu_smoothh--servo-motion-profile)
  - [11. `cpcu_pca9685.h` — servo mechanical limits](#11-cpcu_pca9685h--servo-mechanical-limits)
  - [12. `cpcu_dsp.py` — DSP/ML pipeline](#12-cpcu_dsppy--dspml-pipeline)
  - [13. `cpcu_io.c` — runtime per-servo config](#13-cpcu_ioc--runtime-per-servo-config)
  - [14. `bsau_config.h` — BSAU mode selection](#14-bsau_configh--bsau-mode-selection)
  - [15. Operator commands — runtime, no recompile](#15-operator-commands--runtime-no-recompile)
  - [16. `./launch.sh` — runtime mode selection](#16-launchsh--runtime-mode-selection)
  - [17. Pi system configuration](#17-pi-system-configuration)
- [Part 3 — Recipes](#part-3--recipes)
  - [18. Common configuration recipes](#18-common-configuration-recipes)
  - [19. What's NOT configurable (and why)](#19-whats-not-configurable-and-why)
- [Part 4 — Calibration round-trip](#part-4--calibration-round-trip)
  - [20. `pca_testbench` save flow (v2.3.6)](#20-pca_testbench-save-flow-v236)

---

## TL;DR

Everything you might want to tweak in this project falls into one of
two buckets:

**Bucket 1 — runtime-tunable.** Servo limits, gesture velocities,
smoother accel/deadband, grip levels, per-servo bias offsets. Change
these by editing `cpcu_v2/config/runtime.json` (or, in v2.3.4+, via
the TUI's edit mode). Send `SIGHUP` to `cpcu_kernel` to reload — no
rebuild, no restart of any other process.

**Bucket 2 — compile-time only.** Safety thresholds (radio timeout,
battery cutoffs, thermal limits), packet wire format, IPC schema
version, BSAU radio channel. Change these via `./launch.sh
configure`, then rebuild. There is no runtime escape hatch — these
define what "safe" means and live behind code review.

The split is enforced. The runtime config is mirrored to a
shared-memory region (`IPC_RuntimeConfig`); cpcu_io reads it once per
loop and applies the values, but every value is **clamped against
the compile-time hardware limits** before being written to the PCA.
So a typo in runtime.json can't drive a servo past its mechanical
end-stop, no matter how the JSON is corrupted.

---

# Part 1 — Architecture

## 1. The runtime/compile-time decision

When deciding where a new knob belongs, ask: **"if this is wrong,
what breaks?"**

| If wrong, breaks... | Goes in | Example |
|---|---|---|
| Calibration / feel | `runtime.json` | Servo neutral position, smoother accel, per-gesture velocity |
| User experience | `runtime.json` | Confidence threshold, hysteresis votes, grip touch depth |
| One-session bench tuning | `runtime.json` | Per-servo bias offsets, deadband per channel |
| Safety envelope | `#define` (`./launch.sh configure`) | Radio timeout, VBAT critical, thermal CRIT, ring-overflow limit |
| Wire format | `#define` (`./launch.sh configure`) | Packet structure, IPC version |
| Hardware identity | `#define` (`./launch.sh configure`) | NRF channel, NRF address |

If the worst case is "uncomfortable", it's runtime. If the worst
case is "unsafe", it's compile-time. Tools (TUI editor, dataset
collector, web dashboard) operate on Bucket 1 only.

---

## 2. The runtime side — `runtime.json`

### Location

```
cpcu_v2/config/runtime.json   ← source of truth (git-tracked)
/opt/cpcu/config.json         ← symlink to the above
```

`./launch.sh setup` makes the symlink. Everything in the system
reads `/opt/cpcu/config.json`. There is one canonical config file;
edit the source-tree copy.

### Schema

```json
{
  "version": "2.3.3",
  "servo": {
    "S0": {"min_us": 498,  "max_us": 2500, "bias_us": 0},
    "S1": {"min_us": 1074, "max_us": 1953, "bias_us": 0},
    "S2": {"min_us": 1074, "max_us": 1953, "bias_us": 0},
    "S3": {"min_us": 1001, "max_us": 2002, "bias_us": 0},
    "S4": {"min_us": 1001, "max_us": 2002, "bias_us": 0},
    "S5": {"min_us": 976,  "max_us": 1733, "bias_us": 0}
  },
  "smoother": {
    "velocity_us_per_tick":     [12, 12, 12, 8, 8, 8],
    "accel_us_per_tick2":       [2, 2, 2, 1, 1, 1],
    "deadband_us":              [3, 3, 3, 2, 2, 2]
  },
  "grip": {
    "firm_us":                1100,
    "touch_us":               1200,
    "stall_recover_ms":       2000
  },
  "gesture_velocity": {
    "rest":         {"hold_ms": 0,    "decay_ms": 0},
    "biceps_flex":  {"hold_ms": 250,  "decay_ms": 500},
    "hand_flex":    {"hold_ms": 250,  "decay_ms": 500}
  }
}
```

### What v2.3.3 actually consumes

Not every JSON field is wired into a consumer yet. As of v2.3.3:

| Field | Consumer | Status |
|---|---|---|
| `servo.S*.min_us` | cpcu_io PCA write clamp | live |
| `servo.S*.max_us` | cpcu_io PCA write clamp | live |
| `servo.S*.bias_us` | cpcu_io pose lookup | live |
| `smoother.velocity_us_per_tick` | cpcu_smooth `SMOOTH_Init` | live |
| `smoother.accel_us_per_tick2` | cpcu_smooth `SMOOTH_Init` | live |
| `smoother.deadband_us` | cpcu_smooth `SMOOTH_Step` | live |
| `grip.firm_us` | cpcu_io grip kernel | live (v2.3.7) |
| `grip.touch_us` | cpcu_io grip kernel | live (v2.3.7) |
| `grip.stall_recover_ms` | cpcu_io grip kernel | live (v2.3.7) |
| `gesture_velocity.*` | (deferred to v2.3.5) | parsed but not yet applied |

Fields parsed but not yet applied are validated for shape on load
but ignored at runtime. Adding a new consumer just means reading
the field and writing it through to the right destination — no
schema change needed if the field exists.

### Loading and reload

cpcu_kernel reads `/opt/cpcu/config.json` once at startup, and
re-reads it on `SIGHUP`. The new config is parsed, validated, and
**only then** published to the IPC seqlock. If validation fails,
the old config stays live and an error is logged. cpcu_io picks up
the new values within ~20 ms via the seqlock.

```bash
# Edit the file, then:
kill -HUP $(pgrep cpcu_kernel)
```

The TUI's edit mode (v2.3.4+) does this automatically when you
press Ctrl+S in edit mode.

### Validation

The kernel rejects:
- `min_us > max_us`
- `min_us` or `max_us` outside the compile-time servo envelope
  (set in `cpcu_pca9685.h`)
- `velocity_us_per_tick` zero or negative
- `accel_us_per_tick2` negative
- Missing keys (only `servo.S0..S5` are mandatory; smoother/grip
  defaults are filled in if absent)

If you want to verify a config file before reloading, run:

```bash
cpcu_kernel --check /path/to/runtime.json
```

(The kernel exits 0 if valid, 1 with details if not.)

### Atomic edits and concurrency

All edits to the JSON file should be atomic (write to `*.tmp`, then
`rename()`). The TUI editor and `pca_testbench` round-trip both
use this pattern. If you edit by hand, use:

```bash
cp config/runtime.json config/runtime.json.tmp
$EDITOR config/runtime.json.tmp
mv config/runtime.json.tmp config/runtime.json
kill -HUP $(pgrep cpcu_kernel)
```

Direct in-place edits (`vi config/runtime.json`) are usually fine
because the kernel only reads on SIGHUP — but if SIGHUP fires while
the file is being written, the kernel sees a partial JSON and
rejects it (keeping the old config). Atomic edits avoid the race
entirely.

---

## 3. The compile-time side — `./launch.sh configure`

### What it edits

Compile-time `#define`s in five header files:

| File | What it controls |
|---|---|
| `cpcu_v2/include/cpcu_safety.h` | Radio timeout, vbat thresholds, thermal limits, ring overflow |
| `cpcu_v2/include/cpcu_pca9685.h` | Servo mechanical envelope (min/max micros) |
| `cpcu_v2/include/cpcu_smooth.h` | Smoother defaults (overridden by runtime.json if present) |
| `cpcu_v2/include/wireless_packet.h` | Packet schema version, IPC version |
| `cpcu_v2/../bsau_v2/Core/Inc/bsau_app.h` | BSAU NRF channel + address |

The script knows the exact line each tunable lives on and uses
`sed` with range validation to edit them. No grep-and-replace; no
risk of corrupting the header.

### Usage patterns

All forms route through `./launch.sh configure`:

```bash
./launch.sh configure                          # interactive walkthrough
./launch.sh configure --show                   # all values
./launch.sh configure --diff                   # only modifications
./launch.sh configure --reset                  # restore all defaults
./launch.sh configure --reset --runtime        # also regenerate runtime.json

./launch.sh configure --radio-timeout          # show one current value
./launch.sh configure --radio-timeout 1000     # set one knob
./launch.sh configure --vbat-low 3.1 --thermal-warn 70   # set multiple

./launch.sh configure --bsau                   # only BSAU tunables (channel)
./launch.sh configure --cpcu                   # only CPCU tunables
```

After any edit, the launch wrapper detects the helper's exit code
11 ("rebuild required") and prompts:

```
[LAUNCH] WARN: REBUILD REQUIRED

  Rebuild now? [Y/n]:
```

Yes → runs `./launch.sh build`. No → tells you to run it manually
before next launch.

### Safety rationale

A single sed edit to a `#define` in `cpcu_safety.h` is a code
change. Code changes go through:
1. Edit (via this script — guarantees correct syntax + range)
2. Rebuild (detects compilation errors)
3. Reinstall (caps re-applied automatically by `./launch.sh build`)
4. Restart (the new binary takes effect)

That four-step cycle is what makes `#define` safety-grade. A
runtime knob is one step (`SIGHUP`) — fast, but riskier. The split
is intentional: the bullet-proof step is reserved for things whose
correctness affects safety.

---

## 4. Why bias-then-clamp matters

The data flow on every servo command is:

```
Gesture lookup  →  pose_us[6]              (from compile-time table)
  +
runtime.json bias[6]  →  biased_us[6]      (per-servo offset)
  ↓
clamp(biased_us[i], runtime.json.min[i], runtime.json.max[i])
  ↓
clamp(., compile-time PCA_HW_MIN_US, PCA_HW_MAX_US)
  ↓
PCA9685 write
```

Two clamps, in order. The runtime clamp uses the JSON's `min_us` /
`max_us` (which the operator sets per assembly). The compile-time
clamp uses `cpcu_pca9685.h`'s hardware envelope (the absolute
servo specs). The runtime clamp is *always* tighter (validation
rejects any JSON where it isn't), so in practice only the runtime
clamp fires. But the second clamp is the failsafe: if the JSON
ever loaded with bad values that bypassed validation, the
compile-time clamp catches them.

Bias is applied **before** the clamps. Why this order: bias
represents the operator's deliberate trim ("S3 is mounted 5°
shifted, compensate"). Applying it before the clamps means a bias
that pushes a value out of bounds gets clipped at the bound
(safe), not lost (silent). If the operator sets a 200 µs bias on a
servo whose range is 1000–2000, and the gesture asks for 1900,
the result is 2000 (clamped), not 2100 (bypassing the limit).
That's the right behavior.

---

## 5. SeqLock pattern (for IPC tinkering)

The runtime config is published from cpcu_kernel to all consumers
via a **seqlock** in shared memory:

```c
typedef struct {
    _Atomic uint32_t  seq;          // even = stable, odd = writing
    RuntimeConfig     payload;      // ~512 bytes
    uint8_t           pad[64];      // cache-line separation
} IPC_RuntimeConfig;
```

**Writer (cpcu_kernel, on SIGHUP):**
```c
atomic_fetch_add(&shm->seq, 1);     // → odd, "writing"
memcpy(&shm->payload, &new_config, sizeof(new_config));
atomic_fetch_add(&shm->seq, 1);     // → even, "done"
```

**Reader (cpcu_io, every tick):**
```c
do {
    s1 = atomic_load(&shm->seq);    // odd? wait
    if (s1 & 1) continue;
    memcpy(&local, &shm->payload, sizeof(local));
    s2 = atomic_load(&shm->seq);
} while (s1 != s2);                 // value changed mid-read? retry
```

This is lock-free, wait-free for the writer, and bounded retry
for the reader. The reader sees either the old config or the new
config in full — never a half-updated mix. Latency is sub-µs
typical; pathological retry is bounded by writer throughput
(once per SIGHUP, so at most a single retry).

**If you add a new IPC region for tunables**, follow the same
pattern. Don't roll a mutex. The cpcu_io tick is real-time and
can't block on locks.

---

## 6. Core allocation

The runtime config touches every consumer, but each consumer
runs on a different core:

| Process | Core(s) | Reads runtime config? |
|---|---|---|
| cpcu_kernel | 0 (CFS) | Writes (the publisher) |
| cpcu_tui | 0 (CFS) | Reads on render |
| cpcu_ws | 0 (CFS) | Reads on tick |
| cpcu_dsp.py (Python) | 1, 2 (FIFO 80) | Reads thresholds, gesture map |
| cpcu_io | 3 (FIFO 90) | Reads servo limits, smoother knobs, grip |

cpcu_io's read is the latency-critical one (RT loop at 50 Hz).
It's wait-free thanks to the seqlock. Other consumers are
soft-real-time and can afford the retry loop without missing
deadlines.

---

## 7. Testing

The runtime config has a dedicated testbench:

```bash
./launch.sh test          # runs test_runtime_config + 232 others
```

Coverage: 43 tests in Phase 1. Validates:
- Schema parsing (correct, malformed, missing fields)
- Validation rejection (min > max, zero velocity, etc.)
- Bias-then-clamp ordering (the safety guarantee above)
- SeqLock atomicity under contention
- SIGHUP-induced reload doesn't drop a tick
- Surgical-edit preservation of unrecognized fields (so a future
  field added by a newer kernel isn't lost when an older
  pca_testbench writes the file)

Phase 2 (`./launch.sh test-ipc`) extends this by validating that
the C/Python view of the config struct agrees byte-for-byte —
otherwise cpcu_dsp.py would read garbage off the seqlock.

---

## 8. Operating procedure

### Day 1: bring up

```bash
./launch.sh setup
./launch.sh build
./launch.sh check                 # confirm everything green
./launch.sh test                  # 233 PASS expected
```

After this, `runtime.json` is the defaults — every servo is
`min/max` at its hardware envelope, `bias_us = 0`, smoother at
factory tuning.

### Calibration session

```bash
./launch.sh test-pca              # interactive: set min/max/bias per servo
                                  # (saves to runtime.json on press 's')
./launch.sh tui                   # confirm gestures look right
                                  # — press '7' (Config) then 'e' (edit mode)
                                  # to tweak smoother/grip live
                                  # — Ctrl+S to commit, kernel reloads in ~20 ms
```

This loop uses **only the runtime side**. No rebuild, no kernel
restart, no sudo. If the operator nukes runtime.json by accident,
the kernel rejects it and keeps the old config running.

### Production change to a safety threshold

```bash
./launch.sh configure --diff      # see current modifications
./launch.sh configure --vbat-low 3.05  # propose change
# (launch.sh prompts: rebuild now? [Y/n] — say Y)
./launch.sh check                 # verify post-rebuild state
./launch.sh tui                   # confirm new behavior
```

This change goes through the four-step compile-time cycle. The
operator should always run `./launch.sh test` between propose
and rebuild — the tests cover all the safety thresholds.

### Reverting

Runtime change gone wrong:
```bash
git checkout config/runtime.json
kill -HUP $(pgrep cpcu_kernel)
```

Compile-time change gone wrong:
```bash
./launch.sh configure --reset     # restore all defaults
./launch.sh build                 # accept rebuild prompt
```

Or for surgical revert of one knob:
```bash
git checkout include/cpcu_safety.h    # or whichever header
./launch.sh build
```

---

# Part 2 — Per-file reference

## 9. `cpcu_safety.h` — safety thresholds

**Path:** `include/cpcu_safety.h` in the CPCU source tree (or wherever
your build keeps headers).

### Radio link

| Constant | Default | Meaning |
|---|---|---|
| `SAFETY_RADIO_TIMEOUT_MS` | 750 | Silence (no packet) after which RUNNING → DEGRADED |
| `SAFETY_RADIO_SAFE_MS` | 1500 | DEGRADED-state duration before SAFE (terminal until recovery) |
| `RECOVERY_PKT_COUNT` | 10 | Consecutive good packets needed to leave RECOVERING |
| `SAFETY_RADIO_BOOT_GRACE_MS` | 5000 | **(v2.3.1)** Cold-start grace before the radio timeout fires for the first time. Suppresses a spurious fault when CPCU starts before BSAU. Lifted by either (a) first received packet, or (b) elapsed grace. See [`BOOT_AND_SYNC.md`](BOOT_AND_SYNC.md). |

Lower the timeout values for faster fault detection at the cost of
false alarms during brief radio glitches. Don't drop
`SAFETY_RADIO_TIMEOUT_MS` below ~150 ms or normal RF retries will
trigger it.

**Tuning the boot grace.** 3-5 s is the sweet spot. Lower than 3 s
risks tripping during BSAU's normal boot (~1 s of NRF init plus user
power-on reaction time). Higher than ~10 s makes a genuinely-dead
BSAU look healthy for too long. The default 5 s assumes typical user
behaviour ("flip both switches within a few seconds of each other").

### Link quality classification

| Constant | Default | Meaning |
|---|---|---|
| `SAFETY_LINK_WINDOW` | 1000 | Packets per quality-classification window (= 1 s) |
| `SAFETY_LINK_RETRY_GOOD` | 0.5 | Mean retries below this → potentially GOOD |
| `SAFETY_LINK_RETRY_DEGRADED` | 3.0 | Mean retries below this → DEGRADED, else POOR |
| `SAFETY_LINK_LOSS_GOOD` | 0.001 | Loss rate below this → potentially GOOD |
| `SAFETY_LINK_LOSS_DEGRADED` | 0.05 | Loss rate below this → DEGRADED, else POOR |

Both retry **and** loss must satisfy the GOOD threshold for the link
to be classified GOOD. Otherwise the looser threshold defines DEGRADED.

### Battery

| Constant | Default | Meaning |
|---|---|---|
| `SAFETY_VBAT_LOW_V` | 3.0 | Below this → reported as low (warning) |
| `SAFETY_VBAT_CRITICAL_V` | 2.7 | Below this → enter SAFE |
| `SAFETY_VBAT_RECOVER_V` | 3.0 | Voltage must rise above this to leave critical |
| `SAFETY_VBAT_DIVIDER` | **1.0** | Multiplier on the BSAU's reported voltage |

**`SAFETY_VBAT_DIVIDER` is hardware-dependent.** The BSAU firmware version
in this repo already corrects for the on-board 100k/100k divider before
transmitting, so the multiplier here is 1.0. If your BSAU firmware sends
the **raw** divider'd ADC reading (older versions, or a custom build),
set this back to 2.0. Symptom of getting it wrong: battery reads 2× or
0.5× the real value.

### DSP

| Constant | Default | Meaning |
|---|---|---|
| `SAFETY_DSP_STALL_MS` | 2000 | If no motor cmd from Python in this long → SAFE |

Don't lower below ~500 ms — the DSP needs that long to start up after
a kernel respawn.

### I2C

| Constant | Default | Meaning |
|---|---|---|
| `SAFETY_I2C_MAX_ERRORS` | 5 | Consecutive failed PCA writes before SAFE |

### Ring buffer (v2.3 — recoverable fault)

| Constant | Default | Meaning |
|---|---|---|
| `SAFETY_RING_OVERFLOW_LIMIT` | 100 | Overflows since baseline before SAFE |
| `SAFETY_RING_RECOVER_MS` | 5000 | Quiescence required to clear the fault |

The ring is 1024 entries (1 second of buffering). The trip threshold
of 100 overflows is applied to the **delta since the last quiescent
baseline**, not to the all-time cumulative count — so once the
producer/consumer rebalance and 5 s pass with no new overflows, the
fault clears and the baseline is reset for future bursts. Pre-v2.3
the threshold compared against `io_ring_overflows` directly, which
latched in SAFE forever once tripped. Raise `SAFETY_RING_OVERFLOW_LIMIT`
if you have an unstable DSP that occasionally pauses; raise
`SAFETY_RING_RECOVER_MS` if your bursts come in waves and you want a
longer "all clear" hold time before resuming.

### Thermal

| Constant | Default | Meaning |
|---|---|---|
| `SAFETY_THERMAL_WARN_C` | 75 | Above this → log a warning |
| `SAFETY_THERMAL_CRITICAL_C` | 82 | Above this → SAFE |
| `SAFETY_THERMAL_RECOVER_C` | 70 | Must drop below this to leave SAFE |

Pi 4's thermal throttle starts at ~80 °C; we trip a few degrees above
that so the CPU has already throttled before we kill servos.

### SAFE recovery

| Constant | Default | Meaning |
|---|---|---|
| `SAFETY_SAFE_RECOVER_MS` | 3000 | All faults must be clear for this long before exit |
| `SAFETY_AUTO_CLEAR_MS` | 300000 | LINK_GOOD duration before cumulative counters auto-reset |

Lower `SAFETY_SAFE_RECOVER_MS` for snappier recovery in lab conditions
(say 1000 ms). Don't lower below ~500 ms or the FSM will flap when
voltage hovers around the critical threshold.

---

## 10. `cpcu_smooth.h` — servo motion profile

**Path:** `include/cpcu_smooth.h`.

| Constant | Default | Meaning |
|---|---|---|
| `SMOOTH_DEFAULT_VELOCITY` | 2000 | µs/s — full 2000 µs span in 1.0 s if cruising |
| `SMOOTH_DEFAULT_ACCEL` | 8000 | µs/s² — reach max velocity in 250 ms |
| `SMOOTH_SETTLE_THRESH` | 2 | Within this many µs of target = settled |
| `SMOOTH_DEFAULT_DEADBAND` | 10 | **(v2.3.2)** Hold-pose deadband, µs (≈0.9°). Once a servo settles, fresh PCA writes are suppressed until the target moves more than this from the last latched value. Kills static jitter from the servo's internal P controller being re-triggered every 50 Hz tick. See [`JITTER_MITIGATION.md`](JITTER_MITIGATION.md). |

These are the *defaults*, applied to all 6 channels at `SMOOTH_Init`.
Per-servo overrides happen in `cpcu_io.c` (see §13). Lower
`SMOOTH_DEFAULT_VELOCITY` if your servos mechanically bind at peak
speed. Lower `SMOOTH_DEFAULT_ACCEL` if you hear/see jolts at the start
of motion.

### Tuning the hold-pose deadband

Per-servo via `SMOOTH_SetDeadband(ctx, channel, deadband_us)`.

| Setting | Effect | When to use |
|---|---|---|
| `0` | Deadband disabled — always writes every tick | Debugging; verifying jitter is host-induced (compare with vs without) |
| `4` (≈0.36°) | Tight | Wrist or fine-motion joints where you want responsive small moves |
| `10` (≈0.9°, default) | Balanced | All arm joints unless you have a reason to deviate |
| `25` (≈2.3°) | Loose | Joints that twitch a lot under load, where you accept "steppy" slow motion as a trade |

The deadband does NOT fix gravity-driven sag, mechanical resonance,
power-supply ripple, or cross-coupled jitter from a stalled servo.
See [`JITTER_MITIGATION.md`](JITTER_MITIGATION.md) §5 for the
distinction.

---

## 11. `cpcu_pca9685.h` — servo mechanical limits

**Path:** `include/cpcu_pca9685.h`.

| Constant | Typical | Meaning |
|---|---|---|
| `PCA_SERVO_PERIOD_US` | 20000 | 50 Hz PWM period |
| `PCA_SERVO_NEUTRAL` | 1500 | Center pulse width (µs) |
| `PCA_SERVO_MIN_US` | per-servo array | Mechanical min, clamps targets |
| `PCA_SERVO_MAX_US` | per-servo array | Mechanical max, clamps targets |

`PCA_SERVO_MIN_US` / `PCA_SERVO_MAX_US` are 6-element arrays, one entry
per joint. They get loaded by `PCA_Init` and applied by
`PCA_SafetyClamp`. Calibrate per servo: drive `pca_testbench`, find
the actual mechanical limits with `m`/`M`, then transcribe into the
header and rebuild.

Defaults in this repo (from the header comments):

```c
#define PCA_SERVO_MIN_US  { 498, 1074, 1074, 1001, 1001,  976 }
#define PCA_SERVO_MAX_US  {2500, 1953, 1953, 2002, 2002, 1733 }
```

Logical channels: S0=Base, S1=Upper, S2=Last, S3=Joint-1, S4=Joint-2,
S5=Gripper.

---

## 12. `cpcu_dsp.py` — DSP/ML pipeline

**Path:** `/opt/cpcu/python/cpcu_dsp.py` (after install) or
`python/cpcu_dsp.py` (in source).

### Channel selection

```python
ACTIVE_CHANNELS         =   [0, 1, 2]
CHANNEL_LABELS          =   ['s1', 's2', 's3']      # must match training names
```

3 of the 8 BSAU channels feed the model. **Position in the list maps
directly to the team's sensor names**, which in turn map to physical
electrode positions on the arm:

| List index | Team label | Physical electrode |
|---|---|---|
| `ACTIVE_CHANNELS[0]` | s1 | **Forearm** (wrist flexor) |
| `ACTIVE_CHANNELS[1]` | s2 | **Biceps** |
| `ACTIVE_CHANNELS[2]` | s3 | **Triceps** |

The default `[0, 1, 2]` therefore commits the wiring to:

```
PA0 (BSAU ADC pin) → Forearm electrode
PA1                → Biceps electrode
PA2                → Triceps electrode
```

If your electrodes are wired in a different order, edit
`ACTIVE_CHANNELS`. **Wrong wiring won't crash anything** — the SVM
will just produce consistently-wrong predictions because feature
\[0..3\] is no longer the muscle the model expects.

#### Recipe: re-mapping after a hardware change

You re-wired such that PA3=Forearm, PA0=Biceps, PA5=Triceps. To match:

```python
ACTIVE_CHANNELS         =   [3, 0, 5]
# CHANNEL_LABELS still ['s1','s2','s3'] — the LABEL is what the model
# expects in the feature vector position; ACTIVE_CHANNELS picks WHICH
# raw BSAU channel populates that position.
```

Restart `cpcu_kernel` and confirm via the TUI's DSP/AI page (key 3)
that the per-channel RMS bars track the muscle you expect.

### Sample rate / windowing

```python
INPUT_FS_HZ             =   2000        # CPCU native rate (don't change)
TARGET_FS_HZ            =   200         # Team's training rate
DECIMATE_FACTOR         =   10          # 2000 / 200
WINDOW_MS               =   200         # 40 samples at 200 Hz
STRIDE_MS               =   100         # 50 % overlap
```

`TARGET_FS_HZ` and `WINDOW_MS` must match the rate the model was
trained at. Don't change without retraining (`feature_ex.py` then
`model.py`). `INPUT_FS_HZ` is fixed by BSAU's 2 kHz scan rate.

### Filter band (read carefully — there's a known train/live gap)

The actual bandpass call inside `process_window()` is:

```python
bp = butter_bandpass(centered, 20.0, 450.0, TARGET_FS_HZ)
```

`450.0` looks like a typo until you check `butter_bandpass`'s safety
clamp: at Fs=200 Hz, Nyquist is 100 Hz, so the 450 Hz request gets
auto-clamped to `nyq * 0.95 = 95 Hz`. The effective passband is
**20 – 95 Hz**. This was inherited byte-for-byte from the team's
`predict.py`.

The team's TRAINING pipeline (`feature_ex.py`) uses a **15 – 90 Hz**
bandpass — different by ~5 Hz on each end. On real EMG the two
filters produce features within ~0.2 % RMS, so the model generalises
across the gap. The team has empirically validated this configuration
on the live chain (`predict.py`); we matched their live setup, not
their training setup, on purpose.

If you want to match training exactly (e.g. for retraining
experiments), change the call to:

```python
bp = butter_bandpass(centered, 15.0, 90.0, TARGET_FS_HZ)
```

This will produce features that are byte-identical to what the team's
training pipeline saw, at the cost of departing from `predict.py`.
Don't change unless you intend to retrain.

### ML thresholds (hysteresis / debouncing)

```python
PROBABILITY_THRESHOLD   =   0.65
CONFIRMATION_THRESHOLD  =   3
```

A class change requires `CONFIRMATION_THRESHOLD` consecutive
predictions above `PROBABILITY_THRESHOLD` to take effect. Both values
match `predict.py`. Lower thresholds = snappier gesture transitions
but more flicker; raise for robust deployment.

### Gesture → servo pose map

```python
GESTURE_SERVO_MAP = {
    "rest":         [1500, 1500, 1500, 1500, 1500, 1500],
    "biceps_flex":  [1500, 1700, 1500, 1500, 1500, 1500],
    "hand_flex":    [1700, 1500, 1500, 1700, 1700, 1700],
    "hand_open":    [1300, 1500, 1500, 1300, 1300, 1300],
}
```

Each entry maps a model class label to a 6-tuple of servo pulse widths
(µs). Edit per your hardware. Unmapped labels fall back to all-neutral.

**Strings must exactly match `model.classes_` from your trained
joblib.** The team's `predict.py` color_map confirms at least
`rest`, `biceps_flex`, `hand_flex` are trained classes; `hand_open`
is included as future-proofing (unused by the current model — its
entry simply never fires).

To check: run `cpcu_dsp.py` and watch the startup log line
`[DSP] model + scaler loaded (classes=[...])`. Any class in that list
that's not in `GESTURE_SERVO_MAP` is silently neutral-mapped — so add
an entry for it before that gesture goes live.

### Model file paths

```python
MODEL_DIR               =   "/opt/cpcu/models"
MODEL_PATH              =   os.path.join(MODEL_DIR, "hmi_svm_model_200hz.joblib")
SCALER_PATH             =   os.path.join(MODEL_DIR, "hmi_scaler_200hz.joblib")
THRESHOLDS_PATH         =   os.path.join(MODEL_DIR, "noise_thresholds.json")
```

The `noise_thresholds.json` file is purely a diagnostic side-channel
populated by `cpcu_dsp.py --calibrate N`. It is **not read by
inference** — the trained SVM has learned "rest" from labelled data
during training. (Both team threshold formulas — `proccess.py`'s
`3 × std` and `feature_ex.py`'s `95th × 1.5` — share the same fate
in their pipeline: computed but never consumed by `model.py`.)

`/opt/cpcu/models/` is owned by your user (set by `setup_pi.sh`), so
`cp emg_*.joblib /opt/cpcu/models/` doesn't need sudo.

### Drain rate

```python
DRAIN_PERIOD_S          =   0.020       # 50 Hz
DRAIN_BATCH             =   200
```

Don't lower the period below ~5 ms — the IPC writes are synchronous
and lock contention with cpcu_io rises sharply.

### Recipe: retrain on CPCU-collected data

If you collect a labelled dataset using the TUI's Page 6 capture
workflow (writes `cpcu_v2/datasets/<label>_<idx>.csv` in the team's
expected format), you can retrain locally on the Pi.

The team's training scripts (`feature_ex.py`, `model.py`,
`proccess.py`) aren't part of the CPCU install tree — they're a
separate, evolving toolchain. Drop them in your home directory:

```bash
# 1. Place team scripts in your home, alongside the dataset
cd ~
git clone <team-training-repo> training
cd training

# 2. Edit the DATA_FOLDER constant in feature_ex.py to point at
#    ~/prosthetic_hand/cpcu_v2/datasets/.
#    Skip proccess.py — captures are already filtered by cpcu_io.
#    OR leave proccess.py in the chain if you want the training-side
#    cascaded-filter behaviour (see ARCHITECTURE.md §7.5).

# 3. Run them (deps installed by ./launch.sh setup):
python3 feature_ex.py            # produces features_200hz_segmented.csv
python3 model.py                 # produces hmi_*_200hz.joblib

# 4. Move the new joblibs into place:
mv hmi_*.joblib /opt/cpcu/models/

# 5. Restart so cpcu_dsp.py picks up the new model:
cd ~/prosthetic_hand/cpcu_v2
./launch.sh stop
./launch.sh tui
```

If the new model has classes that aren't in `GESTURE_SERVO_MAP`, edit
the map first and rebuild the launcher's pyc cache (just re-source
the file is enough; cpcu_dsp.py imports it fresh on each startup).

---

## 13. `cpcu_io.c` — runtime per-servo config

In `main()`, just after `SMOOTH_Init(&smooth, PCA_SERVO_NEUTRAL);`, set
per-servo overrides:

```c
SMOOTH_SetEnabled(&smooth, 5, false);   /* Gripper bypassed */
SMOOTH_SetVelocity(&smooth, 0, 1500);   /* Slower Base */
SMOOTH_SetAccel(&smooth, 3, 12000);     /* Snappy Joint-1 */
SMOOTH_SetAccel(&smooth, 4, 12000);     /* Snappy Joint-2 */
```

Logical indices (0..5) match the order in `PCA_SERVO_MIN_US`.

`SetEnabled(false)` makes the smoother a passthrough for that channel —
target is written to the PCA every tick with no ramping. Use for fast
binary actions (open/close gripper) where a 250-ms ramp feels sluggish.

---

## 14. `bsau_config.h` — BSAU mode selection

**Path:** `Inc/bsau_config.h` in the BSAU CubeIDE project.

Pick exactly **one** of:

```c
//#define BSAU_MODE_RELEASE
//#define BSAU_MODE_DEBUG
#define BSAU_MODE_DATASET
//#define BSAU_MODE_TEST_NRF
//#define BSAU_MODE_TEST_CSV
//#define BSAU_MODE_TEST_NRF_LOG
```

| Mode | UART | Radio | Use for |
|---|---|---|---|
| RELEASE | silent | yes | production deployments |
| DEBUG | structured logs | yes | bringup, manual diagnostics |
| DATASET | 8-col CSV @ 1 kHz | non-blocking | training-data collection on PC |
| TEST_NRF | NRF self-test | (ttest) | NRF hardware verification |
| TEST_CSV | sine sweep | yes | end-to-end signal-chain testing |
| TEST_NRF_LOG | NRF + structured | yes | post-mortem of radio issues |

**v2.4 fix:** in DATASET mode, the radio TX is now non-blocking, so
UART output continues even when the radio link is dead. Re-flash the
BSAU after applying the patch.

Other BSAU-side knobs:

| Constant | Default | Meaning |
|---|---|---|
| `BSAU_DATASET_CSV_DECIMATION` | 1 | 1 = every packet (1 kHz CSV); 2 = 500 Hz; etc. |
| `BATT_CRITICAL_THRESHOLD` | (raw ADC) | Below this → BSAU sets WL_BATT_CRIT in flags |
| `BATT_LOW_THRESHOLD` | (raw ADC) | Below this → BSAU sets WL_BATT_LOW |
| `LOG_STATS_INTERVAL` | 1000 | Pkts between structured status log lines |
| `LOG_CSV_DEBUG_INTERVAL` | (mode-dep) | Pkts between SerialPlot CSV lines (DEBUG mode) |

---

## 15. Operator commands — runtime, no recompile

Set by writing bits to `ipc.ctrl->operator_cmd` from cpcu_tui (key
bindings) or any other tool that mmaps `/dev/shm/cpcu_ipc`.

| Bit | Symbol | Effect |
|---|---|---|
| 0x01 | `IPC_OP_ALL_OFF` | cpcu_io calls `PCA_AllOff` and latches "killed" state |
| 0x02 | `IPC_OP_RESUME` | clears killed state, re-inits PCA to neutral |
| 0x04 | `IPC_OP_CLEAR_COUNTERS` | zeros cumulative diag counters |

TUI keybindings (in cpcu_tui v2.4):

| Key | Sets bit |
|---|---|
| `O` or `!` | ALL_OFF |
| `P` | RESUME |
| `C` | CLEAR_COUNTERS |

cpcu_io reads and **clears** the byte each loop with `atomic_exchange`,
so commands are one-shot — pressing the key fires the action once.

---

## 16. `./launch.sh` — runtime mode selection

Five modes, each launches a different combination:

| Mode | Kernel | Foreground tool |
|---|---|---|
| `kernel` | foreground | none (systemd-style) |
| `tui` | background | cpcu_tui dashboard |
| `collect` | background | cpcu_tui (with capture-workflow reminder) |
| `signal` | background | signal_testbench |
| `pca` | not started | pca_testbench |

```bash
sudo /opt/cpcu/launch.sh           # interactive menu (TTY)
sudo /opt/cpcu/launch.sh kernel    # production / systemd
sudo /opt/cpcu/launch.sh tui
sudo /opt/cpcu/launch.sh signal
sudo /opt/cpcu/launch.sh pca
```

See `launch.sh` header for the full documentation block.

---

## 17. Pi system configuration

These are one-time, set during `setup_pi.sh` and persist:

### `/boot/firmware/cmdline.txt`

```
isolcpus=1,2,3 nohz_full=1,2,3 rcu_nocbs=1,2,3
```

Reserves cores 1–3 for our RT pipeline. Without this, you have no
real-time guarantees. Verify with `cat /sys/devices/system/cpu/isolated`
— must print `1-3`.

### `/boot/firmware/config.txt`

```
dtparam=spi=on
dtparam=i2c_arm=on
arm_freq=2800
arm_freq_min=2800
core_freq=750
gpu_freq=750
over_voltage=6
force_turbo=1
```

SPI for the nRF24L01+, I²C for the PCA9685, locked clocks for
deterministic latency. `force_turbo=1` keeps the CPU at full clock —
voids the Pi warranty per RPi Foundation; remove if you care about
that.

### CPU governor

```bash
sudo cpufreq-set -c 0 -g performance
sudo cpufreq-set -c 1 -g performance
sudo cpufreq-set -c 2 -g performance
sudo cpufreq-set -c 3 -g performance
```

Set in `setup_pi.sh`. Can also set `governor=performance` in
systemd-cpufreq.

### systemd unit

`/etc/systemd/system/cpcu.service` — see the unit file in this repo.
Notable fields:

| Field | Why it matters |
|---|---|
| `User=root` | needed for SCHED_FIFO, mlockall, `/dev/i2c-1`, `/dev/spidev0.0` |
| `LimitRTPRIO=90` | allows the priority cpcu_io requests for Core 3 |
| `LimitMEMLOCK=infinity` | allows `mlockall()` to prevent page faults |
| `KillMode=control-group` | SIGTERM goes to kernel + io + dsp together |
| `WatchdogSec=30` | systemd kills service if hung 30 s |
| `Restart=on-failure` | auto-restart on crash, max 5 attempts/min |

To stop/start manually:

```bash
sudo systemctl stop cpcu
sudo systemctl start cpcu
sudo systemctl status cpcu
journalctl -u cpcu -f
```

To disable auto-start at boot:

```bash
sudo systemctl disable cpcu
```

---

# Part 3 — Recipes

## 18. Common configuration recipes

### "Servos move too slowly"

Increase `SMOOTH_DEFAULT_VELOCITY` in `cpcu_smooth.h` from 2000 to 3000,
or apply per-servo via `cpcu_io.c`. Recompile both.

### "Gripper is sluggish"

Already addressed by `SMOOTH_SetEnabled(&smooth, 5, false)` in v2.4.
If you want partial smoothing, leave it enabled but raise its accel:
`SMOOTH_SetAccel(&smooth, 5, 16000)`.

### "Battery shows wrong voltage"

Check `SAFETY_VBAT_DIVIDER`. Set it so:

```
displayed_voltage = (raw_ADC * 3.3 / 4095) * SAFETY_VBAT_DIVIDER
```

If your hardware has the on-board 100k/100k divider, *and* your BSAU
firmware does NOT correct for it before sending, set the multiplier
to 2.0. Otherwise 1.0.

### "Want faster SAFE recovery for testing"

Drop `SAFETY_SAFE_RECOVER_MS` to 1000 in `cpcu_safety.h`. Recompile
cpcu_io. Don't ship this — at <500 ms the FSM will flap on
borderline-voltage batteries.

### "Want CPCU to ignore radio loss completely (lab/bench)"

Don't change the FSM. Instead, run the BSAU in DATASET mode and use
the UART path. CPCU's safety logic is designed for the prosthetic-on-
patient case where loss of radio is genuinely unsafe — bypassing it
in code is more invasive than just ignoring CPCU's SAFE state when
you don't have servos attached.

### "Want to add a new gesture"

1. Collect labelled CSVs via cpcu_tui Dataset page (key `6` since v3.4,
   files preserved in `./datasets/`).
2. Run the team's `feature_ex.py` + `model.py` against the new dataset.
3. Copy the new joblib + scaler to `/opt/cpcu/models/`.
4. Add the new label string to `GESTURE_SERVO_MAP` in `cpcu_dsp.py`
   with a sensible servo pose tuple.
5. Restart the kernel: `sudo systemctl restart cpcu`.

### "All servos should move during specific gestures even if model is uncertain"

Lower `PROBABILITY_THRESHOLD` in `cpcu_dsp.py` (e.g., 0.45). Or shrink
`CONFIRMATION_THRESHOLD` (e.g., 1) for instant gesture switching.

---

## 19. What's NOT configurable (and why)

* **Sample rate (2 kHz at BSAU, 1 kHz packet rate)**. Wired into TIM6
  config and ADC scan timing; changing requires re-deriving the whole
  budget on both sides.
* **8 EMG channels**. Determined by the analog front-end PCB. Reducing
  to 6 channels (which the v1.0 hardware used) would require a packet-
  format change (same channel count constants in `wireless_packet.h`,
  but propagated through the codec, the ring entry size, the DSP, …).
* **Ring buffer size (1024)**. Power-of-two requirement (we mask, not
  modulo). 1024 entries × 64 bytes = 64 KB, sized for ~1 s of buffering
  at the 1 kHz packet rate. Can be 512 (50 % less RAM, 50 % less
  buffering) or 2048, but nothing else.
* **Cache-line alignment in IPC structs**. Hard-required for the
  lock-free SPSC ring buffer to actually be lock-free. Don't `_packed`
  these structs.

---

# Part 4 — Calibration round-trip

## 20. `pca_testbench` save flow (v2.3.6)

The honest workflow for finding good per-servo limits and bias values
is to **physically jog each servo while watching the arm**. That's
exactly what `pca_testbench` is for. Until v2.3.6 the values you
discovered there died on exit. Now they round-trip through `runtime.json`.

### Flow

```bash
# 1. Stop the live system so pca_testbench can own I2C exclusively.
sudo systemctl stop cpcu

# 2. Jog with arrow keys, mark calibration values:
sudo ./pca_testbench --config config/runtime.json
#   UP/DOWN          select servo
#   LEFT/RIGHT       jog +/- 10 us
#   PgUp/PgDn        jog +/- 50 us
#   m / M            jog to current MIN / MAX
#   [                set current jog AS MIN for selected servo
#   ]                set current jog AS MAX for selected servo
#   b                set current deviation from neutral AS BIAS
#   B                clear bias for selected servo
#   v                cycle smoother VELOCITY preset for selected servo
#   a                cycle smoother ACCELERATION preset
#   d                cycle smoother DEADBAND preset
#   , .              fine -/+ on the last-touched smoother knob
#   S                save min/max/bias/smoother values to runtime.json
#   L                reload from disk (discards unsaved jogs)
#   q                quit (warns if unsaved)

# 3. The save patches only the three fields pca_testbench owns:
#    servo_min_us, servo_max_us, servo_bias_us. Everything else
#    (gesture_velocity, smoother params, grip levels, etc.) survives.

# 4. Restart the live system to pick up the new values:
sudo systemctl start cpcu

# Or, if cpcu_kernel is already running and you want a hot reload:
kill -HUP $(pgrep -f cpcu_kernel)
```

### What gets saved

| Key in `runtime.json` | Source in pca_testbench | Set by keystroke |
|---|---|---|
| `servo_min_us[]` | `pca.servo_min[i]` | `[` |
| `servo_max_us[]` | `pca.servo_max[i]` | `]` |
| `servo_bias_us[]` | `servo_bias[i]` | `b` (set), `B` (clear) |
| `smooth_velocity_us_per_s[]` | `smooth_vel[i]` | `v` (cycle preset) |
| `smooth_accel_us_per_s2[]` | `smooth_acc[i]` | `a` (cycle preset) |
| `smooth_deadband_us[]` | `smooth_dead[i]` | `d` (cycle preset) |

The smoother keys cycle through presets (slow/normal/fast) rather
than entering numeric values. Each press steps to the next preset
for the *selected* servo and immediately applies it to the live
smoother — you feel the change on the next jog. Save with `S` to
commit; `L` to reload from disk and discard unsaved changes.

For exact values between presets, `,` and `.` fine-adjust the
**last-touched** smoother knob for the selected servo. Press `v`
to ballpark velocity, then `,` and `.` to nudge in 100 µs/s
increments. Switching servos with UP/DOWN preserves which knob
is being adjusted — the "last touched" semantic is global, not
per-servo. Step sizes:

| Knob | Cycle preset | Fine step (`,` / `.`) | Range |
|---|---|---|---|
| velocity | `v` | 100 µs/s | 100..10000 |
| acceleration | `a` | 500 µs/s² | 500..50000 |
| deadband | `d` | 1 µs | 0..50 |

The fine ranges match what the JSON loader's range-check accepts,
so the bench can never let you set a value the live system would
reject on reload.

The presets, ordered by cycle:

| Knob | Default | Cycle (after default) | Units |
|---|---|---|---|
| velocity | 2000 | 500 / 1000 / 1500 / 3000 / 5000 | µs/s |
| acceleration | 8000 | 2000 / 4000 / 6000 / 12000 / 20000 | µs/s² |
| deadband | 10 | 0 / 5 / 15 / 25 / 50 | µs |

Cycling wraps. If you load a JSON with a custom value (e.g.
velocity = 1750) that isn't a preset, the next `v` press jumps to
the first preset (2000). The cycle is for the bench session; the
actual saved value is whatever was last selected.

### Atomicity

`CFG_PatchFile()` writes to `<path>.cfgtmp.<pid>` then `rename(2)`s
into place. POSIX rename is atomic — readers see either the old file
or the new, never a half-written one. If the write fails partway
(disk full, permission revoked), the original is untouched.

### Surgical edit, not regenerate

The patcher locates each target key as text and splices in the new
values. It does **not** serialize a full struct back to JSON. This
means:

- Fields the C parser ignores (notably `gesture_velocity`, owned by
  cpcu_dsp.py) are preserved byte-for-byte.
- Comments, indentation, ordering, and `// foo` comment-keys all
  survive untouched.
- Adding a new field to the schema doesn't require a save-path
  update — pca_testbench only writes what it knows about.

### Bias semantics

When you press `b` with the selected servo jogged to position `X`,
`servo_bias[i] = X - 1500`. The cpcu_io runtime then adds this to
every smoothed pulse before clamping. So if the servo's natural
"neutral-feeling" pose corresponds to a 1518 µs pulse, bias becomes
+18, and every motor command for that channel gets +18 added before
PCA write — putting the visually-neutral pose at command-time-1500.

Range is clamped to ±100 µs at the testbench (defensive — anything
larger probably means the calibration is wrong, not that you need
huge bias). The C parser enforces the same range on load.

### Refusing to save

If pca_testbench was launched without a `runtime.json` (no `--config`
arg, no symlink, no in-repo file), the `S` key prints
`SAVE FAILED: no config file was loaded`. Solutions:

```bash
# Re-launch with explicit path:
sudo ./pca_testbench --config /absolute/path/to/runtime.json

# Or create the symlink setup_pi.sh would have made:
sudo ln -s "$(pwd)/cpcu_v2/config/runtime.json" /opt/cpcu/config.json
```

### Why this is bench-only

`pca_testbench` writes I2C directly. If `cpcu_io` is also running,
**both processes are writing PCA channels at 50 Hz over the same
I2C bus** — the values fight and tear. Always stop the live system
before running pca_testbench. The cfg-loaded path is convenience
for a separately-controlled bench session, not a way to coexist.

The TUI's edit-mode (see [`TUI_EDITOR.md`](TUI_EDITOR.md) §4) is the
right tool when the system is up and you want to tweak runtime
values without killing the arm. Different layer, different tool.

### How `cpcu_io` picks up the saved values

When `pca_testbench` saves and you restart (or `kill -HUP
$(pgrep cpcu_kernel)`), cpcu_kernel re-parses runtime.json and
republishes `IPC_RuntimeConfig` with a bumped `config_seq`. cpcu_io
notices the seq change on its next servo tick (~20 ms latency) and
calls `apply_runtime_smoother_cfg()`, which re-applies every per-
channel value via `SMOOTH_SetSpeed` / `SMOOTH_SetAccel` /
`SMOOTH_SetDeadband`. Servo bias is read from `cfg_cache` directly
on every PCA write — no separate apply step.

The seq-compare avoids reapplying every tick when nothing's changed.
On a typical session the apply runs once at boot and once per
`kill -HUP`. The cost when it does run is ~3 setter calls × 6
channels = 18 cheap memory writes — negligible against the 50 Hz
servo cadence.

### Live editing from the TUI (deferred to v2.3.7)

The TUI's edit-mode handshake (v2.3.4) parks the arm at neutral so
runtime values can be edited safely, but the actual numeric editor
on the CONFIG page is deferred. Today the banner says "EDITING —
arm parked, Press 'e' to exit, Ctrl+S to save (planned)".

Building the editor is a meaningful chunk of curses code (cursor
navigation through editable fields, in-place numeric entry,
validation, draft-vs-saved state, the Ctrl+S commit via
`CFG_PatchFile`) and is the right v2.3.7 deliverable. The patcher
infrastructure is ready — the Ctrl+S commit can use the same
`CFG_PatchFile` API pca_testbench uses today.

For now, the operator workflow when the live system is up is:
edit `runtime.json` in another terminal, `kill -HUP cpcu_kernel`,
the smoother re-applies within 20 ms.

---

**See also:**
- [`USER_GUIDE.md`](USER_GUIDE.md) — operator-facing walkthrough that uses this doc as reference.
- [`ARCHITECTURE.md`](ARCHITECTURE.md) — system-level architecture; how the cores, IPC, and processes fit together.
- [`TESTING.md`](TESTING.md) — the test suite this doc's safety guarantees rely on.
