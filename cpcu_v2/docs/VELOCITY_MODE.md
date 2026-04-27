# Velocity-Mode Gestures — Stateful Target Integration

**Author:** bugrASl
**Date:** April 2026
**Version:** v2.3.5 (introduced)
**Last updated:** v2.3.5
**Audience:** Anyone tuning gesture-to-motion mappings, anyone
debugging "why does my arm only have one fixed pose per gesture?"

---

## TL;DR

Until v2.3.4, every gesture mapped to a fixed servo pose: hold
`biceps_flex` → arm at position X, holding longer doesn't move the
arm any further. Visible behaviour was a step function of detected
class.

v2.3.5 adds **velocity mode**. Per-gesture, per-servo rates (in
µs/s) live in `runtime.json`'s `gesture_velocity` object. While a
velocity-mode gesture is detected, cpcu_dsp.py **integrates** the
servo target every inference tick:

```
target[s] += rate[s] × dt × confidence_scale
```

Holding the gesture longer = arm closes deeper. Releasing snaps to
"rest" (freeze mode) which holds the last position. Re-engaging the
gesture continues integrating from where you left off.

`rest` is special: it's always freeze-mode and snaps target back
toward neutral, so a long rest period drains the arm to home pose.

The system is **hybrid**: any class without a velocity entry stays
in freeze mode using the legacy `GESTURE_SERVO_MAP` fixed pose,
preserving v2.3.4 behaviour as the safe default.

---

## 1. Why velocity mode

The original fixed-pose model had two problems:

**No fine control.** A flex either snapped the elbow to 1700 µs or
left it at neutral. There's no way to express "close partway",
"close more", or any in-between state. Real prosthetic users want
graded control — squeeze gently, then squeeze harder.

**No state persistence.** A momentary EMG dropout (signal noise,
brief muscle relaxation, classifier hiccup) immediately collapses
the arm back to neutral. The user has to re-flex from scratch.

Velocity mode fixes both:
- Hold the gesture for 2 s instead of 1 s → arm moves twice as far.
- Brief detection dropouts hold the current target instead of
  resetting it. The arm "remembers" where it was.
- Confidence-scaled integration speed means weak/wavering gestures
  creep slowly while strong sustained gestures move at full speed.

---

## 2. The integration formula

Run every inference tick (~10 Hz):

```
dt              = now - last_integrate_t
conf_frac       = svm_probability(current_state) / 100.0

if conf_frac <= interp_floor:           # 0.40 default
    scale = 0.0                         # gesture too weak; freeze
elif conf_frac >= interp_ceil:          # 0.85 default
    scale = 1.0                         # full speed
else:
    scale = (conf_frac - floor) / (ceil - floor)    # linear lerp

if behavior[current_state].mode == "freeze":
    if current_state == "rest":
        target = neutral                # rest drains to home
    else:
        target = target                 # other freeze classes hold
else:    # velocity mode
    for s in range(NUM_SERVOS):
        target[s] += rate[s] * dt * scale
        target[s] = clamp(target[s], SERVO_MIN_US[s], SERVO_MAX_US[s])

publish_motor_cmd(target)
```

A few things worth noting:

**`dt` is real wall-clock**, not a fixed cadence assumption. The
inference loop *aims* for 100 ms but actual ticks vary with system
load. Using real `dt` keeps motion smooth across jitter.

**Confidence-scaled integration is intentional.** A high-confidence
sustained gesture moves at full velocity. A wavering 50% confidence
signal moves at ~25% velocity. This couples the SVM's uncertainty
directly into how aggressively the arm responds — uncertain user =
slow arm = safer.

**Clamping is per-servo.** Each channel has its own min/max
(mechanical limits, NOT the runtime-tunable `servo_min_us`). Once a
channel saturates, further integration is a no-op. cpcu_io clamps
again on its side after applying `servo_bias_us`, so the runtime
config can't escape the safety envelope.

**The integrator runs only outside edit mode.** While
`edit_mode_request` is set, dsp commits to "rest", clears the target
to neutral, and stops publishing — see EDIT_MODE.md §1.

---

## 3. Configuration shape

The schema is documented in
[`RUNTIME_CONFIG.md`](RUNTIME_CONFIG.md) §2. The relevant subset:

```json
{
    "interp_conf_floor_pct": 40,
    "interp_conf_ceil_pct":  85,
    "hysteresis_votes": 3,
    "gesture_velocity": {
        "rest":         [0, 0, 0, 0, 0, 0],
        "biceps_flex":  [0, 200, 0, 0, 0, 0],
        "hand_flex":    [200, 0, 0, 200, 200, 200],
        "hand_open":    [-200, 0, 0, -200, -200, -200]
    }
}
```

### Key rules

- **Class names must match `model.classes_` exactly.** Unknown names
  get a warning and are dropped. Renaming a class in the trained
  model means you have to update this JSON to match.
- **6 entries per row, indexed by servo channel.** Order:
  S0=Base, S1=Upper, S2=Last, S3=Joint1, S4=Joint2, S5=Gripper.
- **Negative values reverse direction.** A `hand_open` row with
  negative rates is the natural antagonist of `hand_flex` —
  same magnitude, opposite sign.
- **All-zero rate row is freeze-mode.** Equivalent to omitting the
  row entirely.
- **Range is ±5000 µs/s.** Out-of-range values get clamped loudly
  rather than silently accepted or rejected. 5000 µs/s = full servo
  range (498→2500 ≈ 2000 µs of travel) in 0.4 seconds — already
  faster than most users want.

### Tuning advice

Start gentle: 100-200 µs/s. With the default `interp_floor=40` and
`interp_ceil=85`, an active gesture at 80% confidence integrates at
~89% × rate = ~178 µs/s. So 200 µs/s feels like "noticeable motion
over ~1 second". 500 µs/s feels rapid; 1000+ µs/s is hard to control
in real-time EMG.

If the arm overshoots, lower the rate. If you can't tell the gesture
is engaged, raise the rate or lower `interp_floor_pct`.

---

## 4. Runtime/static split

The dsp side reads `runtime.json` directly using Python's
`json` module — separate from the C-side parser in `cpcu_config.c`
that publishes `IPC_RuntimeConfig` to shared memory.

Why two parsers? The string-keyed `gesture_velocity` map doesn't
fit the C parser's flat-array model. Adding strings + nested
objects to the C parser would be ~150 lines of code for a feature
that only one consumer (dsp) needs. Doing the parse in Python is a
few `json.loads` calls.

**Consequence**: the IPC `gesture_velocity[][]` field (declared in
`cpcu_ipc.h` since v2.3.3) stays zero-filled in v2.3.5. It exists
only as forward-compat reserve for a possible future C consumer
(no current need).

**Reload semantics**: dsp re-reads `runtime.json` only on full
restart. SIGHUP to cpcu_kernel re-parses the C-side fields and
republishes IPC, but does NOT re-trigger dsp's reload. To pick up
new gesture rows you must restart cpcu_dsp.py:

```bash
sudo systemctl restart cpcu                  # restarts the whole bundle
# or
kill $(pgrep -f cpcu_dsp.py)                 # supervisor respawns it
```

This is intentional — gesture velocities meaningfully change
behaviour, and a hot-swap mid-gesture would be jarring. Restart is
the right cadence.

A future revision could add SIGHUP support to dsp specifically for
this field. Not yet.

---

## 5. Hybrid behaviour: freeze + velocity coexist

Not every class needs to be velocity-mode. A class without a
`gesture_velocity` row stays in freeze-mode with its existing
`GESTURE_SERVO_MAP` fixed pose. This means:

- v2.3.4 deployments upgrade to v2.3.5 with **no behaviour change**
  if the user doesn't add `gesture_velocity` rows.
- A subset of classes can be velocity (e.g. just `biceps_flex` and
  `hand_flex`) while others (e.g. a calibration `wave` class) stay
  fixed-pose.
- "rest" is hardcoded to freeze. It exists as the explicit "stop
  integrating" semantic — re-routing rest to a velocity row would
  fight the integrator and is not allowed.

The decision tree dsp uses, per inference tick:

```
if current_state in behavior_map:
    mode = behavior_map[current_state].mode
    if mode == "velocity":  → integrate (above formula)
    else:                   → freeze (hold target; if rest, snap to neutral)
elif current_state in GESTURE_SERVO_MAP:
    target = GESTURE_SERVO_MAP[current_state]      # legacy v2.3.4 fallback
else:
    target = neutral                                # defensive default
```

In practice the first branch wins for every class once you've added
`gesture_velocity` entries. The third branch should never fire if
the SVM and the JSON agree on class names — but it's there as a
safety net for class-name drift between training and deployment.

---

## 6. Boot rule, fault recovery, edit-mode interaction

**Boot.** Every fresh start initializes `current_target_us =
[neutral]*6`. There is **no snapshot persistence** across runs by
design: a stuck pose from yesterday's session shouldn't define
today's startup. On boot the arm is at neutral, and integration
proceeds from there.

**Fault recovery.** When `system_state` transitions to SAFE
(any safety FSM trigger), dsp snaps `current_target_us` back to
neutral. cpcu_io has already snapped the smoother to neutral on its
side, but dsp's mirror of the target needs explicit reset too —
otherwise when SAFE clears, dsp resumes integration from the
mid-gesture target it had at fault time, which is surprising.

**Edit mode.** While `edit_mode_request` is set, dsp commits state
to "rest", suspends publishing, and clears `current_target_us` to
neutral so the next exit doesn't snap to a held pose. cpcu_io's
edit-mode handshake (see EDIT_MODE.md) parks the arm at neutral
in parallel.

**Combination cases:**
- Boot + immediate SAFE: target stays at neutral (already there).
- Edit-mode entry mid-flex: target goes to neutral; cpcu_io parks.
- SAFE-clear after edit-mode exit: integration resumes from
  neutral (both edit-mode-exit and SAFE-clear reset to neutral).
- Edit-mode entry while in SAFE: edit-mode handshake completes
  (smoother is already at neutral, settles immediately).

---

## 7. Testing

`test/test_dsp_pipeline.py` adds six new tests for the runtime
config loader (TB-DSP11..TB-DSP16, 18 individual checks):

| Group | What |
|---|---|
| TB-DSP11 | Missing config file → defaults, no crash |
| TB-DSP12 | Velocity rows parsed correctly (positive + negative rates) |
| TB-DSP13 | Out-of-range rates clamped to ±5000, not silently accepted |
| TB-DSP14 | Unknown class names dropped with a warning |
| TB-DSP15 | floor ≥ ceil rejected (loader falls back to defaults) |
| TB-DSP16 | JSONC line comments + trailing commas tolerated |

```bash
./run_tests.sh 1
# RESULTS: 186 PASS in Phase 1 (was 168)
#   7 codec + 38 safety + 28 smoother + 30 config + 83 DSP
```

The integration math itself isn't unit-tested in v2.3.5. It runs
inside `run_inference()` which has no clean seam for synthetic
input — the test would have to mock `time.monotonic()` and
`ipc.write_motor_cmd`. Reasonable to add later but not blocking.

For end-to-end verification, the live system on hardware:

```bash
sudo ./scripts/launch.sh release

# Hold biceps_flex for 1 second, watch elbow servo position
# (any servo monitor or scope on the PCA output) — should ramp
# smoothly from neutral toward closed, not snap there.

# Release. Position holds (freeze mode on rest is "snap to neutral
# over the smoother's trapezoidal walk").

# Re-flex briefly. Position resumes ramping from where it stopped,
# not from neutral.
```

---

## 8. Operating procedure

### First-time setup of velocity gestures

```bash
# Edit runtime.json with your initial guess at velocities:
$EDITOR cpcu_v2/config/runtime.json

# Restart so dsp picks up new gesture map:
sudo systemctl restart cpcu

# Watch the log to confirm rows loaded:
journalctl -u cpcu -n 20 | grep "velocity-mode"
# [DSP]   velocity-mode 'biceps_flex': rate=[0, 200, 0, 0, 0, 0]
```

### Iterative tuning

```bash
# 1. Make small change to one rate.
# 2. Restart.
# 3. Test the gesture.
# 4. Repeat.
```

A typical session: pick one channel, halve or double the rate, see
if it feels better. Most gestures stabilize within 3-4 iterations.

### Reverting to v2.3.4 fixed-pose behaviour

Set every velocity row to all zeros, or delete the
`gesture_velocity` block entirely. The dsp falls back to
`GESTURE_SERVO_MAP` for every class.

```json
"gesture_velocity": {}
```

### Troubleshooting

**Arm keeps drifting in one direction even when at rest.** A
velocity row has a non-zero rate that's bigger than your `rest`
class can drain. Either lower the rate, raise `interp_floor_pct`
(so `scale=0` more aggressively), or check that `rest` is correctly
classified by inspecting the TUI's confidence display.

**Arm doesn't move at all.** Check the dsp log for "velocity-mode"
lines on startup — if absent, the JSON didn't parse. Try `python3
-c "import json; print(json.load(open('config/runtime.json')))"` to
catch syntax errors. Also check `interp_conf_ceil_pct` isn't set so
high that you never hit full scale.

**Arm motion is jerky.** dsp's inference cadence is ~10 Hz; cpcu_io
smooths between updates at 50 Hz. If motion is jerky, the smoother's
velocity/accel limits are probably too low — try raising
`smooth_velocity_us_per_s` or `smooth_accel_us_per_s2`.

---

## 9. See also

- [`RUNTIME_CONFIG.md`](RUNTIME_CONFIG.md) — full schema for
  `runtime.json`, including `gesture_velocity`. The runtime/compile
  split this builds on.
- [`EDIT_MODE.md`](EDIT_MODE.md) — handshake that pauses dsp's
  velocity integration during calibration.
- [`JITTER_MITIGATION.md`](JITTER_MITIGATION.md) — the smoother
  whose `SMOOTH_AllSettled()` cpcu_io uses; same smoother absorbs
  velocity-mode targets.
- [`CPCU_ARCHITECTURE.md`](CPCU_ARCHITECTURE.md) §3.3 — core
  allocation. dsp on Cores 1-2 owns the integrator; cpcu_io on
  Core 3 reads the published targets.
- [`cpcu_v2/scripts/cpcu_dsp.py`](../scripts/cpcu_dsp.py) v2.3.5 —
  `load_dsp_runtime_config()`, the velocity integrator block in
  `run_inference()`.
- [`cpcu_v2/config/runtime.json`](../config/runtime.json) — the
  config file with the example `gesture_velocity` block.
- [`cpcu_v2/test/test_dsp_pipeline.py`](../test/test_dsp_pipeline.py) —
  TB-DSP11..TB-DSP16 cover the loader.
