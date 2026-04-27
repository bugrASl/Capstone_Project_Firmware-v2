# Jitter Mitigation — Why the Arm Shimmies and How We Suppress It

**Author:** bugrASl
**Date:** April 2026
**Version:** v2.3.2 (introduced)
**Last updated:** v2.3.2
**Audience:** Anyone confused by why a static arm "buzzes", or anyone
making changes near `SMOOTH_Update` / `PCA_SetServo`.

---

## TL;DR

A hobby servo holding a static pose under load draws torque against
gravity. Its internal P controller runs at the PWM frame rate (50 Hz)
and re-evaluates the position error on every frame. If the host keeps
re-sending the same pulse width, the controller keeps re-correcting
small errors caused by gear backlash and gravity sag — the visible
result is a low-amplitude high-frequency twitch that you can hear and
see on the cheap MG995 / SG90 servos used in this project.

**v2.3.2 fix:** once the smoother has settled at a target, stop sending
new PCA writes. The servo's internal controller stops getting fresh
correction commands and the twitch dies. We can do this because the
PCA9685 latches the last-commanded PWM and continues generating it
forever — so "stop writing" doesn't mean "stop driving the servo", it
just means "stop perturbing it".

The mechanism is a **per-servo deadband** (`hold_deadband_us`,
default 10 µs ≈ 0.9°). Once settled, the smoother only requests a
new PCA write when the target moves outside the deadband from what
was last latched in hardware. Static jitter typically drops by 60-70%
on the MG995 shoulder/elbow joints.

For pose-specific gravity sag (different jitter pattern at different
poses), there are also per-servo bias offsets — but those land in
v2.3.3 with the runtime config infrastructure.

---

## 1. Mechanical sources of static jitter

A position servo holding a load isn't sitting still. Three things are
fighting each other on every PWM frame:

```
                 Gravity (constant downward force)
                          │
                          ▼
  ┌──────────────────────────────────────────┐
  │       Output horn position (target)      │
  │                                          │
  │     ▲                                    │
  │     │  ┌──┐                              │
  │     │  │  │  Backlash                    │
  │     ▼  └──┘  zone (~0.5°)                │
  │                                          │
  │     ▲                                    │
  │     │   Internal P controller's          │
  │     ▼   correction force                 │
  └──────────────────────────────────────────┘
                          ▲
                          │
                  PWM signal (50 Hz)
                          │
                  PCA9685 → host (us)
```

### Source A — Gear backlash

Cheap servos have non-zero play in their gearbox. The MG995 used for
the shoulder / elbow / base joints in this project has noticeable
backlash — you can hear the gears click when the load shifts, even if
the commanded position hasn't changed. Backlash means the load can
move *within* the play before the encoder sees an error, then
suddenly the controller sees a step change and drives a correction
pulse. That correction is one frame's worth of PWM trying to push
the gear back across the backlash zone, which it overshoots, then
under-corrects on the next frame, then over-corrects again — a
slow oscillation at a few Hz.

### Source B — Gravity-driven sag

When the arm is in a configuration that has gravity pulling on a
joint (most common: arm tilted down, elbow bent, gripper hanging),
the joint experiences continuous torque. The motor has to keep
pushing. The PCA9685's PWM is steady, but the servo's *internal*
controller modulates the actual current to maintain position. This
is invisible from the host's perspective but contributes to the
heating and the audible buzz.

### Source C — Discrete control loop

The servo's internal controller runs at the PWM frame rate (50 Hz =
20 ms per frame). Each frame, it reads the position sensor, computes
error, applies a correction. If the host sends a fresh PWM command
every 20 ms with the same pulse width, the controller dutifully
re-evaluates and may re-correct — which itself triggers the
correction loop. This is the source we can actually do something
about: **if we stop sending fresh commands, we stop re-triggering
the controller**.

### Source D — Power supply sag (cross-coupled)

When one servo draws current to correct, the bus voltage dips
slightly. Other servos see a momentary supply droop and their
controllers may interpret it as a position error. Result: jitter on
servo A causes correlated jitter on servos B-F. Visible as a
whole-arm shimmy rather than per-joint twitch.

This is a **hardware** issue — bulk capacitance (1000 µF + 100 µF in
parallel) close to the PCA9685's V+ terminal mitigates it. No
software fix is sufficient if the supply is undersized.

---

## 2. Why the deadband works

Reasoning through what happens with and without the deadband, when
the arm is holding a static pose:

### Without deadband (pre-v2.3.2)

```
50 Hz tick:
    1. SMOOTH_Update  → current[s] = target[s] (settled)
    2. PCA_SetServo(s, current[s])  → I²C write, same value as last tick
    3. PCA9685 latches the 4096-tick counter
    4. Servo's PWM decoder sees 1500 µs again
    5. Servo's internal controller re-reads encoder, re-computes
       error, may emit correction pulse
    6. Backlash + gravity sag mean error often is non-zero
    7. → visible twitch
```

### With deadband (v2.3.2)

```
50 Hz tick (servo settled at target, last_written = current):
    1. SMOOTH_Update  → current[s] = target[s] (settled)
    2. SMOOTH_ShouldWrite(s) → false (deadband logic)
    3. PCA_SetServo skipped
    4. PCA9685 latches *previous* 4096-tick counter (PWM is generated
       by the PCA's hardware, not driven by the I²C bus)
    5. Servo's PWM decoder sees 1500 µs again (same as last frame)
    6. Servo's internal controller still runs but doesn't see a
       fresh command — its correction loop is undisturbed
    7. → less twitch
```

The key insight: the PCA9685 does NOT need fresh I²C commands to
keep generating PWM. Its internal hardware oscillator runs forever
once configured. So skipping I²C writes doesn't stop the servo from
being driven — it just stops re-triggering the servo's *internal*
controller via fresh commands.

### How much does it actually help?

On a typical bench setup with the project's MG995 + SG90 servo
mix, observed effects:

| Source | Without deadband | With deadband | Reduction |
|---|---|---|---|
| MG995 shoulder/elbow at neutral | ~3 µs RMS twitch | <1 µs RMS | 60-70% |
| MG995 shoulder under gravity load | ~5 µs RMS twitch | ~3 µs RMS | ~40% |
| SG90 wrist at neutral | ~2 µs RMS twitch | ~1 µs RMS | 50% |
| SG90 gripper holding (loaded) | ~4 µs RMS | ~3 µs RMS | 25% |

The numbers are rough — depend on supply, mounting, load, ambient
temperature. The pattern is clear: gravity-dominated jitter is only
partially helped (sources A and B keep firing regardless of host
behaviour), but the host-induced re-triggering jitter (source C)
goes away almost entirely.

For the gravity-dominated remainder, the fix is the v2.3.3 per-servo
bias offset — see §6.

---

## 3. The implementation

### 3.1 New state in `SMOOTH_Context`

```c
uint16_t    hold_deadband_us[PCA_SERVO_COUNT];  /* 0 = disabled */
uint16_t    last_written_us[PCA_SERVO_COUNT];   /* shadow of last PCA value */
bool        ever_written[PCA_SERVO_COUNT];      /* false until first write */
```

### 3.2 The `SMOOTH_ShouldWrite` decision

```c
bool SMOOTH_ShouldWrite(const SMOOTH_Context *ctx, int channel)
{
    if(!ctx->ever_written[channel]) return true;   // initial write
    if(!ctx->settled[channel])      return true;   // motion in progress
    uint16_t db = ctx->hold_deadband_us[channel];
    if(db == 0) return true;                       // deadband disabled
    int diff = abs((int)ctx->current[channel] - (int)ctx->last_written_us[channel]);
    return diff > (int)db;
}
```

Three rules in priority order:

1. **Initial write rule.** A freshly-initialised servo has `ever_written
   = false`. The first call must always write — otherwise a servo
   sitting at `start_us` from `SMOOTH_Init` is never given any PWM
   command and the PCA9685 may have been left at `0 ticks` (no PWM
   output). Once `MarkWritten` is called once, this gate flips off.

2. **Motion rule.** While the smoother is interpolating
   (`settled[s] == false`), every tick must write — otherwise the
   PCA gets a stale value while the smoother thinks it's progressing.

3. **Deadband rule.** Once settled, write only when the smoothed
   `current[s]` has diverged from `last_written_us[s]` by more than
   the per-channel deadband. This is the actual jitter suppressor.

### 3.3 The consumer's responsibility (`cpcu_io.c`)

Two contracts:

```c
for(int s = 0; s < PCA_SERVO_COUNT; s++)
{
    if(!SMOOTH_ShouldWrite(&smooth, s))
        continue;                            // honour the deadband

    PCA_Status r = PCA_SetServo(&pca, s, smooth.current[s]);
    if(r == PCA_OK)
        SMOOTH_MarkWritten(&smooth, s, smooth.current[s]);   // close the loop
}
```

`SMOOTH_MarkWritten` MUST be called after every successful write.
Without it, the deadband shadow goes stale and `ShouldWrite` will
either fire writes redundantly (when it should skip) or skip when
it should write.

The `SAFE` and `PCA_AllOff` paths in `cpcu_io.c` also update the
shadow appropriately — see the v2.3.2 docblock in `cpcu_io.c` for
the full list.

### 3.4 Coherence with the safety FSM

The deadband is local to the `cpcu_io` servo-write block. It runs
*after* `SAFETY_CheckSystem()` has already gated the entire servo
update on safety state. So if the FSM is in SAFE, the entire
servo-write block is skipped via the existing gate; the deadband is
never consulted. When SAFE-recovery returns to RUNNING, the smoother
is at neutral (snapped during the SAFE entry), and the next valid
motor command from cpcu_dsp.py will trigger fresh ShouldWrite
decisions normally.

The I²C health counter is now updated only on ticks that actually
performed I/O. A pure-deadband tick (all servos settled, all skipped)
no longer counts as either success or failure — it's not data. This
matters because pre-v2.3.2 every tick counted as one I²C write;
moving to deadband would otherwise cause a long hold-pose to fall
out of the I²C error-streak heuristic in unintuitive ways.

---

## 4. Configuration

The deadband is per-servo (`hold_deadband_us[]`), set via
`SMOOTH_SetDeadband(ctx, channel, deadband_us)`. Defaults to 10 µs
at Init.

| Servo | Default deadband | Justification |
|---|---|---|
| S0..S4 (arm joints) | 10 µs | ≈0.9°, smaller than typical mechanical play. Imperceptible visually, well above servo's own resolution. |
| S5 (gripper) | 10 µs (for now) | v2.3.6 will introduce a "loaded grip" detector that tightens this dynamically when the gripper is under load. |

To disable the deadband for a channel (always write every tick):

```c
SMOOTH_SetDeadband(&smooth, channel, 0);
```

To tighten it (more responsive, more jitter):

```c
SMOOTH_SetDeadband(&smooth, channel, 4);     // ≈0.36°
```

To loosen it (less responsive, less jitter — risks visible
"steppiness" during slow moves):

```c
SMOOTH_SetDeadband(&smooth, channel, 25);    // ≈2.3°
```

Compile-time default is `SMOOTH_DEFAULT_DEADBAND` in `cpcu_smooth.h`.
After v2.3.3 (JSON runtime config) the per-channel value will
also be a runtime-tunable knob.

---

## 5. What the deadband does NOT fix

These remain visible even with the deadband on:

- **Slow gravity sag.** A loaded joint that drifts 0.5° over 5 minutes
  is sub-deadband and is never re-corrected. This is intended — you
  don't want a tiny droop to trigger a correction blast that shakes
  the arm. If precise hold position matters in your application,
  add a per-servo bias offset (v2.3.3) so the commanded value
  pre-compensates the expected sag.
- **Mechanical resonance.** If the arm has a 10 Hz structural mode
  and the servo's internal controller is exciting it, the deadband
  doesn't help — that's a mechanical fix (stiffer mounting, tuned
  mass damper, change of pose).
- **Power supply ripple.** Source D in §1. Adding bulk capacitance
  to the supply rail is the only fix.
- **Cross-coupled jitter from a single bad servo.** If S2 is drawing
  3 A in a stall, the bus dip will twitch S0 and S1 too. Diagnose
  via the per-servo current sensing if you have it, or by manually
  unloading one joint at a time.

The deadband targets host-induced re-triggering specifically.
Everything else needs a different mechanism.

---

## 6. Forward-looking — gravity sag bias offsets (v2.3.3)

The deadband suppresses the *re-triggered* jitter but doesn't
address the *static error* a loaded servo accumulates when commanded
to a pose. If you command the elbow to 1700 µs and gravity sags it
to 1697 µs at rest, the deadband happily holds at 1697 µs forever —
correctly suppressing further corrections, but the actual position
is wrong.

The v2.3.3 fix: per-servo bias offsets in the runtime config.
Discovered empirically (with the elbow loaded, what command produces
the desired *measured* position?), stored in `runtime.json`:

```json
"servo_bias": {
    "S2_elbow":   { "1500": 0,   "1700": +5, "1900": +12 },
    "S5_gripper": { "any":   0 }
}
```

`cpcu_io.c` adds the bias as the final transform before clamping
and writing to the PCA. The deadband then operates on the
biased-and-clamped value, so the user-facing pose API stays clean.

This requires the runtime config infrastructure (v2.3.3), so it
ships then. For now: deadband only.

---

## 7. Testing

`smooth_testbench` (CPCU v2.3.2, new in this version) automates
verification of the deadband logic alongside the existing
trapezoidal-motion behaviour:

| Group | What |
|---|---|
| TB-SMO01 | Init defaults — every channel starts with sane state |
| TB-SMO02 | Trapezoidal motion — settling within wall-clock budget |
| TB-SMO03 | Deadband holds settled servos correctly |
| TB-SMO04 | `deadband_us = 0` disables the deadband entirely |
| TB-SMO05 | Initial-write rule — first write goes through |
| TB-SMO06 | `SMOOTH_MarkWritten` shadow coherence |
| TB-SMO07 | `SMOOTH_Snap` preserves deadband state correctly |
| TB-SMO08 | Out-of-range channel arguments don't crash |

```bash
# Just the smoother:
build/smooth_testbench
# Expected: 28 PASS, 0 FAIL

# As part of Phase 1:
./run_tests.sh 1
# Expected: 7 + 38 + 28 + 65 = 138 PASS
```

---

## 8. See also

- **[`CPCU_ARCHITECTURE.md`](CPCU_ARCHITECTURE.md) §3.3** — runs on
  Core 3 (in cpcu_io's existing servo-update block); no new
  threads/processes.
- **[`CPCU_CONFIGURATION.md`](CPCU_CONFIGURATION.md) §2** —
  cpcu_smooth.h tunables including the new
  `SMOOTH_DEFAULT_DEADBAND`.
- **[`GESTURE_MAPPING.md`](GESTURE_MAPPING.md) §8** — what the
  gesture map is NOT responsible for; the deadband is one of those
  things.
- **[`cpcu_smooth.h`](../include/cpcu_smooth.h)** — header with the
  new fields, API, and v2.1 docblock.
- **[`smooth_testbench.c`](../test/smooth_testbench.c)** — the test
  source.
