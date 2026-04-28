# Soft-Grip Policy + Stall Watchdog

**Author:** bugrASl
**Date:** April 2026
**Version:** v2.3.7 (introduced)
**Last updated:** v2.3.7
**Audience:** Anyone debugging "why does the gripper feel weak when
I hold hand_flex," anyone tuning grip parameters, anyone wondering
why there are two layers and not one.

---

## TL;DR

The gripper (servo 5) is the only channel that can damage itself —
it stalls against objects. v2.3.7 adds a **two-layer policy** to
prevent that:

1. **dsp soft-firm clamp.** In velocity mode, the integrator can't
   close the gripper past `grip_firm_us` (default 1100 µs) even if
   `hand_flex` is held longer. One-sided — opening direction is
   unaffected.
2. **io stall watchdog.** Hardware-protection backstop. If the
   smoother's *current position* sits at the mechanical floor for
   `grip_stall_recover_ms` (default 2000 ms) continuously, retreat
   to `grip_touch_us` (default 1200 µs) and clamp until the
   commanded target naturally rises.

Both run automatically. Both reuse runtime config fields that
have existed since v2.3.3 — no new schema entries.

---

## 1. Why two layers

You might ask: if the dsp clamp prevents the integrator from
reaching the floor, why does io need a watchdog at all?

Three reasons:

**Edit-mode jogs.** When the TUI live editor lands (v2.3.7+ TBD),
a user could manually drag the gripper target below `grip_firm_us`.
dsp isn't publishing during edit mode; the soft clamp wouldn't help.

**Bad config.** A misconfigured `grip_firm_us` (e.g. someone sets
it to 900 µs not realizing the mechanical floor is 976) would
let the integrator run all the way down. The watchdog catches it.

**Future input sources.** Anything that publishes motor_cmd —
WebSocket bridge (v2.4 candidate), test harness, replay mode — gets
hardware protection without each having to know about soft-grip.

The dsp clamp is **policy** ("don't ask me to close that hard");
the io watchdog is **mechanism** ("I won't physically stay there").
Standard layering — policy where the intent is, mechanism where
the hardware is.

---

## 2. dsp soft-firm clamp

In `cpcu_dsp.py`'s velocity-mode integrator block:

```python
for s in range(NUM_SERVOS):
    delta   = rates[s] * dt * scale
    new_v   = current_target_us[s] + delta
    new_v   = max(SERVO_MIN_US[s], min(SERVO_MAX_US[s], new_v))
    if s == 5 and new_v < grip_firm:
        new_v = grip_firm
    current_target_us[s] = new_v
```

The clamp happens **after** the hardware-envelope clamp, so it can
only raise the floor (never lower it past mechanical limits, which
would be unsafe in the other direction). It also only applies to
`s == 5` — the gripper. Other channels are unaffected.

`grip_firm_us` is loaded from `runtime.json` in
`load_dsp_runtime_config()`. The loader range-checks `[800..2200]`
and falls back to `GRIP_FIRM_US_DEFAULT = 1100` on out-of-range or
missing values. Same range as the C parser in `cpcu_config.c`.

### Why one-sided

The clamp is `if new_v < grip_firm: new_v = grip_firm`, not a
two-sided range. Reasons:

- `hand_open` integrates the gripper *upward* toward `grip_open_us`.
  A two-sided clamp would prevent opening past `grip_firm_us`,
  which is wrong.
- The mechanical floor (`SERVO_MIN_US[5]` = 976) is below
  `grip_firm_us` (1100). If `current_target` somehow lands between
  them (race, edge condition), the one-sided clamp pulls it up to
  the safe floor.
- Conceptually: `grip_firm` is the *deepest the integrator should
  ever ask for*. Above it = whatever you want. Below it = no.

### Tuning

If the gripper feels too weak (drops light objects), lower
`grip_firm_us`. If it stalls audibly (servo whining, current spike),
raise it. The default of 1100 µs is a starting guess for SG90 +
typical gripper geometry; adjust per build.

```bash
# From the bench, with the live system stopped:
sudo ./pca_testbench --config config/runtime.json
# (no direct grip_firm key in the bench — edit JSON for now;
#  TUI live editor will own this in v2.3.7+)
```

---

## 3. io stall watchdog

In `cpcu_io.c` immediately after `SMOOTH_Update`:

```
state: gripper_at_floor_since_us, gripper_stall_active,
       gripper_unstall_since_us

INACTIVE branch:
  if (current[5] AND target[5]) <= servo_min[5] + 5us:
    if first time: latch timestamp
    elif elapsed > grip_stall_recover_ms:
      FIRE: SMOOTH_SetTarget(5, grip_touch_us)
      gripper_stall_active = true
      io_gripper_stalls++
      log warning
  else:
    clear timestamp

ACTIVE branch:
  if smooth.target[5] < grip_touch_us:
    re-clamp to grip_touch_us  (overrides dsp's incoming target)
  if smooth.target[5] > grip_touch_us + 5us:
    if first time: latch unstall timestamp
    elif elapsed > 250 ms:
      gripper_stall_active = false
      log info "stall cleared"
  else:
    clear unstall timestamp
```

### Why detect on current AND target

If only `current[5]` were checked: the watchdog would fire even when
the user has *already released* the gesture but the smoother hasn't
caught up. Spurious.

If only `target[5]` were checked: the watchdog would fire whenever
dsp publishes a low target, even if the gripper hadn't physically
gotten there yet (e.g., motion in progress). Premature.

Requiring both means: the gripper has *physically arrived at the
floor* AND *is being told to stay*. That's a real stall.

### The 5 µs margin

`servo_min[5] + 5` not `servo_min[5]` exactly. Reasons:

- The smoother's trapezoidal motion may oscillate by ±1 µs around
  the target. Strict equality would flicker.
- The 1-µs deadband (default) writes are unsuppressed, so over
  several ticks the latched value drifts within the deadband.
- 5 µs is well below human-perceptible motion (1500 µs servo full
  travel ≈ 180°, so 5 µs ≈ 0.6° — inaudible/invisible).

### The 250 ms unstall debounce

After firing, the watchdog clamps `target[5]` to `grip_touch_us`.
The user's intent (via dsp) is still flowing in — they may keep
holding `hand_flex`. Eventually they'll release (`rest`) and dsp
will integrate the target back up toward neutral.

When does the watchdog clear? When the *commanded* target naturally
rises above `grip_touch_us + 5`. Why 250 ms debounce: a single tick
where dsp publishes neutral while the user releases a finger but
isn't fully relaxed yet shouldn't immediately re-engage closing.

The debounce is hardcoded at 250 ms (rather than runtime-tunable)
because there's no good reason to expose it. Smaller and it's
twitchy; larger and the user notices the gripper "lagging" on
release. 250 is a safe middle.

### SAFE clears watchdog

When safety-FSM transitions to SAFE, smoother snaps to neutral.
The watchdog is force-cleared in the same branch:

```c
gripper_stall_active       = false;
gripper_at_floor_since_us  = 0;
gripper_unstall_since_us   = 0;
```

Without this, post-recovery the gripper would still be clamped at
`grip_touch_us` despite having already snapped through neutral.

---

## 4. Diagnostics + visibility

`io_gripper_stalls` (uint32) lives in `IPC_Diagnostics`, allocated
from the existing `_reserved[5]` pool — **no IPC layout change**,
`IPC_VERSION` stays at 0x0204.

The TUI's HEALTH page (page 6) shows it as a row:

```
Gripper stalls    OK    0  (no watchdog activity)
Gripper stalls    WARN  3  (occasional retreats)
Gripper stalls    FAULT 7  (raise grip_firm_us in runtime.json)
```

Thresholds: 0 = green; 1-4 = yellow ("occasional retreats"); ≥5 =
red ("raise grip_firm_us"). The yellow band is wide because some
stalls during testing/tuning are normal — they only become a
problem if they keep happening in normal use.

The counter is `_Atomic` and only ever incremented (no reset),
matching the existing `safe_entries`, `pkts_dropped`, etc. counters.
A reboot clears it. There's no "clear counter" command — that's
deliberate, like all the other diagnostic counters.

### Reading the counter externally

```bash
# IPC region is shared memory at /dev/shm/cpcu_ipc.
# The diag block sits at offset 65856 (per CPCU_ARCHITECTURE).
# Easier to just watch the TUI's HEALTH page.
./cpcu_tui    # press '6' for HEALTH
```

---

## 5. Interaction with edit mode

Edit mode (v2.3.4) parks the arm at neutral when the user opens
the CONFIG page editor. Side effects on soft-grip:

- dsp suspends motor_cmd publishing → integrator is frozen,
  `grip_firm` clamp doesn't run (no integration to clamp).
- io's smoother is parked at neutral → `current[5]` ≈ 1500, far
  from the floor → watchdog stays inactive.

When edit mode exits, dsp resumes integration from neutral (the
target was reset on entry). The watchdog state was already cleared
either by SAFE (if it fired during edit-prep) or by the natural
above-touch trajectory of the smoother as the arm recovered.

Net effect: edit mode is "transparent" to soft-grip. No special
handling needed.

---

## 6. Configuration

Three fields in `runtime.json` (all already present from v2.3.3):

| Field | Default | Range | Consumer |
|---|---|---|---|
| `grip_firm_us` | 1100 | 800..2200 | dsp soft clamp |
| `grip_touch_us` | 1200 | 800..2200 | io watchdog retreat target |
| `grip_stall_recover_ms` | 2000 | 100..30000 | io watchdog timeout |

The dsp loader and the C parser both range-check these.
Out-of-range values fall back to defaults with a warning logged.

**Important relationship:** `grip_touch_us > grip_firm_us`. The
watchdog retreats from "pinned at floor" to *above* the firm
clamp, so when dsp resumes integration it doesn't immediately
ask for the floor again. If you set `grip_touch_us < grip_firm_us`,
nothing breaks but the watchdog's retreat is meaningless — the
firm clamp would already prevent the integrator from getting that
deep. The defaults (firm=1100, touch=1200) get this right.

---

## 7. Testing

`test/test_dsp_pipeline.py` adds TB-DSP17 (3 checks, in 1 group):
- absent `grip_firm_us` defaults to 1100
- present value parsed correctly (1150)
- out-of-range value (99999) rejected with warning, falls back to default

The io-side watchdog isn't unit-tested because it requires:
- IPC fixture with motor_cmd + diag regions
- a fake smoother with controllable position
- a fake clock for `t` advancement

Doable but ~150 lines of test scaffolding for a state machine that's
straightforward to read. Verified on hardware by:

```bash
# Live test:
sudo ./scripts/launch.sh release

# Hold hand_flex against an object for 3+ seconds.
# Within 2000ms of being pinned, the LOG_W line:
#   [IO] gripper stall watchdog fired -> retreat to 1200 us...
# appears in journalctl. The HEALTH page's 'Gripper stalls' row
# increments. Releasing the gesture and the watchdog clears.

journalctl -u cpcu -f | grep gripper
```

End-to-end test pending after first hardware build with object.

---

## 8. Operating procedure

### Adjusting firmness

Symptoms → action:

| Symptom | Likely cause | Adjustment |
|---|---|---|
| Drops light objects | grip_firm too high (jaws don't close enough) | Lower `grip_firm_us` (try 1080) |
| Servo whining when gripping | grip_firm too low (past mechanical sweet spot) | Raise `grip_firm_us` (try 1120) |
| Lots of "stall" warnings in logs | Watchdog firing too often | Raise `grip_firm_us` so dsp clamps before io has to |
| Watchdog never fires but jaws stall | grip_stall_recover_ms too long, or sensor margin too tight | Lower `grip_stall_recover_ms` (try 1500), or increase WD_MARGIN_US in cpcu_io.c |

Edit `runtime.json`, then `kill -HUP $(pgrep cpcu_kernel)`. The
smoother re-applies on `config_seq` change (~20 ms). dsp's
`grip_firm` is loaded once at startup — restart dsp specifically
for that change to take effect (or full system restart).

### Disabling soft-grip

For diagnostic purposes ("is the soft-grip what's making the
gripper feel weak, or is it the SVM?"), set:

```json
"grip_firm_us": 800,
"grip_stall_recover_ms": 30000
```

`grip_firm = 800` is below the mechanical floor (976), so the
clamp is a no-op. `grip_stall_recover_ms = 30000` makes the
watchdog effectively never fire during a normal session. Both
get you back to v2.3.6 behavior.

Don't ship like that. The watchdog protects hardware.

---

## 9. See also

- [`RUNTIME_CONFIG.md`](RUNTIME_CONFIG.md) — schema for the four
  `grip_*` fields. Both the C parser and dsp loader consume them.
- [`VELOCITY_MODE.md`](VELOCITY_MODE.md) — the integrator the soft
  clamp lives inside. The clamp runs only in velocity mode; freeze
  classes hold their target unchanged.
- [`EDIT_MODE.md`](EDIT_MODE.md) — interaction notes (§5 above).
- [`JITTER_MITIGATION.md`](JITTER_MITIGATION.md) — the smoother
  whose `current[5]` and `target[5]` the watchdog observes.
- [`CPCU_ARCHITECTURE.md`](CPCU_ARCHITECTURE.md) §3.3 — core
  allocation. dsp on Cores 1-2 owns soft-firm clamp; cpcu_io on
  Core 3 owns the watchdog.
- [`cpcu_v2/scripts/cpcu_dsp.py`](../scripts/cpcu_dsp.py) v2.3.7 —
  `GRIP_FIRM_US_DEFAULT`, the loader's 5th return value, and the
  one-sided clamp in the integrator.
- [`cpcu_v2/src/cpcu_io.c`](../src/cpcu_io.c) v2.3.7 — the
  watchdog state machine.
- [`cpcu_v2/include/cpcu_ipc.h`](../include/cpcu_ipc.h) — the
  `io_gripper_stalls` counter in `IPC_Diagnostics`.
- [`cpcu_v2/test/test_dsp_pipeline.py`](../test/test_dsp_pipeline.py) —
  TB-DSP17 covers the loader. io-side is hardware-tested.
