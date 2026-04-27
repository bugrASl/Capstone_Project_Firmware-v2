# Edit Mode Handshake — Safe Live Calibration Without Killing the Arm

**Author:** bugrASl
**Date:** April 2026
**Version:** v2.3.4 (introduced)
**Last updated:** v2.3.4
**Audience:** Anyone calibrating gestures or per-servo bias via the
TUI, anyone debugging "why isn't my CONFIG page editable?"

---

## TL;DR

Press `e` on the TUI's CONFIG page (page 7) to enter edit mode. The
arm parks itself at neutral, the TUI banner turns green, and the
config knobs become editable. Press `e` again to exit; the arm
resumes normal gesture-driven motion. A safety fault (radio drop,
battery, thermal, etc.) forces edit mode off unconditionally.

You need this because changing a config value mid-motion is
visibly bad — the user is mid-flex and the elbow velocity changes
under them. Edit mode parks the arm first, then lets you edit, then
restarts cleanly.

The handshake involves all four cores:
- **TUI (Core 0)**: raises `edit_mode_request`.
- **cpcu_dsp.py (Cores 1-2)**: stops publishing motor commands; ACKs.
- **cpcu_io (Core 3)**: parks the smoother at neutral; flips
  `edit_mode_active` once `SMOOTH_AllSettled()`.
- **cpcu_kernel (Core 0)**: doesn't participate directly; safety FSM
  preserves its priority over edit mode.

---

## 1. Why a handshake at all

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

## 2. Wire-level protocol

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

## 3. The DSP UNRESPONSIVE timeout

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

## 4. What edit mode does NOT do (yet)

This v2.3.4 ships the **handshake mechanism**. It does not yet ship:

- **A live numeric editor** — the CONFIG page still shows
  the static spec sheet. Pressing `e` parks the arm; that's it.
- **Ctrl+S to save** — JSON write-back from TUI to runtime.json
  isn't wired yet. Edit by hand in the JSON file (kill -HUP to
  reload), or wait for a future revision.
- **Per-row editing UI** — for a future revision, each row of the
  CONFIG page becomes navigable with arrow keys, and the right
  column becomes editable.

The handshake itself is the foundation; the editor on top is
incremental. By shipping the handshake first, future edits can
trust that the arm is parked when the user is editing — that's the
safety-critical property and now it's in place.

If you want to edit runtime values today:

```bash
$EDITOR cpcu_v2/config/runtime.json
kill -HUP $(pgrep -f cpcu_kernel)        # reload
```

If you want to edit safety thresholds:

```bash
cd cpcu_v2
./configure.sh --radio-timeout 1000      # or --diff, --reset, etc.
cmake --build build
```

Both work without ever touching the TUI's edit mode. The TUI's edit
mode becomes important once the live numeric editor lands.

---

## 5. Safety guarantees

The handshake preserves several invariants:

**Safety FSM has priority over edit mode.** Any SAFE transition
clears `edit_mode_active` unconditionally (in cpcu_io's SAFE-snap
branch). edit mode cannot suppress fault detection.

**The arm is at neutral whenever editing is allowed.** `edit_mode_active`
goes true only after `SMOOTH_AllSettled()` reports settled at the
neutral pose. The user can never be editing while the arm is mid-trajectory.

**Stale ack from a previous session is harmless.** `edit_mode_dsp_ack`
is purely diagnostic; the TUI doesn't gate editing on it. If dsp
crashes mid-edit, the arm stays at neutral (cpcu_io is still
parking it) until the user releases `e`.

**Multiple back-to-back `e` presses are idempotent.** Each press
toggles the request bit. Pressing `e` twice quickly sets request to
1 then 0 — io's response is "park then immediately resume", which
the smoother handles cleanly via the existing trapezoidal profile.

**No race between request and active.** Single writer per byte,
atomic ops. The TUI can read both bytes in either order; the
state machine semantics tolerate any read order.

---

## 6. Core allocation

All four cores participate in the handshake. See
[`CPCU_ARCHITECTURE.md`](CPCU_ARCHITECTURE.md) §3.3 for the full table.

- **Core 0**: TUI handles the keystroke and writes `edit_mode_request`.
  cpcu_kernel doesn't participate (the SIGHUP-driven config reload
  is a separate mechanism).
- **Cores 1-2**: cpcu_dsp.py reads `edit_mode_request` once per
  inference window (~100 ms cadence), commits to rest, ACKs.
- **Core 3**: cpcu_io reads `edit_mode_request` every servo tick
  (50 Hz), drives the smoother to neutral, flips
  `edit_mode_active` based on `SMOOTH_AllSettled()`.

No new processes, no thread changes, no scheduling changes. The
handshake costs ~10 µs per servo tick on Core 3 (one atomic load
plus a smoother target check) and is negligible.

---

## 7. Testing

There's no dedicated unit testbench for the edit-mode handshake in
v2.3.4 — the protocol spans three processes plus shared memory, which
doesn't fit the existing single-binary unit-test model. Verification
is by demo and by inspection:

```bash
# Start the system in demo mode (no hardware needed):
./cpcu_tui --demo

# Press 7 to navigate to CONFIG page.
# Banner shows: Edit mode: [LOCKED]

# Press 'e':
# Banner switches to: Edit mode: [PARKING ARM...] (yellow)
# After a moment (depends on demo's smoother state):
# Banner switches to: Edit mode: [EDITING - arm parked] (green)

# Press 'e' again:
# Banner returns to: Edit mode: [LOCKED] (dim)
```

For the live system (Pi + hardware), the same flow works but the
PARKING phase actually waits for the physical arm to walk to neutral.
Watch the arm: the banner should turn green precisely when the last
servo stops moving.

The existing 168 Phase 1 tests are unchanged — none of them touch
the IPC ControlBlock layout or the edit-mode bytes. The IPC version
bump (0x0203 → 0x0204) does mean any external readers of
`/dev/shm/cpcu_ipc` need to be rebuilt, but the only external reader
in this project is `cpcu_ipc_bridge.py`, which is updated alongside.

A future revision should add an integration test: spin up cpcu_io
+ cpcu_dsp.py + a synthetic TUI-replacement, drive the request bit,
assert the active bit flips within a reasonable budget. That's a
~150-line test, and worth doing once the runtime config editor
actually ships.

---

## 8. Operating procedure

### Calibration session workflow

```
1. Start the system normally.
2. Navigate the TUI to CONFIG (press 7).
3. Press 'e'. Wait for the banner to turn green.
4. (Future) Use arrow keys + Enter to edit individual values.
   For now: open another terminal and edit
   cpcu_v2/config/runtime.json directly. kill -HUP to reload.
5. Test the new values: press 'e' to exit edit mode. The arm
   resumes responding to gestures with the new tuning.
6. Iterate.
7. (Future) Press Ctrl+S in edit mode to commit edits to JSON.
   For now: your JSON edits are already on disk if you used $EDITOR.
```

### Troubleshooting

**Banner stuck on PARKING.** The smoother isn't reporting settled.
Possible causes:
- A motor command is being injected (shouldn't happen if dsp is
  honoring the request, but check `pgrep -af cpcu_dsp.py` is alive).
- The smoother thinks it's mid-trajectory because the deadband is
  larger than the settle threshold. Check
  `smooth.hold_deadband_us[]` vs `SMOOTH_SETTLE_THRESH`.

**Banner red — DSP UNRESPONSIVE.** cpcu_dsp.py either crashed or
hasn't gotten to its IPC poll yet. Check `tail
/var/log/cpcu/log_DSP.csv` for errors.

**Banner shows EDITING but arm twitches.** Some other process is
writing to the PCA. Check that no `pca_testbench` instance is
running concurrently.

**Edit mode silently exits during a session.** A safety fault fired —
check `tail /var/log/cpcu/log_KERN.csv` for the FSM transition.
Re-press `e` after recovery.

---

## 9. See also

- [`CPCU_ARCHITECTURE.md`](CPCU_ARCHITECTURE.md) §3.3 — core
  allocation: which process touches which byte.
- [`RUNTIME_CONFIG.md`](RUNTIME_CONFIG.md) — the runtime tunables
  this edit mode is designed to expose for safe live editing.
- [`JITTER_MITIGATION.md`](JITTER_MITIGATION.md) — the deadband
  semantics that determine when `SMOOTH_AllSettled()` reports true,
  which is the gate cpcu_io uses to flip `edit_mode_active`.
- [`cpcu_v2/include/cpcu_ipc.h`](../include/cpcu_ipc.h) — the
  three new bytes + timestamp in IPC_ControlBlock.
- [`cpcu_v2/src/cpcu_io.c`](../src/cpcu_io.c) v2.3.4 — handshake
  responder.
- [`cpcu_v2/scripts/cpcu_dsp.py`](../scripts/cpcu_dsp.py) v2.3.4 —
  sticky-rest mode + dsp_ack.
- [`cpcu_v2/src/cpcu_tui.c`](../src/cpcu_tui.c) — page-7-local `e`
  keypress handling.
- [`cpcu_v2/src/cpcu_tui_render.c`](../src/cpcu_tui_render.c)
  `draw_page_config()` — banner rendering.
