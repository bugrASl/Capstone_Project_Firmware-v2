# TUI Live Editor — In-System Runtime Tuning

> v2.7: this doc absorbed the previous standalone `TUI_EDITOR.md` §4
> as §4 "Edit-mode handshake protocol". The handshake is what
> makes the editor safe to use while the system is running; it
> deserves to live in the same doc as the editor itself.

**Author:** bugrASl
**Date:** April 2026
**Version:** v2.3.8 (introduced)
**Last updated:** v2.3.8
**Audience:** Anyone using the TUI to tune the live system, anyone
extending the editor with new fields, anyone debugging "I edited it
in the TUI but the change didn't take."

---

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
