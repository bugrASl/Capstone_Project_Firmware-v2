# TUI Live Editor — In-System Runtime Tuning

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

## 4. The save protocol

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

## 5. The `kernel_pid` field in IPC

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

## 6. What's NOT editable (and why)

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
See [`CPCU_CONFIGURATION.md`](CPCU_CONFIGURATION.md).

---

## 7. Visual layout

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

## 8. Interaction with safety

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

## 9. Testing

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

## 10. Operating procedure

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

## 11. See also

- [`EDIT_MODE.md`](EDIT_MODE.md) — the v2.3.4 handshake protocol
  the editor sits on top of. Explains banner states, DSP UNRESPONSIVE
  timeout, SAFE-has-priority semantics.
- [`RUNTIME_CONFIG.md`](RUNTIME_CONFIG.md) — schema for every field
  the editor surfaces. §10 covers pca_testbench round-trip (the same
  `CFG_PatchFile` infrastructure).
- [`SOFT_GRIP.md`](SOFT_GRIP.md) — `grip_firm_us` and friends are
  the most commonly tuned values, and the editor's main use case.
- [`VELOCITY_MODE.md`](VELOCITY_MODE.md) — explains why
  `gesture_velocity` isn't editable from the TUI.
- [`CPCU_ARCHITECTURE.md`](CPCU_ARCHITECTURE.md) §3.3 — core
  allocation. The editor lives entirely on Core 0 (with the rest of
  the TUI).
- [`cpcu_v2/include/cpcu_tui_editor.h`](../include/cpcu_tui_editor.h) —
  full editor API.
- [`cpcu_v2/src/cpcu_tui_editor.c`](../src/cpcu_tui_editor.c) —
  state machine + render + save logic.
- [`cpcu_v2/test/editor_testbench.c`](../test/editor_testbench.c) —
  TB-ED01..ED05 unit tests.
