# v2.7 Test Fixes

Three issues found when running `./launch.sh test-sw` on a real Pi
after the v2.7 restructure:

## Issue 1: TB-DSP — `ModuleNotFoundError: No module named 'cpcu_dsp'`

**Cause:** `test_dsp_pipeline.py` had a hardcoded `sys.path.insert(...,
"../scripts")` from the v2.6 layout. In v2.7, Python modules moved to
`python/` so the import broke.

**Fix:** `test_dsp_pipeline.py` now adds **both** `../python/` (v2.7) and
`../scripts/` (v2.6 fallback) to `sys.path`.

**Drop into:** `cpcu_v2/test/test_dsp_pipeline.py`

## Issue 2: test_ipc_bridge.py — same problem

**Cause:** Same as Issue 1 — hardcoded `../scripts/` path. This one
didn't fire in your `test-sw` run because the IPC bridge test only
runs in Phase 2 (needs the kernel up), but it would have failed there.

**Fix:** Same approach as Issue 1.

**Drop into:** `cpcu_v2/test/test_ipc_bridge.py`

## Issue 3: TB-ED05g — editor testbench reads production config

**Cause:** The editor's path lookup tries `/opt/cpcu/config.json` first,
then falls back to `config/runtime.json`. The testbench's existing
strategy (chdir to /tmp, symlink temp file as `config/runtime.json`)
only works if `/opt/cpcu/config.json` doesn't exist. After
`./launch.sh setup`, that file is a symlink to your real
`config/runtime.json`, so the editor's first-tier lookup wins and
loads production config instead of the test's temp file. The save
test then sees the production value (2000) instead of the test value
(1500).

The original testbench printed a warning when it detected the
conflict but didn't actually solve it — the warning was a TODO that
nobody got back to.

**Fix:** the testbench now temporarily renames `/opt/cpcu/config.json`
to `/opt/cpcu/config.json.test_backup` for the duration of its run,
restores it via `atexit`. The directory is owned by your user (per
`./launch.sh setup`), so no sudo is needed.

**Drop into:** `cpcu_v2/test/editor_testbench.c`

## Issue 4: redundant PYTHONPATH setting

**Cause:** None — this was a defensive workaround I considered adding
to `run_tests.sh` but decided against once the test files self-locate
correctly.

**Fix:** `run_tests.sh` is unchanged — included in this bundle only
for completeness. You don't actually need to apply it.

**Drop into:** N/A — no behavioral change, optional.

---

## After applying

```bash
cd ~/prosthetic_arm/cpcu_v2
./launch.sh test-sw
```

Expected: 233 PASS, 0 FAIL. The three issues above were all in the
test code, not the production code, so no rebuild is needed for
these fixes (the testbench .c file change DOES require a rebuild —
`./launch.sh build` after dropping it in, before re-running tests).
