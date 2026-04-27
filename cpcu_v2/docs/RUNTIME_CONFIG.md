# Runtime Config — JSON-Backed Tunables, Compile-Time Safety Defines

**Author:** bugrASl
**Date:** April 2026
**Version:** v2.3.3 (introduced)
**Last updated:** v2.3.3
**Audience:** Anyone changing a value in this project, anyone debugging
"why doesn't my edit take effect?", anyone wiring a new tunable.

---

## TL;DR

Everything you might want to tweak in this project falls into one of two
buckets:

**Bucket 1 — runtime-tunable.** Servo limits, gesture velocities,
smoother accel/deadband, grip levels, per-servo bias offsets. Change
these by editing `cpcu_v2/config/runtime.json` (or, in v2.3.4+, via the
TUI's edit mode). Send `SIGHUP` to `cpcu_kernel` to reload — no rebuild,
no restart of any other process.

**Bucket 2 — compile-time only.** Safety thresholds (radio timeout,
battery cutoffs, thermal limits), packet wire format, IPC schema
version, BSAU radio channel. Change these via `./configure.sh`, then
rebuild. There is no runtime escape hatch — these define what "safe"
means and live behind code review.

The split is enforced. The runtime config is mirrored to a shared-memory
region (`IPC_RuntimeConfig`); cpcu_io reads it once per loop and applies
the values, but every value is **clamped against the compile-time
hardware limits** before being written to the PCA. So a typo in
runtime.json can't drive a servo past its mechanical end-stop, no matter
how the JSON is corrupted.

---

## 1. The runtime/compile-time decision

When deciding where a new knob belongs, ask: **"if this is wrong, what
breaks?"**

| If wrong, breaks... | Goes in | Example |
|---|---|---|
| Calibration / feel | runtime.json | Servo neutral position, smoother accel, per-gesture velocity |
| User experience | runtime.json | Confidence threshold, hysteresis votes, grip touch depth |
| One-session bench tuning | runtime.json | Per-servo bias offsets, deadband per channel |
| Safety envelope | `#define` (configure.sh) | Radio timeout, VBAT critical, thermal CRIT, ring-overflow limit |
| Wire format | `#define` (configure.sh) | Packet structure, IPC version |
| Hardware identity | `#define` (configure.sh) | NRF channel, NRF address |

A heuristic: if you'd want to test the new value across a 5-minute
calibration session and revert it on a whim, it's runtime. If you'd want
a code review before changing it, it's compile-time.

---

## 2. The runtime side — `runtime.json`

### Location

`cpcu_v2/config/runtime.json` is the canonical source. It's
**git-versioned** so changes show up in `git diff` and you can revert
sessions. `setup_pi.sh` symlinks `/opt/cpcu/config.json` to it so
`cpcu_kernel` finds it at a stable system path.

### Schema

The full schema lives in `cpcu_v2/include/cpcu_ipc.h` as
`IPC_RuntimeConfig`. The JSON keys mirror the struct fields. As of
v2.3.3:

```json
{
    "schema_version": 1,
    "servo_min_us":             [ 498, 1074, 1074, 1001, 1001, 976  ],
    "servo_max_us":             [ 2500, 1953, 1953, 2002, 2002, 1733 ],
    "servo_bias_us":            [ 0, 0, 0, 0, 0, 0 ],
    "smooth_velocity_us_per_s": [ 2000, 2000, 2000, 2000, 2000, 2000 ],
    "smooth_accel_us_per_s2":   [ 8000, 8000, 8000, 8000, 8000, 8000 ],
    "smooth_deadband_us":       [ 10, 10, 10, 10, 10, 10 ],
    "interp_conf_floor_pct": 40,
    "interp_conf_ceil_pct":  85,
    "hysteresis_votes": 3,
    "grip_open_us":   1700,
    "grip_touch_us":  1200,
    "grip_firm_us":   1100,
    "grip_stall_recover_ms": 2000
}
```

`schema_version` is mandatory and must equal the parser's expected value
(currently 1). `servo_min_us` and `servo_max_us` are mandatory.
Everything else is optional — absent fields fall back to compile-time
defaults from `CFG_Defaults()` in `cpcu_config.c`.

### What v2.3.3 actually consumes

Right now, only `servo_bias_us` is wired to a real consumer (cpcu_io
applies it before the PCA write — see §6). The other fields are in the
schema but their consumers haven't shipped yet:

| Field | Consumer | Lands in |
|---|---|---|
| `servo_min_us`, `servo_max_us` | cpcu_io clamping | future v2.3.x (currently uses compile-time) |
| `servo_bias_us` | cpcu_io PCA write | **v2.3.3 (this version)** |
| `smooth_*` | cpcu_io smoother config | future v2.3.x |
| `interp_*`, `hysteresis_votes` | cpcu_dsp.py | v2.3.5 |
| `grip_*` | cpcu_dsp.py + cpcu_io | v2.3.6 |

The infrastructure is laid here so future steps can simply read the
field and apply it — no more IPC schema changes needed for the
established fields.

### Loading and reload

`cpcu_kernel` parses `runtime.json` once at startup, populates the
`IPC_RuntimeConfig` shared-memory region, then reloads on `SIGHUP`:

```bash
# Edit the file:
$EDITOR cpcu_v2/config/runtime.json

# Reload without restart:
sudo systemctl reload cpcu                  # if running under systemd
# or
kill -HUP $(pgrep -f cpcu_kernel)           # if running by hand
```

A failed parse on SIGHUP **keeps the previous values in IPC** and logs
the error. A failed parse on startup makes the kernel **refuse to
start** with a pointer to this doc. There is no silent fallback to
defaults — a missing or broken config is treated as an operator error.

### Validation

Every field is range-checked. Out-of-range values cause a parse
failure with a clear message:

```
[ERROR] [KERN] config load failed: value out of range
       (/opt/cpcu/config.json) — servo_max_us[5] = 9999 out of range [400..2600]
```

Min-must-be-less-than-max sanity is enforced. Optional fields that
break invariants (e.g. `interp_conf_floor_pct >= ceil_pct`) also fail.

### Atomic edits and concurrency

The IPC region uses a seqlock. Writers (only `cpcu_kernel`) bump the
`config_seq` counter to odd, write the payload, bump back to even.
Readers retry up to 4 times if they see an odd seq. So a SIGHUP-driven
reload mid-frame can't ever produce a torn read — readers see either
the old config or the new, never a hybrid.

When the v2.3.4 TUI edit mode lands, file writes will use
write-tmp-then-rename for atomic JSON updates.

---

## 3. The compile-time side — `configure.sh`

### What it edits

`cpcu_v2/configure.sh` knows about the safety thresholds and BSAU
radio channel. The current registry:

| Flag | File | Meaning |
|---|---|---|
| `--radio-timeout`   | `cpcu_safety.h` | `SAFETY_RADIO_TIMEOUT_MS` |
| `--radio-safe`      | `cpcu_safety.h` | `SAFETY_RADIO_SAFE_MS` |
| `--boot-grace`      | `cpcu_safety.h` | `SAFETY_RADIO_BOOT_GRACE_MS` |
| `--vbat-low`        | `cpcu_safety.h` | `SAFETY_VBAT_LOW_V` |
| `--vbat-crit`       | `cpcu_safety.h` | `SAFETY_VBAT_CRITICAL_V` |
| `--thermal-warn`    | `cpcu_safety.h` | `SAFETY_THERMAL_WARN_C` |
| `--thermal-crit`    | `cpcu_safety.h` | `SAFETY_THERMAL_CRITICAL_C` |
| `--i2c-max`         | `cpcu_safety.h` | `SAFETY_I2C_MAX_ERRORS` |
| `--ring-overflow`   | `cpcu_safety.h` | `SAFETY_RING_OVERFLOW_LIMIT` |
| `--nrf-channel`     | `bsau_app.h`    | `NRF_CHANNEL` (BSAU side) |

(`BSAU_MODE` is a multi-line uncomment-one-of-many switch — too
structural for sed editing. Edit `bsau_v2/Core/Inc/bsau_config.h` by
hand for that.)

### Usage patterns

```bash
cd cpcu_v2

# Interactive walk-through:
./configure.sh

# Show the current value of one knob:
./configure.sh --radio-timeout
# -> SAFETY_RADIO_TIMEOUT_MS = 750  (default 750, range 150..10000)

# Set one or more values:
./configure.sh --radio-timeout 1000
./configure.sh --vbat-low 3.1 --thermal-warn 70

# Inspect:
./configure.sh --show           # all values, marked [modified] if changed
./configure.sh --diff           # only the modifications

# Restore defaults:
./configure.sh --reset

# Limit interactive to one scope:
./configure.sh --bsau           # only BSAU tunables
./configure.sh --cpcu           # only CPCU tunables
```

After every edit the script prints a **REBUILD REQUIRED** banner. The
new value doesn't take effect until you run `cmake --build build` (CPCU)
or rebuild + flash via STM32CubeIDE (BSAU). configure.sh deliberately
does not run the build for you — that's a separate decision and you
might want to batch several edits before rebuilding.

### Safety rationale

These thresholds define what "safe" means for the system. Live-tunable
safety thresholds would mean a misclick or a corrupted JSON could
disable protection. Keeping them as `#define`s ensures:

1. Changes go through code review (commit + push).
2. Changes require a rebuild (deliberate, not accidental).
3. Changes are visible in `git log`.
4. The safety envelope is fixed for the lifetime of a binary.

The runtime config can shift everything *inside* this envelope — gestures
move faster, gripper closes deeper, deadband loosens — but the envelope
itself doesn't move at runtime.

---

## 4. Why bias-then-clamp matters

The consumer side of the runtime config in v2.3.3 is one specific
transform applied in cpcu_io's servo-write loop:

```c
int32_t biased = (int32_t)smooth.current[s] +
                 (int32_t)cfg_cache.servo_bias_us[s];
if(biased < (int32_t)pca.servo_min[s]) biased = pca.servo_min[s];
if(biased > (int32_t)pca.servo_max[s]) biased = pca.servo_max[s];
PCA_SetServo(&pca, s, (uint16_t)biased);
```

The `pca.servo_min`/`servo_max` here are the **compile-time** values
from `cpcu_pca9685.h`, NOT the runtime config's `servo_min_us`/`max_us`.
This is deliberate: the runtime values are calibration tweaks within the
hardware safety envelope, but they can't escape it.

Concretely, suppose someone fat-fingers `runtime.json` and sets
`servo_bias_us[2] = 500`. The smoother commands 1700 µs for the elbow.
Bias adds 500 → 2200. Compile-time max is 1953 → clamped to 1953. The
elbow rotates to its physical extreme but doesn't try to drive past it.
A bug in the JSON costs a misposed arm, not a stripped gear.

The same applies to every future runtime knob: the runtime layer
shapes behavior within the compile-time envelope, never escapes it.

---

## 5. SeqLock pattern (for IPC tinkering)

If you're adding a new field to `IPC_RuntimeConfig`, the seqlock
contract is:

**Writer** (only cpcu_kernel — single writer):

```c
atomic_fetch_add(&ctx->config->config_seq, 1, RELEASE);  // odd
memcpy(payload_ptr, src_payload, payload_size);
atomic_fetch_add(&ctx->config->config_seq, 1, RELEASE);  // even
```

**Reader** (any process):

```c
for(int try = 0; try < 4; try++) {
    uint32_t s1 = atomic_load(&ctx->config->config_seq, ACQUIRE);
    if(s1 & 1) continue;            // writer mid-update
    memcpy(local, ctx->config, sizeof(*ctx->config));
    uint32_t s2 = atomic_load(&ctx->config->config_seq, ACQUIRE);
    if(s1 == s2) return true;       // consistent snapshot
}
return false;
```

Writes are infrequent (startup + SIGHUP), reads are once per servo tick
(50 Hz on Core 3). Contention is essentially zero. The retry budget of
4 is a paranoia margin — in practice a write completes in ~1 µs and
readers almost never see an odd seq.

Adding a field: bump nothing visible (no `IPC_VERSION` increment) as
long as you only **add** at the end and don't rearrange existing
fields. The 256-byte `_reserved[]` tail in `IPC_RuntimeConfig` exists
for exactly this reason.

---

## 6. Core allocation

JSON parsing, schema validation, and IPC publishing all happen on
**Core 0** (in `cpcu_kernel`). The isolated cores never touch the JSON
parser. cpcu_io reads the IPC region on Core 3 (cheap memcpy + seqlock,
~50 ns). cpcu_dsp.py reads the same region on Cores 1-2 (Python
`mmap` + `struct.unpack`, also cheap).

This is the right split because JSON parsing takes hundreds of
microseconds — incompatible with cpcu_io's 2 µs poll budget. Doing
it once on Core 0, publishing the parsed result to shared memory, lets
RT cores enjoy structured config without paying the parse cost.

See [`CPCU_ARCHITECTURE.md`](CPCU_ARCHITECTURE.md) §3.3 for the full
core allocation table.

---

## 7. Testing

`config_testbench` (CPCU v2.3.3, new) automates the JSON parser:

| Group | What |
|---|---|
| TB-CFG01 | Valid full file parses every field correctly |
| TB-CFG02 | `CFG_Defaults` produces sane state |
| TB-CFG03 | Missing file returns `CFG_ERR_OPEN`, doesn't crash |
| TB-CFG04 | Wrong `schema_version` returns `CFG_ERR_SCHEMA` |
| TB-CFG05 | Out-of-range value returns `CFG_ERR_RANGE` |
| TB-CFG06 | min >= max sanity check fires per channel |
| TB-CFG07 | Optional fields honoured when present |
| TB-CFG08 | Optional fields default when absent |

```bash
build/config_testbench       # 30 PASS, 0 FAIL
./run_tests.sh 1             # 168 PASS total in Phase 1
```

The IPC seqlock side is exercised indirectly by the live system —
cpcu_io reads cfg_cache every tick during normal operation. There's no
dedicated seqlock unit test; the pattern is well-established and the
implementation matches the existing motor-cmd seqlock.

---

## 8. Operating procedure

### Day 1: bring up

```bash
# Clone + setup once:
./setup_pi.sh                          # creates /opt/cpcu/config.json symlink

# First run:
cmake -S . -B build && cmake --build build
sudo ./scripts/launch.sh debug         # or `release` once stable
```

`cpcu_kernel` reads `/opt/cpcu/config.json → cpcu_v2/config/runtime.json`,
publishes to IPC, spawns cpcu_io and cpcu_dsp.py.

### Calibration session

```bash
# Edit values in the repo:
$EDITOR cpcu_v2/config/runtime.json

# Reload — no process restart:
kill -HUP $(pgrep -f cpcu_kernel)

# Watch the kernel log to confirm the reload:
tail -f /var/log/cpcu/log_KERN.csv
# -> "runtime config loaded from /opt/cpcu/config.json (schema=1, seq=4)"
```

The values take effect within ~20 ms (one cpcu_io tick).

### Production change to a safety threshold

```bash
# Stop the system first — safety thresholds shouldn't change live.
sudo systemctl stop cpcu

# Edit and rebuild:
cd cpcu_v2
./configure.sh --radio-timeout 1000     # one knob
./configure.sh                          # or interactive
cmake --build build
sudo cmake --install build              # if installed under /opt/cpcu/bin

# Restart:
sudo systemctl start cpcu
```

`git diff cpcu_v2/include/cpcu_safety.h` shows the change for review.

### Reverting

```bash
# Runtime: just git revert the JSON.
git checkout cpcu_v2/config/runtime.json
kill -HUP $(pgrep -f cpcu_kernel)

# Compile-time: configure.sh --reset
cd cpcu_v2
./configure.sh --reset
cmake --build build
```

---

## 9. See also

- [`CPCU_ARCHITECTURE.md`](CPCU_ARCHITECTURE.md) §3.3 — core allocation
  for the JSON parser (Core 0) and IPC readers (Cores 1-2-3).
- [`CPCU_CONFIGURATION.md`](CPCU_CONFIGURATION.md) — every compile-time
  `#define`, what it does, when to change it. The static-knob reference;
  this doc is the runtime-knob reference.
- [`JITTER_MITIGATION.md`](JITTER_MITIGATION.md) §6 — per-servo bias
  offsets are introduced there and consumed here.
- [`cpcu_v2/include/cpcu_config.h`](../include/cpcu_config.h) — loader API.
- [`cpcu_v2/src/cpcu_config.c`](../src/cpcu_config.c) — JSON parser.
- [`cpcu_v2/configure.sh`](../configure.sh) — compile-time editor script.
- [`cpcu_v2/config/runtime.json`](../config/runtime.json) — the file itself.
