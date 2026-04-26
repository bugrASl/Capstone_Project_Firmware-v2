# CPCU Configuration Reference — v2.3

**Author:** bugrASl
**Date:** April 2026

Single reference for every configuration option in the CPCU + BSAU
system. Organised by file. Each entry has a default value, what it
does, and how to change it.

---

## 0. How configuration is layered

Configuration lives in five places, in order of how invasive a change is:

1. **Compile-time C constants** in headers (`cpcu_safety.h`,
   `cpcu_smooth.h`, `cpcu_pca9685.h`, `bsau_config.h`). Editing requires
   `cmake --build` and (for BSAU) re-flash. These are the things that
   define the *physics* of the system — sample rate, voltages, mechanical
   limits.

2. **Compile-time Python constants** at the top of `cpcu_dsp.py`. Editing
   requires `sudo cp` to `/opt/cpcu/scripts/` and a kernel restart.
   Active channels, ML thresholds, gesture mappings.

3. **Runtime defines via `cpcu_io.c`** — set during `main()` after
   `SMOOTH_Init`. Per-servo enable, accel, velocity. Editing requires
   recompile of cpcu_io specifically.

4. **Operator commands via the TUI** — runtime, no recompile. Kill,
   resume, clear-counters. Set by writing bits to
   `ipc.ctrl->operator_cmd`.

5. **System-level** — kernel cmdline, raspi-config, systemd unit. Once
   set, persists across reboots.

---

## 1. cpcu_safety.h — safety thresholds

**Path:** `include/cpcu_safety.h` in the CPCU source tree (or wherever
your build keeps headers).

### Radio link

| Constant | Default | Meaning |
|---|---|---|
| `SAFETY_RADIO_TIMEOUT_MS` | 750 | Silence (no packet) after which RUNNING → DEGRADED |
| `SAFETY_RADIO_SAFE_MS` | 1500 | DEGRADED-state duration before SAFE (terminal until recovery) |
| `RECOVERY_PKT_COUNT` | 10 | Consecutive good packets needed to leave RECOVERING |

Lower these for faster fault detection at the cost of false alarms during
brief radio glitches. Don't drop `SAFETY_RADIO_TIMEOUT_MS` below ~150 ms
or normal RF retries will trigger it.

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

## 2. cpcu_smooth.h — servo motion profile

**Path:** `include/cpcu_smooth.h`.

| Constant | Default | Meaning |
|---|---|---|
| `SMOOTH_DEFAULT_VELOCITY` | 2000 | µs/s — full 2000 µs span in 1.0 s if cruising |
| `SMOOTH_DEFAULT_ACCEL` | 8000 | µs/s² — reach max velocity in 250 ms |
| `SMOOTH_SETTLE_THRESH` | 2 | Within this many µs of target = settled |

These are the *defaults*, applied to all 6 channels at `SMOOTH_Init`.
Per-servo overrides happen in `cpcu_io.c` (see §5). Lower
`SMOOTH_DEFAULT_VELOCITY` if your servos mechanically bind at peak
speed. Lower `SMOOTH_DEFAULT_ACCEL` if you hear/see jolts at the start
of motion.

---

## 3. cpcu_pca9685.h — servo mechanical limits

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

## 4. cpcu_dsp.py — DSP/ML pipeline

**Path:** `/opt/cpcu/scripts/cpcu_dsp.py` (after install) or
`scripts/cpcu_dsp.py` (in source).

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
expected format), you can retrain locally on the Pi:

```bash
# 1. Symlink the team's scripts into /opt/cpcu so they can find datasets/
ln -s /home/<you>/team_scripts/feature_ex.py /opt/cpcu/scripts/
ln -s /home/<you>/team_scripts/model.py      /opt/cpcu/scripts/

# 2. Edit the DATA_FOLDER constant in feature_ex.py to point at
#    cpcu_v2/datasets/  (or wherever you saved the captures).
#    Skip proccess.py — captures are already filtered by cpcu_io.
#    OR leave proccess.py in the chain if you want the training-side
#    cascaded-filter behaviour (see CPCU_ARCHITECTURE.md §7.5).

# 3. Run them (they need pandas, scikit-learn, joblib — already
#    installed by setup_pi.sh):
cd /opt/cpcu/scripts
python3 feature_ex.py            # produces features_200hz_segmented.csv
python3 model.py                 # produces hmi_*_200hz.joblib

# 4. Move the new joblibs into place:
mv hmi_*.joblib /opt/cpcu/models/

# 5. Restart so cpcu_dsp.py picks up the new model:
./scripts/launch.sh stop
./scripts/launch.sh tui
```

If the new model has classes that aren't in `GESTURE_SERVO_MAP`, edit
the map first and rebuild the launcher's pyc cache (just re-source
the file is enough; cpcu_dsp.py imports it fresh on each startup).

---

## 5. cpcu_io.c — runtime per-servo config

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

## 6. bsau_config.h — BSAU mode selection

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

## 7. Operator commands — runtime, no recompile

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

## 8. launch.sh — runtime mode selection

Five modes, each launches a different combination:

| Mode | Kernel | Foreground tool |
|---|---|---|
| `kernel` | foreground | none (systemd-style) |
| `tui` | background | cpcu_tui dashboard |
| `collect` | background | cpcu_tui (with capture-workflow reminder) |
| `signal` | background | signal_testbench |
| `pca` | not started | pca_testbench |

```bash
sudo /opt/cpcu/scripts/launch.sh           # interactive menu (TTY)
sudo /opt/cpcu/scripts/launch.sh kernel    # production / systemd
sudo /opt/cpcu/scripts/launch.sh tui
sudo /opt/cpcu/scripts/launch.sh signal
sudo /opt/cpcu/scripts/launch.sh pca
```

See `launch.sh` header for the full documentation block.

---

## 9. Pi system configuration

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

## 10. Common configuration recipes

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

## 11. What's NOT configurable (and why)

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
