# CPCU System Architecture — v3.4

**Author:** bugrASl  
**Board:** Raspberry Pi 5 (BCM2712, 4x Cortex-A76 @ 2.4 GHz, OC to 2.8 GHz)  
**Radio:** NRF24L01+ (2.4 GHz ISM, Enhanced ShockBurst, PRX role)  
**Transmitter:** BSAU — NUCLEO-L432KC (STM32L432KC, Cortex-M4 @ 80 MHz)  
**DSP/ML:** Python 3 (scipy + scikit-learn SVM)  
**Date:** April 2026

---

## 1. System Overview

CPCU (Central Processing & Control Unit) is the receiver, processor, and actuator controller for the InfiniTech prosthetic hand system. It receives 1000 wireless packets per second from the BSAU wearable EMG sensor, runs real-time DSP and ML inference on the 8-channel EMG data, and drives servo actuators for gesture reproduction.

### 1.1 Signal Chain

```
Electrode -> InAmp -> ADC (32x OS) -> DMA -> WL_Pack -> SPI -> NRF24L01+ (BSAU)
    |                                                              |
    | 2.4 GHz Enhanced ShockBurst, 2 Mbps, 32B payload, 1000 pkt/s |
    |                                                              v
NRF24L01+ (CPCU) -> SPI -> WL_Unpack -> SPSC Ring -> DSP/ML -> Smoother -> Servo
                                                         |           |
                          PCA9685 I2C <- SMOOTH_Update <- Motor Cmd <- Python cpcu_dsp.py
```

### 1.2 Platform Migration: STM32H755 -> Raspberry Pi 5

| Aspect | Old (STM32H755ZI-Q) | New (Raspberry Pi 5) |
|---|---|---|
| Processor | CM7 @ 480 MHz + CM4 @ 240 MHz | 4x Cortex-A76 @ 2.8 GHz (OC) |
| Memory | 64 KB SRAM4 shared | 8 GB LPDDR4X (shared via mmap) |
| IPC | HSEM + SRAM4 ring buffer | Lock-free SPSC ring in mmap'd shm |
| Radio | SPI via HAL on CM4 | SPI via spidev on Core 3 |
| Servo | TIM1/TIM8 PWM on CM4 | I2C PCA9685 PWM driver on Core 3 |
| Servo motion | Instant step | Slew-rate limited smoother (2000 us/s) |
| OS | Bare-metal (HAL) | Linux 6.x + isolcpus |
| DSP/ML | CM7 bare-metal threshold | Python: scipy + sklearn on Cores 1-2 |
| EMG channels | 6 -> 8 | 8 |
| Wireless packet | 32 B (v1 layout) | 32 B (v2 layout: 8 ch + metadata) |

Migration justification: 14.3x inference headroom, 8x ring buffer depth, Python ecosystem for ML, SSH debug access.

---

## 2. Hardware Configuration

### 2.1 GPIO Allocation

```
NRF24L01+ (SPI0):
  GPIO 8   SPI0_CE0  -> NRF_CSN      (active-low chip select)
  GPIO 11  SPI0_SCLK -> NRF_SCK      (SPI clock, 8 MHz)
  GPIO 10  SPI0_MOSI -> NRF_MOSI
  GPIO 9   SPI0_MISO -> NRF_MISO
  GPIO 25             -> NRF_CE       (chip enable, held HIGH in RX mode)
  GPIO 24             -> NRF_IRQ      (not used, busy-poll instead)

PCA9685 (I2C1):
  GPIO 2   SDA1      -> PCA_SDA      (I2C data, 400 kHz fast mode)
  GPIO 3   SCL1      -> PCA_SCL      (I2C clock)
```

### 2.2 Kernel Configuration

```
/boot/firmware/config.txt:
  dtparam=spi=on                     Enable SPI0
  dtparam=i2c_arm_baudrate=400000    I2C Fast Mode (400 kHz)
  arm_freq=2800                      Overclock (2.4 -> 2.8 GHz)
  core_freq=750                      Fixed core clock (stable SPI)
  dtoverlay=disable-bt               Free kernel interrupts
  dtparam=fan_temp0=10000            Force max fan speed
  dtparam=fan_temp0_speed=255

/boot/firmware/cmdline.txt (append to existing line):
  isolcpus=1,2,3 nohz_full=1,2,3 rcu_nocbs=1,2,3
```

### 2.3 Power Architecture

```
Logic domain:   5V / 5A USB-C (Pi 5 + NRF + PCA logic)
Servo domain:   6V / 3A separate supply (fused, 1000 uF decoupling)
Common ground:  REQUIRED between Pi GND and servo supply GND
Isolation:      Wireless link provides galvanic isolation between BSAU and CPCU
```

### 2.4 Servo Configuration (Empirical Limits)

Per-servo min/max derived from EE493 Arduino testbench (PCA9685 counts converted to microseconds via `pulse_us = counts * 20000 / 4096`):

```
CH  Name       Motor  Min us  Max us  Range   Resolution
0   Base       MG995   498    2500    2002 us  0.44 deg/step
1   Upper      MG995  1074    1953     879 us  1.00 deg/step
2   Last       MG995  1074    1953     879 us  1.00 deg/step
3   Joint-1    SG90   1001    2002    1001 us  0.88 deg/step
4   Joint-2    SG90   1001    2002    1001 us  0.88 deg/step
5   Gripper    SG90    976    1733     757 us  1.16 deg/step
```

Neutral position for all servos: 1500 us.

---

## 3. Core Allocation

```
Core 0:   Linux Kernel Core (CFS scheduler)
          |- cpcu_kernel: process supervisor, watchdog, telemetry
          |- cpcu_tui: multi-page ncurses dashboard (via SSH)
          |- pca_testbench: interactive servo calibration tool
          |- signal_testbench: end-to-end signal integrity tester
          |- safety_testbench: automated safety FSM harness (Phase 1)
          |- SSH, networking, filesystem I/O, logging
          |- RCU callbacks offloaded from isolated cores
          \- Does NOT participate in real-time pipeline

Core 1:   DSP / AI — SMP pair (isolated, tickless)
Core 2:   DSP / AI — SMP pair (isolated, tickless)
          |- cpcu_dsp.py: Python SVM inference (12-feat, RBF kernel)
          |- scipy bandpass (20-450 Hz) + notch (50 Hz)
          |- Feature extraction: MAV, RMS, WL, ZC, SSC, VAR, LOG_DET
          |- SCHED_FIFO priority 80, mlockall (via taskset)
          \- Ring buffer consumer -> Motor command producer

Core 3:   Real-time I/O Controller (isolated, tickless)
          |- cpcu_io: C process, SCHED_FIFO priority 90
          |- NRF SPI busy-poll receiver (2 us poll interval)
          |- WL_Unpack + sequence/link validation
          |- SPSC ring buffer producer
          |- Servo smoother (SMOOTH_Update at 50 Hz, slew-rate limited)
          |- PCA9685 I2C servo driver (50 Hz, writes smooth.current[])
          |- SAFETY_* system-wide safety monitor (7 fault sources)
          \- Heartbeat to shared memory for watchdog
```

---

## 4. Inter-Process Communication

### 4.1 Shared Memory Layout (/dev/shm/cpcu_ipc)

```
Offset   Size      Section
0        192 B     IPC_ControlBlock (3 cache lines: header | head | tail)
192      64 KB     IPC_SensorEntry[1024] (ring buffer, 64 B per entry)
65728    128 B     IPC_MotorCommand (SeqLock protected)
65856    128 B     IPC_Diagnostics (per-core atomic counters)
65984    256 B     IPC_DSPExport (Python -> TUI telemetry)
                   ─────────────────────────────────────
Total:   ~66 KB
```

### 4.2 Ring Buffer (SPSC)

```
Size:           1024 entries x 64 bytes = 64 KB
Buffering:      1024 ms at 1000 pkt/s (8x old STM32 design)
Producer:       Core 3 (cpcu_io) — sole writer of sensor_head
Consumer:       Cores 1-2 (cpcu_dsp.py) — sole writer of sensor_tail
Peek access:    cpcu_tui reads ring[(head-N) & MASK] without advancing tail
Sync:           C11 _Atomic with acquire/release ordering (LDAR/STLR on ARMv8)
False sharing:  head and tail on separate 64-byte cache lines
Overflow:       Producer overwrites oldest; consumer detects and skips
```

### 4.3 Motor Command (SeqLock)

```
Writer (cpcu_dsp.py):
  1. seq++ (even -> odd: write in progress)
  2. Write servo_us[6], gesture_id, confidence, timestamp
  3. seq++ (odd -> even: write complete)

Reader (cpcu_io):
  1. Read seq; if odd -> retry
  2. Read all fields
  3. Re-read seq; if changed -> retry
  4. Feed to SMOOTH_SetAllTargets() (not directly to PCA)
```

---

## 5. Servo Smoother (cpcu_smooth)

### 5.1 Purpose

Prevents mechanical shock from instantaneous servo position jumps. Without smoothing, a gesture change (e.g., REST at 1500 us to HAND_CLOSE at 900 us) causes a 600 us step in one 20 ms frame — the servo snaps violently.

### 5.2 Algorithm

Per-channel slew rate limiter running inside the 50 Hz servo update cycle in cpcu_io:

```
For each servo channel:
  max_step = max_speed_us_per_s * dt_seconds
  diff = target - current
  if |diff| <= max_step:
    current = target (settled)
  else:
    current += sign(diff) * max_step
```

### 5.3 Parameters

```
Default speed:  2000 us/s (full 2000 us range in 1.0 second)
Step at 50 Hz:  40 us per frame (20 ms * 2000 us/s)
600 us move:    15 frames = 300 ms (matches CDR latency budget)
Settle thresh:  2 us (within this = "settled")
Tracking:       Float internally (sub-step precision, no drift)
Per-channel:    Each servo can have its own speed
```

### 5.4 Safety Behavior

When SAFETY_CheckSystem() returns false, the smoother does NOT gradually ramp. Instead: `SMOOTH_Snap()` instantly sets all channels to 1500 us (neutral), then `PCA_SetAllNeutral()` writes the physical servos. Instant snap, not gradual, because a safety trigger means something is wrong and the arm must stop immediately.

### 5.5 Integration in cpcu_io

```
Servo update (50 Hz):
  if(safety OK):
    1. IPC_ReadMotorCmd()        -> raw DSP targets
    2. PCA_SafetyClamp()         -> clamp to mechanical limits
    3. SMOOTH_SetAllTargets()    -> feed to smoother
    4. SMOOTH_Update(dt)         -> advance positions
    5. PCA_SetServo(smooth.current[]) -> write smoothed values
  else:
    1. SMOOTH_Snap()             -> instant jump to neutral
    2. PCA_SetAllNeutral()       -> write 1500 us to all channels
```

---

## 6. Safety Monitor (7 Fault Sources)

### 6.1 Radio State Machine

```
          boot
           |
           v
      [RADIO_INIT] ---first packet---> [RADIO_RUNNING]
                                            |
                                       750ms silence
                                            |
                                            v
      [RADIO_RUNNING] <--10 OK pkts-- [RADIO_RECOVERING] <--pkt-- [RADIO_DEGRADED]
                                                                       |
                                                                  1500ms total
                                                                       |
                                                                       v
                                                                 [RADIO_SAFE]
                                                                 (terminal)
```

### 6.2 All Fault Sources

| Source | Detection | Threshold | Action | Recovery (v2.3) |
|---|---|---|---|---|
| Radio link | No packet received | 750 ms silence | DEGRADED → SAFE | 10 consecutive OK packets |
| Battery | BSAU reports CRITICAL | V_batt ≤ 2.7 V | Immediate SAFE | V_batt > 3.0 V (hysteresis) |
| DSP stall | No motor cmd from Python | 2000 ms | SMOOTH_Snap + neutral | First fresh motor cmd |
| I²C bus | PCA9685 write failures | 5 consecutive | SMOOTH_Snap + neutral | First successful write |
| Thermal | CPU temperature | > 82 °C | Immediate SAFE | T < 70 °C (hysteresis) |
| Ring overflow | Consumer dead / lap | > 100 overflows since baseline | SMOOTH_Snap + neutral | 5 s of no new overflows, baseline reset |
| NRF hardware | Init failure | SPI readback mismatch | Tracked in diagnostics | cpcu_io re-init (3 s interval) |

Any single failure forces servos to neutral via SMOOTH_Snap() (instant, non-gradual). All seven sources are now individually recoverable and converge back to RUNNING through `SAFETY_UpdateState`'s SAFE-exit path: once every flag has been clear for `SAFETY_SAFE_RECOVER_MS = 3000 ms`, the FSM transitions to RECOVERING (radio-induced cause) or directly to RUNNING (any other cause).

**v2.3 ring-overflow recovery.** Earlier versions tied `ring.faulted` to the *cumulative* atomic counter `io_ring_overflows`. Because that counter is monotonic, once the threshold tripped the FSM latched in SAFE forever, even after the producer/consumer rebalanced. The new logic applies the threshold to the *delta* since the last quiescent baseline, and clears the fault after 5 s of no new growth (then re-baselines). The public API of `SAFETY_FeedRingOverflow(ctx, count)` is unchanged — the timer is maintained internally via `clock_gettime(CLOCK_MONOTONIC)`.

**v2.3 SAFETY_UpdateState now wired.** v2.2 introduced `SAFETY_UpdateState()` to centralise the RUNNING ↔ SAFE transitions for non-radio faults (battery, thermal, dsp, i2c, ring), but `cpcu_io.c` was never updated to actually call it. Boolean flags updated correctly and `SAFETY_CheckSystem()` already gated servo writes on them, so the prosthesis was still mechanically safe; but the FSM `state` shown in the TUI never reflected SAFE for non-radio faults — it stayed `RUNNING` while servos were being parked at neutral, which was confusing during diagnosis. v2.3 wires the call into `cpcu_io`'s main loop step 5.

**v2.3 SAFETY_VBAT_DIVIDER fix.** The constant was set to 1.0 in v2.2 with a comment claiming the BSAU firmware would correct for the on-board 2:1 resistor divider, but that BSAU change never shipped. `bsau_adc.c::BSAU_ADC_GetBattery()` returns the raw post-divider 12-bit ADC count, and `bsau_app.c` passes it directly to `pkt.vbat_raw`. With `SAFETY_VBAT_DIVIDER = 1.0`, every healthy 4.0 V battery reported as 2.00 V on the CPCU side, latching `battery.critical = true`. Restored to 2.0 in v2.3.

---

## 7. DSP/ML Pipeline (Python)

### 7.1 Origin and team alignment

The DSP and ML half of the system was developed by the project's
DSP/AI team in four scripts which together define the trained model:

| Script | Role |
|---|---|
| `proccess.py`  | One-time clean-up of raw recordings. 20–450 Hz BP + 50 Hz notch at native Fs. Writes `datasets/2/*.csv` and `dynamic_noise_thresholds.json`. |
| `feature_ex.py`| Reads `datasets/2/`. **Re-filters** at 15–90 Hz + 50 Hz notch at Fs=200, computes 3 Hz envelope, slides 40-sample / 20-stride window, writes 12-feature CSV (`s1_RMS`, `s1_VAR`, …, `s3_ENV`). |
| `model.py`     | Loads the 12-feature CSV. StratifiedGroupKFold(5, seed=33). StandardScaler. SVM RBF C=10, gamma='scale', class\_weight='balanced', probability=True. Saves `hmi_svm_model_200hz.joblib` + `hmi_scaler_200hz.joblib`. |
| `predict.py`   | Live laptop test rig over serial COM10. Reads ASCII `s1,s2,s3` lines at 200 Hz. Filters at 20–450 Hz (auto-clamps to 20–95 Hz), 50 Hz notch, 3 Hz envelope; same 12-feature path; 3-of-5 hysteresis vote; 0.65 confidence threshold. |

`cpcu_dsp.py` is a faithful port of `predict.py`'s signal chain to the
Pi. It uses identical filter coefficients, identical feature
definitions, identical hysteresis logic. The differences are entirely
on the *transport* side: read from `/dev/shm/cpcu_ipc` (binary)
instead of serial (ASCII), 8 channels instead of 3, 2 kHz native rate
that decimates to 200 Hz instead of native 200 Hz.

### 7.2 Pipeline stages

For one 200 ms inference window:

```
ipc.pop_sensor_batch(200)                           // Drain ring at 50 Hz
   ↓ (n samples, 2 sub-samples each, 8 channels each)
buffers[buf_idx].append(raw[ch] - 2048)             // ADC midrail subtract,
                                                    // PA0/PA1/PA2 only

   ↓ Once 200 samples since last window AND 400 samples total accumulated:

w_hi = last 400 samples per active channel          // 200 ms @ 2 kHz
window_lo = scipy.signal.decimate(w_hi, 10,         // → 40 samples @ 200 Hz
                                  zero_phase=True)  // anti-alias + downsample
centered = window_lo - mean(window_lo)              // DC removal
bp = filtfilt(butter(4, [20/100, 95/100], 'band'),  // bandpass; the 450 Hz
              centered)                             // arg auto-clamps to 95 Hz
cleaned = filtfilt(iirnotch(50/100, q=30), bp)      // 50 Hz mains kill
env = filtfilt(butter(4, 3/100, 'low'),             // 3 Hz envelope on |x|
               abs(cleaned))

features = [rms, var, wl, env_mean]                 // 4 per channel
features_flat = concat(features for each of 3 channels)  // 12 floats

   ↓

Xs = scaler.transform(features_flat.reshape(1, -1))
probs = model.predict_proba(Xs)[0]
ai = argmax(probs); label = model.classes_[ai]; conf = probs[ai]

   ↓ Hysteresis (matches predict.py byte-for-byte):

if conf > 0.65 and label != current_state:
    consecutive_count += 1
    if consecutive_count >= 3:
        current_state = label
        consecutive_count = 0
else:
    consecutive_count = 0

   ↓

servo_us = GESTURE_SERVO_MAP.get(current_state, [1500] * 6)
ipc.write_dsp_export(channel_rms, gesture_name, class_confidence, ...)
ipc.write_motor_cmd(servo_us, last_active_class, conf_pct)
```

### 7.3 Classifier configuration

```
Algorithm:        SVM, RBF kernel
C:                10              (regularization parameter)
gamma:            'scale'         (= 1 / (n_features × X.var()))
class_weight:     'balanced'      (compensates "rest" oversampling)
probability:      True            (predict_proba available for hysteresis)
random_state:     42

Feature space:    12-d (3 sensors × {RMS, VAR, WL, ENV-mean})
Pre-scaling:      StandardScaler (mean/std fit on training set)
Train/test split: StratifiedGroupKFold(n_splits=5, seed=33), 1st split
                  → ensures all windows from a single 5 s recording
                    stay together, AND every class is represented in
                    train and test
```

### 7.4 Class set

The trained class set comes from whatever `label` values exist in the
team's labelled CSV files. From `predict.py`'s color_map we know at
least three are present: `rest`, `biceps_flex`, `hand_flex`.

`cpcu_dsp.py`'s `GESTURE_SERVO_MAP` has those three plus `hand_open`
as future-proofing. Mapping behaviour at inference time:

- Class IS in the map ⇒ the corresponding 6 servo µs values get sent
  to `cpcu_io` via the seqlocked motor command IPC entry.
- Class NOT in the map ⇒ falls back to all-neutral (`[1500]*6`).

To diagnose mismatches: at startup `try_load_model()` prints
`model.classes_`; cross-reference with `GESTURE_SERVO_MAP` keys. The
TUI's DSP/AI page (key 3) also shows live `class_confidence` for every
class the model knows about.

### 7.5 Training-vs-live filter discrepancy

The team's training pipeline filters at **15–90 Hz**; the team's live
test rig (`predict.py`) and `cpcu_dsp.py` both filter at **20–450 Hz**
which scipy auto-clamps to **20–95 Hz** at Fs=200. This is a known,
documented inconsistency in the team's pipeline that they have
empirically tolerated:

| Pipeline | Bandpass | Effective passband at Fs=200 |
|---|---|---|
| Training (`feature_ex.py`) | `butter(4, [15/nyq, 90/nyq], 'band')` | 15.0 – 89.8 Hz |
| Live test (`predict.py`) | `butter_bandpass(20, 450, FS)` | 20.1 – 94.9 Hz |
| Live Pi (`cpcu_dsp.py`) | identical to predict.py | 20.1 – 94.9 Hz |

On real EMG (dominant 30–80 Hz energy) the two filters produce
features within ~0.2 % RMS — the model generalises across the gap
fine. Where the discrepancy matters:

- **15–20 Hz motion artifacts** (electrode shift, jaw clench): training
  saw and learned to ignore them; live discards them upstream, so
  *live features are slightly cleaner* in this band than training was.
- **90–95 Hz spectral edge**: live keeps it, training discarded it.
  Minor contribution to RMS for typical EMG.

`cpcu_dsp.py` deliberately matches `predict.py` (the team's live
validation rig), not `feature_ex.py` (the training pipeline), because
if a discrepancy *does* matter, the team's empirical validation has
been done on the live chain. If you later retrain on data captured
through the CPCU pipeline rather than the team's UART rig, the gap
disappears entirely.

### 7.6 Channel mapping (electrode wiring contract)

The team trained on three sensors with these physical positions:

```
s1 = Forearm   (wrist flexor electrode)
s2 = Biceps
s3 = Triceps
```

`cpcu_dsp.py`'s `ACTIVE_CHANNELS = [0, 1, 2]` commits the BSAU-side
wiring to:

```
PA0 (BSAU) → s1 → Forearm
PA1 (BSAU) → s2 → Biceps
PA2 (BSAU) → s3 → Triceps
```

This is a hidden contract — wrong electrode-to-PA wiring won't crash
anything, the SVM just gets garbage in feature\[0..3\] (the model
thinks channel 0 is the forearm when it's actually e.g. the triceps).
If your physical wiring is different, edit `ACTIVE_CHANNELS`.

### 7.7 Noise-threshold calibration (informational)

The team's pipeline has *two contradictory* threshold formulas:

| Script | Formula | Storage | Consumer |
|---|---|---|---|
| `proccess.py`   | `mean(std_per_file) × 3` | `dynamic_noise_thresholds.json` | none |
| `feature_ex.py` | `percentile(rest_envelope, 95) × 1.5` | print-only | none (the consuming code is commented out) |

Neither set of thresholds reaches the trained SVM at inference time.
They're purely diagnostic — the model learns "rest" from the labelled
training data instead. `cpcu_dsp.py`'s `--calibrate N` mode reproduces
the `proccess.py` formula (3×std per channel, written to
`/opt/cpcu/models/noise_thresholds.json`) for the case when you later
retrain on CPCU-collected data and want a similar diagnostic. It is
not read by inference.

### 7.8 Graceful degradation

If `/opt/cpcu/models/{hmi_svm_model_200hz.joblib, hmi_scaler_200hz.joblib}`
exist, `try_load_model()` loads them and inference runs as above. If
either is missing or doesn't load (sklearn version skew, `n_features_in_`
mismatch, …), the function returns `(None, None)` and the main loop:

- Still drains the ring at 50 Hz so the SPSC ring never overflows
- Still extracts and publishes the 12 features so the TUI's DSP/AI
  page lights up with live RMS / WL bars
- Just *skips* the predict-and-decide step, leaving `current_state =
  "rest"` and servos at neutral
- Sets `inference_enabled = False` so the TUI shows "feature-only
  mode" instead of a stale gesture label

The safety FSM is unaffected either way — it gates on radio + battery
+ thermal + ring + I²C, not on inference success. So a missing
`.joblib` degrades the system to a calibration / smoke-test rig
without compromising safety.

### 7.9 Model deployment workflow

The trained `.joblib` artefacts are not in this repo. To deploy:

1. On the team's Windows machine, run `feature_ex.py` (produces
   `features_200hz_segmented.csv`) and then `model.py` (produces
   `hmi_svm_model_200hz.joblib` + `hmi_scaler_200hz.joblib`).
2. SCP both files to the Pi: `scp hmi_*.joblib pi@<ip>:/opt/cpcu/models/`
3. Restart `cpcu_kernel` (`sudo systemctl restart cpcu` or
   `./scripts/launch.sh stop && ./scripts/launch.sh tui`). The
   spawned `cpcu_dsp.py` will print `[DSP] model + scaler loaded
   (classes=[…])` on stdout, visible in the tmux KERNEL window.

`/opt/cpcu/models/` is owned by your user (set by `setup_pi.sh`), so
the `scp` doesn't need sudo on the Pi end.

---

## 8. TUI System (cpcu_tui v3.4)

### 8.1 Multi-Page Dashboard

The TUI is a single binary (`cpcu_tui`) with **7 switchable pages**. Press `1`/`2`/`3`/`4`/`5`/`6`/`7` to switch. All live-data pages are read-only (peek at shared memory, never consume ring entries); only Dataset (page 6) opens a file for writing while a capture is armed.

**v3.4 page order change:** the static CONFIG spec sheet was moved from page 5 to page 7. Live-data pages now occupy the first six tabs so the most-watched information is on the lowest-numbered keys, and the spec reference is parked at the end.

```
Page 1 — Overview:   Rolled-up HEALTH banner (six green/yellow/red pills +
                     overall NOMINAL/WARNING/DEGRADED verdict), system state,
                     radio link summary, EMG channel bars (% of 4095), 6 servo
                     sliders, battery pack voltage + level, DSP summary, ML
                     classification with softmax bars, filtered RMS per ch.

Page 2 — Radio/IO:   NRF24L01+ status (channel + GHz, address, SPI speed),
                     IO heartbeat age (proves RT loop alive), SAFE-entries
                     counter, battery voltage. Packet stats (total RX, rate,
                     dropped vs gaps distinction, max-poll µs, loss rate,
                     live ring-fill bar, last-packet retry count). Last
                     packet raw fields + decoded BSAU flags banner
                     (CLIP/ELEC/OVRN/TX_SAT/CAL/FIRST).

Page 3 — DSP/AI:     Pipeline stats (DSP windows processed + /s rate,
                     inferences + /s rate, max latency µs, ring fill,
                     underflows, export rate Hz, motor cmd count + /s
                     + age in ms). Active gesture banner with confidence,
                     last inference time µs, export-seq counter ticking.
                     Per-class softmax confidence bars (3-4 classes,
                     active one magenta). Per-channel filtered RMS (bar
                     = % of 0.5 V full-scale + absolute V).

Page 4 — Waveforms:  Live 8-channel line-trace waveforms (' ` - . ,
                     sub-row glyphs with / \ connectors, 5× vertical
                     sub-sampling). Per-channel Hz (from zero-crossing
                     rate), Vpp, Vrms, red CLIP indicator when ADC hits
                     rails. BSAU-flags banner + glyph legend at top.
                     UP/DOWN selects channel, TAB switches to zoomed
                     single-channel detail (adds DC offset + time-axis
                     scale). Peek-based — safe alongside cpcu_dsp.py.

Page 5 — Health:     Traffic-light rollup. 10 subsystem rows (Safety FSM,
                     Radio, IO loop, IPC ring, Pkt integrity, Battery,
                     DSP pipeline, ML export, BSAU sensor, SAFE trips),
                     each [OK]/[WARN]/[FAULT] with a one-line "why"
                     explanation. Top banner tallies N OK | N WARN |
                     N FAULT, shows overall verdict. Put this on a
                     second monitor during hardware testing.

Page 6 — Dataset:    Interactive 8-channel CSV capture. LEFT/RIGHT cycles
                     the gesture label (REST, H.SLO, H.HRD, H.OPN, A.BND<,
                     A.BND=, A.BND>, A.SLO, A.FST, BICEP), s/SPACE
                     starts/stops, r cancels and deletes the partial
                     file, t toggles RAW ADC ↔ FILTERED output. The
                     capture state machine drains the ring every tick
                     regardless of which page is currently rendered, so
                     flipping pages mid-capture does not lose samples.

Page 7 — Config:     Static compile-time + hardware spec reference. Four
                     sections: BSAU/CPCU topology, wireless+IPC layout,
                     motor+ML pipeline, build info (TUI version, compiler,
                     C standard, build date/time). Nothing updates at
                     runtime; this is the "what system am I looking at"
                     page, parked at the end of the tab order in v3.4.
```

### 8.2 Hotkey Reference

**Universal (all pages):**

| Key        | Action                                                |
|------------|-------------------------------------------------------|
| `1`..`7`   | Switch page                                           |
| `q` `Q`    | Quit                                                  |
| `UP`/`DN`  | Select channel (Page 4 only)                          |
| `TAB`      | Toggle grid ↔ single-channel detail (Page 4 only)     |
| `← / →`    | Cycle gesture label (Page 6 only, when not capturing) |
| `s` `SPACE`| Start / stop capture (Page 6 only)                    |
| `t` `T`    | Toggle RAW ↔ FILTERED capture (Page 6 only, idle)     |
| `r`        | Cancel + delete in-progress capture (Page 6 only)     |

**Demo-mode-only** (`cpcu_tui --demo`):

| Key     | Action                                                  |
|---------|---------------------------------------------------------|
| `w` `W` | Cycle waveform: SINE → SQUARE → TRI → SAW → NOISE → EMG → ECG → CHIRP |
| `[`     | Halve frequency (floor 10 Hz)                           |
| `]`     | Double frequency (ceiling 1000 Hz)                      |
| `F`     | Inject radio freeze (triggers DEGRADED→SAFE in 2.25 s)  |
| `B`     | Inject low battery (triggers SAFE on `VBAT_CRITICAL`)   |
| `G`     | Inject sequence-gap storm                               |
| `O`     | Inject ring overflow (auto-clears once burst ends, v2.3)|
| `I`     | Inject I²C error streak                                 |
| `R`     | Master reset: clears all faults AND zeros every counter |

The `R` master reset is implemented by `demo_full_reset()` (v3.4 helper) which clears the injected-fault mask and zeros every cumulative IPC diag counter (packets, gaps, overflows, SAFE entries, inferences, latency, underflows, drops). On Dataset page mid-capture `r` instead cancels and deletes the partial file, since the master reset would be destructive there.

### 8.3 Demo Mode

All TUIs support `--demo` for operation without hardware or shared memory. Demo mode feeds the ring buffer with 100 synthetic sensor packets per frame, exercising the full codec → ring → atomics → seqlock path — so it's a legitimate smoke test, not a stub.

Eight selectable waveforms are generated by a shared header `demo_signals.h` used by both `cpcu_tui` and `signal_testbench`:

```
SINE       pure tone
SQUARE     50 % duty
TRIANGLE   symmetric
SAWTOOTH   rising ramp
NOISE      uniform white centered on 1.65 V
EMG_BURST  1 s rest + 1 s contraction (realistic prosthetic-control signal)
ECG        PQRST template with R-spikes at freq BPM
CHIRP      frequency sweep f → 5f over 2 s, repeating
```

```bash
./cpcu_tui --demo           # Full 7-page TUI with synthetic data
./signal_testbench --demo   # Signal analysis with selectable waveform
./pca_testbench             # Has built-in dry-run if no I2C
./safety_testbench          # Automated safety-FSM harness (33 checks, v2.3)
```

### 8.4 Waveform Renderer (Page 4)

Earlier versions used Unicode block glyphs (`▁▂▃▄▅▆▇█`) which broke on SSH clients without full Unicode support. v3.2 uses an ASCII-only **line-trace renderer**:

- Each column gets a single glyph chosen from `'` `` ` `` `-` `.` `,` by sub-row position (5 sub-cells per row)
- Vertical gaps between adjacent samples are filled with `/` or `\` connectors
- Result: 5× more effective vertical resolution than the row count suggests, and renders correctly on every terminal

The glyph legend is shown in the top banner of Page 4 so readers immediately know what they're looking at.

### 8.5 Per-Module CSV Logging

When launched with `--log` (which `launch.sh` passes automatically), every `LOG_I/W/E/D` call also appends a CSV row to `/var/log/cpcu/log_<module>.csv`:

```
log_kern.csv    supervisor events
log_wdg.csv     watchdog events
log_io.csv      cpcu_io RT-loop events
log_nrf.csv     radio init, retry bursts
log_pca.csv     I²C writes, init reads, failure streaks
```

Format: `timestamp_s,timestamp_us,proc,level,"message"`. Files are `fflush`ed after every write so `Ctrl+C` never loses buffered lines. Directory is created by `launch.sh` with mode 755.

### 8.6 Monitoring via SSH (tmux)

```bash
ssh pi@<pi-ip>
tmux new -s cpcu

# Suggested 4-pane layout:
#   Pane 0: journalctl -u cpcu -f
#   Pane 1: cpcu_tui (press 1-7, use w/[/] in --demo)
#   Pane 2: watch -n 2 "vcgencmd measure_temp; ps -eo pid,comm,psr,pri | grep cpcu"
#   Pane 3: /opt/cpcu/bin/pca_testbench

tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

# Detach: Ctrl+b d     Reattach: tmux attach -t cpcu
```

During hardware testing, put `cpcu_tui` Page 5 (HEALTH) in pane 1 — it's the fastest way to catch regressions live.

---

## 9. Process Model

```
Process 0: cpcu_kernel (Core 0)
  |- Creates shared memory (IPC_Create)
  |- Spawns cpcu_io via spawn_native("taskset -c 3 chrt -f 90 ./cpcu_io")
  |- Spawns cpcu_dsp.py via spawn_python("taskset -c 1,2 chrt -f 80 python3 cpcu_dsp.py")
  |- Monitors heartbeats (2s timeout -> SIGKILL + respawn)
  |- Pets /dev/watchdog every 5s (15s hardware timeout)
  \- Prints telemetry every 5s

Process 1: cpcu_dsp.py (Cores 1-2, Python)
  |- Opens shared memory (IPCBridge)
  |- Loads SVM model + scaler from .joblib
  |- Ring buffer consumer
  |- scipy DSP pipeline
  |- Motor command producer (SeqLock)
  \- Writes DSP export for TUI

Process 2: cpcu_io (Core 3, C)
  |- Opens shared memory (IPC_Open)
  |- NRF SPI driver (8 MHz busy-poll)
  |- Servo smoother (SMOOTH_Update at 50 Hz)
  |- PCA9685 I2C driver (writes smooth.current[])
  |- SAFETY_* system-wide safety monitor
  |- Ring buffer producer
  \- System state machine
```

---

## 10. Software File Map

```
File                     Layer  Core     What It Does
wireless_packet.h/c      0      any     Byte <-> struct codec (shared with BSAU)
nrf24l01_linux.h/c       1      3       NRF SPI driver (spidev + gpiod)
cpcu_pca9685.h/c         1      3       PCA9685 I2C servo PWM driver
cpcu_smooth.h/c          1      3       Per-channel servo slew rate limiter
cpcu_ipc.h/c             2      all     POSIX shared memory IPC (SPSC + SeqLock)
cpcu_safety.h/c          3      3       System-wide safety monitor (7 sources)
cpcu_log.h/c             3      all     Structured colored logging + per-module CSV
cpcu_io.c                4      3       Real-time I/O main loop + smoother
cpcu_dsp.py              4      1-2     Python DSP + ML pipeline
cpcu_ipc_bridge.py       4      1-2     Python shared memory bridge
cpcu_kernel.c            4      0       Process supervisor
cpcu_tui.c               5      0/SSH   Multi-page ncurses dashboard (6 pages)
demo_signals.h           5      any     Shared 8-waveform generator (cpcu_tui +
                                        signal_testbench, demo mode)
pca_testbench.c          test   0/SSH   Interactive servo calibration TUI
signal_testbench.c       test   0/SSH   End-to-end signal integrity TUI
safety_testbench.c       test   any     Automated safety-FSM harness
                                        (33 checks across 7 test groups, v2.3)
test_codec.c             test   any     Codec round-trip tests
test_ipc_bridge.py       test   any     IPC offset validation
test_dsp_pipeline.py     test   any     DSP filter + feature + model tests
```

---

## 11. Build System (CMake)

### 11.1 How CMake Works

CMake is a build system generator. It reads `CMakeLists.txt` and produces Makefiles. The workflow is: `cmake ..` (configure, detect compilers and libraries) then `make` (build). Out-of-source builds keep the source tree clean.

### 11.2 Our CMakeLists.txt Structure

```
Static libraries (shared code compiled once, linked into multiple targets):
  cpcu_codec    — wireless_packet.c (used by cpcu_io, test_codec)
  cpcu_ipc      — cpcu_ipc.c + librt (used by cpcu_io, cpcu_tui, signal_testbench)
  cpcu_log      — cpcu_log.c (used by cpcu_io, cpcu_kernel)

Executables:
  cpcu_io       — Core 3 RT loop (codec + ipc + log + nrf + pca + safety + smooth)
  cpcu_kernel   — Core 0 supervisor (ipc + log)
  cpcu_tui      — Dashboard (codec + ipc + ncurses) [guarded by CURSES_FOUND]
  pca_testbench — Servo test (pca9685 + ncurses) [guarded by CURSES_FOUND]
  signal_testbench — Signal test (codec + ipc + ncurses) [guarded by CURSES_FOUND]
  safety_testbench — Automated safety FSM harness (safety + ipc, Phase 1)
  test_codec    — Unit tests (codec only, no hardware)

Install targets:
  /opt/cpcu/bin/          — cpcu_io, cpcu_kernel, cpcu_tui
  /opt/cpcu/scripts/      — cpcu_dsp.py, cpcu_ipc_bridge.py, launch.sh
  /opt/cpcu/test/         — test_ipc_bridge.py, test_dsp_pipeline.py
  /opt/cpcu/models/       — emg_rf_model.pkl (empty dir, user copies model)
  /etc/systemd/system/    — cpcu.service
```

### 11.3 Building Individual Targets

```bash
cd build
cmake --build . --target cpcu_io            # Just the RT loop
cmake --build . --target pca_testbench      # Just the servo testbench
cmake --build . --target signal_testbench   # Just the signal testbench
cmake --build . --target safety_testbench   # Just the safety harness
cmake --build . --target test_codec         # Just the unit tests
```

---

## 12. Systemd Service (cpcu.service)

### 12.1 What It Does

The systemd unit file manages the CPCU as a Linux service. The boot chain is: `power on -> systemd -> cpcu.service -> launch.sh -> cpcu_kernel -> fork(cpcu_io) + fork(cpcu_dsp.py)`.

### 12.2 Key Configuration

```
Type=simple         — systemd tracks the PID of launch.sh (which exec's into cpcu_kernel)
ExecStart           — /opt/cpcu/scripts/launch.sh (pre-flight checks then exec)
Restart=on-failure  — auto-restart on crash, 5s delay, max 5 attempts per minute
LimitRTPRIO=90      — allows SCHED_FIFO up to priority 90 (cpcu_io needs this)
LimitMEMLOCK=infinity — allows mlockall() (prevent page faults on RT cores)
KillMode=control-group — SIGTERM goes to ALL processes (kernel + io + dsp), not just PID 1
WatchdogSec=30      — systemd kills service if unresponsive for 30s (second safety layer)
```

### 12.3 launch.sh Pre-Flight Checks

Before starting cpcu_kernel, launch.sh validates: binaries exist, Python dependencies available, DSP script present, ML model file present, core isolation active, SPI device present, I2C device present. Warnings are logged but non-fatal (system can run without DSP, for example).

---

## 13. Shell Scripts

### 13.1 setup_pi.sh — One-Time Pi Configuration

Run once on a fresh Raspbian install. Installs apt packages (build-essential, cmake, libncurses-dev, i2c-tools, python3-numpy, python3-scipy), pip packages (joblib, scikit-learn), creates /opt/cpcu directory structure, configures SPI/I2C/core isolation in boot config files, sets permissions for I2C/SPI groups.

### 13.2 launch.sh — Boot Script

Called by systemd at boot. Does pre-flight checks then `exec taskset -c 0 ./cpcu_kernel` which replaces the shell process with the kernel supervisor. The `exec` is important: systemd tracks the cpcu_kernel PID directly, so signals (SIGTERM on `systemctl stop`) go to the right process.

### 13.3 run_tests.sh — Test Runner

Supports both automated phases and interactive testbenches:

```bash
./run_tests.sh              # All automated phases (1 2 3)
./run_tests.sh 1            # Phase 1 only (software, no hardware)
./run_tests.sh 1 2          # Phases 1 and 2
./run_tests.sh pca          # Launch PCA9685 servo testbench TUI
./run_tests.sh signal       # Launch signal integrity testbench (live data)
./run_tests.sh signal-demo  # Launch signal testbench (synthetic, no hardware)
```

Interactive phases (`pca`, `signal`, `signal-demo`) use `exec` to replace the shell with the TUI binary, so quitting the TUI returns you to your terminal cleanly.

---

## 14. Performance Summary

| Metric | Value | Margin |
|---|---|---|
| Packet throughput | 1000 pkt/s | NRF air: 16.5% utilized |
| Core 3 utilization | 4.6% | 95.4% idle |
| Core 1-2 utilization (Python) | ~30% peak | 70% headroom |
| Ring buffer depth | 1024 ms | 8x old design |
| ADC-to-servo latency (typical) | ~26 ms | 11.5x below 300 ms target |
| Servo smoother latency | 300 ms for full range | Matches CDR budget |
| NRF poll latency | 2 us | 5-25x faster than IRQ |
| Servo resolution | 0.44 deg (Base) | Below mechanical backlash |

---

## 15. Design Risks and Mitigations

| Risk | Severity | Mitigation |
|---|---|---|
| Linux is not a real-time OS | Medium | isolcpus + nohz_full + rcu_nocbs + SCHED_FIFO + mlockall |
| Python GC pauses | Low | gc.disable(). Ring buffer 1024 ms deep absorbs any pause |
| filtfilt vs sosfilt mismatch | Medium | test_dsp_pipeline.py validates. Retrain if accuracy drops > 5% |
| I2C bus stall blocks radio poll | Low | PCA clock-stretch max 100 us. NRF FIFO depth 3 absorbs 1 missed poll |
| sklearn version mismatch | Medium | Pin versions. ONNX export as fallback |
| Servo stall current exceeds PSU | Medium | 3A PSU may be insufficient for 6-servo stall. Add 1000 uF cap or upgrade to 6A |
| Thermal throttle at 85C | Low | SAFETY_FeedTemperature forces SAFE at 82C |
| Servo snap on gesture change | Eliminated | cpcu_smooth slew limiter: max 40 us/frame at 50 Hz |
