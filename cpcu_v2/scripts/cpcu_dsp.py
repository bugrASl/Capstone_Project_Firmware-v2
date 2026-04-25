#!/usr/bin/env python3
"""
cpcu_dsp.py -- Live DSP + ML pipeline for the CPCU.

Mirrors the offline Python pipeline developed by the DSP/AI team
(proccess.py + feature_ex.py + model.py + predict.py), but reads from
shared memory (/dev/shm/cpcu_ipc) instead of CSV files and writes
inference results back through IPC for cpcu_tui and cpcu_io to consume.

────────────────────────────────────────────────────────────────────────
HOW THE TEAM'S CSV PIPELINE MAPS TO LIVE STREAMING
────────────────────────────────────────────────────────────────────────

  Team (offline, Windows)           ↔   CPCU (live, Pi)
  ──────────────────────────────────────────────────────────────────
  pd.read_csv("rest_3.csv")         ↔   ipc.pop_sensor_batch()
  df['s1'].values                   ↔   buffers[ACTIVE_CHANNELS[0]]
  40-sample window @ 200 Hz         ↔   400 samples @ 2 kHz,
                                         decimated to 40 @ 200 Hz
  apply_filters → extract_features  ↔   identical (same coefficients)
  model.predict_proba(...)          ↔   identical
  CSV row appended                  ↔   ipc.write_dsp_export(...) +
                                         ipc.write_motor_cmd(...)

────────────────────────────────────────────────────────────────────────
NOTE ON THE NOISE-THRESHOLD CALIBRATION
────────────────────────────────────────────────────────────────────────

proccess.py computes per-sensor noise thresholds (3 * std of rest-state
signal) and saves them to dynamic_noise_thresholds.json. predict.py
never reads that JSON: the trained SVM operates purely on the 12
numeric features. So *for live inference*, no calibration data is
required. We provide --calibrate here for when you later retrain on
CPCU-collected data.

────────────────────────────────────────────────────────────────────────
CHANNEL AND SAMPLE-RATE MISMATCH
────────────────────────────────────────────────────────────────────────

Team trained on 3 sensors (Forearm/Biceps/Triceps) at Fs = 200 Hz.
CPCU has 8 channels at Fs = 2 kHz.

  - ACTIVE_CHANNELS picks which 3 of 8 BSAU channels feed the model.
    Edit this when you know which BSAU inputs your electrodes are on.
  - Each 200 ms window is decimated 10x before the filter chain runs.
    The 12 features that come out are byte-comparable to what the
    team's model was trained on.

If /opt/cpcu/models/{hmi_svm_model_200hz.joblib, hmi_scaler_200hz.joblib}
exist, we run inference. If not, we still drain the ring and publish
features so cpcu_tui Page 3 lights up -- just no gesture classification.

────────────────────────────────────────────────────────────────────────
USAGE
────────────────────────────────────────────────────────────────────────

  Normal run (spawned by cpcu_kernel via launch.sh):
    python3 cpcu_dsp.py

  Standalone debug (cpcu_kernel must already be running):
    python3 cpcu_dsp.py --verbose

  Calibrate noise thresholds from N seconds of "rest" capture:
    python3 cpcu_dsp.py --calibrate 10
    # writes /opt/cpcu/models/noise_thresholds.json
"""

import argparse
import json
import os
import signal
import sys
import time
from collections import deque

import numpy as np
from scipy.signal import butter, filtfilt, iirnotch, decimate

from cpcu_ipc_bridge import IPCBridge


# ══════════════════════════════════════════════════════════════════════
#  CONFIGURATION  ── edit these for your setup
# ══════════════════════════════════════════════════════════════════════

# Which 3 of 8 BSAU channels feed the model. Order matters: position 0
# is the team's "s1" (forearm), 1 is "s2" (biceps), 2 is "s3" (triceps).
# Re-map when you know which BSAU inputs your electrodes are wired to.
ACTIVE_CHANNELS         =   [0, 1, 2]
CHANNEL_LABELS          =   ['s1', 's2', 's3']
NUM_ACTIVE_CH           =   len(ACTIVE_CHANNELS)

# Sampling rates
INPUT_FS_HZ             =   2000        # CPCU native rate (2 samples/pkt @ 1 kpkt/s)
TARGET_FS_HZ            =   200         # Team's training rate
DECIMATE_FACTOR         =   INPUT_FS_HZ // TARGET_FS_HZ     # 10

# Window geometry (matches feature_ex.py)
WINDOW_MS               =   200
STRIDE_MS               =   100         # 50% overlap

WINDOW_SAMPLES_HI       =   INPUT_FS_HZ  * WINDOW_MS // 1000     # 400
STRIDE_SAMPLES_HI       =   INPUT_FS_HZ  * STRIDE_MS // 1000     # 200
WINDOW_SAMPLES_LO       =   TARGET_FS_HZ * WINDOW_MS // 1000     # 40

# Buffer headroom: keep enough history that decimate's edge effects
# stay outside the analysis window. 4 windows worth is plenty.
BUFFER_SAMPLES          =   WINDOW_SAMPLES_HI * 4                # 1600

# BSAU samples are uint16 with mid-rail ~2048 (1.65 V at 3.3 V reference).
# Center to int by subtracting mid-rail, matching predict.py line 117.
ADC_MIDRAIL             =   2048

# Model + scaler (set probability=True at training time per model.py)
MODEL_DIR               =   "/opt/cpcu/models"
MODEL_PATH              =   os.path.join(MODEL_DIR, "hmi_svm_model_200hz.joblib")
SCALER_PATH             =   os.path.join(MODEL_DIR, "hmi_scaler_200hz.joblib")
THRESHOLDS_PATH         =   os.path.join(MODEL_DIR, "noise_thresholds.json")

# Debounce (matches predict.py)
PROBABILITY_THRESHOLD   =   0.65
CONFIRMATION_THRESHOLD  =   3

# Servo defaults
SERVO_NEUTRAL_US        =   1500
NUM_SERVOS              =   6

# Drain rate (independent of inference rate so the ring never backs up)
DRAIN_PERIOD_S          =   0.020       # 50 Hz
DRAIN_BATCH             =   200

# Default servo poses per gesture. Edit per your hardware. Unrecognized
# labels fall back to all-neutral. 6 servos, in microseconds.
GESTURE_SERVO_MAP       =   {
    "rest":         [1500, 1500, 1500, 1500, 1500, 1500],
    "biceps_flex":  [1500, 1700, 1500, 1500, 1500, 1500],
    "hand_flex":    [1700, 1500, 1500, 1700, 1700, 1700],
    "hand_open":    [1300, 1500, 1500, 1300, 1300, 1300],
}


# ══════════════════════════════════════════════════════════════════════
#  FILTER PRIMITIVES  ── direct ports of the team's functions
# ══════════════════════════════════════════════════════════════════════

def butter_bandpass(data, low, high, fs, order=4):
    """Direct port of predict.py:butter_bandpass_filter. Auto-caps high
    cut to <Nyquist if it would otherwise blow up at low Fs (a safety
    net carried over from the team's code)."""
    nyq                 =   0.5 * fs
    if high >= nyq:
        high            =   nyq * 0.95
    b, a                =   butter(order, [low/nyq, high/nyq], btype='band')
    return filtfilt(b, a, data)


def notch_filter(data, f0, fs, q=30.0):
    """Direct port of predict.py:notch_filter."""
    nyq                 =   0.5 * fs
    w0                  =   f0 / nyq
    if w0 >= 1:
        return data
    b, a                =   iirnotch(w0, q)
    return filtfilt(b, a, data)


def envelope(data, fs, cutoff=3.0):
    """Direct port of predict.py:lowpass_envelope_filter on |x|."""
    nyq                 =   0.5 * fs
    b, a                =   butter(4, cutoff/nyq, btype='low')
    return filtfilt(b, a, np.abs(data))


def extract_features(clean, env):
    """Direct port of feature_ex.py:extract_features. 4 features per
    channel: RMS, VAR, WL, ENV-mean."""
    eps                 =   1e-8
    rms                 =   float(np.sqrt(np.mean(clean ** 2) + eps))
    var                 =   float(np.var(clean))
    wl                  =   float(np.sum(np.abs(np.diff(clean))) / len(clean))
    em                  =   float(np.mean(env))
    return [rms, var, wl, em]


def process_window(window_hi):
    """One full pipeline pass on a 400-sample @ 2 kHz window.
    Decimates → DC-removes → BP → notch → envelope → 4 features."""
    # Anti-alias + downsample to 200 Hz (40 samples)
    window_lo           =   decimate(window_hi, DECIMATE_FACTOR, zero_phase=True)
    # DC offset removal (matches predict.py: data - np.mean)
    centered            =   window_lo - np.mean(window_lo)
    # Bandpass + notch (the 450 Hz cap auto-clamps to ~95 Hz at Fs=200)
    bp                  =   butter_bandpass(centered, 20.0, 450.0, TARGET_FS_HZ)
    cleaned             =   notch_filter(bp, 50.0, TARGET_FS_HZ)
    # Envelope on |cleaned|
    env                 =   envelope(cleaned, TARGET_FS_HZ, cutoff=3.0)
    return cleaned, env, extract_features(cleaned, env)


# ══════════════════════════════════════════════════════════════════════
#  MODEL LOADING  ── graceful degradation if files missing
# ══════════════════════════════════════════════════════════════════════

def try_load_model():
    """Returns (model, scaler) or (None, None) if either is unavailable.
    Doesn't raise -- live drain must keep working even without ML."""
    try:
        import joblib
    except ImportError:
        print("[DSP] joblib not installed -- feature-only mode", flush=True)
        return None, None

    if not (os.path.exists(MODEL_PATH) and os.path.exists(SCALER_PATH)):
        print(f"[DSP] {MODEL_PATH} or scaler missing -- feature-only mode",
              flush=True)
        return None, None

    try:
        model           =   joblib.load(MODEL_PATH)
        scaler          =   joblib.load(SCALER_PATH)
    except Exception as e:
        print(f"[DSP] model load failed ({e}) -- feature-only mode",
              flush=True)
        return None, None

    # Sanity: model must accept 12 features. If it doesn't, the team's
    # joblib is for a different number of channels and we'd crash on
    # predict_proba. Refuse to use it rather than poison downstream.
    n_expected          =   NUM_ACTIVE_CH * 4
    n_model             =   getattr(scaler, 'n_features_in_',
                                    getattr(model, 'n_features_in_', n_expected))
    if n_model != n_expected:
        print(f"[DSP] model expects {n_model} features, "
              f"this build produces {n_expected} -- feature-only mode",
              flush=True)
        return None, None

    print(f"[DSP] model + scaler loaded (classes={list(model.classes_)})",
          flush=True)
    return model, scaler


# ══════════════════════════════════════════════════════════════════════
#  MAIN LOOP  ── inference mode
# ══════════════════════════════════════════════════════════════════════

def run_inference(verbose=False):
    ipc                 =   IPCBridge()
    ipc.set_dsp_ready()
    print(f"[DSP] connected, dsp_ready=1, "
          f"active_channels={ACTIVE_CHANNELS}, fs_in={INPUT_FS_HZ}Hz, "
          f"fs_target={TARGET_FS_HZ}Hz", flush=True)

    model, scaler       =   try_load_model()
    inference_enabled   =   (model is not None)

    # Per-channel rolling buffers at 2 kHz
    buffers             =   [deque([0]*BUFFER_SAMPLES, maxlen=BUFFER_SAMPLES)
                              for _ in range(NUM_ACTIVE_CH)]

    # Stride bookkeeping
    samples_since_window=   0
    total_samples_seen  =   0

    # Debouncing state (matches predict.py)
    current_state       =   "rest"
    consecutive_count   =   0
    last_active_class   =   0

    # Periodic-report state
    last_report_t       =   time.monotonic()
    inferences_done     =   0

    # Graceful shutdown
    running             =   [True]
    def stop(signum, frame):
        running[0]      =   False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT,  stop)

    while running[0]:
        loop_t0         =   time.monotonic()

        # ── 1. Drain ring (always, regardless of window readiness) ──
        batch           =   ipc.pop_sensor_batch(DRAIN_BATCH)
        n               =   batch.get('count', 0)
        if n > 0:
            ipc.inc_dsp_batches(n)
            samples     =   batch['samples']           # (n, 2, 8) uint16
            for entry_idx in range(n):
                for sub_idx in range(2):               # 2 samples / packet
                    for buf_idx, ch in enumerate(ACTIVE_CHANNELS):
                        v   =   int(samples[entry_idx, sub_idx, ch]) - ADC_MIDRAIL
                        buffers[buf_idx].append(v)
            samples_added           =   n * 2
            samples_since_window   +=   samples_added
            total_samples_seen     +=   samples_added

        # ── 2. New analysis window every STRIDE_SAMPLES_HI ──
        ready           =   (samples_since_window >= STRIDE_SAMPLES_HI
                              and total_samples_seen >= WINDOW_SAMPLES_HI)

        if ready:
            t_inf0      =   time.monotonic()
            samples_since_window    =   0

            # Take latest window per channel
            features_flat           =   []
            rms_per_ch              =   [0.0] * 8        # full 8-ch RMS to publish
            for buf_idx, ch in enumerate(ACTIVE_CHANNELS):
                w_hi    =   np.array(list(buffers[buf_idx])[-WINDOW_SAMPLES_HI:],
                                     dtype=np.float64)
                _cleaned, _env, feats = process_window(w_hi)
                features_flat.extend(feats)
                rms_per_ch[ch]      =   feats[0]

            # ── 3. Inference + debounce (if model loaded) ──
            if inference_enabled:
                X       =   np.asarray(features_flat, dtype=np.float64).reshape(1, -1)
                Xs      =   scaler.transform(X)
                probs   =   model.predict_proba(Xs)[0]
                ai      =   int(np.argmax(probs))
                label   =   str(model.classes_[ai])
                conf    =   float(probs[ai])

                # Hysteresis matching predict.py:
                # need CONFIRMATION_THRESHOLD consecutive predictions
                # above PROBABILITY_THRESHOLD to switch state
                if conf > PROBABILITY_THRESHOLD and label != current_state:
                    consecutive_count  +=   1
                    if consecutive_count >= CONFIRMATION_THRESHOLD:
                        current_state   =   label
                        consecutive_count   =   0
                else:
                    consecutive_count   =   0

                last_active_class       =   ai
                class_conf_full         =   list(probs)
                conf_pct                =   int(round(conf * 100.0))
            else:
                class_conf_full         =   []
                conf_pct                =   0

            # ── 4. Publish to IPC ──
            inference_us                =   int((time.monotonic() - t_inf0) * 1e6)
            ipc.update_dsp_max_latency(inference_us)
            ipc.write_dsp_export(
                channel_rms             =   rms_per_ch,
                gesture_name            =   current_state,
                class_confidence        =   class_conf_full,
                active_class            =   last_active_class,
                inference_time_us       =   inference_us,
            )

            # Drive servos from the gesture map (or neutral if unmapped)
            servo_us    =   GESTURE_SERVO_MAP.get(
                                current_state,
                                [SERVO_NEUTRAL_US] * NUM_SERVOS
                            )
            ipc.write_motor_cmd(servo_us, last_active_class, conf_pct)

            ipc.inc_dsp_inferences()
            inferences_done    +=   1

            if verbose:
                print(f"[DSP] state={current_state} conf={conf_pct}% "
                      f"rms={[f'{r:.1f}' for r in rms_per_ch[:NUM_ACTIVE_CH]]} "
                      f"lat_us={inference_us}", flush=True)

        # ── 5. Periodic report (parallel to launch.sh log lines) ──
        if loop_t0 - last_report_t >= 5.0:
            ring_now    =   ipc.sensor_count()
            print(f"[DSP] inferences={inferences_done} ring={ring_now} "
                  f"state={current_state}", flush=True)
            last_report_t       =   loop_t0

        # ── 6. Sleep the remainder of the 20 ms tick ──
        elapsed         =   time.monotonic() - loop_t0
        sleep_for       =   DRAIN_PERIOD_S - elapsed
        if sleep_for > 0:
            time.sleep(sleep_for)

    # Clean shutdown
    print("[DSP] shutdown clean", flush=True)
    ipc.close()
    return 0


# ══════════════════════════════════════════════════════════════════════
#  CALIBRATION MODE  ── reproduces proccess.py's noise_thresholds.json
# ══════════════════════════════════════════════════════════════════════

def run_calibrate(seconds):
    """Collect `seconds` of live samples (assumed REST) from the IPC,
    run the full pipeline, compute 3*std per channel on the cleaned
    signal, dump to noise_thresholds.json. Reproduces proccess.py's
    final_thresholds calculation but on Pi-collected data."""
    ipc                 =   IPCBridge()
    print(f"[DSP-CAL] collecting {seconds}s of REST data, sit still...",
          flush=True)

    buffers             =   [deque(maxlen=int(INPUT_FS_HZ * (seconds + 1)))
                              for _ in range(NUM_ACTIVE_CH)]

    t_end               =   time.monotonic() + seconds
    while time.monotonic() < t_end:
        batch           =   ipc.pop_sensor_batch(DRAIN_BATCH)
        n               =   batch.get('count', 0)
        if n > 0:
            samples     =   batch['samples']
            for entry_idx in range(n):
                for sub_idx in range(2):
                    for buf_idx, ch in enumerate(ACTIVE_CHANNELS):
                        v   =   int(samples[entry_idx, sub_idx, ch]) - ADC_MIDRAIL
                        buffers[buf_idx].append(v)
        time.sleep(DRAIN_PERIOD_S)

    print(f"[DSP-CAL] collected {[len(b) for b in buffers]} samples/ch, "
          f"running pipeline...", flush=True)

    # Run the cleaning pipeline on the full capture, exactly like
    # proccess.py does on a CSV column, then take 3*std of the result.
    thresholds          =   {}
    for buf_idx, label in enumerate(CHANNEL_LABELS):
        sig             =   np.array(buffers[buf_idx], dtype=np.float64)
        if len(sig) < WINDOW_SAMPLES_HI:
            print(f"[DSP-CAL] {label}: insufficient data, fallback=50.0",
                  flush=True)
            thresholds[label]   =   50.0
            continue
        # Decimate full capture to 200 Hz
        sig_lo          =   decimate(sig, DECIMATE_FACTOR, zero_phase=True)
        centered        =   sig_lo - np.mean(sig_lo)
        bp              =   butter_bandpass(centered, 20.0, 450.0, TARGET_FS_HZ)
        cleaned         =   notch_filter(bp, 50.0, TARGET_FS_HZ)
        # Threshold = 3 * std (matches proccess.py final_thresholds calc)
        thr             =   float(np.std(cleaned) * 3.0)
        thresholds[label]   =   thr
        print(f"[DSP-CAL] {label} (BSAU ch{ACTIVE_CHANNELS[buf_idx]}): "
              f"std={np.std(cleaned):.2f}  threshold={thr:.2f}",
              flush=True)

    os.makedirs(MODEL_DIR, exist_ok=True)
    with open(THRESHOLDS_PATH, 'w') as f:
        json.dump(thresholds, f, indent=4)
    print(f"[DSP-CAL] saved {THRESHOLDS_PATH}", flush=True)

    ipc.close()
    return 0


# ══════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════

def main():
    ap                  =   argparse.ArgumentParser(
                                description="CPCU live DSP + ML pipeline")
    ap.add_argument('--calibrate', type=float, metavar='SECONDS',
                    default=None,
                    help="Collect N seconds of rest, write noise_thresholds.json, exit")
    ap.add_argument('--verbose', action='store_true',
                    help="Print per-window state to stdout")
    args                =   ap.parse_args()

    if args.calibrate is not None:
        return run_calibrate(args.calibrate)
    return run_inference(verbose=args.verbose)


if __name__ == "__main__":
    sys.exit(main())
