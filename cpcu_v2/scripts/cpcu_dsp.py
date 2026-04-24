#!/usr/bin/env python3
"""
cpcu_dsp.py — Cores 1-2 DSP/AI pipeline (Python)

Replaces the C cpcu_dsp placeholder. Consumes EMG data from the SPSC
ring buffer, runs scipy causal filters + sklearn RandomForest, writes
servo commands via SeqLock.

Launch: taskset -c 1,2 chrt -f 80 python3 cpcu_dsp.py

Author: bugrASl + Aleyna (ML model)
Date:   April 2026

SMP NOTE: CPython's GIL means only one thread runs Python at a time.
  However, numpy/scipy release the GIL for C-level operations (sosfilt,
  matrix multiply, predict). With taskset -c 1,2, the kernel scheduler
  can migrate the process between cores when one is interrupted by GC.
  This gives us GC-pause resilience, not parallelism. True parallelism
  would require multiprocessing, but at ~30ms/inference on a 100ms
  budget, we have 70% headroom — no need to complicate things.
"""

import gc
import os
import signal
import sys
import time

import numpy as np

# ══════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ══════════════════════════════════════════════════════════════════════

# Sampling
FS                      =   2000            # BSAU sample rate (Hz)
FEATURE_WINDOW_MS       =   200             # Window size (ms), matches training
STRIDE_MS               =   100             # Stride (ms)
WIN_SAMPLES             =   int(FS * FEATURE_WINDOW_MS / 1000)     # 400
STRIDE_SAMPLES          =   int(FS * STRIDE_MS / 1000)             # 200

# Channel mapping: which BSAU ADC channels correspond to training sensors
# IMPORTANT: Verify these with oscilloscope on actual hardware!
S1_CHANNEL              =   0               # Forearm flexors (hand sensor)
S2_CHANNEL              =   4               # Biceps sensor

# Noise gate: force REST if both channels below this RMS threshold
# NOTE: Re-calibrate on real hardware. This was tuned for raw ADC @ 1kHz.
NOISE_GATE_RMS          =   0.005           # In volts after filtering

# Model
MODEL_PATH              =   "/opt/cpcu/models/emg_rf_model.pkl"
MODEL_PATH_ALT          =   "./emg_rf_model.pkl"       # Dev fallback

# Ring buffer
BATCH_SIZE              =   100

# Diagnostics
DIAG_INTERVAL_S         =   1.0

# ══════════════════════════════════════════════════════════════════════
#  GESTURE → SERVO MAPPING
# ══════════════════════════════════════════════════════════════════════
# Servo indices: S0=Thumb S1=Index S2=Middle S3=Ring S4=Pinky S5=Wrist
# Values in microseconds. PCA driver clamps to per-joint min/max.

GESTURE_SERVO_MAP       =   {
    0:  [1500, 1500, 1500, 1500, 1500, 1500],      # REST
    1:  [1700, 1700, 1700, 1700, 1700, 1500],      # HAND SLOW
    2:  [2200, 2200, 2200, 2200, 2200, 1500],      # HAND HARD
    3:  [1000, 1000, 1000, 1000, 1000, 1500],      # HAND OPEN
    4:  [1500, 1500, 1500, 1500, 1500, 1650],      # ARM BEND LESS
    5:  [1500, 1500, 1500, 1500, 1500, 1800],      # ARM BEND MIDDLE
    6:  [1500, 1500, 1500, 1500, 1500, 2000],      # ARM BEND MOST
    7:  [1500, 1500, 1500, 1500, 1500, 1650],      # ARM SLOW
    8:  [1500, 1500, 1500, 1500, 1500, 2000],      # ARM FAST
    9:  [1500, 1500, 1500, 1500, 1500, 1500],      # BICEPS ONLY
}

CLASS_NAMES             =   {
    0: 'REST',          1: 'HAND SLOW',     2: 'HAND HARD',
    3: 'HAND OPEN',     4: 'ARM BND L',     5: 'ARM BND M',
    6: 'ARM BND H',     7: 'ARM SLOW',      8: 'ARM FAST',
    9: 'BICEP ONLY',
}

SERVO_NEUTRAL           =   [1500, 1500, 1500, 1500, 1500, 1500]

# ══════════════════════════════════════════════════════════════════════
#  SIGNAL HANDLER
# ══════════════════════════════════════════════════════════════════════

g_running               =   True

def _on_signal(sig, frame):
    global g_running
    g_running           =   False

signal.signal(signal.SIGINT, _on_signal)
signal.signal(signal.SIGTERM, _on_signal)

# ══════════════════════════════════════════════════════════════════════
#  FEATURE EXTRACTION — MUST match prep.py get_features() EXACTLY
# ══════════════════════════════════════════════════════════════════════

def get_features(window):
    """
    Extract 7 features from a filtered EMG window.
    Order: [mav, rms, wl, zc, ssc, var, log_det]
    
    DO NOT MODIFY without retraining the model.
    """
    eps                 =   1e-6
    mav                 =   np.mean(np.abs(window))
    rms                 =   np.sqrt(np.mean(window ** 2))
    wl                  =   np.sum(np.abs(np.diff(window)))
    zc                  =   np.sum(
                                (window[:-1] * window[1:] < 0) &
                                (np.abs(window[:-1] - window[1:]) > 0.01)
                            )
    diff1               =   np.diff(window)
    ssc                 =   np.sum(
                                (diff1[:-1] * diff1[1:] < 0) &
                                (np.abs(diff1[:-1] - diff1[1:]) > 0.01)
                            )
    var                 =   np.var(window)
    log_det             =   np.exp(np.mean(np.log(np.abs(window) + eps)))
    return [mav, rms, wl, zc, ssc, var, log_det]


# ══════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════

def main():
    # Late imports so error messages are cleaner if missing
    from scipy.signal import butter, iirnotch, sosfilt, sosfilt_zi, lfilter, lfilter_zi
    import joblib
    from cpcu_ipc_bridge import IPCBridge, NUM_SERVOS

    print("=== CPCU DSP/AI (Python, Cores 1-2) ===")

    # ── Disable cyclic GC for reduced latency jitter ──
    gc.disable()
    print("[DSP] GC disabled (ref-counting only)")

    # ── Load model ──
    model_path          =   MODEL_PATH if os.path.exists(MODEL_PATH) else MODEL_PATH_ALT
    if not os.path.exists(model_path):
        print(f"[DSP] FATAL: model not found at {MODEL_PATH} or {MODEL_PATH_ALT}")
        sys.exit(1)

    try:
        model_data      =   joblib.load(model_path)
        model           =   model_data["model"]
        scaler          =   model_data["scaler"]
        print(f"[DSP] Model loaded from {model_path}")
        print(f"[DSP] Features expected: {scaler.n_features_in_}")
    except Exception as e:
        print(f"[DSP] FATAL: model load failed: {e}")
        sys.exit(1)

    # ── Open shared memory ──
    try:
        ipc             =   IPCBridge()
        print("[DSP] IPC bridge opened")
    except Exception as e:
        print(f"[DSP] FATAL: IPC open failed: {e}")
        sys.exit(1)

    # ── Design causal filters (same specs as prep.py, but SOS form) ──
    nyq                 =   0.5 * FS
    sos_band            =   butter(4, [20 / nyq, 450 / nyq], btype='band', output='sos')
    b_notch, a_notch    =   iirnotch(50, 30, FS)

    # Per-channel filter state (2 channels: s1 and s2)
    zi_band             =   [sosfilt_zi(sos_band).copy() for _ in range(2)]
    zi_notch            =   [lfilter_zi(b_notch, a_notch).copy() for _ in range(2)]

    # ── Sliding window buffers ──
    win_s1              =   np.zeros(WIN_SAMPLES, dtype=np.float64)
    win_s2              =   np.zeros(WIN_SAMPLES, dtype=np.float64)
    wpos                =   0
    wcount              =   0
    stride_ctr          =   0

    # ── State ──
    prediction          =   0
    confidence          =   0
    last_rms_s1         =   0.0
    last_rms_s2         =   0.0
    lat_us              =   0.0

    # ── Signal ready ──
    ipc.set_dsp_ready()
    print(f"[DSP] Ready. S1=ch{S1_CHANNEL} S2=ch{S2_CHANNEL} "
          f"win={WIN_SAMPLES} stride={STRIDE_SAMPLES}")

    t_diag              =   time.monotonic()

    # ══════════════════════════════════════════════════════════════════
    #  MAIN LOOP
    # ══════════════════════════════════════════════════════════════════

    while g_running:
        batch           =   ipc.pop_sensor_batch(max_count=BATCH_SIZE)

        if batch['count'] == 0:
            time.sleep(0.0001)      # 100 us
            continue

        n               =   batch['count']
        samples         =   batch['samples']        # shape (n, 2, 8) uint16

        for i in range(n):
            for s in range(2):      # 2 samples per packet
                # Extract the two channels we use
                raw_s1  =   float(samples[i, s, S1_CHANNEL])
                raw_s2  =   float(samples[i, s, S2_CHANNEL])

                # 12-bit ADC -> voltage -> remove DC bias (1.65V midpoint)
                v_s1    =   raw_s1 * 3.3 / 4095.0 - 1.65
                v_s2    =   raw_s2 * 3.3 / 4095.0 - 1.65

                # Causal bandpass + notch filter (per-sample, maintaining state)
                out, zi_band[0]     =   sosfilt(sos_band, [v_s1], zi=zi_band[0])
                out, zi_notch[0]    =   lfilter(b_notch, a_notch, out, zi=zi_notch[0])
                filt_s1             =   out[0]

                out, zi_band[1]     =   sosfilt(sos_band, [v_s2], zi=zi_band[1])
                out, zi_notch[1]    =   lfilter(b_notch, a_notch, out, zi=zi_notch[1])
                filt_s2             =   out[0]

                # Push into sliding window
                win_s1[wpos]        =   filt_s1
                win_s2[wpos]        =   filt_s2
                wpos                =   (wpos + 1) % WIN_SAMPLES
                if wcount < WIN_SAMPLES:
                    wcount         +=   1
                stride_ctr         +=   1

                # ── Inference at stride boundary ──
                if stride_ctr >= STRIDE_SAMPLES and wcount >= WIN_SAMPLES:
                    stride_ctr      =   0
                    t0              =   time.monotonic()

                    # Reorder circular buffer to linear (numpy roll)
                    ordered_s1      =   np.roll(win_s1, -wpos)
                    ordered_s2      =   np.roll(win_s2, -wpos)

                    # Feature extraction (MUST match prep.py order)
                    feat_s1         =   get_features(ordered_s1)
                    feat_s2         =   get_features(ordered_s2)
                    combined        =   np.array(feat_s1 + feat_s2).reshape(1, -1)

                    # Scale and predict
                    scaled          =   scaler.transform(combined)
                    prediction      =   int(model.predict(scaled)[0])

                    # Per-class confidence
                    proba           =   model.predict_proba(scaled)[0]
                    confidence      =   int(proba[prediction] * 100)

                    # Noise gate: force REST if both channels quiet
                    last_rms_s1     =   feat_s1[1]      # rms is index 1
                    last_rms_s2     =   feat_s2[1]
                    if last_rms_s1 < NOISE_GATE_RMS and last_rms_s2 < NOISE_GATE_RMS:
                        prediction  =   0
                        confidence  =   100

                    # Map gesture -> servo pulse widths
                    servo_us        =   GESTURE_SERVO_MAP.get(prediction, SERVO_NEUTRAL)

                    # Write motor command via SeqLock
                    ipc.write_motor_cmd(servo_us, prediction, confidence)
                    ipc.inc_dsp_inferences()

                    lat_us          =   (time.monotonic() - t0) * 1_000_000
                    ipc.update_dsp_max_latency(int(lat_us))

                    # Write DSP export for TUI
                    channel_rms     =   [0.0] * 8
                    channel_rms[S1_CHANNEL] =   last_rms_s1
                    channel_rms[S2_CHANNEL] =   last_rms_s2
                    gesture_name    =   CLASS_NAMES.get(prediction, "???")

                    ipc.write_dsp_export(
                        channel_rms         =   channel_rms,
                        gesture_name        =   gesture_name,
                        class_confidence    =   proba.tolist(),
                        active_class        =   prediction,
                        inference_time_us   =   int(lat_us),
                    )

        ipc.inc_dsp_batches(n)

        # ── Periodic diagnostics to stdout ──
        t                   =   time.monotonic()
        if t - t_diag >= DIAG_INTERVAL_S:
            t_diag          =   t
            name            =   CLASS_NAMES.get(prediction, "???")
            print(f"[DSP] gesture={name} conf={confidence}% "
                  f"rms_s1={last_rms_s1:.4f} rms_s2={last_rms_s2:.4f} "
                  f"lat={lat_us:.0f}us ring={ipc.sensor_count()}")

    # ── Cleanup ──
    print("[DSP] Shutdown")
    ipc.close()


if __name__ == "__main__":
    main()
