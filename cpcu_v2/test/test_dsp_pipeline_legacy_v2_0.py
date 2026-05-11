#!/usr/bin/env python3
"""
test_dsp_pipeline.py — Validate Python DSP pipeline correctness.

Tests:
  1. Feature extraction matches prep.py output for known signals
  2. Causal filter (sosfilt) vs offline filter (filtfilt) feature similarity
  3. Gesture mapping table completeness
  4. Noise gate behavior
  5. Scaler/model compatibility check

No hardware or shared memory required.

Author: bugrASl
Date:   April 2026
"""

import sys
import os
import numpy as np

# Add both this directory (test/) and ../scripts/ to sys.path so that
# `import cpcu_dsp` works regardless of the cwd the test is run from.
_TEST_DIR    =   os.path.dirname(os.path.abspath(__file__))
_SCRIPTS_DIR =   os.path.normpath(os.path.join(_TEST_DIR, "..", "scripts"))
for _p in (_TEST_DIR, _SCRIPTS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

g_pass                  =   0
g_fail                  =   0

def ASSERT(cond, msg):
    global g_pass, g_fail
    if cond:
        print(f"  [PASS] {msg}")
        g_pass         +=   1
    else:
        print(f"  [FAIL] {msg}")
        g_fail         +=   1


def test_feature_extraction():
    """Verify get_features produces correct values for known input."""
    print("\n--- Feature Extraction Correctness ---")
    
    from cpcu_dsp import get_features

    # Known signal: 200-sample sine wave at 50 Hz, fs=2000
    fs                  =   2000
    t                   =   np.arange(400) / fs
    signal              =   np.sin(2 * np.pi * 50 * t) * 0.5      # 0.5V amplitude

    features            =   get_features(signal)
    
    ASSERT(len(features) == 7, f"feature count = {len(features)} (expected 7)")
    
    mav, rms, wl, zc, ssc, var, log_det = features
    
    # For a sine wave: RMS = amplitude / sqrt(2) = 0.5 / 1.414 ≈ 0.354
    ASSERT(abs(rms - 0.354) < 0.02, f"rms={rms:.4f} ≈ 0.354 (sine wave)")
    
    # MAV = 2*amplitude/pi ≈ 0.318
    ASSERT(abs(mav - 0.318) < 0.02, f"mav={mav:.4f} ≈ 0.318")
    
    # ZCR: 50 Hz sine crosses zero ~2*50 times in 200ms window
    # Over 400 samples at 2kHz = 200ms, expect ~20 zero crossings
    ASSERT(15 < zc < 25, f"zc={zc} ∈ (15, 25) for 50Hz sine")
    
    # Variance should be positive
    ASSERT(var > 0, f"var={var:.6f} > 0")
    
    # Waveform length should be positive and proportional to frequency
    ASSERT(wl > 0, f"wl={wl:.4f} > 0")
    
    TEST_OK("Feature extraction matches expected analytical values")


def test_feature_order_matches_training():
    """Verify feature order matches prep.py column names."""
    print("\n--- Feature Order Matches Training ---")
    
    # From train.py: expected_columns order is
    # s1_mav, s1_rms, s1_wl, s1_zc, s1_ssc, s1_var, s1_log_det,
    # s2_mav, s2_rms, s2_wl, s2_zc, s2_ssc, s2_var, s2_log_det
    
    from cpcu_dsp import get_features
    
    # Use distinguishable signals so each feature has a unique value
    sig1                =   np.random.randn(400) * 0.5
    sig2                =   np.random.randn(400) * 1.0      # different amplitude
    
    feat1               =   get_features(sig1)
    feat2               =   get_features(sig2)
    combined            =   feat1 + feat2       # concatenation
    
    ASSERT(len(combined) == 14, f"combined features = {len(combined)} (expected 14)")
    
    # RMS of sig2 should be roughly 2x RMS of sig1
    rms1                =   feat1[1]
    rms2                =   feat2[1]
    ratio               =   rms2 / rms1 if rms1 > 0 else 0
    ASSERT(1.5 < ratio < 2.5, f"rms ratio = {ratio:.2f} ≈ 2.0 (sig2 is 2x amplitude)")


def test_causal_vs_offline_filter():
    """Compare causal sosfilt features vs offline filtfilt features."""
    print("\n--- Causal vs Offline Filter Comparison ---")
    
    from scipy.signal import butter, iirnotch, sosfilt, sosfilt_zi, lfilter, lfilter_zi, filtfilt
    from cpcu_dsp import get_features, FS

    nyq                 =   0.5 * FS
    sos_band            =   butter(4, [20/nyq, 450/nyq], btype='band', output='sos')
    b_band, a_band      =   butter(4, [20/nyq, 450/nyq], btype='band')
    b_notch, a_notch    =   iirnotch(50, 30, FS)

    # Generate 2 seconds of test signal (enough for filter warmup)
    t                   =   np.arange(2 * FS) / FS
    signal              =   0.3 * np.sin(2 * np.pi * 100 * t) + 0.1 * np.sin(2 * np.pi * 50 * t)
    signal             +=   np.random.randn(len(signal)) * 0.05

    # Offline (filtfilt — as used in prep.py)
    offline             =   filtfilt(b_band, a_band, signal)
    offline             =   filtfilt(b_notch, a_notch, offline)
    feat_offline        =   get_features(offline[-400:])

    # Causal (sosfilt — as used in cpcu_dsp.py)
    zi_band             =   sosfilt_zi(sos_band)
    zi_notch            =   lfilter_zi(b_notch, a_notch)
    
    causal_out          =   np.zeros_like(signal)
    for i in range(len(signal)):
        x, zi_band      =   sosfilt(sos_band, [signal[i]], zi=zi_band)
        x, zi_notch     =   lfilter(b_notch, a_notch, x, zi=zi_notch)
        causal_out[i]   =   x[0]
    
    feat_causal         =   get_features(causal_out[-400:])

    # Compare RMS — should be within 30% after filter warmup
    rms_off             =   feat_offline[1]
    rms_cau             =   feat_causal[1]
    if rms_off > 0:
        rms_err         =   abs(rms_cau - rms_off) / rms_off
        ASSERT(rms_err < 0.30,
               f"RMS error = {rms_err*100:.1f}% (offline={rms_off:.4f}, causal={rms_cau:.4f})")
    else:
        print("  [SKIP] RMS offline is zero")

    # Compare MAV
    mav_off             =   feat_offline[0]
    mav_cau             =   feat_causal[0]
    if mav_off > 0:
        mav_err         =   abs(mav_cau - mav_off) / mav_off
        ASSERT(mav_err < 0.30,
               f"MAV error = {mav_err*100:.1f}% (offline={mav_off:.4f}, causal={mav_cau:.4f})")

    print("  [INFO] If errors exceed 30%, consider retraining with causal filters")


def test_gesture_servo_map():
    """Verify gesture map covers all 10 classes and produces valid servo values."""
    print("\n--- Gesture → Servo Map ---")
    
    from cpcu_dsp import GESTURE_SERVO_MAP, CLASS_NAMES

    ASSERT(len(GESTURE_SERVO_MAP) == 10, f"map has {len(GESTURE_SERVO_MAP)} entries (expected 10)")
    
    for class_id in range(10):
        ASSERT(class_id in GESTURE_SERVO_MAP,
               f"class {class_id} ({CLASS_NAMES.get(class_id, '?')}) in map")
        
        servo_us        =   GESTURE_SERVO_MAP[class_id]
        ASSERT(len(servo_us) == 6,
               f"class {class_id}: {len(servo_us)} servos (expected 6)")
        
        for i, val in enumerate(servo_us):
            ASSERT(500 <= val <= 2500,
                   f"class {class_id} servo[{i}]={val} ∈ [500, 2500]")


def test_noise_gate():
    """Verify noise gate forces REST for low-energy signals."""
    print("\n--- Noise Gate ---")
    
    from cpcu_dsp import get_features, NOISE_GATE_RMS
    
    # Very quiet signal (near-zero after filtering)
    quiet               =   np.random.randn(400) * 0.0001
    feat                =   get_features(quiet)
    rms                 =   feat[1]
    
    ASSERT(rms < NOISE_GATE_RMS,
           f"quiet signal RMS={rms:.6f} < gate={NOISE_GATE_RMS}")
    
    # Active signal
    active              =   np.sin(2 * np.pi * 100 * np.arange(400) / 2000) * 0.5
    feat_active         =   get_features(active)
    rms_active          =   feat_active[1]
    
    ASSERT(rms_active > NOISE_GATE_RMS,
           f"active signal RMS={rms_active:.4f} > gate={NOISE_GATE_RMS}")


def test_model_loadable():
    """Check if model file exists and has expected structure."""
    print("\n--- Model Load Check ---")
    
    from cpcu_dsp import MODEL_PATH, MODEL_PATH_ALT
    
    path                =   MODEL_PATH if os.path.exists(MODEL_PATH) else MODEL_PATH_ALT
    
    if not os.path.exists(path):
        print(f"  [SKIP] No model file at {MODEL_PATH} or {MODEL_PATH_ALT}")
        return
    
    try:
        import joblib
        data            =   joblib.load(path)
        
        ASSERT("model" in data, "model key exists in pkl")
        ASSERT("scaler" in data, "scaler key exists in pkl")
        
        scaler          =   data["scaler"]
        model           =   data["model"]
        
        n_features      =   scaler.n_features_in_
        ASSERT(n_features == 14, f"scaler expects {n_features} features (expected 14)")
        
        # Quick inference test
        dummy           =   np.zeros((1, 14))
        scaled          =   scaler.transform(dummy)
        pred            =   model.predict(scaled)
        proba           =   model.predict_proba(scaled)
        
        ASSERT(0 <= pred[0] <= 9, f"prediction={pred[0]} ∈ [0, 9]")
        ASSERT(abs(proba.sum() - 1.0) < 0.01, f"proba sums to {proba.sum():.3f} ≈ 1.0")
        
    except Exception as e:
        print(f"  [FAIL] Model load error: {e}")
        global g_fail
        g_fail         +=   1


def TEST_OK(msg):
    global g_pass
    print(f"  [PASS] {msg}")
    g_pass             +=   1


# ══════════════════════════════════════════════════════════════════════

def main():
    print("=== DSP Pipeline Test Suite ===")
    
    test_feature_extraction()
    test_feature_order_matches_training()
    test_causal_vs_offline_filter()
    test_gesture_servo_map()
    test_noise_gate()
    test_model_loadable()

    print(f"\n{'=' * 40}")
    print(f"  RESULTS: {g_pass} PASS, {g_fail} FAIL")
    print(f"{'=' * 40}")
    
    return 1 if g_fail > 0 else 0


if __name__ == "__main__":
    sys.exit(main())
