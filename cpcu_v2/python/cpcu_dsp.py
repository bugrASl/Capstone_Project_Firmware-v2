#!/usr/bin/env python3
"""
cpcu_dsp.py — Live DSP + ML inference pipeline for the CPCU.

Mirrors the offline Python pipeline developed by the DSP/AI team
(proccess.py + feature_ex.py + model.py + predict.py), but reads from
shared memory (/dev/shm/cpcu_ipc) instead of CSV files and writes
inference results back through IPC for cpcu_tui and cpcu_io to consume.

────────────────────────────────────────────────────────────────────────
HOW THE TEAM'S PIPELINE WORKS (read this before changing anything here)
────────────────────────────────────────────────────────────────────────

The team has FOUR scripts that together define the model:

  proccess.py     One-time clean-up. Reads raw CSV recordings from
                  datasets/1/, applies a 20-450 Hz Butterworth bandpass
                  + 50 Hz notch (filtfilt, zero-phase). Writes to
                  datasets/2/. Also computes per-sensor noise thresholds
                  as 3 * std(rest signal) into dynamic_noise_thresholds.json.
                  *Not used by the trained model;* purely a preprocessing
                  utility.

  feature_ex.py   Reads datasets/2/. Applies ANOTHER 15-90 Hz bandpass +
                  50 Hz notch (NOTE: different band from proccess.py),
                  then 3 Hz lowpass on |x| for the envelope. Slides a
                  40-sample (200 ms) window with 20-sample (100 ms)
                  stride. Writes 12-feature CSV rows: per sensor (s1, s2,
                  s3) × {RMS, VAR, WL, ENV_mean}.

  model.py        Loads features_200hz_segmented.csv. StratifiedGroupKFold
                  split (5-fold, seed 33, take first split). StandardScaler.
                  SVM with RBF kernel, C=10, gamma='scale',
                  class_weight='balanced', probability=True. Saves
                  hmi_svm_model_200hz.joblib + hmi_scaler_200hz.joblib.

  predict.py      Live test rig: serial COM10 @ 115200, ASCII "s1,s2,s3"
                  lines at 200 Hz. Applies a 20-450 Hz bandpass (which
                  auto-clamps to 20-95 Hz at Fs=200 — see TRAINING-VS-
                  LIVE FILTER MISMATCH below) + 50 Hz notch + 3 Hz
                  envelope. 3-of-5 hysteresis vote, 0.65 confidence
                  threshold.

cpcu_dsp.py is a faithful port of predict.py's signal chain to the Pi:
same filters, same hysteresis, same feature names. The differences are
all on the *transport* side:

  Team (predict.py, offline)        CPCU (cpcu_dsp.py, live)
  ─────────────────────────────────────────────────────────────
  serial COM10, 115200 baud         /dev/shm/cpcu_ipc SPSC ring
  3 channels @ 200 Hz native        8 channels @ 2 kHz from BSAU
  ASCII "s1,s2,s3" lines            binary IPC_SensorEntry
  matplotlib FuncAnimation           cpcu_kernel-spawned daemon
  prints to status box               writes to dsp_export + motor_cmd

────────────────────────────────────────────────────────────────────────
TRAINING-VS-LIVE FILTER MISMATCH (caveat — accept and document)
────────────────────────────────────────────────────────────────────────

The training pipeline (feature_ex.py) bandpasses at 15-90 Hz.
The live pipeline (predict.py and this file) bandpasses at 20-450 Hz,
which scipy auto-clamps to 20-95 Hz at Fs=200. So the two filters
nominally pass slightly different bands:

    Training:  15.0 -  89.8 Hz  (-3 dB points, 4th-order Butterworth)
    Live:      20.1 -  94.9 Hz

In practice, on real EMG (dominant 30-80 Hz energy), the two filters
produce features that match within ~0.2% RMS — the model generalises
across that gap fine. Where you DO see a difference:

  - 15-20 Hz motion artifacts (electrode shift, jaw clench): training
    "saw" them and learned to ignore them; live discards them upstream,
    so live features are slightly cleaner than training in this band.
  - 90-95 Hz spectral edge: live keeps it, training discarded it. Minor
    contribution to total RMS for typical EMG.

This file matches predict.py (the team's live validation rig), not
feature_ex.py (the training pipeline), because if a discrepancy
*does* matter the team's empirical validation has been on the live
chain. If you decide to retrain on data captured through this exact
pipeline (CPCU-collected rather than the team's UART rig), the gap
disappears entirely. Either way the existing trained model works.

────────────────────────────────────────────────────────────────────────
CHANNEL MAPPING (set this correctly before bringing up live electrodes!)
────────────────────────────────────────────────────────────────────────

The team trained on three sensors with these physical positions:

    s1 = Forearm  (wrist flexor)
    s2 = Biceps
    s3 = Triceps

ACTIVE_CHANNELS below picks which 3 of the 8 BSAU ADC channels feed
the model, in s1/s2/s3 order. The default is [0, 1, 2] meaning BSAU
PA0 → s1 (Forearm), PA1 → s2 (Biceps), PA2 → s3 (Triceps).

If your electrodes are wired in a different physical-to-PA mapping,
edit ACTIVE_CHANNELS to match. Wrong mapping won't crash anything —
the SVM will just consistently mis-classify because feature[0..3] is
no longer the muscle the model thinks it is.

────────────────────────────────────────────────────────────────────────
NOISE-THRESHOLD CALIBRATION
────────────────────────────────────────────────────────────────────────

proccess.py computes per-sensor noise thresholds (3 × std of rest-state
signal) and saves them to dynamic_noise_thresholds.json. feature_ex.py
ALSO computes thresholds, but with a different formula (95th percentile
× 1.5 of rest envelope). Neither set of thresholds is used by the
trained SVM at inference time — the model operates purely on the 12
numeric features. So *for live inference, no calibration data is
required.* The model has learned the rest distribution from labelled
training data.

This file provides a `--calibrate N` mode for when you later retrain
on CPCU-collected data and want to reproduce the proccess.py-style
threshold JSON. It writes /opt/cpcu/models/noise_thresholds.json with
the 3×std formula.

────────────────────────────────────────────────────────────────────────
GESTURE-SERVO MAP COVERAGE
────────────────────────────────────────────────────────────────────────

GESTURE_SERVO_MAP below has entries for "rest", "biceps_flex",
"hand_flex", and "hand_open". The team's predict.py only references
the first three (in its color_map). If the trained .joblib's
.classes_ matches that — i.e., 3 classes — the "hand_open" entry is
never reached at inference time. That's fine; it's just future-proofing
for when the model is retrained with more classes. Any class the
model emits that isn't in the map falls back to all-neutral servos.

If model.classes_ has classes that ARE missing from the map, you'll
silently get neutral-servo output for that gesture. To diagnose, look
at the cpcu_tui DSP/AI page (key 3) which prints model.classes_ at
startup and the live class confidence vector.

────────────────────────────────────────────────────────────────────────
GRACEFUL DEGRADATION
────────────────────────────────────────────────────────────────────────

If /opt/cpcu/models/{hmi_svm_model_200hz.joblib, hmi_scaler_200hz.joblib}
exist, we run inference. If not, we still drain the ring and publish
features so cpcu_tui Page 3 (DSP/AI) lights up — just no gesture
classification, all servos held at neutral. Safety FSM is unaffected
either way (it gates on radio + battery + thermal + ring + i2c, not
on inference success).

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

# Which 3 of 8 BSAU channels feed the model. Order matters: the
# trained SVM expects features [s1_*, s2_*, s3_*] where:
#   s1 = Forearm   (wrist flexor electrode)
#   s2 = Biceps
#   s3 = Triceps
# Default [0, 1, 2] ⇒ BSAU PA0=Forearm, PA1=Biceps, PA2=Triceps.
# If your electrodes are wired differently, edit this — wrong mapping
# won't crash anything, you'll just get consistently-wrong gestures.
ACTIVE_CHANNELS         =   [0, 1, 2]
CHANNEL_LABELS          =   ['s1', 's2', 's3']      # must match training feat names
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

# Debounce defaults — overridden by runtime.json on startup if present.
PROBABILITY_THRESHOLD   =   0.65
CONFIRMATION_THRESHOLD  =   3                      # falls back to JSON's hysteresis_votes

# Servo defaults
SERVO_NEUTRAL_US        =   1500
NUM_SERVOS              =   6

# Drain rate (independent of inference rate so the ring never backs up)
DRAIN_PERIOD_S          =   0.020       # 50 Hz
DRAIN_BATCH             =   200

# hardware safety envelope (compile-time mins/maxes from
# cpcu_pca9685.h). dsp clamps published targets to these, cpcu_io
# clamps again on its side after applying servo_bias_us. Both clamps
# are needed: dsp's clamp gives the user immediate feedback in the
# TUI confidence display ("you've integrated past the limit, bias is
# saturated"), io's clamp is the absolute safety net.
SERVO_MIN_US            =   [ 498, 1074, 1074, 1001, 1001,  976]
SERVO_MAX_US            =   [2500, 1953, 1953, 2002, 2002, 1733]

# gripper soft-firm clamp. The integrator is prevented from
# closing the gripper below this position even if a velocity-mode
# gesture is held longer. One-sided — opening direction is unaffected.
# Loaded from runtime.json's grip_firm_us (default 1100). Range
# 800..2200 enforced by both the C parser (cpcu_config.c) and the
# dsp loader (load_dsp_runtime_config). See SOFT_GRIP.md.
GRIP_FIRM_US_DEFAULT    =   1100

# Default servo poses per gesture, in microseconds (6 servos).
# Gestures the team's predict.py acknowledges in its color_map (and
# therefore the trained SVM is known to emit):
#     "rest", "ext", "flex", "hand"
# "hand_open" is included as future-proofing for when the team retrains
# with more classes; if the current .joblib doesn't know that class,
# the entry simply never fires.
# Any model.classes_ entry NOT in this map falls back to all-neutral.
#
# this map is now the FALLBACK for "freeze-mode" gestures only.
# Velocity-mode gestures use GESTURE_BEHAVIOR (loaded from runtime.json,
# see VELOCITY_MODE.md). Any class without a velocity entry falls back
# to GESTURE_SERVO_MAP[label] as a fixed pose, preserving legacy
# behaviour.
# Class names must match aleynask.pkl model.classes_:
#   ['ext', 'flex', 'hand', 'rest']
GESTURE_SERVO_MAP       =   {
    "rest":         [1500, 1500, 1500, 1500, 1500, 1500],  # all neutral
    "ext":          [1500, 1500, 1500, 1500, 1500, 1700],  # extension (open hand)
    "flex":         [1500, 1700, 1500, 1500, 1500, 1500],  # flexion (biceps curl)
    "hand":         [1500, 1500, 1500, 1500, 1500, 1100],  # hand close (grip)
    # Legacy names kept as fallback if old model is loaded
    "biceps_flex":  [1500, 1700, 1500, 1500, 1500, 1500],
    "hand_flex":    [1500, 1500, 1500, 1500, 1500, 1100],
    "hand_open":    [1500, 1500, 1500, 1500, 1500, 1700],
}

# gesture-behaviour map. Keys are class names, values are
# dicts with:
#   "mode": "freeze" | "velocity"
#   "rate": [int]*NUM_SERVOS    (us/s, signed, only used when mode=velocity)
#
# "rest" is always freeze (target unchanged, hold pose).
# Other classes default to freeze if absent from the velocity map
# in runtime.json — keeping legacy behaviour as the safe default.
#
# Loaded from runtime.json's "gesture_velocity" object on startup.
# Format in JSON:
#   "gesture_velocity": {
#       "flex": [0, 200, 0, 0, 0, 0],       // biceps curl (elbow close)
#       "ext":  [0, -200, 0, 0, 0, 0],      // arm extend (elbow open)
#       "hand": [0, 0, 0, 0, 0, -200]       // hand close (grip)
#   }
# Negative values reverse direction. Zero rates effectively disable
# velocity mode for that channel (target += 0 = unchanged).
GESTURE_BEHAVIOR        =   {
    "rest":         {"mode": "freeze", "rate": [0]*NUM_SERVOS},
    # Other classes populated at runtime from JSON; default freeze.
}

# Confidence interpolation for velocity scaling. When the SVM's
# probability for the active class is at the floor, integration speed
# is 0 (effectively frozen). When at the ceiling, full speed. Linear
# ramp between. Loaded from runtime.json on startup.
INTERP_CONF_FLOOR       =   0.40        # 40%
INTERP_CONF_CEIL        =   0.85        # 85%


# ══════════════════════════════════════════════════════════════════════
#  RUNTIME CONFIG LOADER
# ══════════════════════════════════════════════════════════════════════
# The kernel reads cpcu_v2/config/runtime.json and the C-side parser
# in cpcu_config.c, mirrored to IPC_RuntimeConfig in shared memory.
# That covers everything cpcu_io needs (servo limits, deadband, bias).
#
# This step needs the gesture-velocity rows, which are
# string-keyed by class name and so don't fit cleanly into the C
# parser's flat-array model. So dsp loads its own slice of the JSON
# directly. Same file, different consumer.
#
# Three rules:
#   1. If runtime.json is missing or unparseable, log a WARNING and
#      use the in-code defaults. dsp must boot — kernel already
#      refuse-to-started if the JSON was truly broken.
#   2. Velocity rows for unknown classes (not in model.classes_) are
#      silently ignored. Adding a class to JSON doesn't crash dsp.
#   3. Classes WITHOUT a velocity row stay in freeze mode (the safe
#      default).

RUNTIME_CONFIG_PATH_DEFAULT  =  "/opt/cpcu/config.json"
RUNTIME_CONFIG_PATH_FALLBACK = "config/runtime.json"


def load_dsp_runtime_config(model_classes, path=None):
    """Read the dsp-side slice of runtime.json.

    Returns (interp_floor, interp_ceil, hysteresis_votes, behavior_map,
             grip_firm_us). The grip_firm_us is
             is the soft floor cpcu_dsp.py clamps the gripper integrator
             at — see SOFT_GRIP.md.

    If the file can't be opened or parsed, defaults are used and a
    warning is printed. cpcu_kernel will already have failed earlier
    if the JSON was structurally bad.
    """
    floor    = INTERP_CONF_FLOOR
    ceil_    = INTERP_CONF_CEIL
    votes    = CONFIRMATION_THRESHOLD
    grip_firm = GRIP_FIRM_US_DEFAULT
    behavior = {"rest": {"mode": "freeze", "rate": [0]*NUM_SERVOS}}
    # Default: every model class is freeze with its existing pose.
    for cls in model_classes:
        if cls != "rest":
            behavior[cls] = {"mode": "freeze", "rate": [0]*NUM_SERVOS}

    candidate_paths = [path] if path else [
        RUNTIME_CONFIG_PATH_DEFAULT,
        RUNTIME_CONFIG_PATH_FALLBACK,
    ]
    raw = None
    used_path = None
    for p in candidate_paths:
        if not p: continue
        try:
            with open(p, "r") as f:
                # Strip // comment-keys on the fly. Same lenient
                # convention as the C parser.
                text = f.read()
            raw = json.loads(_strip_jsonc_comments(text))
            used_path = p
            break
        except (OSError, ValueError) as e:
            print(f"[DSP] runtime config {p} not usable: {e}", flush=True)
            continue

    if raw is None:
        print(f"[DSP] WARNING: no runtime config loaded, using defaults",
              flush=True)
        return floor, ceil_, votes, behavior, grip_firm

    # Tolerant: missing fields keep their defaults.
    try:
        if "interp_conf_floor_pct" in raw:
            floor = float(raw["interp_conf_floor_pct"]) / 100.0
        if "interp_conf_ceil_pct" in raw:
            ceil_ = float(raw["interp_conf_ceil_pct"]) / 100.0
        if "hysteresis_votes" in raw:
            votes = int(raw["hysteresis_votes"])

        # Sanity: floor < ceil. Otherwise the lerp blows up.
        if floor >= ceil_:
            print(f"[DSP] WARNING: interp floor {floor:.2f} >= ceil "
                  f"{ceil_:.2f} from runtime.json — using defaults",
                  flush=True)
            floor, ceil_ = INTERP_CONF_FLOOR, INTERP_CONF_CEIL

        gv = raw.get("gesture_velocity", {})
        if not isinstance(gv, dict):
            print(f"[DSP] WARNING: gesture_velocity must be an object, "
                  f"got {type(gv).__name__} — ignoring", flush=True)
        else:
            for cls_name, rates in gv.items():
                if cls_name not in model_classes:
                    print(f"[DSP] gesture_velocity['{cls_name}'] not in "
                          f"model.classes_ — ignored", flush=True)
                    continue
                if not isinstance(rates, list) or len(rates) != NUM_SERVOS:
                    print(f"[DSP] gesture_velocity['{cls_name}']: "
                          f"expected list of {NUM_SERVOS}, ignoring",
                          flush=True)
                    continue
                # Range check — clamp loudly rather than silently.
                clean = []
                for r in rates:
                    try:
                        v = int(r)
                    except (TypeError, ValueError):
                        v = 0
                    # +/- 5000 us/s is already extreme; stay defensive.
                    if v < -5000 or v > 5000:
                        print(f"[DSP] gesture_velocity['{cls_name}']: "
                              f"rate {v} clamped to +/-5000", flush=True)
                        v = max(-5000, min(5000, v))
                    clean.append(v)
                # Velocity mode if any rate is non-zero, else freeze.
                mode = "velocity" if any(v != 0 for v in clean) else "freeze"
                behavior[cls_name] = {"mode": mode, "rate": clean}

        # gripper soft-firm clamp. dsp uses this in velocity
        # mode to prevent hand_flex from integrating past a safe
        # firm-hold position. Range-checked against the loader's
        # JSON validation (800..2200). Default 1100 from CFG_Defaults.
        if "grip_firm_us" in raw:
            v = int(raw["grip_firm_us"])
            if 800 <= v <= 2200:
                grip_firm = v
            else:
                print(f"[DSP] WARNING: grip_firm_us {v} out of "
                      f"range [800..2200], keeping default {grip_firm}",
                      flush=True)
    except Exception as e:
        print(f"[DSP] runtime config parse error: {e}, using defaults",
              flush=True)
        return (INTERP_CONF_FLOOR, INTERP_CONF_CEIL,
                CONFIRMATION_THRESHOLD, behavior, GRIP_FIRM_US_DEFAULT)

    print(f"[DSP] runtime config loaded from {used_path}: "
          f"floor={floor:.2f} ceil={ceil_:.2f} votes={votes} "
          f"grip_firm={grip_firm}", flush=True)
    for cls_name, beh in behavior.items():
        if beh["mode"] == "velocity":
            print(f"[DSP]   velocity-mode '{cls_name}': "
                  f"rate={beh['rate']}", flush=True)

    return floor, ceil_, votes, behavior, grip_firm


def _strip_jsonc_comments(text):
    """Strip // line comments so the C parser's lenient JSONC works
    in Python's strict json too.

    Naive line-by-line stripping breaks on lines like:
        "// schema_version": "REQUIRED..."
    where the // is INSIDE a string literal — we must not strip that.
    Walk the file character-by-character tracking whether we're inside
    a quoted string. Backslash-quote sequences inside strings are
    handled. Block comments (/* ... */) are also stripped for safety
    even though the project doesn't use them in JSON.
    """
    out = []
    i = 0
    n = len(text)
    in_str = False
    while i < n:
        c = text[i]
        if in_str:
            out.append(c)
            if c == "\\" and i + 1 < n:
                out.append(text[i+1])
                i += 2
                continue
            if c == '"':
                in_str = False
            i += 1
            continue
        # Not in a string.
        if c == '"':
            in_str = True
            out.append(c)
            i += 1
            continue
        if c == "/" and i + 1 < n and text[i+1] == "/":
            # Line comment — skip to newline (preserve the newline).
            while i < n and text[i] != "\n":
                i += 1
            continue
        if c == "/" and i + 1 < n and text[i+1] == "*":
            # Block comment.
            i += 2
            while i + 1 < n and not (text[i] == "*" and text[i+1] == "/"):
                i += 1
            i += 2
            continue
        out.append(c)
        i += 1

    text = "".join(out)
    # Drop trailing commas before } and ] (the runtime.json template
    # is well-formed but tolerate hand-edited mistakes).
    import re
    text = re.sub(r",(\s*[}\]])", r"\1", text)
    return text


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
    Decimates → DC-removes → BP → notch → envelope → 4 features.

    Filter band note: the bandpass arguments below (20.0, 450.0) match
    predict.py byte-for-byte. scipy auto-clamps the high cutoff to
    Nyquist*0.95 = 95 Hz at Fs=200, so the actual passband is 20-95 Hz.
    Training (feature_ex.py) used 15-90 Hz. The two filters produce
    features that match within ~0.2% RMS on real EMG; see the header
    docstring's TRAINING-VS-LIVE FILTER MISMATCH section for the full
    explanation."""
    # Anti-alias + downsample to 200 Hz (40 samples)
    window_lo           =   decimate(window_hi, DECIMATE_FACTOR, zero_phase=True)
    # DC offset removal (matches predict.py: data - np.mean)
    centered            =   window_lo - np.mean(window_lo)
    # Bandpass + notch (the 450 Hz cap auto-clamps to 95 Hz at Fs=200)
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
    Doesn't raise -- live drain must keep working even without ML.

    Searches in order:
      1. Legacy separate files: MODEL_PATH + SCALER_PATH (.joblib)
      2. Any *.pkl in MODEL_DIR — expected dict with 'model' + 'scaler' keys
    """
    try:
        import joblib
    except ImportError:
        print("[DSP] joblib not installed -- feature-only mode", flush=True)
        return None, None

    model               =   None
    scaler              =   None

    # Path 1: legacy separate .joblib files
    if os.path.exists(MODEL_PATH) and os.path.exists(SCALER_PATH):
        try:
            model       =   joblib.load(MODEL_PATH)
            scaler      =   joblib.load(SCALER_PATH)
            print(f"[DSP] loaded separate files: {MODEL_PATH}", flush=True)
        except Exception as e:
            print(f"[DSP] legacy load failed ({e}), trying .pkl", flush=True)
            model = scaler = None

    # Path 2: combined .pkl checkpoint (dict with 'model' + 'scaler')
    if model is None:
        import glob
        pkl_files       =   sorted(glob.glob(os.path.join(MODEL_DIR, "*.pkl")))
        for pkl_path in pkl_files:
            try:
                checkpoint  =   joblib.load(pkl_path)
                if isinstance(checkpoint, dict) and "model" in checkpoint and "scaler" in checkpoint:
                    model   =   checkpoint["model"]
                    scaler  =   checkpoint["scaler"]
                    print(f"[DSP] loaded combined .pkl: {pkl_path}", flush=True)
                    if "feature_names" in checkpoint:
                        print(f"[DSP]   feature_names={checkpoint['feature_names']}", flush=True)
                    break
                else:
                    print(f"[DSP] {pkl_path} not a dict with model+scaler, skipping", flush=True)
            except Exception as e:
                print(f"[DSP] {pkl_path} load failed ({e})", flush=True)

    if model is None or scaler is None:
        print(f"[DSP] no model found in {MODEL_DIR}/ -- feature-only mode", flush=True)
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

    # edit-mode handshake state
    edit_mode_seen      =   False

    # velocity-mode integrator state.
    # current_target_us[s] is the dsp's authoritative target for each
    # servo. cpcu_io's smoother trapezoidally walks toward whatever we
    # publish. Initialized to neutral on every fresh boot — there is
    # no snapshot persistence across runs, by design (a stuck pose
    # from yesterday's session shouldn't define today's startup).
    current_target_us   =   [SERVO_NEUTRAL_US] * NUM_SERVOS
    last_integrate_t    =   time.monotonic()
    last_safe_state     =   False        # for fault-recovery snap

    # Load runtime-config slice owned by dsp (interp thresholds,
    # hysteresis votes, gesture-velocity rows). model.classes_ tells
    # us which class names exist, so unknown JSON entries can be
    # warned about cleanly. If model load failed earlier,
    # model.classes_ is empty and behavior_map will only contain
    # "rest" — velocity mode is then a no-op (correct behaviour).
    if model is not None:
        model_class_names = [str(c) for c in model.classes_]
    else:
        model_class_names = ["rest"]
    interp_floor, interp_ceil, hysteresis_votes, behavior_map, grip_firm = \
        load_dsp_runtime_config(model_class_names)

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
                # publish envelope into IPC_DspFiltered for the
                # web dashboard. _env is ~40 samples @ TARGET_FS_HZ
                # (= 200 Hz). The bridge appends them to a 200-sample
                # rolling buffer (= 1 s of trailing envelope per channel),
                # which both the Waves tab (envelope plot) and the
                # Spectrum tab (browser-side FFT) consume. Cost is a
                # bytes-shift of 800 B per channel, negligible.
                try:
                    ipc.write_dsp_filtered_window(ch, _env, sample_rate_hz=TARGET_FS_HZ)
                except Exception as _ex:
                    # Don't let a publication error kill inference.
                    # (e.g. running against an old shm without the new
                    # region — we'd write past the mmap end.)
                    if buf_idx == 0:
                        print(f"[DSP] write_dsp_filtered_window failed: {_ex}",
                              flush=True)

            # ── 3. Inference + debounce (if model loaded) ──
            if inference_enabled:
                X       =   np.asarray(features_flat, dtype=np.float64).reshape(1, -1)
                Xs      =   scaler.transform(X)
                probs   =   model.predict_proba(Xs)[0]
                ai      =   int(np.argmax(probs))
                label   =   str(model.classes_[ai])
                conf    =   float(probs[ai])

                # Hysteresis: need `hysteresis_votes` consecutive
                # predictions above PROBABILITY_THRESHOLD to switch
                # state. Threshold is loaded from runtime.json on
                # startup (default 3, range 1-20).
                if conf > PROBABILITY_THRESHOLD and label != current_state:
                    consecutive_count  +=   1
                    if consecutive_count >= hysteresis_votes:
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

            # Edit-mode handshake.
            # If the TUI has requested edit mode, stop publishing motor
            # commands. cpcu_io ignores them anyway (sticky-park) but
            # not flooding IPC keeps the diagnostic tracelog clean.
            # We also commit the inference state to "rest" so any later
            # edit-mode-exit doesn't carry stale gesture history.
            # The dsp_ack byte tells the TUI we've seen the request.
            edit_req = ipc.read_edit_request()
            if edit_req:
                if not edit_mode_seen:
                    # First tick we noticed it — commit to rest, stop
                    # publishing, ack. Reset target to neutral so on
                    # exit we don't snap back to a held pose.
                    current_state    =   "rest"
                    consecutive_count =  0
                    last_active_class =  CLASS_REST
                    current_target_us =  [SERVO_NEUTRAL_US] * NUM_SERVOS
                    ipc.write_edit_dsp_ack(1)
                    edit_mode_seen   =   True
                    if verbose:
                        print("[DSP] edit-mode requested -> sticky-rest, "
                              "motor cmds suspended", flush=True)
                # Don't publish motor_cmd. cpcu_io is parking at neutral
                # via its own handshake responder.
                last_integrate_t  =   time.monotonic()  # avoid dt spike on exit
            else:
                if edit_mode_seen:
                    # Just exited edit mode. Clear ack so TUI sees us
                    # back on the bus. Resume normal motor publishing.
                    ipc.write_edit_dsp_ack(0)
                    edit_mode_seen   =   False
                    if verbose:
                        print("[DSP] edit-mode exited -> resuming", flush=True)

                # fault-recovery snap. If cpcu_io has been forced
                # SAFE since our last integration step, target snaps back
                # to neutral so we begin re-integration from a known pose
                # when the system recovers. Without this, a fault during
                # mid-flex would have us continuing to integrate from the
                # mid-flex target as soon as SAFE clears — surprising.
                sys_state = ipc.read_system_state()
                in_safe   = (sys_state == 2)        # IPC_STATE_SAFE
                if in_safe and not last_safe_state:
                    current_target_us = [SERVO_NEUTRAL_US] * NUM_SERVOS
                    if verbose:
                        print("[DSP] SAFE detected -> target snapped to neutral",
                              flush=True)
                last_safe_state = in_safe

                # Per-tick dt for the integrator. Inference cadence is
                # ~10 Hz (every WINDOW_HOP samples), so dt ≈ 0.1 s.
                # Exact value matters because rate is in us/s.
                now_t           =   time.monotonic()
                dt              =   now_t - last_integrate_t
                last_integrate_t =  now_t
                if dt > 0.5:    # we just stalled (paused, debugger), ignore
                    dt          =   0.0

                # Confidence scaling: linear lerp between floor and ceil.
                # Below floor -> 0 (no integration, hold).
                # Above ceil  -> 1 (full speed).
                # Linear ramp between.
                # conf is the SVM probability of `current_state`. If the
                # model is disabled, conf_pct stays 0 and scale = 0.
                conf_frac       =   conf_pct / 100.0
                if conf_frac <= interp_floor:
                    scale       =   0.0
                elif conf_frac >= interp_ceil:
                    scale       =   1.0
                else:
                    scale       =   (conf_frac - interp_floor) / \
                                    (interp_ceil - interp_floor)

                # Behaviour lookup: freeze (target unchanged) or
                # velocity (integrate target += rate * dt * scale).
                # Unknown class falls back to fixed-pose from
                # GESTURE_SERVO_MAP (preserves legacy behaviour).
                beh             =   behavior_map.get(current_state)
                if beh is None:
                    # Class predicted but not in our behaviour map.
                    # Use the legacy fixed pose if known.
                    fallback     =  GESTURE_SERVO_MAP.get(
                                        current_state,
                                        [SERVO_NEUTRAL_US]*NUM_SERVOS)
                    current_target_us = list(fallback)
                elif beh["mode"] == "freeze":
                    # Hold whatever target we currently have. For
                    # 'rest' specifically, snap to neutral so a long
                    # rest period drains us back to the home pose
                    # rather than holding the last gesture's target.
                    if current_state == "rest":
                        current_target_us = [SERVO_NEUTRAL_US]*NUM_SERVOS
                    # else: just hold (no-op).
                else:    # velocity mode
                    rates       =   beh["rate"]
                    for s in range(NUM_SERVOS):
                        delta   =   rates[s] * dt * scale
                        new_v   =   current_target_us[s] + delta
                        # Clamp to compile-time hardware limits. cpcu_io
                        # clamps again after applying servo_bias_us;
                        # this clamp is for dsp's own sanity.
                        new_v   =   max(SERVO_MIN_US[s],
                                        min(SERVO_MAX_US[s], new_v))
                        # gripper soft-firm clamp. The integrator
                        # is prevented from closing the gripper past the
                        # configured firm-hold position even if the
                        # gesture is held longer. ONE-SIDED — opening
                        # direction (target > grip_firm) is not affected,
                        # so hand_open still works normally. cpcu_io's
                        # stall watchdog is the lower-layer backstop;
                        # this is the policy layer.
                        if s == 5 and new_v < grip_firm:
                            new_v = grip_firm
                        current_target_us[s] = new_v

                # Publish the (possibly clamped, possibly integrated) target.
                # Round to int us for the IPC u16 field.
                servo_us        =   [int(round(v)) for v in current_target_us]
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
