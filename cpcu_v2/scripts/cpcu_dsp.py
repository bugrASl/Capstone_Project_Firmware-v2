################################################################################
#  cpcu_dsp.py  PATCH  v1.1  ──────────────────────────────────────────────────
#
#  The only change to cpcu_dsp.py is the misleading comment header above
#  GESTURE_SERVO_MAP. The map values themselves are NOT touched — those
#  are calibration data the team already chose for each gesture and should
#  be re-tuned on the hardware separately, with the new logical->physical
#  mapping in place.
#
#  FIND this block (around line 75-78):
#
#      # ══════════════════════════════════════════════════════════════════════
#      #  GESTURE → SERVO MAPPING
#      # ══════════════════════════════════════════════════════════════════════
#      # Servo indices: S0=Thumb S1=Index S2=Middle S3=Ring S4=Pinky S5=Wrist
#      # Values in microseconds. PCA driver clamps to per-joint min/max.
#
#  REPLACE with:
################################################################################

# ══════════════════════════════════════════════════════════════════════
#  GESTURE → SERVO MAPPING
# ══════════════════════════════════════════════════════════════════════
# Logical servo indices match cpcu_pca9685.h v1.1:
#   S0 = Base       (MG995, PCA terminal 0,   498-2500 us)
#   S1 = Upper      (MG995, PCA terminal 1,  1074-1953 us)
#   S2 = Last       (MG995, PCA terminal 11, 1074-1953 us)
#   S3 = Joint-1    (SG90,  PCA terminal 8,  1001-2002 us)
#   S4 = Joint-2    (SG90,  PCA terminal 5,  1001-2002 us)
#   S5 = Gripper    (SG90,  PCA terminal 4,   976-1733 us)
#
# Values in microseconds. The PCA9685 driver clamps to per-joint min/max
# at write-time; values that fall outside the range above will be silently
# saturated to the nearest limit.
#
# NOTE: The numeric values below were calibrated for the v1.0 driver
# (which assumed servos on contiguous channels 0..5). With the new
# logical->physical mapping, the SAME logical indices reach the SAME
# motors, so these values continue to make sense. But the gesture
# semantics for this 6-joint arm (Base/Upper/Last/Joint-1/Joint-2/Gripper)
# may want re-tuning — the original author was thinking in terms of
# fingers (Thumb/Index/Middle/Ring/Pinky/Wrist), and "all S0..S4 to
# 1700 us" doesn't translate cleanly to a kinematic-arm gesture. Re-
# calibrate on the bench once the channel mapping is verified.
