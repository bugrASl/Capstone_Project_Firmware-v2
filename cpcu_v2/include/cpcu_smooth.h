/**
 *  @file       cpcu_smooth.h
 *  @brief      Per-servo trapezoidal motion profile smoother.
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    2.1
 *
 *  v2.1 (2026-04, CPCU v2.3.2):
 *      - Hold-pose deadband. Once a servo has settled at its target,
 *        further PCA writes are suppressed until the target changes
 *        beyond `hold_deadband_us` from the last-written value. This
 *        kills static jitter caused by the servo's internal control
 *        loop fighting backlash and gravity sag.
 *      - Per-servo `hold_deadband_us` (default 10 µs ≈ 0.9°). Set to
 *        0 to disable deadband for a specific servo.
 *      - New `last_written_us[]` shadow tracks what was last sent to
 *        the PCA, so the deadband test is "should-I-write-now"
 *        rather than "is-the-smoother-settled" (the two differ for
 *        a servo that's settled but hasn't yet been written once).
 *      - SMOOTH_ShouldWrite(ctx, ch) is the new query. cpcu_io.c gates
 *        its PCA_SetServo call on this. Motion always writes; settled
 *        servos write only when target moves outside the deadband.
 *      - SMOOTH_MarkWritten(ctx, ch) MUST be called by the consumer
 *        after each successful write, to update last_written_us[].
 *      - Full design: cpcu_v2/docs/JITTER_MITIGATION.md.
 *
 *  v2.0.1 (2026-04):
 *      - Added SMOOTH_SetSpeed as a static-inline alias for
 *        SMOOTH_SetVelocity. v1.0 callers (pca_testbench, etc.) keep
 *        compiling unchanged. New code should prefer SMOOTH_SetVelocity.
 *
 *  v2.0 (2026-04):
 *      - Trapezoidal velocity profile: accelerate to max_velocity, cruise,
 *        decelerate to a stop at the target. Replaces v1.0's constant-
 *        velocity slew which produced the "creeping then jolt" feel.
 *      - Per-servo `enabled` flag. When false, the smoother is a passthrough:
 *        target_us is written straight to current[] every tick. Used for
 *        the Gripper, where smoothing makes the grip-action sluggish.
 *      - Per-servo max_velocity AND max_accel, both in us/s and us/s².
 *      - SMOOTH_Snap retained for emergency neutral / init.
 *
 *  Profile geometry (one servo, single move):
 *
 *           v ↑
 *   max_v ─ │   ╭─────────────╮
 *           │  ╱               ╲
 *           │ ╱                 ╲
 *           │╱                   ╲
 *           └────────────────────────→ t
 *           ↑accel  cruise   decel↑
 *
 *  For short moves where the cruise phase doesn't fit, the profile
 *  collapses into a triangle and the peak velocity is whatever the
 *  symmetric accel/decel can reach in time.
 *
 *  Defaults (SMOOTH_Init):
 *      max_velocity     =   2000 us/s    (full 2000 us span in 1.0 s if cruising)
 *      max_accel        =   8000 us/s²   (reach max_velocity in 250 ms)
 *      enabled          =   true         (call SMOOTH_SetEnabled to bypass)
 *      hold_deadband_us =   10           (≈0.9°; 0 disables, see v2.1)
 */

#ifndef CPCU_SMOOTH_H
#define CPCU_SMOOTH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cpcu_pca9685.h"

#include <stdint.h>
#include <stdbool.h>

/*============= DEFAULTS ===================================================*/

#define SMOOTH_DEFAULT_VELOCITY     2000     /* us per second */
#define SMOOTH_DEFAULT_ACCEL        8000     /* us per second² */
#define SMOOTH_SETTLE_THRESH        2        /* within this distance = settled */
#define SMOOTH_DEFAULT_DEADBAND     10       /* v2.1: ≈0.9° hold-pose deadband */

/*============= CONTEXT ====================================================*/

typedef struct
{
    /* Live state per servo */
    float       current_f[PCA_SERVO_COUNT];     /* sub-us position */
    float       velocity_f[PCA_SERVO_COUNT];    /* signed, us/s */
    uint16_t    current[PCA_SERVO_COUNT];       /* integer pose for PCA */
    uint16_t    target[PCA_SERVO_COUNT];

    /* Configuration per servo */
    uint16_t    max_velocity[PCA_SERVO_COUNT];  /* us/s */
    uint16_t    max_accel[PCA_SERVO_COUNT];     /* us/s² */
    bool        enabled[PCA_SERVO_COUNT];       /* false = bypass smoother */

    /* v2.1: hold-pose deadband. See SMOOTH_ShouldWrite/MarkWritten. */
    uint16_t    hold_deadband_us[PCA_SERVO_COUNT];   /* 0 = disabled */
    uint16_t    last_written_us[PCA_SERVO_COUNT];    /* shadow of last PCA value */
    bool        ever_written[PCA_SERVO_COUNT];       /* false until first write */

    /* Status per servo */
    bool        settled[PCA_SERVO_COUNT];
} SMOOTH_Context;

/*============= API ========================================================*/

/*  Initialise all channels to start_us with the default profile and
 *  enabled=true. Call SMOOTH_SetEnabled afterwards to bypass specific
 *  channels (e.g. Gripper).                                              */
void SMOOTH_Init(SMOOTH_Context *ctx, uint16_t start_us);

/*  Configure one channel. */
void SMOOTH_SetEnabled (SMOOTH_Context *ctx, int channel, bool enabled);
void SMOOTH_SetVelocity(SMOOTH_Context *ctx, int channel, uint16_t v_us_per_s);
void SMOOTH_SetAccel   (SMOOTH_Context *ctx, int channel, uint16_t a_us_per_s2);

/*  v2.1: hold-pose deadband configuration.
 *  Once a servo settles at its target, SMOOTH_ShouldWrite() returns
 *  false until the target moves more than `deadband_us` from the
 *  last-written PCA value. Suppresses static jitter on cheap hobby
 *  servos. Pass 0 to disable. */
void SMOOTH_SetDeadband(SMOOTH_Context *ctx, int channel, uint16_t deadband_us);

/*  v1.0 compatibility shim — old code called this "SetSpeed". Maps to
 *  SetVelocity in v2.0. Keep using SMOOTH_SetVelocity in new code.      */
static inline void SMOOTH_SetSpeed(SMOOTH_Context *ctx, int channel,
                                   uint16_t speed_us_per_s)
{
    SMOOTH_SetVelocity(ctx, channel, speed_us_per_s);
}

/*  Target setting. */
void SMOOTH_SetTarget    (SMOOTH_Context *ctx, int channel, uint16_t target_us);
void SMOOTH_SetAllTargets(SMOOTH_Context *ctx, const uint16_t targets[PCA_SERVO_COUNT]);

/*  Advance positions by dt. dt_us = 20000 for 50 Hz tick. */
void SMOOTH_Update(SMOOTH_Context *ctx, uint32_t dt_us);

/*  Force every channel to its target now (zero velocity). */
void SMOOTH_Snap(SMOOTH_Context *ctx);

/*  v2.1: should the consumer issue a fresh PCA write for this channel?
 *  Returns true when:
 *      - the channel has never been written, OR
 *      - the smoother is not settled (motion in progress), OR
 *      - the smoother is settled but |current - last_written| exceeds the
 *        deadband (target moved enough to warrant an update).
 *  Returns false when the channel is settled at a value within the
 *  deadband of what the PCA already has — in that case, skipping the
 *  write avoids re-triggering the servo's internal correction loop. */
bool SMOOTH_ShouldWrite(const SMOOTH_Context *ctx, int channel);

/*  v2.1: caller MUST invoke this after each successful PCA write so
 *  the deadband logic knows what's currently latched in the hardware. */
void SMOOTH_MarkWritten(SMOOTH_Context *ctx, int channel, uint16_t written_us);

/*  Diagnostic */
bool SMOOTH_AllSettled(const SMOOTH_Context *ctx);

#ifdef __cplusplus
}
#endif

#endif /* CPCU_SMOOTH_H */
