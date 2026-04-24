/**
 *  @file       cpcu_smooth.h
 *  @brief      Non-blocking servo slew rate limiter
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    1.0
 *
 *  @details    Limits the rate of change of servo pulse widths to prevent
 *              mechanical shock and jerk. Runs inside the 50 Hz servo update
 *              cycle in cpcu_io.c — no blocking, no delays.
 *
 *              Algorithm: Each call, the current position moves toward the
 *              target by at most (max_speed_us_per_s * dt) microseconds.
 *              This gives smooth, predictable motion regardless of how
 *              large the commanded step is.
 *
 *              Default slew rate: 2000 us/s
 *                  Full range (500-2500 us = 2000 us) in 1.0 second
 *                  Per 50 Hz tick (20 ms): max step = 40 us
 *
 *              The smoother also provides a "settled" flag per channel,
 *              indicating when the servo has reached its target. This is
 *              useful for sequencing multi-joint motions.
 *
 *  Usage:
 *      SMOOTH_Context sm;
 *      SMOOTH_Init(&sm, 1500);             // Start all at neutral
 *      SMOOTH_SetTarget(&sm, 0, 900);      // Command servo 0 to 900 us
 *      // In 50 Hz loop:
 *      SMOOTH_Update(&sm, dt_us);          // Advance positions
 *      uint16_t *out = sm.current;         // Use sm.current[] for PCA write
 */

#ifndef CPCU_SMOOTH_H
#define CPCU_SMOOTH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cpcu_pca9685.h"

#include <stdint.h>
#include <stdbool.h>

/*============= CONFIGURATION ============================*/

#define SMOOTH_DEFAULT_SPEED    2000    /* us per second (full range in 1.0s) */
#define SMOOTH_SETTLE_THRESH    2       /* Within this many us = "settled" */

/*============= CONTEXT ==================================*/

typedef struct
{
    float       current_f[PCA_SERVO_COUNT];     /* Fractional position (us, float for smooth sub-step) */
    uint16_t    current[PCA_SERVO_COUNT];        /* Integer position sent to PCA (us) */
    uint16_t    target[PCA_SERVO_COUNT];         /* Commanded target (us) */
    uint16_t    max_speed[PCA_SERVO_COUNT];      /* Max slew rate per channel (us/s) */
    bool        settled[PCA_SERVO_COUNT];         /* True when current ~= target */
} SMOOTH_Context;

/*============= API ======================================*/

/**
 *  Initialize all channels to `start_us` with default slew rate.
 */
void SMOOTH_Init(SMOOTH_Context *ctx, uint16_t start_us);

/**
 *  Set a new target for one channel.
 *  The smoother will ramp toward it over subsequent Update() calls.
 */
void SMOOTH_SetTarget(SMOOTH_Context *ctx, int channel, uint16_t target_us);

/**
 *  Set targets for all channels at once.
 */
void SMOOTH_SetAllTargets(SMOOTH_Context *ctx, const uint16_t targets[PCA_SERVO_COUNT]);

/**
 *  Set the slew rate for a specific channel (us per second).
 */
void SMOOTH_SetSpeed(SMOOTH_Context *ctx, int channel, uint16_t speed_us_per_s);

/**
 *  Advance all channels toward their targets by one time step.
 *  Call this at 50 Hz with dt_us = 20000.
 */
void SMOOTH_Update(SMOOTH_Context *ctx, uint32_t dt_us);

/**
 *  Check if all channels have reached their targets.
 */
bool SMOOTH_AllSettled(const SMOOTH_Context *ctx);

/**
 *  Force all channels to jump immediately to their targets (bypass smoothing).
 *  Use for emergency neutral or init.
 */
void SMOOTH_Snap(SMOOTH_Context *ctx);

#ifdef __cplusplus
}
#endif

#endif /* CPCU_SMOOTH_H */
