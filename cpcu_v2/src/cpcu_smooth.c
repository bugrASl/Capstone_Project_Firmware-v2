/**
 *  @file       cpcu_smooth.c
 *  @brief      Per-servo trapezoidal motion profile smoother.
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    2.0
 */

#include "cpcu_smooth.h"

#include <math.h>

/*============= INIT =======================================================*/

void SMOOTH_Init(SMOOTH_Context *ctx, uint16_t start_us)
{
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        ctx->current_f[i]    = (float)start_us;
        ctx->velocity_f[i]   = 0.0f;
        ctx->current[i]      = start_us;
        ctx->target[i]       = start_us;
        ctx->max_velocity[i] = SMOOTH_DEFAULT_VELOCITY;
        ctx->max_accel[i]    = SMOOTH_DEFAULT_ACCEL;
        ctx->enabled[i]      = true;
        ctx->settled[i]      = true;
    }
}

/*============= CONFIG =====================================================*/

void SMOOTH_SetEnabled(SMOOTH_Context *ctx, int channel, bool enabled)
{
    if(channel < 0 || channel >= PCA_SERVO_COUNT) return;
    ctx->enabled[channel] = enabled;
    if(!enabled)
    {
        /* Bypassed: snap immediately so we don't hold a stale interim pose */
        ctx->current_f[channel]  = (float)ctx->target[channel];
        ctx->current[channel]    = ctx->target[channel];
        ctx->velocity_f[channel] = 0.0f;
        ctx->settled[channel]    = true;
    }
}

void SMOOTH_SetVelocity(SMOOTH_Context *ctx, int channel, uint16_t v)
{
    if(channel < 0 || channel >= PCA_SERVO_COUNT) return;
    ctx->max_velocity[channel] = v;
}

void SMOOTH_SetAccel(SMOOTH_Context *ctx, int channel, uint16_t a)
{
    if(channel < 0 || channel >= PCA_SERVO_COUNT) return;
    ctx->max_accel[channel] = (a > 0) ? a : 1;   /* avoid div-by-0 */
}

/*============= TARGET =====================================================*/

void SMOOTH_SetTarget(SMOOTH_Context *ctx, int channel, uint16_t target_us)
{
    if(channel < 0 || channel >= PCA_SERVO_COUNT) return;

    ctx->target[channel] = target_us;

    int diff = (int)target_us - (int)ctx->current[channel];
    if(diff < -SMOOTH_SETTLE_THRESH || diff > SMOOTH_SETTLE_THRESH)
        ctx->settled[channel] = false;
}

void SMOOTH_SetAllTargets(SMOOTH_Context *ctx,
                          const uint16_t targets[PCA_SERVO_COUNT])
{
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
        SMOOTH_SetTarget(ctx, i, targets[i]);
}

/*============= UPDATE — TRAPEZOIDAL PROFILE ===============================*/
/**
 *  Each tick, for each enabled servo:
 *
 *  1. dist  = target - current_f               (signed)
 *     |dist| / 2*a is the "stopping distance" if we decel from
 *     current velocity at max_accel.
 *  2. If we're inside or at the stopping distance OR very close to
 *     target → decelerate (|v| -= a*dt, clamp to ≥ 0).
 *  3. Else if |v| < max_v → accelerate (|v| += a*dt, clamp to max_v).
 *  4. Else → cruise (|v| unchanged).
 *
 *  Direction is the sign of `dist`. When dist crosses zero (overshoot
 *  guard) we snap to target and zero the velocity — settled.
 */
void SMOOTH_Update(SMOOTH_Context *ctx, uint32_t dt_us)
{
    const float dt_s = (float)dt_us * 1.0e-6f;

    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        /* Bypass: just write target straight through */
        if(!ctx->enabled[i])
        {
            ctx->current_f[i]   = (float)ctx->target[i];
            ctx->current[i]     = ctx->target[i];
            ctx->velocity_f[i]  = 0.0f;
            ctx->settled[i]     = true;
            continue;
        }

        if(ctx->settled[i] && ctx->current[i] == ctx->target[i])
            continue;

        float pos      = ctx->current_f[i];
        float vel      = ctx->velocity_f[i];
        float tgt      = (float)ctx->target[i];
        float dist     = tgt - pos;
        float dist_abs = fabsf(dist);
        float vel_abs  = fabsf(vel);

        float a        = (float)ctx->max_accel[i];
        float v_max    = (float)ctx->max_velocity[i];

        /* If sitting close enough, finish */
        if(dist_abs < (float)SMOOTH_SETTLE_THRESH && vel_abs < a * dt_s)
        {
            ctx->current_f[i]   = tgt;
            ctx->current[i]     = ctx->target[i];
            ctx->velocity_f[i]  = 0.0f;
            ctx->settled[i]     = true;
            continue;
        }

        /* Distance needed to stop from current velocity */
        float decel_dist = (vel_abs * vel_abs) / (2.0f * a);

        /* Phase decision */
        float new_vel_abs;
        if(dist_abs <= decel_dist)
        {
            /* Decelerate */
            new_vel_abs = vel_abs - a * dt_s;
            if(new_vel_abs < 0.0f) new_vel_abs = 0.0f;
        }
        else if(vel_abs < v_max)
        {
            /* Accelerate */
            new_vel_abs = vel_abs + a * dt_s;
            if(new_vel_abs > v_max) new_vel_abs = v_max;
        }
        else
        {
            /* Cruise */
            new_vel_abs = v_max;
        }

        /* Sign comes from current direction to target. If we somehow
         * crossed zero on the previous tick, the velocity sign needs
         * to flip — handled by the dist_abs sign check below. */
        float dir = (dist >= 0.0f) ? 1.0f : -1.0f;

        /* If we were moving the wrong way (rare: target changed mid-flight),
         * decelerate first before accelerating in the new direction. */
        if((vel > 0.0f && dist < 0.0f) || (vel < 0.0f && dist > 0.0f))
        {
            new_vel_abs = vel_abs - a * dt_s;
            if(new_vel_abs < 0.0f) new_vel_abs = 0.0f;
            dir = (vel > 0.0f) ? 1.0f : -1.0f;   /* keep current direction while braking */
        }

        float new_vel = new_vel_abs * dir;
        float new_pos = pos + new_vel * dt_s;

        /* Overshoot guard */
        if((dist > 0.0f && new_pos > tgt) || (dist < 0.0f && new_pos < tgt))
        {
            new_pos = tgt;
            new_vel = 0.0f;
            ctx->settled[i] = true;
        }
        else
        {
            ctx->settled[i] = false;
        }

        ctx->current_f[i]   = new_pos;
        ctx->velocity_f[i]  = new_vel;
        ctx->current[i]     = (uint16_t)(new_pos + 0.5f);
    }
}

/*============= SNAP =======================================================*/

void SMOOTH_Snap(SMOOTH_Context *ctx)
{
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        ctx->current_f[i]   = (float)ctx->target[i];
        ctx->current[i]     = ctx->target[i];
        ctx->velocity_f[i]  = 0.0f;
        ctx->settled[i]     = true;
    }
}

/*============= QUERY ======================================================*/

bool SMOOTH_AllSettled(const SMOOTH_Context *ctx)
{
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
        if(!ctx->settled[i]) return false;
    return true;
}
