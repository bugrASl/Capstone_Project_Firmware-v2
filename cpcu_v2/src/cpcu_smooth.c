/**
 *  @file       cpcu_smooth.c
 *  @brief      Non-blocking servo slew rate limiter
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    1.0
 */

#include "cpcu_smooth.h"

#include <math.h>

/*============= INIT =====================================================*/

void SMOOTH_Init(SMOOTH_Context *ctx, uint16_t start_us)
{
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        ctx->current_f[i]   = (float)start_us;
        ctx->current[i]     = start_us;
        ctx->target[i]      = start_us;
        ctx->max_speed[i]   = SMOOTH_DEFAULT_SPEED;
        ctx->settled[i]     = true;
    }
}

/*============= TARGET SETTING ===========================================*/

void SMOOTH_SetTarget(SMOOTH_Context *ctx, int channel, uint16_t target_us)
{
    if(channel < 0 || channel >= PCA_SERVO_COUNT) return;

    ctx->target[channel]    = target_us;
    
    int diff = (int)target_us - (int)ctx->current[channel];
    if(diff < -SMOOTH_SETTLE_THRESH || diff > SMOOTH_SETTLE_THRESH)
    {
        ctx->settled[channel] = false;
    }
}

void SMOOTH_SetAllTargets(SMOOTH_Context *ctx, const uint16_t targets[PCA_SERVO_COUNT])
{
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        SMOOTH_SetTarget(ctx, i, targets[i]);
    }
}

/*============= SPEED ====================================================*/

void SMOOTH_SetSpeed(SMOOTH_Context *ctx, int channel, uint16_t speed_us_per_s)
{
    if(channel < 0 || channel >= PCA_SERVO_COUNT) return;
    ctx->max_speed[channel] = speed_us_per_s;
}

/*============= UPDATE ===================================================*/

void SMOOTH_Update(SMOOTH_Context *ctx, uint32_t dt_us)
{
    float dt_s = (float)dt_us / 1000000.0f;

    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        if(ctx->settled[i]) continue;

        float target    = (float)ctx->target[i];
        float current   = ctx->current_f[i];
        float diff      = target - current;
        float max_step  = (float)ctx->max_speed[i] * dt_s;

        if(fabsf(diff) <= max_step)
        {
            /* Close enough — snap to target */
            ctx->current_f[i]   = target;
            ctx->current[i]     = ctx->target[i];
            ctx->settled[i]     = true;
        }
        else
        {
            /* Move toward target by max_step */
            float step          = (diff > 0.0f) ? max_step : -max_step;
            ctx->current_f[i]   += step;
            ctx->current[i]     = (uint16_t)(ctx->current_f[i] + 0.5f);
        }
    }
}

/*============= QUERY ====================================================*/

bool SMOOTH_AllSettled(const SMOOTH_Context *ctx)
{
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        if(!ctx->settled[i]) return false;
    }
    return true;
}

/*============= SNAP =====================================================*/

void SMOOTH_Snap(SMOOTH_Context *ctx)
{
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        ctx->current_f[i]   = (float)ctx->target[i];
        ctx->current[i]     = ctx->target[i];
        ctx->settled[i]     = true;
    }
}

/*========================================================================*/
