/**
 *  @file       cpcu_safety.c
 *  @brief      System-wide safety monitor implementation, v2.2.
 *  @author     bugrASl
 *  @date       April 2026
 *
 *  v2.2 changes (2026-04):
 *      - Replaced SAFETY_TryRecover with SAFETY_UpdateState — handles
 *        BOTH transitions into and out of RADIO_SAFE in one place.
 *        Non-radio non-battery causes (thermal, dsp, i2c, ring) now
 *        explicitly drive the FSM into SAFE (previously they only
 *        gated CheckSystem without changing state, which was confusing).
 *      - SAFE-exit logic no longer requires last_pkt_rcv_us to be fresh
 *        (was over-zealous; if radio recovered before SAFE entry, this
 *        was double-checking the same thing). Now it just checks the
 *        actual fault flags.
 *
 *  Caller contract: see header. The most important parts:
 *      - cpcu_io must call FeedI2C from BOTH the success path AND the
 *        else-branch fallback (where it writes neutrals via PCA), or
 *        i2c.faulted will never clear once tripped.
 *      - cpcu_io must call FeedMotorCMD whenever IPC_ReadMotorCmd
 *        returns true, regardless of safety state. Otherwise dsp.stalled
 *        latches and SAFE recovery cannot succeed.
 */

#include "cpcu_safety.h"

#include <string.h>
#include <stdio.h>

/*============= INIT =======================================================*/

void SAFETY_Init(SAFETY_Context *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->state          =   RADIO_INIT;
    ctx->last_fault     =   SAFETY_OK;
}

/*============= SEQUENCE GAP ===============================================*/

uint32_t SAFETY_SeqGap(SAFETY_Context *ctx, uint8_t seq)
{
    if(!ctx->seq_init)
    {
        ctx->expected_seq   =   seq;
        ctx->seq_init       =   true;
        return 0;
    }
    uint8_t gap         =   (seq - ctx->expected_seq) & 0xFF;
    ctx->expected_seq   =   (seq + 1) & 0xFF;
    return (gap <= 1) ? 0 : gap;
}

/*============= LINK FEED ==================================================*/

static void link_feed(LINK_Stats *l, const WL_Packet *pkt, uint32_t gap,
                      uint64_t now_us)
{
    l->w_packets++;
    l->w_gaps       +=  gap;
    l->w_retry_sum  +=  pkt->tx_retry;

    if(l->w_packets >= SAFETY_LINK_WINDOW)
    {
        l->mean_retry           =   (float)l->w_retry_sum / (float)l->w_packets;
        uint32_t total_expected =   l->w_packets + l->w_gaps;
        l->loss_rate            =   (total_expected > 0)
                                     ? (float)l->w_gaps / (float)total_expected
                                     : 0.0f;

        LINK_Quality prev_q = l->quality;
        if(l->mean_retry < SAFETY_LINK_RETRY_GOOD &&
           l->loss_rate < SAFETY_LINK_LOSS_GOOD)
            l->quality = LINK_GOOD;
        else if(l->mean_retry < SAFETY_LINK_RETRY_DEGRADED &&
                l->loss_rate < SAFETY_LINK_LOSS_DEGRADED)
            l->quality = LINK_DEGRADED;
        else
            l->quality = LINK_POOR;

        if(l->quality == LINK_GOOD)
        {
            if(prev_q != LINK_GOOD || l->good_since_us == 0)
                l->good_since_us = now_us;
        }
        else
        {
            l->good_since_us = 0;
        }

        l->w_packets    =   0;
        l->w_gaps       =   0;
        l->w_retry_sum  =   0;
    }
}

/*============= FEED PACKET ================================================*/

void SAFETY_FeedPacket(SAFETY_Context *ctx, const WL_Packet *pkt,
                       uint64_t now_us)
{
    if(pkt->flags & WL_FLAG_FIRST_PACKET)
    {
        ctx->expected_seq   =   pkt->seq;
        ctx->seq_init       =   true;
    }

    uint32_t gap            =   SAFETY_SeqGap(ctx, pkt->seq);
    link_feed(&ctx->link, pkt, gap, now_us);

    /* Battery with hysteresis */
    ctx->battery.raw        =   pkt->vbat_raw;
    ctx->battery.voltage    =   pkt->vbat_raw * (3.3f / 4095.0f) * SAFETY_VBAT_DIVIDER;
    ctx->battery.level      =   WL_BATT_GET(pkt->flags);

    if(!ctx->battery.critical)
    {
        ctx->battery.critical = (ctx->battery.level == WL_BATT_CRIT)
                              || (ctx->battery.voltage < SAFETY_VBAT_CRITICAL_V
                                  && ctx->battery.voltage > 0.1f);
        if(ctx->battery.critical)
            ctx->last_fault = SAFETY_ERR_BATTERY;
    }
    else if(ctx->battery.voltage > SAFETY_VBAT_RECOVER_V &&
            ctx->battery.level != WL_BATT_CRIT)
    {
        ctx->battery.critical = false;
    }

    ctx->last_pkt_rcv_us    =   now_us;

    /* Radio FSM (battery + thermal + others handled in UpdateState) */
    switch(ctx->state)
    {
        case RADIO_INIT:
            ctx->state          =   RADIO_RUNNING;
            ctx->last_fault     =   SAFETY_OK;
            break;

        case RADIO_DEGRADED:
            ctx->state          =   RADIO_RECOVERING;
            ctx->recovery_cnt   =   (gap == 0) ? 1 : 0;
            break;

        case RADIO_RECOVERING:
            if(gap == 0 && ++ctx->recovery_cnt >= RECOVERY_PKT_COUNT)
            {
                ctx->state          =   RADIO_RUNNING;
                ctx->last_fault     =   SAFETY_OK;
            }
            else if(gap > 0)
            {
                ctx->recovery_cnt   =   0;
            }
            break;

        case RADIO_RUNNING:
        case RADIO_SAFE:
            /* Other transitions (battery, thermal, dsp, i2c, ring) are
             * driven from UpdateState rather than here. */
            break;
    }
}

/*============= TIMEOUT CHECK ==============================================*/

void SAFETY_CheckTimeout(SAFETY_Context *ctx, uint64_t now_us)
{
    if(ctx->state == RADIO_INIT) return;
    if(ctx->state == RADIO_SAFE) return;     /* recovery handled by UpdateState */

    uint64_t silence_ms = (now_us - ctx->last_pkt_rcv_us) / 1000;

    if((ctx->state == RADIO_RUNNING || ctx->state == RADIO_RECOVERING) &&
       silence_ms > SAFETY_RADIO_TIMEOUT_MS)
    {
        ctx->state              =   RADIO_DEGRADED;
        ctx->degraded_entry_us  =   now_us;
        ctx->recovery_cnt       =   0;
        ctx->last_fault         =   SAFETY_ERR_RADIO;
    }
    else if(ctx->state == RADIO_DEGRADED &&
            (now_us - ctx->degraded_entry_us) / 1000 > SAFETY_RADIO_SAFE_MS)
    {
        ctx->state              =   RADIO_SAFE;
        ctx->last_fault         =   SAFETY_ERR_RADIO;
        ctx->safe_entry_us      =   now_us;
        ctx->safe_clear_since_us = 0;
    }
}

/*============= DSP HEALTH =================================================*/

void SAFETY_FeedMotorCMD(SAFETY_Context *ctx, uint64_t now_us)
{
    ctx->dsp.last_cmd_us    =   now_us;
    ctx->dsp.active         =   true;
    ctx->dsp.stalled        =   false;
}

void SAFETY_CheckDSP(SAFETY_Context *ctx, uint64_t now_us)
{
    if(!ctx->dsp.active) return;

    uint64_t ms = (now_us - ctx->dsp.last_cmd_us) / 1000;
    bool was_stalled = ctx->dsp.stalled;
    ctx->dsp.stalled = (ms > SAFETY_DSP_STALL_MS);

    if(ctx->dsp.stalled && !was_stalled)
        ctx->last_fault = SAFETY_ERR_DSP_STALL;
}

/*============= I2C ========================================================*/

void SAFETY_FeedI2C(SAFETY_Context *ctx, bool success)
{
    if(success)
    {
        ctx->i2c.consecutive_errors =   0;
        ctx->i2c.faulted            =   false;     /* clears on first success */
    }
    else
    {
        ctx->i2c.consecutive_errors++;
        if(ctx->i2c.consecutive_errors >= SAFETY_I2C_MAX_ERRORS &&
           !ctx->i2c.faulted)
        {
            ctx->i2c.faulted        =   true;
            ctx->last_fault         =   SAFETY_ERR_I2C_BUS;
        }
    }
}

/*============= RING =======================================================*/

void SAFETY_FeedRingOverflow(SAFETY_Context *ctx, uint32_t overflow_count)
{
    ctx->ring.overflow_count    =   overflow_count;
    bool was_faulted            =   ctx->ring.faulted;
    ctx->ring.faulted           =   (overflow_count > SAFETY_RING_OVERFLOW_LIMIT);

    if(ctx->ring.faulted && !was_faulted)
        ctx->last_fault         =   SAFETY_ERR_RING_OVF;
}

/*============= THERMAL ====================================================*/

void SAFETY_FeedTemperature(SAFETY_Context *ctx, float temp_c)
{
    ctx->thermal.temperature_c  =   temp_c;
    ctx->thermal.warning        =   (temp_c > (float)SAFETY_THERMAL_WARN_C);

    if(!ctx->thermal.critical)
    {
        ctx->thermal.critical   =   (temp_c > (float)SAFETY_THERMAL_CRITICAL_C);
        if(ctx->thermal.critical)
            ctx->last_fault     =   SAFETY_ERR_THERMAL;
    }
    else if(temp_c < (float)SAFETY_THERMAL_RECOVER_C)
    {
        ctx->thermal.critical   =   false;
    }
}

/*============= DECISION (read-only) =======================================*/

bool SAFETY_CheckSystem(const SAFETY_Context *ctx)
{
    if(ctx->state != RADIO_RUNNING) return false;
    if(ctx->battery.critical)       return false;
    if(ctx->dsp.stalled)            return false;
    if(ctx->i2c.faulted)            return false;
    if(ctx->thermal.critical)       return false;
    if(ctx->ring.faulted)           return false;
    return true;
}

/*============= STATE TRANSITIONS (v2.2) ===================================*/
/**
 *  Single function that owns FSM transitions for non-radio-timing causes.
 *
 *  Entry to SAFE:
 *      If we're in RUNNING and any of {battery.critical, dsp.stalled,
 *      i2c.faulted, thermal.critical, ring.faulted} is true, transition
 *      to SAFE and record which.
 *
 *  Exit from SAFE:
 *      All five fault flags must be clear. Once they are, start the
 *      stable-clear timer; after SAFETY_SAFE_RECOVER_MS, transition out
 *      via RECOVERING (if the original cause was the radio link) or
 *      directly to RUNNING.
 */
void SAFETY_UpdateState(SAFETY_Context *ctx, uint64_t now_us)
{
    bool any_fault = ctx->battery.critical ||
                     ctx->dsp.stalled       ||
                     ctx->i2c.faulted       ||
                     ctx->thermal.critical  ||
                     ctx->ring.faulted;

    if(ctx->state == RADIO_RUNNING && any_fault)
    {
        /* Promote which one is "the" cause for telemetry */
        if(ctx->battery.critical)       ctx->last_fault = SAFETY_ERR_BATTERY;
        else if(ctx->thermal.critical)  ctx->last_fault = SAFETY_ERR_THERMAL;
        else if(ctx->dsp.stalled)       ctx->last_fault = SAFETY_ERR_DSP_STALL;
        else if(ctx->i2c.faulted)       ctx->last_fault = SAFETY_ERR_I2C_BUS;
        else if(ctx->ring.faulted)      ctx->last_fault = SAFETY_ERR_RING_OVF;

        ctx->state              =   RADIO_SAFE;
        ctx->safe_entry_us      =   now_us;
        ctx->safe_clear_since_us = 0;
        return;
    }

    if(ctx->state != RADIO_SAFE) return;

    /* In SAFE: try to recover */
    if(any_fault)
    {
        ctx->safe_clear_since_us = 0;
        return;
    }

    /* All flags clear — start or check the stable timer */
    if(ctx->safe_clear_since_us == 0)
    {
        ctx->safe_clear_since_us = now_us;
        return;
    }

    if((now_us - ctx->safe_clear_since_us) / 1000 < SAFETY_SAFE_RECOVER_MS)
        return;

    /* Stable for required hold time — exit SAFE */
    if(ctx->last_fault == SAFETY_ERR_RADIO)
    {
        ctx->state              =   RADIO_RECOVERING;
        ctx->recovery_cnt       =   0;
    }
    else
    {
        ctx->state              =   RADIO_RUNNING;
        ctx->last_fault         =   SAFETY_OK;
    }
    ctx->safe_clear_since_us    =   0;
}

/*============= AUTO-CLEAR =================================================*/

bool SAFETY_ShouldAutoClear(SAFETY_Context *ctx, uint64_t now_us)
{
    if(ctx->link.quality != LINK_GOOD) return false;
    if(ctx->link.good_since_us == 0)   return false;

    uint64_t good_ms = (now_us - ctx->link.good_since_us) / 1000;
    if(good_ms >= SAFETY_AUTO_CLEAR_MS)
    {
        ctx->link.good_since_us = now_us;
        return true;
    }
    return false;
}

/*============= STRINGS ====================================================*/

const char *SAFETY_StatusStr(SAFETY_Status s)
{
    switch(s)
    {
        case SAFETY_OK:             return "OK";
        case SAFETY_ERR_RADIO:      return "RADIO_LOST";
        case SAFETY_ERR_BATTERY:    return "BATT_CRITICAL";
        case SAFETY_ERR_DSP_STALL:  return "DSP_STALLED";
        case SAFETY_ERR_NRF_HW:     return "NRF_HW_FAIL";
        case SAFETY_ERR_I2C_BUS:    return "I2C_BUS_FAIL";
        case SAFETY_ERR_RING_OVF:   return "RING_OVERFLOW";
        case SAFETY_ERR_THERMAL:    return "OVERTEMP";
        default:                    return "UNKNOWN";
    }
}

const char *SAFETY_RadioStr(RADIO_State s)
{
    switch(s)
    {
        case RADIO_INIT:        return "INIT";
        case RADIO_RUNNING:     return "RUNNING";
        case RADIO_DEGRADED:    return "DEGRADED";
        case RADIO_RECOVERING:  return "RECOVERING";
        case RADIO_SAFE:        return "SAFE";
        default:                return "???";
    }
}
