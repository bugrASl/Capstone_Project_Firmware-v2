/**
 *  @file       cpcu_safety.c
 *  @brief      System-wide safety monitor implementation.
 *  @author     bugrASl
 *  @date       13.04.2026
 *  @version    2.0
 *  @details    Seven fault sources, one decision.
 *              Every function takes SAFETY_Context* — no globals.
 */

#include "cpcu_safety.h"

#include <string.h>
#include <stdio.h>

/*============= SAFETY_Init ================================================================*/

void SAFETY_Init(SAFETY_Context *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->state          =   RADIO_INIT;
    ctx->last_fault     =   SAFETY_OK;
    ctx->dsp.active     =   false;
    ctx->dsp.stalled    =   false;
}

/*============= SAFETY_SeqGap ==============================================================*/
/**
 *  @brief      Wrapping uint8 subtraction: (received - expected) & 0xFF.
 *              First call initializes expected_seq and returns 0.
 */

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

/*============= link_feed (static) =========================================================*/
/**
 *  @brief      Rolling window link quality computation.
 *
 *              Accumulates packets, gaps, and retries over SAFETY_LINK_WINDOW packets.
 *              At the window boundary, computes mean_retry and loss_rate, classifies
 *              quality, then resets accumulators for the next window.
 */

static void link_feed(LINK_Stats *l, const WL_Packet *pkt, uint32_t gap)
{
    l->w_packets++;
    l->w_gaps       +=  gap;
    l->w_retry_sum  +=  pkt->tx_retry;

    if(l->w_packets >= SAFETY_LINK_WINDOW)
    {
        /* Compute statistics */
        l->mean_retry               =   (float)l->w_retry_sum / (float)l->w_packets;
        uint32_t total_expected     =   l->w_packets + l->w_gaps;
        l->loss_rate                =   (total_expected > 0)
                                        ? (float)l->w_gaps / (float)total_expected
                                        : 0.0f;

        /* Classify quality */
        if(l->mean_retry < SAFETY_LINK_RETRY_GOOD && l->loss_rate < SAFETY_LINK_LOSS_GOOD)
        {
            l->quality              =   LINK_GOOD;
        }
        else if(l->mean_retry < SAFETY_LINK_RETRY_DEGRADED && l->loss_rate < SAFETY_LINK_LOSS_DEGRADED)
        {
            l->quality              =   LINK_DEGRADED;
        }
        else
        {
            l->quality              =   LINK_POOR;
        }

        /* Reset accumulators */
        l->w_packets        =   0;
        l->w_gaps           =   0;
        l->w_retry_sum      =   0;
    }
}

/*============= SAFETY_FeedPacket ==========================================================*/
/**
 *  @brief      Called for every received packet (~1000/s). 
 *              Does three things:
 *                  1. Update diagnostics (seq gap, link stats, battery)
 *                  2. Record reception timestamp
 *                  3. Advance radio state machine
 */

void SAFETY_FeedPacket(SAFETY_Context *ctx, const WL_Packet *pkt, uint64_t now_us)
{
    /* Handle FIRST_PACKET flag: BSAU just booted, reset seq tracking */
    if(pkt->flags & WL_FLAG_FIRST_PACKET)
    {
        ctx->expected_seq   =   pkt->seq;
        ctx->seq_init       =   true;
    }

    /* Sequence gap detection */
    uint32_t gap    =   SAFETY_SeqGap(ctx, pkt->seq);

    /* Link quality accumulation */
    link_feed(&ctx->link, pkt, gap);

    /* Battery state reconstruction */
    ctx->battery.raw        =   pkt->vbat_raw;
    ctx->battery.voltage    =   pkt->vbat_raw * (3.3f / 4095.0f) * SAFETY_VBAT_DIVIDER;
    ctx->battery.level      =   WL_BATT_GET(pkt->flags);
    ctx->battery.critical   =   (ctx->battery.level == WL_BATT_CRIT)
                                || (ctx->battery.voltage < SAFETY_VBAT_CRITICAL_V
                                && ctx->battery.voltage > 0.1f);

    /* Record timestamp */
    ctx->last_pkt_rcv_us    =   now_us;

    /* Radio state machine */
    switch(ctx->state)
    {
        case RADIO_INIT:
            /* First valid packet -> link is alive */
            ctx->state          =   RADIO_RUNNING;
            ctx->last_fault     =   SAFETY_OK;
            break;

        case RADIO_DEGRADED:
            /* Packet arrived after silence -> start recovery */
            ctx->state          =   RADIO_RECOVERING;
            ctx->recovery_cnt   =   (gap == 0) ? 1 : 0;
            break;

        case RADIO_RECOVERING:
            /* Count consecutive good packets */
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
            /* Battery critical -> force SAFE */
            if(ctx->battery.critical)
            {
                ctx->state          =   RADIO_SAFE;
                ctx->last_fault     =   SAFETY_ERR_BATTERY;
            }
            break;

        case RADIO_SAFE:
            /* Terminal state: ignore packets */
            break;
    }
}

/*============= SAFETY_CheckTimeout ========================================================*/
/**
 *  @brief      Called every loop iteration, even when no packet arrived.
 *
 *              Two-tier timeout:
 *                  Tier 1: 
 *                      RUNNING/RECOVERING + 750ms silence -> DEGRADED
 *                  Tier 2: 
 *                      DEGRADED for 1500ms total -> SAFE (terminal)
 */

void SAFETY_CheckTimeout(SAFETY_Context *ctx, uint64_t now_us)
{
    /* Don't timeout if already SAFE or never started */
    if(ctx->state == RADIO_SAFE || ctx->state == RADIO_INIT)
    {
        return;
    }

    uint64_t silence_ms =   (now_us - ctx->last_pkt_rcv_us) / 1000;

    if( (ctx->state == RADIO_RUNNING || ctx->state == RADIO_RECOVERING)
        && silence_ms > SAFETY_RADIO_TIMEOUT_MS )
    {
        /* Tier 1: silence detected */
        ctx->state                  =   RADIO_DEGRADED;
        ctx->degraded_entry_us      =   now_us;
        ctx->recovery_cnt           =   0;
        ctx->last_fault             =   SAFETY_ERR_RADIO;
    }
    else if( ctx->state == RADIO_DEGRADED
             && (now_us - ctx->degraded_entry_us) / 1000 > SAFETY_RADIO_SAFE_MS )
    {
        /* Tier 2: prolonged degradation -> terminal SAFE */
        ctx->state          =   RADIO_SAFE;
        ctx->last_fault     =   SAFETY_ERR_RADIO;
    }
}

/*============= SAFETY_FeedMotorCMD ========================================================*/
/**
 *  @brief      Notify safety that a new motor command was written by DSP.
 */

void SAFETY_FeedMotorCMD(SAFETY_Context *ctx, uint64_t now_us)
{
    ctx->dsp.last_cmd_us    =   now_us;
    ctx->dsp.active         =   true;
    ctx->dsp.stalled        =   false;
}

/*============= SAFETY_CheckDSP ============================================================*/
/**
 *  @brief      If the DSP pipeline hasn't produced a motor command in SAFETY_DSP_STALL_MS,
 *              something is wrong (Python crashed, GC infinite loop, model load failure).
 *
 *              The radio link may still be fine, so we don't change the radio state.
 *              Instead we set dsp.stalled which SAFETY_CheckSystem reads.
 */

void SAFETY_CheckDSP(SAFETY_Context *ctx, uint64_t now_us)
{
    /* DSP hasn't started yet — don't fault on startup delay */
    if(!ctx->dsp.active)
    {
        return;
    }

    uint64_t ms =   (now_us - ctx->dsp.last_cmd_us) / 1000;

    if(ms > SAFETY_DSP_STALL_MS)
    {
        ctx->dsp.stalled    =   true;
        ctx->last_fault     =   SAFETY_ERR_DSP_STALL;
    }
    else
    {
        ctx->dsp.stalled    =   false;
    }
}

/*============= SAFETY_FeedI2C =============================================================*/
/**
 *  @brief      Track consecutive PCA9685 I2C write results.
 *              One success resets the counter (transient error recovered).
 *              N consecutive failures -> bus fault (loose wire, PCA dead, etc.)
 */

void SAFETY_FeedI2C(SAFETY_Context *ctx, bool success)
{
    if(success)
    {
        ctx->i2c.consecutive_errors =   0;
        ctx->i2c.faulted            =   false;
    }
    else
    {
        ctx->i2c.consecutive_errors++;
        if(ctx->i2c.consecutive_errors >= SAFETY_I2C_MAX_ERRORS)
        {
            ctx->i2c.faulted        =   true;
            ctx->last_fault         =   SAFETY_ERR_I2C_BUS;
        }
    }
}

/*============= SAFETY_FeedRingOverflow ====================================================*/
/**
 *  @brief      If the ring buffer accumulates more than SAFETY_RING_OVERFLOW_LIMIT
 *              overflow events, the consumer (DSP) is effectively dead.
 */

void SAFETY_FeedRingOverflow(SAFETY_Context *ctx, uint32_t overflow_count)
{
    ctx->ring.overflow_count    =   overflow_count;
    ctx->ring.faulted           =   (overflow_count > SAFETY_RING_OVERFLOW_LIMIT);

    if(ctx->ring.faulted)
    {
        ctx->last_fault         =   SAFETY_ERR_RING_OVF;
    }
}

/*============= SAFETY_FeedTemperature =====================================================*/
/**
 *  @brief      On Raspberry Pi, read from:
 *              /sys/class/thermal/thermal_zone0/temp
 */

void SAFETY_FeedTemperature(SAFETY_Context *ctx, float temp_c)
{
    ctx->thermal.temperature_c  =   temp_c;
    ctx->thermal.warning        =   (temp_c > (float)SAFETY_THERMAL_WARN_C);
    ctx->thermal.critical       =   (temp_c > (float)SAFETY_THERMAL_CRITICAL_C);

    if(ctx->thermal.critical)
    {
        ctx->state          =   RADIO_SAFE;
        ctx->last_fault     =   SAFETY_ERR_THERMAL;
    }
}

/*============= SAFETY_CheckSystem =========================================================*/
/**
 *  @brief      THE decision function. 
 *              Returns true ONLY when ALL conditions are met:
 *                  1. Radio state is RUNNING
 *                  2. Battery is not critical
 *                  3. DSP is not stalled
 *                  4. I2C bus is not faulted
 *                  5. CPU is not overheated
 *                  6. Ring buffer is not in sustained overflow
 *
 *                  If ANY condition fails -> servos go to neutral.
 */

bool SAFETY_CheckSystem(const SAFETY_Context *ctx)
{
    if(ctx->state != RADIO_RUNNING)     
    {
        return false;
    }
    if(ctx->battery.critical)           
    {
        return false;
    }
    if(ctx->dsp.stalled)                
    {
        return false;
    }
    if(ctx->i2c.faulted)               
    {
        return false;
    }
    if(ctx->thermal.critical)           
    {
        return false;
    }
    if(ctx->ring.faulted)              
    {
        return false;
    }

    return true;
}

/*============= SAFETY_StatusStr ===========================================================*/

const char *SAFETY_StatusStr(SAFETY_Status status)
{
    switch(status)
    {
        case SAFETY_OK:                 
            return "OK";
        case SAFETY_ERR_RADIO:          
            return "RADIO_LOST";
        case SAFETY_ERR_BATTERY:        
            return "BATT_CRITICAL";
        case SAFETY_ERR_DSP_STALL:      
            return "DSP_STALLED";
        case SAFETY_ERR_NRF_HW:         
            return "NRF_HW_FAIL";
        case SAFETY_ERR_I2C_BUS:        
            return "I2C_BUS_FAIL";
        case SAFETY_ERR_RING_OVF:       
            return "RING_OVERFLOW";
        case SAFETY_ERR_THERMAL:        
            return "OVERTEMP";
        default:                        
            return "UNKNOWN";
    }
}

/*============= SAFETY_RadioStr ============================================================*/

const char *SAFETY_RadioStr(RADIO_State state)
{
    switch(state)
    {
        case RADIO_INIT:                
            return "INIT";
        case RADIO_RUNNING:             
            return "RUNNING";
        case RADIO_DEGRADED:            
            return "DEGRADED";
        case RADIO_RECOVERING:          
            return "RECOVERING";
        case RADIO_SAFE:                
            return "SAFE";
        default:                        
            return "???";
    }
}

/*==========================================================================================*/
