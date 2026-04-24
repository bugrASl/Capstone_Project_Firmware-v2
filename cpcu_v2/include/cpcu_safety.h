/**
 *  @file       cpcu_safety.h
 *  @brief      System-wide safety monitor: radio link, battery, DSP health,
 *              NRF hardware, I2C bus, thermal, and ring buffer overflow.
 *  @author     bugrASl
 *  @date       13.04.2026
 *  @version    2.0
 *  @details    This module is the single authority on whether servo motors
 *              may move or not. It gathers fault conditions from every
 *              subsystem into one: SAFETY_CheckSystem() which returns
 *              true or false.
 *
 *              Fault Sources:
 *                  1. Radio Link   :       Packet timeout, sequence gaps, retransmit rate 
 *                  2. Battery      :       Critical voltage level
 *                  3. DSP Pipeline :       Inference stall
 *                  4. NRF Hardware :       Init failure, SPI errors
 *                  5. I2C Bus      :       Write errors
 *                  6. Ring Buffer  :       Sustained overflow
 *                  7. Thermal      :       CPU temperature exceeds threshold
 */

#ifndef CPCU_SAFETY_H
#define CPCU_SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif    

#include "wireless_packet.h"

#include <stdint.h>
#include <stdbool.h>

/*============= CONFIGURATION CONSTANTS ============================================================*/

/* Radio Timing */
#define SAFETY_RADIO_TIMEOUT_MS         750         /* Silence  -> Degraded */
#define SAFETY_RADIO_SAFE_MS            1500        /* Degraded -> Safe     */
#define RECOVERY_PKT_COUNT              10

/* Link Quality */    
#define SAFETY_LINK_WINDOW              1000        /* packets per statistics window */
#define SAFETY_LINK_RETRY_GOOD          0.5f        /* mean_retry < this -> GOOD */
#define SAFETY_LINK_RETRY_DEGRADED      3.0f        /* mean_retry < this -> DEGRADED, else POOR */
#define SAFETY_LINK_LOSS_GOOD           0.001f      /* loss_rate < this -> GOOD */
#define SAFETY_LINK_LOSS_DEGRADED       0.05f       /* loss_rate < this -> DEGRADED, else POOR */
 
/* Battery voltage thresholds */
#define SAFETY_VBAT_LOW_V               3.0f        /* Below this -> warning */
#define SAFETY_VBAT_CRITICAL_V          2.7f        /* Below this -> SAFE */
#define SAFETY_VBAT_DIVIDER             2.0f        /* Resistor divider factor on BSAU */    

/* DSP pipeline health */
#define SAFETY_DSP_STALL_MS             2000        /* No motor cmd for this long -> fault */
 
/* I2C bus health */
#define SAFETY_I2C_MAX_ERRORS           5           /* Consecutive PCA write failures -> fault */
 
/* Ring buffer */
#define SAFETY_RING_OVERFLOW_LIMIT      100         /* Cumulative overflows -> fault */
 
/* Thermal */
#define SAFETY_THERMAL_WARN_C           75          /* Log warning above this */
#define SAFETY_THERMAL_CRITICAL_C       82          /* Force SAFE above this */

/*============= STATUS RETURN ======================================================================*/

typedef enum
{
    SAFETY_OK = 0,
    SAFETY_ERR_RADIO,
    SAFETY_ERR_BATTERY,
    SAFETY_ERR_DSP_STALL,
    SAFETY_ERR_NRF_HW,
    SAFETY_ERR_I2C_BUS,
    SAFETY_ERR_RING_OVF,
    SAFETY_ERR_THERMAL,
} SAFETY_Status;

/*============= RADIO STATES =======================================================================*/

typedef enum
{
    RADIO_INIT = 0,
    RADIO_RUNNING,
    RADIO_DEGRADED,
    RADIO_RECOVERING,
    RADIO_SAFE,
} RADIO_State;

/*============= LINK QUALITY =======================================================================*/

typedef enum
{
    LINK_GOOD = 0,
    LINK_DEGRADED,
    LINK_POOR,
} LINK_Quality;

/*============= LINK STATISTICS ====================================================================*/

typedef struct
{
    LINK_Quality    quality;                        
    uint32_t        w_packets;                      /* Packets in current window */
    uint32_t        w_gaps;                         /* Sequence gap in current window */ 
    uint32_t        w_retry_sum;                    /* Sum of tx_retry in current window */
    float           mean_retry;                       
    float           loss_rate;
    uint16_t        last_timestamp;                 /* Previous packet timestamp */
    bool            has_previous;                   /* True after first timestamp seen */
} LINK_Stats;

/*============= BATTERY STATES =====================================================================*/

typedef struct
{
    uint16_t        raw;
    float           voltage;
    uint8_t         level;
    bool            critical;
} BATTERY_State;

/*============= DSP PIPELINE HEALTH ================================================================*/

typedef struct
{
    uint64_t        last_cmd_us;
    bool            stalled;
    bool            active;
} DSP_Health;

/*============= I2C HEALTH =========================================================================*/

typedef struct
{
    uint32_t        consecutive_errors;
    bool            faulted;
} I2C_Health;

/*============= RING BUFFER HEALTH =================================================================*/

typedef struct
{
    uint32_t        overflow_count;
    bool            faulted;
} RING_Health;

/*============= THERMAL HEALTH =====================================================================*/

typedef struct
{
    float           temperature_c;
    bool            warning;
    bool            critical;
} THERMAL_Health;

/*============= CONTEXT HANDLE =====================================================================*/

typedef struct
{
    /* Primary State Machine */
    RADIO_State     state;
    SAFETY_Status   last_fault;

    /* Radio Tracking */
    uint64_t        last_pkt_rcv_us;
    uint64_t        degraded_entry_us;
    uint32_t        recovery_cnt;
    uint8_t         expected_seq;
    bool            seq_init;

    /* Subsystem Monitoring */
    LINK_Stats      link;
    BATTERY_State   battery;
    DSP_Health      dsp;
    I2C_Health      i2c;
    RING_Health     ring;
    THERMAL_Health  thermal;
} SAFETY_Context;

/*============= API ================================================================================*/

void        SAFETY_Init(SAFETY_Context *ctx);

/* Radio Link Monitoring */
uint32_t    SAFETY_SeqGap(SAFETY_Context *ctx, uint8_t seq);
void        SAFETY_FeedPacket(SAFETY_Context *ctx, const WL_Packet *pkt, uint64_t now_us);
void        SAFETY_CheckTimeout(SAFETY_Context *ctx, uint64_t now_us);

/* DSP Pipeline Health */
void        SAFETY_FeedMotorCMD(SAFETY_Context *ctx, uint64_t now_us);
void        SAFETY_CheckDSP(SAFETY_Context *ctx, uint64_t now_us);

/* I2C Bus Health */
void        SAFETY_FeedI2C(SAFETY_Context *ctx, bool success);

/* Ring Buffer Health */
void        SAFETY_FeedRingOverflow(SAFETY_Context *ctx, uint32_t overflow_count);

/* Thermal Monitoring */
void        SAFETY_FeedTemperature(SAFETY_Context *ctx, float temp_c);

/* System Check */
bool        SAFETY_CheckSystem(const SAFETY_Context *ctx);

/* Helpers */
const char  *SAFETY_StatusStr(SAFETY_Status status);
const char  *SAFETY_RadioStr(RADIO_State state);

#ifdef __cplusplus
}
#endif

#endif /* CPCU_SAFETY_H */
