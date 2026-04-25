/**
 *  @file       cpcu_safety.h
 *  @brief      System-wide safety monitor.
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    2.2
 *
 *  v2.2 changes (2026-04):
 *      - Renamed SAFETY_TryRecover → SAFETY_UpdateState. The function now
 *        handles BOTH entry into SAFE (for thermal/dsp/i2c/ring conditions
 *        that were previously not transitioning the FSM) AND exit from
 *        SAFE once all triggers clear.
 *      - last_pkt_rcv_us is no longer used as a recovery gate while in
 *        SAFE. Recovery hinges on the actual fault flags (battery.critical,
 *        thermal.critical, etc.), not on packet arrival timing.
 *
 *  v2.1 changes (still in effect):
 *      - SAFE state is recoverable. After all triggering conditions
 *        have been clear for SAFETY_SAFE_RECOVER_MS, FSM exits SAFE.
 *      - Battery hysteresis (critical at <2.7 V, recover above 3.0 V).
 *      - Thermal hysteresis (critical at >82 °C, recover below 70 °C).
 *      - SAFETY_VBAT_DIVIDER = 1.0 (was 2.0; BSAU firmware corrects
 *        before transmitting).
 *      - LINK auto-clear after 5 min of LINK_GOOD.
 *
 *  Design contract:
 *      The OWNING file (cpcu_io.c) calls these in order each loop:
 *          1.  SAFETY_FeedPacket       (when a packet arrives)
 *          2.  SAFETY_CheckTimeout     (every loop, regardless of packet)
 *          3.  SAFETY_CheckDSP         (every loop)
 *          4.  SAFETY_FeedRingOverflow (every loop)
 *          5.  SAFETY_FeedTemperature  (periodic, e.g. 1 Hz)
 *          6.  SAFETY_FeedI2C          (after every I2C write attempt —
 *                                        IMPORTANT: feed it even from the
 *                                        SAFE-state fallback or i2c.faulted
 *                                        will never clear)
 *          7.  SAFETY_FeedMotorCMD     (whenever a motor cmd is read —
 *                                        IMPORTANT: read it every loop
 *                                        even in SAFE, otherwise dsp.stalled
 *                                        will never clear)
 *          8.  SAFETY_UpdateState      (every loop, decides FSM transitions)
 *          9.  SAFETY_CheckSystem      (every loop, gates servo motion)
 *         10.  SAFETY_ShouldAutoClear  (every loop, signals counter reset)
 */

#ifndef CPCU_SAFETY_H
#define CPCU_SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "wireless_packet.h"

#include <stdint.h>
#include <stdbool.h>

/*============= CONFIGURATION ==============================================*/

/* Radio Timing */
#define SAFETY_RADIO_TIMEOUT_MS         750
#define SAFETY_RADIO_SAFE_MS            1500
#define RECOVERY_PKT_COUNT              10

/* Link Quality */
#define SAFETY_LINK_WINDOW              1000
#define SAFETY_LINK_RETRY_GOOD          0.5f
#define SAFETY_LINK_RETRY_DEGRADED      3.0f
#define SAFETY_LINK_LOSS_GOOD           0.001f
#define SAFETY_LINK_LOSS_DEGRADED       0.05f

/* Battery (with hysteresis) */
#define SAFETY_VBAT_LOW_V               3.0f
#define SAFETY_VBAT_CRITICAL_V          2.7f
#define SAFETY_VBAT_RECOVER_V           3.0f
#define SAFETY_VBAT_DIVIDER             1.0f    /* See header note */

/* DSP */
#define SAFETY_DSP_STALL_MS             2000

/* I2C */
#define SAFETY_I2C_MAX_ERRORS           5

/* Ring */
#define SAFETY_RING_OVERFLOW_LIMIT      100

/* Thermal (with hysteresis) */
#define SAFETY_THERMAL_WARN_C           75
#define SAFETY_THERMAL_CRITICAL_C       82
#define SAFETY_THERMAL_RECOVER_C        70

/* SAFE recovery */
#define SAFETY_SAFE_RECOVER_MS          3000

/* Auto-clear of cumulative diag counters */
#define SAFETY_AUTO_CLEAR_MS            300000

/*============= STATUS =====================================================*/

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

typedef enum
{
    RADIO_INIT = 0,
    RADIO_RUNNING,
    RADIO_DEGRADED,
    RADIO_RECOVERING,
    RADIO_SAFE,
} RADIO_State;

typedef enum
{
    LINK_GOOD = 0,
    LINK_DEGRADED,
    LINK_POOR,
} LINK_Quality;

typedef struct
{
    LINK_Quality    quality;
    uint32_t        w_packets;
    uint32_t        w_gaps;
    uint32_t        w_retry_sum;
    float           mean_retry;
    float           loss_rate;
    uint16_t        last_timestamp;
    bool            has_previous;
    uint64_t        good_since_us;
} LINK_Stats;

typedef struct
{
    uint16_t        raw;
    float           voltage;
    uint8_t         level;
    bool            critical;
} BATTERY_State;

typedef struct
{
    uint64_t        last_cmd_us;
    bool            stalled;
    bool            active;
} DSP_Health;

typedef struct
{
    uint32_t        consecutive_errors;
    bool            faulted;
} I2C_Health;

typedef struct
{
    uint32_t        overflow_count;
    bool            faulted;
} RING_Health;

typedef struct
{
    float           temperature_c;
    bool            warning;
    bool            critical;
} THERMAL_Health;

typedef struct
{
    RADIO_State     state;
    SAFETY_Status   last_fault;

    uint64_t        last_pkt_rcv_us;
    uint64_t        degraded_entry_us;
    uint32_t        recovery_cnt;
    uint8_t         expected_seq;
    bool            seq_init;

    LINK_Stats      link;
    BATTERY_State   battery;
    DSP_Health      dsp;
    I2C_Health      i2c;
    RING_Health     ring;
    THERMAL_Health  thermal;

    uint64_t        safe_clear_since_us;
    uint64_t        safe_entry_us;
} SAFETY_Context;

/*============= API ========================================================*/

void        SAFETY_Init(SAFETY_Context *ctx);

uint32_t    SAFETY_SeqGap(SAFETY_Context *ctx, uint8_t seq);
void        SAFETY_FeedPacket(SAFETY_Context *ctx, const WL_Packet *pkt, uint64_t now_us);
void        SAFETY_CheckTimeout(SAFETY_Context *ctx, uint64_t now_us);

void        SAFETY_FeedMotorCMD(SAFETY_Context *ctx, uint64_t now_us);
void        SAFETY_CheckDSP(SAFETY_Context *ctx, uint64_t now_us);

void        SAFETY_FeedI2C(SAFETY_Context *ctx, bool success);
void        SAFETY_FeedRingOverflow(SAFETY_Context *ctx, uint32_t overflow_count);
void        SAFETY_FeedTemperature(SAFETY_Context *ctx, float temp_c);

bool        SAFETY_CheckSystem(const SAFETY_Context *ctx);

/*  v2.2: Single function that handles ALL FSM transitions.
 *  Call it once per loop after all the Feed/Check functions have
 *  refreshed the boolean fault flags. It will:
 *      - Move RUNNING → SAFE if any non-radio/non-batt fault arose
 *        (battery and radio are handled inside FeedPacket / CheckTimeout
 *         since they're the original two state-machine drivers)
 *      - Move SAFE → RECOVERING/RUNNING once all faults have been clear
 *        for SAFETY_SAFE_RECOVER_MS                                       */
void        SAFETY_UpdateState(SAFETY_Context *ctx, uint64_t now_us);

bool        SAFETY_ShouldAutoClear(SAFETY_Context *ctx, uint64_t now_us);

const char  *SAFETY_StatusStr(SAFETY_Status status);
const char  *SAFETY_RadioStr(RADIO_State state);

#ifdef __cplusplus
}
#endif

#endif /* CPCU_SAFETY_H */
