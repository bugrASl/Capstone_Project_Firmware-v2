/**
 *  @file       cpcu_safety.h
 *  @brief      System-wide safety monitor — public interface, v2.3.1.
 *  @author     bugrASl
 *  @date       April 2026
 *
 *  v2.3.1 changes (2026-04):
 *      - Cold-start radio grace period. SAFETY_CheckTimeout now
 *        suppresses the radio fault until either (a) the first
 *        valid packet has been received, or (b) SAFETY_RADIO_BOOT_
 *        GRACE_MS (5 s default) has elapsed since SAFETY_Init.
 *        Distinguishes "BSAU and CPCU were powered on independently
 *        and we just haven't met yet" from "we used to be talking
 *        and now we're not", which used to look identical to the
 *        FSM. Two new fields in SAFETY_Context: boot_us,
 *        first_packet_seen. Public API unchanged.
 *      - See cpcu_v2/docs/BOOT_AND_SYNC.md for the full design.
 *
 *  v2.3 changes (2026-04):
 *      - RING_Health now tracks a baseline + last-growth timestamp so the
 *        fault flag can clear once the SPSC ring stops dropping packets
 *        for SAFETY_RING_RECOVER_MS. Previously, since io_ring_overflows
 *        is a monotonic cumulative atomic counter, once it crossed the
 *        100-overflow threshold the FSM would latch in SAFE forever
 *        even after the producer/consumer rebalanced. The fault threshold
 *        is now applied to the *delta* since the last quiescent baseline,
 *        making it a true rate detector with hysteresis.
 *      - Public API of SAFETY_FeedRingOverflow is unchanged (still takes
 *        the cumulative count); the recovery logic is fully internal so
 *        cpcu_io and the safety testbench remain source-compatible.
 *      - SAFETY_VBAT_DIVIDER restored to 2.0 (was incorrectly set to 1.0
 *        in v2.2 with a comment claiming the BSAU firmware would
 *        correct; that BSAU change never shipped, so the v2.2 value
 *        of 1.0 made every healthy 4 V battery read as 2.00 V on the
 *        CPCU side, latching battery.critical true). See note next to
 *        the macro.
 *      - cpcu_io.c now calls SAFETY_UpdateState() once per loop after
 *        the existing Feed/Check calls. This is the last piece of the
 *        v2.2 refactor that wasn't actually wired in production —
 *        without it, non-radio fault transitions (battery, thermal,
 *        i2c, ring) updated the boolean flags but never moved the
 *        FSM state out of RUNNING. CheckSystem already gated on the
 *        flags so servos were parked correctly, but the TUI state
 *        indicator never said SAFE.
 *
 *  v2.2 changes (2026-04):
 *      - Replaced SAFETY_TryRecover with SAFETY_UpdateState — handles
 *        BOTH transitions into and out of RADIO_SAFE in one place.
 *
 *  Caller contract:
 *      - cpcu_io must call FeedI2C from BOTH the success path AND the
 *        else-branch fallback (where it writes neutrals via PCA), or
 *        i2c.faulted will never clear once tripped.
 *      - cpcu_io must call FeedMotorCMD whenever IPC_ReadMotorCmd
 *        returns true, regardless of safety state. Otherwise dsp.stalled
 *        latches and SAFE recovery cannot succeed.
 *      - cpcu_io must call FeedRingOverflow every loop, even when no
 *        new overflows occurred — the recovery timer needs a heartbeat.
 *
 *  Ordered call sequence per loop:
 *          1.  SAFETY_FeedPacket          (whenever a packet arrives)
 *          2.  SAFETY_CheckTimeout        (every loop)
 *          3.  SAFETY_CheckDSP            (every loop)
 *          4.  SAFETY_FeedRingOverflow    (every loop)
 *          5.  SAFETY_FeedI2C             (whenever I2C is touched)
 *          6.  SAFETY_FeedTemperature     (every ~5 s)
 *          7.  SAFETY_UpdateState         (every loop, after the above)
 */

#ifndef CPCU_SAFETY_H
#define CPCU_SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "wireless_packet.h"

/*============= TUNABLE CONFIG =============================================*/

/* Radio Timing */
#define SAFETY_RADIO_TIMEOUT_MS         750
#define SAFETY_RADIO_SAFE_MS            1500
#define SAFETY_RADIO_BOOT_GRACE_MS      5000        /* v2.3.1: cold-start
                                                       grace before radio
                                                       timeout fires. Distin-
                                                       guishes "no packet ever
                                                       received from this BSAU"
                                                       (initial sync) from
                                                       "packets stopped"
                                                       (genuine fault). See
                                                       docs/BOOT_AND_SYNC.md
                                                       for the full rationale. */
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
#define SAFETY_VBAT_DIVIDER             2.0f    /*  BSAU PB0 has a 2:1 resistor
                                                 *  divider so a 4.0 V battery
                                                 *  reads as 2.0 V at the ADC.
                                                 *  bsau_app.c sends the raw
                                                 *  post-divider 12-bit count
                                                 *  directly, so CPCU must
                                                 *  multiply by 2.0 here to
                                                 *  recover the battery V.
                                                 *  v2.2 set this to 1.0 with
                                                 *  a doc-comment claiming the
                                                 *  BSAU firmware now corrects
                                                 *  — but BSAU_ADC_GetBattery()
                                                 *  in bsau_adc.c is unchanged
                                                 *  and passes the raw count
                                                 *  through, so 1.0 was a real
                                                 *  bug: every healthy 4.0 V
                                                 *  battery was being read as
                                                 *  2.00 V on the CPCU side
                                                 *  → battery.critical stuck
                                                 *  true → CheckSystem stuck
                                                 *  false → servos always
                                                 *  parked at neutral.
                                                 *  Restored to 2.0 in v2.3.*/

/* DSP */
#define SAFETY_DSP_STALL_MS             2000

/* I2C */
#define SAFETY_I2C_MAX_ERRORS           5

/* Ring (with recovery, v2.3) */
#define SAFETY_RING_OVERFLOW_LIMIT      100         /* trip threshold (delta) */
#define SAFETY_RING_RECOVER_MS          5000        /* clear after this much
                                                       quiescence            */

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

/**
 *  v2.3 RING_Health.
 *      overflow_count   — last cumulative count we sampled (atomic from IPC).
 *      baseline_count   — count value at the last quiescent re-baseline.
 *                         The trip threshold is applied to (overflow_count
 *                         - baseline_count) so the FSM doesn't latch on a
 *                         monotonic counter.
 *      last_growth_us   — last time the cumulative count went up. Used by
 *                         the recovery side: faulted clears once
 *                         SAFETY_RING_RECOVER_MS pass with no new growth.
 *      faulted          — public flag consumed by SAFETY_CheckSystem and
 *                         SAFETY_UpdateState.
 */
typedef struct
{
    uint32_t        overflow_count;
    uint32_t        baseline_count;
    uint64_t        last_growth_us;
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

    /* v2.3.1: cold-start grace tracking. boot_us is set by
     * SAFETY_Init to the moment the safety subsystem started.
     * first_packet_seen flips to true the first time
     * SAFETY_FeedPacket is invoked. SAFETY_CheckTimeout suppresses
     * the radio fault until either is true OR until the grace has
     * elapsed. See docs/BOOT_AND_SYNC.md. */
    uint64_t        boot_us;
    bool            first_packet_seen;

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
uint32_t    SAFETY_FeedPacket(SAFETY_Context *ctx, const WL_Packet *pkt, uint64_t now_us);
void        SAFETY_CheckTimeout(SAFETY_Context *ctx, uint64_t now_us);

void        SAFETY_FeedMotorCMD(SAFETY_Context *ctx, uint64_t now_us);
void        SAFETY_CheckDSP(SAFETY_Context *ctx, uint64_t now_us);

void        SAFETY_FeedI2C(SAFETY_Context *ctx, bool success);

/**
 *  v2.3: API unchanged (still takes the cumulative atomic counter from
 *  ipc.diag->io_ring_overflows), but the recovery logic is now internal
 *  — the function uses clock_gettime() to maintain its own quiescence
 *  timer so cpcu_io and the safety testbench did not have to change.
 *  Call this every loop, even when no new overflows occurred.
 */
void        SAFETY_FeedRingOverflow(SAFETY_Context *ctx, uint32_t overflow_count);

void        SAFETY_FeedTemperature(SAFETY_Context *ctx, float temp_c);

bool        SAFETY_CheckSystem(const SAFETY_Context *ctx);

/*  v2.2: Single function that handles ALL FSM transitions.
 *  Call it once per loop after all the Feed/Check functions have
 *  refreshed the boolean fault flags.                                   */
void        SAFETY_UpdateState(SAFETY_Context *ctx, uint64_t now_us);

bool        SAFETY_ShouldAutoClear(SAFETY_Context *ctx, uint64_t now_us);

const char  *SAFETY_StatusStr(SAFETY_Status status);
const char  *SAFETY_RadioStr(RADIO_State state);

#ifdef __cplusplus
}
#endif

#endif /* CPCU_SAFETY_H */
