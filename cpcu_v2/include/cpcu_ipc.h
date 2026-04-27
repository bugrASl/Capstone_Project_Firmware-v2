/**
 *  @file       cpcu_ipc.h
 *  @brief      Inter-process communication via POSIX shared memory
 *  @author     bugrASl
 *  @date       12.04.2026
 *  @version    2.1
 *  @details    Lock-free SPSC ring buffer + SeqLock motor command buffer
 *              All inter-core communication through /dev/shm/cpcu_ipc.
 *
 *                          Shared Memory Layout
 *  ────────────────────────────────────────────────────────────────────────────────────
 *          Offset      Size        Contents
 *          ────────────────────────────────────────────────────────────────────────────
 *          0           192 B       IPC_ControlBlock(3 cache lines: header | head | tail)
 *          192         64 KB       IPC_SensorEntry[1024] (ring buffer, 64 B per entry)
 *          65728       128 B       IPC_MotorCommand (SeqLock protected)
 *          65856       128 B       IPC_Diagnostics (per-core atomic counters)
 *          65984       256 B       IPC_DSPExport (Python DSP -> TUI)
 */

#ifndef CPCU_IPC_H
#define CPCU_IPC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "wireless_packet.h"

#include <stdatomic.h>
#include <stdint.h>
#include <stdbool.h>    

/*============= CONSTANTS ==============================================================*/

#define IPC_SHM_NAME            "/cpcu_ipc"
#define IPC_SHM_PERMS           0644            /* Owner RW, Group/Others R */
#define IPC_MAGIC               0x494E4654UL    /* "INFT" - Infinitech */
#define IPC_VERSION             0x0204          /* v2.3.4 — added edit_mode_* control bytes */

#define IPC_SENSOR_RING_SIZE    1024
#define IPC_SENSOR_RING_MASK    (IPC_SENSOR_RING_SIZE - 1)
#define IPC_NUM_SERVOS          6

#define IPC_STATE_INIT          0
#define IPC_STATE_RUNNING       1
#define IPC_STATE_SAFE          2

/*============= CONTROL BLOCK ==========================================================*/

typedef struct __attribute__((aligned(64)))
{
    /* Cache line 0:    Header | System State */
    uint32_t            magic;
    uint16_t            version;
    _Atomic uint8_t     io_ready;
    _Atomic uint8_t     dsp_ready;
    _Atomic uint8_t     system_state;
    uint8_t             _pad0[3];
    _Atomic uint64_t    io_heartbeat_us;
    _Atomic uint32_t    motor_cmd_ack;

    /* v2.3.4: Edit-mode handshake.
     *   request:  TUI -> world. 1 = "I want edit mode", 0 = "I want to exit".
     *   active:   io+dsp -> TUI. 1 = "we're parked, you may edit",
     *             0 = "we're not in edit mode" (either not requested, or
     *             still walking to neutral, or fault forced exit).
     *   request_us: when the request was raised, for TUI's 500 ms timeout.
     * 3 atomic bytes + 5 pad + 8 timestamp = 16 bytes, fits in the 36 B
     * reserve below. The TUI is the sole writer of request/request_us;
     * cpcu_io is the sole writer of active. cpcu_dsp.py reads request,
     * never writes.
     * See cpcu_v2/docs/EDIT_MODE.md for the full handshake protocol. */
    _Atomic uint8_t     edit_mode_request;
    _Atomic uint8_t     edit_mode_active;
    _Atomic uint8_t     edit_mode_dsp_ack;          /* dsp acks it saw the request */
    uint8_t             _pad_edit[5];
    _Atomic uint64_t    edit_mode_request_us;

    uint8_t             _reserved0[12];

    /* Cache line 1:    Producer Index (Core 3 writes, Cores 1-2 read) */
    _Atomic uint32_t    sensor_head __attribute__((aligned(64)));
    uint8_t             _pad1[60];

    /* Cache line 2:    Consumer Index (Cores 1-2 write, Core 3 reads) */
    _Atomic uint32_t    sensor_tail __attribute__((aligned(64)));
    uint8_t             _pad2[60];
} IPC_ControlBlock;

_Static_assert( sizeof(IPC_ControlBlock) == 192, "IPC_ControlBlock must be 192 bytes (3 cache lines)" );

/*============= SENSOR RING ENTRY ======================================================*/

typedef struct __attribute__((aligned(64)))
{
    WL_SampleSet        samples[WL_SAMPLES_PER_PACKET];
    uint8_t             seq;
    uint8_t             flags;
    uint8_t             tx_retry;
    uint8_t             pkt_loss;
    uint16_t            timestamp;
    uint16_t            vbat_raw;
    uint64_t            rx_time_us;
    uint8_t             _pad[16];
} IPC_SensorEntry;

_Static_assert( sizeof(IPC_SensorEntry) == 64, "IPC_SensorEntry must be 64 bytes (1 cache line)" );
_Static_assert( (IPC_SENSOR_RING_SIZE & IPC_SENSOR_RING_MASK) == 0, "Ring size must be power of 2" );

/*============= MOTOR COMMAND ==========================================================*/

typedef struct __attribute__((aligned(64)))
{
    /* Cache line 0: */
    _Atomic uint32_t    seq;
    uint16_t            servo_us[IPC_NUM_SERVOS];
    uint8_t             gesture_id;
    uint8_t             confidence;
    uint16_t            _pad0;                                  
    uint64_t            timestamp_us;
    uint8_t             _pad1[28];

    /* Cache line 1:    Future Extension */
    uint8_t             _reserved[64];
} IPC_MotorCommand;

_Static_assert( sizeof(IPC_MotorCommand) == 128, "IPC_MotorCommand must be 128 bytes (2 cache lines)" );

/*============= DIAGNOSTICS ============================================================*/

typedef struct __attribute__((aligned(128)))
{
    /* Written by Core 3 only */
    _Atomic uint32_t    io_pkts_received;
    _Atomic uint32_t    io_pkts_dropped;
    _Atomic uint32_t    io_ring_overflows;
    _Atomic uint32_t    io_seq_gaps;
    _Atomic uint32_t    io_nrf_init_status;
    _Atomic uint32_t    io_safe_entries;
    _Atomic uint32_t    io_max_poll_us;
    
    /* Written by Core 1-2 only */
    _Atomic uint32_t    dsp_batches;
    _Atomic uint32_t    dsp_max_latency_us;
    _Atomic uint32_t    dsp_ring_underflows;
    _Atomic uint32_t    dsp_inferences;

    uint32_t            _reserved[5];
} IPC_Diagnostics;

_Static_assert( sizeof(IPC_Diagnostics) == 128, "IPC_Diagnostics must be 128 bytes (2 cache lines)" );

/*============= DSP EXPORT (Python -> TUI) =============================================*/
/**
 *  @brief      Extra telemetry written by Python DSP, read by TUI.
 *  @details    Contains per-channel RMS, gesture name, per-class confidence,
 *              and inference timing — data the standard MotorCommand doesn't carry.
 */

#define IPC_MAX_GESTURE_NAME    16
#define IPC_MAX_CLASSES         10

typedef struct __attribute__((aligned(64)))
{
    float               channel_rms[WL_NUM_CHANNELS];           /* Filtered RMS per channel  */
    char                gesture_name[IPC_MAX_GESTURE_NAME];     /* "REST", "HAND SLOW" etc.  */
    float               class_confidence[IPC_MAX_CLASSES];      /* Per-class probability     */
    uint8_t             num_classes;
    uint8_t             active_class;
    uint16_t            _pad0;
    uint32_t            inference_time_us;                      /* Last inference wall time   */
    _Atomic uint32_t    update_seq;                             /* Bumped on each write       */
    uint8_t             _pad1[132];                             /* Pad to 256 bytes           */
} IPC_DSPExport;

_Static_assert( sizeof(IPC_DSPExport) == 256, "IPC_DSPExport must be 256 bytes (4 cache lines)" );

/*============= RUNTIME CONFIG (v2.3.3) ================================================*/
/*
 *  cpcu_kernel reads cpcu_v2/config/runtime.json at startup and on
 *  SIGHUP, populating this region. Other processes (cpcu_io, cpcu_dsp.py
 *  via Python's mmap) read it directly. Writes are seqlock-style: kernel
 *  bumps config_seq, writes, bumps again. Readers retry on torn reads.
 *
 *  All fields are EXPLICITLY-sized so the Python side can mmap & unpack
 *  with `struct` predictably. Pad to a fixed total to keep the IPC
 *  layout deterministic across schema additions within v2.3.x.
 *
 *  See cpcu_v2/docs/RUNTIME_CONFIG.md for the full schema and the
 *  consumer wiring.
 */

#define IPC_CFG_NUM_SERVOS          6
#define IPC_CFG_VALID_MAGIC         0x43464702      /* "CFG\x02" */

typedef struct __attribute__((aligned(64)))
{
    /* Header */
    _Atomic uint32_t    config_seq;                         /* seqlock: odd = mid-write */
    uint32_t            magic;                              /* IPC_CFG_VALID_MAGIC when populated */
    uint32_t            schema_version;                     /* must match expected */
    uint32_t            _pad_hdr;

    /* Servo limits (mirror of compile-time PCA_SERVO_MIN/MAX_US arrays).
     * These ARE runtime-tunable for calibration, but cpcu_io still
     * clamps to compile-time hardware limits as a final safety. */
    uint16_t            servo_min_us[IPC_CFG_NUM_SERVOS];
    uint16_t            servo_max_us[IPC_CFG_NUM_SERVOS];

    /* Per-servo gravity-sag bias offsets (signed us, applied AFTER
     * smoothing, BEFORE clamping). v2.3.3 first consumer of the
     * runtime-config infrastructure. See JITTER_MITIGATION.md §6. */
    int16_t             servo_bias_us[IPC_CFG_NUM_SERVOS];

    /* Smoother per-servo overrides (zero = use compile-time default). */
    uint16_t            smooth_velocity_us_per_s[IPC_CFG_NUM_SERVOS];
    uint16_t            smooth_accel_us_per_s2[IPC_CFG_NUM_SERVOS];
    uint16_t            smooth_deadband_us[IPC_CFG_NUM_SERVOS];

    /* Gesture velocities (us/s, signed) — v2.3.5 consumer.
     * Indexed by [class_id][servo_id]. class_id 0 == rest. */
    int16_t             gesture_velocity[IPC_MAX_CLASSES][IPC_CFG_NUM_SERVOS];

    /* DSP/AI thresholds — v2.3.5/v2.3.6 consumers. */
    uint8_t             interp_conf_floor_pct;              /* 0-100, default 40 */
    uint8_t             interp_conf_ceil_pct;               /* 0-100, default 85 */
    uint8_t             hysteresis_votes;                   /* default 3 */
    uint8_t             _pad_dsp;
    uint16_t            grip_open_us;                       /* default 1700 */
    uint16_t            grip_touch_us;                      /* default 1200 */
    uint16_t            grip_firm_us;                       /* default 1100 */
    uint16_t            grip_stall_recover_ms;              /* default 2000 */

    /* Pad to a fixed size so future v2.3.x additions don't change the
     * IPC layout binary-incompatibly. Reserve generously. */
    uint8_t             _reserved[256];
} IPC_RuntimeConfig;

_Static_assert(sizeof(IPC_RuntimeConfig) >= 512,
               "IPC_RuntimeConfig must be >= 512 bytes (8 cache lines minimum)");

/*============= TOTAL SHM SIZE =========================================================*/

#define IPC_SHM_SIZE    (\
        sizeof(IPC_ControlBlock)                            +\
        sizeof(IPC_SensorEntry) *   IPC_SENSOR_RING_SIZE    +\
        sizeof(IPC_MotorCommand)                            +\
        sizeof(IPC_Diagnostics)                             +\
        sizeof(IPC_DSPExport)                               +\
        sizeof(IPC_RuntimeConfig)                            \
        )

/*============= CONTEXT HANDLE =========================================================*/

typedef struct
{
    void                *base;
    IPC_ControlBlock    *ctrl;
    IPC_SensorEntry     *ring;
    IPC_MotorCommand    *motor;
    IPC_Diagnostics     *diag;
    IPC_DSPExport       *dsp_export;
    IPC_RuntimeConfig   *config;            /* v2.3.3 */
    int                 shm_fd;
} IPC_Context;

/*============= API ====================================================================*/

int         IPC_Create(IPC_Context *ctx);
int         IPC_Open(IPC_Context *ctx);
void        IPC_Close(IPC_Context *ctx);
void        IPC_Destroy(void);

/* Ring Buffer */
void        IPC_PushSensor(IPC_Context *ctx, const WL_Packet *pkt, uint64_t rx_time_us);
uint32_t    IPC_PopSensorBatch(IPC_Context *ctx, IPC_SensorEntry *out, uint32_t max_count);
uint32_t    IPC_SensorCount(IPC_Context *ctx);

/* Motor Command (SeqLock) */
void        IPC_WriteMotorCmd(IPC_Context *ctx, const uint16_t servo_us[IPC_NUM_SERVOS],
                              uint8_t gesture_id, uint8_t confidence, uint64_t timestamp_us);
bool        IPC_ReadMotorCmd(IPC_Context *ctx, uint16_t servo_us[IPC_NUM_SERVOS],
                             uint8_t *gesture_id, uint8_t *confidence, uint32_t *last_ack);

/* Runtime Config (SeqLock, v2.3.3).
 * Writers (cpcu_kernel only) call IPC_WriteRuntimeConfig. Readers
 * (cpcu_io, cpcu_dsp.py) call IPC_ReadRuntimeConfig — it copies the
 * full struct out under a torn-read-retry loop. The copy is cheap
 * (~512 bytes); readers should call this once per loop iteration
 * rather than holding pointers into shared memory across barriers. */
void        IPC_WriteRuntimeConfig(IPC_Context *ctx, const IPC_RuntimeConfig *src);
bool        IPC_ReadRuntimeConfig (IPC_Context *ctx, IPC_RuntimeConfig *dst);
uint32_t    IPC_RuntimeConfigSeq  (IPC_Context *ctx);    /* cheap polling check */

#ifdef __cplusplus
}
#endif

#endif  /* CPCU_IPC_H */                          
