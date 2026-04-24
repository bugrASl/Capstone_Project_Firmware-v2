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
#define IPC_VERSION             0x0201          /* v2.1 */

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
    uint8_t             _reserved0[36];

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

/*============= TOTAL SHM SIZE =========================================================*/

#define IPC_SHM_SIZE    (\
        sizeof(IPC_ControlBlock)                            +\
        sizeof(IPC_SensorEntry) *   IPC_SENSOR_RING_SIZE    +\
        sizeof(IPC_MotorCommand)                            +\
        sizeof(IPC_Diagnostics)                             +\
        sizeof(IPC_DSPExport)                                \
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

#ifdef __cplusplus
}
#endif

#endif  /* CPCU_IPC_H */                          
