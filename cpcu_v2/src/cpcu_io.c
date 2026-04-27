/**
 *  @file       cpcu_io.c
 *  @brief      Core 3 — Real-time I/O controller
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    2.3.3
 *  @details    Deterministic loop:
 *                  1. Busy-poll NRF via spidev         -> read payload
 *                  2. WL_Unpack -> seq/safety/link     -> push to SPSC ring
 *                  3. Rate-limited (50 Hz) PCA servo update via I2C
 *                  4. Safety checks: radio, DSP, I2C, thermal, ring
 *                  5. Heartbeat to shared memory for watchdog
 *
 *              v2.3.3 changes:
 *                  - Reads IPC_RuntimeConfig once per servo tick
 *                    (cfg_cache local). Per-servo bias offsets
 *                    (cfg_cache.servo_bias_us[]) are added to the
 *                    smoothed pulse-width before clamping to
 *                    compile-time hardware limits and writing the PCA.
 *                    Bias is signed (typically +/- 20 us) and is
 *                    intended for static gravity-sag compensation;
 *                    runtime tunable from JSON, picked up on next
 *                    SIGHUP-driven kernel reload. The bias-then-clamp
 *                    order means the runtime config can never escape
 *                    the compile-time safety envelope. See
 *                    cpcu_v2/docs/RUNTIME_CONFIG.md.
 *                  - SMOOTH_MarkWritten now records the BIASED pulse
 *                    width that actually went to the PCA, so the
 *                    deadband logic stays coherent against the true
 *                    hardware state.
 *
 *              v2.3.2 changes:
 *                  - Per-servo PCA writes are now gated on
 *                    SMOOTH_ShouldWrite() to suppress redundant refreshes
 *                    of settled servos. A servo holding its target within
 *                    its hold_deadband_us[] window stops being commanded,
 *                    which kills static jitter caused by the servo's
 *                    internal P controller fighting backlash and gravity
 *                    sag on every 50 Hz tick.
 *                  - SAFETY_FeedI2C is only invoked on ticks that
 *                    actually performed I/O, so a pure-deadband tick
 *                    (all servos settled, no writes) doesn't pollute
 *                    the I²C health counters.
 *                  - SMOOTH_MarkWritten called after each successful
 *                    write to keep the deadband shadow coherent.
 *                  - PCA_SetAllNeutral (SAFE-snap path) and PCA_AllOff
 *                    (I²C-streak path) update the shadow appropriately:
 *                    the SAFE path marks all written; the AllOff path
 *                    clears ever_written so the first post-recovery
 *                    write always goes through.
 *                  - See cpcu_v2/docs/JITTER_MITIGATION.md.
 *
 *              v2.3 changes:
 *                  - Now calls SAFETY_UpdateState() once per loop after the
 *                    Feed/Check calls. The v2.2 architectural change that
 *                    moved battery / thermal / i2c / ring transitions out
 *                    of FeedPacket and into UpdateState was never wired up
 *                    here; without this call the FSM would update boolean
 *                    flags but never move out of RUNNING for non-radio
 *                    faults. SAFETY_CheckSystem already gated on the flags
 *                    so servos were neutralised correctly, but the TUI
 *                    state indicator lied during a battery / thermal /
 *                    ring fault.
 *
 *              v2.2 changes:
 *                  - Uses cpcu_log LOG_* macros everywhere (no more printf)
 *                  - Supports --log flag for per-module CSV output
 *                  - On sustained SPI errors, now calls NRF_FlushRX / NRF_ClearIRQ /
 *                    NRF_PowerDown before re-init (cleaner recovery)
 *                  - Reports NRF_GetStatus + STATUS bits in 1 Hz telemetry
 *                  - Calls PCA_AllOff on graceful shutdown (instead of neutral)
 *                    so no current flows after exit
 *                  - Reports SMOOTH_AllSettled() in telemetry for motion tracking
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>

#include "nrf24l01_linux.h"
#include "wireless_packet.h"
#include "cpcu_ipc.h"
#include "cpcu_pca9685.h"
#include "cpcu_safety.h"
#include "cpcu_smooth.h"
#include "cpcu_log.h"
#include "cpcu_config.h"

/*============= CONFIG =====================================================================*/

#define SPI_DEVICE              "/dev/spidev0.0"
#define SPI_SPEED               8000000
#define I2C_DEVICE              "/dev/i2c-1"
#define PCA9685_ADDR            0x40
#define GPIO_CE                 25
#define NRF_CHANNEL             76
#define NRF_ADDRESS             {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}

#define NRF_INIT_RETRIES        3
#define NRF_INIT_POR_MS         200
#define NRF_REINIT_INTERVAL_US  3000000ULL      /* 3 s */
#define SERVO_INTERVAL_US       20000ULL        /* 50 Hz */
#define HEARTBEAT_INTERVAL_US   100000ULL       /* 100 ms */
#define DIAG_INTERVAL_US        1000000ULL      /* 1 s */
#define THERMAL_INTERVAL_US     5000000ULL      /* 5 s */

#define THERMAL_PATH            "/sys/class/thermal/thermal_zone0/temp"

/*============= TIMING =====================================================================*/

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

static void sleep_ms(uint32_t ms)
{
    struct timespec ts  =   { .tv_sec = ms / 1000, .tv_nsec = (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

/*============= THERMAL READ ===============================================================*/

static float read_cpu_temp(void)
{
    FILE *f                         =   fopen(THERMAL_PATH, "r");
    if(!f) return 0.0f;

    int millidegrees                =   0;
    if(fscanf(f, "%d", &millidegrees) != 1)
        millidegrees                =   0;
    fclose(f);

    return millidegrees / 1000.0f;
}

/*============= GLOBALS ====================================================================*/

static volatile sig_atomic_t g_run  =   1;

static void on_signal(int s)
{
    (void)s; g_run = 0;
}

/*============= MAIN =======================================================================*/

int main(int argc, char *argv[])
{
    /* Bring up logging first so every step below is captured */
    Log_Init("IO", LOG_INFO);

    /* Parse CLI */
    bool enable_file_log  =   false;
    for(int i = 1; i < argc; i++)
    {
        if(strcmp(argv[i], "--log") == 0)
            enable_file_log = true;
        else if(strcmp(argv[i], "--debug") == 0)
            Log_SetLevel(LOG_DEBUG);
        else if(strcmp(argv[i], "--trace") == 0)
            Log_SetLevel(LOG_TRACE);
    }
    if(enable_file_log)
    {
        Log_EnableFiles(LOG_DIR_DEFAULT);
        LOG_I("IO", "file logging enabled -> %s/log_*.csv", LOG_DIR_DEFAULT);
    }

    LOG_I("IO", "=== CPCU I/O Controller (Core 3) v2.2 ===");
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    mlockall(MCL_CURRENT | MCL_FUTURE);

    /* Open GPIO chip */
    int gpio_fd =   open("/dev/gpiochip4", O_RDWR);
    if(gpio_fd < 0)
    {
        gpio_fd =   open("/dev/gpiochip0", O_RDWR);
    }
    if(gpio_fd < 0)
    {
        LOG_F("IO", "gpiochip open failed: %s", strerror(errno));
        Log_CloseFiles();
        return 1;
    }

    /* Open IPC */
    IPC_Context ipc;
    if(IPC_Open(&ipc) != 0)
    {
        LOG_F("IO", "IPC open failed — is cpcu_kernel running?");
        close(gpio_fd);
        Log_CloseFiles();
        return 1;
    }

    /*  Init NRF with retry */
    NRF_Handle nrf;
    const uint8_t addr[NRF_ADDR_WIDTH]  =   NRF_ADDRESS;
    NRF_Status nrf_ret                  =   NRF_ERR_NOT_DETECTED;
    bool nrf_ok                         =   false;

    for(int i = 0; i < NRF_INIT_RETRIES; i++)
    {
        if(i > 0)
        {
            sleep_ms(NRF_INIT_POR_MS);
        }
        nrf_ret =   NRF_Init(&nrf, SPI_DEVICE, SPI_SPEED,
                            gpio_fd, GPIO_CE,
                            NRF_CHANNEL, addr);
        if(nrf_ret == NRF_OK)
        {
            nrf_ok = true;
            /* Read back the config register as a post-init sanity check */
            uint8_t cfg     =   NRF_ReadReg(&nrf, 0x00);       /* CONFIG */
            uint8_t en_rxa  =   NRF_ReadReg(&nrf, 0x02);       /* EN_RXADDR */
            uint8_t rf_ch   =   NRF_ReadReg(&nrf, 0x05);       /* RF_CH */
            LOG_I("NRF", "init OK  CONFIG=0x%02X EN_RXADDR=0x%02X RF_CH=%u",
                  cfg, en_rxa, rf_ch);
            break;
        }

        LOG_W("NRF", "init attempt %d failed (%d)", i + 1, nrf_ret);
    }
    atomic_store(&ipc.diag->io_nrf_init_status, (uint32_t)nrf_ret);

    /* Init PCA9685 */
    PCA_Handle pca;
    PCA_Status pca_init_ret =   PCA_Init(&pca, I2C_DEVICE, PCA9685_ADDR);
    bool       pca_ok       =   (pca_init_ret == PCA_OK);
    if(pca_ok)
    {
        /* Read back MODE1 as a connectivity check */
        uint8_t mode1 = 0;
        if(PCA_ReadReg(&pca, PCA_REG_MODE1, &mode1) == PCA_OK)
            LOG_I("PCA", "init OK  addr=0x%02X prescale=%u MODE1=0x%02X",
                  pca.addr, pca.prescaler, mode1);
    }
    else
    {
        LOG_E("PCA", "init failed (%d)", pca_init_ret);
    }

    /* Init safety */
    SAFETY_Context safety;
    SAFETY_Init(&safety);

    /* Init servo smoother (start at neutral) */
    SMOOTH_Context smooth;
    SMOOTH_Init(&smooth, PCA_SERVO_NEUTRAL);

    /* Per-channel slew-rate override: gripper wants a gentler profile because
     * SG90 is lightweight and overshoots when commanded full-range instantly.
     * This exercises SMOOTH_SetSpeed, which was previously unused. */
    SMOOTH_SetSpeed(&smooth, 5, 1200);      /* Gripper: slower, 1200 us/s */

    /* Signal ready */
    atomic_store(&ipc.ctrl->io_ready, 1);
    atomic_store(&ipc.ctrl->system_state, nrf_ok ? IPC_STATE_RUNNING : IPC_STATE_SAFE);
    LOG_I("IO", "ready (NRF=%s PCA=%s). Entering loop.",
          nrf_ok ? "OK" : "FAIL", pca_ok ? "OK" : "FAIL");

    /* Loop state */
    uint8_t     raw[NRF_PAYLOAD_SIZE];
    WL_Packet   pkt;
    uint16_t    servo_us[PCA_SERVO_COUNT];
    uint8_t     gesture_id;
    uint8_t     confidence;

    uint32_t    mcmd_ack    =   0;
    uint64_t    t_servo     =   now_us();
    uint64_t    t_hb        =   now_us();
    uint64_t    t_diag      =   now_us();
    uint64_t    t_thermal   =   now_us();
    uint64_t    t_reinit    =   0;
    uint32_t    i2c_err_streak  =   0;

    /* v2.3.3: runtime config snapshot. We read the IPC region into this
     * local copy at the top of every servo-tick (50 Hz), so updates from
     * cpcu_kernel's SIGHUP reload show up within ~20 ms and there's no
     * risk of a torn read mid-write. The seqlock retry logic is in
     * IPC_ReadRuntimeConfig itself.
     *
     * If the read fails (writer is mid-update or magic invalid), the
     * cache keeps its previous values — better to use slightly stale
     * config than zero-filled garbage. */
    IPC_RuntimeConfig cfg_cache;
    if(!IPC_ReadRuntimeConfig(&ipc, &cfg_cache))
    {
        /* Initial read failed — fall back to compile-time defaults so
         * we have *something* sane while the kernel re-tries. */
        CFG_Defaults(&cfg_cache);
        LOG_W("IO", "initial config read failed, using defaults");
    }

    /*---------------------------------------------------------------------------*/

    while(g_run)
    {
        uint64_t t  =   now_us();

        /* 1. NRF poll */
        if(nrf_ok && NRF_DataAvailable(&nrf))
        {
            while(NRF_ReadPayload(&nrf, raw) == NRF_OK)
            {
                WL_Unpack(raw, &pkt);

                uint32_t gap    =   SAFETY_SeqGap(&safety, pkt.seq);
                if(gap > 0)
                {
                    atomic_fetch_add(&ipc.diag->io_seq_gaps, gap);
                }

                SAFETY_FeedPacket(&safety, &pkt, t);
                IPC_PushSensor(&ipc, &pkt, t);
                atomic_fetch_add(&ipc.diag->io_pkts_received, 1);
            }

            /* Belt-and-braces: clear RX_DR (bit 6) if it's still asserted.
             * NRF_ReadPayload already clears it, but this guards against
             * a stuck FIFO ISR from a spurious glitch. */
            uint8_t sr = NRF_GetStatus(&nrf);
            if(sr & 0x40)
                NRF_ClearIRQ(&nrf, 0x40);
        }

        /* 2. Safety: radio timeout */
        SAFETY_CheckTimeout(&safety, t);

        /* 3. Safety: DSP stall check */
        SAFETY_CheckDSP(&safety, t);

        /* 4. Safety: ring buffer overflow (v2.3 — recoverable) */
        SAFETY_FeedRingOverflow(&safety, atomic_load(&ipc.diag->io_ring_overflows));

        /* 5. Safety: drive non-radio FSM transitions (v2.3 — was dead in v2.2).
         * UpdateState owns the RUNNING <-> SAFE transitions for battery /
         * thermal / dsp / i2c / ring fault flags. Without this call the
         * boolean flags would update correctly (so SAFETY_CheckSystem()
         * already neutralises the servos), but the FSM state shown in the
         * TUI would stay RUNNING through any non-radio fault. */
        SAFETY_UpdateState(&safety, t);

        /* 6. Update system state in shared memory */
        atomic_store(&ipc.ctrl->system_state,
                     safety.state == RADIO_SAFE ?   IPC_STATE_SAFE :
                     safety.state == RADIO_RUNNING  ?   IPC_STATE_RUNNING : IPC_STATE_INIT);

        /* 7. NRF re-init if failed (full clean-reset sequence) */
        if(!nrf_ok && (t - t_reinit) > NRF_REINIT_INTERVAL_US)
        {
            t_reinit    =   t;
            LOG_W("NRF", "attempting recovery...");

            /* If we still have an SPI handle, drain FIFOs and power down
             * cleanly before re-initing. Wires in NRF_FlushRX/TX/PowerDown
             * which were previously dead code. */
            NRF_FlushRX(&nrf);
            NRF_FlushTX(&nrf);
            NRF_ClearIRQ(&nrf, 0x70);       /* all IRQ flags */
            NRF_PowerDown(&nrf);
            sleep_ms(50);

            nrf_ret     =   NRF_Init(&nrf, SPI_DEVICE, SPI_SPEED,
                                    gpio_fd, GPIO_CE, NRF_CHANNEL, addr);

            atomic_store(&ipc.diag->io_nrf_init_status, (uint32_t)nrf_ret);
            if(nrf_ret == NRF_OK)
            {
                nrf_ok = true;
                LOG_I("NRF", "recovered");
            }
            else
            {
                LOG_E("NRF", "recovery failed (%d)", nrf_ret);
            }
        }

        /* 8. Servo update (50 Hz with smoothing) */
        if((t - t_servo) >= SERVO_INTERVAL_US)
        {
            uint32_t servo_dt   =   (uint32_t)(t - t_servo);
            t_servo =   t;

            if(SAFETY_CheckSystem(&safety) && pca_ok)
            {
                if(IPC_ReadMotorCmd(&ipc, servo_us, &gesture_id, &confidence, &mcmd_ack))
                {
                    /* Notify safety: DSP is alive */
                    SAFETY_FeedMotorCMD(&safety, t);

                    /* Clamp targets to safe range, feed to smoother */
                    PCA_SafetyClamp(&pca, servo_us);
                    SMOOTH_SetAllTargets(&smooth, servo_us);
                }

                /* Advance smoother toward targets */
                SMOOTH_Update(&smooth, servo_dt);

                /* v2.3.3: refresh runtime config snapshot. If the kernel
                 * just reloaded JSON (SIGHUP), this picks up the new
                 * values within one servo tick. Failed reads (writer
                 * mid-update) leave cfg_cache untouched. */
                IPC_ReadRuntimeConfig(&ipc, &cfg_cache);

                /* Write smoothed positions to servos (one I2C burst per
                 * channel). v2.3.2: gate each write on SMOOTH_ShouldWrite
                 * to suppress redundant refreshes of settled servos —
                 * see docs/JITTER_MITIGATION.md. v2.3.3: apply per-servo
                 * bias offset (gravity-sag compensation) to the smoothed
                 * value before clamping & write — see RUNTIME_CONFIG.md.
                 * Servos still in motion, or sitting outside the deadband
                 * from their last latched value, write every tick as before. */
                PCA_Status pca_ret  =   PCA_OK;
                bool       any_io   =   false;
                for(int s = 0; s < PCA_SERVO_COUNT; s++)
                {
                    if(!SMOOTH_ShouldWrite(&smooth, s))
                        continue;       /* deadband — let the servo coast */

                    /* v2.3.3: bias is signed (typ. +/- 20 us). Clamp the
                     * BIASED value to compile-time hardware limits — the
                     * runtime config can't escape the safety envelope. */
                    int32_t biased = (int32_t)smooth.current[s] +
                                     (int32_t)cfg_cache.servo_bias_us[s];
                    if(biased < (int32_t)pca.servo_min[s]) biased = pca.servo_min[s];
                    if(biased > (int32_t)pca.servo_max[s]) biased = pca.servo_max[s];
                    uint16_t pulse_us = (uint16_t)biased;

                    PCA_Status r    =   PCA_SetServo(&pca, (uint8_t)s, pulse_us);
                    if(r == PCA_OK)
                    {
                        SMOOTH_MarkWritten(&smooth, s, pulse_us);
                    }
                    else
                    {
                        pca_ret =   r;
                    }
                    any_io = true;
                }

                /* Only feed I2C health if we actually performed I/O this
                 * tick. A pure-deadband tick (all servos settled, no
                 * writes) shouldn't be miscounted as either a success
                 * or a failure. */
                if(any_io)
                {
                    SAFETY_FeedI2C(&safety, pca_ret == PCA_OK);

                    if(pca_ret == PCA_OK)   i2c_err_streak  =   0;
                    else                    i2c_err_streak++;
                }

                /* If I2C has been failing for over 500 ms (25 ticks @ 50 Hz),
                 * assume the bus is wedged and kill all outputs with PCA_AllOff.
                 * This wires in PCA_AllOff from cpcu_io (previously only used
                 * in pca_testbench). */
                if(i2c_err_streak >= 25)
                {
                    LOG_E("PCA", "I2C streak %u — forcing AllOff",
                          i2c_err_streak);
                    PCA_AllOff(&pca);
                    i2c_err_streak = 0;

                    /* v2.3.2: AllOff clears the PCA's PWM registers, so
                     * what was "last_written" no longer reflects hardware.
                     * Reset the deadband shadow so the first write after
                     * the bus recovers always goes through. */
                    for(int s = 0; s < PCA_SERVO_COUNT; s++)
                        smooth.ever_written[s] = false;
                }
            }
            else if(pca_ok)
            {
                /* Safety triggered -> instant snap to neutral */
                for(int s = 0; s < PCA_SERVO_COUNT; s++)
                    smooth.target[s] = PCA_SERVO_NEUTRAL;
                SMOOTH_Snap(&smooth);

                PCA_SetAllNeutral(&pca);

                /* v2.3.2: keep the deadband shadow coherent with what's
                 * actually latched in the PCA. Without this, the next
                 * SMOOTH_ShouldWrite would see ever_written=true with
                 * last_written != neutral, and write another redundant
                 * neutral on the very next tick. */
                for(int s = 0; s < PCA_SERVO_COUNT; s++)
                    SMOOTH_MarkWritten(&smooth, s, PCA_SERVO_NEUTRAL);
            }
        }

        /* 9. Thermal check */
        if((t - t_thermal) >= THERMAL_INTERVAL_US)
        {
            t_thermal   =   t;
            float temp  =   read_cpu_temp();
            SAFETY_FeedTemperature(&safety, temp);
        }

        /* 10. Heartbeat */
        if((t - t_hb) >= HEARTBEAT_INTERVAL_US)
        {
            t_hb    =   t;
            atomic_store(&ipc.ctrl->io_heartbeat_us, t);
        }

        /* 11. Diagnostics (1 Hz — this is also the only file-log pressure point) */
        if((t - t_diag) >= DIAG_INTERVAL_US)
        {
            t_diag      =   t;
            uint8_t sr  =   nrf_ok ? NRF_GetStatus(&nrf) : 0xFF;

            LOG_I("IO",
                  "pkts=%u gaps=%u ring=%u state=%s fault=%s "
                  "link=%d batt=%.2fV temp=%.1fC nrf_sr=0x%02X motion=%s",
                  atomic_load(&ipc.diag->io_pkts_received),
                  atomic_load(&ipc.diag->io_seq_gaps),
                  IPC_SensorCount(&ipc),
                  SAFETY_RadioStr(safety.state),
                  SAFETY_StatusStr(safety.last_fault),
                  safety.link.quality,
                  safety.battery.voltage,
                  safety.thermal.temperature_c,
                  sr,
                  SMOOTH_AllSettled(&smooth) ? "IDLE" : "MOVING");
        }
    }

    /* Cleanup */
    LOG_I("IO", "shutting down");

    if(pca_ok)
    {
        /* Bring servos to a safe neutral pose first (so the hand doesn't
         * fall into an awkward position), then PCA_AllOff disables PWM
         * entirely to cut torque. */
        PCA_SetAllNeutral(&pca);
        sleep_ms(300);
        PCA_AllOff(&pca);
        PCA_Close(&pca);
    }

    if(nrf_ok)
    {
        NRF_PowerDown(&nrf);
        NRF_Close(&nrf);
    }

    IPC_Close(&ipc);
    close(gpio_fd);
    Log_CloseFiles();
    return 0;
}
