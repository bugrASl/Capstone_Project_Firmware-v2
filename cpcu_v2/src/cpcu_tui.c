/**
 *  @file       cpcu_tui.c
 *  @brief      Terminal User Interface — multi-page live system monitor.
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    3.3-dataset (build marker: DSET-APR24)
 *  @version    3.2-linetrace
 *  @version    3.1
 *
 *  @details    Reads /dev/shm/cpcu_ipc (read-only on pages 1-6) and displays
 *              a real-time dashboard with 7 switchable pages:
 *
 *              Page 1 (Overview)   :  System state, radio, EMG bars, servos,
 *                                     battery, DSP summary, ML classification.
 *              Page 2 (Radio/IO)   :  NRF deep-dive, safety FSM, packet stats,
 *                                     retry distribution, raw packet dump.
 *              Page 3 (DSP/AI)     :  DSP pipeline, gesture classification,
 *                                     per-class confidence, filtered RMS, servos.
 *              Page 4 (Waveforms)  :  Live 8-channel line-trace plots,
 *                                     per-channel Vpp/DC, detail view with TAB.
 *              Page 5 (Config)     :  Static compile-time + hardware spec sheet.
 *              Page 6 (Health)     :  10-row traffic-light rollup.
 *              Page 7 (Dataset)    :  v2.1 — capture 8-channel CSV recordings
 *                                     labelled by gesture. LEFT/RIGHT cycles
 *                                     the label, s/SPACE starts/stops, r
 *                                     cancels (deletes partial), t toggles
 *                                     RAW ADC ↔ FILTERED output. Works fully
 *                                     in --demo mode against synthetic data.
 *
 *              Supports --demo mode with synthetic data (no hardware needed).
 *              Pages 1-6 are read-only peek-access; page 7 opens a CSV file
 *              for writing when capture is armed.
 *              Runs on Core 0 alongside cpcu_kernel or via SSH.
 *
 *              v3.3 changes (2026-04):
 *                  - PAGE_DATASET (page 7): interactive EMG capture UI that
 *                    produces files byte-compatible with the output of
 *                    bsau_dataset_collector.py (RAW mode) or matching the
 *                    voltage-domain output of cpcu_dsp.py (FILTERED mode).
 *                  - Capture works in --demo mode against synthetic packets
 *                    (same codec path as real RX) so the DSP/AI team can
 *                    rehearse the workflow without a BSAU in the loop.
 *                  - Footer hotkey hint updated to "1-7:pg".
 *
 *              v3.2 changes (2026-04):
 *                  - Line-trace waveform renderer for Page 4 (see §draw_waveform).
 *
 *              v3.1 changes (2026-04):
 *                  - Full-screen dynamic layout — respects getmaxyx() on every
 *                    frame, so pages fill whatever terminal size you give them.
 *                  - Splash screen on startup (--no-splash to skip).
 *                  - Demo mode now produces 100 ring entries per 10 Hz tick,
 *                    i.e. a true 1 kHz synthetic packet stream, so Page 4
 *                    waveforms actually show sine waves in demo.
 *                  - Demo mode round-trips WL_Pack/WL_Unpack so it exercises
 *                    the same codec path the real RX uses.
 *
 *  Build:      gcc -o cpcu_tui cpcu_tui.c cpcu_ipc.c wireless_packet.c \
 *                  -lncurses -lrt -lm
 *  Run:        ./cpcu_tui                (live, needs cpcu_kernel)
 *              ./cpcu_tui --demo         (synthetic data, no hardware)
 *              ./cpcu_tui --no-splash    (skip splash screen)
 *  Pages:      1=Overview 2=Radio/IO 3=DSP/AI 4=Waves 5=Config 6=Health 7=Dataset
 *  Quit:       press 'q'
 */

#include <ncurses.h>
#include <locale.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "cpcu_ipc.h"
#include "wireless_packet.h"
#include "demo_signals.h"

/*============= COLOR PAIRS ================================================================*/

#define CP_NORMAL       1
#define CP_GOOD         2
#define CP_WARN         3
#define CP_BAD          4
#define CP_CYAN         5
#define CP_DIM          6
#define CP_HEADER       7
#define CP_BAR_FILL     8
#define CP_BAR_EMPTY    9
#define CP_MAGENTA      10

/*============= DYNAMIC LAYOUT =============================================================*/

/* Minimum width we can sensibly draw into; below this we crop to what fits. */
#define TUI_MIN_WIDTH       72
#define TUI_MIN_HEIGHT      24
#define REFRESH_US          100000      /* 10 Hz */

/* Populated by layout_update() at the top of every frame. */
static int  g_term_w    =   80;
static int  g_term_h    =   24;
static int  g_tui_w     =   76;         /* min(g_term_w, sensible max)       */
static int  g_col_r     =   39;         /* right-column x = g_tui_w/2        */
static int  g_bar_w     =   20;         /* scaled with width                 */
static int  g_slider_w  =   20;

static void layout_update(void)
{
    getmaxyx(stdscr, g_term_h, g_term_w);
    g_tui_w     =   g_term_w;
    if(g_tui_w  <   TUI_MIN_WIDTH)      g_tui_w = TUI_MIN_WIDTH;
    g_col_r     =   g_tui_w / 2;
    /* Bars and sliders scale with width: roughly quarter of the screen */
    g_bar_w     =   (g_col_r - 8);
    if(g_bar_w  <   14) g_bar_w = 14;
    if(g_bar_w  >   32) g_bar_w = 32;
    g_slider_w  =   g_bar_w;
}

/*============= SERVO CONFIG ===============================================================*/

#define SERVO_COUNT     PCA_SERVO_COUNT

static const char *SERVO_NAMES[]    =   { "Base", "Upper", "Last", "Jnt-1", "Jnt-2", "Grip" };
static const uint16_t SERVO_MIN[]   =   {  498, 1074, 1074, 1001, 1001,  976 };
static const uint16_t SERVO_MAX[]   =   { 2500, 1953, 1953, 2002, 2002, 1733 };

static const char *CLS_NAMES[]      =   {
    "REST",   "H.SLO",  "H.HRD",  "H.OPN",  "A.BND<",
    "A.BND=", "A.BND>", "A.SLO",  "A.FST",  "BICEP"
};

/*============= PAGES ======================================================================*/

typedef enum {
    PAGE_OVERVIEW = 0,
    PAGE_RADIO,
    PAGE_DSP,
    PAGE_WAVES,
    PAGE_CONFIG,
    PAGE_HEALTH,
    PAGE_DATASET,
    PAGE_COUNT
} Page;

static const char *PAGE_TITLES[] = {
    "OVERVIEW",
    "RADIO/IO",
    "DSP/AI",
    "WAVES",
    "CONFIG",
    "HEALTH",
    "DATASET",
};

/*============= GLOBALS ====================================================================*/

static volatile sig_atomic_t g_run  =   1;
static void on_sig(int s) { (void)s; g_run = 0; }

static Page     current_page    =   PAGE_OVERVIEW;
static bool     demo_mode       =   false;
static bool     show_splash     =   true;

/*============= DEMO STATE =================================================================*/

static uint32_t demo_pkts       =   0;
static uint32_t demo_gaps       =   0;
static double   demo_phase      =   0.0;
static uint32_t demo_inf_count  =   0;

/*----- DEMO MODE FAULT INJECTION -----------------------------------------
 *  Keybinds available in --demo mode to exercise the safety FSM:
 *    F  freeze radio (stop feeding packets → silence → DEGRADED → SAFE)
 *    B  low battery  (inject vbat_raw below critical threshold)
 *    G  sequence-gap storm (burst of missed-seq events)
 *    O  ring overflow (DSP can't keep up)
 *    I  I2C error streak (PCA9685 write failures)
 *    R  reset — clear all injected faults, return to healthy state
 *
 *  Each fault only manipulates the shared-memory state the existing
 *  Page 1/2 widgets already display, so the fault's visible effect is
 *  whatever cpcu_tui normally shows for that condition. The real
 *  cpcu_safety C module is exercised by safety_testbench (automated).
 */
typedef enum {
    FAULT_NONE          = 0,
    FAULT_RADIO_FREEZE  = 1 << 0,
    FAULT_BATT_LOW      = 1 << 1,
    FAULT_GAP_STORM     = 1 << 2,
    FAULT_RING_OVF      = 1 << 3,
    FAULT_I2C_FAIL      = 1 << 4,
} DemoFault;

static uint32_t  demo_fault_mask       =   FAULT_NONE;
static uint64_t  demo_fault_onset_ms   =   0;   /* when FAULT_RADIO_FREEZE was pressed */

/*----- DEMO MODE WAVEFORM SELECTION --------------------------------------
 *  In --demo mode these can be changed live via hotkeys:
 *    w / W    Cycle through SINE → SQUARE → TRI → SAW → NOISE → EMG →
 *             ECG → CHIRP → SINE (wraps)
 *    [        Halve frequency  (minimum 10 Hz)
 *    ]        Double frequency (maximum 1000 Hz)
 */
static DemoWave  demo_wave             =   WAVE_SINE;
static float     demo_freq_hz          =   100.0f;

static const char *fault_banner(uint32_t mask)
{
    /* Return a single short label for the footer. Priority: radio > batt > others */
    if(mask & FAULT_RADIO_FREEZE)   return "[INJ:RADIO_FREEZE]";
    if(mask & FAULT_BATT_LOW)       return "[INJ:BATT_LOW]";
    if(mask & FAULT_GAP_STORM)      return "[INJ:GAP_STORM]";
    if(mask & FAULT_RING_OVF)       return "[INJ:RING_OVF]";
    if(mask & FAULT_I2C_FAIL)       return "[INJ:I2C_FAIL]";
    return NULL;
}

static uint64_t now_ms_wall(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000);
}
static uint8_t  demo_gesture    =   1;      /* HAND_SLOW */
static uint8_t  demo_conf       =   94;

/* Demo IPC regions (fake, in-process) */
static IPC_ControlBlock  demo_ctrl;
static IPC_SensorEntry   demo_ring[IPC_SENSOR_RING_SIZE];
static IPC_MotorCommand  demo_motor;
static IPC_Diagnostics   demo_diag;
static IPC_DSPExport     demo_dsp_export;

static void demo_init(IPC_Context *ipc)
{
    memset(&demo_ctrl, 0, sizeof(demo_ctrl));
    memset(demo_ring, 0, sizeof(demo_ring));
    memset(&demo_motor, 0, sizeof(demo_motor));
    memset(&demo_diag, 0, sizeof(demo_diag));
    memset(&demo_dsp_export, 0, sizeof(demo_dsp_export));

    demo_ctrl.magic         =   IPC_MAGIC;
    demo_ctrl.version       =   IPC_VERSION;
    atomic_store(&demo_ctrl.io_ready, 1);
    atomic_store(&demo_ctrl.dsp_ready, 1);
    atomic_store(&demo_ctrl.system_state, IPC_STATE_RUNNING);

    ipc->ctrl       =   &demo_ctrl;
    ipc->ring       =   demo_ring;
    ipc->motor      =   &demo_motor;
    ipc->diag       =   &demo_diag;
    ipc->dsp_export =   &demo_dsp_export;
}

/**
 *  Build one synthetic packet, pack it to wire format with WL_Pack, then
 *  unpack with WL_Unpack into the ring. This deliberately exercises the
 *  codec so demo mode represents the real RX path end-to-end (and so
 *  WL_Pack gets coverage on the CPCU side — it's otherwise only called
 *  on the BSAU).
 */
static void demo_push_packet(IPC_Context *ipc, uint32_t p_idx)
{
    WL_Packet pkt;
    memset(&pkt, 0, sizeof(pkt));

    for(int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
    {
        for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
        {
            /* Call the shared generator from demo_signals.h. Each channel
             * gets a small phase offset so all 8 don't stack perfectly —
             * that's what real per-electrode variation looks like. */
            float phase_off = (float)ch * 0.2f;
            float v = demo_gen(demo_wave, (float)demo_phase, demo_freq_hz, phase_off);

            /* Add tiny ADC quantisation noise so the trace doesn't look
             * mathematically perfect */
            float noise = ((float)(rand() % 5) - 2.0f) / 4095.0f * 3.3f;
            v += noise;
            if(v < 0.0f)  v = 0.0f;
            if(v > 3.3f)  v = 3.3f;
            pkt.samples[s].ch[ch] = (uint16_t)(v / 3.3f * 4095.0f);
        }
        demo_phase += 1.0 / 2000.0;   /* 2 kHz sample rate */
    }
    pkt.vbat_raw    =   4031;
    pkt.flags       =   0;
    pkt.seq         =   (uint8_t)((demo_pkts + p_idx) & 0xFF);
    pkt.tx_retry    =   0;
    pkt.pkt_loss    =   0;
    pkt.timestamp   =   (uint16_t)((demo_pkts + p_idx) & 0xFFFF);

    /* Round-trip through the wire codec */
    uint8_t raw[WL_PAYLOAD_SIZE];
    WL_Pack(&pkt, raw);
    WL_Packet rebuilt;
    WL_Unpack(raw, &rebuilt);

    /* Deposit the decoded packet in the fake ring at ctrl->sensor_head */
    uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
    IPC_SensorEntry *e = &ipc->ring[head & IPC_SENSOR_RING_MASK];
    memcpy(e->samples,  rebuilt.samples, sizeof(e->samples));
    e->vbat_raw     =   rebuilt.vbat_raw;
    e->flags        =   rebuilt.flags;
    e->seq          =   rebuilt.seq;
    e->tx_retry     =   rebuilt.tx_retry;
    e->pkt_loss     =   rebuilt.pkt_loss;
    e->timestamp    =   rebuilt.timestamp;
    atomic_store(&ipc->ctrl->sensor_head, head + 1);
}

static void demo_tick(IPC_Context *ipc)
{
    uint64_t now = now_ms_wall();

    /*---- Apply fault injections ----*/
    bool radio_silent      = (demo_fault_mask & FAULT_RADIO_FREEZE) != 0;
    bool batt_low          = (demo_fault_mask & FAULT_BATT_LOW)     != 0;
    bool inject_gap_storm  = (demo_fault_mask & FAULT_GAP_STORM)    != 0;
    bool inject_ring_ovf   = (demo_fault_mask & FAULT_RING_OVF)     != 0;
    bool inject_i2c_fail   = (demo_fault_mask & FAULT_I2C_FAIL)     != 0;

    /* Radio freeze: stop feeding packets. Transition state based on how
     * long the freeze has been active (mirrors SAFETY_RADIO_TIMEOUT_MS +
     * SAFETY_RADIO_SAFE_MS thresholds from cpcu_safety.h). */
    if(radio_silent)
    {
        uint64_t silence_ms = now - demo_fault_onset_ms;
        if(silence_ms > 2250)           /* 750 timeout + 1500 in degraded */
            atomic_store(&ipc->ctrl->system_state, IPC_STATE_SAFE);
        else if(silence_ms > 750)
            atomic_store(&ipc->ctrl->system_state, IPC_STATE_INIT);  /* DEGRADED */
        /* No packets this tick. */
    }
    else
    {
        /* Healthy or non-radio fault: pump 100 packets */
        uint16_t vbat = batt_low ? 1600 : 4031;     /* ~2.58V : ~6.50V */
        for(uint32_t p = 0; p < 100; p++)
        {
            /* Patch vbat via demo_push_packet's constant by temporarily
             * manipulating the shared state post-push: simplest is to
             * rewrite the ring entry after the push. */
            demo_push_packet(ipc, p);
            uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
            if(head > 0) {
                IPC_SensorEntry *e = &ipc->ring[(head - 1) & IPC_SENSOR_RING_MASK];
                e->vbat_raw = vbat;
            }
        }
        demo_pkts += 100;

        /* Gap storm: every tick add ~10 gap events */
        if(inject_gap_storm)
            demo_gaps += 10;

        /* Battery low → SAFE */
        if(batt_low)
            atomic_store(&ipc->ctrl->system_state, IPC_STATE_SAFE);
        else if(inject_i2c_fail || inject_ring_ovf)
            atomic_store(&ipc->ctrl->system_state, IPC_STATE_SAFE);
        else
            atomic_store(&ipc->ctrl->system_state, IPC_STATE_RUNNING);
    }

    demo_inf_count++;

    /* Cycle gesture every 5 seconds (only when RUNNING) */
    if(atomic_load(&ipc->ctrl->system_state) == IPC_STATE_RUNNING
       && demo_inf_count % 50 == 0)
        demo_gesture = (demo_gesture + 1) % 10;

    atomic_store(&ipc->diag->io_pkts_received, demo_pkts);
    atomic_store(&ipc->diag->io_seq_gaps, demo_gaps);
    atomic_store(&ipc->diag->io_nrf_init_status, 0);

    /* Ring overflow: push the diag counter past SAFETY threshold (100) */
    atomic_store(&ipc->diag->io_ring_overflows,
                 inject_ring_ovf ? 150U : 0U);

    atomic_store(&ipc->diag->dsp_inferences, demo_inf_count);
    atomic_store(&ipc->diag->dsp_batches, demo_inf_count * 2);
    atomic_store(&ipc->diag->dsp_max_latency_us, 3200);

    /* Synthetic motor command: when SAFE, servos snap to neutral — same
     * behaviour the real cpcu_io enforces on SAFE entry. */
    ipc->motor->gesture_id      =   demo_gesture;
    ipc->motor->confidence      =   demo_conf;
    for(int i = 0; i < IPC_NUM_SERVOS; i++)
        ipc->motor->servo_us[i] =   1500;

    /* Synthetic DSP export */
    atomic_store(&ipc->dsp_export->update_seq, demo_inf_count);
    ipc->dsp_export->num_classes    =   10;
    ipc->dsp_export->active_class   =   demo_gesture;
    ipc->dsp_export->inference_time_us = 2800;
    snprintf((char *)ipc->dsp_export->gesture_name, IPC_MAX_GESTURE_NAME,
             "%s", CLS_NAMES[demo_gesture]);
    for(int c = 0; c < 10; c++)
        ipc->dsp_export->class_confidence[c] = (c == demo_gesture) ? 0.94f : 0.01f;
    for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
        ipc->dsp_export->channel_rms[ch] = 0.28f + 0.04f * (ch % 3);
}

/*============= DATASET CAPTURE (Page 7) ===================================================
 *
 *  Interactive 8-channel EMG recorder. Produces one CSV file per capture
 *  under DATASET_OUT_DIR, auto-incrementing the trailing index so nothing
 *  ever overwrites:
 *
 *      ./datasets/REST_0.csv
 *      ./datasets/REST_1.csv    (next run, same label)
 *      ./datasets/H_OPN_0.csv   (different label, index resets)
 *
 *  RAW mode      : 8 comma-separated uint16 ADC readings per row,
 *                  byte-compatible with bsau_dataset_collector.py output.
 *  FILTERED mode : 8 comma-separated floats (volts) after the same
 *                  high-pass / low-pass cascade the production DSP uses —
 *                  see DatasetFilter below. Approximates cpcu_dsp.py for
 *                  ground-truth comparison without requiring Python to
 *                  be in the loop.
 *
 *  Demo mode: the ring is fed by demo_push_packet() at ~1 kHz with
 *  synthetic packets that have already round-tripped WL_Pack/WL_Unpack,
 *  so the capture path is indistinguishable from the real RX path.
 *  This lets the whole workflow be rehearsed without a BSAU attached.
 *
 *  Reads from the sensor ring WITHOUT advancing sensor_tail (peek-only,
 *  same discipline as Page 4) — so page 7 never steals entries from the
 *  real consumer (cpcu_dsp.py) when running live.
 *===========================================================================================*/

#define DATASET_OUT_DIR     "./datasets"
#define DATASET_LABEL_COUNT 10
#define DATASET_PATH_MAX    256
#define DATASET_LINE_MAX    192     /* 8 cols * ~20 chars + commas + CRLF */

typedef enum {
    DS_IDLE = 0,        /* Waiting for user to pick a label and press s */
    DS_COLLECTING,      /* Writing to ds_file; counters tick up */
    DS_SAVED,           /* Transient (~2 s) 'SAVED' banner after stop */
    DS_CANCELLED,       /* Transient 'CANCELLED' banner after r */
} DatasetState;

typedef enum {
    DS_MODE_FILTERED = 0,
    DS_MODE_RAW,
} DatasetMode;

/*----- One-pole IIR cascade. Crude but cheap alternative to scipy sosfilt.
 *      HP at 20 Hz (DC removal)  +  LP at 450 Hz (band limit).  Applied
 *      per channel in filtered mode; state lives in DatasetFilter. ------*/
typedef struct {
    double hp_prev_x;       /* Input memory for high-pass         */
    double hp_prev_y;       /* Output memory for high-pass        */
    double lp_prev_y;       /* Output memory for low-pass         */
} DatasetFilter;

/* alpha values for Fs=2000 Hz; see design note at filter apply site. */
#define DS_HP_ALPHA         0.9409      /* ~20 Hz HP   */
#define DS_LP_ALPHA         0.7520      /* ~450 Hz LP  */

static DatasetState ds_state        =   DS_IDLE;
static DatasetMode  ds_mode         =   DS_MODE_FILTERED;
static int          ds_label_idx    =   0;
static FILE        *ds_file         =   NULL;
static char         ds_path[DATASET_PATH_MAX] = {0};
static uint32_t     ds_samples      =   0;
static uint32_t     ds_gaps         =   0;
static uint32_t     ds_missed       =   0;
static uint32_t     ds_last_head    =   0;
static uint64_t     ds_start_ms     =   0;
static uint64_t     ds_msg_until    =   0;
static uint8_t      ds_prev_seq     =   0;
static bool         ds_seq_seeded   =   false;
static DatasetFilter ds_filter[WL_NUM_CHANNELS];

/**
 *  Sanitise a label to a filesystem-safe stem. Must match the mapping in
 *  bsau_dataset_collector.py's sanitize_label() so that a BSAU-side UART
 *  capture and a CPCU-side radio capture land on filenames that share
 *  the same stem.
 */
static void ds_sanitize(const char *in, char *out, size_t outsz)
{
    size_t j = 0;
    for(size_t i = 0; in[i] && j + 4 < outsz; i++)
    {
        char c = in[i];
        if(c == '.')            { if(j + 1 < outsz) out[j++] = '_'; }
        else if(c == '<')       { if(j + 3 < outsz) { out[j++]='_'; out[j++]='L'; out[j++]='T'; } }
        else if(c == '>')       { if(j + 3 < outsz) { out[j++]='_'; out[j++]='G'; out[j++]='T'; } }
        else if(c == '=')       { if(j + 3 < outsz) { out[j++]='_'; out[j++]='E'; out[j++]='Q'; } }
        else if(isalnum((unsigned char)c) || c == '_' || c == '-')
                                { out[j++] = c; }
        /* else: drop character */
    }
    if(j == 0)
    {
        const char *fallback = "unlabeled";
        for(size_t k = 0; fallback[k] && j + 1 < outsz; k++) out[j++] = fallback[k];
    }
    out[j] = '\0';
}

/**
 *  Scan DATASET_OUT_DIR for files matching "{stem}_N.csv", return max N + 1.
 *  Returns 0 if the directory doesn't exist or contains no matching files.
 */
static int ds_next_index(const char *out_dir, const char *stem)
{
    DIR *d = opendir(out_dir);
    if(!d) return 0;

    size_t stem_len = strlen(stem);
    int max_n = -1;
    struct dirent *ent;
    while((ent = readdir(d)) != NULL)
    {
        const char *name = ent->d_name;
        if(strncmp(name, stem, stem_len) != 0) continue;
        if(name[stem_len] != '_') continue;

        /* Expect "N.csv" after the underscore. */
        const char *nstr = name + stem_len + 1;
        char *endp = NULL;
        long n = strtol(nstr, &endp, 10);
        if(endp == nstr) continue;
        if(strcmp(endp, ".csv") != 0) continue;
        if(n > max_n) max_n = (int)n;
    }
    closedir(d);
    return max_n + 1;
}

/**
 *  Arm capture. Builds the next free filename, opens the file, zeroes
 *  counters, seeds filter state. Returns 0 on success, -1 on open failure.
 */
static int ds_start_capture(IPC_Context *ipc)
{
    char stem[32];
    ds_sanitize(CLS_NAMES[ds_label_idx], stem, sizeof(stem));

    /* Create output directory if missing. mkdir() returns -1/EEXIST if
     * it already exists, which is fine. */
    if(mkdir(DATASET_OUT_DIR, 0755) != 0 && errno != EEXIST)
    {
        /* Directory creation failed for some other reason — caller will
         * see the open() failure below, which is enough signal. */
    }

    int idx = ds_next_index(DATASET_OUT_DIR, stem);
    snprintf(ds_path, sizeof(ds_path), "%s/%s_%d.csv",
             DATASET_OUT_DIR, stem, idx);

    ds_file = fopen(ds_path, "w");
    if(!ds_file)
    {
        ds_path[0] = '\0';
        return -1;
    }

    /* Line-buffer so tail -f works while capturing. */
    setvbuf(ds_file, NULL, _IOLBF, 0);

    ds_samples      = 0;
    ds_gaps         = 0;
    ds_missed       = 0;
    ds_seq_seeded   = false;
    ds_start_ms     = now_ms_wall();

    /* Start draining from the current ring head so we don't replay old entries. */
    ds_last_head    = atomic_load(&ipc->ctrl->sensor_head);

    memset(ds_filter, 0, sizeof(ds_filter));

    ds_state = DS_COLLECTING;
    return 0;
}

/**
 *  Stop capture. `save = true`  → close file, report SAVED banner.
 *                `save = false` → close file, unlink the partial, CANCELLED banner.
 */
static void ds_stop_capture(bool save)
{
    if(ds_file)
    {
        fflush(ds_file);
        fclose(ds_file);
        ds_file = NULL;
    }

    if(!save && ds_path[0])
    {
        /* Throw away the partial file. remove() is best-effort; if it
         * fails the user can delete it manually — we don't surface the
         * error in the UI because the banner is already transient. */
        (void)remove(ds_path);
    }

    ds_state     = save ? DS_SAVED : DS_CANCELLED;
    ds_msg_until = now_ms_wall() + 2000;    /* banner visible for 2 s */
}

/**
 *  Apply the HP+LP cascade to one sample on one channel. The alphas are
 *  tuned for Fs = 2000 Hz, cut-offs ~20 Hz (HP) and ~450 Hz (LP), which
 *  approximates the scipy 4th-order Butterworth 20-450 Hz bandpass used
 *  in cpcu_dsp.py. A true SOS cascade would match better, but this is
 *  good enough for offline CSV inspection — and a drop-in replacement if
 *  anyone wants to paste real biquad coefficients in later.
 */
static double ds_filter_step(DatasetFilter *f, double x)
{
    /* High-pass: y[n] = alpha * (y[n-1] + x[n] - x[n-1]) */
    double y_hp = DS_HP_ALPHA * (f->hp_prev_y + x - f->hp_prev_x);
    f->hp_prev_x = x;
    f->hp_prev_y = y_hp;

    /* Low-pass: y[n] = alpha * y[n-1] + (1-alpha) * x[n] */
    double y_lp = DS_LP_ALPHA * f->lp_prev_y + (1.0 - DS_LP_ALPHA) * y_hp;
    f->lp_prev_y = y_lp;

    return y_lp;
}

/**
 *  Walk the ring from ds_last_head to the current head, writing each
 *  sample-pair to the CSV file. Safe to call while not collecting (it's
 *  a no-op). Never advances sensor_tail — that's cpcu_dsp.py's job.
 */
static void ds_drain_ring_to_file(IPC_Context *ipc)
{
    if(ds_state != DS_COLLECTING || !ds_file) return;

    uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
    uint32_t new_entries = head - ds_last_head;

    if(new_entries == 0) return;

    /* If the producer lapped us, we lost ring-capacity worth of entries. */
    if(new_entries > IPC_SENSOR_RING_SIZE)
    {
        ds_missed   += new_entries - IPC_SENSOR_RING_SIZE;
        new_entries  = IPC_SENSOR_RING_SIZE;
        /* Skip ahead so subsequent iterations see the freshest entries. */
        ds_last_head = head - IPC_SENSOR_RING_SIZE;
    }

    char line[DATASET_LINE_MAX];

    for(uint32_t i = 0; i < new_entries; i++)
    {
        uint32_t idx = (ds_last_head + i) & IPC_SENSOR_RING_MASK;
        IPC_SensorEntry *e = &ipc->ring[idx];

        /* Track sequence jumps. Each CPCU ring entry corresponds to one
         * BSAU packet; seq increments by 1 per packet (mod 256). */
        if(ds_seq_seeded)
        {
            uint8_t expected = (uint8_t)(ds_prev_seq + 1);
            if(e->seq != expected)
            {
                /* Forward gap count, wrap-safe. */
                uint8_t delta = (uint8_t)(e->seq - expected);
                ds_gaps += delta;
            }
        }
        ds_prev_seq   = e->seq;
        ds_seq_seeded = true;

        for(int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
        {
            int n = 0;
            if(ds_mode == DS_MODE_RAW)
            {
                /* Raw 12-bit ADC, unsigned. Byte-compatible with BSAU UART
                 * collector output. */
                n = snprintf(line, sizeof(line),
                             "%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                             (unsigned)e->samples[s].ch[0],
                             (unsigned)e->samples[s].ch[1],
                             (unsigned)e->samples[s].ch[2],
                             (unsigned)e->samples[s].ch[3],
                             (unsigned)e->samples[s].ch[4],
                             (unsigned)e->samples[s].ch[5],
                             (unsigned)e->samples[s].ch[6],
                             (unsigned)e->samples[s].ch[7]);
            }
            else
            {
                /* Filtered volts. Convert raw ADC -> centered voltage,
                 * then cascade HP+LP per channel. */
                double v[WL_NUM_CHANNELS];
                for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
                {
                    double raw   = (double)e->samples[s].ch[ch];
                    double volts = raw * 3.3 / 4095.0 - 1.65;
                    v[ch]        = ds_filter_step(&ds_filter[ch], volts);
                }
                n = snprintf(line, sizeof(line),
                             "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\r\n",
                             v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
            }

            if(n > 0 && (size_t)n < sizeof(line))
            {
                if(fwrite(line, 1, (size_t)n, ds_file) == (size_t)n)
                    ds_samples++;
            }
        }
    }

    ds_last_head = head;
}

/*============= WAVEFORM BUFFER (Page 4) ===================================================*/

#define WAVE_BUF_SIZE   512         /* Rolling samples per channel */
#define WAVE_PLOT_H     8           /* Rows per mini-plot (all-ch)  */
#define WAVE_PLOT_H_BIG 14          /* Rows for single-ch detail    */
#define WAVE_ADC_MAX    4095.0f

/* Waveform plot dimensions and auto-scale constants */

static uint16_t wave_buf[WL_NUM_CHANNELS][WAVE_BUF_SIZE];
static uint32_t wave_wr       = 0;
static uint32_t wave_count    = 0;
static uint32_t wave_last_head = 0;
static int      wave_sel_ch   = 0;
static bool     wave_detail   = false;

static void wave_peek_ring(IPC_Context *ipc)
{
    uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
    uint32_t new_entries = head - wave_last_head;
    if(new_entries == 0) return;
    if(new_entries > IPC_SENSOR_RING_SIZE) new_entries = IPC_SENSOR_RING_SIZE;

    uint32_t start = wave_last_head;
    for(uint32_t i = 0; i < new_entries; i++)
    {
        uint32_t idx = (start + i) & IPC_SENSOR_RING_MASK;
        IPC_SensorEntry *e = &ipc->ring[idx];

        for(int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
        {
            for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
                wave_buf[ch][wave_wr] = e->samples[s].ch[ch];

            wave_wr = (wave_wr + 1) % WAVE_BUF_SIZE;
            if(wave_count < WAVE_BUF_SIZE) wave_count++;
        }
    }
    wave_last_head = head;
}

/**
 *  Line-trace waveform renderer — one glyph per column on the sample row,
 *  plus connector chars between adjacent samples so the trace reads as a
 *  continuous curve the way a scope trace does.
 *
 *  Sub-cell resolution: within the single row a sample falls in we pick
 *  one of three glyphs based on where inside the row the sample sits —
 *  '.' (upper third), '-' (middle), 'o' (lower third) — giving roughly
 *  3x vertical resolution without the area-fill clutter.
 *
 *  Connectors ('/' '\' '|' '-') fill the gap between two adjacent samples
 *  that are at different rows so the eye follows the curve.
 *
 *  Works identically on narrow libncurses and wide libncursesw — every
 *  glyph is a single 7-bit ASCII character.
 */
static void draw_waveform(int row, int col, int width, int height,
                          int ch_idx, int color_pair)
{
    if(wave_count < 2) return;

    /* Auto-scale to the channel's actual min/max */
    uint16_t vmin = 4095, vmax = 0;
    uint32_t avail = (wave_count < WAVE_BUF_SIZE) ? wave_count : WAVE_BUF_SIZE;
    for(uint32_t i = 0; i < avail; i++)
    {
        uint16_t v = wave_buf[ch_idx][i];
        if(v < vmin) vmin = v;
        if(v > vmax) vmax = v;
    }
    if(vmax <= vmin) vmax = vmin + 1;

    /* Axes */
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    for(int y = 0; y < height; y++)
        mvaddch(row + y, col - 1, ACS_VLINE);
    mvhline(row + height, col, ACS_HLINE, width);
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);

    uint32_t show_n = (avail < (uint32_t)width) ? avail : (uint32_t)width;
    if(show_n < 2) return;

    attron(COLOR_PAIR(color_pair));

    /* Total sub-row resolution: 5 sub-cells per row × height rows.
     * Glyph picks a visual height inside the cell:
     *    rem 0  → "'"   (top ~80-100%)
     *    rem 1  → "`"   (upper)
     *    rem 2  → "-"   (middle)
     *    rem 3  → "."   (lower)
     *    rem 4  → ","   (bottom ~0-20%)
     */
    int total_sub = height * 5;

    /* Map a sample value → (row, sub_in_row) where row is the ncurses
     * row index counting from the top of the plot (row = 0 at top). */
    int prev_row = -1;
    int prev_sub = -1;

    for(uint32_t x = 0; x < show_n; x++)
    {
        uint32_t buf_off = (avail * x) / show_n;
        uint32_t idx = (wave_wr + WAVE_BUF_SIZE - avail + buf_off) % WAVE_BUF_SIZE;
        uint16_t val = wave_buf[ch_idx][idx];

        float frac = (float)(val - vmin) / (float)(vmax - vmin);
        if(frac < 0.0f) frac = 0.0f;
        if(frac > 1.0f) frac = 1.0f;

        /* sub position from the top, 0..total_sub-1 */
        int sub = (int)((1.0f - frac) * (float)(total_sub - 1) + 0.5f);
        if(sub < 0)              sub = 0;
        if(sub > total_sub - 1)  sub = total_sub - 1;

        int y_row  = sub / 5;
        int y_rem  = sub - y_row * 5;   /* 0..4  (0 = upper part) */

        /* Pick the sample glyph by sub-row position */
        const char *pt;
        switch(y_rem) {
            case 0:  pt = "'";  break;
            case 1:  pt = "`";  break;
            case 2:  pt = "-";  break;
            case 3:  pt = ".";  break;
            case 4:
            default: pt = ",";  break;
        }

        mvaddstr(row + y_row, col + (int)x, pt);

        /* Connector between previous sample and this one.
         * If adjacent samples are at different rows, fill the in-between
         * rows with a diagonal/vertical glyph so the trace doesn't break. */
        if(prev_row >= 0 && prev_row != y_row)
        {
            int step  = (y_row > prev_row) ? 1 : -1;
            const char *conn = (step > 0) ? "\\" : "/";
            for(int r = prev_row + step; r != y_row; r += step)
                mvaddstr(row + r, col + (int)x, conn);
        }

        prev_row = y_row;
        prev_sub = sub;
    }
    (void)prev_sub;

    attroff(COLOR_PAIR(color_pair));
}

/*============= DRAW: Page 4 — Waveforms ==================================================*/

static void draw_hline(int row, int col, int len);
static void draw_section(int row, int col, const char *title);
static void wl_flags_decode(uint8_t flags, char *out, size_t outsz);
static float estimate_zcr_hz(const uint16_t *buf, uint32_t count);
static float compute_rms_v(const uint16_t *buf, uint32_t count);
static void draw_page_config(int r);
static void draw_page_health(int r, IPC_Context *ipc,
                             uint32_t pkt_rate, float loss_rate);

static void draw_page_waves(int r, IPC_Context *ipc)
{
    (void)ipc;

    /*---- Top bar: live BSAU flags of the most recent packet ----*/
    uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
    uint8_t  last_flags = 0;
    if(head > 0)
        last_flags = ipc->ring[(head - 1) & IPC_SENSOR_RING_MASK].flags;

    {
        char fbuf[64];
        wl_flags_decode(last_flags, fbuf, sizeof(fbuf));
        int severe = (last_flags &
                      (WL_FLAG_CLIPPING | WL_FLAG_ELEC_OFF | WL_FLAG_ADC_OVRN));
        int warn   = (last_flags & WL_FLAG_TX_SAT);
        int cp = fbuf[0] == '\0' ? CP_GOOD
               : severe          ? CP_BAD
               : warn            ? CP_WARN
                                 : CP_CYAN;

        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, 1, "BSAU flags:");
        attroff(COLOR_PAIR(CP_DIM));
        attron(COLOR_PAIR(cp) | A_BOLD);
        printw(" %s", (fbuf[0] == '\0') ? "OK" : fbuf);
        attroff(COLOR_PAIR(cp) | A_BOLD);

        /* Tell the user the glyph ramp they're looking at */
        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, g_col_r, "glyphs top->bot: '  `  -  .  ,     (/ \\ connectors)");
        attroff(COLOR_PAIR(CP_DIM));
        r++;
    }

    if(wave_detail)
    {
        /* ───── SINGLE CHANNEL DETAIL ───── */
        uint32_t avail = (wave_count < WAVE_BUF_SIZE) ? wave_count : WAVE_BUF_SIZE;
        uint32_t sum = 0;
        uint16_t vmin = 4095, vmax = 0;
        for(uint32_t i = 0; i < avail; i++)
        {
            uint16_t v = wave_buf[wave_sel_ch][i];
            sum += v;
            if(v < vmin) vmin = v;
            if(v > vmax) vmax = v;
        }
        float dc_v  = (avail > 0) ? ((float)sum / avail / WAVE_ADC_MAX * 3.3f) : 0.0f;
        float vpp_v = (float)(vmax - vmin) / WAVE_ADC_MAX * 3.3f;
        float rms_v = compute_rms_v(wave_buf[wave_sel_ch], avail);
        float hz    = estimate_zcr_hz(wave_buf[wave_sel_ch], avail);

        /* CLIP indicator: raw min/max sitting at ADC rails (within 1%)  */
        bool clip_lo = (vmin <= 40);             /* 40 / 4095 < 1% */
        bool clip_hi = (vmax >= 4055);
        bool clip    = clip_lo || clip_hi;

        attron(A_BOLD);
        mvprintw(r, 1, "CHANNEL %d", wave_sel_ch);
        attroff(A_BOLD);

        mvprintw(r, 12, "Hz:");
        attron(COLOR_PAIR(CP_MAGENTA) | A_BOLD);
        printw("%6.0f", hz);
        attroff(COLOR_PAIR(CP_MAGENTA) | A_BOLD);

        printw("  Vpp:");
        attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
        printw("%.3fV", vpp_v);
        attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);

        printw("  DC:");
        attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
        printw("%.3fV", dc_v);
        attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);

        printw("  RMS:");
        attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
        printw("%.3fV", rms_v);
        attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);

        printw("  ADC:");
        attron(COLOR_PAIR(clip ? CP_BAD : CP_DIM));
        printw("%u-%u%s", vmin, vmax, clip ? " CLIP!" : "");
        attroff(COLOR_PAIR(clip ? CP_BAD : CP_DIM));
        r++;

        /* Horizontal axis: time-per-screen = WAVE_BUF_SIZE / 2 kHz */
        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, 3, "t = 0");
        mvprintw(r, (g_tui_w / 2) - 5, "t = %.0f ms",
                 (float)WAVE_BUF_SIZE / 2000.0f * 1000.0f / 2.0f);
        mvprintw(r, g_tui_w - 14, "t = %.0f ms",
                 (float)WAVE_BUF_SIZE / 2000.0f * 1000.0f);
        attroff(COLOR_PAIR(CP_DIM));
        r++;

        int big_w = g_tui_w - 6;
        draw_waveform(r, 3, big_w, WAVE_PLOT_H_BIG, wave_sel_ch, CP_GOOD);
        r += WAVE_PLOT_H_BIG + 2;
    }
    else
    {
        /* ───── ALL 8 CHANNELS (mini-plots, 2 columns x 4 rows) ───── */
        int mini_h = WAVE_PLOT_H;
        int mini_w = g_col_r - 4;
        if(mini_w < 20) mini_w = 20;

        for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
        {
            int c_col = (ch < 4) ? 1 : g_col_r;
            int c_row = r + (ch % 4) * (mini_h + 2);
            bool sel  = (ch == wave_sel_ch);

            int lbl_cp = sel ? CP_MAGENTA : CP_NORMAL;
            attron(COLOR_PAIR(lbl_cp) | (sel ? A_BOLD : 0));
            mvprintw(c_row, c_col, "%sch%d", sel ? ">" : " ", ch);
            attroff(COLOR_PAIR(lbl_cp) | (sel ? A_BOLD : 0));

            /* Stats over the rolling buffer */
            uint32_t avail = (wave_count < WAVE_BUF_SIZE) ? wave_count : WAVE_BUF_SIZE;
            uint16_t vmin = 4095, vmax = 0;
            for(uint32_t i = 0; i < avail; i++)
            {
                uint16_t v = wave_buf[ch][i];
                if(v < vmin) vmin = v;
                if(v > vmax) vmax = v;
            }
            float vpp = (float)(vmax - vmin) / WAVE_ADC_MAX * 3.3f;
            float rms = compute_rms_v(wave_buf[ch], avail);
            float hz  = estimate_zcr_hz(wave_buf[ch], avail);
            bool  clip = (vmin <= 40) || (vmax >= 4055);

            /* Right-aligned stats line next to label, colored by clip state */
            attron(COLOR_PAIR(CP_DIM));
            printw(" %4.0fHz %.2fVpp %.2fVrms", hz, vpp, rms);
            attroff(COLOR_PAIR(CP_DIM));

            if(clip)
            {
                attron(COLOR_PAIR(CP_BAD) | A_BOLD);
                printw(" CLIP");
                attroff(COLOR_PAIR(CP_BAD) | A_BOLD);
            }

            int wave_cp = sel ? CP_MAGENTA : CP_GOOD;
            draw_waveform(c_row + 1, c_col + 1, mini_w, mini_h, ch, wave_cp);
        }
    }
}

/*============= HELPERS: State Strings =====================================================*/

static const char *state_str(uint8_t s)
{
    switch(s)
    {
        case IPC_STATE_INIT:    return "INIT";
        case IPC_STATE_RUNNING: return "RUNNING";
        case IPC_STATE_SAFE:    return "SAFE";
        default:                return "???";
    }
}

static int state_color(uint8_t s)
{
    switch(s)
    {
        case IPC_STATE_RUNNING: return CP_GOOD;
        case IPC_STATE_SAFE:    return CP_BAD;
        default:                return CP_WARN;
    }
}

static const char *batt_str(uint8_t flags)
{
    switch(flags & 0x03)
    {
        case 0: return "OK";
        case 1: return "LOW";
        case 2: return "CRITICAL";
        case 3: return "CHARGING";
        default: return "???";
    }
}

static int batt_color(uint8_t flags)
{
    switch(flags & 0x03)
    {
        case 0: return CP_GOOD;
        case 1: return CP_WARN;
        case 2: return CP_BAD;
        case 3: return CP_CYAN;
        default: return CP_WARN;
    }
}

/**
 *  Decode BSAU packet flags into a compact banner string.
 *  Each set flag adds a short tag; result looks like "CLIP ELEC CAL".
 *  Returns empty string if no non-battery flags are set.
 */
static void wl_flags_decode(uint8_t flags, char *out, size_t outsz)
{
    out[0] = '\0';
    size_t n = 0;
    const struct { uint8_t bit; const char *tag; } tags[] = {
        { WL_FLAG_CLIPPING,     "CLIP" },
        { WL_FLAG_ELEC_OFF,     "ELEC" },
        { WL_FLAG_ADC_OVRN,     "OVRN" },
        { WL_FLAG_TX_SAT,       "TX_SAT" },
        { WL_FLAG_CAL,          "CAL"  },
        { WL_FLAG_FIRST_PACKET, "FIRST" },
    };
    for(size_t i = 0; i < sizeof(tags)/sizeof(tags[0]); i++)
    {
        if(flags & tags[i].bit)
        {
            int m = snprintf(out + n, outsz - n, "%s%s",
                             n ? " " : "", tags[i].tag);
            if(m < 0 || (size_t)m >= outsz - n) break;
            n += (size_t)m;
        }
    }
}

/**
 *  Compute approximate zero-crossing frequency (Hz) from a rolling buffer.
 *  Sample rate is assumed 2 kHz. Counts crossings around the DC mean, so
 *  this gives the fundamental of sine/square/triangle nicely; returns 0
 *  for pure noise or DC.
 */
static float estimate_zcr_hz(const uint16_t *buf, uint32_t count)
{
    if(count < 4) return 0.0f;

    /* Mean */
    uint64_t sum = 0;
    for(uint32_t i = 0; i < count; i++) sum += buf[i];
    float mean = (float)sum / (float)count;

    /* Crossings with hysteresis (1 % of span) */
    uint16_t mn = 4095, mx = 0;
    for(uint32_t i = 0; i < count; i++) {
        if(buf[i] < mn) mn = buf[i];
        if(buf[i] > mx) mx = buf[i];
    }
    float hyst = (float)(mx - mn) * 0.01f;
    if(hyst < 1.0f) hyst = 1.0f;

    int side = 0;                /* -1, 0, +1 */
    uint32_t cross = 0;
    for(uint32_t i = 0; i < count; i++)
    {
        if((float)buf[i] > mean + hyst)
        {
            if(side == -1) cross++;
            side = +1;
        }
        else if((float)buf[i] < mean - hyst)
        {
            if(side == +1) cross++;
            side = -1;
        }
    }

    /* Each full cycle has 2 crossings */
    float period_s = (float)count / 2000.0f;
    if(period_s <= 0.0f) return 0.0f;
    return (float)cross * 0.5f / period_s;
}

/**
 *  Compute RMS in volts from a rolling raw-ADC buffer.
 */
static float compute_rms_v(const uint16_t *buf, uint32_t count)
{
    if(count == 0) return 0.0f;
    double sumsq = 0.0;
    double mean_sum = 0.0;
    for(uint32_t i = 0; i < count; i++) mean_sum += buf[i];
    double mean = mean_sum / (double)count;
    for(uint32_t i = 0; i < count; i++)
    {
        double d = (double)buf[i] - mean;
        sumsq += d * d;
    }
    double rms_adc = sqrt(sumsq / (double)count);
    return (float)(rms_adc / 4095.0 * 3.3);
}

/*============= HELPERS: Drawing ===========================================================*/

static void draw_hline(int row, int col, int len)
{
    mvhline(row, col, ACS_HLINE, len);
}

static void draw_lv(int row, int col, const char *label, int cp, const char *fmt, ...)
{
    mvprintw(row, col, "%-14s", label);
    attron(COLOR_PAIR(cp) | A_BOLD);
    va_list ap;
    va_start(ap, fmt);
    char buf[64];
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    printw("%s", buf);
    attroff(COLOR_PAIR(cp) | A_BOLD);
}

static void draw_bar(int row, int col, int width, float frac, int cp_f, int cp_e)
{
    if(frac < 0.0f) frac = 0.0f;
    if(frac > 1.0f) frac = 1.0f;
    int filled = (int)(frac * width);

    move(row, col);
    attron(COLOR_PAIR(cp_f));
    for(int i = 0; i < filled; i++) addch(ACS_BLOCK);
    attroff(COLOR_PAIR(cp_f));

    attron(COLOR_PAIR(cp_e) | A_DIM);
    for(int i = filled; i < width; i++) addch(ACS_BOARD);
    attroff(COLOR_PAIR(cp_e) | A_DIM);

    printw(" %3d%%", (int)(frac * 100));
}

static void draw_slider(int row, int col, uint16_t val, uint16_t lo, uint16_t hi, int width)
{
    float frac = (hi > lo) ? (float)(val - lo) / (float)(hi - lo) : 0.0f;
    if(frac < 0.0f) frac = 0.0f;
    if(frac > 1.0f) frac = 1.0f;
    int pos = (int)(frac * (width - 1));

    move(row, col);
    for(int i = 0; i < width; i++)
    {
        if(i == pos)
        {
            int cp = CP_GOOD;
            if(frac < 0.05f || frac > 0.95f)      cp = CP_BAD;
            else if(frac < 0.15f || frac > 0.85f)  cp = CP_WARN;
            attron(COLOR_PAIR(cp) | A_BOLD);
            addch('O');
            attroff(COLOR_PAIR(cp) | A_BOLD);
        }
        else
        {
            attron(COLOR_PAIR(CP_DIM) | A_DIM);
            addch(ACS_HLINE);
            attroff(COLOR_PAIR(CP_DIM) | A_DIM);
        }
    }
}

static void draw_section(int row, int col, const char *title)
{
    attron(A_BOLD);
    mvprintw(row, col, "%s", title);
    attroff(A_BOLD);
}

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)(ts.tv_sec) * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000);
}

/*============= DRAW: Tab Bar ==============================================================*/

static int draw_header(int r)
{
    attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
    mvprintw(r, 0, "%-*s", g_tui_w,
             demo_mode ? "  CPCU MONITOR - InfiniTech v3.2-linetrace [DEMO]"
                       : "  CPCU MONITOR - InfiniTech v3.2-linetrace");
    attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);
    r++;

    move(r, 1);
    for(int p = 0; p < PAGE_COUNT; p++)
    {
        if(p == (int)current_page)
        {
            attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
            printw("[%d:%s]", p + 1, PAGE_TITLES[p]);
            attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);
        }
        else
        {
            attron(COLOR_PAIR(CP_DIM));
            printw(" %d:%s ", p + 1, PAGE_TITLES[p]);
            attroff(COLOR_PAIR(CP_DIM));
        }
        printw("  ");
    }
    r++;
    draw_hline(r, 0, g_tui_w);
    return r + 1;
}

/*============= DRAW: Page 1 — Overview ====================================================*/

static void draw_page_overview(int r, IPC_Context *ipc,
                               uint32_t pkt_rate, float loss_rate,
                               uint32_t up_h, uint32_t up_m, uint32_t up_s)
{
    uint8_t  sys_state  =   atomic_load(&ipc->ctrl->system_state);
    uint8_t  io_rdy     =   atomic_load(&ipc->ctrl->io_ready);
    uint8_t  dsp_rdy    =   atomic_load(&ipc->ctrl->dsp_ready);
    uint32_t pkts       =   atomic_load(&ipc->diag->io_pkts_received);
    uint32_t gaps       =   atomic_load(&ipc->diag->io_seq_gaps);
    uint32_t overflows  =   atomic_load(&ipc->diag->io_ring_overflows);
    uint32_t nrf_status =   atomic_load(&ipc->diag->io_nrf_init_status);
    uint32_t dsp_inf    =   atomic_load(&ipc->diag->dsp_inferences);
    uint32_t dsp_lat    =   atomic_load(&ipc->diag->dsp_max_latency_us);
    uint32_t dsp_batch  =   atomic_load(&ipc->diag->dsp_batches);
    uint32_t ring_fill  =   IPC_SensorCount(ipc);

    uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
    IPC_SensorEntry latest;
    memset(&latest, 0, sizeof(latest));
    if(head > 0) latest = ipc->ring[(head - 1) & IPC_SENSOR_RING_MASK];

    uint16_t servo[IPC_NUM_SERVOS];
    memcpy(servo, (const void *)ipc->motor->servo_us, sizeof(servo));
    uint8_t gesture     =   ipc->motor->gesture_id;
    uint8_t confidence  =   ipc->motor->confidence;
    float batt_v        =   latest.vbat_raw * (3.3f / 4095.0f) * 2.0f;

    /*===== HEALTH SUMMARY BANNER (rolled up from page 6) =====
     * One line showing green/yellow/red per subsystem so the user
     * can see the whole system state at a glance without switching
     * to page 6.  Each pill is [OK]/[WARN]/[FAULT]. */
    {
        uint32_t nrf_status = atomic_load(&ipc->diag->io_nrf_init_status);
        uint32_t overflows  = atomic_load(&ipc->diag->io_ring_overflows);
        uint32_t dsp_lat    = atomic_load(&ipc->diag->dsp_max_latency_us);
        uint64_t io_hb_us   = atomic_load(&ipc->ctrl->io_heartbeat_us);
        uint32_t hb_age_ms  = 0;
        {
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
            if(io_hb_us > 0 && now_us > io_hb_us)
                hb_age_ms = (uint32_t)((now_us - io_hb_us) / 1000ULL);
        }

        int radio = (nrf_status != 0)    ? 2 : (pkt_rate < 900 ? 1 : 0);
        int ioh   = (!io_rdy)            ? 2 : (hb_age_ms > 100 ? 2 : (hb_age_ms > 20 ? 1 : 0));
        int ipcs  = (overflows > 0)      ? 2 : (ring_fill > 900 ? 1 : 0);
        int batt  = (latest.vbat_raw > 0 && batt_v < 2.7f) ? 2
                  : (latest.vbat_raw > 0 && batt_v < 3.0f) ? 1 : 0;
        int dsph  = (!dsp_rdy)           ? 2 : (dsp_lat > 50000 ? 2 : (dsp_lat > 20000 ? 1 : 0));
        int fsm   = (sys_state == IPC_STATE_SAFE)    ? 2
                  : (sys_state == IPC_STATE_RUNNING) ? 0 : 1;

        #define PILL(lbl, s) do { \
            int cp_ = (s) == 0 ? CP_GOOD : (s) == 1 ? CP_WARN : CP_BAD; \
            const char *st = (s) == 0 ? "OK" : (s) == 1 ? "WARN" : "FAULT"; \
            attron(COLOR_PAIR(CP_DIM)); printw("%s:", (lbl)); attroff(COLOR_PAIR(CP_DIM)); \
            attron(COLOR_PAIR(cp_) | A_BOLD); printw("%s", st); attroff(COLOR_PAIR(cp_) | A_BOLD); \
            printw("  "); \
        } while(0)

        mvprintw(r, 1, "HEALTH  ");
        PILL("radio",  radio);
        PILL("io",     ioh);
        PILL("ipc",    ipcs);
        PILL("batt",   batt);
        PILL("dsp",    dsph);
        PILL("fsm",    fsm);

        /* Overall verdict at right edge */
        int worst = 0;
        int stats[] = { radio, ioh, ipcs, batt, dsph, fsm };
        for(size_t i = 0; i < sizeof(stats)/sizeof(stats[0]); i++)
            if(stats[i] > worst) worst = stats[i];
        const char *vr = worst == 0 ? "NOMINAL" : worst == 1 ? "WARNING" : "DEGRADED";
        int vr_cp     = worst == 0 ? CP_GOOD   : worst == 1 ? CP_WARN   : CP_BAD;
        int vr_len    = (int)strlen(vr);
        attron(COLOR_PAIR(vr_cp) | A_BOLD);
        mvprintw(r, g_tui_w - vr_len - 2, "%s", vr);
        attroff(COLOR_PAIR(vr_cp) | A_BOLD);

        #undef PILL
        r++;
        draw_hline(r, 0, g_tui_w);
        r++;
    }

    draw_section(r, 1, "SYSTEM");
    draw_section(r, g_col_r, "RADIO LINK");
    r++;

    draw_lv(r, 1,       "State:",      state_color(sys_state), "%s", state_str(sys_state));
    draw_lv(r, g_col_r, "Packets/s:",  pkt_rate > 900 ? CP_GOOD : CP_WARN, "%u pkt/s", pkt_rate);
    r++;
    draw_lv(r, 1,       "Uptime:",     CP_CYAN, "%02u:%02u:%02u", up_h, up_m, up_s);
    draw_lv(r, g_col_r, "Total pkts:", CP_CYAN, "%u  (since boot)", pkts);
    r++;
    draw_lv(r, 1,       "IO ready:",   io_rdy ? CP_GOOD : CP_BAD, "%s", io_rdy ? "YES" : "NO");
    draw_lv(r, g_col_r, "Seq gaps:",   gaps > 10 ? CP_WARN : CP_GOOD, "%u  (missed pkts)", gaps);
    r++;
    draw_lv(r, 1,       "DSP ready:",  dsp_rdy ? CP_GOOD : CP_BAD, "%s", dsp_rdy ? "YES" : "NO");
    draw_lv(r, g_col_r, "Loss rate:",  loss_rate > 0.01f ? CP_WARN : CP_GOOD, "%.3f %%  (of last 1k)", loss_rate * 100);
    r++;
    draw_lv(r, 1,       "NRF init:",   nrf_status == 0 ? CP_GOOD : CP_BAD, "%s",
            nrf_status == 0 ? "OK" : "FAIL");
    r += 2;

    draw_hline(r - 1, 0, g_tui_w);
    draw_section(r, 1, "EMG CHANNELS (latest raw ADC, % of 4095)");
    r++;
    for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
    {
        int col = (ch < 4) ? 1 : g_col_r;
        int row = r + (ch % 4);
        float frac = (float)latest.samples[0].ch[ch] / 4095.0f;
        mvprintw(row, col, "ch%d ", ch);
        draw_bar(row, col + 4, g_bar_w, frac, CP_BAR_FILL, CP_BAR_EMPTY);
    }
    r += 5;

    draw_hline(r - 1, 0, g_tui_w);
    draw_section(r, 1, "SERVOS (pulse width, us)");
    draw_section(r, g_col_r, "BATTERY (BSAU pack)");
    r++;
    for(int i = 0; i < IPC_NUM_SERVOS; i++)
    {
        mvprintw(r + i, 1, "S%d %-5s %4u ", i, SERVO_NAMES[i], servo[i]);
        draw_slider(r + i, 16, servo[i], SERVO_MIN[i], SERVO_MAX[i], g_slider_w);
    }
    draw_lv(r,     g_col_r, "Voltage:",   batt_v < 3.0f ? CP_BAD : CP_GOOD, "%.2f V  (pack)", batt_v);
    draw_lv(r + 1, g_col_r, "Raw ADC:",   CP_CYAN, "%u  (12-bit, 0..4095)", latest.vbat_raw);
    draw_lv(r + 2, g_col_r, "Level:",     batt_color(latest.flags), "%s", batt_str(latest.flags));
    draw_lv(r + 4, g_col_r, "Ring fill:", ring_fill > 100 ? CP_WARN : CP_GOOD,
            "%u / %u  (IPC buffer)", ring_fill, IPC_SENSOR_RING_SIZE);
    draw_lv(r + 5, g_col_r, "Overflows:", overflows > 0 ? CP_BAD : CP_GOOD, "%u  (dropped by IO)", overflows);
    r += 7;

    draw_hline(r - 1, 0, g_tui_w);
    draw_section(r, 1, "DSP PIPELINE");
    draw_section(r, g_col_r, "INFERENCE");
    r++;
    draw_lv(r, 1,       "DSP windows:", CP_CYAN, "%u  (400-sample FFTs)", dsp_batch);
    draw_lv(r, g_col_r, "Inferences:",  CP_CYAN, "%u  (ML predictions)", dsp_inf);
    r++;
    draw_lv(r, 1,       "Max latency:", dsp_lat > 50000 ? CP_WARN : CP_GOOD, "%u us  (worst batch)", dsp_lat);
    /* Use the named gesture from dsp_export if available, otherwise fall
     * back to the numeric motor->gesture_id. */
    {
        uint32_t exp_sq = atomic_load(&ipc->dsp_export->update_seq);
        if(exp_sq > 0)
        {
            char gname[IPC_MAX_GESTURE_NAME];
            memcpy(gname, (const void *)ipc->dsp_export->gesture_name, sizeof(gname));
            gname[IPC_MAX_GESTURE_NAME - 1] = '\0';
            draw_lv(r, g_col_r, "Gesture:", CP_GOOD, "%s  (%u%%)", gname, confidence);
        }
        else
        {
            draw_lv(r, g_col_r, "Gesture:", CP_GOOD, "#%u  (%u%%)", gesture, confidence);
        }
    }
    r += 2;

    uint32_t export_seq = atomic_load(&ipc->dsp_export->update_seq);
    if(export_seq > 0)
    {
        draw_hline(r - 1, 0, g_tui_w);
        draw_section(r, 1, "ML CLASSIFICATION (Python)");
        r++;

        char export_name[IPC_MAX_GESTURE_NAME];
        memcpy(export_name, (const void *)ipc->dsp_export->gesture_name, sizeof(export_name));
        export_name[IPC_MAX_GESTURE_NAME - 1] = '\0';
        uint32_t inf_us  = ipc->dsp_export->inference_time_us;
        uint8_t  active  = ipc->dsp_export->active_class;

        mvprintw(r, 1, "Gesture: ");
        attron(COLOR_PAIR(CP_MAGENTA) | A_BOLD);
        printw("%-16s", export_name);
        attroff(COLOR_PAIR(CP_MAGENTA) | A_BOLD);
        mvprintw(r, g_col_r, "Inf. time: ");
        attron(COLOR_PAIR(inf_us > 50000 ? CP_WARN : CP_CYAN) | A_BOLD);
        printw("%u us", inf_us);
        attroff(COLOR_PAIR(inf_us > 50000 ? CP_WARN : CP_CYAN) | A_BOLD);
        r++;

        uint8_t nc = ipc->dsp_export->num_classes;
        if(nc > IPC_MAX_CLASSES) nc = IPC_MAX_CLASSES;
        for(int c = 0; c < (int)nc; c++)
        {
            int col = (c < 5) ? 1 : g_col_r;
            int row = r + (c % 5);
            float conf = ipc->dsp_export->class_confidence[c];
            if(conf < 0.0f) conf = 0.0f;
            if(conf > 1.0f) conf = 1.0f;
            mvprintw(row, col, "%-6s", CLS_NAMES[c]);
            draw_bar(row, col + 7, 12, conf,
                     c == active ? CP_MAGENTA : CP_BAR_FILL, CP_BAR_EMPTY);
        }
        r += 6;

        draw_section(r, 1, "FILTERED RMS (Python, bar=% of 0.5V full-scale)");
        r++;
        for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
        {
            int col  = (ch < 4) ? 1 : g_col_r;
            int row  = r + (ch % 4);
            float rv = ipc->dsp_export->channel_rms[ch];
            float fr = rv / 0.5f;
            if(fr > 1.0f) fr = 1.0f;
            mvprintw(row, col, "ch%d ", ch);
            draw_bar(row, col + 4, 14, fr, CP_CYAN, CP_BAR_EMPTY);
            printw(" %.4f V", rv);
        }
    }
}

/*============= DRAW: Page 2 — Radio / I/O =================================================*/

static void draw_page_radio(int r, IPC_Context *ipc,
                            uint32_t pkt_rate, float loss_rate,
                            uint32_t up_h, uint32_t up_m, uint32_t up_s)
{
    uint8_t  sys_state  =   atomic_load(&ipc->ctrl->system_state);
    uint8_t  io_rdy     =   atomic_load(&ipc->ctrl->io_ready);
    uint32_t pkts       =   atomic_load(&ipc->diag->io_pkts_received);
    uint32_t pkts_drp   =   atomic_load(&ipc->diag->io_pkts_dropped);
    uint32_t gaps       =   atomic_load(&ipc->diag->io_seq_gaps);
    uint32_t overflows  =   atomic_load(&ipc->diag->io_ring_overflows);
    uint32_t nrf_status =   atomic_load(&ipc->diag->io_nrf_init_status);
    uint32_t max_poll   =   atomic_load(&ipc->diag->io_max_poll_us);
    uint32_t safe_ents  =   atomic_load(&ipc->diag->io_safe_entries);
    uint64_t io_hb_us   =   atomic_load(&ipc->ctrl->io_heartbeat_us);
    uint32_t ring_fill  =   IPC_SensorCount(ipc);

    uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
    IPC_SensorEntry latest;
    memset(&latest, 0, sizeof(latest));
    if(head > 0) latest = ipc->ring[(head - 1) & IPC_SENSOR_RING_MASK];

    float batt_v = latest.vbat_raw * (3.3f / 4095.0f) * 2.0f;

    /* IO heartbeat age (how long since cpcu_io updated heartbeat timestamp).
     * If > 100 ms, the RT loop is stalling. */
    uint32_t hb_age_ms = 0;
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
        if(io_hb_us > 0 && now_us > io_hb_us)
            hb_age_ms = (uint32_t)((now_us - io_hb_us) / 1000ULL);
    }

    /*==================== NRF24L01+ STATUS ====================*/
    draw_section(r, 1, "NRF24L01+ STATUS");
    draw_section(r, g_col_r, "SAFETY FSM");
    r++;

    draw_lv(r, 1,       "Init:",        nrf_status == 0 ? CP_GOOD : CP_BAD, "%s",
            nrf_status == 0 ? "OK" : "FAIL");
    draw_lv(r, g_col_r, "State:",       state_color(sys_state), "%s", state_str(sys_state));
    r++;
    draw_lv(r, 1,       "Channel:",     CP_CYAN, "76  (2.476 GHz)");
    draw_lv(r, g_col_r, "IO ready:",    io_rdy ? CP_GOOD : CP_BAD, "%s", io_rdy ? "YES" : "NO");
    r++;
    draw_lv(r, 1,       "Address:",     CP_CYAN, "E7:E7:E7:E7:E7  (5-byte)");
    draw_lv(r, g_col_r, "IO heartbeat:",
            hb_age_ms > 100 ? CP_BAD : hb_age_ms > 20 ? CP_WARN : CP_GOOD,
            "%u ms ago  (RT loop)", hb_age_ms);
    r++;
    draw_lv(r, 1,       "SPI speed:",   CP_CYAN, "8 MHz");
    draw_lv(r, g_col_r, "SAFE entries:",
            safe_ents > 0 ? CP_WARN : CP_GOOD, "%u  (times FSM went SAFE)", safe_ents);
    r++;
    draw_lv(r, 1,       "Payload:",     CP_CYAN, "32 B fixed");
    draw_lv(r, g_col_r, "Batt (pack):", batt_v > 3.0f ? CP_GOOD : CP_BAD, "%.2f V", batt_v);
    r += 2;

    draw_hline(r - 1, 0, g_tui_w);

    /*==================== PACKET STATISTICS ====================*/
    draw_section(r, 1, "PACKET STATISTICS");
    draw_section(r, g_col_r, "TIMING / RT");
    r++;

    draw_lv(r, 1,       "Total RX:",    CP_CYAN, "%u pkts  (since boot)", pkts);
    draw_lv(r, g_col_r, "Uptime:",      CP_CYAN, "%02u:%02u:%02u", up_h, up_m, up_s);
    r++;
    draw_lv(r, 1,       "Rate:",        pkt_rate > 900 ? CP_GOOD : CP_WARN, "%u pkt/s", pkt_rate);
    draw_lv(r, g_col_r, "Max poll:",    max_poll > 100 ? CP_WARN : CP_GOOD, "%u us  (worst SPI read)", max_poll);
    r++;
    draw_lv(r, 1,       "Dropped:",     pkts_drp > 0 ? CP_BAD : CP_GOOD, "%u  (ring full)", pkts_drp);
    draw_lv(r, g_col_r, "Ring OVF:",    overflows > 0 ? CP_BAD : CP_GOOD, "%u  (events)", overflows);
    r++;
    draw_lv(r, 1,       "Seq gaps:",    gaps > 10 ? CP_WARN : CP_GOOD, "%u  (missed seqs)", gaps);

    /*---- Ring-fill bar ----*/
    mvprintw(r, g_col_r, "Ring fill:");
    {
        float frac = (float)ring_fill / (float)IPC_SENSOR_RING_SIZE;
        int ring_bar_w = g_bar_w - 4;
        if(ring_bar_w < 10) ring_bar_w = 10;
        draw_bar(r, g_col_r + 11, ring_bar_w, frac,
                 ring_fill > 100 ? CP_WARN : CP_BAR_FILL, CP_BAR_EMPTY);
        attron(COLOR_PAIR(CP_DIM));
        printw(" %u/%u", ring_fill, IPC_SENSOR_RING_SIZE);
        attroff(COLOR_PAIR(CP_DIM));
    }
    r++;

    draw_lv(r, 1,       "Loss rate:",   loss_rate > 0.01f ? CP_WARN : CP_GOOD,
            "%.4f %%  (of last 1k)", loss_rate * 100.0f);
    draw_lv(r, g_col_r, "Retry (last):",
            latest.tx_retry > 2 ? CP_WARN : CP_GOOD, "%u  (nRF auto-retries)", latest.tx_retry);
    r += 2;

    draw_hline(r - 1, 0, g_tui_w);

    /*==================== LAST PACKET RAW ====================*/
    draw_section(r, 1, "LAST PACKET RAW");
    draw_section(r, g_col_r, "BSAU FLAGS");
    r++;

    /* Legend line — tells the user what these terse field names mean */
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    mvprintw(r, 1, "seq=8-bit  flags=hex  retry=count  loss=last-pkt  ts=BSAU ms  vbat=ADC 0..4095");
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);
    r++;

    /* left col: fields */
    mvprintw(r, 1, "seq=");
    attron(COLOR_PAIR(CP_CYAN)); printw("%-3u", latest.seq); attroff(COLOR_PAIR(CP_CYAN));
    printw("  flags=");
    attron(COLOR_PAIR(CP_CYAN)); printw("0x%02X", latest.flags); attroff(COLOR_PAIR(CP_CYAN));
    printw("  retry=");
    int rt_cp = latest.tx_retry > 2 ? CP_WARN : CP_GOOD;
    attron(COLOR_PAIR(rt_cp)); printw("%u", latest.tx_retry); attroff(COLOR_PAIR(rt_cp));
    printw("  loss=");
    attron(COLOR_PAIR(CP_CYAN)); printw("%u", latest.pkt_loss); attroff(COLOR_PAIR(CP_CYAN));
    printw("  ts=");
    attron(COLOR_PAIR(CP_CYAN)); printw("%u ms", latest.timestamp); attroff(COLOR_PAIR(CP_CYAN));

    /* right col: decoded BSAU flags. CLIP=red, ELEC=red, OVRN=red,
     * TX_SAT=yellow, CAL=cyan, FIRST=cyan, empty=green OK */
    {
        char buf[64];
        wl_flags_decode(latest.flags, buf, sizeof(buf));
        int severe = (latest.flags &
                      (WL_FLAG_CLIPPING | WL_FLAG_ELEC_OFF | WL_FLAG_ADC_OVRN));
        int warn   = (latest.flags & WL_FLAG_TX_SAT);
        int cp = buf[0] == '\0' ? CP_GOOD
               : severe        ? CP_BAD
               : warn          ? CP_WARN
                               : CP_CYAN;
        mvprintw(r, g_col_r, "%-*s",
                 g_tui_w - g_col_r - 1,
                 (buf[0] == '\0') ? "OK" : buf);
        (void)cp;   /* color applied via next attr block if needed */
        attron(COLOR_PAIR(cp) | A_BOLD);
        mvprintw(r, g_col_r, "%s", (buf[0] == '\0') ? "OK" : buf);
        attroff(COLOR_PAIR(cp) | A_BOLD);
    }
    r++;

    /* RX latency for the latest packet: how long ago did cpcu_io receive it? */
    uint32_t rx_age_ms = 0;
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
        if(latest.rx_time_us > 0 && now_us > latest.rx_time_us)
            rx_age_ms = (uint32_t)((now_us - latest.rx_time_us) / 1000ULL);
    }
    mvprintw(r, 1, "rx_age=");
    attron(COLOR_PAIR(rx_age_ms > 50 ? CP_WARN : CP_CYAN));
    printw("%u ms", rx_age_ms);
    attroff(COLOR_PAIR(rx_age_ms > 50 ? CP_WARN : CP_CYAN));
    printw("  vbat=");
    attron(COLOR_PAIR(CP_CYAN)); printw("%u", latest.vbat_raw); attroff(COLOR_PAIR(CP_CYAN));
    printw("  batt=");
    attron(COLOR_PAIR(batt_color(latest.flags)) | A_BOLD);
    printw("%s", batt_str(latest.flags));
    attroff(COLOR_PAIR(batt_color(latest.flags)) | A_BOLD);
    r++;

    mvprintw(r, 1, "ch[0-3]: ");
    attron(COLOR_PAIR(CP_CYAN));
    printw("%4u %4u %4u %4u",
           latest.samples[0].ch[0], latest.samples[0].ch[1],
           latest.samples[0].ch[2], latest.samples[0].ch[3]);
    attroff(COLOR_PAIR(CP_CYAN));
    printw("   ch[4-7]: ");
    attron(COLOR_PAIR(CP_CYAN));
    printw("%4u %4u %4u %4u",
           latest.samples[0].ch[4], latest.samples[0].ch[5],
           latest.samples[0].ch[6], latest.samples[0].ch[7]);
    attroff(COLOR_PAIR(CP_CYAN));
    r += 2;

    draw_hline(r - 1, 0, g_tui_w);

    /*==================== EMG CHANNELS BAR GRAPHS ====================*/
    draw_section(r, 1, "EMG CHANNELS (latest raw ADC, % of 4095)");
    r++;
    for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
    {
        int col = (ch < 4) ? 1 : g_col_r;
        int row = r + (ch % 4);
        float frac = (float)latest.samples[0].ch[ch] / 4095.0f;
        mvprintw(row, col, "ch%d ", ch);
        draw_bar(row, col + 4, g_bar_w, frac, CP_BAR_FILL, CP_BAR_EMPTY);
    }
}

/*============= DRAW: Page 3 — DSP / AI ====================================================*/

static void draw_page_dsp(int r, IPC_Context *ipc)
{
    uint8_t  dsp_rdy    =   atomic_load(&ipc->ctrl->dsp_ready);
    uint32_t dsp_inf    =   atomic_load(&ipc->diag->dsp_inferences);
    uint32_t dsp_lat    =   atomic_load(&ipc->diag->dsp_max_latency_us);
    uint32_t dsp_batch  =   atomic_load(&ipc->diag->dsp_batches);
    uint32_t dsp_under  =   atomic_load(&ipc->diag->dsp_ring_underflows);
    uint32_t ring_fill  =   IPC_SensorCount(ipc);
    uint32_t export_seq =   atomic_load(&ipc->dsp_export->update_seq);
    uint32_t motor_seq  =   atomic_load(&ipc->motor->seq);
    uint64_t motor_ts   =   ipc->motor->timestamp_us;

    uint16_t servo[IPC_NUM_SERVOS];
    memcpy(servo, (const void *)ipc->motor->servo_us, sizeof(servo));

    /*---- Rolling deltas (for rates), tracked across frames ----*/
    static uint32_t prev_inf       = 0;
    static uint32_t prev_batch     = 0;
    static uint32_t prev_export_sq = 0;
    static uint32_t prev_motor_sq  = 0;
    static uint64_t prev_tick_ms   = 0;

    uint64_t now = now_ms_wall();
    uint32_t inf_rate    = 0;      /* inferences/s */
    uint32_t batch_rate  = 0;
    uint32_t export_rate = 0;
    uint32_t motor_rate  = 0;
    if(prev_tick_ms > 0 && now > prev_tick_ms)
    {
        uint32_t dt = (uint32_t)(now - prev_tick_ms);
        if(dt > 0)
        {
            inf_rate    = (dsp_inf    - prev_inf)       * 1000 / dt;
            batch_rate  = (dsp_batch  - prev_batch)     * 1000 / dt;
            export_rate = (export_seq - prev_export_sq) * 1000 / dt;
            motor_rate  = (motor_seq  - prev_motor_sq)  * 1000 / dt;
        }
    }
    prev_inf       = dsp_inf;
    prev_batch     = dsp_batch;
    prev_export_sq = export_seq;
    prev_motor_sq  = motor_seq;
    prev_tick_ms   = now;

    /* Motor command age: now − motor->timestamp_us */
    uint32_t motor_age_ms = 0;
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
        if(motor_ts > 0 && now_us > motor_ts)
            motor_age_ms = (uint32_t)((now_us - motor_ts) / 1000ULL);
    }

    /*==================== DSP PIPELINE ====================*/
    draw_section(r, 1, "DSP PIPELINE");
    draw_section(r, g_col_r, "INFERENCE ENGINE");
    r++;

    draw_lv(r, 1,       "DSP ready:",   dsp_rdy ? CP_GOOD : CP_BAD, "%s", dsp_rdy ? "YES" : "NO");
    draw_lv(r, g_col_r, "Model:",       CP_CYAN, "RandomForest");
    r++;
    draw_lv(r, 1,       "DSP windows:", CP_CYAN, "%u  (%u/s, 400-sample FFTs)", dsp_batch, batch_rate);
    draw_lv(r, g_col_r, "Inferences:",  CP_CYAN, "%u  (%u/s, ML preds)", dsp_inf, inf_rate);
    r++;
    draw_lv(r, 1,       "Max latency:", dsp_lat > 50000 ? CP_WARN : CP_GOOD, "%u us  (worst batch)", dsp_lat);
    draw_lv(r, g_col_r, "Window:",      CP_CYAN, "400 samples  (@2 kHz = 200 ms)");
    r++;
    draw_lv(r, 1,       "Ring fill:",   ring_fill > 100 ? CP_WARN : CP_GOOD,
            "%u / %u  (IPC samples)", ring_fill, IPC_SENSOR_RING_SIZE);
    draw_lv(r, g_col_r, "Stride:",      CP_CYAN, "200 samples  (50 %% overlap)");
    r++;
    draw_lv(r, 1,       "Underflows:",  dsp_under > 0 ? CP_WARN : CP_GOOD, "%u  (ring-empty events)", dsp_under);
    draw_lv(r, g_col_r, "Export rate:",
            export_rate > 0 ? CP_GOOD : CP_WARN, "%u Hz  (Python publishes)", export_rate);
    r++;
    draw_lv(r, 1,       "Motor cmds:",  CP_CYAN, "%u  (%u/s to servos)", motor_seq, motor_rate);
    draw_lv(r, g_col_r, "Cmd age:",
            motor_age_ms > 100 ? CP_WARN : CP_GOOD, "%u ms  (since last write)", motor_age_ms);
    r += 2;

    if(export_seq > 0)
    {
        char gname[IPC_MAX_GESTURE_NAME];
        memcpy(gname, (const void *)ipc->dsp_export->gesture_name, sizeof(gname));
        gname[IPC_MAX_GESTURE_NAME - 1] = '\0';
        uint8_t  active = ipc->dsp_export->active_class;
        uint32_t inf_us = ipc->dsp_export->inference_time_us;

        draw_hline(r - 1, 0, g_tui_w);
        draw_section(r, 1, "ACTIVE GESTURE");
        r++;

        mvprintw(r, 3, ">> ");
        attron(COLOR_PAIR(CP_MAGENTA) | A_BOLD);
        printw("%-16s", gname);
        attroff(COLOR_PAIR(CP_MAGENTA) | A_BOLD);
        printw("   confidence: ");
        float ac = ipc->dsp_export->class_confidence[active];
        attron(COLOR_PAIR(ac > 0.8f ? CP_GOOD : ac > 0.5f ? CP_WARN : CP_BAD) | A_BOLD);
        printw("%3d %%", (int)(ac * 100));
        attroff(COLOR_PAIR(ac > 0.8f ? CP_GOOD : ac > 0.5f ? CP_WARN : CP_BAD) | A_BOLD);
        printw("   inf time: ");
        attron(COLOR_PAIR(inf_us > 50000 ? CP_WARN : CP_CYAN));
        printw("%u us", inf_us);
        attroff(COLOR_PAIR(inf_us > 50000 ? CP_WARN : CP_CYAN));

        /* Show export_seq in parens so you can see it ticking */
        printw("   seq#");
        attron(COLOR_PAIR(CP_DIM));
        printw("%u", export_seq);
        attroff(COLOR_PAIR(CP_DIM));
        r += 2;

        draw_hline(r - 1, 0, g_tui_w);
        draw_section(r, 1, "CLASS CONFIDENCE (% softmax probability)");
        draw_section(r, g_col_r, "FILTERED RMS (bar=% of 0.5V, +abs V)");
        r++;

        uint8_t nc = ipc->dsp_export->num_classes;
        if(nc > IPC_MAX_CLASSES) nc = IPC_MAX_CLASSES;

        int max_rows = (nc > WL_NUM_CHANNELS) ? nc : WL_NUM_CHANNELS;

        for(int i = 0; i < max_rows; i++)
        {
            if(i < (int)nc)
            {
                float conf = ipc->dsp_export->class_confidence[i];
                if(conf < 0.0f) conf = 0.0f;
                if(conf > 1.0f) conf = 1.0f;
                mvprintw(r + i, 1, "%-6s", CLS_NAMES[i]);
                draw_bar(r + i, 8, 14, conf,
                         i == active ? CP_MAGENTA : CP_BAR_FILL, CP_BAR_EMPTY);
            }

            if(i < WL_NUM_CHANNELS)
            {
                float rv = ipc->dsp_export->channel_rms[i];
                float fr = rv / 0.5f;
                if(fr > 1.0f) fr = 1.0f;
                mvprintw(r + i, g_col_r, "ch%d ", i);
                draw_bar(r + i, g_col_r + 4, 14, fr, CP_CYAN, CP_BAR_EMPTY);
                printw(" %.3f V", rv);
            }
        }
        r += max_rows + 1;
    }
    else
    {
        r++;
        attron(COLOR_PAIR(CP_WARN));
        mvprintw(r, 1, "DSP export not available (Python cpcu_dsp.py not running?)");
        attroff(COLOR_PAIR(CP_WARN));
        r += 2;
    }

    draw_hline(r - 1, 0, g_tui_w);
    draw_section(r, 1, "SERVO OUTPUT (smoothed PWM pulse width, us)");
    r++;

    for(int i = 0; i < IPC_NUM_SERVOS; i++)
    {
        int col = (i < 3) ? 1 : g_col_r;
        int row = r + (i % 3);
        mvprintw(row, col, "S%d %-5s ", i, SERVO_NAMES[i]);
        draw_slider(row, col + 10, servo[i], SERVO_MIN[i], SERVO_MAX[i], 14);
        printw(" %4u us", servo[i]);
    }
}

/*============= DRAW: Page 5 — System Configuration ========================================*/

/**
 *  Static compile-time + hardware/topology info. Nothing here changes at
 *  runtime — it's a "spec sheet" so new readers understand the system
 *  they're looking at without digging through source/docs.
 */
static void draw_page_config(int r)
{
    /*==================== BSAU SIDE ====================*/
    draw_section(r, 1,       "BSAU (BIOSIGNAL ACQUISITION UNIT)");
    draw_section(r, g_col_r, "CPCU (CENTRAL PROCESSING & CONTROL UNIT)");
    r++;

    draw_lv(r, 1,       "MCU:",           CP_CYAN, "STM32L432KC  (ARM Cortex-M4F, 80 MHz)");
    draw_lv(r, g_col_r, "SBC:",           CP_CYAN, "Raspberry Pi 4B/5  (ARM Cortex-A72/A76)");
    r++;
    draw_lv(r, 1,       "EMG channels:",  CP_CYAN, "%d  (Soldered INA333 front-ends)", WL_NUM_CHANNELS);
    draw_lv(r, g_col_r, "OS / kernel:",   CP_CYAN, "Raspberry Pi OS 64-bit  (6.x)");
    r++;
    draw_lv(r, 1,       "ADC:",           CP_CYAN, "STM32 12-bit ADC  (0..4095)");
    draw_lv(r, g_col_r, "RT isolation:",  CP_CYAN, "isolcpus=1,2,3  (nohz_full+rcu_nocbs)");
    r++;
    draw_lv(r, 1,       "Sample rate:",   CP_CYAN, "2 kHz per channel");
    draw_lv(r, g_col_r, "Core 0:",        CP_CYAN, "Supervisor / TUI / logger");
    r++;
    draw_lv(r, 1,       "Samples/pkt:",   CP_CYAN, "%d  (packed 12-bit)", WL_SAMPLES_PER_PACKET);
    draw_lv(r, g_col_r, "Cores 1-2:",     CP_CYAN, "Python DSP + RandomForest ML");
    r++;
    draw_lv(r, 1,       "Packet rate:",   CP_CYAN, "1000 pkt/s  (2 samples @ 2 kHz)");
    draw_lv(r, g_col_r, "Core 3:",        CP_CYAN, "cpcu_io  (SCHED_FIFO prio 80)");
    r++;
    draw_lv(r, 1,       "Battery:",       CP_CYAN, "2S Li-ion pack + 2:1 divider");
    draw_lv(r, g_col_r, "Scheduler:",     CP_CYAN, "SCHED_FIFO realtime, mlockall");
    r += 2;

    draw_hline(r - 1, 0, g_tui_w);

    /*==================== WIRELESS LINK + IPC ====================*/
    draw_section(r, 1,       "WIRELESS LINK");
    draw_section(r, g_col_r, "IPC (SHARED MEMORY)");
    r++;

    draw_lv(r, 1,       "Radio:",         CP_CYAN, "nRF24L01+  (2.4 GHz GFSK)");
    draw_lv(r, g_col_r, "Path:",          CP_CYAN, "/dev/shm/cpcu_ipc  (66240 B)");
    r++;
    draw_lv(r, 1,       "Channel:",       CP_CYAN, "76  (2.476 GHz, ISM)");
    draw_lv(r, g_col_r, "Layout:",        CP_CYAN, "ctrl + ring + motor + dsp_export");
    r++;
    draw_lv(r, 1,       "Address:",       CP_CYAN, "E7:E7:E7:E7:E7");
    draw_lv(r, g_col_r, "Ring size:",     CP_CYAN, "%u entries  (64 B each)", IPC_SENSOR_RING_SIZE);
    r++;
    draw_lv(r, 1,       "SPI clock:",     CP_CYAN, "8 MHz");
    draw_lv(r, g_col_r, "Ring type:",     CP_CYAN, "SPSC lock-free  (seq head/tail)");
    r++;
    draw_lv(r, 1,       "Payload:",       CP_CYAN, "%d B fixed", WL_PAYLOAD_SIZE);
    draw_lv(r, g_col_r, "Motor cmd:",     CP_CYAN, "128 B  (seqlock updates)");
    r++;
    draw_lv(r, 1,       "Auto-ACK:",      CP_CYAN, "on, up to 3 auto-retries");
    draw_lv(r, g_col_r, "DSP export:",    CP_CYAN, "256 B  (gesture+confs+RMS)");
    r += 2;

    draw_hline(r - 1, 0, g_tui_w);

    /*==================== MOTOR + ML ====================*/
    draw_section(r, 1,       "MOTOR STAGE");
    draw_section(r, g_col_r, "DSP / ML");
    r++;

    draw_lv(r, 1,       "PWM driver:",    CP_CYAN, "PCA9685  (I2C @ 400 kHz, 50 Hz PWM)");
    draw_lv(r, g_col_r, "Window:",        CP_CYAN, "400 samples  (= 200 ms @ 2 kHz)");
    r++;
    draw_lv(r, 1,       "Servos:",        CP_CYAN, "%d × SG90  (1.0-2.0 ms pulse)", IPC_NUM_SERVOS);
    draw_lv(r, g_col_r, "Stride:",        CP_CYAN, "200 samples  (50 %% overlap)");
    r++;
    draw_lv(r, 1,       "Slew limit:",    CP_CYAN, "2000 us/s  (1200 us/s on gripper)");
    draw_lv(r, g_col_r, "Feature extr:",  CP_CYAN, "MAV, WL, ZC, SSC, RMS, spectral");
    r++;
    draw_lv(r, 1,       "Safety cmd:",    CP_CYAN, "Servos → neutral (1500 us) on SAFE");
    draw_lv(r, g_col_r, "Classifier:",    CP_CYAN, "RandomForest (scikit-learn)");
    r++;
    draw_lv(r, 1,       "Safety thr:",    CP_CYAN, "Radio 750/1500 ms, Vbatt 2.7/3.0 V");
    draw_lv(r, g_col_r, "Classes:",       CP_CYAN, "%d  (REST, H.SLO, H.HRD, ...)", IPC_MAX_CLASSES);
    r += 2;

    draw_hline(r - 1, 0, g_tui_w);

    /*==================== BUILD INFO ====================*/
    draw_section(r, 1, "BUILD");
    r++;

    draw_lv(r, 1, "TUI version:",  CP_CYAN, "v3.2-linetrace");
    draw_lv(r, g_col_r, "Built:",  CP_DIM, "%s %s", __DATE__, __TIME__);
    r++;
    draw_lv(r, 1, "Compiler:",     CP_CYAN,
#ifdef __GNUC__
            "GCC %d.%d", __GNUC__, __GNUC_MINOR__
#else
            "unknown"
#endif
            );
    draw_lv(r, g_col_r, "Std:",    CP_CYAN, "C%ld", __STDC_VERSION__ / 100L);
}

/*============= DRAW: Page 6 — System Health ===============================================*/

/**
 *  Traffic-light rollup dashboard. One row per subsystem, with a
 *  colored status cell and a short explanation of what's being checked
 *  and (on fault) why it's unhappy. Intended as the first page a
 *  debugger should glance at — "is everything OK?"
 */
static void draw_page_health(int r, IPC_Context *ipc,
                             uint32_t pkt_rate, float loss_rate)
{
    /*---- Gather the telemetry we'll compare against thresholds ----*/
    uint8_t  sys_state  = atomic_load(&ipc->ctrl->system_state);
    uint8_t  io_rdy     = atomic_load(&ipc->ctrl->io_ready);
    uint8_t  dsp_rdy    = atomic_load(&ipc->ctrl->dsp_ready);
    uint32_t nrf_status = atomic_load(&ipc->diag->io_nrf_init_status);
    uint32_t overflows  = atomic_load(&ipc->diag->io_ring_overflows);
    uint32_t gaps       = atomic_load(&ipc->diag->io_seq_gaps);
    uint32_t pkts_drp   = atomic_load(&ipc->diag->io_pkts_dropped);
    uint32_t safe_ents  = atomic_load(&ipc->diag->io_safe_entries);
    uint32_t max_poll   = atomic_load(&ipc->diag->io_max_poll_us);
    uint32_t dsp_lat    = atomic_load(&ipc->diag->dsp_max_latency_us);
    uint32_t dsp_under  = atomic_load(&ipc->diag->dsp_ring_underflows);
    uint32_t export_seq = atomic_load(&ipc->dsp_export->update_seq);
    uint32_t ring_fill  = IPC_SensorCount(ipc);
    uint64_t io_hb_us   = atomic_load(&ipc->ctrl->io_heartbeat_us);

    uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
    IPC_SensorEntry latest;
    memset(&latest, 0, sizeof(latest));
    if(head > 0) latest = ipc->ring[(head - 1) & IPC_SENSOR_RING_MASK];
    float batt_v = latest.vbat_raw * (3.3f / 4095.0f) * 2.0f;

    uint32_t hb_age_ms = 0;
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
        if(io_hb_us > 0 && now_us > io_hb_us)
            hb_age_ms = (uint32_t)((now_us - io_hb_us) / 1000ULL);
    }

    /*---- Compute per-subsystem status ----
     * Each entry is (name, status_code, detail_text).
     * status_code: 0=OK (green), 1=WARN (yellow), 2=FAULT (red) */
    typedef struct {
        const char *name;
        int         status;         /* 0 OK, 1 WARN, 2 FAULT */
        char        detail[128];
    } HealthRow;
    HealthRow rows[12];
    int nrows = 0;

    #define ADD_ROW(NAME, STAT, ...) do { \
        rows[nrows].name   = (NAME); \
        rows[nrows].status = (STAT); \
        snprintf(rows[nrows].detail, sizeof(rows[nrows].detail), __VA_ARGS__); \
        nrows++; \
    } while(0)

    /* 1. Safety FSM */
    if(sys_state == IPC_STATE_RUNNING)
        ADD_ROW("Safety FSM",  0, "RUNNING  (normal operation)");
    else if(sys_state == IPC_STATE_SAFE)
        ADD_ROW("Safety FSM",  2, "SAFE  (FSM tripped - check other rows)");
    else
        ADD_ROW("Safety FSM",  1, "INIT / DEGRADED  (not yet stable)");

    /* 2. Radio link */
    if(nrf_status != 0)
        ADD_ROW("Radio (nRF)", 2, "NRF init FAILED  (check wiring/SPI)");
    else if(pkt_rate < 900)
        ADD_ROW("Radio (nRF)", 2, "Rate %u/s  (expected ~1000)", pkt_rate);
    else if(loss_rate > 0.01f)
        ADD_ROW("Radio (nRF)", 1, "Loss %.3f %%  (>1 %% over last 1k)", loss_rate * 100.0f);
    else
        ADD_ROW("Radio (nRF)", 0, "%u pkt/s, loss %.3f %%", pkt_rate, loss_rate * 100.0f);

    /* 3. IO realtime loop */
    if(!io_rdy)
        ADD_ROW("IO loop",     2, "IO not ready  (cpcu_io not started?)");
    else if(hb_age_ms > 100)
        ADD_ROW("IO loop",     2, "Heartbeat %u ms stale  (RT loop stalled)", hb_age_ms);
    else if(hb_age_ms > 20 || max_poll > 100)
        ADD_ROW("IO loop",     1, "hb %u ms, poll %u us", hb_age_ms, max_poll);
    else
        ADD_ROW("IO loop",     0, "hb %u ms, poll %u us", hb_age_ms, max_poll);

    /* 4. IPC ring */
    if(overflows > 0)
        ADD_ROW("IPC ring",    2, "%u overflows  (DSP can't keep up)", overflows);
    else if(ring_fill > 900)
        ADD_ROW("IPC ring",    1, "%u / %u near full", ring_fill, IPC_SENSOR_RING_SIZE);
    else if(pkts_drp > 0)
        ADD_ROW("IPC ring",    1, "%u dropped", pkts_drp);
    else
        ADD_ROW("IPC ring",    0, "%u / %u  healthy", ring_fill, IPC_SENSOR_RING_SIZE);

    /* 5. Packet sequence integrity */
    if(gaps > 50)
        ADD_ROW("Pkt integrity", 2, "%u seq gaps  (heavy loss)", gaps);
    else if(gaps > 10)
        ADD_ROW("Pkt integrity", 1, "%u seq gaps", gaps);
    else
        ADD_ROW("Pkt integrity", 0, "%u seq gaps", gaps);

    /* 6. Battery */
    if(batt_v < 2.7f && latest.vbat_raw > 0)
        ADD_ROW("Battery",     2, "%.2f V  (< 2.7 V critical, SAFE trip)", batt_v);
    else if(batt_v < 3.0f && latest.vbat_raw > 0)
        ADD_ROW("Battery",     1, "%.2f V  (< 3.0 V warning)", batt_v);
    else
        ADD_ROW("Battery",     0, "%.2f V  (pack OK)", batt_v);

    /* 7. DSP pipeline */
    if(!dsp_rdy)
        ADD_ROW("DSP pipeline", 2, "DSP not ready  (Python not started?)");
    else if(dsp_lat > 50000)
        ADD_ROW("DSP pipeline", 2, "Max latency %u us  (> 50 ms)", dsp_lat);
    else if(dsp_under > 0)
        ADD_ROW("DSP pipeline", 1, "%u ring-empty events", dsp_under);
    else if(dsp_lat > 20000)
        ADD_ROW("DSP pipeline", 1, "Max latency %u us", dsp_lat);
    else
        ADD_ROW("DSP pipeline", 0, "max lat %u us  (under budget)", dsp_lat);

    /* 8. ML export */
    if(export_seq == 0)
        ADD_ROW("ML export",   1, "No updates yet  (first cycle?)");
    else
        ADD_ROW("ML export",   0, "seq#%u  (DSP writes actively)", export_seq);

    /* 9. BSAU flags (latest packet) */
    {
        uint8_t severe = latest.flags &
                        (WL_FLAG_CLIPPING | WL_FLAG_ELEC_OFF | WL_FLAG_ADC_OVRN);
        uint8_t warn   = latest.flags & WL_FLAG_TX_SAT;
        char fbuf[64];
        wl_flags_decode(latest.flags, fbuf, sizeof(fbuf));
        if(severe)
            ADD_ROW("BSAU sensor", 2, "flags: %s  (hardware fault)", fbuf);
        else if(warn)
            ADD_ROW("BSAU sensor", 1, "flags: %s  (transient issue)", fbuf);
        else
            ADD_ROW("BSAU sensor", 0, "flags: OK  (no CLIP/ELEC/OVRN)");
    }

    /* 10. SAFE entries count */
    if(safe_ents == 0)
        ADD_ROW("SAFE trips",  0, "0  (never entered SAFE since boot)");
    else if(safe_ents < 3)
        ADD_ROW("SAFE trips",  1, "%u  (recovered)", safe_ents);
    else
        ADD_ROW("SAFE trips",  2, "%u  (persistent instability)", safe_ents);

    #undef ADD_ROW

    /*---- Count the overall statuses for summary line ----*/
    int n_ok = 0, n_warn = 0, n_fault = 0;
    for(int i = 0; i < nrows; i++)
    {
        if      (rows[i].status == 2) n_fault++;
        else if (rows[i].status == 1) n_warn++;
        else                          n_ok++;
    }

    /*---- Draw summary banner ----*/
    attron(A_BOLD);
    mvprintw(r, 1, "SYSTEM STATUS:");
    attroff(A_BOLD);
    mvprintw(r, 16, "[ ");
    attron(COLOR_PAIR(CP_GOOD) | A_BOLD); printw("%d OK", n_ok);        attroff(COLOR_PAIR(CP_GOOD) | A_BOLD);
    printw(" | ");
    attron(COLOR_PAIR(CP_WARN) | A_BOLD); printw("%d warn", n_warn);    attroff(COLOR_PAIR(CP_WARN) | A_BOLD);
    printw(" | ");
    attron(COLOR_PAIR(CP_BAD)  | A_BOLD); printw("%d fault", n_fault);  attroff(COLOR_PAIR(CP_BAD)  | A_BOLD);
    printw(" ]");

    /* Overall verdict in the right side */
    const char *verdict;
    int         verdict_cp;
    if      (n_fault > 0) { verdict = "DEGRADED";  verdict_cp = CP_BAD;  }
    else if (n_warn  > 0) { verdict = "OPERATIONAL (warnings)"; verdict_cp = CP_WARN; }
    else                  { verdict = "NOMINAL";   verdict_cp = CP_GOOD; }
    int vlen = (int)strlen(verdict);
    attron(COLOR_PAIR(verdict_cp) | A_BOLD);
    mvprintw(r, g_tui_w - vlen - 2, "%s", verdict);
    attroff(COLOR_PAIR(verdict_cp) | A_BOLD);
    r++;

    draw_hline(r, 0, g_tui_w);
    r++;

    /*---- Column headers ----*/
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    mvprintw(r, 2,  "SUBSYSTEM");
    mvprintw(r, 20, "STATUS");
    mvprintw(r, 32, "DETAIL");
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);
    r++;

    /*---- One row per subsystem ----*/
    for(int i = 0; i < nrows; i++)
    {
        mvprintw(r, 2, "%-16s", rows[i].name);

        const char *stag;
        int         scp;
        switch(rows[i].status) {
            case 0:  stag = "  OK  ";  scp = CP_GOOD; break;
            case 1:  stag = " WARN ";  scp = CP_WARN; break;
            case 2:  stag = "FAULT ";  scp = CP_BAD;  break;
            default: stag = "  ??  ";  scp = CP_DIM;  break;
        }
        attron(COLOR_PAIR(scp) | A_BOLD);
        mvprintw(r, 20, "[%s]", stag);
        attroff(COLOR_PAIR(scp) | A_BOLD);

        attron(COLOR_PAIR(rows[i].status == 0 ? CP_DIM : scp));
        mvprintw(r, 32, "%s", rows[i].detail);
        attroff(COLOR_PAIR(rows[i].status == 0 ? CP_DIM : scp));
        r++;
    }
    r++;

    /*---- Legend + reminder footer ----*/
    draw_hline(r, 0, g_tui_w);
    r++;
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    mvprintw(r, 2, "Press ");
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);
    attron(A_BOLD); printw("R"); attroff(A_BOLD);
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    printw(" in demo mode to clear injected faults and reset counters");
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);
}

/*============= DRAW: Page 7 — Dataset Capture =============================================*/

static void draw_page_dataset(int r, IPC_Context *ipc)
{
    /* Drain is done once per tick in the main loop, not here — so that
     * captures keep advancing even when the user flips to another page. */

    /* Expire the transient SAVED / CANCELLED banner after its TTL. */
    if((ds_state == DS_SAVED || ds_state == DS_CANCELLED) &&
       now_ms_wall() > ds_msg_until)
    {
        ds_state = DS_IDLE;
    }

    /*---- Header row: state | label | mode ------------------------------*/
    const char *state_tag;
    int         state_cp;
    switch(ds_state)
    {
        case DS_COLLECTING: state_tag = "* COLLECTING"; state_cp = CP_BAD;  break;
        case DS_SAVED:      state_tag = "v SAVED";      state_cp = CP_GOOD; break;
        case DS_CANCELLED:  state_tag = "x CANCELLED";  state_cp = CP_WARN; break;
        case DS_IDLE:
        default:            state_tag = "IDLE";         state_cp = CP_DIM;  break;
    }

    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, 1, "State:");
    attroff(COLOR_PAIR(CP_DIM));
    attron(COLOR_PAIR(state_cp) | A_BOLD);
    mvprintw(r, 8, "%-14s", state_tag);
    attroff(COLOR_PAIR(state_cp) | A_BOLD);

    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, 26, "Label:");
    attroff(COLOR_PAIR(CP_DIM));
    attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
    mvprintw(r, 33, "[%d] %-8s", ds_label_idx, CLS_NAMES[ds_label_idx]);
    attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);

    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, 51, "Mode:");
    attroff(COLOR_PAIR(CP_DIM));
    attron(A_BOLD);
    mvprintw(r, 57, "%s", ds_mode == DS_MODE_FILTERED ? "FILTERED" : "RAW");
    attroff(A_BOLD);
    r++;

    /*---- Stats row: samples, elapsed, gaps, missed ---------------------*/
    uint64_t elapsed_ms = 0;
    if(ds_state == DS_COLLECTING)
        elapsed_ms = now_ms_wall() - ds_start_ms;
    else if(ds_state == DS_SAVED || ds_state == DS_CANCELLED)
        /* Freeze the displayed elapsed when transient — feels right. */
        elapsed_ms = (ds_msg_until > ds_start_ms + 2000)
                   ? (ds_msg_until - ds_start_ms - 2000)
                   : 0;

    double elapsed_s = elapsed_ms / 1000.0;

    attron(COLOR_PAIR(CP_DIM)); mvprintw(r, 1,  "Samples:"); attroff(COLOR_PAIR(CP_DIM));
    attron(A_BOLD);             mvprintw(r, 10, "%-10u", ds_samples); attroff(A_BOLD);

    attron(COLOR_PAIR(CP_DIM)); mvprintw(r, 22, "Elapsed:"); attroff(COLOR_PAIR(CP_DIM));
    attron(A_BOLD);             mvprintw(r, 31, "%7.3fs", elapsed_s); attroff(A_BOLD);

    attron(COLOR_PAIR(CP_DIM)); mvprintw(r, 43, "Gaps:"); attroff(COLOR_PAIR(CP_DIM));
    {
        int cp = ds_gaps > 0 ? CP_WARN : CP_GOOD;
        attron(COLOR_PAIR(cp) | A_BOLD);
        mvprintw(r, 49, "%-6u", ds_gaps);
        attroff(COLOR_PAIR(cp) | A_BOLD);
    }

    attron(COLOR_PAIR(CP_DIM)); mvprintw(r, 58, "Missed:"); attroff(COLOR_PAIR(CP_DIM));
    {
        int cp = ds_missed > 0 ? CP_BAD : CP_GOOD;
        attron(COLOR_PAIR(cp) | A_BOLD);
        mvprintw(r, 66, "%u", ds_missed);
        attroff(COLOR_PAIR(cp) | A_BOLD);
    }
    r++;

    /*---- Paths ---------------------------------------------------------*/
    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, 1, "Out dir:  %s/", DATASET_OUT_DIR);
    attroff(COLOR_PAIR(CP_DIM));
    r++;

    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, 1, "File:     ");
    attroff(COLOR_PAIR(CP_DIM));
    if(ds_path[0])
    {
        int cp = ds_state == DS_CANCELLED ? CP_WARN
               : ds_state == DS_COLLECTING ? CP_CYAN
               : CP_GOOD;
        attron(COLOR_PAIR(cp));
        printw("%s", ds_path);
        attroff(COLOR_PAIR(cp));
    }
    else
    {
        attron(COLOR_PAIR(CP_DIM) | A_DIM);
        printw("(none yet)");
        attroff(COLOR_PAIR(CP_DIM) | A_DIM);
    }
    r++;

    /*---- Demo banner ---------------------------------------------------*/
    if(demo_mode)
    {
        attron(COLOR_PAIR(CP_CYAN) | A_DIM);
        mvprintw(r, 1,
                 "Demo mode: capture writes real CSV from synthetic %s @ %gHz packets.",
                 demo_wave_label(demo_wave), (double)demo_freq_hz);
        attroff(COLOR_PAIR(CP_CYAN) | A_DIM);
        r++;
    }

    r++;
    draw_hline(r, 0, g_tui_w);
    r++;

    /*---- Label strip ---------------------------------------------------*/
    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, 1,
             "Labels (LEFT/RIGHT cycle | s,SPACE start/stop | r cancel | t raw<->filt):");
    attroff(COLOR_PAIR(CP_DIM));
    r++;

    /* Lay out labels in one row, each padded to 7 cols so they align. */
    int x = 2;
    for(int i = 0; i < DATASET_LABEL_COUNT; i++)
    {
        if(i == ds_label_idx)
        {
            /* Selected: reverse-video. Disable selection change visually
             * while collecting so the user knows they can't change labels
             * mid-capture. */
            int cp = (ds_state == DS_COLLECTING) ? CP_BAD : CP_CYAN;
            attron(COLOR_PAIR(cp) | A_REVERSE | A_BOLD);
            mvprintw(r, x, " %-7s", CLS_NAMES[i]);
            attroff(COLOR_PAIR(cp) | A_REVERSE | A_BOLD);
        }
        else
        {
            attron(COLOR_PAIR(CP_DIM));
            mvprintw(r, x, " %-7s", CLS_NAMES[i]);
            attroff(COLOR_PAIR(CP_DIM));
        }
        x += 8;   /* 1 space + 7 label chars */
        if(x + 8 > g_tui_w) { r++; x = 2; }   /* wrap if terminal narrow */
    }
    r++;

    r++;
    draw_hline(r, 0, g_tui_w);
    r++;

    /*---- Live waveforms ------------------------------------------------*/
    {
        char fbuf[64];
        uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
        uint8_t  last_flags = (head > 0)
            ? ipc->ring[(head - 1) & IPC_SENSOR_RING_MASK].flags
            : 0;
        wl_flags_decode(last_flags, fbuf, sizeof(fbuf));
        int severe = (last_flags &
                      (WL_FLAG_CLIPPING | WL_FLAG_ELEC_OFF | WL_FLAG_ADC_OVRN));
        int cp = (fbuf[0] == '\0') ? CP_GOOD : (severe ? CP_BAD : CP_WARN);

        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, 1, "Live waveforms  (raw ADC, 8 ch, BSAU flags:");
        attroff(COLOR_PAIR(CP_DIM));
        attron(COLOR_PAIR(cp) | A_BOLD);
        printw(" %s", fbuf[0] ? fbuf : "OK");
        attroff(COLOR_PAIR(cp) | A_BOLD);
        attron(COLOR_PAIR(CP_DIM));
        printw(")");
        attroff(COLOR_PAIR(CP_DIM));
    }
    r++;

    int half_w = g_tui_w / 2;
    int plot_w = half_w - 8;
    if(plot_w < 16) plot_w = 16;
    int plot_h = 2;

    /* Stop at g_term_h - 3 to leave room for the footer (separator + line). */
    int max_r = g_term_h - 3;

    for(int i = 0; i < 4 && r + plot_h + 1 <= max_r; i++)
    {
        /* Left column: ch 0..3 */
        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, 1, "ch%d", i);
        attroff(COLOR_PAIR(CP_DIM));
        draw_waveform(r, 5, plot_w, plot_h, i, CP_GOOD);

        /* Right column: ch 4..7 */
        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, half_w + 1, "ch%d", i + 4);
        attroff(COLOR_PAIR(CP_DIM));
        draw_waveform(r, half_w + 5, plot_w, plot_h, i + 4, CP_CYAN);

        r += plot_h + 1;   /* plot rows + axis */
    }
}

/*============= DRAW: Footer ===============================================================*/

static void draw_footer(int r)
{
    draw_hline(r, 0, g_tui_w);
    r++;
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    if(demo_mode)
    {
        /* Extended footer in demo mode: shows fault-injection + waveform hotkeys */
        if(current_page == PAGE_WAVES)
            mvprintw(r, 1,
                "1-7:pg UP/DN:ch TAB q:quit | w:wave [/]:freq | F=radio B=batt G=gaps O=ring I=i2c R=reset");
        else if(current_page == PAGE_DATASET)
            mvprintw(r, 1,
                "1-7:pg q:quit | LEFT/RIGHT:label s,SPACE:start/stop r:cancel t:raw/filt | w:wave [/]:freq");
        else
            mvprintw(r, 1,
                "1-7:pg q:quit | w:wave [/]:freq | FAULT INJ: F=radio B=batt G=gaps O=ring I=i2c R=reset");
    }
    else
    {
        if(current_page == PAGE_WAVES)
            mvprintw(r, 1, "1-7:pages  UP/DN:ch  TAB:detail  q:quit  10 Hz");
        else if(current_page == PAGE_DATASET)
            mvprintw(r, 1, "1-7:pg  LEFT/RIGHT:label  s,SPACE:start/stop  r:cancel  t:raw/filt  q:quit");
        else
            mvprintw(r, 1, "1-7:pages  q:quit  10 Hz  |  read-only (zero RT impact)");
    }
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);

    /* Right-side tags: fault banner (red) takes precedence over [DEMO] */
    if(demo_mode)
    {
        const char *inj = fault_banner(demo_fault_mask);
        if(inj)
        {
            int inj_len = (int)strlen(inj);
            attron(COLOR_PAIR(CP_BAD) | A_BOLD);
            mvprintw(r, g_tui_w - inj_len - 1, "%s", inj);
            attroff(COLOR_PAIR(CP_BAD) | A_BOLD);
        }
        else
        {
            /* No fault active — show current waveform + frequency instead
             * of the plain [DEMO] tag. e.g. "[SINE 100Hz]" */
            char tag[24];
            snprintf(tag, sizeof(tag), "[%s %gHz]",
                     demo_wave_label(demo_wave), (double)demo_freq_hz);
            int tag_len = (int)strlen(tag);
            attron(COLOR_PAIR(CP_CYAN));
            mvprintw(r, g_tui_w - tag_len - 1, "%s", tag);
            attroff(COLOR_PAIR(CP_CYAN));
        }
    }
}

/*============= SPLASH =====================================================================*/

static void draw_splash(void)
{
    /* ASCII-only block art — deliberately avoids Unicode box-drawing, which
     * was reported as "garbled" in prior versions on some SSH clients.
     *
     * Each letter is a rigid 6-col × 8-row block, separated by a 2-space
     * gutter → 30 columns total. This keeps stems aligned vertically
     * across letters (previous version had mismatched widths that made
     * "CPCU" look lopsided). */
    static const char *art[] = {
        " ####   #####    ####   ##  ##",
        "##  ##  ##  ##  ##  ##  ##  ##",
        "##      ##  ##  ##      ##  ##",
        "##      #####   ##      ##  ##",
        "##      ##      ##      ##  ##",
        "##      ##      ##      ##  ##",
        "##  ##  ##      ##  ##  ##  ##",
        " ####   ##       ####    #### ",
    };
    const int lines = (int)(sizeof(art) / sizeof(art[0]));

    erase();

    /* Vertical centering: art is `lines` rows tall, plus 6 rows of text
     * below (gap, title, subtitle1, subtitle2, gap, hint). Center the
     * whole block. */
    int total_h = lines + 6;
    int cy = (g_term_h - total_h) / 2;
    if(cy < 1) cy = 1;

    int art_w = (int)strlen(art[0]);
    int cx = (g_term_w - art_w) / 2;
    if(cx < 0) cx = 0;

    attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
    for(int i = 0; i < lines; i++)
        mvprintw(cy + i, cx, "%s", art[i]);
    attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);

    const char *t1 = "CPCU Monitor v3.2-linetrace  -  Prosthetic Hand Controller";
    const char *t2 = "EE493/494 Capstone Design Project";
    const char *t3 = "METU - 2026";
    const char *t4 = "Press any key to continue";

    attron(A_BOLD);
    mvprintw(cy + lines + 2, (g_term_w - (int)strlen(t1)) / 2, "%s", t1);
    attroff(A_BOLD);
    mvprintw(cy + lines + 3, (g_term_w - (int)strlen(t2)) / 2, "%s", t2);
    mvprintw(cy + lines + 4, (g_term_w - (int)strlen(t3)) / 2, "%s", t3);

    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    mvprintw(cy + lines + 6, (g_term_w - (int)strlen(t4)) / 2, "%s", t4);
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);
    refresh();
}

static void run_splash(void)
{
    layout_update();
    draw_splash();
    nodelay(stdscr, FALSE);
    timeout(-1);                        /* Block until a key is pressed */
    getch();
    nodelay(stdscr, TRUE);
}

/*============= MAIN =======================================================================*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    for(int i = 1; i < argc; i++)
    {
        if(strcmp(argv[i], "--demo") == 0 || strcmp(argv[i], "-d") == 0)
            demo_mode = true;
        else if(strcmp(argv[i], "--no-splash") == 0)
            show_splash = false;
    }

    IPC_Context ipc;
    memset(&ipc, 0, sizeof(ipc));

    if(demo_mode)
    {
        demo_init(&ipc);
        printf("[TUI] Demo mode — synthetic data.\n");
    }
    else
    {
        if(IPC_Open(&ipc) != 0)
        {
            fprintf(stderr, "[TUI] Cannot open shared memory. Is cpcu_kernel running?\n");
            fprintf(stderr, "  Try: ./cpcu_tui --demo\n");
            return 1;
        }
    }

    initscr();
    cbreak();
    noecho();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);

    if(has_colors())
    {
        start_color();
        use_default_colors();
        init_pair(CP_NORMAL,    COLOR_WHITE,    -1);
        init_pair(CP_GOOD,      COLOR_GREEN,    -1);
        init_pair(CP_WARN,      COLOR_YELLOW,   -1);
        init_pair(CP_BAD,       COLOR_RED,      -1);
        init_pair(CP_CYAN,      COLOR_CYAN,     -1);
        init_pair(CP_DIM,       COLOR_WHITE,    -1);
        init_pair(CP_HEADER,    COLOR_BLACK,    COLOR_CYAN);
        init_pair(CP_BAR_FILL,  COLOR_GREEN,    -1);
        init_pair(CP_BAR_EMPTY, COLOR_WHITE,    -1);
        init_pair(CP_MAGENTA,   COLOR_MAGENTA,  -1);
    }

    if(show_splash)
        run_splash();

    uint32_t prev_pkts  =   0;
    uint64_t prev_time  =   0;
    uint64_t boot_time  =   now_ms();

    while(g_run)
    {
        layout_update();

        if(demo_mode) demo_tick(&ipc);

        uint32_t pkts       =   atomic_load(&ipc.diag->io_pkts_received);
        uint32_t gaps       =   atomic_load(&ipc.diag->io_seq_gaps);

        uint64_t t_now      =   now_ms();
        uint32_t pkt_rate   =   0;
        if(prev_time > 0 && t_now > prev_time)
        {
            uint32_t dt_ms = (uint32_t)(t_now - prev_time);
            if(dt_ms > 0)
                pkt_rate = (pkts - prev_pkts) * 1000 / dt_ms;
        }
        prev_pkts = pkts;
        prev_time = t_now;

        float loss_rate = (pkts > 0) ? (float)gaps / (float)pkts : 0.0f;

        uint32_t uptime_s = (uint32_t)((t_now - boot_time) / 1000);
        uint32_t up_h = uptime_s / 3600;
        uint32_t up_m = (uptime_s / 60) % 60;
        uint32_t up_s = uptime_s % 60;

        wave_peek_ring(&ipc);

        /* Drain ring to CSV every tick while capture is armed — independent
         * of which page is currently being rendered, so a capture started on
         * page 7 keeps running if the user flips to another page. */
        ds_drain_ring_to_file(&ipc);

        erase();
        int r = draw_header(0);

        switch(current_page)
        {
            case PAGE_OVERVIEW:
                draw_page_overview(r, &ipc, pkt_rate, loss_rate, up_h, up_m, up_s);
                break;
            case PAGE_RADIO:
                draw_page_radio(r, &ipc, pkt_rate, loss_rate, up_h, up_m, up_s);
                break;
            case PAGE_DSP:
                draw_page_dsp(r, &ipc);
                break;
            case PAGE_WAVES:
                draw_page_waves(r, &ipc);
                break;
            case PAGE_CONFIG:
                draw_page_config(r);
                break;
            case PAGE_HEALTH:
                draw_page_health(r, &ipc, pkt_rate, loss_rate);
                break;
            case PAGE_DATASET:
                draw_page_dataset(r, &ipc);
                break;
            default:
                break;
        }

        draw_footer(g_term_h - 2);
        refresh();

        int ch = getch();
        switch(ch)
        {
            case '1': current_page = PAGE_OVERVIEW; break;
            case '2': current_page = PAGE_RADIO;    break;
            case '3': current_page = PAGE_DSP;      break;
            case '4': current_page = PAGE_WAVES;    break;
            case '5': current_page = PAGE_CONFIG;   break;
            case '6': current_page = PAGE_HEALTH;   break;
            case '7': current_page = PAGE_DATASET;  break;
            case KEY_UP:
                if(current_page == PAGE_WAVES)
                    wave_sel_ch = (wave_sel_ch - 1 + WL_NUM_CHANNELS) % WL_NUM_CHANNELS;
                break;
            case KEY_DOWN:
                if(current_page == PAGE_WAVES)
                    wave_sel_ch = (wave_sel_ch + 1) % WL_NUM_CHANNELS;
                break;
            case KEY_LEFT:
                /* Page 7: cycle label (blocked while collecting). */
                if(current_page == PAGE_DATASET && ds_state != DS_COLLECTING)
                    ds_label_idx = (ds_label_idx - 1 + DATASET_LABEL_COUNT)
                                 % DATASET_LABEL_COUNT;
                break;
            case KEY_RIGHT:
                if(current_page == PAGE_DATASET && ds_state != DS_COLLECTING)
                    ds_label_idx = (ds_label_idx + 1) % DATASET_LABEL_COUNT;
                break;
            case '\t':
                if(current_page == PAGE_WAVES)
                    wave_detail = !wave_detail;
                break;

            /*-- Page 7: dataset capture controls -----------------------*/
            case 's': case 'S': case ' ':
                if(current_page == PAGE_DATASET)
                {
                    if(ds_state == DS_COLLECTING)
                        ds_stop_capture(true);
                    else if(ds_state == DS_IDLE)
                        (void)ds_start_capture(&ipc);
                    /* else: transient SAVED/CANCELLED banner active — ignore */
                }
                break;
            case 't': case 'T':
                if(current_page == PAGE_DATASET && ds_state != DS_COLLECTING)
                    ds_mode = (ds_mode == DS_MODE_FILTERED)
                            ? DS_MODE_RAW : DS_MODE_FILTERED;
                break;

            /*-- Fault-injection hotkeys (demo mode only) --*/
            case 'f': case 'F':
                if(demo_mode)
                {
                    demo_fault_mask ^= FAULT_RADIO_FREEZE;
                    if(demo_fault_mask & FAULT_RADIO_FREEZE)
                        demo_fault_onset_ms = now_ms_wall();
                }
                break;
            case 'b': case 'B':
                if(demo_mode) demo_fault_mask ^= FAULT_BATT_LOW;
                break;
            case 'g': case 'G':
                if(demo_mode) demo_fault_mask ^= FAULT_GAP_STORM;
                break;
            case 'o': case 'O':
                if(demo_mode) demo_fault_mask ^= FAULT_RING_OVF;
                break;
            case 'i': case 'I':
                if(demo_mode) demo_fault_mask ^= FAULT_I2C_FAIL;
                break;
            case 'r': case 'R':
                /* On page 7 while collecting: cancel-and-delete.
                 * Everywhere else in demo mode: reset fault injections. */
                if(current_page == PAGE_DATASET && ds_state == DS_COLLECTING)
                {
                    ds_stop_capture(false);
                }
                else if(demo_mode)
                {
                    /* Full reset — clear fault injections AND zero out the
                     * accumulated counters so the UI immediately snaps back
                     * to a "clean boot" look. Without this, sticky counters
                     * like seq gaps / inferences / batches keep their old
                     * values even after the fault is cleared. */
                    demo_fault_mask     = FAULT_NONE;
                    demo_fault_onset_ms = 0;
                    demo_pkts           = 0;
                    demo_gaps           = 0;
                    demo_inf_count      = 0;
                    demo_gesture        = 0;
                    atomic_store(&ipc.ctrl->system_state,          IPC_STATE_RUNNING);
                    atomic_store(&ipc.ctrl->io_ready,              1);
                    atomic_store(&ipc.ctrl->dsp_ready,             1);
                    atomic_store(&ipc.diag->io_pkts_received,      0);
                    atomic_store(&ipc.diag->io_pkts_dropped,       0);
                    atomic_store(&ipc.diag->io_seq_gaps,           0);
                    atomic_store(&ipc.diag->io_ring_overflows,     0);
                    atomic_store(&ipc.diag->io_safe_entries,       0);
                    atomic_store(&ipc.diag->io_max_poll_us,        0);
                    atomic_store(&ipc.diag->dsp_batches,           0);
                    atomic_store(&ipc.diag->dsp_inferences,        0);
                    atomic_store(&ipc.diag->dsp_max_latency_us,    0);
                    atomic_store(&ipc.diag->dsp_ring_underflows,   0);
                }
                break;

            /*-- Demo waveform selection (cpcu_tui --demo only) --*/
            case 'w': case 'W':
                if(demo_mode)
                {
                    /* Cycle SINE(1) → ... → CHIRP(8) → SINE(1) */
                    demo_wave = (DemoWave)(((int)demo_wave % 8) + 1);
                }
                break;
            case '[':
                if(demo_mode)
                {
                    demo_freq_hz *= 0.5f;
                    if(demo_freq_hz < 10.0f)    demo_freq_hz = 10.0f;
                }
                break;
            case ']':
                if(demo_mode)
                {
                    demo_freq_hz *= 2.0f;
                    if(demo_freq_hz > 1000.0f)  demo_freq_hz = 1000.0f;
                }
                break;

            case 'q': case 'Q': g_run = 0; break;
            default: break;
        }

        usleep(REFRESH_US);
    }

    endwin();

    /* Save anything that was being captured when the user quit — better
     * than silently dropping the file. A partial file is still useful
     * if it was running for long enough. */
    if(ds_state == DS_COLLECTING)
    {
        ds_stop_capture(true);
        printf("[TUI] Auto-saved in-progress capture: %s  (%u samples)\n",
               ds_path, ds_samples);
    }

    if(!demo_mode) IPC_Close(&ipc);
    printf("[TUI] Exited cleanly.\n");
    return 0;
}

/*==========================================================================================*/
