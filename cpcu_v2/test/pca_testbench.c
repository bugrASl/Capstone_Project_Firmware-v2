/**
 *  @file       pca_testbench.c
 *  @brief      Standalone PCA9685 servo testbench with ncurses TUI
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    1.2
 *
 *  @details    Interactive servo calibration and testing tool. Talks
 *              directly to the PCA9685 over I2C — no kernel or IPC needed.
 *
 *              v1.2 changes:
 *                  - Scenario engine: F1-F6 run predefined motion profiles
 *                    (step, sweep, triangle, small_step, burst, slow_creep)
 *                    on the selected servo. Auto-enables the smoother.
 *                    Tune velocity/accel/deadband with v/a/d keys, then
 *                    re-run the same scenario to feel the difference.
 *                  - Esc aborts a running scenario (returns to neutral).
 *                  - Live progress bar in the detail panel during scenarios.
 *
 *              v1.1 changes:
 *                  - Full-screen dynamic layout via getmaxyx().
 *                  - --min / --max CLI arguments for fast re-calibration
 *                    without recompiling:
 *                        sudo ./pca_testbench \
 *                            --min 600,1100,1100,1000,1000,950 \
 *                            --max 2400,1900,1900,2000,2000,1700
 *                  - --smooth enables the same slew-rate limiter used in
 *                    cpcu_io.c. When on, the slider commands become
 *                    *targets*; SMOOTH_Update moves the real servos there
 *                    at the limited rate. Good for visual QA of the
 *                    smoother's behaviour on real hardware.
 *                  - 'r' dumps PCA MODE1/MODE2/PRESCALE registers by
 *                    reading them live via PCA_ReadReg (previously
 *                    unexercised API). Useful when a servo is twitching
 *                    and you want to confirm the chip hasn't been reset
 *                    under you.
 *                  - 'A' (all at once) uses PCA_SetAllServos — also
 *                    previously unexercised.
 *
 *  Build:      gcc -o pca_testbench pca_testbench.c cpcu_pca9685.c \
 *                  cpcu_smooth.c -lncurses -lm
 *  Run:        sudo ./pca_testbench
 *              sudo ./pca_testbench /dev/i2c-1
 *              sudo ./pca_testbench --smooth
 *              sudo ./pca_testbench --min 600,1100,1100,1000,1000,950 \
 *                                   --max 2400,1900,1900,2000,2000,1700
 *  Quit:       press 'q'
 */

#include <ncurses.h>
#include <locale.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "cpcu_pca9685.h"
#include "cpcu_smooth.h"
#include "cpcu_config.h"        /* v2.3.6: load/save calibration */

/*============= COLOR PAIRS ================================================================*/

#define CP_NORMAL       1
#define CP_GOOD         2
#define CP_WARN         3
#define CP_BAD          4
#define CP_CYAN         5
#define CP_DIM          6
#define CP_HEADER       7
#define CP_SELECTED     8
#define CP_MAGENTA      9

/*============= LAYOUT =====================================================================*/

#define STEP_FINE       10          /* Arrow key step (us). MUST be >
                                       PCA9685's 4.88 us/tick resolution
                                       so each keypress reliably crosses
                                       a tick boundary. 5 us was on the
                                       boundary and was a no-op every
                                       other press. 10 us = ~2 ticks
                                       per press = always visible. Also
                                       coincides with SMOOTH_DEFAULT_DEAD-
                                       BAND so a sub-deadband manual move
                                       can never be silently swallowed. */
#define STEP_COARSE     50          /* Page Up/Down step (us) */
#define REFRESH_US      50000       /* 20 Hz */
#define SMOOTH_DT_US    50000       /* 20 Hz update if smoother is on */

/* Absolute hardware safety limits. The per-servo servo_min/servo_max
 * (loaded from runtime.json) are the *calibrated* limits — they're
 * what gets enforced during normal operation. But during calibration
 * here in the testbench, the user needs to be able to push *past*
 * the current MIN/MAX to discover the true mechanical range, then
 * record that with [ or ]. So jog operations clamp only to these
 * absolute hardware limits, not to the soft calibration values.
 *
 * 500 µs / 2500 µs are the conservative bounds for hobby servos:
 * almost no commercially available servo can travel beyond these,
 * and most stop responding at their internal endstops well before
 * reaching them. Commanding outside this range won't damage the
 * PCA9685 either — it just produces a pulse the servo's controller
 * ignores or clamps internally. So this is safe to expose as the
 * jog ceiling/floor.
 *
 * The [ and ] keys (set MIN, set MAX) write the *current* pulse
 * width into servo_min/servo_max regardless of how it compares to
 * the previous values, so this lets the user widen the calibrated
 * range. */
#define PCA_HARD_FLOOR_US   500
#define PCA_HARD_CEIL_US    2500

/* Populated every frame by layout_update() */
static int  g_term_w    =   80;
static int  g_term_h    =   24;
static int  g_tui_w     =   80;
static int  g_slider_w  =   40;

static void layout_update(void)
{
    getmaxyx(stdscr, g_term_h, g_term_w);
    g_tui_w     =   g_term_w;
    if(g_tui_w < 72) g_tui_w = 72;
    /* Slider: ~ half of the width, bounded */
    g_slider_w  =   g_tui_w - 36;
    if(g_slider_w < 20) g_slider_w = 20;
    if(g_slider_w > 60) g_slider_w = 60;
}

/*============= SERVO NAMES ================================================================*/

static const char *SERVO_NAMES[PCA_SERVO_COUNT] =
{
    "S0 MG995 Base   ",
    "S1 MG995 Upper  ",
    "S2 MG995 Last   ",
    "S3 SG90  Joint-1",
    "S4 SG90  Joint-2",
    "S5 SG90  Gripper",
};

/*============= GLOBALS ====================================================================*/

static volatile sig_atomic_t g_run = 1;
static void on_sig(int s) { (void)s; g_run = 0; }

static PCA_Handle       pca;
static SMOOTH_Context   smooth;
static uint16_t         servo_us[PCA_SERVO_COUNT];      /* user-commanded target */
static int              selected        =   0;
static bool             hw_connected    =   false;
static bool             smooth_enabled  =   false;
static const char      *i2c_dev         =   "/dev/i2c-1";
static uint8_t          i2c_addr        =   PCA_I2C_ADDR_BASE;

/* Live register snapshot triggered by 'r' */
static bool             show_regs   =   false;
static uint8_t          reg_mode1   =   0;
static uint8_t          reg_mode2   =   0;
static uint8_t          reg_presc   =   0;

/* v2.3.6: calibration state. servo_bias[] is loaded from runtime.json
 * on startup and saved back on 'S'. cal_dirty tracks whether the
 * in-memory state has diverged from disk so we can warn before quit
 * and indicate to the user that there are unsaved changes. */
static int16_t          servo_bias[PCA_SERVO_COUNT] = {0};
static char             cfg_path_used[256]          = {0};
static bool             cal_dirty                   = false;
static char             status_line[256]            = {0};
static time_t           status_until                = 0;

/* Help-overlay toggle. '?' or 'h' flips this; when true, draw_screen
 * renders a full-screen explanation instead of the normal layout. */
static bool             g_help_visible              = false;

/* v2.3.6: per-channel smoother state, mirrored to runtime.json on
 * save. Stored as int16 for symmetry with servo_bias / the patcher
 * API; values fit comfortably (max 10000 us/s velocity, 50000 us/s^2
 * accel — both within int16 if we use int16 for vel and uint16 for
 * accel and just clamp carefully).
 *
 * v/a/d keystrokes cycle through preset values rather than entering
 * numbers — single keypress, no modal numeric entry, immediately
 * felt in motion. The presets are the values experienced builders
 * tend to want: 'slow / normal / fast' for each axis. */
static uint16_t         smooth_vel[PCA_SERVO_COUNT];      /* us/s */
static uint16_t         smooth_acc[PCA_SERVO_COUNT];      /* us/s^2 */
static uint16_t         smooth_dead[PCA_SERVO_COUNT];     /* us */

/* Presets cycled by v/a/d. First entry is the default (the value
 * SMOOTH_Init applies). Cycling wraps. Chosen to span the useful
 * tuning range for typical hobby servos: very slow for delicate
 * manipulation, very fast for "is this the smoother that's slow or
 * the SVM that's slow?" diagnosis. */
static const uint16_t   VEL_PRESETS[]  = { 2000, 500, 1000, 1500, 3000, 5000 };
static const uint16_t   ACC_PRESETS[]  = { 8000, 2000, 4000, 6000, 12000, 20000 };
static const uint16_t   DEAD_PRESETS[] = { 10, 0, 5, 15, 25, 50 };
#define VEL_PRESET_COUNT  (int)(sizeof(VEL_PRESETS)  / sizeof(VEL_PRESETS[0]))
#define ACC_PRESET_COUNT  (int)(sizeof(ACC_PRESETS)  / sizeof(ACC_PRESETS[0]))
#define DEAD_PRESET_COUNT (int)(sizeof(DEAD_PRESETS) / sizeof(DEAD_PRESETS[0]))

/* Find current preset index (or -1 if the value isn't a preset).
 * On cycle, we step from this index to the next, or to index 0 if
 * not on a preset (so a custom value loaded from JSON jumps to the
 * first preset on the first press). */
static int find_preset_idx(uint16_t value, const uint16_t *list, int n)
{
    for(int i = 0; i < n; i++) if(list[i] == value) return i;
    return -1;
}

/* Fine-step adjustment: ',' and '.' nudge the most-recently-touched
 * smoother knob (vel/acc/dead) for the SELECTED servo. This pairs
 * with v/a/d (cycle preset) — press v to ballpark, ',' / '.' to
 * dial in to an exact value. The "last touched" semantic is global
 * (not per-servo), so switching servos with UP/DOWN preserves which
 * knob is being adjusted.
 *
 * Step sizes match each knob's natural granularity:
 *   velocity  100 us/s    (range 100..10000 per JSON loader)
 *   accel     500 us/s^2  (range 500..50000)
 *   deadband    1 us      (range 0..50) */
typedef enum { SMK_NONE = 0, SMK_VEL, SMK_ACC, SMK_DEAD } SmoothKnob;
static SmoothKnob        last_smoother_knob = SMK_NONE;
#define VEL_STEP    100
#define ACC_STEP    500
#define DEAD_STEP   1
#define VEL_LO      100
#define VEL_HI      10000
#define ACC_LO      500
#define ACC_HI      50000
#define DEAD_LO     0
#define DEAD_HI     50

/*============= SCENARIO ENGINE ============================================================*/
/*
 *  Predefined motion profiles that exercise the selected servo through
 *  the smoother. Triggered by F1-F6, runs on the currently selected
 *  servo. The smoother is auto-enabled if off. Other servos hold their
 *  current position during the scenario.
 *
 *  Each waypoint is (hold_seconds, target_us). The engine builds an
 *  absolute timeline on start, then sets the smoother target each tick.
 *  The existing smoother update in the main loop drives the PCA.
 */

#define SC_MAX_WAYPOINTS    8
#define SC_COUNT            6

typedef struct {
    float       t;              /* absolute time (seconds) */
    uint16_t    target;         /* target pulse width */
} SC_Waypoint;

typedef struct {
    const char *name;
    const char *key_label;      /* F1, F2, etc. */
    int         wp_count;       /* filled at start time (servo-dependent) */
} SC_Def;

static const SC_Def SC_DEFS[SC_COUNT] = {
    { "step",       "F1" },
    { "sweep",      "F2" },
    { "triangle",   "F3" },
    { "small_step", "F4" },
    { "burst",      "F5" },
    { "slow_creep", "F6" },
};

/* Build waypoints for a scenario + servo. Returns waypoint count. */
static int sc_build(int sc_idx, int servo_idx,
                    SC_Waypoint out[SC_MAX_WAYPOINTS])
{
    uint16_t mn  = pca.servo_min[servo_idx];
    uint16_t mx  = pca.servo_max[servo_idx];
    uint16_t mid = (mn + mx) / 2;
    uint16_t neu = PCA_SERVO_NEUTRAL;
    int n = 0;

    switch(sc_idx)
    {
        case 0: /* step: neutral -> max -> neutral */
            out[n++] = (SC_Waypoint){ 0.0f, neu };
            out[n++] = (SC_Waypoint){ 1.0f, mx  };
            out[n++] = (SC_Waypoint){ 3.5f, neu };
            break;
        case 1: /* sweep: neutral -> min -> max -> neutral */
            out[n++] = (SC_Waypoint){ 0.0f, neu };
            out[n++] = (SC_Waypoint){ 1.0f, mn  };
            out[n++] = (SC_Waypoint){ 3.0f, mx  };
            out[n++] = (SC_Waypoint){ 5.0f, neu };
            break;
        case 2: /* triangle: mid -> max -> min -> max -> mid */
            out[n++] = (SC_Waypoint){ 0.0f, mid };
            out[n++] = (SC_Waypoint){ 0.5f, mx  };
            out[n++] = (SC_Waypoint){ 2.0f, mn  };
            out[n++] = (SC_Waypoint){ 3.5f, mx  };
            out[n++] = (SC_Waypoint){ 5.0f, mid };
            break;
        case 3: /* small_step: ±50 around neutral */
        {
            uint16_t lo = (neu >= mn + 50) ? neu - 50 : mn;
            uint16_t hi = (neu + 50 <= mx) ? neu + 50 : mx;
            out[n++] = (SC_Waypoint){ 0.0f, neu };
            out[n++] = (SC_Waypoint){ 1.0f, hi  };
            out[n++] = (SC_Waypoint){ 3.0f, lo  };
            out[n++] = (SC_Waypoint){ 5.0f, neu };
            break;
        }
        case 4: /* burst: rapid min/max */
            out[n++] = (SC_Waypoint){ 0.0f, neu };
            out[n++] = (SC_Waypoint){ 0.3f, mx  };
            out[n++] = (SC_Waypoint){ 1.1f, mn  };
            out[n++] = (SC_Waypoint){ 1.9f, mx  };
            out[n++] = (SC_Waypoint){ 2.7f, mn  };
            out[n++] = (SC_Waypoint){ 3.5f, neu };
            break;
        case 5: /* slow_creep: neutral -> neutral+200 */
        {
            uint16_t hi = (neu + 200 <= mx) ? neu + 200 : mx;
            out[n++] = (SC_Waypoint){ 0.0f, neu };
            out[n++] = (SC_Waypoint){ 1.0f, hi  };
            out[n++] = (SC_Waypoint){ 5.0f, neu };
            break;
        }
    }
    return n;
}

/* Runtime state */
static bool         sc_active       = false;
static int          sc_index        = -1;       /* which scenario (0-5) */
static int          sc_servo        = -1;       /* which servo it's running on */
static int          sc_wp_count     = 0;
static SC_Waypoint  sc_wps[SC_MAX_WAYPOINTS];
static float        sc_total_time   = 0.0f;
static struct timespec sc_start_ts;

/* Forward declaration — defined after sc_tick, called from within it. */
static void clamp_servo(int idx);

static float sc_elapsed(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (float)(now.tv_sec - sc_start_ts.tv_sec)
         + (float)(now.tv_nsec - sc_start_ts.tv_nsec) * 1e-9f;
}

static void sc_start(int scenario, int servo)
{
    sc_wp_count = sc_build(scenario, servo, sc_wps);
    if(sc_wp_count < 2) return;

    sc_active     = true;
    sc_index      = scenario;
    sc_servo      = servo;
    sc_total_time = sc_wps[sc_wp_count - 1].t + 1.5f;  /* +1.5s settle */
    clock_gettime(CLOCK_MONOTONIC, &sc_start_ts);

    /* Auto-enable smoother — the whole point of scenarios */
    if(!smooth_enabled)
    {
        smooth_enabled = true;
        for(int i = 0; i < PCA_SERVO_COUNT; i++)
        {
            smooth.target[i]    = servo_us[i];
            smooth.current[i]   = servo_us[i];
            smooth.current_f[i] = (float)servo_us[i];
        }
    }

    snprintf(status_line, sizeof(status_line),
             "SCENARIO: %s on %s — running...",
             SC_DEFS[scenario].name, SERVO_NAMES[servo]);
    status_until = time(NULL) + (long)sc_total_time + 2;
}

/* Called every main-loop tick while sc_active. Sets the smoother target
 * based on elapsed time, then the existing smoother update drives PCA. */
static void sc_tick(void)
{
    if(!sc_active) return;

    float elapsed = sc_elapsed();
    if(elapsed >= sc_total_time)
    {
        /* Done — return to neutral */
        servo_us[sc_servo] = PCA_SERVO_NEUTRAL;
        clamp_servo(sc_servo);
        SMOOTH_SetTarget(&smooth, sc_servo, servo_us[sc_servo]);

        snprintf(status_line, sizeof(status_line),
                 "SCENARIO: %s on %s — done (%.1fs)",
                 SC_DEFS[sc_index].name, SERVO_NAMES[sc_servo], elapsed);
        status_until = time(NULL) + 4;
        sc_active = false;
        return;
    }

    /* Find current waypoint */
    int wp = 0;
    for(int i = sc_wp_count - 1; i >= 0; i--)
    {
        if(elapsed >= sc_wps[i].t) { wp = i; break; }
    }

    uint16_t target = sc_wps[wp].target;
    servo_us[sc_servo] = target;
    SMOOTH_SetTarget(&smooth, sc_servo, target);

    /* Update status with progress bar */
    float pct = elapsed / sc_total_time * 100.0f;
    int bar_w = 20;
    int filled = (int)(pct / 100.0f * bar_w);
    if(filled > bar_w) filled = bar_w;
    char bar[32];
    for(int i = 0; i < bar_w; i++)
        bar[i] = (i < filled) ? '#' : '.';
    bar[bar_w] = '\0';

    snprintf(status_line, sizeof(status_line),
             "RUN %s [%s] %3.0f%%  tgt=%uus  wp=%d/%d",
             SC_DEFS[sc_index].name, bar, pct,
             target, wp + 1, sc_wp_count);
    status_until = time(NULL) + 2;
}

/*============= HELPERS ====================================================================*/

static void clamp_servo(int idx)
{
    if(servo_us[idx] < pca.servo_min[idx])  servo_us[idx] = pca.servo_min[idx];
    if(servo_us[idx] > pca.servo_max[idx])  servo_us[idx] = pca.servo_max[idx];
}

static float servo_fraction(int idx)
{
    uint16_t range = pca.servo_max[idx] - pca.servo_min[idx];
    if(range == 0) return 0.0f;
    return (float)(servo_us[idx] - pca.servo_min[idx]) / (float)range;
}

static uint16_t servo_counts(int idx)
{
    return PCA_PulseToCounts(servo_us[idx]);
}

/**
 *  Write the user-commanded target to hardware. When --smooth is on, the
 *  target is handed to the smoother and the PCA is driven from
 *  smooth.current[] inside main(); otherwise we write the user target
 *  directly like v1.0 did.
 */
static void write_servo(int idx)
{
    clamp_servo(idx);
    if(!hw_connected) return;

    if(smooth_enabled)
    {
        SMOOTH_SetTarget(&smooth, idx, servo_us[idx]);
    }
    else
    {
        PCA_SetServo(&pca, (uint8_t)idx, servo_us[idx]);
    }
}

static void write_all_servos(void)
{
    if(!hw_connected) return;

    if(smooth_enabled)
    {
        for(int i = 0; i < PCA_SERVO_COUNT; i++)
        {
            clamp_servo(i);
            SMOOTH_SetTarget(&smooth, i, servo_us[i]);
        }
    }
    else
    {
        for(int i = 0; i < PCA_SERVO_COUNT; i++)  clamp_servo(i);
        /* Exercise PCA_SetAllServos (bulk update) instead of looping */
        PCA_SetAllServos(&pca, servo_us);
    }
}

/**
 *  Parse comma-separated "a,b,c,d,e,f" into an array of 6 uint16_t.
 *  Returns the number of successfully-parsed values.
 */
static int parse_csv_u16(const char *s, uint16_t *out, int max_n)
{
    int n = 0;
    while(*s && n < max_n)
    {
        char *end = NULL;
        long v = strtol(s, &end, 10);
        if(end == s) break;
        if(v < 0)    v = 0;
        if(v > 0xFFFF) v = 0xFFFF;
        out[n++] = (uint16_t)v;
        s = end;
        if(*s == ',') s++;
    }
    return n;
}

/*============= DRAWING ====================================================================*/

static void draw_hline(int row, int col, int len)
{
    mvhline(row, col, ACS_HLINE, len);
}

static void draw_slider(int row, int col, int idx, int width, bool is_selected)
{
    float frac  = servo_fraction(idx);
    int pos     = (int)(frac * (width - 1));

    move(row, col);
    for(int i = 0; i < width; i++)
    {
        if(i == pos)
        {
            int cp = CP_GOOD;
            if(frac < 0.05f || frac > 0.95f)       cp = CP_BAD;
            else if(frac < 0.15f || frac > 0.85f)   cp = CP_WARN;

            if(is_selected) cp = CP_SELECTED;

            attron(COLOR_PAIR(cp) | A_BOLD);
            addch(is_selected ? ACS_DIAMOND : 'O');
            attroff(COLOR_PAIR(cp) | A_BOLD);
        }
        else
        {
            int cp = is_selected ? CP_CYAN : CP_DIM;
            attron(COLOR_PAIR(cp) | (is_selected ? 0 : A_DIM));
            addch(ACS_HLINE);
            attroff(COLOR_PAIR(cp) | (is_selected ? 0 : A_DIM));
        }
    }
}

static void draw_servo_row(int row, int idx, bool is_selected)
{
    if(is_selected)
    {
        attron(COLOR_PAIR(CP_SELECTED) | A_BOLD);
        mvprintw(row, 1, ">");
        attroff(COLOR_PAIR(CP_SELECTED) | A_BOLD);
    }
    else
    {
        mvprintw(row, 1, " ");
    }

    int name_cp = is_selected ? CP_SELECTED : CP_NORMAL;
    attron(COLOR_PAIR(name_cp) | (is_selected ? A_BOLD : 0));
    mvprintw(row, 3, "%-16s", SERVO_NAMES[idx]);
    attroff(COLOR_PAIR(name_cp) | (is_selected ? A_BOLD : 0));

    draw_slider(row, 21, idx, g_slider_w, is_selected);

    int val_cp = is_selected ? CP_SELECTED : CP_CYAN;
    attron(COLOR_PAIR(val_cp) | A_BOLD);
    printw(" %4u us", servo_us[idx]);
    attroff(COLOR_PAIR(val_cp) | A_BOLD);

    attron(COLOR_PAIR(CP_DIM));
    printw(" (%3u)", servo_counts(idx));

    /* Show live hardware value (possibly different if smoothing is still
     * chasing the target) — this is also our SMOOTH_AllSettled visual hint */
    if(smooth_enabled && hw_connected)
    {
        uint16_t live = smooth.current[idx];
        uint16_t tgt  = servo_us[idx];
        if(live != tgt)
            printw(" -> %4u", live);
        else
            printw(" = ok  ");
    }
    attroff(COLOR_PAIR(CP_DIM));
}

/* ─────────────────────────────────────────────────────────────────
 *  HELP OVERLAY
 *
 *  Toggled by '?' or 'h'. Walks the user through what this tool is
 *  for, the typical workflow, and groups the keys by task. The
 *  normal layout is replaced (not overlaid) so even on a small
 *  terminal the help is fully readable.
 *
 *  The text below is intentionally written for someone who has
 *  never used pca_testbench before. Returning users can press '?'
 *  again to dismiss.
 * ───────────────────────────────────────────────────────────────── */
static void draw_help(void)
{
    erase();
    int r = 0;

    /* Two-column layout: keys on the left, explanation on the right.
     * Centred title bar across the full width. */
    const int LEFT_COL  = 2;
    const int RIGHT_COL = (g_tui_w / 2) + 1;
    const int RIGHT_W   = g_tui_w - RIGHT_COL - 2;
    (void)RIGHT_W;  /* reserved for future right-column truncation */

    /* ─── Header ─── */
    attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
    mvprintw(r, 0, "%-*s", g_tui_w,
             "  PCA9685 SERVO TESTBENCH — HELP");
    attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);
    r += 2;

    /* ─── What this is ─── */
    attron(A_BOLD);
    mvprintw(r++, LEFT_COL, "WHAT THIS TOOL IS");
    attroff(A_BOLD);
    mvprintw(r++, LEFT_COL, "  Direct-drive servo calibration. Talks to the PCA9685 chip");
    mvprintw(r++, LEFT_COL, "  over I2C — no kernel, no IPC, no DSP. You use it to teach");
    mvprintw(r++, LEFT_COL, "  the system the safe pulse-width range of each servo before");
    mvprintw(r++, LEFT_COL, "  the kernel ever drives the arm with EMG signals.");
    r++;

    /* ─── Workflow ─── */
    attron(A_BOLD);
    mvprintw(r++, LEFT_COL, "TYPICAL WORKFLOW (do this once per servo)");
    attroff(A_BOLD);
    mvprintw(r++, LEFT_COL, "  1. UP/DOWN  to pick a servo (the > marker shows which)");
    mvprintw(r++, LEFT_COL, "  2. LEFT/RIGHT  jog -10/+10us until the servo just reaches its");
    mvprintw(r++, LEFT_COL, "                 mechanical end-stop. Listen for the stall.");
    mvprintw(r++, LEFT_COL, "  3. Back off ~10-20us so you don't command the stall point.");
    mvprintw(r++, LEFT_COL, "  4. Press [  to record that position as the new MIN. Repeat");
    mvprintw(r++, LEFT_COL, "             at the other end of travel and press ]  for MAX.");
    mvprintw(r++, LEFT_COL, "  5. Press n  to verify the neutral 1500us pose looks right.");
    mvprintw(r++, LEFT_COL, "             If not, jog to where neutral SHOULD be and press");
    mvprintw(r++, LEFT_COL, "             b  to record that offset as BIAS.");
    mvprintw(r++, LEFT_COL, "  6. Press S  to save the calibration to runtime.json.");
    mvprintw(r++, LEFT_COL, "  7. If the kernel is running:  kill -HUP $(pgrep cpcu_kernel)");
    mvprintw(r++, LEFT_COL, "                                to apply changes live.");
    r++;

    /* ─── Status fields ─── */
    attron(A_BOLD);
    mvprintw(r++, LEFT_COL, "STATUS LINE AT THE TOP");
    attroff(A_BOLD);
    mvprintw(r++, LEFT_COL, "  I2C Device:  CONNECTED  if /dev/i2c-1 opened + PCA ACK'd");
    mvprintw(r++, LEFT_COL, "  bus=...      which I2C bus device is in use");
    mvprintw(r++, LEFT_COL, "  addr=0x40    PCA9685 7-bit I2C address (its default)");
    mvprintw(r++, LEFT_COL, "  pre=121      PWM prescaler register. 121 == ~50 Hz refresh,");
    mvprintw(r++, LEFT_COL, "               which is the standard hobby-servo update rate.");
    r++;

    attron(A_BOLD);
    mvprintw(r++, LEFT_COL, "SELECTED-SERVO PANEL (middle of the screen)");
    attroff(A_BOLD);
    mvprintw(r++, LEFT_COL, "  Range:    the recorded safe MIN-MAX pulse range");
    mvprintw(r++, LEFT_COL, "  Current:  what's being commanded RIGHT NOW (in us, then");
    mvprintw(r++, LEFT_COL, "            counts -> us round-trip through the PCA chip)");
    mvprintw(r++, LEFT_COL, "  Angle:    estimated joint angle from current pulse + range");
    mvprintw(r++, LEFT_COL, "  Bias:     signed offset added before clamping to MIN-MAX");

    /* ─── Right-side column: keys + explanations ─── */
    int rr = 2;     /* start one row below the header line */

    attron(A_BOLD);
    mvprintw(rr++, RIGHT_COL, "MOVE THE SELECTED SERVO");
    attroff(A_BOLD);
    attron(COLOR_PAIR(CP_DIM));
    mvprintw(rr++, RIGHT_COL, "  UP/DOWN     pick which servo");
    mvprintw(rr++, RIGHT_COL, "  LEFT/RIGHT  jog -/+ 10us");
    mvprintw(rr++, RIGHT_COL, "  PgUp/PgDn   jog -/+ 50us  (coarse)");
    mvprintw(rr++, RIGHT_COL, "  n           snap to neutral 1500us");
    mvprintw(rr++, RIGHT_COL, "  m  M        snap to recorded MIN/MAX");
    mvprintw(rr++, RIGHT_COL, "  N           ALL servos to neutral");
    mvprintw(rr++, RIGHT_COL, "  A           write ALL servos in one bulk");
    mvprintw(rr++, RIGHT_COL, "              I2C transaction (vs default of");
    mvprintw(rr++, RIGHT_COL, "              one-at-a-time)");
    attroff(COLOR_PAIR(CP_DIM));
    rr++;

    attron(A_BOLD);
    mvprintw(rr++, RIGHT_COL, "RECORD CALIBRATION (RAM only)");
    attroff(A_BOLD);
    attron(COLOR_PAIR(CP_DIM));
    mvprintw(rr++, RIGHT_COL, "  [           set MIN to current pulse");
    mvprintw(rr++, RIGHT_COL, "  ]           set MAX to current pulse");
    mvprintw(rr++, RIGHT_COL, "  b  B        set / clear BIAS offset");
    attroff(COLOR_PAIR(CP_DIM));
    rr++;

    attron(A_BOLD);
    mvprintw(rr++, RIGHT_COL, "TUNE MOTION SMOOTHER");
    attroff(A_BOLD);
    attron(COLOR_PAIR(CP_DIM));
    mvprintw(rr++, RIGHT_COL, "  s    smoother on/off (default OFF)");
    mvprintw(rr++, RIGHT_COL, "  v    cycle VELocity preset");
    mvprintw(rr++, RIGHT_COL, "       (max us/s the servo will travel)");
    mvprintw(rr++, RIGHT_COL, "  a    cycle ACCeleration preset");
    mvprintw(rr++, RIGHT_COL, "       (us/s^2 ramp-up/down rate)");
    mvprintw(rr++, RIGHT_COL, "  d    cycle DEADband preset");
    mvprintw(rr++, RIGHT_COL, "       (us window where small changes");
    mvprintw(rr++, RIGHT_COL, "        suppress jitter)");
    mvprintw(rr++, RIGHT_COL, "  ,  . fine -/+ on the last v/a/d");
    attroff(COLOR_PAIR(CP_DIM));
    rr++;

    attron(A_BOLD);
    mvprintw(rr++, RIGHT_COL, "PERSIST / DIAGNOSE");
    attroff(A_BOLD);
    attron(COLOR_PAIR(CP_DIM));
    mvprintw(rr++, RIGHT_COL, "  S    save to runtime.json");
    mvprintw(rr++, RIGHT_COL, "  L    reload from runtime.json");
    mvprintw(rr++, RIGHT_COL, "  r    re-read I2C registers");
    mvprintw(rr++, RIGHT_COL, "  0    all servos OFF (PWM disabled,");
    mvprintw(rr++, RIGHT_COL, "       motors go limp)");
    mvprintw(rr++, RIGHT_COL, "  ?  h toggle this help");
    mvprintw(rr++, RIGHT_COL, "  q  Q quit (warns on unsaved)");
    attroff(COLOR_PAIR(CP_DIM));
    rr++;

    attron(A_BOLD);
    mvprintw(rr++, RIGHT_COL, "MOTION SCENARIOS (smoother auto-ON)");
    attroff(A_BOLD);
    attron(COLOR_PAIR(CP_DIM));
    mvprintw(rr++, RIGHT_COL, "  F1   step     neu->max->neu");
    mvprintw(rr++, RIGHT_COL, "  F2   sweep    neu->min->max->neu");
    mvprintw(rr++, RIGHT_COL, "  F3   triangle mid->max->min->max");
    mvprintw(rr++, RIGHT_COL, "  F4   small    +/-50us (deadband)");
    mvprintw(rr++, RIGHT_COL, "  F5   burst    rapid min/max toggle");
    mvprintw(rr++, RIGHT_COL, "  F6   creep    neu->neu+200 (ramp)");
    mvprintw(rr++, RIGHT_COL, "  Esc  abort running scenario");
    mvprintw(rr++, RIGHT_COL, "  Runs on the UP/DOWN-selected servo.");
    mvprintw(rr++, RIGHT_COL, "  Tune v/a/d then re-run to compare.");
    attroff(COLOR_PAIR(CP_DIM));

    /* ─── Footer ─── */
    if(g_term_h >= 4)
    {
        int footer_r = g_term_h - 1;
        attron(COLOR_PAIR(CP_HEADER));
        mvprintw(footer_r, 0, "%-*s", g_tui_w,
                 "  Press '?' or 'h' again to return to the calibration view");
        attroff(COLOR_PAIR(CP_HEADER));
    }
}

static void draw_screen(void)
{
    /* If help overlay is on, hand off entirely. The user keeps the
     * key state (selected servo, draft values, etc.) so dismissing
     * help drops them back exactly where they were. */
    if(g_help_visible)
    {
        draw_help();
        return;
    }

    erase();
    int r = 0;

    /* HEADER */
    attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
    mvprintw(r, 0, "%-*s", g_tui_w, "  PCA9685 SERVO TESTBENCH - InfiniTech v1.2");
    attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);
    r += 2;

    /* Connection status */
    mvprintw(r, 1, "I2C Device:");
    attron(COLOR_PAIR(hw_connected ? CP_GOOD : CP_BAD) | A_BOLD);
    printw("  %s", hw_connected ? "CONNECTED" : "NOT CONNECTED (dry-run)");
    attroff(COLOR_PAIR(hw_connected ? CP_GOOD : CP_BAD) | A_BOLD);

    if(hw_connected)
    {
        attron(COLOR_PAIR(CP_DIM));
        printw("  bus=%s addr=0x%02X  pre=%u", i2c_dev, pca.addr, pca.prescaler);
        attroff(COLOR_PAIR(CP_DIM));
    }

    if(smooth_enabled)
    {
        mvprintw(r, g_tui_w - 12, "[SMOOTH:%s]",
                 SMOOTH_AllSettled(&smooth) ? "IDLE" : "MOVE");
    }
    r += 2;

    draw_hline(r - 1, 0, g_tui_w);

    mvprintw(r, 1, "SERVO");
    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, 21, "%-*s", g_slider_w, "MIN                              MAX");
    mvprintw(r, 21 + g_slider_w + 1, " pulse (cnt)  live");
    attroff(COLOR_PAIR(CP_DIM));
    r++;

    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        draw_servo_row(r, i, i == selected);
        r++;
    }
    r++;

    /* DETAIL BOX for selected servo */
    draw_hline(r - 1, 0, g_tui_w);

    attron(A_BOLD);
    mvprintw(r, 1, "SELECTED: %s", SERVO_NAMES[selected]);
    attroff(A_BOLD);
    r++;

    mvprintw(r, 3, "Range:   ");
    attron(COLOR_PAIR(CP_CYAN));
    printw("%4u - %4u us", pca.servo_min[selected], pca.servo_max[selected]);
    attroff(COLOR_PAIR(CP_CYAN));
    mvprintw(r, 38, "(%3u - %3u counts)",
           PCA_PulseToCounts(pca.servo_min[selected]),
           PCA_PulseToCounts(pca.servo_max[selected]));
    r++;

    mvprintw(r, 3, "Current: ");
    attron(COLOR_PAIR(CP_GOOD) | A_BOLD);
    printw("%4u us", servo_us[selected]);
    attroff(COLOR_PAIR(CP_GOOD) | A_BOLD);
    /* Show PCA_CountsToPulse round-trip too — previously unused */
    uint16_t cnts = servo_counts(selected);
    uint16_t back = PCA_CountsToPulse(cnts);
    mvprintw(r, 38, "(%3u counts -> %4u us)", cnts, back);
    r++;

    float angle_est = servo_fraction(selected) * 180.0f;
    mvprintw(r, 3, "Angle:   ");
    attron(COLOR_PAIR(CP_MAGENTA) | A_BOLD);
    printw("~%.1f deg", angle_est);
    attroff(COLOR_PAIR(CP_MAGENTA) | A_BOLD);
    /* v2.3.6: bias display in the same row */
    mvprintw(r, 38, "Bias: ");
    if(servo_bias[selected] != 0)
    {
        attron(COLOR_PAIR(CP_WARN) | A_BOLD);
        printw("%+d us", (int)servo_bias[selected]);
        attroff(COLOR_PAIR(CP_WARN) | A_BOLD);
    }
    else
    {
        attron(COLOR_PAIR(CP_DIM));
        printw("none");
        attroff(COLOR_PAIR(CP_DIM));
    }
    r++;
    /* v2.3.6: smoother knobs for the selected servo */
    mvprintw(r, 3, "Smooth:  ");
    attron(COLOR_PAIR(CP_CYAN));
    printw("vel %5u us/s    acc %5u us/s^2    dead %2u us",
           smooth_vel[selected], smooth_acc[selected], smooth_dead[selected]);
    attroff(COLOR_PAIR(CP_CYAN));
    r++;

    /* Scenario progress line (only when running on this servo) */
    if(sc_active && sc_servo == selected)
    {
        float elapsed = sc_elapsed();
        float pct = elapsed / sc_total_time * 100.0f;
        if(pct > 100.0f) pct = 100.0f;
        int bar_w = 25;
        int filled = (int)(pct / 100.0f * bar_w);

        mvprintw(r, 3, "Scenario:");
        attron(COLOR_PAIR(CP_WARN) | A_BOLD);
        printw(" %s ", SC_DEFS[sc_index].name);
        attroff(COLOR_PAIR(CP_WARN) | A_BOLD);
        attron(COLOR_PAIR(CP_GOOD));
        printw("[");
        for(int i = 0; i < bar_w; i++)
            addch(i < filled ? '#' : '.');
        printw("] %3.0f%%", pct);
        attroff(COLOR_PAIR(CP_GOOD));

        /* Show smoother chasing indicator */
        uint16_t live = smooth.current[selected];
        uint16_t tgt  = servo_us[selected];
        if(live != tgt)
        {
            attron(COLOR_PAIR(CP_CYAN));
            printw("  %u->%u", live, tgt);
            attroff(COLOR_PAIR(CP_CYAN));
        }
        r++;
    }

    /* Live register read-back */
    if(show_regs && hw_connected)
    {
        draw_hline(r - 1, 0, g_tui_w);
        attron(A_BOLD);
        mvprintw(r, 1, "LIVE REGISTERS");
        attroff(A_BOLD);
        r++;
        attron(COLOR_PAIR(CP_CYAN));
        mvprintw(r, 3, "MODE1=0x%02X  MODE2=0x%02X  PRESCALE=0x%02X",
                 reg_mode1, reg_mode2, reg_presc);
        attroff(COLOR_PAIR(CP_CYAN));
        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r + 1, 3, "(AI=%c SLEEP=%c RESTART=%c  OUTDRV=%c INVRT=%c)",
                 (reg_mode1 & PCA_MODE1_AI)      ? 'Y' : 'N',
                 (reg_mode1 & PCA_MODE1_SLEEP)   ? 'Y' : 'N',
                 (reg_mode1 & PCA_MODE1_RESTART) ? 'Y' : 'N',
                 (reg_mode2 & PCA_MODE2_OUTDRV)  ? 'Y' : 'N',
                 (reg_mode2 & PCA_MODE2_INVRT)   ? 'Y' : 'N');
        attroff(COLOR_PAIR(CP_DIM));
        r += 3;
    }

    /* KEYBINDINGS — two-column layout with fixed-width key column.
     * Each cell is 12 chars for the key label, then the action.
     * If you change the format string here, keep the columns aligned.
     * Press '?' for the full annotated reference. */
    draw_hline(r - 1, 0, g_tui_w);
    attron(COLOR_PAIR(CP_DIM));

    /* Row 0 — discovery hint stands alone, full width */
    mvprintw(r,     1, "  %-12s %-32s   %-12s %s",
             "?", "show help (full reference)", "q", "quit");

    /* Row 1 — selecting + jogging */
    {
        char fine_str[32], coarse_str[32];
        snprintf(fine_str,   sizeof(fine_str),   "jog -/+ %dus",         STEP_FINE);
        snprintf(coarse_str, sizeof(coarse_str), "jog -/+ %dus (coarse)", STEP_COARSE);

        mvprintw(r + 1, 1, "  %-12s %-32s   %-12s %s",
                 "UP/DOWN",     "pick servo",
                 "LEFT/RIGHT",  fine_str);
        mvprintw(r + 2, 1, "  %-12s %-32s   %-12s %s",
                 "PgUp/PgDn",   coarse_str,
                 "n",           "snap to neutral 1500us");
    }
    mvprintw(r + 3, 1, "  %-12s %-32s   %-12s %s",
             "m / M",       "snap to recorded MIN / MAX",
             "N / A",       "all neutral / bulk write all");

    /* Row 4 — calibration recording */
    mvprintw(r + 4, 1, "  %-12s %-32s   %-12s %s",
             "[  /  ]",     "set MIN / MAX from current",
             "b / B",       "set / clear BIAS");

    /* Row 5 — smoother controls */
    mvprintw(r + 5, 1, "  %-12s %-32s   %-12s %s",
             "v / a / d",   "cycle VEL / ACC / DEAD preset",
             ", / .",       "fine -/+ on last v/a/d knob");

    /* Row 6 — persistence + diagnose, plus smoother on/off state */
    mvprintw(r + 6, 1, "  %-12s %-32s   %-12s %s",
             "S / L",       "save / reload runtime.json",
             "r",           "refresh I2C registers");
    mvprintw(r + 7, 1, "  %-12s %-32s   %-12s smoother: %s",
             "0",           "all servos OFF (PWM disabled)",
             "s",           smooth_enabled ? "ON " : "OFF");

    /* Row 8 — scenario keys */
    mvprintw(r + 8, 1, "  %-12s %-32s   %-12s %s",
             "F1-F6",      "run scenario on selected servo",
             "Esc",        sc_active ? "ABORT scenario" : "");

    /* Row 9 — scenario names */
    mvprintw(r + 9, 1, "   F1=step F2=sweep F3=triangle F4=small_step F5=burst F6=slow_creep");

    attroff(COLOR_PAIR(CP_DIM));
    r += 10;

    /* v2.3.6: status line for save/load/calibration feedback. Shown
     * for ~3-5 seconds (set per action). The dirty marker shows
     * persistently whenever there are unsaved changes. */
    if(status_until > 0 && time(NULL) <= status_until)
    {
        attron(COLOR_PAIR(CP_GOOD) | A_BOLD);
        mvprintw(r, 1, "%-*s", g_tui_w - 2, status_line);
        attroff(COLOR_PAIR(CP_GOOD) | A_BOLD);
        r++;
    }
    else if(cal_dirty)
    {
        attron(COLOR_PAIR(CP_WARN));
        mvprintw(r, 1, "* unsaved calibration changes — press 'S' to save *");
        attroff(COLOR_PAIR(CP_WARN));
        r++;
    }

    /* FOOTER */
    draw_hline(r, 0, g_tui_w);
    r++;
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    if(cfg_path_used[0])
        mvprintw(r, 1, "20 Hz refresh  |  direct I2C (no IPC/kernel)  |  cfg: %s",
                 cfg_path_used);
    else
        mvprintw(r, 1, "20 Hz refresh  |  direct I2C (no IPC/kernel)  |  cfg: <none>");
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);

    refresh();
}

/*============= MAIN =======================================================================*/

static void print_usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [i2c-dev] [addr] [options]\n"
        "  i2c-dev           default /dev/i2c-1\n"
        "  addr              default 0x40\n"
        "Options:\n"
        "  --min A,B,C,D,E,F   override per-servo MIN pulse widths (us)\n"
        "  --max A,B,C,D,E,F   override per-servo MAX pulse widths (us)\n"
        "  --smooth            use the slew-rate limiter (same as cpcu_io)\n"
        "  --config <path>     runtime.json to load + save into\n"
        "                      (default: /opt/cpcu/config.json then config/runtime.json)\n"
        "  --help              this message\n"
        "Calibration keys (v2.3.6):\n"
        "  [          set the current jog as MIN for the selected servo\n"
        "  ]          set the current jog as MAX for the selected servo\n"
        "  b          set the current deviation from neutral as BIAS\n"
        "  B          clear BIAS for the selected servo\n"
        "  v          cycle smoother VELOCITY preset for selected servo\n"
        "  a          cycle smoother ACCELERATION preset\n"
        "  d          cycle smoother DEADBAND preset\n"
        "  , .        fine -/+ on the LAST-touched smoother knob (vel/acc/dead)\n"
        "             step sizes: vel %d us/s, acc %d us/s^2, dead %d us\n"
        "  S          save min/max/bias/smoother values to runtime.json\n"
        "  L          reload from disk (discards unsaved jogs)\n"
        "Example workflow:\n"
        "  sudo %s --config config/runtime.json\n"
        "  # jog with arrows, press [ at the mechanical min, ] at the max,\n"
        "  # then S to save. Then on the live system: kill -HUP $(pgrep cpcu_kernel)\n",
        prog, VEL_STEP, ACC_STEP, DEAD_STEP, prog);
}

int main(int argc, char *argv[])
{
    uint16_t override_min[PCA_SERVO_COUNT] = {0};
    uint16_t override_max[PCA_SERVO_COUNT] = {0};
    bool     have_min = false, have_max = false;
    const char *cli_config_path = NULL;             /* v2.3.6 */

    for(int i = 1; i < argc; i++)
    {
        if(strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
        {
            print_usage(argv[0]);
            return 0;
        }
        else if(strcmp(argv[i], "--min") == 0 && i + 1 < argc)
        {
            int n = parse_csv_u16(argv[++i], override_min, PCA_SERVO_COUNT);
            if(n != PCA_SERVO_COUNT)
            {
                fprintf(stderr, "--min needs %d comma-separated values, got %d\n",
                        PCA_SERVO_COUNT, n);
                return 1;
            }
            have_min = true;
        }
        else if(strcmp(argv[i], "--max") == 0 && i + 1 < argc)
        {
            int n = parse_csv_u16(argv[++i], override_max, PCA_SERVO_COUNT);
            if(n != PCA_SERVO_COUNT)
            {
                fprintf(stderr, "--max needs %d comma-separated values, got %d\n",
                        PCA_SERVO_COUNT, n);
                return 1;
            }
            have_max = true;
        }
        else if(strcmp(argv[i], "--smooth") == 0)
        {
            smooth_enabled = true;
        }
        else if(strcmp(argv[i], "--config") == 0 && i + 1 < argc)
        {
            cli_config_path = argv[++i];
        }
        else if(argv[i][0] != '-')
        {
            /* Positional: first = device, second = addr */
            if(i == 1) i2c_dev = argv[i];
            else if(i == 2) i2c_addr = (uint8_t)strtol(argv[i], NULL, 0);
        }
    }

    setlocale(LC_ALL, "");
    signal(SIGINT,  on_sig);
    signal(SIGTERM, on_sig);

    /* Try to init PCA9685 hardware */
    PCA_Status st = PCA_Init(&pca, i2c_dev, i2c_addr);
    if(st == PCA_OK)
    {
        hw_connected = true;
    }
    else
    {
        /* Dry-run mode: load limits manually so TUI still works */
        fprintf(stderr, "[TESTBENCH] PCA init failed (status=%d) - dry-run mode\n", st);
        const uint16_t mins[] = PCA_SERVO_MIN_US;
        const uint16_t maxs[] = PCA_SERVO_MAX_US;
        memcpy(pca.servo_min, mins, sizeof(mins));
        memcpy(pca.servo_max, maxs, sizeof(maxs));
        pca.addr = i2c_addr;
        hw_connected = false;
    }

    /* Apply CLI overrides on top of whatever init produced */
    if(have_min)
    {
        for(int i = 0; i < PCA_SERVO_COUNT; i++) pca.servo_min[i] = override_min[i];
        fprintf(stderr, "[TESTBENCH] MIN overridden from CLI.\n");
    }
    if(have_max)
    {
        for(int i = 0; i < PCA_SERVO_COUNT; i++) pca.servo_max[i] = override_max[i];
        fprintf(stderr, "[TESTBENCH] MAX overridden from CLI.\n");
    }

    /* v2.3.6: Load runtime.json so the testbench starts with whatever
     * the user has previously calibrated. Two-tier path search matches
     * cpcu_kernel: prefer the system symlink, fall back to the in-repo
     * file. CLI overrides above already won; this fills in everything
     * else (and per-servo bias). On any failure, keep the compile-time
     * defaults — pca_testbench is a bench tool, refusing to start would
     * be hostile to the workflow.
     */
    {
        const char *paths[] = {
            cli_config_path ? cli_config_path : "/opt/cpcu/config.json",
            "config/runtime.json",
            NULL
        };
        IPC_RuntimeConfig cfg;
        char err[256] = {0};
        bool cfg_loaded = false;
        for(int p = 0; paths[p]; p++)
        {
            if(!paths[p]) continue;
            CFG_Status st_cfg = CFG_LoadFromFile(paths[p], &cfg, err, sizeof(err));
            if(st_cfg == CFG_OK)
            {
                if(!have_min)
                    for(int i = 0; i < PCA_SERVO_COUNT; i++)
                        pca.servo_min[i] = cfg.servo_min_us[i];
                if(!have_max)
                    for(int i = 0; i < PCA_SERVO_COUNT; i++)
                        pca.servo_max[i] = cfg.servo_max_us[i];
                for(int i = 0; i < PCA_SERVO_COUNT; i++)
                {
                    servo_bias[i] = cfg.servo_bias_us[i];
                    /* v2.3.6: smoother values. Zero means "use default";
                     * we substitute the SMOOTH_DEFAULT_* values so the
                     * UI shows a real number rather than 0. */
                    smooth_vel[i]  = cfg.smooth_velocity_us_per_s[i]
                                     ? cfg.smooth_velocity_us_per_s[i]
                                     : VEL_PRESETS[0];
                    smooth_acc[i]  = cfg.smooth_accel_us_per_s2[i]
                                     ? cfg.smooth_accel_us_per_s2[i]
                                     : ACC_PRESETS[0];
                    smooth_dead[i] = cfg.smooth_deadband_us[i];
                }
                strncpy(cfg_path_used, paths[p], sizeof(cfg_path_used) - 1);
                cfg_loaded = true;
                fprintf(stderr,
                        "[TESTBENCH] Loaded calibration from %s\n", paths[p]);
                break;
            }
        }
        if(!cfg_loaded)
        {
            fprintf(stderr, "[TESTBENCH] No usable runtime.json found "
                            "(last err: %s) - using compile-time defaults\n",
                    err[0] ? err : "no candidates tried");
            /* Defaults so the UI shows real numbers either way. */
            for(int i = 0; i < PCA_SERVO_COUNT; i++)
            {
                smooth_vel[i]  = VEL_PRESETS[0];
                smooth_acc[i]  = ACC_PRESETS[0];
                smooth_dead[i] = DEAD_PRESETS[0];
            }
        }
    }

    /* Start all servos at neutral */
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        servo_us[i] = PCA_SERVO_NEUTRAL;
        clamp_servo(i);
    }

    /* Init smoother (used only if --smooth) */
    SMOOTH_Init(&smooth, PCA_SERVO_NEUTRAL);
    /* v2.3.6: apply per-channel smoother values loaded from
     * runtime.json (or the defaults if no JSON was loaded). The
     * 'v'/'a'/'d' keys cycle these at runtime; 'S' saves them back. */
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        SMOOTH_SetSpeed(&smooth, i, smooth_vel[i]);
        SMOOTH_SetAccel(&smooth, i, smooth_acc[i]);
        SMOOTH_SetDeadband(&smooth, i, smooth_dead[i]);
    }

    if(hw_connected)
    {
        if(smooth_enabled)
        {
            for(int i = 0; i < PCA_SERVO_COUNT; i++)
                SMOOTH_SetTarget(&smooth, i, servo_us[i]);
            SMOOTH_Snap(&smooth);
        }
        PCA_SetAllServos(&pca, servo_us);
    }

    /* Init ncurses */
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);
    timeout(0);

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
        init_pair(CP_SELECTED,  COLOR_YELLOW,   -1);
        init_pair(CP_MAGENTA,   COLOR_MAGENTA,  -1);
    }

    struct timespec t_last, t_now;
    clock_gettime(CLOCK_MONOTONIC, &t_last);

    /* Main loop */
    while(g_run)
    {
        layout_update();
        draw_screen();

        /* Smoother step (if enabled) — it's the testbench's equivalent of
         * the 50 Hz servo tick inside cpcu_io. */
        if(smooth_enabled && hw_connected)
        {
            clock_gettime(CLOCK_MONOTONIC, &t_now);
            uint32_t dt_us = (uint32_t)((t_now.tv_sec - t_last.tv_sec) * 1000000ULL
                          + (t_now.tv_nsec - t_last.tv_nsec) / 1000ULL);
            t_last = t_now;

            SMOOTH_Update(&smooth, dt_us);
            for(int s = 0; s < PCA_SERVO_COUNT; s++)
                PCA_SetServo(&pca, (uint8_t)s, smooth.current[s]);
        }

        /* Scenario engine tick — advances waypoints, sets targets.
         * Must run BEFORE getch() so the scenario progresses even
         * when no key is pressed. The smoother update above drives
         * the PCA from the targets we set here. */
        sc_tick();

        int ch = getch();
        switch(ch)
        {
            case KEY_UP:
                selected = (selected - 1 + PCA_SERVO_COUNT) % PCA_SERVO_COUNT;
                break;
            case KEY_DOWN:
                selected = (selected + 1) % PCA_SERVO_COUNT;
                break;

            case KEY_LEFT:
                /* Allow probing below the recorded servo_min during
                 * calibration. Clamp only to hardware-absolute floor. */
                if(servo_us[selected] >= PCA_HARD_FLOOR_US + STEP_FINE)
                    servo_us[selected] -= STEP_FINE;
                else
                    servo_us[selected] = PCA_HARD_FLOOR_US;
                write_servo(selected);
                break;
            case KEY_RIGHT:
                /* Allow probing above the recorded servo_max during
                 * calibration. Clamp only to hardware-absolute ceiling. */
                if(servo_us[selected] <= PCA_HARD_CEIL_US - STEP_FINE)
                    servo_us[selected] += STEP_FINE;
                else
                    servo_us[selected] = PCA_HARD_CEIL_US;
                write_servo(selected);
                break;

            case KEY_PPAGE:
                if(servo_us[selected] <= PCA_HARD_CEIL_US - STEP_COARSE)
                    servo_us[selected] += STEP_COARSE;
                else
                    servo_us[selected] = PCA_HARD_CEIL_US;
                write_servo(selected);
                break;
            case KEY_NPAGE:
                if(servo_us[selected] >= PCA_HARD_FLOOR_US + STEP_COARSE)
                    servo_us[selected] -= STEP_COARSE;
                else
                    servo_us[selected] = PCA_HARD_FLOOR_US;
                write_servo(selected);
                break;

            case 'n':
                servo_us[selected] = PCA_SERVO_NEUTRAL;
                clamp_servo(selected);
                write_servo(selected);
                break;
            case 'N':
                for(int i = 0; i < PCA_SERVO_COUNT; i++)
                {
                    servo_us[i] = PCA_SERVO_NEUTRAL;
                    clamp_servo(i);
                }
                write_all_servos();
                break;
            case 'm':
                servo_us[selected] = pca.servo_min[selected];
                write_servo(selected);
                break;
            case 'M':
                servo_us[selected] = pca.servo_max[selected];
                write_servo(selected);
                break;
            case 'A':
                write_all_servos();     /* exercises PCA_SetAllServos */
                break;

            /* v2.3.6: calibration keys --------------------------------*/
            /* The user has jogged to a position they like — these keys
             * commit that position as a calibration value. Nothing
             * persists to disk until 'S'. */
            case '[':
                /* Capture current jog as new MIN for this servo. */
                pca.servo_min[selected] = servo_us[selected];
                cal_dirty = true;
                snprintf(status_line, sizeof(status_line),
                         "Servo %d MIN = %u us (unsaved)",
                         selected, servo_us[selected]);
                status_until = time(NULL) + 3;
                break;
            case ']':
                /* Capture current jog as new MAX. */
                pca.servo_max[selected] = servo_us[selected];
                cal_dirty = true;
                snprintf(status_line, sizeof(status_line),
                         "Servo %d MAX = %u us (unsaved)",
                         selected, servo_us[selected]);
                status_until = time(NULL) + 3;
                break;
            case 'b':
                /* Capture deviation from neutral as this servo's bias.
                 * If the user jogged to 1518 us as the visually-neutral
                 * resting pose, bias becomes +18 us. cpcu_io will add
                 * this to whatever target the smoother computes. */
                {
                    int delta = (int)servo_us[selected] - (int)PCA_SERVO_NEUTRAL;
                    if(delta < -100) delta = -100;
                    if(delta >  100) delta =  100;
                    servo_bias[selected] = (int16_t)delta;
                    cal_dirty = true;
                    snprintf(status_line, sizeof(status_line),
                             "Servo %d BIAS = %+d us (unsaved)",
                             selected, delta);
                    status_until = time(NULL) + 3;
                }
                break;
            case 'B':
                /* Clear bias for this servo. */
                servo_bias[selected] = 0;
                cal_dirty = true;
                snprintf(status_line, sizeof(status_line),
                         "Servo %d BIAS cleared (unsaved)", selected);
                status_until = time(NULL) + 3;
                break;

            /* v2.3.6: smoother per-channel cycling. Each press steps
             * to the next preset for the SELECTED servo. The change
             * is applied immediately to the smoother so the next
             * jog uses the new values — you can feel the difference
             * within ~50 ms. Save with 'S' to persist to JSON.
             *
             * After cycling, ',' / '.' fine-adjust the same knob. */
            case 'v':
                {
                    int idx = find_preset_idx(smooth_vel[selected],
                                              VEL_PRESETS, VEL_PRESET_COUNT);
                    int next = (idx < 0) ? 0 : ((idx + 1) % VEL_PRESET_COUNT);
                    smooth_vel[selected] = VEL_PRESETS[next];
                    SMOOTH_SetSpeed(&smooth, selected, smooth_vel[selected]);
                    last_smoother_knob = SMK_VEL;
                    cal_dirty = true;
                    snprintf(status_line, sizeof(status_line),
                             "Servo %d VEL = %u us/s (unsaved, ',' / '.' to fine-adjust)",
                             selected, smooth_vel[selected]);
                    status_until = time(NULL) + 3;
                }
                break;
            case 'a':
                {
                    int idx = find_preset_idx(smooth_acc[selected],
                                              ACC_PRESETS, ACC_PRESET_COUNT);
                    int next = (idx < 0) ? 0 : ((idx + 1) % ACC_PRESET_COUNT);
                    smooth_acc[selected] = ACC_PRESETS[next];
                    SMOOTH_SetAccel(&smooth, selected, smooth_acc[selected]);
                    last_smoother_knob = SMK_ACC;
                    cal_dirty = true;
                    snprintf(status_line, sizeof(status_line),
                             "Servo %d ACC = %u us/s^2 (unsaved, ',' / '.' to fine-adjust)",
                             selected, smooth_acc[selected]);
                    status_until = time(NULL) + 3;
                }
                break;
            case 'd':
                {
                    int idx = find_preset_idx(smooth_dead[selected],
                                              DEAD_PRESETS, DEAD_PRESET_COUNT);
                    int next = (idx < 0) ? 0 : ((idx + 1) % DEAD_PRESET_COUNT);
                    smooth_dead[selected] = DEAD_PRESETS[next];
                    SMOOTH_SetDeadband(&smooth, selected, smooth_dead[selected]);
                    last_smoother_knob = SMK_DEAD;
                    cal_dirty = true;
                    snprintf(status_line, sizeof(status_line),
                             "Servo %d DEAD = %u us (unsaved, ',' / '.' to fine-adjust)",
                             selected, smooth_dead[selected]);
                    status_until = time(NULL) + 3;
                }
                break;

            /* v2.3.6: ',' and '.' fine-adjust the LAST-TOUCHED smoother
             * knob (vel/acc/dead) for the SELECTED servo. Range-clamped
             * to match the JSON loader's accepted bounds so save can't
             * fail later on a value the bench let you type. */
            case ',':
            case '.':
                {
                    bool inc = (ch == '.');
                    if(last_smoother_knob == SMK_NONE)
                    {
                        snprintf(status_line, sizeof(status_line),
                                 "Press v/a/d first to select a smoother knob, "
                                 "then ',' / '.' to fine-adjust");
                        status_until = time(NULL) + 3;
                        break;
                    }
                    if(last_smoother_knob == SMK_VEL)
                    {
                        int v = (int)smooth_vel[selected]
                              + (inc ? VEL_STEP : -VEL_STEP);
                        if(v < VEL_LO) v = VEL_LO;
                        if(v > VEL_HI) v = VEL_HI;
                        smooth_vel[selected] = (uint16_t)v;
                        SMOOTH_SetSpeed(&smooth, selected, smooth_vel[selected]);
                        cal_dirty = true;
                        snprintf(status_line, sizeof(status_line),
                                 "Servo %d VEL = %u us/s (unsaved)",
                                 selected, smooth_vel[selected]);
                    }
                    else if(last_smoother_knob == SMK_ACC)
                    {
                        int v = (int)smooth_acc[selected]
                              + (inc ? ACC_STEP : -ACC_STEP);
                        if(v < ACC_LO) v = ACC_LO;
                        if(v > ACC_HI) v = ACC_HI;
                        smooth_acc[selected] = (uint16_t)v;
                        SMOOTH_SetAccel(&smooth, selected, smooth_acc[selected]);
                        cal_dirty = true;
                        snprintf(status_line, sizeof(status_line),
                                 "Servo %d ACC = %u us/s^2 (unsaved)",
                                 selected, smooth_acc[selected]);
                    }
                    else /* SMK_DEAD */
                    {
                        int v = (int)smooth_dead[selected]
                              + (inc ? DEAD_STEP : -DEAD_STEP);
                        if(v < DEAD_LO) v = DEAD_LO;
                        if(v > DEAD_HI) v = DEAD_HI;
                        smooth_dead[selected] = (uint16_t)v;
                        SMOOTH_SetDeadband(&smooth, selected, smooth_dead[selected]);
                        cal_dirty = true;
                        snprintf(status_line, sizeof(status_line),
                                 "Servo %d DEAD = %u us (unsaved)",
                                 selected, smooth_dead[selected]);
                    }
                    status_until = time(NULL) + 3;
                }
                break;

            case 'S':
                /* Save calibration to runtime.json (or wherever
                 * cfg_path_used pointed). Patch only the three fields
                 * we own — gesture_velocity etc. survive untouched
                 * thanks to CFG_PatchFile's surgical edit semantics. */
                if(cfg_path_used[0] == '\0')
                {
                    snprintf(status_line, sizeof(status_line),
                             "SAVE FAILED: no config file was loaded "
                             "(retry with --config <path>)");
                    status_until = time(NULL) + 5;
                }
                else
                {
                    /* uint16 -> int16 for the patch API. Both arrays
                     * are well within int16 range (positive servo
                     * pulses are ~500-2500). */
                    int16_t mins_i16[PCA_SERVO_COUNT];
                    int16_t maxs_i16[PCA_SERVO_COUNT];
                    int16_t vel_i16[PCA_SERVO_COUNT];
                    int16_t acc_i16[PCA_SERVO_COUNT];
                    int16_t dead_i16[PCA_SERVO_COUNT];
                    for(int i = 0; i < PCA_SERVO_COUNT; i++)
                    {
                        mins_i16[i] = (int16_t)pca.servo_min[i];
                        maxs_i16[i] = (int16_t)pca.servo_max[i];
                        /* v2.3.6: smoother values. uint16 -> int16
                         * conversion is safe for vel (max preset 5000)
                         * and dead (max 50). For accel, max preset is
                         * 20000 which fits in int16's 32767. */
                        vel_i16[i]  = (int16_t)smooth_vel[i];
                        acc_i16[i]  = (int16_t)smooth_acc[i];
                        dead_i16[i] = (int16_t)smooth_dead[i];
                    }
                    CFG_PatchEntry patches[] = {
                        { "servo_min_us",             mins_i16,    PCA_SERVO_COUNT },
                        { "servo_max_us",             maxs_i16,    PCA_SERVO_COUNT },
                        { "servo_bias_us",            servo_bias,  PCA_SERVO_COUNT },
                        { "smooth_velocity_us_per_s", vel_i16,     PCA_SERVO_COUNT },
                        { "smooth_accel_us_per_s2",   acc_i16,     PCA_SERVO_COUNT },
                        { "smooth_deadband_us",       dead_i16,    PCA_SERVO_COUNT },
                    };
                    char err[256] = {0};
                    CFG_Status sst = CFG_PatchFile(cfg_path_used, patches,
                                                   sizeof(patches)/sizeof(patches[0]),
                                                   err, sizeof(err));
                    if(sst == CFG_OK)
                    {
                        cal_dirty = false;
                        snprintf(status_line, sizeof(status_line),
                                 "SAVED to %s — kill -HUP cpcu_kernel "
                                 "to reload live", cfg_path_used);
                        status_until = time(NULL) + 5;
                    }
                    else
                    {
                        snprintf(status_line, sizeof(status_line),
                                 "SAVE FAILED: %s (%s)",
                                 CFG_StatusStr(sst), err);
                        status_until = time(NULL) + 5;
                    }
                }
                break;
            case 'L':
                /* Reload from disk — discards unsaved jogs. */
                if(cfg_path_used[0] == '\0')
                {
                    snprintf(status_line, sizeof(status_line),
                             "RELOAD: no config file in use");
                    status_until = time(NULL) + 3;
                }
                else
                {
                    IPC_RuntimeConfig cfg;
                    char err[256] = {0};
                    CFG_Status sst = CFG_LoadFromFile(cfg_path_used, &cfg,
                                                     err, sizeof(err));
                    if(sst == CFG_OK)
                    {
                        for(int i = 0; i < PCA_SERVO_COUNT; i++)
                        {
                            pca.servo_min[i] = cfg.servo_min_us[i];
                            pca.servo_max[i] = cfg.servo_max_us[i];
                            servo_bias[i]    = cfg.servo_bias_us[i];
                            /* v2.3.6: smoother values too. Apply them
                             * back to the live smoother so the next
                             * jog feels different (or the same — the
                             * user can compare against unsaved). */
                            smooth_vel[i]  = cfg.smooth_velocity_us_per_s[i]
                                             ? cfg.smooth_velocity_us_per_s[i]
                                             : VEL_PRESETS[0];
                            smooth_acc[i]  = cfg.smooth_accel_us_per_s2[i]
                                             ? cfg.smooth_accel_us_per_s2[i]
                                             : ACC_PRESETS[0];
                            smooth_dead[i] = cfg.smooth_deadband_us[i];
                            SMOOTH_SetSpeed(&smooth, i, smooth_vel[i]);
                            SMOOTH_SetAccel(&smooth, i, smooth_acc[i]);
                            SMOOTH_SetDeadband(&smooth, i, smooth_dead[i]);
                        }
                        cal_dirty = false;
                        snprintf(status_line, sizeof(status_line),
                                 "Reloaded calibration from %s",
                                 cfg_path_used);
                        status_until = time(NULL) + 3;
                    }
                    else
                    {
                        snprintf(status_line, sizeof(status_line),
                                 "RELOAD FAILED: %s (%s)",
                                 CFG_StatusStr(sst), err);
                        status_until = time(NULL) + 5;
                    }
                }
                break;

            case 's':
                smooth_enabled = !smooth_enabled;
                if(smooth_enabled && hw_connected)
                {
                    /* Re-seed smoother at the current physical pose */
                    for(int i = 0; i < PCA_SERVO_COUNT; i++)
                    {
                        smooth.target[i] = servo_us[i];
                        smooth.current[i] = servo_us[i];
                        smooth.current_f[i] = (float)servo_us[i];
                    }
                }
                break;
            case 'r':
                if(hw_connected)
                {
                    /* Exercise PCA_ReadReg (previously unused) */
                    PCA_ReadReg(&pca, PCA_REG_MODE1,     &reg_mode1);
                    PCA_ReadReg(&pca, PCA_REG_MODE2,     &reg_mode2);
                    PCA_ReadReg(&pca, PCA_REG_PRE_SCALE, &reg_presc);
                    show_regs = true;
                }
                break;

            case '0':
                if(hw_connected) PCA_AllOff(&pca);
                break;

            /* Scenario keys: F1-F6 start predefined motions on selected servo */
            case KEY_F(1): case KEY_F(2): case KEY_F(3):
            case KEY_F(4): case KEY_F(5): case KEY_F(6):
            {
                int sc_num = ch - KEY_F(1);  /* 0-5 */
                if(sc_active)
                {
                    /* Abort current, start new */
                    servo_us[sc_servo] = PCA_SERVO_NEUTRAL;
                    clamp_servo(sc_servo);
                    SMOOTH_SetTarget(&smooth, sc_servo, servo_us[sc_servo]);
                    sc_active = false;
                }
                if(hw_connected)
                {
                    sc_start(sc_num, selected);
                }
                else
                {
                    snprintf(status_line, sizeof(status_line),
                             "Cannot run scenario — PCA9685 not connected");
                    status_until = time(NULL) + 3;
                }
                break;
            }

            case 27: /* Esc — abort running scenario */
                if(sc_active)
                {
                    servo_us[sc_servo] = PCA_SERVO_NEUTRAL;
                    clamp_servo(sc_servo);
                    SMOOTH_SetTarget(&smooth, sc_servo, servo_us[sc_servo]);
                    snprintf(status_line, sizeof(status_line),
                             "SCENARIO: %s ABORTED — returning to neutral",
                             SC_DEFS[sc_index].name);
                    status_until = time(NULL) + 3;
                    sc_active = false;
                }
                break;

            case '?':
            case 'h':
            case 'H':
                /* Toggle the help overlay. State is preserved underneath,
                 * so dismissing returns the user exactly where they were. */
                g_help_visible = !g_help_visible;
                break;

            case 'q':
            case 'Q':
                g_run = 0;
                break;

            default:
                break;
        }

        usleep(REFRESH_US);
    }

    /* Cleanup */
    endwin();

    if(hw_connected)
    {
        PCA_SetAllNeutral(&pca);
        PCA_Close(&pca);
    }

    printf("[TESTBENCH] Exited cleanly. Servos returned to neutral.\n");
    return 0;
}

/*==========================================================================================*/
