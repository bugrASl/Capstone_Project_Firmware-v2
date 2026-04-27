/**
 *  @file       pca_testbench.c
 *  @brief      Standalone PCA9685 servo testbench with ncurses TUI
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    1.1
 *
 *  @details    Interactive servo calibration and testing tool. Talks
 *              directly to the PCA9685 over I2C — no kernel or IPC needed.
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

static void draw_screen(void)
{
    erase();
    int r = 0;

    /* HEADER */
    attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
    mvprintw(r, 0, "%-*s", g_tui_w, "  PCA9685 SERVO TESTBENCH - InfiniTech v1.1");
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

    /* KEYBINDINGS */
    draw_hline(r - 1, 0, g_tui_w);
    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, 1,     "UP/DOWN   select servo       LEFT/RIGHT  +/- %d us", STEP_FINE);
    mvprintw(r + 1, 1, "PgUp/PgDn +/- %d us          n  neutral (1500)", STEP_COARSE);
    mvprintw(r + 2, 1, "m  jog to MIN  M  jog to MAX  N  all neutral   A  write all at once");
    mvprintw(r + 3, 1, "[  set MIN     ]  set MAX     b  set BIAS  B  clear BIAS");
    mvprintw(r + 4, 1, "v  cycle VEL   a  cycle ACC   d  cycle DEAD     S  save  L  reload");
    mvprintw(r + 5, 1, "r  refresh registers          s  toggle smoother (now: %s)",
             smooth_enabled ? "ON" : "OFF");
    mvprintw(r + 6, 1, "0  all OFF (disable PWM)      q  quit");
    attroff(COLOR_PAIR(CP_DIM));
    r += 7;

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
        "  S          save min/max/bias to the loaded runtime.json\n"
        "  L          reload from disk (discards unsaved jogs)\n"
        "Example workflow:\n"
        "  sudo %s --config config/runtime.json\n"
        "  # jog with arrows, press [ at the mechanical min, ] at the max,\n"
        "  # then S to save. Then on the live system: kill -HUP $(pgrep cpcu_kernel)\n",
        prog, prog);
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
                if(servo_us[selected] >= pca.servo_min[selected] + STEP_FINE)
                    servo_us[selected] -= STEP_FINE;
                else
                    servo_us[selected] = pca.servo_min[selected];
                write_servo(selected);
                break;
            case KEY_RIGHT:
                if(servo_us[selected] <= pca.servo_max[selected] - STEP_FINE)
                    servo_us[selected] += STEP_FINE;
                else
                    servo_us[selected] = pca.servo_max[selected];
                write_servo(selected);
                break;

            case KEY_PPAGE:
                if(servo_us[selected] <= pca.servo_max[selected] - STEP_COARSE)
                    servo_us[selected] += STEP_COARSE;
                else
                    servo_us[selected] = pca.servo_max[selected];
                write_servo(selected);
                break;
            case KEY_NPAGE:
                if(servo_us[selected] >= pca.servo_min[selected] + STEP_COARSE)
                    servo_us[selected] -= STEP_COARSE;
                else
                    servo_us[selected] = pca.servo_min[selected];
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
             * within ~50 ms. Save with 'S' to persist to JSON. */
            case 'v':
                {
                    int idx = find_preset_idx(smooth_vel[selected],
                                              VEL_PRESETS, VEL_PRESET_COUNT);
                    int next = (idx < 0) ? 0 : ((idx + 1) % VEL_PRESET_COUNT);
                    smooth_vel[selected] = VEL_PRESETS[next];
                    SMOOTH_SetSpeed(&smooth, selected, smooth_vel[selected]);
                    cal_dirty = true;
                    snprintf(status_line, sizeof(status_line),
                             "Servo %d VEL = %u us/s (unsaved)",
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
                    cal_dirty = true;
                    snprintf(status_line, sizeof(status_line),
                             "Servo %d ACC = %u us/s^2 (unsaved)",
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
                    cal_dirty = true;
                    snprintf(status_line, sizeof(status_line),
                             "Servo %d DEAD = %u us (unsaved)",
                             selected, smooth_dead[selected]);
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
