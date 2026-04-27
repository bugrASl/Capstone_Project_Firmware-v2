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
    mvprintw(r, 38, "(linear estimate)");
    r += 2;

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
    mvprintw(r + 2, 1, "m  set MIN    M  set MAX      N  all neutral   A  write all at once");
    mvprintw(r + 3, 1, "r  refresh registers          s  toggle smoother (now: %s)",
             smooth_enabled ? "ON" : "OFF");
    mvprintw(r + 4, 1, "0  all OFF (disable PWM)      q  quit");
    attroff(COLOR_PAIR(CP_DIM));
    r += 5;

    /* FOOTER */
    draw_hline(r, 0, g_tui_w);
    r++;
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    mvprintw(r, 1, "20 Hz refresh  |  direct I2C (no IPC/kernel needed)");
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
        "  --help              this message\n"
        "Example:\n"
        "  sudo %s --min 600,1100,1100,1000,1000,950 --max 2400,1900,1900,2000,2000,1700\n",
        prog, prog);
}

int main(int argc, char *argv[])
{
    uint16_t override_min[PCA_SERVO_COUNT] = {0};
    uint16_t override_max[PCA_SERVO_COUNT] = {0};
    bool     have_min = false, have_max = false;

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

    /* Start all servos at neutral */
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        servo_us[i] = PCA_SERVO_NEUTRAL;
        clamp_servo(i);
    }

    /* Init smoother (used only if --smooth) */
    SMOOTH_Init(&smooth, PCA_SERVO_NEUTRAL);
    /* Gripper: conservative rate, matches cpcu_io default */
    SMOOTH_SetSpeed(&smooth, 5, 1200);

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
