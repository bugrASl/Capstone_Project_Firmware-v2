#!/bin/bash
##
##  launch.sh — CPCU v2.3 Boot Script (multi-mode launcher)
##  Author: bugrASl
##  Date:   April 2026
##
##  v2.3 (2026-04):
##      Adds mode selection. Started by systemd in default `kernel` mode
##      (production), or interactively for development workflows.
##
##  Modes:
##      kernel     Kernel + cpcu_io + cpcu_dsp only (foreground).
##                 Used by systemd. No interactive UI.
##      tui        Kernel + cpcu_dsp + cpcu_tui dashboard. General-purpose;
##                 use Page 7 for capture, Page 4 for waveforms.
##      signal     Kernel + signal_testbench. Focused signal-integrity TUI.
##      collect    Same as tui, but prints a reminder to press '7'.
##      pca        pca_testbench only (no kernel). Servo calibration.
##      menu       Show interactive menu (default when launched
##                 from a terminal with no argument).
##
##  Usage:
##      sudo /opt/cpcu/scripts/launch.sh              # menu (TTY) / kernel (systemd)
##      sudo /opt/cpcu/scripts/launch.sh tui
##      sudo /opt/cpcu/scripts/launch.sh signal
##      sudo /opt/cpcu/scripts/launch.sh collect
##      sudo /opt/cpcu/scripts/launch.sh pca
##      sudo /opt/cpcu/scripts/launch.sh kernel       # explicit
##
##  Install: sudo cp launch.sh /opt/cpcu/scripts/
##           sudo chmod +x /opt/cpcu/scripts/launch.sh
##

set -e

CPCU_DIR="/opt/cpcu"
BIN_DIR="${CPCU_DIR}/bin"
SCRIPT_DIR="${CPCU_DIR}/scripts"
MODEL_DIR="${CPCU_DIR}/models"
LOG_DIR="/var/log/cpcu"

# Used by background-mode cleanup
KERNEL_PID=""

# ANSI colors (best-effort; degrades to no color over plain serial)
if [ -t 1 ]; then
    C_RED="\033[31m";   C_GRN="\033[32m";   C_YEL="\033[33m"
    C_CYN="\033[36m";   C_BLD="\033[1m";    C_RST="\033[0m"
else
    C_RED=""; C_GRN=""; C_YEL=""; C_CYN=""; C_BLD=""; C_RST=""
fi

log()   { echo -e "${C_CYN}[LAUNCH]${C_RST} $*"; }
warn()  { echo -e "${C_YEL}[LAUNCH] WARN:${C_RST} $*"; }
err()   { echo -e "${C_RED}[LAUNCH] ERROR:${C_RST} $*" >&2; }
fatal() { err "$*"; exit 1; }


# ══════════════════════════════════════════════════════════════════════
#  PRE-FLIGHT CHECKS  (run for any mode that touches the kernel)
# ══════════════════════════════════════════════════════════════════════

preflight_kernel() {
    log "CPCU v2.3 pre-flight checks..."

    mkdir -p "${LOG_DIR}"
    chmod 755 "${LOG_DIR}"

    [ -x "${BIN_DIR}/cpcu_kernel" ] || fatal "${BIN_DIR}/cpcu_kernel not found"
    [ -x "${BIN_DIR}/cpcu_io" ]     || fatal "${BIN_DIR}/cpcu_io not found"

    if ! python3 -c "import numpy, scipy, joblib" 2>/dev/null; then
        warn "Python deps missing (numpy/scipy/joblib). DSP features may degrade."
        warn "Install: sudo apt install python3-numpy python3-scipy && pip3 install joblib --break-system-packages"
    fi

    [ -f "${SCRIPT_DIR}/cpcu_dsp.py" ] || warn "${SCRIPT_DIR}/cpcu_dsp.py missing — kernel will run IO-only"

    if [ ! -f "${MODEL_DIR}/hmi_svm_model_200hz.joblib" ] && \
       [ ! -f "${MODEL_DIR}/emg_rf_model.pkl" ]; then
        warn "No ML model in ${MODEL_DIR} — DSP will run in feature-only mode"
    fi

    ISOLATED=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "")
    if [ -z "${ISOLATED}" ]; then
        warn "No cores isolated — RT guarantees void. Add to /boot/firmware/cmdline.txt:"
        warn "  isolcpus=1,2,3 nohz_full=1,2,3 rcu_nocbs=1,2,3"
    else
        log "Isolated cores: ${ISOLATED}"
    fi

    [ -e /dev/spidev0.0 ] || warn "/dev/spidev0.0 missing — enable SPI in raspi-config"
    [ -e /dev/i2c-1 ]     || warn "/dev/i2c-1 missing — enable I2C in raspi-config"
}

preflight_pca() {
    [ -x "${BIN_DIR}/pca_testbench" ] || fatal "${BIN_DIR}/pca_testbench not found"
    [ -e /dev/i2c-1 ]                 || warn "/dev/i2c-1 missing — pca_testbench will run dry"
}

preflight_tui() {
    [ -x "${BIN_DIR}/cpcu_tui" ] || fatal "${BIN_DIR}/cpcu_tui not found"
}

preflight_signal() {
    [ -x "${BIN_DIR}/signal_testbench" ] || fatal "${BIN_DIR}/signal_testbench not found"
}


# ══════════════════════════════════════════════════════════════════════
#  KERNEL LIFECYCLE  (background spawn + cleanup)
# ══════════════════════════════════════════════════════════════════════

kernel_start_background() {
    log "Starting cpcu_kernel in background (logs → ${LOG_DIR}/cpcu.log)..."
    cd "${BIN_DIR}"
    # Background it, but tee output to both the log file and the main terminal
    # so the user sees IO heartbeats and DSP messages alongside their tool.
    ( taskset -c 0 ./cpcu_kernel --log 2>&1 | tee -a "${LOG_DIR}/cpcu.log" ) &
    KERNEL_PID=$!
    log "cpcu_kernel pid=${KERNEL_PID}"

    # Wait up to 15s for /dev/shm/cpcu_ipc to appear
    log "Waiting for shared memory..."
    for i in $(seq 1 30); do
        if [ -e /dev/shm/cpcu_ipc ]; then
            log "Shared memory ready after $((i*500))ms"
            sleep 1   # give cpcu_io a moment to set io_ready
            return 0
        fi
        sleep 0.5
    done
    err "Timed out waiting for /dev/shm/cpcu_ipc"
    return 1
}

kernel_stop_background() {
    if [ -n "${KERNEL_PID}" ] && kill -0 "${KERNEL_PID}" 2>/dev/null; then
        log "Stopping cpcu_kernel (pid=${KERNEL_PID})..."
        # SIGTERM goes to the process group so kernel + io + dsp all see it
        kill -TERM "-${KERNEL_PID}" 2>/dev/null || kill -TERM "${KERNEL_PID}" 2>/dev/null || true
        # Give it 5s to clean up servos + flush logs + release shm
        for i in $(seq 1 10); do
            kill -0 "${KERNEL_PID}" 2>/dev/null || break
            sleep 0.5
        done
        if kill -0 "${KERNEL_PID}" 2>/dev/null; then
            warn "cpcu_kernel didn't exit cleanly, sending SIGKILL"
            kill -KILL "${KERNEL_PID}" 2>/dev/null || true
        fi
        wait "${KERNEL_PID}" 2>/dev/null || true
        KERNEL_PID=""
    fi
}

# Make sure the kernel always gets cleaned up if THIS script exits, no
# matter how (foreground tool quit, Ctrl+C, error, signal).
trap kernel_stop_background EXIT INT TERM


# ══════════════════════════════════════════════════════════════════════
#  MODE IMPLEMENTATIONS
# ══════════════════════════════════════════════════════════════════════

run_kernel_only() {
    preflight_kernel
    log "Mode: KERNEL (foreground; systemd-style)"
    log "cpcu_kernel will spawn cpcu_io and cpcu_dsp internally"
    log "Per-module CSVs → ${LOG_DIR}/log_*.csv"

    # Disable the EXIT trap because exec replaces this shell — the trap
    # would never fire, and even if it did, there's no background PID to
    # kill. Keeps the script's contract identical to v2.2 in this mode.
    trap - EXIT INT TERM

    cd "${BIN_DIR}"
    exec taskset -c 0 ./cpcu_kernel --log 2>&1 | tee -a "${LOG_DIR}/cpcu.log"
}

run_tui() {
    preflight_kernel
    preflight_tui
    log "Mode: TUI (kernel + dashboard)"

    kernel_start_background || fatal "Kernel failed to come up"

    # Give the user a moment to see the IO ready line
    sleep 1
    log "Launching cpcu_tui — press 'q' to quit (kernel will be cleaned up)"
    sleep 0.5

    cd "${BIN_DIR}"
    "${BIN_DIR}/cpcu_tui"
    rc=$?
    log "cpcu_tui exited (rc=${rc})"
    return ${rc}
}

run_collect() {
    preflight_kernel
    preflight_tui
    log "Mode: COLLECT (kernel + dashboard, capture-focused)"

    kernel_start_background || fatal "Kernel failed to come up"

    sleep 1
    echo
    log "${C_BLD}Capture workflow:${C_RST}"
    log "  1. Press ${C_GRN}7${C_RST} to jump to the DATASET page"
    log "  2. ${C_GRN}←/→${C_RST} to cycle labels (REST, H_OPN, A_BND<, ...)"
    log "  3. ${C_GRN}t${C_RST} to toggle RAW ↔ FILTERED output (default FILTERED)"
    log "  4. ${C_GRN}s${C_RST} or SPACE to start, ${C_GRN}s${C_RST} again to stop+save"
    log "  5. ${C_GRN}r${C_RST} cancels and deletes a partial capture"
    log "  6. ${C_GRN}q${C_RST} when done. Files land in ./datasets/ relative to cwd:"
    log "     $(pwd)/datasets/"
    echo
    sleep 1.5

    cd "${BIN_DIR}"
    "${BIN_DIR}/cpcu_tui"
    rc=$?
    log "cpcu_tui exited (rc=${rc})"
    return ${rc}
}

run_signal() {
    preflight_kernel
    preflight_signal
    log "Mode: SIGNAL (kernel + signal_testbench)"

    kernel_start_background || fatal "Kernel failed to come up"

    sleep 1
    log "Launching signal_testbench — UP/DOWN to select channel, TAB to toggle view, q to quit"
    sleep 0.5

    cd "${BIN_DIR}"
    "${BIN_DIR}/signal_testbench"
    rc=$?
    log "signal_testbench exited (rc=${rc})"
    return ${rc}
}

run_pca() {
    preflight_pca
    log "Mode: PCA (no kernel; direct I2C servo calibration)"
    log "Controls: arrows, m/M=min/max, n=neutral, 0=kill, A=write all, q=quit"
    sleep 0.5

    # PCA testbench needs sudo for I2C, but doesn't touch shm or RT cores.
    # No kernel cleanup needed since we never started one.
    trap - EXIT INT TERM

    cd "${BIN_DIR}"
    exec "${BIN_DIR}/pca_testbench"
}


# ══════════════════════════════════════════════════════════════════════
#  INTERACTIVE MENU
# ══════════════════════════════════════════════════════════════════════

show_menu() {
    cat <<'EOF'

  ┌──────────────────────────────────────────────────────────────────┐
  │                  CPCU v2.3 — Mode Selection                      │
  ├──────────────────────────────────────────────────────────────────┤
  │  1) kernel    Run kernel only  (foreground, no UI)               │
  │  2) tui       Kernel + dashboard  (1-7 pages, all features)      │
  │  3) collect   Kernel + dashboard, jumps to capture instructions  │
  │  4) signal    Kernel + signal-integrity testbench  (Goertzel,    │
  │               per-channel waveform + frequency analysis)         │
  │  5) pca       PCA9685 servo calibration only  (no kernel)        │
  │  q) quit      Don't launch anything                              │
  └──────────────────────────────────────────────────────────────────┘
EOF
    while true; do
        read -p "  Choice [1-5/q]: " choice
        case "${choice}" in
            1|kernel)  run_kernel_only;  return $? ;;
            2|tui)     run_tui;          return $? ;;
            3|collect) run_collect;      return $? ;;
            4|signal)  run_signal;       return $? ;;
            5|pca)     run_pca;          return $? ;;
            q|Q|quit)  log "Exiting without launching."; return 0 ;;
            *)         echo "  Invalid choice." ;;
        esac
    done
}


# ══════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════

show_help() {
    sed -n '2,30p' "$0"
    exit 0
}

MODE="${1:-}"

case "${MODE}" in
    -h|--help|help)
        show_help
        ;;
    kernel|"")
        # Empty arg: if STDIN is a TTY, show menu; else default to kernel
        # (this is what systemd hits — no TTY → kernel mode, identical to v2.2)
        if [ -z "${MODE}" ] && [ -t 0 ] && [ -t 1 ]; then
            show_menu
        else
            run_kernel_only
        fi
        ;;
    tui)        run_tui ;;
    collect)    run_collect ;;
    signal)     run_signal ;;
    pca)        run_pca ;;
    menu)       show_menu ;;
    *)
        err "Unknown mode: ${MODE}"
        echo "Usage: $0 [kernel|tui|collect|signal|pca|menu]"
        exit 2
        ;;
esac
