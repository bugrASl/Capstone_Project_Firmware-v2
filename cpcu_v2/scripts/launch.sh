#!/bin/bash
##
##  launch.sh — CPCU v2.4 Boot Script (tmux multi-mode launcher)
##  Author: bugrASl
##  Date:   April 2026
##
##  v2.4 (2026-04):
##      Multi-window tmux dispatch. Each role runs in its own named
##      window — KERNEL, TUI, SIGNAL — so kernel logs no longer overwrite
##      the ncurses display. After bringup, the script attaches the user
##      to the session at the role's window.
##
##      Keybindings inside tmux (default prefix is Ctrl-b):
##          Ctrl-b 0 / 1 / 2   switch to window N
##          Ctrl-b w           interactive window picker
##          Ctrl-b d           detach (session keeps running, processes alive)
##          Ctrl-b ?           tmux help
##
##      After detaching:
##          sudo tmux attach -t cpcu                re-attach
##          sudo tmux kill-session -t cpcu          stop everything
##
##      If tmux isn't installed, falls back to v2.3 background-mode
##      behavior with a warning. Install with:
##          sudo apt install tmux
##
##  Modes:
##      kernel     Kernel only (foreground; systemd path, no tmux).
##      tui        tmux: [KERNEL][TUI]
##      signal     tmux: [KERNEL][SIGNAL]
##      collect    tmux: [KERNEL][TUI] with capture-workflow reminder.
##      pca        pca_testbench only (no kernel, no tmux).
##      menu       Interactive picker (default for TTY launch).
##
##  Usage:
##      sudo /opt/cpcu/scripts/launch.sh              # menu (TTY) / kernel (systemd)
##      sudo /opt/cpcu/scripts/launch.sh tui
##      sudo /opt/cpcu/scripts/launch.sh signal
##      sudo /opt/cpcu/scripts/launch.sh collect
##      sudo /opt/cpcu/scripts/launch.sh pca
##      sudo /opt/cpcu/scripts/launch.sh kernel
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

SESSION_NAME="cpcu"

# Cleanup-flag: 1 while we own a tmux session that the user hasn't yet
# attached to. Cleared after attach returns (user has agency at that point).
TMUX_OWNED=""
KERNEL_PID=""           # populated only in fallback (no-tmux) path

# ANSI colours
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
#  PRE-FLIGHT CHECKS
# ══════════════════════════════════════════════════════════════════════

preflight_kernel() {
    log "CPCU v2.4 pre-flight..."

    mkdir -p "${LOG_DIR}"
    chmod 755 "${LOG_DIR}"

    [ -x "${BIN_DIR}/cpcu_kernel" ] || fatal "${BIN_DIR}/cpcu_kernel not found"
    [ -x "${BIN_DIR}/cpcu_io" ]     || fatal "${BIN_DIR}/cpcu_io not found"

    if ! python3 -c "import numpy, scipy, joblib" 2>/dev/null; then
        warn "Python deps missing (numpy/scipy/joblib). DSP may degrade."
    fi

    [ -f "${SCRIPT_DIR}/cpcu_dsp.py" ] || warn "${SCRIPT_DIR}/cpcu_dsp.py missing"

    if [ ! -f "${MODEL_DIR}/hmi_svm_model_200hz.joblib" ] && \
       [ ! -f "${MODEL_DIR}/emg_rf_model.pkl" ]; then
        warn "No ML model in ${MODEL_DIR} — DSP feature-only mode"
    fi

    ISOLATED=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "")
    if [ -z "${ISOLATED}" ]; then
        warn "No cores isolated — RT guarantees void."
        warn "Add 'isolcpus=1,2,3 nohz_full=1,2,3 rcu_nocbs=1,2,3' to /boot/firmware/cmdline.txt"
    else
        log "Isolated cores: ${ISOLATED}"
    fi

    [ -e /dev/spidev0.0 ] || warn "/dev/spidev0.0 missing — enable SPI"
    [ -e /dev/i2c-1 ]     || warn "/dev/i2c-1 missing — enable I2C"
}

preflight_pca()    { [ -x "${BIN_DIR}/pca_testbench" ]    || fatal "${BIN_DIR}/pca_testbench not found"; }
preflight_tui()    { [ -x "${BIN_DIR}/cpcu_tui" ]         || fatal "${BIN_DIR}/cpcu_tui not found"; }
preflight_signal() { [ -x "${BIN_DIR}/signal_testbench" ] || fatal "${BIN_DIR}/signal_testbench not found"; }


# ══════════════════════════════════════════════════════════════════════
#  TMUX HELPERS
# ══════════════════════════════════════════════════════════════════════

require_tmux() {
    command -v tmux >/dev/null 2>&1
}

tmux_kill_existing() {
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log "Killing existing tmux session '$SESSION_NAME'..."
        tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
        sleep 0.5
    fi
}

# Create the session with the KERNEL window. Subsequent windows are
# added by the mode-specific functions.
tmux_create_with_kernel() {
    tmux_kill_existing

    log "Creating tmux session '$SESSION_NAME', spawning KERNEL..."
    tmux new-session -d -s "$SESSION_NAME" -n "KERNEL" \
        "bash -c 'cd ${BIN_DIR} && exec taskset -c 0 ./cpcu_kernel --log 2>&1 | tee -a ${LOG_DIR}/cpcu.log'"

    # Keep windows visible after their command exits, with [Process
    # exited] in the title. Useful for catching kernel crashes.
    tmux set-option -t "$SESSION_NAME" remain-on-exit on >/dev/null

    # Give the user a status line showing all windows
    tmux set-option -t "$SESSION_NAME" status on            >/dev/null
    tmux set-option -t "$SESSION_NAME" status-justify centre >/dev/null
    tmux set-option -t "$SESSION_NAME" status-left  " #S " >/dev/null
    tmux set-option -t "$SESSION_NAME" status-right " %H:%M " >/dev/null

    TMUX_OWNED=1

    # Wait for shm
    log "Waiting for shared memory..."
    for i in $(seq 1 30); do
        if [ -e /dev/shm/cpcu_ipc ]; then
            log "Shared memory ready after $((i*500))ms"
            sleep 1     # io_ready takes a beat after shm appears
            return 0
        fi
        sleep 0.5
    done

    err "Kernel didn't bring up /dev/shm/cpcu_ipc within 15s"
    err "Inspect what happened: sudo tmux attach -t $SESSION_NAME"
    return 1
}

tmux_add_window() {
    local name="$1"
    local cmd="$2"
    tmux new-window -t "$SESSION_NAME" -n "$name" "$cmd"
}

tmux_attach_at() {
    local target_window="$1"

    tmux select-window -t "${SESSION_NAME}:${target_window}"

    echo
    log "${C_BLD}tmux session ready.${C_RST} Active window: ${C_GRN}${target_window}${C_RST}"
    log "  ${C_GRN}Ctrl-b 0${C_RST}    KERNEL window  (kernel + io + dsp logs)"
    log "  ${C_GRN}Ctrl-b 1${C_RST}    ${target_window} window"
    log "  ${C_GRN}Ctrl-b w${C_RST}    interactive window picker"
    log "  ${C_GRN}Ctrl-b d${C_RST}    detach (everything keeps running)"
    log "  ${C_GRN}Ctrl-b &${C_RST}    close current window (with confirm)"
    log "Re-attach later:  ${C_BLD}sudo tmux attach -t ${SESSION_NAME}${C_RST}"
    log "Stop everything:  ${C_BLD}sudo tmux kill-session -t ${SESSION_NAME}${C_RST}"
    sleep 2

    tmux attach -t "$SESSION_NAME"

    # Returned from attach — user either detached or killed the session.
    # User now owns the session lifetime; we don't.
    TMUX_OWNED=""

    echo
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log "Detached. Session ${C_BLD}${SESSION_NAME}${C_RST} still running in background."
        log "  Re-attach:  ${C_BLD}sudo tmux attach -t ${SESSION_NAME}${C_RST}"
        log "  Stop:       ${C_BLD}sudo tmux kill-session -t ${SESSION_NAME}${C_RST}"
    else
        log "Session ended."
    fi
}


# ══════════════════════════════════════════════════════════════════════
#  FALLBACK (NO TMUX)
# ══════════════════════════════════════════════════════════════════════

kernel_start_background_fallback() {
    log "Starting cpcu_kernel in background (logs → ${LOG_DIR}/cpcu.log)..."
    cd "${BIN_DIR}"
    ( taskset -c 0 ./cpcu_kernel --log 2>&1 | tee -a "${LOG_DIR}/cpcu.log" ) &
    KERNEL_PID=$!

    log "Waiting for shared memory..."
    for i in $(seq 1 30); do
        [ -e /dev/shm/cpcu_ipc ] && { sleep 1; return 0; }
        sleep 0.5
    done
    err "Timed out waiting for /dev/shm/cpcu_ipc"
    return 1
}

kernel_stop_background_fallback() {
    [ -z "${KERNEL_PID}" ] && return
    kill -0 "${KERNEL_PID}" 2>/dev/null || return

    log "Stopping cpcu_kernel (pid=${KERNEL_PID})..."
    kill -TERM "-${KERNEL_PID}" 2>/dev/null || kill -TERM "${KERNEL_PID}" 2>/dev/null || true
    for i in $(seq 1 10); do
        kill -0 "${KERNEL_PID}" 2>/dev/null || break
        sleep 0.5
    done
    if kill -0 "${KERNEL_PID}" 2>/dev/null; then
        warn "Kernel didn't exit cleanly, sending SIGKILL"
        kill -KILL "${KERNEL_PID}" 2>/dev/null || true
    fi
    wait "${KERNEL_PID}" 2>/dev/null || true
    KERNEL_PID=""
}


# ══════════════════════════════════════════════════════════════════════
#  CLEANUP TRAP
# ══════════════════════════════════════════════════════════════════════

cleanup() {
    # If user Ctrl-C'd before tmux_attach completed, tear down the
    # session we created.
    if [ -n "${TMUX_OWNED}" ] && command -v tmux >/dev/null 2>&1 && \
       tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        warn "Cleanup: tearing down tmux session $SESSION_NAME"
        tmux kill-session -t "$SESSION_NAME" 2>/dev/null
    fi

    # Same for fallback path
    kernel_stop_background_fallback
}
trap cleanup EXIT INT TERM


# ══════════════════════════════════════════════════════════════════════
#  MODE IMPLEMENTATIONS
# ══════════════════════════════════════════════════════════════════════

run_kernel_only() {
    preflight_kernel
    log "Mode: KERNEL (foreground; systemd path, no tmux)"
    log "Per-module CSVs → ${LOG_DIR}/log_*.csv"

    # exec replaces this shell; trap won't fire. There's no tmux
    # session and no background pid, so nothing to clean up anyway.
    trap - EXIT INT TERM

    cd "${BIN_DIR}"
    exec taskset -c 0 ./cpcu_kernel --log 2>&1 | tee -a "${LOG_DIR}/cpcu.log"
}

run_tui_tmux() {
    preflight_kernel
    preflight_tui
    log "Mode: TUI (tmux: KERNEL + TUI)"

    tmux_create_with_kernel || fatal "Couldn't bring up kernel"
    tmux_add_window "TUI" "${BIN_DIR}/cpcu_tui"
    tmux_attach_at "TUI"
}

run_collect_tmux() {
    preflight_kernel
    preflight_tui
    log "Mode: COLLECT (tmux: KERNEL + TUI, capture-focused)"

    tmux_create_with_kernel || fatal "Couldn't bring up kernel"
    tmux_add_window "TUI" "${BIN_DIR}/cpcu_tui"

    echo
    log "${C_BLD}Capture workflow:${C_RST}"
    log "  1. Inside the TUI window, press ${C_GRN}7${C_RST} to jump to DATASET page"
    log "  2. ${C_GRN}←/→${C_RST} cycle labels (REST, H_OPN, A_BND<, ...)"
    log "  3. ${C_GRN}t${C_RST} toggle RAW ↔ FILTERED  (default FILTERED)"
    log "  4. ${C_GRN}s${C_RST} or SPACE start, again to stop+save"
    log "  5. ${C_GRN}r${C_RST} cancel + delete partial capture"
    log "  6. ${C_GRN}q${C_RST} quit. Files land in ./datasets/ relative to cwd:"
    log "     ${BIN_DIR}/datasets/"
    echo

    tmux_attach_at "TUI"
}

run_signal_tmux() {
    preflight_kernel
    preflight_signal
    log "Mode: SIGNAL (tmux: KERNEL + SIGNAL)"

    tmux_create_with_kernel || fatal "Couldn't bring up kernel"
    tmux_add_window "SIGNAL" "${BIN_DIR}/signal_testbench"
    tmux_attach_at "SIGNAL"
}

run_pca() {
    preflight_pca
    log "Mode: PCA (no kernel; direct I2C servo calibration)"
    log "Controls: arrows, m/M=min/max, n=neutral, 0=kill, A=write all, q=quit"
    sleep 0.5

    trap - EXIT INT TERM
    cd "${BIN_DIR}"
    exec "${BIN_DIR}/pca_testbench"
}

# ───── Fallbacks (no tmux installed) ─────

run_tui_fallback() {
    preflight_kernel
    preflight_tui
    log "Mode: TUI (background-mode fallback — kernel logs may overlap UI)"
    kernel_start_background_fallback || fatal "Kernel failed"
    sleep 1
    cd "${BIN_DIR}"
    "${BIN_DIR}/cpcu_tui"
    log "cpcu_tui exited"
}

run_collect_fallback() {
    preflight_kernel
    preflight_tui
    log "Mode: COLLECT (background-mode fallback)"
    kernel_start_background_fallback || fatal "Kernel failed"
    sleep 1
    log "Press 7 inside the TUI to capture."
    cd "${BIN_DIR}"
    "${BIN_DIR}/cpcu_tui"
}

run_signal_fallback() {
    preflight_kernel
    preflight_signal
    log "Mode: SIGNAL (background-mode fallback)"
    kernel_start_background_fallback || fatal "Kernel failed"
    sleep 1
    cd "${BIN_DIR}"
    "${BIN_DIR}/signal_testbench"
}

# ───── Wrappers that pick tmux vs fallback ─────

run_tui() {
    if require_tmux; then run_tui_tmux
    else
        warn "tmux not installed — install with: sudo apt install tmux"
        warn "Falling back to background mode (kernel logs may overlap UI)"
        run_tui_fallback
    fi
}

run_collect() {
    if require_tmux; then run_collect_tmux
    else
        warn "tmux not installed — install with: sudo apt install tmux"
        run_collect_fallback
    fi
}

run_signal() {
    if require_tmux; then run_signal_tmux
    else
        warn "tmux not installed — install with: sudo apt install tmux"
        run_signal_fallback
    fi
}


# ══════════════════════════════════════════════════════════════════════
#  INTERACTIVE MENU
# ══════════════════════════════════════════════════════════════════════

show_menu() {
    cat <<'EOF'

  ┌──────────────────────────────────────────────────────────────────┐
  │              CPCU v2.4 — Mode Selection (tmux)                   │
  ├──────────────────────────────────────────────────────────────────┤
  │  1) kernel    Run kernel only  (foreground, no UI)               │
  │  2) tui       tmux: [KERNEL][TUI]   — main dashboard             │
  │  3) collect   tmux: [KERNEL][TUI]   — capture workflow guide     │
  │  4) signal    tmux: [KERNEL][SIGNAL] — signal-integrity testbench│
  │  5) pca       PCA9685 servo calibration only  (no kernel)        │
  │  q) quit                                                         │
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

show_help() { sed -n '2,50p' "$0"; exit 0; }

MODE="${1:-}"

case "${MODE}" in
    -h|--help|help)
        show_help
        ;;
    kernel|"")
        # Empty arg: TTY → menu, no TTY (= systemd) → kernel mode
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
    attach)
        # Convenience: re-attach to running session
        if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
            log "Attaching to existing session '$SESSION_NAME'..."
            trap - EXIT INT TERM
            exec tmux attach -t "$SESSION_NAME"
        else
            err "No tmux session named '$SESSION_NAME' is running."
            err "Start one with: $0 tui  (or signal / collect)"
            exit 1
        fi
        ;;
    stop)
        # Convenience: kill running session
        if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
            log "Killing tmux session '$SESSION_NAME'..."
            tmux kill-session -t "$SESSION_NAME"
            log "Done."
        else
            log "No tmux session '$SESSION_NAME' running."
        fi
        ;;
    *)
        err "Unknown mode: ${MODE}"
        echo "Usage: $0 [kernel|tui|collect|signal|pca|menu|attach|stop]"
        exit 2
        ;;
esac
