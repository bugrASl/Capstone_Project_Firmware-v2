#!/bin/bash
##
##  launch.sh — CPCU v2.6 Boot Script (tmux multi-mode launcher)
##  Author: bugrASl
##  Date:   April 2026
##
##  USAGE — RUN AS REGULAR USER (no sudo at the prompt):
##      ./scripts/launch.sh              # menu (TTY) / kernel (systemd)
##      ./scripts/launch.sh tui          # tmux: kernel + cpcu_tui
##      ./scripts/launch.sh signal       # tmux: kernel + signal_testbench
##      ./scripts/launch.sh collect      # tmux: kernel + cpcu_tui (capture-mode reminder)
##      ./scripts/launch.sh pca          # pca_testbench only (no kernel, no tmux)
##      ./scripts/launch.sh kernel       # kernel only (foreground)
##      ./scripts/launch.sh build        # cmake configure (lazy) + build + install + grant-caps
##      ./scripts/launch.sh test         # run Phase 1 software tests (no hardware)
##      ./scripts/launch.sh check        # pre-flight: report environment problems and exit
##      ./scripts/launch.sh attach       # re-attach to running tmux session
##      ./scripts/launch.sh stop         # kill the tmux session and all children
##      ./scripts/launch.sh install-service  # set up systemd unit + enable
##
##  Pre-condition: ./setup_pi.sh has been run once. That script adds
##  you to the spi/i2c/gpio groups, sets up udev rules, and chowns
##  /opt/cpcu and /var/log/cpcu to your user — so the binaries below
##  run without sudo. SCHED_FIFO / mlockall need CAP_SYS_NICE +
##  CAP_IPC_LOCK; the install step grants those via setcap on the
##  binaries (see grant_caps below).
##
##  v2.6 changes (2026-04):
##      - Added `build` mode wrapping the dev rebuild loop:
##        cmake configure (lazy) + cmake --build + cmake --install +
##        grant-caps. One command for the whole rebuild cycle.
##      - Added `test` mode as a thin wrapper for `./run_tests.sh 1`.
##      - Added `check` mode: standalone pre-flight that classifies
##        environment problems as fatal or warning, exits 0/1
##        accordingly. Useful in CI and as a "before I demo" sanity.
##
##      Operations intentionally NOT wrapped:
##        * `apt update` / `apt upgrade` — wrong scope (OS-level),
##          wrong privilege model (sudo every launch), wrong frequency
##          (weekly vs many launches/day), and apt upgrade can replace
##          the running kernel mid-session. Stays a manual user op.
##        * `setup_pi.sh` — one-time provisioning that modifies
##          /boot/firmware and requires reboot. Different scope from
##          launch.sh; stays in setup_pi.sh.
##
##      The split: setup_pi.sh provisions the Pi, launch.sh
##      builds + runs, run_tests.sh validates. One coherent scope each.
##
##  v2.5 changes (2026-04):
##      - All sudo invocations removed from the user-facing path. The
##        only sudo prompts the user sees come from internal re-exec
##        when this script is invoked with `install-service` or
##        `grant-caps` — both of which need root for one specific
##        operation and self-elevate.
##      - Added `install-service` mode that copies the systemd unit,
##        runs setcap on the installed binaries, enables and starts
##        the service.
##      - Re-attach / kill commands now run as user (tmux session is
##        owned by the user, not root).
##
##  v2.4 changes (2026-04):
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
##          ./scripts/launch.sh attach   re-attach
##          ./scripts/launch.sh stop     stop everything
##
##      If tmux isn't installed, falls back to v2.3 background-mode
##      behavior with a warning. Install with:
##          ./setup_pi.sh                (re-run; tmux is now installed by default)
##
##  Modes:
##      kernel          Kernel only (foreground; systemd path, no tmux).
##      tui             tmux: [KERNEL][TUI]
##      signal          tmux: [KERNEL][SIGNAL]
##      collect         tmux: [KERNEL][TUI] with capture-workflow reminder.
##      pca             pca_testbench only (no kernel, no tmux).
##      menu            Interactive picker (default for TTY launch).
##      install-service Install systemd unit + setcap; self-elevates.
##      grant-caps      Re-apply CAP_SYS_NICE + CAP_IPC_LOCK to the binaries.
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


# ──────────────────────────────────────────────────────────────────────
#  BUILD / TEST / CHECK helpers (v2.6)
# ──────────────────────────────────────────────────────────────────────

# Locate the repo root (contains CMakeLists.txt + scripts/). Resolves
# whether the script was invoked from the install path
# (/opt/cpcu/scripts/launch.sh) or directly from the repo
# (cpcu_v2/scripts/launch.sh).
find_repo_root() {
    local me d fallback
    me="$(readlink -f "$0")"
    d="$(dirname "$(dirname "${me}")")"
    if [ -f "${d}/CMakeLists.txt" ]; then echo "${d}"; return 0; fi
    fallback="${HOME}/prosthetic_hand/cpcu_v2"
    if [ -f "${fallback}/CMakeLists.txt" ]; then echo "${fallback}"; return 0; fi
    return 1
}

# Lazy cmake configure: only re-runs when CMakeCache.txt is missing
# or older than CMakeLists.txt. Saves ~5 s on every build call.
run_cmake_configure() {
    local repo="$1"
    if [ ! -f "${repo}/build/CMakeCache.txt" ]; then
        log "First build — running cmake configure..."
        ( cd "${repo}" && cmake -S . -B build ) || fatal "cmake configure failed"
    elif [ "${repo}/CMakeLists.txt" -nt "${repo}/build/CMakeCache.txt" ]; then
        log "CMakeLists.txt is newer than cache — re-running cmake configure..."
        ( cd "${repo}" && cmake -S . -B build ) || fatal "cmake configure failed"
    else
        log "cmake cache up to date — skipping configure"
    fi
}

# Wrap the full dev rebuild loop: configure (lazy) + build + install
# + grant-caps. Only the grant-caps step needs sudo; the rest runs as
# user thanks to setup_pi.sh's chown of /opt/cpcu.
run_build() {
    local repo
    repo="$(find_repo_root)" \
        || fatal "couldn't find repo root — checked \$0 and ~/prosthetic_hand/cpcu_v2"
    log "Repo root: ${repo}"

    run_cmake_configure "${repo}"

    log "Building..."
    ( cd "${repo}" && cmake --build build -j4 ) || fatal "build failed"

    log "Installing to /opt/cpcu..."
    ( cd "${repo}" && cmake --install build ) || fatal "install failed"

    log "Re-applying RT capabilities..."
    "$0" grant-caps || fatal "grant-caps failed"

    log "${C_GRN}${C_BLD}Build complete.${C_RST} Run '${C_BLD}$0 tui${C_RST}' to start the system."
}

# Phase 1 software tests (no hardware needed).
run_phase1_tests() {
    local repo
    repo="$(find_repo_root)" || fatal "couldn't find repo root"
    [ -x "${repo}/run_tests.sh" ] || fatal "${repo}/run_tests.sh not found or not executable"

    log "Running Phase 1 software tests..."
    ( cd "${repo}" && ./run_tests.sh 1 )
}

# Standalone pre-flight: report what's wrong, exit 1 if a fatal-class
# issue would prevent launch, exit 0 otherwise. Warn-class issues
# (missing model, PCA not detected) are reported but don't fail.
run_check() {
    log "Standalone pre-flight check..."
    local fatal_count=0 warn_count=0

    [ -x "${BIN_DIR}/cpcu_kernel" ] \
        || { err "MISSING ${BIN_DIR}/cpcu_kernel — run '$0 build'"; fatal_count=$((fatal_count+1)); }
    [ -x "${BIN_DIR}/cpcu_io" ] \
        || { err "MISSING ${BIN_DIR}/cpcu_io — run '$0 build'";     fatal_count=$((fatal_count+1)); }
    [ -x "${BIN_DIR}/cpcu_tui" ] \
        || { warn "${BIN_DIR}/cpcu_tui not installed";              warn_count=$((warn_count+1)); }

    # Capabilities — fatal if binaries exist but aren't capped.
    if [ -x "${BIN_DIR}/cpcu_kernel" ]; then
        if ! getcap "${BIN_DIR}/cpcu_kernel" 2>/dev/null | grep -q cap_sys_nice; then
            err "cpcu_kernel missing CAP_SYS_NICE — run '$0 grant-caps'"
            fatal_count=$((fatal_count+1))
        fi
    fi

    # Python deps
    if ! python3 -c "import numpy, scipy, joblib" 2>/dev/null; then
        warn "Python deps missing — DSP will be feature-only"
        warn_count=$((warn_count+1))
    fi

    # Model files — warn only; cpcu_dsp.py supports feature-only mode.
    if [ ! -f "${MODEL_DIR}/hmi_svm_model_200hz.joblib" ] \
       || [ ! -f "${MODEL_DIR}/hmi_scaler_200hz.joblib" ]; then
        warn "ML model not in ${MODEL_DIR} — DSP will run feature-only"
        warn_count=$((warn_count+1))
    fi

    # Runtime config symlink
    if [ ! -f "/opt/cpcu/config.json" ]; then
        err "No /opt/cpcu/config.json — symlink missing (re-run setup_pi.sh)"
        fatal_count=$((fatal_count+1))
    fi

    # isolcpus
    local isolated
    isolated=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "")
    if [ -z "${isolated}" ]; then
        err "No cores isolated — RT guarantees void (add isolcpus to cmdline.txt + reboot)"
        fatal_count=$((fatal_count+1))
    fi

    # Peripherals
    [ -e /dev/spidev0.0 ] \
        || { err "/dev/spidev0.0 missing — enable SPI in raspi-config"; fatal_count=$((fatal_count+1)); }
    [ -e /dev/i2c-1 ] \
        || { err "/dev/i2c-1 missing — enable I2C in raspi-config";     fatal_count=$((fatal_count+1)); }

    # PCA detection — warn only; bench testing without PCA is OK.
    if command -v i2cdetect >/dev/null 2>&1; then
        if ! i2cdetect -y 1 2>/dev/null | grep -q " 40 "; then
            warn "PCA9685 not detected at I²C 0x40"
            warn_count=$((warn_count+1))
        fi
    fi

    # Group membership
    if ! groups | grep -qE '\bspi\b' || ! groups | grep -qE '\bi2c\b'; then
        warn "Not in spi/i2c groups — log out and back in"
        warn_count=$((warn_count+1))
    fi

    echo
    if [ "${fatal_count}" -gt 0 ]; then
        err "${fatal_count} fatal issue(s), ${warn_count} warning(s) — system will NOT start"
        exit 1
    elif [ "${warn_count}" -gt 0 ]; then
        log "${C_YEL}${warn_count} warning(s) — system can start with degraded behavior${C_RST}"
        exit 0
    else
        log "${C_GRN}All checks passed — system is ready to launch${C_RST}"
        exit 0
    fi
}


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
    err "Inspect what happened: ./scripts/launch.sh attach"
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
    log "Re-attach later:  ${C_BLD}./scripts/launch.sh attach${C_RST}"
    log "Stop everything:  ${C_BLD}./scripts/launch.sh stop${C_RST}"
    sleep 2

    tmux attach -t "$SESSION_NAME"

    # Returned from attach — user either detached or killed the session.
    # User now owns the session lifetime; we don't.
    TMUX_OWNED=""

    echo
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log "Detached. Session ${C_BLD}${SESSION_NAME}${C_RST} still running in background."
        log "  Re-attach:  ${C_BLD}./scripts/launch.sh attach${C_RST}"
        log "  Stop:       ${C_BLD}./scripts/launch.sh stop${C_RST}"
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
        warn "tmux not installed — install with: ./setup_pi.sh   (tmux is now installed by default)"
        warn "Falling back to background mode (kernel logs may overlap UI)"
        run_tui_fallback
    fi
}

run_collect() {
    if require_tmux; then run_collect_tmux
    else
        warn "tmux not installed — install with: ./setup_pi.sh   (tmux is now installed by default)"
        run_collect_fallback
    fi
}

run_signal() {
    if require_tmux; then run_signal_tmux
    else
        warn "tmux not installed — install with: ./setup_pi.sh   (tmux is now installed by default)"
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
    build)      run_build ;;
    test)       run_phase1_tests ;;
    check)      run_check ;;
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
    grant-caps)
        # Re-apply CAP_SYS_NICE (SCHED_FIFO) + CAP_IPC_LOCK (mlockall) to
        # the installed binaries. Self-elevates.
        if [ "$(id -u)" -ne 0 ]; then
            log "grant-caps needs root (one-shot setcap on installed binaries)"
            log "Re-execing under sudo (you'll be prompted for your password)..."
            exec sudo "$0" "$@"
        fi
        for B in cpcu_io cpcu_kernel; do
            if [ -x "${BIN_DIR}/${B}" ]; then
                setcap 'cap_sys_nice,cap_ipc_lock+ep' "${BIN_DIR}/${B}" \
                    && log "  setcap OK: ${BIN_DIR}/${B}" \
                    || warn "  setcap FAILED on ${BIN_DIR}/${B}"
            else
                warn "  ${BIN_DIR}/${B} not found — install first"
            fi
        done
        ;;
    install-service)
        # Generate /etc/systemd/system/cpcu.service, run setcap, enable and
        # start. Self-elevates because writing to /etc/systemd needs root.
        if [ "$(id -u)" -ne 0 ]; then
            log "install-service needs root (writes /etc/systemd/system/cpcu.service)"
            log "Re-execing under sudo (you'll be prompted for your password)..."
            exec sudo "$0" "$@"
        fi

        REAL_USER="${SUDO_USER:-$(logname 2>/dev/null || echo root)}"

        cat > /etc/systemd/system/cpcu.service << SVCEOF
[Unit]
Description=CPCU Prosthetic Hand Controller
After=network.target

[Service]
Type=simple
User=${REAL_USER}
ExecStart=${SCRIPT_DIR}/launch.sh kernel
Restart=on-failure
RestartSec=2
StandardOutput=append:${LOG_DIR}/cpcu.log
StandardError=append:${LOG_DIR}/cpcu.log

# RT scheduling needs nice + mlockall
AmbientCapabilities=CAP_SYS_NICE CAP_IPC_LOCK
LimitMEMLOCK=infinity
LimitRTPRIO=99

[Install]
WantedBy=multi-user.target
SVCEOF
        log "Wrote /etc/systemd/system/cpcu.service (User=${REAL_USER})"

        # Apply setcap so the binaries can also be invoked manually
        for B in cpcu_io cpcu_kernel; do
            if [ -x "${BIN_DIR}/${B}" ]; then
                setcap 'cap_sys_nice,cap_ipc_lock+ep' "${BIN_DIR}/${B}" \
                    && log "  setcap OK: ${BIN_DIR}/${B}"
            fi
        done

        systemctl daemon-reload
        systemctl enable cpcu.service
        log "Service enabled. Start with:"
        log "  sudo systemctl start cpcu          (or just reboot)"
        log "Stop / status:"
        log "  sudo systemctl stop cpcu"
        log "  sudo systemctl status cpcu"
        log "  journalctl -u cpcu -f              (no sudo needed)"
        ;;
    ws)
        # v2.4.0: launch the read-only web bridge in the foreground.
        # Reads /dev/shm/cpcu_ipc and serves the CPCU Dashboard at
        # http://<pi-ip>:8765. Defaults to 0.0.0.0 (LAN-shared); pass
        # --bind ws://127.0.0.1:8765 in argv if you want loopback only.
        [ -x "${BIN_DIR}/cpcu_ws" ] || fatal "${BIN_DIR}/cpcu_ws not found"
        log "Starting cpcu_ws (web bridge) — Ctrl+C to stop"
        log "Dashboard at http://$(hostname -I | awk '{print $1}'):8765"
        exec "${BIN_DIR}/cpcu_ws" --static "${SCRIPT_DIR}/../web/static" "$@"
        ;;
    install-ws-service)
        # v2.4.0: generate /etc/systemd/system/cpcu_ws.service so the
        # bridge starts on boot alongside cpcu.service. Self-elevates.
        if [ "$(id -u)" -ne 0 ]; then
            log "install-ws-service needs root (writes /etc/systemd/system/cpcu_ws.service)"
            log "Re-execing under sudo (you'll be prompted for your password)..."
            exec sudo "$0" "$@"
        fi
        REAL_USER="${SUDO_USER:-$(logname 2>/dev/null || echo root)}"
        # Try the deployed-static-dir first; fall back to scripts/../web/static.
        if [ -d /opt/cpcu/ws_static ]; then
            STATIC_DIR=/opt/cpcu/ws_static
        else
            STATIC_DIR="${SCRIPT_DIR}/../web/static"
        fi
        cat > /etc/systemd/system/cpcu_ws.service << WSEOF
[Unit]
Description=CPCU Dashboard — read-only web bridge
After=cpcu.service network.target
Requires=cpcu.service

[Service]
Type=simple
User=${REAL_USER}
ExecStart=${BIN_DIR}/cpcu_ws --bind ws://0.0.0.0:8765 --static ${STATIC_DIR}
Restart=on-failure
RestartSec=2
StandardOutput=append:${LOG_DIR}/cpcu_ws.log
StandardError=append:${LOG_DIR}/cpcu_ws.log

[Install]
WantedBy=multi-user.target
WSEOF
        log "Wrote /etc/systemd/system/cpcu_ws.service (User=${REAL_USER})"
        log "  bind: ws://0.0.0.0:8765 (LAN-shared)"
        log "  static dir: ${STATIC_DIR}"
        systemctl daemon-reload
        systemctl enable cpcu_ws.service
        log "Service enabled. Start with:"
        log "  sudo systemctl start cpcu_ws         (or just reboot)"
        log "Stop / status:"
        log "  sudo systemctl stop cpcu_ws"
        log "  sudo systemctl status cpcu_ws"
        log "  journalctl -u cpcu_ws -f"
        ;;
    *)
        err "Unknown mode: ${MODE}"
        echo "Usage: $0 [kernel|tui|collect|signal|pca|ws|menu|attach|stop|install-service|install-ws-service|grant-caps]"
        exit 2
        ;;
esac
