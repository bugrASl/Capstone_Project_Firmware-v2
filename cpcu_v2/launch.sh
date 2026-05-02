#!/bin/bash
##
##  launch.sh — InfiniTech CPCU Unified User API
##  Author:  bugrASl
##  Date:    April 2026
##  Version: v2.7
##
##  ════════════════════════════════════════════════════════════════════
##  THIS IS THE ONLY SCRIPT YOU NEED TO INVOKE. EVERY SYSTEM OPERATION
##  GOES THROUGH `./launch.sh <command>`.
##  ════════════════════════════════════════════════════════════════════
##
##  Run `./launch.sh help` for the full command reference, or
##  `./launch.sh help <command>` for command-specific detail.
##
##  Quick map:
##
##    SETUP / BUILD (once-per-Pi, then once-per-source-change):
##      ./launch.sh setup                  Configure the Pi (one-time)
##      ./launch.sh build                  Compile + install the project
##      ./launch.sh check                  Verify everything is ready
##
##    TESTING (verify subsystems before running live):
##      ./launch.sh test-sw                Software-only tests (233 PASS)
##      ./launch.sh test-ipc               + IPC validation (kernel needed)
##      ./launch.sh test-hw                + Pi hardware probes
##      ./launch.sh test-pca               Interactive servo motion check
##      ./launch.sh test-signal            Live signal-integrity TUI (needs BSAU)
##      ./launch.sh test-signal-demo       Same TUI with synthetic data
##      ./launch.sh test-safety-demo       Fault-injection demo (no hardware)
##
##    RUNTIME / COMPILE-TIME TUNING:
##      ./launch.sh configure              Interactive (compile-time)
##      ./launch.sh configure --show       Show all values
##      ./launch.sh configure --diff       Show changes from defaults
##      ./launch.sh configure --reset      Restore defaults
##      ./launch.sh configure --<name> <value>   Set one knob
##
##    OPERATING THE LIVE SYSTEM:
##      ./launch.sh tui                    Interactive: kernel + TUI dashboard
##      ./launch.sh signal                 Interactive: kernel + signal_testbench
##      ./launch.sh collect                Interactive: kernel + TUI for dataset capture
##      ./launch.sh pca                    Direct PCA9685 calibration (no kernel)
##      ./launch.sh kernel                 Kernel only, foreground (for systemd)
##      ./launch.sh ws                     Web dashboard (browser-accessible)
##      ./launch.sh menu                   Interactive picker (default on TTY)
##      ./launch.sh attach                 Re-attach to a running session
##      ./launch.sh stop                   Stop the running session
##
##    SERVICES (start at boot):
##      ./launch.sh install-service        systemd unit for the kernel
##      ./launch.sh install-ws-service     systemd unit for the web dashboard
##
##    INTERNAL:
##      ./launch.sh grant-caps             Re-apply RT capabilities to binaries
##      ./launch.sh help [<cmd>]           Help text
##      ./launch.sh version                Show version
##
##  v2.7 architectural changes:
##      - launch.sh moved from cpcu_v2/scripts/launch.sh to cpcu_v2/launch.sh.
##      - cpcu_v2/scripts/ now contains internal shell helpers only
##        (setup_pi.sh, configure.sh, run_tests.sh).
##      - cpcu_v2/python/ contains Python modules (cpcu_dsp.py, etc).
##      - launch.sh exposes ALL helper functionality. Users never invoke
##        the helpers directly.
##

set -e

# ─── Self-locate ─────────────────────────────────────────────────────
# launch.sh lives at cpcu_v2/launch.sh. Find ourselves robustly so the
# script works regardless of cwd or symlinks.
LAUNCH_SCRIPT="$(readlink -f "${BASH_SOURCE[0]}")"
CPCU_ROOT="$(dirname "${LAUNCH_SCRIPT}")"
SCRIPTS_DIR="${CPCU_ROOT}/scripts"
PYTHON_DIR="${CPCU_ROOT}/python"

# ─── Install paths (must match CMakeLists.txt) ──────────────────────
CPCU_DIR="/opt/cpcu"
BIN_DIR="${CPCU_DIR}/bin"
PYTHON_INSTALL_DIR="${CPCU_DIR}/python"
SCRIPTS_INSTALL_DIR="${CPCU_DIR}/scripts"
MODEL_DIR="${CPCU_DIR}/models"
LOG_DIR="/var/log/cpcu"

SESSION_NAME="cpcu"

# Cleanup-flag: 1 while we own a tmux session that the user hasn't yet
# attached to. Cleared after attach returns.
TMUX_OWNED=""
KERNEL_PID=""

# ─── ANSI colors ─────────────────────────────────────────────────────
if [ -t 1 ]; then
    C_RED="\033[31m";  C_GRN="\033[32m";  C_YEL="\033[33m"
    C_CYN="\033[36m";  C_BLD="\033[1m";   C_RST="\033[0m"
else
    C_RED=""; C_GRN=""; C_YEL=""; C_CYN=""; C_BLD=""; C_RST=""
fi

log()   { echo -e "${C_CYN}[LAUNCH]${C_RST} $*"; }
warn()  { echo -e "${C_YEL}[LAUNCH] WARN:${C_RST} $*"; }
err()   { echo -e "${C_RED}[LAUNCH] ERROR:${C_RST} $*" >&2; }
fatal() { err "$*"; exit 1; }
ok()    { echo -e "${C_GRN}[LAUNCH] OK:${C_RST} $*"; }

# Verify a helper script is runnable. If it exists but isn't executable
# (common right after a fresh clone — git doesn't always preserve the
# +x bit across some hosts), self-heal by chmod +x'ing it. This avoids
# the confusing "missing — incomplete source tree?" error when the file
# is actually present.
ensure_helper_executable() {
    local helper="$1"
    if [ -x "${helper}" ]; then
        return 0
    fi
    if [ -f "${helper}" ]; then
        warn "${helper} exists but isn't executable. Fixing with chmod +x..."
        if chmod +x "${helper}" 2>/dev/null; then
            ok "Fixed ${helper}. Continuing."
            return 0
        fi
        fatal "Couldn't chmod +x ${helper}. Try: chmod +x scripts/*.sh"
    fi
    fatal "Missing ${helper} — is your source tree complete?"
}

# Prompt-yes-or-no (default depends on second arg)
prompt_yn() {
    local msg="$1" default="${2:-n}" reply
    if [ "${default}" = "y" ]; then
        read -rp "  ${msg} [Y/n]: " reply
        reply="${reply:-y}"
    else
        read -rp "  ${msg} [y/N]: " reply
        reply="${reply:-n}"
    fi
    case "${reply}" in y|Y|yes|YES) return 0 ;; *) return 1 ;; esac
}


# ════════════════════════════════════════════════════════════════════════
#  PRE-FLIGHT CHECKS
# ════════════════════════════════════════════════════════════════════════

preflight_kernel() {
    log "Pre-flight..."

    mkdir -p "${LOG_DIR}" 2>/dev/null || true
    chmod 755 "${LOG_DIR}" 2>/dev/null || true

    [ -x "${BIN_DIR}/cpcu_kernel" ] \
        || fatal "Missing ${BIN_DIR}/cpcu_kernel — run './launch.sh build'"
    [ -x "${BIN_DIR}/cpcu_io" ] \
        || fatal "Missing ${BIN_DIR}/cpcu_io — run './launch.sh build'"

    if ! python3 -c "import numpy, scipy, joblib" 2>/dev/null; then
        warn "Python deps missing — DSP will run in feature-only mode."
        warn "Install with: ./launch.sh setup"
    fi

    [ -f "${PYTHON_INSTALL_DIR}/cpcu_dsp.py" ] \
        || warn "${PYTHON_INSTALL_DIR}/cpcu_dsp.py missing — re-run './launch.sh build'"

    if [ ! -f "${MODEL_DIR}/hmi_svm_model_200hz.joblib" ] \
       || [ ! -f "${MODEL_DIR}/hmi_scaler_200hz.joblib" ]; then
        warn "No ML model in ${MODEL_DIR} — DSP will run feature-only"
        warn "  Place your trained model files in that directory and"
        warn "  send 'kill -HUP \$(pgrep cpcu_kernel)' to reload."
    fi

    local isolated
    isolated=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "")
    if [ -z "${isolated}" ]; then
        warn "No CPU cores isolated — real-time guarantees void."
        warn "Run './launch.sh setup' and reboot to fix."
    else
        log "Isolated cores: ${isolated}"
    fi

    [ -e /dev/spidev0.0 ] || warn "/dev/spidev0.0 missing — run './launch.sh setup'"
    [ -e /dev/i2c-1 ]     || warn "/dev/i2c-1 missing — run './launch.sh setup'"
}

preflight_pca()    { resolve_bin pca_testbench;    }
preflight_tui()    { resolve_bin cpcu_tui;          }
preflight_signal() { resolve_bin signal_testbench;  }

# Locate a binary, preferring the installed copy under /opt/cpcu/bin/
# but falling back to ${CPCU_ROOT}/build/ for developers who haven't
# (yet) run `cmake --install`. Sets the global ${RESOLVED_BIN} on
# success; fatals on failure.
#
# Why both paths: the v2.7 layout installs testbenches to
# /opt/cpcu/bin/ via CMakeLists.txt's install() rules. Older trees
# (or trees that built without installing) leave them in build/
# only — running ./launch.sh signal there should "just work" rather
# than nag the user to re-install.
resolve_bin() {
    local name="$1"
    local installed="${BIN_DIR}/${name}"
    local local_build="${CPCU_ROOT}/build/${name}"
    if [ -x "${installed}" ]; then
        RESOLVED_BIN="${installed}"
    elif [ -x "${local_build}" ]; then
        RESOLVED_BIN="${local_build}"
        warn "Using build/${name} (not installed). Run './launch.sh build' to install."
    else
        fatal "${name} not found in ${BIN_DIR} or ${CPCU_ROOT}/build — run './launch.sh build'"
    fi
}


# ════════════════════════════════════════════════════════════════════════
#  TMUX HELPERS
# ════════════════════════════════════════════════════════════════════════

require_tmux() { command -v tmux >/dev/null 2>&1; }

tmux_kill_existing() {
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log "Killing existing tmux session '$SESSION_NAME'..."
        tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
        sleep 0.5
    fi
}

tmux_create_with_kernel() {
    tmux_kill_existing
    log "Creating tmux session '$SESSION_NAME', spawning KERNEL..."
    tmux new-session -d -s "$SESSION_NAME" -n "KERNEL" \
        "bash -c 'cd ${BIN_DIR} && exec taskset -c 0 ./cpcu_kernel --log 2>&1 | tee -a ${LOG_DIR}/cpcu.log'"

    # Wait briefly for the new session to be reachable. tmux's set-option
    # calls below otherwise race with the daemon and emit harmless but
    # noisy "no server running" stderr warnings.
    for i in 1 2 3 4 5; do
        if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
            break
        fi
        sleep 0.1
    done

    tmux set-option -t "$SESSION_NAME" remain-on-exit on        >/dev/null 2>&1
    tmux set-option -t "$SESSION_NAME" status on                >/dev/null 2>&1
    tmux set-option -t "$SESSION_NAME" status-justify centre    >/dev/null 2>&1
    tmux set-option -t "$SESSION_NAME" status-left  " #S "      >/dev/null 2>&1
    tmux set-option -t "$SESSION_NAME" status-right " %H:%M "   >/dev/null 2>&1

    TMUX_OWNED=1

    log "Waiting for shared memory..."
    for i in $(seq 1 30); do
        if [ -e /dev/shm/cpcu_ipc ]; then
            log "Shared memory ready after $((i*500))ms"
            sleep 1
            return 0
        fi
        sleep 0.5
    done

    err "Kernel didn't bring up /dev/shm/cpcu_ipc within 15s"
    err "Inspect what happened: ./launch.sh attach"
    return 1
}

tmux_add_window() {
    tmux new-window -t "$SESSION_NAME" -n "$1" "$2" 2>/dev/null
}

tmux_attach_at() {
    local target_window="$1"
    tmux select-window -t "${SESSION_NAME}:${target_window}" 2>/dev/null

    # If the session is gone (kernel crashed and ate it), bail with
    # a clear message instead of silently trying to attach to nothing.
    if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        err "tmux session '${SESSION_NAME}' is no longer running."
        err "The kernel probably crashed at startup. Check what went wrong:"
        err "  /opt/cpcu/bin/cpcu_kernel       # run it directly to see the error"
        err "  tail -50 ${LOG_DIR}/cpcu.log    # check the log"
        return 1
    fi

    echo
    log "${C_BLD}Session ready.${C_RST} You're attached to the ${C_GRN}${target_window}${C_RST} window."
    log "Keys (press ${C_BLD}Ctrl-b${C_RST} first, then the letter):"
    log "  ${C_GRN}Ctrl-b 0${C_RST}   switch to KERNEL window (kernel + io + dsp logs)"
    log "  ${C_GRN}Ctrl-b 1${C_RST}   switch to ${target_window} window"
    log "  ${C_GRN}Ctrl-b w${C_RST}   pick a window from a menu"
    log "  ${C_GRN}Ctrl-b d${C_RST}   detach (kernel keeps running in background)"
    log "  ${C_GRN}Ctrl-b ?${C_RST}   tmux help"
    log "Re-attach later: ${C_BLD}./launch.sh attach${C_RST}"
    log "Stop everything: ${C_BLD}./launch.sh stop${C_RST}"
    sleep 2

    tmux attach -t "$SESSION_NAME"
    TMUX_OWNED=""

    echo
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log "Detached. Session ${C_BLD}${SESSION_NAME}${C_RST} keeps running."
        log "  Re-attach: ${C_BLD}./launch.sh attach${C_RST}"
        log "  Stop:      ${C_BLD}./launch.sh stop${C_RST}"
    else
        log "Session ended."
    fi
}


# ════════════════════════════════════════════════════════════════════════
#  FALLBACK (NO TMUX)
# ════════════════════════════════════════════════════════════════════════

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


# ════════════════════════════════════════════════════════════════════════
#  CLEANUP TRAP
# ════════════════════════════════════════════════════════════════════════

cleanup() {
    if [ -n "${TMUX_OWNED}" ] && command -v tmux >/dev/null 2>&1 && \
       tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        warn "Cleanup: tearing down tmux session $SESSION_NAME"
        tmux kill-session -t "$SESSION_NAME" 2>/dev/null
    fi
    kernel_stop_background_fallback
}
trap cleanup EXIT INT TERM


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: setup
# ════════════════════════════════════════════════════════════════════════

cmd_setup() {
    # Self-heal: ensure launch.sh and all helpers are executable. This
    # lets the user invoke us once via `bash launch.sh setup` and never
    # need to type chmod themselves, even if their checkout came from a
    # source that strips the +x bit (zip download, FAT32, scp from
    # Windows, etc.).
    chmod +x "${LAUNCH_SCRIPT}" 2>/dev/null || true
    chmod +x "${SCRIPTS_DIR}"/*.sh 2>/dev/null || true
    ensure_helper_executable "${SCRIPTS_DIR}/setup_pi.sh"

    log "Running one-time Pi setup..."
    log "This will:"
    log "  - Install build tools and Python libraries (apt + pip)"
    log "  - Enable SPI and I²C in /boot/firmware/config.txt"
    log "  - Reserve cores 1-3 for real-time use (isolcpus)"
    log "  - Add you to the spi/i2c/gpio groups"
    log "  - Create /opt/cpcu/{bin,scripts,python,models} owned by you"
    echo
    log "You'll see one sudo password prompt."
    echo

    local rc=0
    "${SCRIPTS_DIR}/setup_pi.sh" "$@" || rc=$?

    case "${rc}" in
        0)
            ok "Setup complete. No reboot needed."
            log "Next step: ${C_BLD}./launch.sh build${C_RST}"
            ;;
        10)
            echo
            warn "════════════════════════════════════════════════════════════"
            warn "  REBOOT REQUIRED"
            warn ""
            warn "  Setup modified /boot/firmware/config.txt and/or"
            warn "  cmdline.txt. The Pi must reboot for those to take effect."
            warn "════════════════════════════════════════════════════════════"
            echo
            if prompt_yn "Reboot now?" n; then
                log "Rebooting in 3 seconds... (Ctrl-C to cancel)"
                sleep 3
                sudo reboot
            else
                warn "Reboot deferred. Run 'sudo reboot' before building."
            fi
            ;;
        *)
            fatal "setup_pi.sh exited with code ${rc}"
            ;;
    esac
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: build
# ════════════════════════════════════════════════════════════════════════

# Lazy cmake configure: only re-runs if CMakeCache.txt is missing or
# CMakeLists.txt is newer.
cmake_configure_if_needed() {
    if [ ! -f "${CPCU_ROOT}/build/CMakeCache.txt" ]; then
        log "First build — running cmake configure..."
        ( cd "${CPCU_ROOT}" && cmake -S . -B build ) || fatal "cmake configure failed"
    elif [ "${CPCU_ROOT}/CMakeLists.txt" -nt "${CPCU_ROOT}/build/CMakeCache.txt" ]; then
        log "CMakeLists.txt is newer than cache — re-running cmake configure..."
        ( cd "${CPCU_ROOT}" && cmake -S . -B build ) || fatal "cmake configure failed"
    fi
}

cmd_build() {
    [ -f "${CPCU_ROOT}/CMakeLists.txt" ] \
        || fatal "No CMakeLists.txt at ${CPCU_ROOT} — is this a CPCU source tree?"

    if [ ! -d /opt/cpcu ]; then
        fatal "/opt/cpcu doesn't exist. Run './launch.sh setup' first."
    fi

    # --clean forces a fresh build directory. Use this if you've
    # changed CMakeLists.txt in a way that requires regenerating the
    # install plan (added/removed targets), or if the cached config
    # has otherwise gone stale.
    if [ "${1:-}" = "--clean" ]; then
        log "Removing build/ directory for a clean rebuild..."
        rm -rf "${CPCU_ROOT}/build"
    fi

    cmake_configure_if_needed

    log "Building..."
    ( cd "${CPCU_ROOT}" && cmake --build build -j4 ) || fatal "build failed"

    log "Installing to /opt/cpcu..."
    ( cd "${CPCU_ROOT}" && cmake --install build ) || fatal "install failed"

    # Sanity check: did the testbenches actually land in /opt/cpcu/bin?
    # If they built but didn't install, the user's CMakeLists.txt is
    # older than the cmake cache (preserved-mtime cp, or someone
    # changed the install rules without invalidating the cache).
    # Detect and offer the recovery command.
    local missing=()
    for tb in pca_testbench signal_testbench editor_testbench; do
        if [ -x "${CPCU_ROOT}/build/${tb}" ] && [ ! -x "${BIN_DIR}/${tb}" ]; then
            missing+=("${tb}")
        fi
    done
    if [ "${#missing[@]}" -gt 0 ]; then
        warn "Built but not installed: ${missing[*]}"
        warn "Your cmake build cache is stale. Recovering with a clean rebuild..."
        rm -rf "${CPCU_ROOT}/build"
        cmake_configure_if_needed
        ( cd "${CPCU_ROOT}" && cmake --build build -j4 ) || fatal "clean rebuild failed"
        ( cd "${CPCU_ROOT}" && cmake --install build ) || fatal "clean install failed"
    fi

    log "Re-applying real-time capabilities (you'll see one sudo prompt)..."
    cmd_grant_caps_internal

    ok "Build complete. Run '${C_BLD}./launch.sh check${C_RST}' to verify, or '${C_BLD}./launch.sh tui${C_RST}' to start."
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: check
# ════════════════════════════════════════════════════════════════════════

cmd_check() {
    log "Standalone pre-flight..."
    local fatal_count=0 warn_count=0

    [ -x "${BIN_DIR}/cpcu_kernel" ] \
        || { err "MISSING ${BIN_DIR}/cpcu_kernel — run './launch.sh build'"; fatal_count=$((fatal_count+1)); }
    [ -x "${BIN_DIR}/cpcu_io" ] \
        || { err "MISSING ${BIN_DIR}/cpcu_io — run './launch.sh build'"; fatal_count=$((fatal_count+1)); }
    [ -x "${BIN_DIR}/cpcu_tui" ] \
        || { warn "${BIN_DIR}/cpcu_tui not installed"; warn_count=$((warn_count+1)); }

    if [ -x "${BIN_DIR}/cpcu_kernel" ]; then
        if ! getcap "${BIN_DIR}/cpcu_kernel" 2>/dev/null | grep -q cap_sys_nice; then
            err "cpcu_kernel missing CAP_SYS_NICE — run './launch.sh grant-caps'"
            fatal_count=$((fatal_count+1))
        fi
    fi

    if ! python3 -c "import numpy, scipy, joblib" 2>/dev/null; then
        warn "Python deps missing — DSP will run feature-only"
        warn_count=$((warn_count+1))
    fi

    if [ ! -f "${MODEL_DIR}/hmi_svm_model_200hz.joblib" ] \
       || [ ! -f "${MODEL_DIR}/hmi_scaler_200hz.joblib" ]; then
        warn "ML model not in ${MODEL_DIR} — DSP will run feature-only"
        warn_count=$((warn_count+1))
    fi

    if [ ! -f "/opt/cpcu/config.json" ]; then
        err "No /opt/cpcu/config.json — symlink missing (re-run './launch.sh setup')"
        fatal_count=$((fatal_count+1))
    fi

    local isolated
    isolated=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "")
    if [ -z "${isolated}" ]; then
        err "No cores isolated — re-run './launch.sh setup' + reboot"
        fatal_count=$((fatal_count+1))
    fi

    [ -e /dev/spidev0.0 ] || { err "/dev/spidev0.0 missing — re-run './launch.sh setup'"; fatal_count=$((fatal_count+1)); }
    [ -e /dev/i2c-1 ]     || { err "/dev/i2c-1 missing — re-run './launch.sh setup'"; fatal_count=$((fatal_count+1)); }

    if command -v i2cdetect >/dev/null 2>&1; then
        if ! i2cdetect -y 1 2>/dev/null | grep -q " 40 "; then
            warn "PCA9685 not detected at I²C 0x40 — check wiring + power"
            warn_count=$((warn_count+1))
        fi
    fi

    if ! groups | grep -qE '\bspi\b' || ! groups | grep -qE '\bi2c\b'; then
        warn "Not in spi/i2c groups — log out and back in (or re-run './launch.sh setup')"
        warn_count=$((warn_count+1))
    fi

    echo
    if [ "${fatal_count}" -gt 0 ]; then
        err "${fatal_count} fatal issue(s), ${warn_count} warning(s) — system will NOT start"
        exit 1
    elif [ "${warn_count}" -gt 0 ]; then
        warn "${warn_count} warning(s) — system can start with degraded behavior"
        exit 0
    else
        ok "All checks passed — system is ready to launch"
        exit 0
    fi
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: configure
# ════════════════════════════════════════════════════════════════════════

cmd_configure() {
    ensure_helper_executable "${SCRIPTS_DIR}/configure.sh"

    local rc=0
    "${SCRIPTS_DIR}/configure.sh" "$@" || rc=$?

    case "${rc}" in
        0) : ;;  # nothing to do, no rebuild needed
        11)
            echo
            warn "════════════════════════════════════════════════════════════"
            warn "  REBUILD REQUIRED"
            warn ""
            warn "  You changed compile-time values. The new values won't"
            warn "  take effect until the binaries are rebuilt and installed."
            warn "════════════════════════════════════════════════════════════"
            echo
            if prompt_yn "Rebuild now?" y; then
                cmd_build
            else
                warn "Rebuild deferred. Run './launch.sh build' before launching."
            fi
            ;;
        *) fatal "configure.sh exited with code ${rc}" ;;
    esac
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: test*
# ════════════════════════════════════════════════════════════════════════

cmd_test_phase() {
    local phases="$1"
    ensure_helper_executable "${SCRIPTS_DIR}/run_tests.sh"
    "${SCRIPTS_DIR}/run_tests.sh" ${phases}
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: tui / signal / collect / pca / kernel
# ════════════════════════════════════════════════════════════════════════

run_kernel_only() {
    preflight_kernel
    log "Mode: KERNEL (foreground; systemd path, no tmux)"
    log "Per-module CSVs → ${LOG_DIR}/log_*.csv"
    trap - EXIT INT TERM
    cd "${BIN_DIR}"
    exec taskset -c 0 ./cpcu_kernel --log 2>&1 | tee -a "${LOG_DIR}/cpcu.log"
}

run_tui_tmux() {
    preflight_kernel
    preflight_tui
    local tui_bin="${RESOLVED_BIN}"
    if [ "${WITH_WS:-0}" = "1" ]; then
        with_ws_preflight    || fatal "WS preflight failed"
        log "Mode: TUI + WS (tmux: KERNEL + TUI + WS)"
    else
        log "Mode: TUI (tmux: KERNEL + TUI)"
    fi
    tmux_create_with_kernel || fatal "Couldn't bring up kernel"
    tmux_add_window "TUI" "${tui_bin}"
    if [ "${WITH_WS:-0}" = "1" ]; then
        tmux_add_window "WS" "$(ws_window_cmd)"
        log "Web dashboard at http://$(hostname -I | awk '{print $1}'):8765"
    fi
    tmux_attach_at "TUI"
}

run_collect_tmux() {
    preflight_kernel
    preflight_tui
    local tui_bin="${RESOLVED_BIN}"
    if [ "${WITH_WS:-0}" = "1" ]; then
        with_ws_preflight    || fatal "WS preflight failed"
        log "Mode: COLLECT + WS (tmux: KERNEL + TUI + WS, capture-focused)"
    else
        log "Mode: COLLECT (tmux: KERNEL + TUI, capture-focused)"
    fi
    tmux_create_with_kernel || fatal "Couldn't bring up kernel"
    tmux_add_window "TUI" "${tui_bin}"
    if [ "${WITH_WS:-0}" = "1" ]; then
        tmux_add_window "WS" "$(ws_window_cmd)"
        log "Web dashboard at http://$(hostname -I | awk '{print $1}'):8765"
    fi

    echo
    log "${C_BLD}Capture workflow:${C_RST}"
    log "  1. In the TUI, press ${C_GRN}7${C_RST} to jump to the DATASET page"
    log "  2. ${C_GRN}←/→${C_RST} cycle labels (REST, H_OPN, A_BND<, ...)"
    log "  3. ${C_GRN}t${C_RST} toggle RAW ↔ FILTERED  (default FILTERED)"
    log "  4. ${C_GRN}s${C_RST} or SPACE: start, again to stop+save"
    log "  5. ${C_GRN}r${C_RST} cancel + delete partial capture"
    log "  6. ${C_GRN}q${C_RST} quit. Files land in ${BIN_DIR}/datasets/"
    echo

    tmux_attach_at "TUI"
}

run_signal_tmux() {
    preflight_kernel
    preflight_signal
    local sig_bin="${RESOLVED_BIN}"
    if [ "${WITH_WS:-0}" = "1" ]; then
        with_ws_preflight    || fatal "WS preflight failed"
        log "Mode: SIGNAL + WS (tmux: KERNEL + SIGNAL + WS)"
    else
        log "Mode: SIGNAL (tmux: KERNEL + SIGNAL)"
    fi
    tmux_create_with_kernel || fatal "Couldn't bring up kernel"
    tmux_add_window "SIGNAL" "${sig_bin}"
    if [ "${WITH_WS:-0}" = "1" ]; then
        tmux_add_window "WS" "$(ws_window_cmd)"
        log "Web dashboard at http://$(hostname -I | awk '{print $1}'):8765"
    fi
    tmux_attach_at "SIGNAL"
}

# Helpers for --with-ws composition. Both invoked from the run_*_tmux
# functions above. Kept as separate functions so they're visible at the
# top of this section and easy to find.
ws_static_dir() {
    if [ -d /opt/cpcu/ws_static ]; then
        echo "/opt/cpcu/ws_static"
    elif [ -d "${CPCU_ROOT}/web/static" ]; then
        echo "${CPCU_ROOT}/web/static"
    else
        return 1
    fi
}

# Run before adding the WS window — abort early on missing binary or
# stub builds so we don't bring up the whole tmux session only to have
# the WS window die immediately.
with_ws_preflight() {
    [ -x "${BIN_DIR}/cpcu_ws" ] \
        || { err "${BIN_DIR}/cpcu_ws not found — run './launch.sh build'"; return 1; }
    if cpcu_ws_is_stub; then
        err "cpcu_ws is a STUB build — run './launch.sh vendor' first."
        return 1
    fi
    ws_static_dir >/dev/null \
        || { err "Web static dir not found (looked at /opt/cpcu/ws_static and ${CPCU_ROOT}/web/static)"; return 1; }
    return 0
}

ws_window_cmd() {
    local sd
    sd="$(ws_static_dir)"
    echo "${BIN_DIR}/cpcu_ws --static ${sd}"
}

# Demo variant: signal_testbench --demo. Generates synthetic 100 Hz
# sines on all 8 channels internally — no kernel, no /dev/shm/cpcu_ipc,
# no BSAU needed. Useful for screenshots, verifying TUI rendering,
# and sanity-checking that the testbench builds correctly.
run_signal_demo() {
    preflight_signal
    local sig_bin="${RESOLVED_BIN}"
    log "Mode: SIGNAL DEMO (synthetic data; no kernel, no shared memory)"
    log "  Inside the TUI:  w cycle wave types  [/] change frequency  q quit"
    sleep 0.5
    trap - EXIT INT TERM
    cd "$(dirname "${sig_bin}")"
    exec "${sig_bin}" --demo
}

run_pca() {
    preflight_pca
    local pca_bin="${RESOLVED_BIN}"
    log "Mode: PCA (no kernel; direct I²C servo calibration)"
    log "Controls: arrows / m / M / n / 0 / A / q — press '?' inside for help"
    sleep 0.5
    trap - EXIT INT TERM
    cd "$(dirname "${pca_bin}")"
    # Resolve config path explicitly — the cd above breaks the relative
    # fallback ("config/runtime.json") inside pca_testbench, and the
    # /opt/cpcu/config.json symlink may be stale. CPCU_ROOT is always
    # the repo root, so this is the most reliable source.
    if [ -r "${CPCU_ROOT}/config/runtime.json" ]; then
        exec "${pca_bin}" --config "${CPCU_ROOT}/config/runtime.json"
    elif [ -r "/opt/cpcu/config.json" ]; then
        exec "${pca_bin}" --config "/opt/cpcu/config.json"
    else
        warn "No runtime.json found — pca_testbench will use compile-time defaults"
        exec "${pca_bin}"
    fi
}

# ───── Fallbacks (no tmux installed) ─────

run_tui_fallback() {
    preflight_kernel; preflight_tui
    local tui_bin="${RESOLVED_BIN}"
    log "Mode: TUI (background-mode fallback — kernel logs may overlap UI)"
    kernel_start_background_fallback || fatal "Kernel failed"
    sleep 1
    cd "$(dirname "${tui_bin}")" && "${tui_bin}"
    log "cpcu_tui exited"
}

run_collect_fallback() {
    preflight_kernel; preflight_tui
    local tui_bin="${RESOLVED_BIN}"
    log "Mode: COLLECT (background-mode fallback)"
    kernel_start_background_fallback || fatal "Kernel failed"
    sleep 1
    log "Press 7 inside the TUI to capture."
    cd "$(dirname "${tui_bin}")" && "${tui_bin}"
}

run_signal_fallback() {
    preflight_kernel; preflight_signal
    local sig_bin="${RESOLVED_BIN}"
    log "Mode: SIGNAL (background-mode fallback)"
    kernel_start_background_fallback || fatal "Kernel failed"
    sleep 1
    cd "$(dirname "${sig_bin}")" && "${sig_bin}"
}

run_tui()    { if require_tmux; then run_tui_tmux;    else warn "tmux not installed — install via './launch.sh setup'"; run_tui_fallback;    fi; }
run_collect(){ if require_tmux; then run_collect_tmux; else warn "tmux not installed — install via './launch.sh setup'"; run_collect_fallback; fi; }
run_signal() { if require_tmux; then run_signal_tmux; else warn "tmux not installed — install via './launch.sh setup'"; run_signal_fallback; fi; }


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: vendor (fetch third-party deps, currently just Mongoose)
# ════════════════════════════════════════════════════════════════════════

cmd_vendor() {
    local fetcher="${CPCU_ROOT}/web/vendor/fetch.sh"
    local mongoose_c="${CPCU_ROOT}/web/vendor/mongoose.c"
    local mongoose_h="${CPCU_ROOT}/web/vendor/mongoose.h"

    if [ ! -f "${fetcher}" ]; then
        fatal "${fetcher} not found — is this a CPCU source tree?"
    fi

    # Self-heal +x bit. Files freshly written by web upload, scp, or
    # tar without -p commonly lose the executable bit.
    if [ ! -x "${fetcher}" ]; then
        warn "${fetcher} is missing the executable bit — fixing..."
        chmod +x "${fetcher}" || fatal "couldn't chmod +x ${fetcher}"
    fi

    # Skip the network round-trip if the files are already there
    # AND the user hasn't asked for --force. Mongoose is version-pinned
    # in the script itself, so a re-fetch only matters when the script
    # gets bumped.
    if [ -f "${mongoose_c}" ] && [ -f "${mongoose_h}" ] && [ "${1:-}" != "--force" ]; then
        ok "Mongoose already vendored at web/vendor/mongoose.{c,h}."
        log "  Pass --force to re-download."
    else
        log "Fetching Mongoose source into web/vendor/..."
        ( cd "${CPCU_ROOT}/web/vendor" && bash "${fetcher}" ) \
            || fatal "fetch.sh failed — check network or web/vendor/ permissions"
        ok "Mongoose fetched."
    fi

    # CMake's WS_HAS_MONGOOSE branch is decided at configure-time, not
    # build-time. So even though the files are now in place, the
    # cached install plan still says "build cpcu_ws as a stub". Force
    # a clean reconfigure so cpcu_ws picks up the real source.
    log "Triggering a clean rebuild so cpcu_ws picks up Mongoose..."
    cmd_build --clean
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: ws (web dashboard)
# ════════════════════════════════════════════════════════════════════════

# Detects whether the installed cpcu_ws is the "stub" build (no
# Mongoose, no real HTTP server). The stub prints a recognisable line
# in its startup banner; we grep for it here. Returns 0 if it's a stub.
cpcu_ws_is_stub() {
    [ -x "${BIN_DIR}/cpcu_ws" ] || return 1
    "${BIN_DIR}/cpcu_ws" --version 2>&1 | grep -q "BUILT WITHOUT MONGOOSE" && return 0
    # Older builds that don't support --version: fall back to checking
    # whether mongoose.c was vendored at build time. Heuristic — if
    # it's not in the source tree, the binary almost certainly is a
    # stub.
    [ ! -f "${CPCU_ROOT}/web/vendor/mongoose.c" ] && return 0
    return 1
}

cmd_ws() {
    [ -x "${BIN_DIR}/cpcu_ws" ] || fatal "${BIN_DIR}/cpcu_ws not found — run './launch.sh build'"

    # If the installed cpcu_ws is a stub (built without Mongoose), it
    # won't actually serve HTTP. Catch this here instead of letting
    # the user wait, point a browser at it, and discover that the
    # banner printed "BUILT WITHOUT MONGOOSE" while nothing bound to
    # the port. Offer to fix it in one prompt.
    if cpcu_ws_is_stub; then
        warn "cpcu_ws is a STUB build — Mongoose was not vendored when this was compiled."
        warn "  As a stub it prints diagnostics but does NOT serve HTTP/WS."
        log "  Fix it with: ${C_BLD}./launch.sh vendor${C_RST}"
        log "    (downloads Mongoose, then does a clean rebuild — ~30 sec total)"
        if [ -t 0 ] && [ -t 1 ]; then
            read -rp "  Run vendor + rebuild now? [Y/n] " ans
            case "${ans}" in
                ""|y|Y|yes)
                    cmd_vendor
                    log "Continuing with the freshly-built cpcu_ws..."
                    ;;
                *)
                    err "Aborting. Run './launch.sh vendor' when you're ready."
                    exit 1
                    ;;
            esac
        else
            err "Non-interactive shell — can't prompt. Run './launch.sh vendor' manually."
            exit 1
        fi
    fi

    local static_dir
    if [ -d /opt/cpcu/ws_static ]; then
        static_dir=/opt/cpcu/ws_static
    elif [ -d "${CPCU_ROOT}/web/static" ]; then
        static_dir="${CPCU_ROOT}/web/static"
    else
        fatal "Web static dir not found (looked at /opt/cpcu/ws_static and ${CPCU_ROOT}/web/static)"
    fi

    # cpcu_ws reads from /dev/shm/cpcu_ipc — it needs the kernel to
    # have created that region. Mirror the run_tui / run_signal
    # pattern: if the kernel isn't already up, spawn it in a tmux
    # session, then put cpcu_ws in a second window. ./launch.sh stop
    # tears everything down cleanly.
    if [ ! -e /dev/shm/cpcu_ipc ]; then
        if ! require_tmux; then
            err "tmux not installed — install via './launch.sh setup',"
            err "  or start cpcu_kernel manually first then re-run './launch.sh ws'."
            exit 1
        fi
        log "Mode: WS (tmux: KERNEL + WS)"
        log "Dashboard at http://$(hostname -I | awk '{print $1}'):8765"
        log "  (or http://${HOSTNAME}.local:8765 if mDNS is enabled)"
        log "Static dir: ${static_dir}"
        log "${C_YEL}Note:${C_RST} default bind is 0.0.0.0 — anyone on your LAN can view."
        log "      Pass --bind ws://127.0.0.1:8765 to restrict to localhost."

        tmux_create_with_kernel || fatal "Couldn't bring up kernel"
        tmux_add_window "WS" "${BIN_DIR}/cpcu_ws --static \"${static_dir}\" $*"
        tmux_attach_at "WS"
        return $?
    fi

    # Shared memory already exists — kernel is running somewhere
    # (existing tmux session, systemd, or another launch.sh
    # invocation). Run cpcu_ws in the foreground attached to that
    # kernel; the user manages the kernel lifecycle separately.
    log "Starting cpcu_ws (web bridge) — Ctrl+C to stop"
    log "Dashboard at http://$(hostname -I | awk '{print $1}'):8765"
    log "  (or http://${HOSTNAME}.local:8765 if mDNS is enabled)"
    log "Static dir: ${static_dir}"
    log "${C_YEL}Note:${C_RST} default bind is 0.0.0.0 — anyone on your LAN can view."
    log "      Pass --bind ws://127.0.0.1:8765 to restrict to localhost."
    exec "${BIN_DIR}/cpcu_ws" --static "${static_dir}" "$@"
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: attach / stop
# ════════════════════════════════════════════════════════════════════════

cmd_attach() {
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log "Attaching to existing session '$SESSION_NAME'..."
        trap - EXIT INT TERM
        exec tmux attach -t "$SESSION_NAME"
    else
        err "No tmux session named '$SESSION_NAME' is running."
        err "Start one with: ./launch.sh tui  (or signal, collect)"
        exit 1
    fi
}

cmd_stop() {
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log "Killing tmux session '$SESSION_NAME'..."
        tmux kill-session -t "$SESSION_NAME"
        ok "Done."
    else
        log "No tmux session '$SESSION_NAME' running."
    fi
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: grant-caps
# ════════════════════════════════════════════════════════════════════════

cmd_grant_caps_internal() {
    if [ "$(id -u)" -ne 0 ]; then
        log "  (one sudo prompt for setcap)..."
        exec sudo "$0" grant-caps
    fi
    for B in cpcu_io cpcu_kernel; do
        if [ -x "${BIN_DIR}/${B}" ]; then
            setcap 'cap_sys_nice,cap_ipc_lock+ep' "${BIN_DIR}/${B}" \
                && log "  setcap OK: ${BIN_DIR}/${B}" \
                || warn "  setcap FAILED on ${BIN_DIR}/${B}"
        else
            warn "  ${BIN_DIR}/${B} not found — install first with './launch.sh build'"
        fi
    done
}

cmd_grant_caps() {
    log "Re-applying CAP_SYS_NICE + CAP_IPC_LOCK to installed binaries..."
    cmd_grant_caps_internal
    ok "Capabilities re-applied. They persist until the next rebuild."
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: install-service / install-ws-service
# ════════════════════════════════════════════════════════════════════════

cmd_install_service() {
    if [ "$(id -u)" -ne 0 ]; then
        log "install-service writes /etc/systemd/system/cpcu.service (one sudo prompt)..."
        exec sudo "$0" install-service
    fi

    REAL_USER="${SUDO_USER:-$(logname 2>/dev/null || echo root)}"

    cat > /etc/systemd/system/cpcu.service << SVCEOF
[Unit]
Description=CPCU Prosthetic Hand Controller
After=network.target

[Service]
Type=simple
User=${REAL_USER}
ExecStart=${CPCU_DIR}/launch.sh kernel
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

    for B in cpcu_io cpcu_kernel; do
        if [ -x "${BIN_DIR}/${B}" ]; then
            setcap 'cap_sys_nice,cap_ipc_lock+ep' "${BIN_DIR}/${B}" \
                && log "  setcap OK: ${BIN_DIR}/${B}"
        fi
    done

    systemctl daemon-reload
    systemctl enable cpcu.service
    ok "Service enabled. The kernel will auto-start at next boot."
    log "Manual control:"
    log "  sudo systemctl start cpcu"
    log "  sudo systemctl stop cpcu"
    log "  sudo systemctl status cpcu"
    log "  journalctl -u cpcu -f              (no sudo needed)"
}

cmd_install_ws_service() {
    if [ "$(id -u)" -ne 0 ]; then
        log "install-ws-service writes /etc/systemd/system/cpcu_ws.service (one sudo prompt)..."
        exec sudo "$0" install-ws-service
    fi
    REAL_USER="${SUDO_USER:-$(logname 2>/dev/null || echo root)}"

    local static_dir
    if [ -d /opt/cpcu/ws_static ]; then
        static_dir=/opt/cpcu/ws_static
    else
        static_dir="${CPCU_ROOT}/web/static"
    fi

    cat > /etc/systemd/system/cpcu_ws.service << WSEOF
[Unit]
Description=CPCU Dashboard — read-only web bridge
After=cpcu.service network.target
Requires=cpcu.service

[Service]
Type=simple
User=${REAL_USER}
ExecStart=${BIN_DIR}/cpcu_ws --bind ws://0.0.0.0:8765 --static ${static_dir}
Restart=on-failure
RestartSec=2
StandardOutput=append:${LOG_DIR}/cpcu_ws.log
StandardError=append:${LOG_DIR}/cpcu_ws.log

[Install]
WantedBy=multi-user.target
WSEOF
    log "Wrote /etc/systemd/system/cpcu_ws.service (User=${REAL_USER})"
    log "  bind:        ws://0.0.0.0:8765 (LAN-shared)"
    log "  static dir:  ${static_dir}"
    systemctl daemon-reload
    systemctl enable cpcu_ws.service
    ok "Web dashboard service enabled. Will start at next boot or:"
    log "  sudo systemctl start cpcu_ws"
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: menu
# ════════════════════════════════════════════════════════════════════════

show_menu() {
    cat <<'EOF'

  ┌──────────────────────────────────────────────────────────────────┐
  │              CPCU v2.7 — Mode Selection (tmux)                   │
  ├──────────────────────────────────────────────────────────────────┤
  │  1) kernel    Run kernel only  (foreground, no UI)               │
  │  2) tui       tmux: [KERNEL][TUI]   — main dashboard             │
  │  3) collect   tmux: [KERNEL][TUI]   — capture workflow guide     │
  │  4) signal    tmux: [KERNEL][SIGNAL] — signal-integrity testbench│
  │  5) pca       PCA9685 servo calibration only  (no kernel)        │
  │  6) ws        Web dashboard (browser-accessible)                 │
  │  q) quit                                                         │
  └──────────────────────────────────────────────────────────────────┘
EOF
    while true; do
        read -p "  Choice [1-6/q]: " choice
        case "${choice}" in
            1|kernel)  run_kernel_only;  return $? ;;
            2|tui)     run_tui;          return $? ;;
            3|collect) run_collect;      return $? ;;
            4|signal)  run_signal;       return $? ;;
            5|pca)     run_pca;          return $? ;;
            6|ws)      cmd_ws;           return $? ;;
            q|Q|quit)  log "Exiting without launching."; return 0 ;;
            *)         echo "  Invalid choice." ;;
        esac
    done
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: help
# ════════════════════════════════════════════════════════════════════════

cmd_help() {
    local topic="${1:-}"
    case "${topic}" in
        ""|main)
            sed -n '2,55p' "$0" | sed 's/^##  \?//; s/^##$//'
            ;;
        setup)
            cat <<'EOF'

./launch.sh setup
─────────────────
Configures the Raspberry Pi for CPCU. Run this once on a fresh Pi.

What it does:
  - Installs build tools (gcc, cmake, libncurses-dev, tmux, i2c-tools)
  - Installs Python libs (numpy, scipy, joblib, scikit-learn)
  - Enables SPI and I²C in /boot/firmware/config.txt
  - Reserves CPU cores 1-3 for real-time use (isolcpus boot parameter)
  - Disables Bluetooth (frees up kernel interrupts)
  - Adds you to the spi/i2c/gpio groups
  - Creates /opt/cpcu/{bin,scripts,python,models,test} owned by you
  - Symlinks /opt/cpcu/config.json to your config/runtime.json

Side effects: requires sudo (one prompt), may require reboot after.
The script is idempotent — safe to re-run.

EOF
            ;;
        build)
            cat <<'EOF'

./launch.sh build [--clean]
───────────────────────────
Compiles the C binaries and installs them to /opt/cpcu.

What it does:
  - Runs cmake configure if needed (lazy: only when CMakeLists.txt changed)
  - Compiles all binaries (cpcu_io, cpcu_kernel, cpcu_tui, testbenches)
  - Copies binaries to /opt/cpcu/bin
  - Copies Python helpers to /opt/cpcu/python
  - Copies launch.sh to /opt/cpcu/launch.sh
  - Re-applies real-time capabilities (CAP_SYS_NICE, CAP_IPC_LOCK)
  - Auto-detects "built but not installed" inconsistencies and
    self-recovers with a clean rebuild

Pass --clean to force a fresh build directory. Use this when you've
replaced CMakeLists.txt and the cmake cache has gone stale (added or
removed install targets, changed compile flags, etc.). Equivalent to
`rm -rf build && ./launch.sh build`.

Run this every time you change C source. You'll see one sudo prompt
at the end (for the capability granting step).

EOF
            ;;
        check)
            cat <<'EOF'

./launch.sh check
─────────────────
Verifies the system is ready to launch. Reports what's wrong without
trying to start anything.

Pass criteria:
  - cpcu_kernel + cpcu_io binaries exist and are installed
  - Real-time capabilities applied
  - Cores 1-3 isolated
  - SPI and I²C enabled
  - PCA9685 detected at I²C 0x40
  - Runtime config symlink exists
  - You're in the spi/i2c groups

Warning-level (system can still start):
  - ML model files missing → DSP runs in feature-only mode
  - Python libs missing → DSP degrades

Exit code: 0 if launch is OK, 1 if there's a fatal-class problem.

EOF
            ;;
        test-sw|test-ipc|test-hw|test-pca|test-signal|test-signal-demo|test-safety-demo)
            cat <<'EOF'

./launch.sh test*
─────────────────
Test commands run subsystem verifications. Each command runs its own
phase plus all earlier phases.

  ./launch.sh test-sw
    Phase 1: software-only tests. Codec, safety FSM, smoother, DSP
    pipeline, runtime config loader, TUI editor, JSON serializer.
    Expected: 233 PASS. No hardware needed.

  ./launch.sh test-ipc
    Phase 1 + Phase 2: also validates IPC bridge offsets between C
    and Python by spinning up the kernel briefly.

  ./launch.sh test-hw
    Phase 1 + 2 + 3: also probes Pi hardware (SPI, I²C, PCA9685,
    core isolation, CPU temp).

  ./launch.sh test-pca
    Interactive PCA9685 servo calibration TUI. Direct I²C, no kernel
    running. Use arrows to select servo, m/M for min/max, etc.

  ./launch.sh test-signal
    Live signal-integrity TUI. Needs cpcu_kernel running and BSAU
    transmitting. Plots raw 8-channel ADC streams. Function generator
    on PA0 is the typical input.

  ./launch.sh test-signal-demo
    Same TUI but with synthetic 100 Hz sine waves. No hardware needed.

  ./launch.sh test-safety-demo
    cpcu_tui --demo with fault-injection hotkeys (F=radio loss,
    B=battery low, G=seq-gap storm, etc.).

EOF
            ;;
        configure)
            cat <<'EOF'

./launch.sh configure [args...]
───────────────────────────────
Edit compile-time #defines in safety headers (radio timeout, battery
thresholds, thermal limits, NRF channel). After editing, prompts to
rebuild.

Sub-commands (passed through to configure.sh):
  ./launch.sh configure                        Interactive walkthrough
  ./launch.sh configure --show                 Show all current values
  ./launch.sh configure --diff                 Show only changes from defaults
  ./launch.sh configure --reset                Restore all defaults
  ./launch.sh configure --reset --runtime      Also regenerate runtime.json
  ./launch.sh configure --<name>               Show one current value
  ./launch.sh configure --<name> <value>       Set one value

Available knobs (--name):
  --radio-timeout    silence (ms) before RUNNING → DEGRADED
  --radio-safe       DEGRADED duration (ms) before SAFE
  --boot-grace       cold-start grace before radio fault arms
  --vbat-low         battery LOW threshold (V)
  --vbat-crit        battery CRITICAL threshold (V)
  --thermal-warn     thermal WARN (°C)
  --thermal-crit     thermal CRITICAL (°C)
  --i2c-max          consecutive I²C failures before SAFE
  --ring-overflow    ring overflows before fault
  --nrf-channel      BSAU NRF channel (0-125)

Note: this is for SAFETY thresholds. For RUNTIME tunables (servo
limits, gesture velocities, smoother knobs, grip levels), edit
config/runtime.json directly OR use the TUI's edit mode (press 'e'
on the CONFIG page).

EOF
            ;;
        tui|signal|collect|pca|kernel|ws|vendor|menu|attach|stop|install-service|install-ws-service|grant-caps)
            cat <<EOF

./launch.sh ${topic}
$(printf '─%.0s' $(seq 1 $((${#topic} + 13))))

EOF
            case "${topic}" in
                tui)
                    cat <<'EOF'
Bring up the kernel + the ncurses dashboard inside a tmux session.
Two windows: KERNEL (logs) and TUI (live dashboard).

Inside the TUI, the 7 pages are:
  1=Overview   2=Radio/IO   3=DSP/AI   4=Waves
  5=Health     6=Dataset    7=Config
Press 'e' on the Config page to enter live edit mode (arm parks).
Press 'q' to quit the TUI (kernel keeps running).

Detach with Ctrl-b d. Re-attach later with './launch.sh attach'.

Combine with the web dashboard:
  ./launch.sh tui --with-ws
This adds a third tmux window (WS) running cpcu_ws so others can
watch from a browser at http://<pi-ip>:8765 while you work in the
TUI. Both views share one kernel and one IPC region — no duplication.
EOF
                    ;;
                signal)
                    cat <<'EOF'
Bring up the kernel + signal-integrity testbench inside a tmux session.
Plots raw 8-channel ADC data straight off the IPC ring.

Pass criteria when a function generator drives PA0 with a 100 Hz sine,
0.6 V amplitude, 1.65 V DC offset:
  - Clean sinusoid on each channel
  - Dominant frequency ≈ 100 Hz
  - Vpp ≈ 1.2 V, SNR > 20 dB
  - Packet rate ≈ 1000/s, loss < 0.1%

Press TAB for all-channel view, q to quit.

Combine with the web dashboard:
  ./launch.sh signal --with-ws
This adds a third tmux window (WS). Useful for showing the live
signal stream to someone in a browser while you watch it locally.
EOF
                    ;;
                collect)
                    cat <<'EOF'
Like 'tui' but with on-screen reminders for the dataset capture
workflow. Use this when you're recording EMG to .csv files for
training.

Inside the TUI:
  - Press 7 to jump to the DATASET page
  - ←/→ cycle gesture labels
  - t to toggle RAW/FILTERED capture
  - s or SPACE to start, again to stop and save
  - r to cancel and delete a partial capture
  - q to quit. Files land in /opt/cpcu/bin/datasets/.

Combine with the web dashboard:
  ./launch.sh collect --with-ws
The browser dashboard mirrors the data being captured, useful when
recording with someone watching remotely.
EOF
                    ;;
                pca)
                    cat <<'EOF'
Interactive PCA9685 servo calibration. Direct I²C, NO kernel running
— use this to tune per-servo limits and bias before running the live
system.

Inside the TUI:
  - ↑/↓ select servo (S0..S5)
  - ←/→ jog ±10 µs
  - PgUp/PgDn jog ±50 µs
  - m / M jog to current MIN / MAX
  - [ set current jog as MIN
  - ] set current jog as MAX
  - b set current deviation as bias
  - B clear bias
  - s save changes to config/runtime.json
  - q quit (prompts if dirty)
EOF
                    ;;
                kernel)
                    cat <<'EOF'
Run cpcu_kernel only, in the foreground. This is the path systemd
uses. You won't see the TUI; logs go to /var/log/cpcu/cpcu.log.

For interactive use, prefer ./launch.sh tui instead.
EOF
                    ;;
                ws)
                    cat <<'EOF'
Start the web dashboard on http://<pi-ip>:8765. Read-only,
multi-viewer, browser-accessible from any device on the LAN.

Tabs in the dashboard:
  Overview    System state, packet rate, classification
  Waves       8-channel rolling raw + filtered plots
  Spectrum    FFT + waterfall on a selected channel
  Tools       NRF / PCA / DSP diagnostics

Default bind is 0.0.0.0 — anyone on your LAN can view. To restrict
to localhost only, run: ./launch.sh ws --bind ws://127.0.0.1:8765

If cpcu_ws was built without Mongoose (the embedded HTTP/WS library),
this command will detect that and offer to fix it for you in one
prompt — the alternative is './launch.sh vendor' run manually.
EOF
                    ;;
                vendor)
                    cat <<'EOF'
Fetch third-party dependencies into the source tree. Currently this
means downloading Mongoose (the embedded HTTP+WebSocket library used
by cpcu_ws) into web/vendor/, then doing a clean rebuild so cpcu_ws
links the real library instead of the no-network stub.

Why this is a separate command:
  - Mongoose is ~26k lines of vendored source. Keeping it out of the
    main source tree makes git history cleaner and lets us version-
    pin via web/vendor/fetch.sh (currently 7.14).
  - The fetch is a one-time step per checkout. Once vendored, the
    files persist until you delete them.

Options:
  --force    re-download even if web/vendor/mongoose.{c,h} exist
             (use after editing fetch.sh to bump the pinned version)

Network: needs internet access to raw.githubusercontent.com. If your
Pi is on an air-gapped network, fetch on a connected machine, copy
web/vendor/mongoose.{c,h} over, then run './launch.sh build --clean'.
EOF
                    ;;
                menu)
                    cat <<'EOF'
Show the interactive mode picker. This is the default if you run
./launch.sh with no argument from a TTY.
EOF
                    ;;
                attach)
                    cat <<'EOF'
Re-attach to a previously detached tmux session. If you ran
./launch.sh tui and pressed Ctrl-b d, the session keeps running in
the background; this brings you back to it.
EOF
                    ;;
                stop)
                    cat <<'EOF'
Kill a running tmux session and all its child processes (kernel, io,
dsp, TUI). Use this when you're done for the day.
EOF
                    ;;
                install-service)
                    cat <<'EOF'
Install a systemd unit so cpcu_kernel starts automatically at boot.
Writes /etc/systemd/system/cpcu.service (one sudo prompt).

After installing:
  sudo systemctl start cpcu      # start now
  sudo systemctl stop cpcu       # stop
  sudo systemctl status cpcu     # check
  journalctl -u cpcu -f          # live log tail
EOF
                    ;;
                install-ws-service)
                    cat <<'EOF'
Install a systemd unit for the web dashboard. Depends on cpcu.service.
Writes /etc/systemd/system/cpcu_ws.service (one sudo prompt).
EOF
                    ;;
                grant-caps)
                    cat <<'EOF'
Re-apply CAP_SYS_NICE (real-time scheduling) and CAP_IPC_LOCK (page
locking) capabilities to cpcu_io and cpcu_kernel. Capabilities are
stored on the binary's inode and lost when a rebuild creates a new
file, so you need to re-run this after every rebuild.

Note: ./launch.sh build runs grant-caps automatically. You only need
to run it manually if something went wrong.
EOF
                    ;;
            esac
            ;;
        *)
            err "No help topic '${topic}'."
            log "Try: ./launch.sh help        (top-level)"
            log "  or: ./launch.sh help setup"
            exit 1
            ;;
    esac
}

cmd_version() { echo "InfiniTech CPCU launch.sh v2.7 (April 2026)"; }


# ════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ════════════════════════════════════════════════════════════════════════

MODE="${1:-}"
shift || true

# Composable flag: --with-ws (or --ws) appended to a kernel-aware
# command makes the tmux session also include a WS window. Examples:
#   ./launch.sh tui --with-ws
#   ./launch.sh signal --with-ws
#   ./launch.sh collect --with-ws
# We strip the flag here from $@ so the remaining args pass cleanly
# through to the underlying command.
WITH_WS=0
NEW_ARGS=()
for arg in "$@"; do
    case "${arg}" in
        --with-ws|--ws) WITH_WS=1 ;;
        *)              NEW_ARGS+=("${arg}") ;;
    esac
done
set -- "${NEW_ARGS[@]+"${NEW_ARGS[@]}"}"

case "${MODE}" in
    -h|--help|help)         cmd_help "$@" ;;
    -v|--version|version)   cmd_version ;;

    setup)                  cmd_setup "$@" ;;
    build)                  cmd_build "$@" ;;
    vendor)                 cmd_vendor "$@" ;;
    check)                  cmd_check ;;
    configure)              cmd_configure "$@" ;;

    test-sw)                cmd_test_phase "1" ;;
    test-ipc)               cmd_test_phase "1 2" ;;
    test-hw)                cmd_test_phase "1 2 3" ;;
    # The interactive testbench dispatches go through launch.sh's own
    # kernel-aware helpers (which spawn cpcu_kernel inside a tmux
    # session as needed), not through run_tests.sh — `test-pca`,
    # `test-signal`, and `test-signal-demo` all become aliases for
    # the equivalent operating-mode commands so users get a usable
    # session out of the box without having to start the kernel
    # manually first.
    test-pca)               run_pca ;;
    test-signal)            run_signal ;;
    test-signal-demo)       run_signal_demo ;;
    test-safety-demo)       cmd_test_phase "safety-demo" ;;

    kernel|"")
        if [ -z "${MODE}" ] && [ -t 0 ] && [ -t 1 ]; then
            show_menu
        else
            run_kernel_only
        fi
        ;;
    tui)                    run_tui ;;
    collect)                run_collect ;;
    signal)                 run_signal ;;
    pca)                    run_pca ;;
    menu)                   show_menu ;;
    ws)                     cmd_ws "$@" ;;

    attach)                 cmd_attach ;;
    stop)                   cmd_stop ;;

    grant-caps)             cmd_grant_caps ;;
    install-service)        cmd_install_service ;;
    install-ws-service)     cmd_install_ws_service ;;

    *)
        err "Unknown command: ${MODE}"
        echo
        echo "Run './launch.sh help' for the full command reference."
        echo "Common commands: setup, build, check, test-sw, tui, ws, stop, help"
        exit 2
        ;;
esac
