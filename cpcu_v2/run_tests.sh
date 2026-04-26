#!/bin/bash
##
##  run_tests.sh — CPCU v2.3 Test Runner
##  Author: bugrASl
##  Date:   April 2026
##
##  Runs all test phases in dependency order.
##  Phase 1 needs no hardware. Phase 2 needs shared memory. Phase 3 needs Pi hardware.
##
##  Usage — RUN AS REGULAR USER (no sudo at the prompt):
##      ./run_tests.sh              # Run all automated phases (1 2 3)
##      ./run_tests.sh 1            # Run Phase 1 only (codec + safety + DSP)
##      ./run_tests.sh 1 2          # Run Phases 1 and 2
##      ./run_tests.sh pca          # Launch interactive PCA9685 servo testbench
##      ./run_tests.sh signal       # Launch interactive signal integrity testbench
##      ./run_tests.sh signal-demo  # Launch signal testbench with synthetic data (no hw)
##      ./run_tests.sh safety-demo  # Launch cpcu_tui --demo with fault-injection hotkeys
##
##  v2.3 changes:
##      - All sudo invocations removed. The PCA / signal testbench
##        previously did `exec sudo ./binary`; now they just `exec
##        ./binary`. setup_pi.sh adds you to the spi+i2c+gpio groups
##        so the device files are reachable without root, and shm_open
##        uses 0666 so /dev/shm/cpcu_ipc is also reachable.
##      - Build location auto-detected: tries `./binary` first (legacy),
##        then `build/binary` (modern out-of-tree cmake), then bails
##        with a clear error if neither exists.
##

set -e

GREEN="\033[32m"
RED="\033[31m"
YELLOW="\033[33m"
CYAN="\033[36m"
RESET="\033[0m"

PASS=0
FAIL=0

run_test() {
    local name="$1"
    local cmd="$2"
    
    echo -e "\n${CYAN}━━━ ${name} ━━━${RESET}"
    
    if eval "${cmd}"; then
        echo -e "${GREEN}[PASS]${RESET} ${name}"
        PASS=$((PASS + 1))
    else
        echo -e "${RED}[FAIL]${RESET} ${name}"
        FAIL=$((FAIL + 1))
    fi
}

PHASES="${@:-1 2 3}"

##============= INTERACTIVE: PCA9685 Servo Testbench =======================================

if echo "$PHASES" | grep -qw "pca"; then
    echo -e "\n${YELLOW}══════════════════════════════════════════${RESET}"
    echo -e "${YELLOW}  PCA9685 Interactive Servo Testbench${RESET}"
    echo -e "${YELLOW}══════════════════════════════════════════${RESET}"

    ## Find binary
    TB_BIN=""
    if [ -f ./pca_testbench ]; then
        TB_BIN="./pca_testbench"
    elif [ -f build/pca_testbench ]; then
        TB_BIN="build/pca_testbench"
    fi

    if [ -z "${TB_BIN}" ]; then
        echo -e "${RED}[ERROR]${RESET} pca_testbench not found."
        echo "  Build it first:"
        echo "    cd build && cmake --build . --target pca_testbench"
        echo "  Or standalone:"
        echo "    gcc -Iinc -o pca_testbench test/pca_testbench.c src/cpcu_pca9685.c -lncurses -lm"
        exit 1
    fi

    ## Check I2C
    if [ ! -e /dev/i2c-1 ]; then
        echo -e "${YELLOW}[WARN]${RESET} /dev/i2c-1 not found — testbench will run in dry-run mode"
    fi

    ## Check PCA9685
    if command -v i2cdetect &>/dev/null; then
        if ! i2cdetect -y 1 2>/dev/null | grep -q " 40 "; then
            echo -e "${YELLOW}[WARN]${RESET} PCA9685 not detected at 0x40 — testbench will run in dry-run mode"
        fi
    fi

    echo -e "\n${CYAN}Launching testbench... (press 'q' inside TUI to quit)${RESET}\n"
    ## No sudo here: setup_pi.sh added you to the i2c group, so /dev/i2c-1
    ## is readable. If you skipped setup_pi.sh, the testbench falls back
    ## to dry-run mode automatically.
    exec ${TB_BIN} /dev/i2c-1
fi

##============= INTERACTIVE: Signal Integrity Testbench ====================================

if echo "$PHASES" | grep -qw "signal"; then
    echo -e "\n${YELLOW}══════════════════════════════════════════${RESET}"
    echo -e "${YELLOW}  Signal Integrity Testbench${RESET}"
    echo -e "${YELLOW}══════════════════════════════════════════${RESET}"

    ## Find binary
    SIG_BIN=""
    if [ -f ./signal_testbench ]; then
        SIG_BIN="./signal_testbench"
    elif [ -f build/signal_testbench ]; then
        SIG_BIN="build/signal_testbench"
    fi

    if [ -z "${SIG_BIN}" ]; then
        echo -e "${RED}[ERROR]${RESET} signal_testbench not found."
        echo "  Build it first:"
        echo "    cd build && cmake --build . --target signal_testbench"
        echo "  Or standalone:"
        echo "    gcc -Iinc -o signal_testbench test/signal_testbench.c src/cpcu_ipc.c \\"
        echo "        src/wireless_packet.c -lncurses -lrt -lm"
        exit 1
    fi

    ## Check shared memory
    if [ ! -f /dev/shm/cpcu_ipc ]; then
        echo -e "${RED}[ERROR]${RESET} Shared memory not found."
        echo "  Start cpcu_kernel first:"
        echo "      ./scripts/launch.sh kernel"
        echo "  (cpcu_io must also be running to receive NRF packets)"
        exit 1
    fi

    echo -e "\n${CYAN}Test Setup:${RESET}"
    echo "  1. Connect sine wave (func gen) to all 8 EMG inputs on BSAU"
    echo "  2. Connect 3.3V to VBAT divider input"
    echo "  3. cpcu_kernel + cpcu_io must be running"
    echo ""
    echo -e "${CYAN}Launching testbench... (press 'q' inside TUI to quit)${RESET}\n"
    ## /dev/shm/cpcu_ipc is readable by the user that created it (the
    ## kernel + io processes), and shm_open uses 0666 by default — no
    ## sudo needed.
    exec ${SIG_BIN}
fi

##============= INTERACTIVE: Safety Fault Injector (no hardware) ===========================

if echo "$PHASES" | grep -qw "safety-demo"; then
    echo -e "\n${YELLOW}══════════════════════════════════════════${RESET}"
    echo -e "${YELLOW}  Safety Fault Injector [DEMO MODE]${RESET}"
    echo -e "${YELLOW}══════════════════════════════════════════${RESET}"

    TUI_BIN=""
    if [ -f ./cpcu_tui ]; then
        TUI_BIN="./cpcu_tui"
    elif [ -f build/cpcu_tui ]; then
        TUI_BIN="build/cpcu_tui"
    fi

    if [ -z "${TUI_BIN}" ]; then
        echo -e "${RED}[ERROR]${RESET} cpcu_tui not found."
        echo "  Build it first:  cd build && cmake --build . --target cpcu_tui"
        exit 1
    fi

    echo -e "\n${CYAN}Fault-injection hotkeys (inside the TUI):${RESET}"
    echo "    F  Radio freeze        → no packets → RUNNING → DEGRADED → SAFE"
    echo "    B  Battery low         → vbat_raw=1600 → SAFE (battery fault)"
    echo "    G  Sequence gap storm  → bump io_seq_gaps rapidly"
    echo "    O  Ring overflow       → io_ring_overflows > 100"
    echo "    I  I2C error streak    → PCA9685 write failure simulation"
    echo "    R  Reset               → clear all faults, return to RUNNING"
    echo ""
    echo "  Watch the state transitions on Page 1 (overview) and Page 2 (radio/IO)."
    echo "  Active fault is shown in red at the footer's right edge."
    echo ""
    echo -e "${CYAN}Launching... (press 'q' inside TUI to quit)${RESET}\n"
    exec ${TUI_BIN} --demo
fi

##============= INTERACTIVE: Signal Testbench DEMO (no hardware) ===========================

if echo "$PHASES" | grep -qw "signal-demo"; then
    echo -e "\n${YELLOW}══════════════════════════════════════════${RESET}"
    echo -e "${YELLOW}  Signal Integrity Testbench [DEMO MODE]${RESET}"
    echo -e "${YELLOW}══════════════════════════════════════════${RESET}"

    ## Find binary
    SIG_BIN=""
    if [ -f ./signal_testbench ]; then
        SIG_BIN="./signal_testbench"
    elif [ -f build/signal_testbench ]; then
        SIG_BIN="build/signal_testbench"
    fi

    if [ -z "${SIG_BIN}" ]; then
        echo -e "${RED}[ERROR]${RESET} signal_testbench not found."
        echo "  Build it first:"
        echo "    cd build && cmake --build . --target signal_testbench"
        exit 1
    fi

    echo "  Generating synthetic 100 Hz sine waves on all 8 channels."
    echo "  No hardware or shared memory needed."
    echo ""
    echo -e "${CYAN}Launching... (press 'q' inside TUI to quit)${RESET}\n"
    exec ${SIG_BIN} --demo
fi

##============= PHASE 1: Pure Software (no hardware, no shared memory) ====================

if echo "$PHASES" | grep -q "1"; then
    echo -e "\n${YELLOW}══════════════════════════════════════════${RESET}"
    echo -e "${YELLOW}  PHASE 1: Pure Software Tests${RESET}"
    echo -e "${YELLOW}══════════════════════════════════════════${RESET}"

    ## Find or build test_codec
    CODEC_BIN=""
    if [ -f ./test_codec ]; then
        CODEC_BIN="./test_codec"
    elif [ -f build/test_codec ]; then
        CODEC_BIN="build/test_codec"
    else
        echo "[BUILD] Compiling test_codec..."
        if [ -d build ]; then
            cd build && make test_codec && cd ..
            CODEC_BIN="build/test_codec"
        else
            gcc -O2 -Iinc -o test_codec \
                test/test_codec.c src/wireless_packet.c -lm
            CODEC_BIN="./test_codec"
        fi
    fi

    run_test "TB-C100..C106: Codec + IPC Unit Tests" \
        "${CODEC_BIN}"

    ## Find or build safety_testbench
    SAF_BIN=""
    if [ -f ./safety_testbench ]; then
        SAF_BIN="./safety_testbench"
    elif [ -f build/safety_testbench ]; then
        SAF_BIN="build/safety_testbench"
    else
        echo "[BUILD] Compiling safety_testbench..."
        if [ -d build ]; then
            cd build && make safety_testbench && cd ..
            SAF_BIN="build/safety_testbench"
        fi
    fi

    if [ -n "${SAF_BIN}" ]; then
        run_test "TB-SAF01..SAF07: Safety FSM Validation" \
            "${SAF_BIN}"
    else
        echo -e "${YELLOW}[SKIP]${RESET} safety_testbench not built"
    fi

    run_test "TB-DSP: DSP Pipeline Validation" \
        "python3 test/test_dsp_pipeline.py"
fi

##============= PHASE 2: Shared Memory (needs cpcu_kernel running) ========================

if echo "$PHASES" | grep -q "2"; then
    echo -e "\n${YELLOW}══════════════════════════════════════════${RESET}"
    echo -e "${YELLOW}  PHASE 2: IPC Bridge Tests (need cpcu_kernel)${RESET}"
    echo -e "${YELLOW}══════════════════════════════════════════${RESET}"

    ## Check if shared memory exists
    if [ ! -f /dev/shm/cpcu_ipc ]; then
        echo -e "${YELLOW}[INFO]${RESET} Shared memory not found. Starting cpcu_kernel..."
        if [ -f ./cpcu_kernel ] || [ -f build/cpcu_kernel ]; then
            KERNEL_BIN=$([ -f ./cpcu_kernel ] && echo "./cpcu_kernel" || echo "build/cpcu_kernel")
            ${KERNEL_BIN} &
            KERNEL_PID=$!
            sleep 2
            echo "[INFO] cpcu_kernel started (pid=${KERNEL_PID})"
        else
            echo -e "${RED}[SKIP]${RESET} cpcu_kernel not found. Build first."
            FAIL=$((FAIL + 1))
        fi
    fi

    if [ -f /dev/shm/cpcu_ipc ]; then
        run_test "TB-IPC: IPC Bridge Offset Validation" \
            "python3 test/test_ipc_bridge.py"
    fi

    ## Clean up kernel if we started it
    if [ -n "${KERNEL_PID}" ]; then
        kill ${KERNEL_PID} 2>/dev/null || true
        wait ${KERNEL_PID} 2>/dev/null || true
        echo "[INFO] cpcu_kernel stopped"
    fi
fi

##============= PHASE 3: Hardware Verification (needs Pi + peripherals) ===================

if echo "$PHASES" | grep -q "3"; then
    echo -e "\n${YELLOW}══════════════════════════════════════════${RESET}"
    echo -e "${YELLOW}  PHASE 3: Hardware Verification${RESET}"
    echo -e "${YELLOW}══════════════════════════════════════════${RESET}"

    ## Core isolation check
    run_test "Core Isolation" \
        '[ "$(cat /sys/devices/system/cpu/isolated 2>/dev/null)" = "1-3" ]'

    ## SPI device check
    run_test "SPI0 Available" \
        "[ -e /dev/spidev0.0 ]"

    ## I2C device check
    run_test "I2C-1 Available" \
        "[ -e /dev/i2c-1 ]"

    ## PCA9685 detection
    run_test "PCA9685 Detected at 0x40" \
        'i2cdetect -y 1 2>/dev/null | grep -q " 40 "'

    ## PCA9685 register read-back (verify prescaler is writable)
    run_test "PCA9685 Prescaler Read-back" \
        'i2cget -y 1 0x40 0xFE 2>/dev/null | grep -qv "Error"'

    ## Build pca_testbench if not already built
    if [ -f ./pca_testbench ] || [ -f build/pca_testbench ]; then
        TB_BIN=$([ -f ./pca_testbench ] && echo "./pca_testbench" || echo "build/pca_testbench")

        ## Automated PCA init smoke test (starts testbench, waits 2s, sends 'q')
        ## This validates I2C open, prescaler write, MODE1 verify, neutral set
        run_test "PCA9685 Init + Neutral (pca_testbench dry run)" \
            'echo "q" | timeout 3 '"${TB_BIN}"' /dev/i2c-1 2>&1 | grep -q "TESTBENCH"'
    else
        echo -e "${YELLOW}[SKIP]${RESET} pca_testbench not built (build with: cmake --build build --target pca_testbench)"
    fi

    ## CPU frequency check
    run_test "CPU Frequency >= 2.8 GHz" \
        '[ $(vcgencmd measure_clock arm 2>/dev/null | cut -d= -f2) -ge 2800000000 ] 2>/dev/null'

    ## Temperature check
    run_test "CPU Temperature < 80C" \
        '[ $(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo 99000) -lt 80000 ]'
fi

##============= SUMMARY ====================================================================

echo -e "\n${CYAN}══════════════════════════════════════════${RESET}"
echo -e "  RESULTS: ${GREEN}${PASS} PASS${RESET}, ${RED}${FAIL} FAIL${RESET}"
echo -e "${CYAN}══════════════════════════════════════════${RESET}"

exit ${FAIL}
