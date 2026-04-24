#!/bin/bash
##
##  launch.sh — CPCU v2.2 Boot Script
##  Author: bugrASl
##  Date:   April 2026
##
##  Started by systemd (cpcu.service) at boot.
##  cpcu_kernel handles all spawning internally — this script just sets up
##  the environment, ensures directories exist, and runs the supervisor.
##
##  v2.2:
##      - Passes --log to cpcu_kernel so per-module CSVs are written to
##        /var/log/cpcu/log_{module}.csv (kern, io, nrf, pca, safe, dsp...)
##      - These CSVs are separate from the journalctl/stderr stream and
##        are what you post-process with pandas/matlab/gnuplot after a run.
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

##============= PRE-FLIGHT CHECKS =========================================

echo "[LAUNCH] CPCU v2.2 starting..."

## Create log directory (world-writable so unprivileged TUIs reading it work)
mkdir -p "${LOG_DIR}"
chmod 755 "${LOG_DIR}"

## Check binaries exist
if [ ! -x "${BIN_DIR}/cpcu_kernel" ]; then
    echo "[LAUNCH] FATAL: ${BIN_DIR}/cpcu_kernel not found"
    exit 1
fi

if [ ! -x "${BIN_DIR}/cpcu_io" ]; then
    echo "[LAUNCH] FATAL: ${BIN_DIR}/cpcu_io not found"
    exit 1
fi

## Check Python dependencies
if ! python3 -c "import numpy, scipy, joblib, sklearn" 2>/dev/null; then
    echo "[LAUNCH] WARN: Python dependencies missing. Install with:"
    echo "  sudo apt install python3-numpy python3-scipy"
    echo "  pip3 install joblib scikit-learn --break-system-packages"
fi

## Check DSP script
if [ ! -f "${SCRIPT_DIR}/cpcu_dsp.py" ]; then
    echo "[LAUNCH] WARN: ${SCRIPT_DIR}/cpcu_dsp.py not found"
    echo "  DSP pipeline will not start. System runs in IO-only mode."
fi

## Check model file
if [ ! -f "${MODEL_DIR}/emg_rf_model.pkl" ]; then
    echo "[LAUNCH] WARN: ${MODEL_DIR}/emg_rf_model.pkl not found"
    echo "  Copy trained model: cp emg_rf_model.pkl ${MODEL_DIR}/"
fi

## Check core isolation
ISOLATED=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "none")
echo "[LAUNCH] Isolated cores: ${ISOLATED}"
if [ "${ISOLATED}" = "none" ] || [ -z "${ISOLATED}" ]; then
    echo "[LAUNCH] WARN: No cores isolated! Add to /boot/firmware/cmdline.txt:"
    echo "  isolcpus=1,2,3 nohz_full=1,2,3 rcu_nocbs=1,2,3"
fi

## Check SPI
if [ ! -e /dev/spidev0.0 ]; then
    echo "[LAUNCH] WARN: /dev/spidev0.0 not found. Enable SPI in config.txt"
fi

## Check I2C
if [ ! -e /dev/i2c-1 ]; then
    echo "[LAUNCH] WARN: /dev/i2c-1 not found. Enable I2C in config.txt"
fi

##============= LAUNCH ====================================================

echo "[LAUNCH] Starting cpcu_kernel (supervisor on Core 0) with --log..."
echo "[LAUNCH] cpcu_kernel will spawn cpcu_io and cpcu_dsp internally."
echo "[LAUNCH] Per-module CSVs -> ${LOG_DIR}/log_*.csv"

cd "${BIN_DIR}"
exec taskset -c 0 ./cpcu_kernel --log 2>&1 | tee -a "${LOG_DIR}/cpcu.log"
