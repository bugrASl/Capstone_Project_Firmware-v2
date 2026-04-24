#!/bin/bash
##
##  setup_pi.sh — One-time Raspberry Pi Setup for CPCU
##  Author: bugrASl
##  Date:   April 2026
##
##  Run once on a fresh Raspbian install:
##      sudo bash setup_pi.sh
##

set -e

echo "=== CPCU v2.1 Raspberry Pi Setup ==="

##============= APT DEPENDENCIES ===========================================================

echo "[1/6] Installing system packages..."
apt update
apt install -y \
    build-essential \
    cmake \
    libncurses-dev \
    i2c-tools \
    python3-numpy \
    python3-scipy \
    python3-pip

##============= PYTHON DEPENDENCIES ========================================================

echo "[2/6] Installing Python packages..."
pip3 install --break-system-packages \
    joblib \
    scikit-learn

##============= DIRECTORY STRUCTURE ========================================================

echo "[3/6] Creating directory structure..."
mkdir -p /opt/cpcu/bin
mkdir -p /opt/cpcu/scripts
mkdir -p /opt/cpcu/models
mkdir -p /opt/cpcu/test
mkdir -p /var/log/cpcu

##============= KERNEL CONFIG CHECK ========================================================

echo "[4/6] Checking kernel configuration..."

## Check config.txt
CONFIG="/boot/firmware/config.txt"
if [ -f "${CONFIG}" ]; then
    NEEDS_REBOOT=0
    
    if ! grep -q "dtparam=spi=on" "${CONFIG}"; then
        echo "  Adding SPI enable to ${CONFIG}"
        echo "dtparam=spi=on" >> "${CONFIG}"
        NEEDS_REBOOT=1
    fi
    
    if ! grep -q "i2c_arm_baudrate=400000" "${CONFIG}"; then
        echo "  Adding I2C 400kHz to ${CONFIG}"
        echo "dtparam=i2c_arm_baudrate=400000" >> "${CONFIG}"
        NEEDS_REBOOT=1
    fi
    
    if ! grep -q "arm_freq=2800" "${CONFIG}"; then
        echo "  NOTE: Overclock (arm_freq=2800) not set. Add manually if desired."
    fi
    
    if ! grep -q "dtoverlay=disable-bt" "${CONFIG}"; then
        echo "  Adding Bluetooth disable to ${CONFIG}"
        echo "dtoverlay=disable-bt" >> "${CONFIG}"
        NEEDS_REBOOT=1
    fi
else
    echo "  WARNING: ${CONFIG} not found"
fi

## Check cmdline.txt
CMDLINE="/boot/firmware/cmdline.txt"
if [ -f "${CMDLINE}" ]; then
    if ! grep -q "isolcpus" "${CMDLINE}"; then
        echo "  Adding core isolation to ${CMDLINE}"
        sed -i 's/$/ isolcpus=1,2,3 nohz_full=1,2,3 rcu_nocbs=1,2,3/' "${CMDLINE}"
        NEEDS_REBOOT=1
    else
        echo "  Core isolation already configured"
    fi
else
    echo "  WARNING: ${CMDLINE} not found"
fi

##============= PERMISSIONS ================================================================

echo "[5/6] Setting permissions..."

## Allow non-root access to SPI and I2C (for development only)
## In production, cpcu runs as root via systemd
if ! getent group spi >/dev/null 2>&1; then
    groupadd spi
fi
if ! getent group i2c >/dev/null 2>&1; then
    groupadd i2c
fi

## udev rules for SPI and I2C
cat > /etc/udev/rules.d/90-cpcu.rules << 'EOF'
# CPCU: Allow spi and i2c group access to devices
SUBSYSTEM=="spidev", GROUP="spi", MODE="0660"
SUBSYSTEM=="i2c-dev", GROUP="i2c", MODE="0660"
EOF

## Add current user to groups
CURRENT_USER="${SUDO_USER:-$(whoami)}"
if [ "${CURRENT_USER}" != "root" ]; then
    usermod -aG spi "${CURRENT_USER}" 2>/dev/null || true
    usermod -aG i2c "${CURRENT_USER}" 2>/dev/null || true
    echo "  Added ${CURRENT_USER} to spi and i2c groups"
fi

##============= VERIFY ====================================================================

echo "[6/6] Verification..."

echo "  Python3: $(python3 --version 2>&1)"
echo "  NumPy:   $(python3 -c 'import numpy; print(numpy.__version__)' 2>&1)"
echo "  SciPy:   $(python3 -c 'import scipy; print(scipy.__version__)' 2>&1)"
echo "  sklearn: $(python3 -c 'import sklearn; print(sklearn.__version__)' 2>&1)"
echo "  CMake:   $(cmake --version 2>&1 | head -1)"
echo "  GCC:     $(gcc --version 2>&1 | head -1)"

if [ -e /dev/spidev0.0 ]; then
    echo "  SPI0:    OK (/dev/spidev0.0 exists)"
else
    echo "  SPI0:    NOT READY (reboot needed)"
fi

if [ -e /dev/i2c-1 ]; then
    echo "  I2C-1:   OK (/dev/i2c-1 exists)"
else
    echo "  I2C-1:   NOT READY (reboot needed)"
fi

ISOLATED=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "none")
echo "  Isolated: ${ISOLATED}"

##============= DONE ======================================================================

echo ""
echo "=== Setup Complete ==="
echo ""
echo "Next steps:"
echo "  1. Copy your code:  cd /opt/cpcu && git clone <repo>"
echo "  2. Build:           mkdir build && cd build && cmake .. && make -j4"
echo "  3. Install:         sudo make install"
echo "  4. Copy model:      cp emg_rf_model.pkl /opt/cpcu/models/"
echo "  5. Enable service:  sudo systemctl enable cpcu"
echo "  6. Start:           sudo systemctl start cpcu"
echo "  7. Monitor:         journalctl -u cpcu -f"
echo "  8. TUI:             /opt/cpcu/bin/cpcu_tui"

if [ "${NEEDS_REBOOT:-0}" -eq 1 ]; then
    echo ""
    echo "*** REBOOT REQUIRED for kernel config changes ***"
    echo "    sudo reboot"
fi
