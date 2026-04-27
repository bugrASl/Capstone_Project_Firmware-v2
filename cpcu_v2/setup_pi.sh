#!/bin/bash
##
##  setup_pi.sh — One-time Raspberry Pi Setup for CPCU (v2.3)
##  Author: bugrASl
##  Date:   April 2026
##
##  USAGE — RUN AS REGULAR USER (no sudo needed at the prompt):
##      ./setup_pi.sh
##
##  The script self-elevates to root via sudo for the operations that
##  truly require it (apt, file edits in /boot/firmware, udev rules,
##  group creation). All user-facing invocation is sudo-free; you'll
##  see ONE password prompt from sudo when the script re-execs itself.
##
##  Idempotent — safe to run multiple times. Skips work that's already
##  done (groups exist, lines already in config.txt, etc.).
##
##  v2.3 changes:
##      - Self-elevates via `exec sudo "$0" "$@"` rather than failing
##        with a "run as root" error. The user now invokes everything
##        with `./setup_pi.sh`, never `sudo bash setup_pi.sh`.
##      - Splits work into two phases: "as root" (apt + /boot/firmware
##        + udev) and "as user" (verify Python imports, print next-step
##        commands using `./` invocations not `sudo` ones).
##      - The "Next steps" hint at the end now points at `./run_tests.sh`
##        and `./launch.sh tui` — both are sudo-wrapped, so the user
##        never has to type sudo manually after this.
##

set -e

##============= SELF-ELEVATE ===============================================================
##
##  If we're not root, re-exec ourselves under sudo. The whole script
##  body below then runs as root for the duration of this invocation —
##  but the user only typed "./setup_pi.sh" and saw one password prompt.
##
if [ "$(id -u)" -ne 0 ]; then
    echo "[setup_pi] Re-exec under sudo (you'll be prompted for your password)..."
    exec sudo --preserve-env=HOME,USER,SUDO_USER "$0" "$@"
fi

REAL_USER="${SUDO_USER:-$(logname 2>/dev/null || echo root)}"
echo "=== CPCU v2.3 Raspberry Pi Setup ==="
echo "  Acting as: root  (real user: ${REAL_USER})"
echo ""

##============= APT DEPENDENCIES ===========================================================

echo "[1/6] Installing system packages..."
apt update
apt install -y \
    build-essential \
    cmake \
    libncurses-dev \
    i2c-tools \
    tmux \
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

# /opt/cpcu and /var/log/cpcu writeable by the real user so they can
# `cmake --install` and tail logs without sudo.
chown -R "${REAL_USER}:${REAL_USER}" /opt/cpcu /var/log/cpcu

##============= RUNTIME CONFIG SYMLINK (v2.3.3) ============================================
##
## cpcu_kernel reads /opt/cpcu/config.json on startup and on SIGHUP. We
## want the file to live in the repo (so it's git-versioned) but be
## accessible at a stable system path. Symlink does both:
##
##     /opt/cpcu/config.json -> <REPO>/cpcu_v2/config/runtime.json
##
## REAL_USER edits the file in the repo via configure.sh or directly,
## and the daemon picks it up on next SIGHUP.
##
## We resolve the repo location by looking up where this setup_pi.sh
## actually lives — that's the most robust regardless of how the user
## cloned or moved the tree.

REPO_RUNTIME="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/config/runtime.json"
SYS_CONFIG="/opt/cpcu/config.json"

if [ -f "${REPO_RUNTIME}" ]; then
    if [ -L "${SYS_CONFIG}" ] && [ "$(readlink "${SYS_CONFIG}")" = "${REPO_RUNTIME}" ]; then
        echo "  config.json symlink already correct"
    else
        ln -sfn "${REPO_RUNTIME}" "${SYS_CONFIG}"
        echo "  Linked ${SYS_CONFIG} -> ${REPO_RUNTIME}"
    fi
else
    echo "  WARNING: ${REPO_RUNTIME} not found — runtime config not linked."
    echo "           cpcu_kernel will refuse to start until you create one."
    echo "           Run: ./configure.sh --reset --runtime"
fi

##============= KERNEL CONFIG CHECK ========================================================

echo "[4/6] Checking kernel configuration..."

NEEDS_REBOOT=0

## Check config.txt
CONFIG="/boot/firmware/config.txt"
if [ -f "${CONFIG}" ]; then
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

## spi and i2c groups so cpcu_io can talk to /dev/spidev0.0 + /dev/i2c-1
## without needing root at runtime.
if ! getent group spi >/dev/null 2>&1; then
    groupadd spi
fi
if ! getent group i2c >/dev/null 2>&1; then
    groupadd i2c
fi

cat > /etc/udev/rules.d/90-cpcu.rules << 'EOF'
# CPCU: spi/i2c group access; gpiochip0 access for nrf24l01_linux's CE pin
SUBSYSTEM=="spidev",   GROUP="spi", MODE="0660"
SUBSYSTEM=="i2c-dev",  GROUP="i2c", MODE="0660"
SUBSYSTEM=="gpio",     GROUP="gpio", MODE="0660"
KERNEL=="gpiochip[0-9]*", GROUP="gpio", MODE="0660"
EOF

udevadm control --reload-rules 2>/dev/null || true

## Add the real user to spi/i2c/gpio so they can run cpcu_io / cpcu_kernel
## without sudo at runtime.
if [ "${REAL_USER}" != "root" ]; then
    usermod -aG spi  "${REAL_USER}" 2>/dev/null || true
    usermod -aG i2c  "${REAL_USER}" 2>/dev/null || true
    usermod -aG gpio "${REAL_USER}" 2>/dev/null || true
    echo "  Added ${REAL_USER} to spi, i2c, gpio groups"
fi

## SCHED_FIFO requires CAP_SYS_NICE. Easiest path that avoids sudo at
## runtime: grant the binaries themselves the capability after install.
## We can't do that here (binaries don't exist yet), but launch.sh
## documents the one-line setcap to apply after `cmake --install`.

##============= VERIFY =====================================================================

echo "[6/6] Verification..."

echo "  Python3: $(python3 --version 2>&1)"
echo "  NumPy:   $(python3 -c 'import numpy; print(numpy.__version__)' 2>&1)"
echo "  SciPy:   $(python3 -c 'import scipy; print(scipy.__version__)' 2>&1)"
echo "  sklearn: $(python3 -c 'import sklearn; print(sklearn.__version__)' 2>&1)"
echo "  CMake:   $(cmake --version 2>&1 | head -1)"
echo "  GCC:     $(gcc --version 2>&1 | head -1)"
echo "  tmux:    $(tmux -V 2>&1 || echo 'NOT INSTALLED')"

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
echo "  Isolated cores: ${ISOLATED}"

##============= DONE =======================================================================

echo ""
echo "=== Setup Complete ==="
echo ""
echo "Next steps — none of these need sudo at the prompt; the scripts"
echo "self-elevate when needed:"
echo ""
echo "  1. Build:           cmake -S . -B build && cmake --build build -j4"
echo "  2. Install:         cmake --install build      # writes to /opt/cpcu/{bin,scripts}"
echo "  3. Drop ML model:   cp /path/to/emg_rf_model.pkl /opt/cpcu/models/"
echo "  4. Run tests:       ./run_tests.sh 1           # software-only smoke"
echo "                      ./run_tests.sh             # all phases (Pi hardware)"
echo "                      ./run_tests.sh pca         # interactive servo TUI"
echo "  5. Launch live:     ./scripts/launch.sh tui    # tmux: kernel + TUI"
echo "                      ./scripts/launch.sh menu   # interactive picker"
echo "  6. Enable on boot:  ./scripts/launch.sh install-service"
echo ""
echo "If a script needs root for a specific operation, it'll re-exec"
echo "itself under sudo and prompt you exactly once. You should never"
echo "have to type 'sudo' before any of the above commands."

if [ "${NEEDS_REBOOT:-0}" -eq 1 ]; then
    echo ""
    echo "*** REBOOT REQUIRED for kernel config changes ***"
    echo "    Run: sudo reboot   (the only sudo you'll need today)"
fi
