#!/bin/bash
# ============================================================
# NVIDIA Driver Installation
# For: NVIDIA A10G GPU (g5.4xlarge)
# ============================================================

set -e

echo "=== NVIDIA Driver Installation ==="

# Check if NVIDIA driver is already installed
if command -v nvidia-smi &> /dev/null; then
    echo "NVIDIA driver already installed:"
    nvidia-smi
    exit 0
fi

# 1. Install ubuntu-drivers-common
echo "[1/3] Installing driver utilities..."
sudo apt install -y ubuntu-drivers-common

# 2. Auto-install recommended driver
echo "[2/3] Installing NVIDIA driver..."
sudo ubuntu-drivers autoinstall

# 3. Reboot notice
echo "[3/3] Installation complete."
echo ""
echo "=== NVIDIA Driver Setup Complete ==="
echo ""
echo "*** REBOOT REQUIRED ***"
echo "Run: sudo reboot"
echo ""
echo "After reboot, verify with: nvidia-smi"
echo "Then run: 03_isaac_sim_install.sh"
