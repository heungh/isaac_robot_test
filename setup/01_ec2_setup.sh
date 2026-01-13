#!/bin/bash
# ============================================================
# EC2 Instance Initial Setup
# Instance Type: g5.4xlarge (NVIDIA A10G GPU)
# OS: Ubuntu 22.04 LTS
# ============================================================

set -e

echo "=== EC2 Instance Setup ==="

# 1. System Update
echo "[1/4] Updating system packages..."
sudo apt update && sudo apt upgrade -y

# 2. Install Essential Packages
echo "[2/4] Installing essential packages..."
sudo apt install -y \
    git \
    git-lfs \
    wget \
    curl \
    unzip \
    python3.10 \
    python3.10-venv \
    python3-pip \
    build-essential \
    libssl-dev \
    libffi-dev \
    net-tools \
    htop

# 3. Install Desktop Environment (XFCE4)
echo "[3/4] Installing XFCE4 desktop environment..."
sudo apt install -y xfce4 xfce4-goodies

# 4. Set User Password
echo "[4/4] Setting user password..."
echo "ubuntu:isaac123" | sudo chpasswd

echo ""
echo "=== EC2 Setup Complete ==="
echo "Next: Run 02_nvidia_driver.sh"
