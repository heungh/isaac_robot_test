#!/bin/bash
# ============================================================
# Isaac Sim Installation
# Version: 5.1.0
# ============================================================

set -e

echo "=== Isaac Sim Installation ==="

# Verify NVIDIA driver
if ! command -v nvidia-smi &> /dev/null; then
    echo "ERROR: NVIDIA driver not found!"
    echo "Run 02_nvidia_driver.sh first and reboot."
    exit 1
fi

echo "NVIDIA Driver verified:"
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader

# 1. Create workspace
echo "[1/3] Creating workspace..."
mkdir -p ~/isaac-sim
cd ~/isaac-sim

# 2. Clone Isaac Sim repository
echo "[2/3] Cloning Isaac Sim repository..."
if [ ! -d "IsaacSim" ]; then
    git clone https://github.com/isaac-sim/IsaacSim.git
    cd IsaacSim
    git lfs pull
else
    echo "IsaacSim already cloned, updating..."
    cd IsaacSim
    git pull
    git lfs pull
fi

# 3. Set environment variables
echo "[3/3] Setting environment variables..."
if ! grep -q "ISAAC_SIM_PATH" ~/.bashrc; then
    cat >> ~/.bashrc << 'EOF'

# Isaac Sim Environment
export ISAAC_SIM_PATH="$HOME/isaac-sim/IsaacSim"
export PYTHONPATH="$ISAAC_SIM_PATH:$PYTHONPATH"
EOF
fi

source ~/.bashrc

echo ""
echo "=== Isaac Sim Installation Complete ==="
echo ""
echo "Isaac Sim Location: ~/isaac-sim/IsaacSim"
echo ""
echo "To run Isaac Sim:"
echo "  cd ~/isaac-sim/IsaacSim/_build/linux-x86_64/release"
echo "  ./isaac-sim.sh"
echo ""
echo "To run Python scripts:"
echo "  ./python.sh ~/your_script.py"
echo ""
echo "Next: Run 04_nicedcv_setup.sh for remote desktop access"
