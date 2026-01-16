#!/bin/bash
# ============================================================
# NICE DCV Server Setup
# For: GPU-accelerated remote desktop
# Port: 8443
# ============================================================

set -e

echo "=== NICE DCV Setup ==="

# 1. Download DCV packages
echo "[1/5] Downloading NICE DCV..."
cd /tmp
wget -q https://d1uj6qtbmh3dt5.cloudfront.net/2023.1/Servers/nice-dcv-2023.1-16220-ubuntu2204-x86_64.tgz
tar -xzf nice-dcv-2023.1-16220-ubuntu2204-x86_64.tgz
cd nice-dcv-2023.1-16220-ubuntu2204-x86_64

# 2. Install DCV packages
echo "[2/5] Installing DCV packages..."
sudo dpkg -i nice-dcv-server_*.deb || sudo apt-get install -f -y
sudo dpkg -i nice-xdcv_*.deb || sudo apt-get install -f -y

# 3. Configure DCV (with auto-session on reboot)
echo "[3/5] Configuring DCV..."
sudo tee /etc/dcv/dcv.conf > /dev/null << 'EOF'
[license]
[log]

[session-management]
virtual-session-xdg-runtime-dir="/tmp/.dcv-xdg-runtime-dir"
create-session = true

[session-management/defaults]

[session-management/automatic-console-session]
owner="ubuntu"
storage-root="/home/ubuntu"
session-type="virtual"
name="main"

[display]
cuda-devices=["0"]

[connectivity]
idle-timeout=60
enable-quic-frontend=false

[security]
authentication="system"
no-tls-strict=true
EOF

# 4. Create XFCE session startup
echo "[4/5] Creating session startup..."
cat > ~/.xsession << 'EOF'
#!/bin/bash
exec startxfce4
EOF
chmod +x ~/.xsession

# 5. Start DCV service
echo "[5/5] Starting DCV service..."
sudo systemctl enable dcvserver
sudo systemctl restart dcvserver

# Create virtual session
sleep 3
sudo dcv create-session --type=virtual --owner ubuntu ubuntu-session || true

echo ""
echo "=== NICE DCV Setup Complete ==="
echo ""
echo "Connection Info:"
echo "  Address: <EC2-PUBLIC-IP>:8443"
echo "  Session: ubuntu-session"
echo "  Username: ubuntu"
echo "  Password: (your ubuntu password)"
echo ""
echo "Client Download: https://download.nice-dcv.com/"
echo ""
echo "Useful Commands:"
echo "  List sessions:   sudo dcv list-sessions"
echo "  Create session:  sudo dcv create-session --type=virtual --owner ubuntu ubuntu-session"
echo "  Close session:   sudo dcv close-session ubuntu-session"
echo "  Server status:   sudo systemctl status dcvserver"
