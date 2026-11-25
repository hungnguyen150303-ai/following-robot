#!/bin/bash
# MiR Robot Person Follower - Quick Start Script

set -e

# Auto-detect docker-compose or docker compose
if command -v docker-compose &> /dev/null; then
    DOCKER_COMPOSE="docker-compose"
elif docker compose version &> /dev/null 2>&1; then
    DOCKER_COMPOSE="docker compose"
else
    echo "❌ Error: Neither 'docker-compose' nor 'docker compose' found!"
    echo "Please install Docker Compose:"
    echo "  sudo apt-get update"
    echo "  sudo apt-get install docker-compose-plugin"
    exit 1
fi

echo "🤖 Starting MiR Robot Person Follower..."
echo "Using: $DOCKER_COMPOSE"

# Get robot IP from Python dialog BEFORE starting container
echo "📡 Getting robot IP..."
ROBOT_IP=$(python3 -c "
from PyQt5.QtWidgets import QApplication, QInputDialog, QLineEdit, QMessageBox
import sys

app = QApplication(sys.argv)
default_ip = '192.168.12.20'
ip, ok = QInputDialog.getText(None, 'Kết nối Robot MiR', 
                                'Nhập địa chỉ IP của robot MiR:', 
                                QLineEdit.Normal, default_ip)
if ok and ip:
    parts = ip.split('.')
    if len(parts) == 4 and all(part.isdigit() and 0 <= int(part) <= 255 for part in parts):
        print(ip)
        sys.exit(0)
    else:
        QMessageBox.warning(None, 'Lỗi', 'Địa chỉ IP không hợp lệ!')
        sys.exit(1)
else:
    QMessageBox.warning(None, 'Lỗi', 'Cần nhập IP robot để tiếp tục!')
    sys.exit(1)
" 2>/dev/null)

if [ $? -ne 0 ] || [ -z "$ROBOT_IP" ]; then
    echo "❌ Không nhận được IP hợp lệ. Thoát."
    exit 1
fi

export ROBOT_IP
export ROS_MASTER_URI="http://localhost:11311"
export ROS_HOSTNAME=$(hostname -I | awk '{print $1}')

echo "📡 Robot IP: ${ROBOT_IP}"
echo "🔗 ROS Master: http://localhost:11311 (container)"

# Set up X11 and PulseAudio
xhost +local:docker > /dev/null 2>&1 || true
[ -z "$DISPLAY" ] && export DISPLAY=:0

# Start PulseAudio if needed
if ! pulseaudio --check 2>/dev/null; then
    pulseaudio --start --exit-idle-time=-1 2>/dev/null || true
fi
pactl load-module module-native-protocol-unix socket=/tmp/pulseaudio.socket 2>/dev/null || true

# Create assets directory
mkdir -p assets

# Check if image exists
echo "🔍 Checking Docker image..."
if docker images mir_follower:latest | grep -q mir_follower; then
    echo "✅ Image found, using cache (only rebuild if files changed)"
    BUILD_FLAG=""
else
    echo "⚠️  Image not found, building from scratch..."
    BUILD_FLAG="--build"
fi

# Build and run
echo "🚀 Launching application..."
$DOCKER_COMPOSE down 2>/dev/null || true
$DOCKER_COMPOSE up $BUILD_FLAG

# Cleanup
$DOCKER_COMPOSE down 2>/dev/null || true
echo "✅ MiR Follower stopped"
