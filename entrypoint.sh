#!/bin/bash
set -e

# Source ROS
source /opt/ros/noetic/setup.bash

# Source catkin workspace with MiR packages
if [ -f /root/catkin_ws/devel/setup.bash ]; then
  source /root/catkin_ws/devel/setup.bash
fi

# Get host IP and robot IP
ROBOT_IP=${ROBOT_IP:-192.168.12.20}
HOST_IP=$(hostname -I | awk '{print $1}')

# Set ROS environment for local master (container runs roscore)
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=$HOST_IP
export ROS_HOSTNAME=$HOST_IP

echo "========================================"
echo "🤖 MiR FOLLOWER CONTAINER STARTING"
echo "========================================"
echo "ROS Master URI: $ROS_MASTER_URI (local)"
echo "ROS IP: $ROS_IP"
echo "ROS Hostname: $ROS_HOSTNAME"
echo "Target Robot IP: $ROBOT_IP"
echo "========================================"
echo ""

# Start roscore in background
echo "🔧 Starting ROS master..."
roscore &
ROSCORE_PID=$!
sleep 3

# Launch MiR driver to connect to robot
echo "� Launching MiR driver (connecting to robot at $ROBOT_IP)..."
roslaunch mir_driver mir.launch mir_hostname:=$ROBOT_IP &
MIR_DRIVER_PID=$!
sleep 5

# Check if topics are available
echo ""
echo "🔍 Checking ROS topics from robot..."
if timeout 5 rostopic list >/dev/null 2>&1; then
  echo "✅ Connected to robot via MiR driver!"
  echo ""
  echo "📋 Available ROS topics (first 20):"
  rostopic list | head -20
  echo "..."
else
  echo "⚠️  Cannot see robot topics yet, driver may still be connecting..."
fi
echo ""

# Execute main command
exec "$@"
