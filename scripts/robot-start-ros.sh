#!/usr/bin/env bash
# Run on the Raspbot V2 (Raspberry Pi) to start base driver + rosbridge.
# Usage: ./robot-start-ros.sh   (or bash robot-start-ros.sh)
# Bringup runs in background; rosbridge runs in foreground (Ctrl+C stops rosbridge only).

set -e
cd -P "$(dirname "$0")"

# Source ROS2 if not already (adjust if your install uses a different path)
if [ -z "$ROS_DISTRO" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    fi
fi

echo "Starting yahboomcar bringup in background..."
nohup ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py > /tmp/raspbot-bringup.log 2>&1 &
BRINGUP_PID=$!
echo "Bringup PID: $BRINGUP_PID (log: /tmp/raspbot-bringup.log)"

echo "Waiting 5s for bringup to bind..."
sleep 5

echo "Starting rosbridge (foreground; Ctrl+C to stop)..."
exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml
