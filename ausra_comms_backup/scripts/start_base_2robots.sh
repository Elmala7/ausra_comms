#!/bin/bash
# ============================================================
# FILE: start_base_2robots.sh
# RUNS ON: Laptop — 2-robot mode
# PURPOSE: Laptop acts as ausra_2 AND base station:
#          - Fake ausra_2 publisher (pose + heartbeat + map)
#          - Map merge (AUSRA architecture)
#          - RViz2
#          ausra_1 topics arrive from Jetson via DDS.
#
# USAGE: ./start_base_2robots.sh
#
# PREREQUISITES:
#   - Both machines on the same WiFi network
#   - Jetson running hardware_full_stack + relay_node
#
# PLACEHOLDER: INSERT_JETSON_IP_HERE
# ============================================================
set -e

# --- Source ROS2 Humble and workspace ---
source /opt/ros/humble/setup.bash
source ~/ausra_ws/install/setup.bash

# --- ROS2 environment ---
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "============================================"
echo "[Base] 2-robot mode — Laptop as ausra_2 + Base Station"
echo "[Base] Checking Jetson connectivity..."
echo "============================================"

# ┌──────────────────────────────────────────────┐
# │  REPLACE THIS with Jetson's WiFi IP          │
# │  Find it on Jetson with: ip addr show wlan0  │
# └──────────────────────────────────────────────┘
JETSON_IP="192.168.1.33"

if [ "$JETSON_IP" = "INSERT_JETSON_IP_HERE" ]; then
    echo "[Base] ⚠  You haven't set JETSON_IP yet!"
    echo "[Base]    Edit this file: $(readlink -f $0)"
    echo "[Base]    Replace INSERT_JETSON_IP_HERE with Jetson's WiFi IP"
    exit 1
fi

if ping -c 1 -W 2 "$JETSON_IP" > /dev/null 2>&1; then
    echo "[Base] ✓ Jetson (${JETSON_IP}) — reachable"
else
    echo "[Base] ✗ Jetson (${JETSON_IP}) — NOT reachable"
    echo "[Base] Check WiFi connection. Both machines must be on same network."
    exit 1
fi

echo "============================================"
echo "[Base] Launching: fake ausra_2 + map merge + RViz2"
echo "============================================"

# --- Fake ausra_2 publisher ---
echo "[Base] Starting fake ausra_2 publisher..."
ros2 run ausra_comms fake_robot_pub --ros-args \
    -p robot_name:=ausra_2 \
    -p robot_index:=2 \
    -p map_interval_sec:=10.0 &
FAKE_PID=$!
sleep 1

# --- Map merge (AUSRA architecture) ---
echo "[Base] Starting map merge..."
ros2 launch ausra_comms map_merge.launch.py &
MERGE_PID=$!
sleep 3

# --- RViz2 ---
echo "[Base] Starting RViz2..."
ros2 run rviz2 rviz2 &
RVIZ_PID=$!

echo "============================================"
echo "[Base] All components running:"
echo "  Fake ausra_2:   PID $FAKE_PID"
echo "  Map Merge:      PID $MERGE_PID"
echo "  RViz2:          PID $RVIZ_PID"
echo ""
echo "  ausra_1 data arrives via DDS from Jetson."
echo "  Press Ctrl+C to stop everything."
echo "============================================"
echo ""
echo "  Verify in another terminal:"
echo "    source /opt/ros/humble/setup.bash && source ~/ausra_ws/install/setup.bash"
echo "    export ROS_DOMAIN_ID=0"
echo "    ros2 topic echo /ausra_1/heartbeat   # from real Jetson"
echo "    ros2 topic echo /ausra_2/heartbeat   # from fake publisher"
echo "    ros2 topic echo /map_merged --no-arr  # merged map"
echo ""

# --- Cleanup on Ctrl+C ---
cleanup() {
    echo ""
    echo "[Base] Shutting down..."
    kill $RVIZ_PID $MERGE_PID $FAKE_PID 2>/dev/null
    wait 2>/dev/null
    echo "[Base] Done."
}
trap cleanup SIGINT SIGTERM

wait
