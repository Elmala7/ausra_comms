#!/bin/bash
# ============================================================
# FILE: start_base.sh
# PACKAGE: ausra_comms_base
# RUNS ON: Laptop — 2-Jetson + Laptop deployment
# PURPOSE: Base station for the AUSRA swarm:
#          - Verifies both Jetsons are reachable
#          - Launches map merge pipeline (map_expansion + merge)
#          - Launches RViz2 for visualization
#          Both ausra_1 and ausra_2 data arrive via DDS from
#          real Jetsons — no fake publishers needed.
#
# USAGE: ./start_base.sh
#
# PREREQUISITES:
#   - All machines on the same WiFi network
#   - Both Jetsons running hardware_with_comms.launch.py
#
# PLACEHOLDERS: JETSON1_IP, JETSON2_IP
# ============================================================
set -e

# --- Source ROS2 Humble and workspace ---
source /opt/ros/humble/setup.bash
source ~/ausra_ws/install/setup.bash

# --- ROS2 environment ---
export ROS_DOMAIN_ID=0
# export ROS_LOCALHOST_ONLY=0

echo "============================================"
echo "[Base] AUSRA Base Station — 2-Jetson Mode"
echo "[Base] Checking Jetson connectivity..."
echo "============================================"

# ┌──────────────────────────────────────────────────────┐
# │  REPLACE THESE with each Jetson's WiFi IP            │
# │  Find them on each Jetson: ip addr show wlan0        │
# └──────────────────────────────────────────────────────┘
JETSON1_IP="192.168.1.3"
JETSON2_IP="192.168.1.4"

# --- Connectivity checks ---
ALL_OK=true

if [ "$JETSON1_IP" = "INSERT_JETSON1_IP_HERE" ]; then
    echo "[Base] ⚠  JETSON1_IP not set!"
    ALL_OK=false
elif ping -c 1 -W 2 "$JETSON1_IP" > /dev/null 2>&1; then
    echo "[Base] ✓ Jetson 1 / ausra_1 (${JETSON1_IP}) — reachable"
else
    echo "[Base] ✗ Jetson 1 / ausra_1 (${JETSON1_IP}) — NOT reachable"
    ALL_OK=false
fi

if [ "$JETSON2_IP" = "INSERT_JETSON2_IP_HERE" ]; then
    echo "[Base] ⚠  JETSON2_IP not set!"
    ALL_OK=false
elif ping -c 1 -W 2 "$JETSON2_IP" > /dev/null 2>&1; then
    echo "[Base] ✓ Jetson 2 / ausra_2 (${JETSON2_IP}) — reachable"
else
    echo "[Base] ✗ Jetson 2 / ausra_2 (${JETSON2_IP}) — NOT reachable"
    ALL_OK=false
fi

if [ "$ALL_OK" = false ]; then
    echo ""
    echo "[Base] ⚠  One or more Jetsons unreachable."
    echo "[Base]    Edit this file: $(readlink -f $0)"
    echo "[Base]    Set JETSON1_IP and JETSON2_IP to real WiFi IPs."
    echo ""
    read -p "[Base] Continue anyway? (y/N) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "============================================"
echo "[Base] Launching: map merge + RViz2"
echo "============================================"

# ──────────────────────────────────────────────────────────
# FALLBACK: If only 1 Jetson is available, uncomment these
# lines to run a fake publisher for the missing robot.
#
# echo "[Base] Starting fake ausra_2 publisher..."
# ros2 run ausra_comms_base fake_robot_pub --ros-args \
#     -p robot_name:=ausra_2 \
#     -p robot_index:=2 \
#     -p map_interval_sec:=10.0 &
# FAKE_PID=$!
# sleep 1
# ──────────────────────────────────────────────────────────

# --- Map merge (AUSRA architecture) ---
echo "[Base] Starting map merge pipeline..."
ros2 launch ausra_comms_base map_merge.launch.py &
MERGE_PID=$!
sleep 3

# --- RViz2 ---
echo "[Base] Starting RViz2..."
ros2 run rviz2 rviz2 &
RVIZ_PID=$!

echo "============================================"
echo "[Base] All components running:"
echo "  Map Merge:      PID $MERGE_PID"
echo "  RViz2:          PID $RVIZ_PID"
echo ""
echo "  ausra_1 data arrives via DDS from Jetson 1."
echo "  ausra_2 data arrives via DDS from Jetson 2."
echo "  Press Ctrl+C to stop everything."
echo "============================================"
echo ""
echo "  Verify in another terminal:"
echo "    source /opt/ros/humble/setup.bash && source ~/ausra_ws/install/setup.bash"
echo "    export ROS_DOMAIN_ID=0"
echo "    ros2 topic echo /ausra_1/heartbeat   # from Jetson 1"
echo "    ros2 topic echo /ausra_2/heartbeat   # from Jetson 2"
echo "    ros2 topic echo /map_merged --no-arr  # merged map"
echo ""

# --- Cleanup on Ctrl+C ---
cleanup() {
    echo ""
    echo "[Base] Shutting down..."
    kill $RVIZ_PID $MERGE_PID 2>/dev/null
    # Uncomment if using fake publisher:
    # kill $FAKE_PID 2>/dev/null
    wait 2>/dev/null
    echo "[Base] Done."
}
trap cleanup SIGINT SIGTERM

wait
