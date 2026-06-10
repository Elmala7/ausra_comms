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

# Zenoh is now the only cross-machine channel. Pin DDS to loopback so the
# WiFi flood (full DDS discovery + topic gossip) cannot leave this machine.
# To revert to plain DDS, set USE_ZENOH=false and unset ROS_LOCALHOST_ONLY.
# See ZENOH_GUIDE.md → "Revert".
USE_ZENOH="${USE_ZENOH:-true}"
if [ "$USE_ZENOH" = "true" ]; then
    export ROS_LOCALHOST_ONLY=1
else
    export ROS_LOCALHOST_ONLY=0
fi

echo "============================================"
echo "[Base] AUSRA Base Station — 2-Jetson Mode"
echo "[Base] Checking Jetson connectivity..."
echo "============================================"

# ┌──────────────────────────────────────────────────────┐
# │  REPLACE THESE with each Jetson's WiFi IP            │
# │  Find them on each Jetson: ip addr show wlan0        │
# └──────────────────────────────────────────────────────┘
JETSON1_IP="192.168.1.33"
JETSON2_IP="192.168.1.42"

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
echo "[Base] Launching: Zenoh bridge + map merge + RViz2"
echo "[Base] USE_ZENOH=${USE_ZENOH}  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
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

# --- Full base station: Zenoh bridge + map merge + RViz2 ---
# base_station.launch.py wraps everything (Zenoh bridge with respawn,
# map_merge.launch.py, RViz2). use_zenoh is forwarded from this script.
echo "[Base] Starting base_station.launch.py..."
ros2 launch ausra_comms_base base_station.launch.py use_zenoh:=${USE_ZENOH} &
BASE_PID=$!
sleep 3

echo "============================================"
echo "[Base] All components running:"
echo "  Base Station:   PID $BASE_PID"
echo ""
if [ "$USE_ZENOH" = "true" ]; then
    echo "  Transport: Zenoh bridge (peer mesh on tcp/7447)"
    echo "  DDS scope: loopback only (ROS_LOCALHOST_ONLY=1)"
    echo "  Allowlist: /ausra_*/map  /ausra_*/pose  /ausra_*/heartbeat"
else
    echo "  Transport: plain DDS multicast (Zenoh disabled)"
fi
echo "  Press Ctrl+C to stop everything."
echo "============================================"
echo ""
echo "  Verify in another terminal:"
echo "    source /opt/ros/humble/setup.bash && source ~/ausra_ws/install/setup.bash"
echo "    export ROS_DOMAIN_ID=0"
if [ "$USE_ZENOH" = "true" ]; then
    echo "    export ROS_LOCALHOST_ONLY=1"
fi
echo "    ros2 topic echo /ausra_1/heartbeat   # from Jetson 1"
echo "    ros2 topic echo /ausra_2/heartbeat   # from Jetson 2"
echo "    ros2 topic echo /map_merged --no-arr  # merged map"
echo ""

# --- Cleanup on Ctrl+C ---
cleanup() {
    echo ""
    echo "[Base] Shutting down..."
    kill $BASE_PID 2>/dev/null
    # Uncomment if using fake publisher:
    # kill $FAKE_PID 2>/dev/null
    wait 2>/dev/null
    echo "[Base] Done."
}
trap cleanup SIGINT SIGTERM

wait
