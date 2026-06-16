#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source ~/ausra_ws/install/setup.bash

export ROS_DOMAIN_ID=0

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

JETSON1_IP="192.168.0.165"
JETSON2_IP="192.168.0.129"

ALL_OK=true

if [ "$JETSON1_IP" = "INSERT_JETSON1_IP_HERE" ]; then
    echo "[Base] JETSON1_IP not set!"
    ALL_OK=false
elif ping -c 1 -W 2 "$JETSON1_IP" > /dev/null 2>&1; then
    echo "[Base] Jetson 1 reachable: ${JETSON1_IP}"
else
    echo "[Base] Jetson 1 NOT reachable: ${JETSON1_IP}"
    ALL_OK=false
fi

if [ "$JETSON2_IP" = "INSERT_JETSON2_IP_HERE" ]; then
    echo "[Base] JETSON2_IP not set!"
    ALL_OK=false
elif ping -c 1 -W 2 "$JETSON2_IP" > /dev/null 2>&1; then
    echo "[Base] Jetson 2 reachable: ${JETSON2_IP}"
else
    echo "[Base] Jetson 2 NOT reachable: ${JETSON2_IP}"
    ALL_OK=false
fi

if [ "$ALL_OK" = false ]; then
    read -p "[Base] Continue anyway? (y/N) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "============================================"
echo "[Base] Launching: Zenoh bridge + decompressor + map merge + RViz2"
echo "[Base] USE_ZENOH=${USE_ZENOH}  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "============================================"

ros2 launch ausra_comms_base base_station.launch.py use_zenoh:=${USE_ZENOH} "$@"
