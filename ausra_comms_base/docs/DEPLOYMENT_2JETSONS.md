# Deployment Guide — 2 Jetsons + Laptop

Step-by-step guide for deploying the AUSRA swarm with **2 Jetson Orin Nanos** (ausra_1 + ausra_2) and **1 Laptop** (base station).

---

## Overview

| Machine | Workspace | Role | Packages needed | What it runs |
|---------|-----------|------|-----------------|-------------|
| **Jetson 1** | `~/ausra_NM_ws/` | ausra_1 (real SLAM) | `ausra_comms` + `lidar_slam_pkg` (already installed) | `hardware_with_comms.launch.py` |
| **Jetson 2** | `~/ausra_NM_ws/` | ausra_2 (real SLAM) | `ausra_comms` + `lidar_slam_pkg` (already installed) | `hardware_with_comms.launch.py` |
| **Laptop** | `~/ausra_ws/` | Base station | `ausra_comms_base` + `ausra_map_merge_HW` + `multirobot_map_merge` | `start_base.sh` |

### Data Flow

```
[ JETSON 1 — ausra_1 ]                              [ Zenoh ]           [ LAPTOP — Base Station ]

SLAM ──► /map ──► relay_node ──(throttle)──► /ausra_1/map_relay  ═══►  map_expansion_node ──► /ausra_1/map_fixed ──┐
                   relay_node (1Hz) ─────────► /ausra_1/heartbeat ═══►                                              │
                                                                                                                    ├──► map_merge ──► /map_merged
[ JETSON 2 — ausra_2 ]                                                                                             │
                                                                                                                    │
SLAM ──► /map ──► relay_node ──(throttle)──► /ausra_2/map_relay  ═══►  map_expansion_node ──► /ausra_2/map_fixed ──┘
                   relay_node (1Hz) ─────────► /ausra_2/heartbeat ═══►
```

---

## Step 0 — Copy `ausra_comms` to Both Jetsons (One Time)

From the laptop, SCP the **Jetson package only** (`ausra_comms`) to each Jetson:

```bash
# To Jetson 1:
scp -r ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms \
       user@<JETSON1_IP>:~/ausra_NM_ws/src/AUSRA-Autonomous-System/ausra_comms

# To Jetson 2:
scp -r ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms \
       user@<JETSON2_IP>:~/ausra_NM_ws/src/AUSRA-Autonomous-System/ausra_comms
```

> **Important:** Do NOT copy `ausra_comms_base`, `ausra_map_merge_HW`, or `m-explore-ros2` to the Jetsons. They are laptop-only packages.

---

## Step 1 — Build on All Machines

### On Jetson 1 (SSH in)

```bash
cd ~/ausra_NM_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ausra_comms
source install/setup.bash
```

### On Jetson 2 (SSH in)

```bash
cd ~/ausra_NM_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ausra_comms
source install/setup.bash
```

### On Laptop

```bash
cd ~/ausra_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ausra_comms_base ausra_map_merge_HW multirobot_map_merge
source install/setup.bash
```

---

## Step 2 — Get IP Addresses

On each machine, find the WiFi IP:

```bash
ip addr show wlan0 | grep "inet "
```

### Edit `start_base.sh` on the laptop

```bash
nano ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms_base/scripts/start_base.sh
```

Set `JETSON1_IP` and `JETSON2_IP` to the real IPs. Then rebuild:
```bash
cd ~/ausra_ws && colcon build --packages-select ausra_comms_base && source install/setup.bash
```

---

## Step 3 — Run (In Order)

### 3A — Jetson 1: Start Hardware + Comms

```bash
cd ~/ausra_NM_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1

ros2 launch ausra_comms hardware_with_comms.launch.py robot_name:=ausra_1
```

Wait until you see:
```
>>> Starting relay_node...
[relay_node]: Relay active → ausra_1 | map throttle 5.0s | compression=False
>>> Starting zenoh-bridge-ros2dds for ausra_1 ...
```

### 3B — Jetson 2: Start Hardware + Comms

```bash
cd ~/ausra_NM_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1

ros2 launch ausra_comms hardware_with_comms.launch.py robot_name:=ausra_2
```

### 3C — Laptop: Start Base Station

```bash
cd ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms_base/scripts
./start_base.sh
```

---

## Step 4 — Verify

Open a **new terminal** on the laptop:

```bash
source /opt/ros/humble/setup.bash
source ~/ausra_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1

# 1. Heartbeats from both Jetsons
ros2 topic echo /ausra_1/heartbeat --once
# Expected: "ausra_1 alive | maps=N"

ros2 topic echo /ausra_2/heartbeat --once
# Expected: "ausra_2 alive | maps=N"

# 2. List robot topics
ros2 topic list | grep ausra
# Expected:
#   /ausra_1/heartbeat
#   /ausra_1/map_relay
#   /ausra_1/map_fixed
#   /ausra_2/heartbeat
#   /ausra_2/map_relay
#   /ausra_2/map_fixed

# 3. Check merged map
ros2 topic echo /map_merged --no-arr --once
```

### RViz2 Setup

1. To see merged map: set **Fixed Frame** to `map`, add `/map_merged` → Map
2. To see individual robot maps: add `/ausra_1/map_relay` → Map, set **Fixed Frame** to the frame shown in that topic's header (e.g. `ausra_1/map` or `map`)

---

## Troubleshooting

### Heartbeat doesn't appear on Laptop

Check Zenoh bridge is running on both sides:
```bash
ps aux | grep zenoh
```

Check `ROS_LOCALHOST_ONLY=1` is set on all machines.

### Map doesn't appear in map_merge pipeline

1. Check relay_node is receiving maps: look for `Map relayed #N` in Jetson terminal
2. Check map_relay topic arrives on laptop: `ros2 topic hz /ausra_1/map_relay`
3. Check expansion node is subscribing: `ros2 topic info /ausra_1/map_relay -v`

### Robot doesn't move with micro-ROS after using Zenoh

**Kill leftover Zenoh processes** before switching back to plain hardware_full_stack:
```bash
pkill -f zenoh-bridge
unset ROS_LOCALHOST_ONLY
```

Zenoh bridge + `ROS_LOCALHOST_ONLY=1` left in the environment breaks micro-ROS agent DDS discovery.

---

## Files Summary

### What lives on the Jetsons (`ausra_comms`)

| File | Purpose |
|------|---------|
| `ausra_comms/relay_node.py` | Throttles map, publishes heartbeat |
| `launch/hardware_with_comms.launch.py` | **THE ONE YOU RUN** — hardware stack + relay + Zenoh bridge |
| `config/zenoh_bridge_jetson.json5` | Zenoh topic allowlist |

### What lives on the Laptop (`ausra_comms_base`)

| File | Purpose |
|------|---------|
| `ausra_comms_base/map_decompressor_node.py` | Decompresses maps (when compression enabled) |
| `ausra_comms_base/fake_robot_pub.py` | Dummy robot publisher (fallback for testing) |
| `launch/base_station.launch.py` | Zenoh bridge + decompressor + map merge + RViz2 |
| `launch/map_merge.launch.py` | Map expansion nodes + merge engine |
| `config/map_merge_swarm_params.yaml` | Merge params |
| `config/zenoh_bridge_laptop.json5` | Zenoh topic allowlist |
| `scripts/start_base.sh` | Convenience script — ping + launch |

### Also needed on the Laptop (separate packages)

| Package | Purpose |
|---------|---------|
| `ausra_map_merge_HW` | Provides `map_expansion_node` |
| `m-explore-ros2` → `multirobot_map_merge` | Provides the merge engine |
