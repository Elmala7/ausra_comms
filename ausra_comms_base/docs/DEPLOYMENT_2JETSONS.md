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
[ JETSON 1 — ausra_1 ]                           [ WiFi / DDS ]      [ LAPTOP — Base Station ]

SLAM ──► /map ──► relay_node ──(throttle)──► /ausra_1/map  ════►  map_expansion_node ──► /ausra_1/map_fixed ──┐
     ──► /pose ──► relay_node ─────────────► /ausra_1/pose ════►                                               │
                    relay_node (1Hz) ────────► /ausra_1/hb  ════►                                               ├──► map_merge ──► /map_merged
                                                                                                                │
[ JETSON 2 — ausra_2 ]                                                                                         │
                                                                                                                │
SLAM ──► /map ──► relay_node ──(throttle)──► /ausra_2/map  ════►  map_expansion_node ──► /ausra_2/map_fixed ──┤
     ──► /pose ──► relay_node ─────────────► /ausra_2/pose ════►                                               │
                    relay_node (1Hz) ────────► /ausra_2/hb  ════►                                               │
                                                                                                                │
                                                                   phantom_node ──► /ausra_99/map_fixed ────────┘
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
colcon build --packages-select ausra_comms ausra_comms_base ausra_map_merge_HW multirobot_map_merge
source install/setup.bash
```

---

## Step 2 — Get IP Addresses

On each machine, find the WiFi IP:

```bash
ip addr show wlan0 | grep "inet "
```

| Machine | Example IP |
|---------|-----------|
| Jetson 1 | `192.168.1.33` |
| Jetson 2 | `192.168.1.XX` (find tomorrow) |
| Laptop | `192.168.1.YY` (find tomorrow) |

### Edit `start_base.sh` on the laptop

```bash
nano ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms_base/scripts/start_base.sh
```

Set `JETSON1_IP` and `JETSON2_IP` to the real IPs.

Then rebuild:
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
export ROS_LOCALHOST_ONLY=1   # Zenoh is the only cross-WiFi channel

ros2 launch ausra_comms hardware_with_comms.launch.py robot_name:=ausra_1
```

> **Note:** `ROS_LOCALHOST_ONLY=1` is required when `use_zenoh:=true` (the
> default). It pins DDS to loopback so only Zenoh-allowlisted topics cross
> WiFi. See [`ZENOH_GUIDE.md`](../../ZENOH_GUIDE.md) for full details and
> the revert path.

Wait until you see:
```
>>> Starting relay_node (AUSRA comms layer)...
[relay_node]: Relay active → ausra_1 | map throttle every 5.0s
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

Wait until you see:
```
>>> Starting relay_node (AUSRA comms layer)...
[relay_node]: Relay active → ausra_2 | map throttle every 5.0s
```

### 3C — Laptop: Start Base Station

```bash
cd ~/ausra_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1

cd src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms_base/scripts
./start_base.sh
```

`start_base.sh` sets `ROS_LOCALHOST_ONLY` itself based on `USE_ZENOH` (defaults to `true`); the explicit export above is for any terminal you spawn alongside it.

> **Alternative:** If only 1 Jetson is available, edit `start_base.sh` and uncomment the fake publisher section for the missing robot.

---

## Step 4 — Verify

Open a **new terminal** on the laptop:

```bash
source /opt/ros/humble/setup.bash
source ~/ausra_ws/install/setup.bash
export ROS_DOMAIN_ID=0

# 1. Heartbeats from both Jetsons
ros2 topic echo /ausra_1/heartbeat --once
# Expected: "ausra_1 alive"

ros2 topic echo /ausra_2/heartbeat --once
# Expected: "ausra_2 alive"

# 2. List all robot topics
ros2 topic list | grep ausra
# Expected:
#   /ausra_1/heartbeat   ← real Jetson 1 via DDS
#   /ausra_1/map         ← real SLAM via DDS
#   /ausra_1/pose        ← real pose via DDS
#   /ausra_2/heartbeat   ← real Jetson 2 via DDS
#   /ausra_2/map         ← real SLAM via DDS
#   /ausra_2/pose        ← real pose via DDS

# 3. Check map_fixed topics (from expansion nodes)
ros2 topic list | grep map_fixed
# Expected:
#   /ausra_1/map_fixed
#   /ausra_2/map_fixed
#   /ausra_99/map_fixed  (phantom)

# 4. Check merged map
ros2 topic echo /map_merged --no-arr --once
# Expected: OccupancyGrid with frame_id='map'
```

### RViz2 Setup

1. Set **Fixed Frame** to `map`
2. **Add → By topic → /map_merged → Map**
3. **Add → By topic → /ausra_1/map → Map** (set Color Scheme to `costmap`, Alpha to 0.5)
4. **Add → By topic → /ausra_2/map → Map** (set Alpha to 0.3)

---

## Troubleshooting

### Heartbeat doesn't appear on Laptop

DDS multicast discovery is failing. Check on **all 3 machines**:

```bash
echo $ROS_DOMAIN_ID      # Must be 0
echo $ROS_LOCALHOST_ONLY  # Must be 0
```

Ping test from laptop:
```bash
ping <JETSON1_IP>   # Must succeed
ping <JETSON2_IP>   # Must succeed
```

If ping works but topics don't appear:
- Your router may be blocking multicast. Try a **mobile hotspot** instead.
- Check firewall: `sudo ufw status` — should be inactive or allow UDP.
- Try setting `export FASTRTPS_DEFAULT_PROFILES_FILE=""` to clear any FastDDS overrides.

### `/ausra_X/map` topic exists but no data

QoS mismatch. Check on the Jetson:
```bash
ros2 topic info /ausra_1/map -v
```
Should show `Reliability: Reliable` and `Durability: Transient local`.

### `map_expansion_node` shows RESOLUTION MISMATCH

Your SLAM config uses `resolution: 0.05` — this must match `CANVAS_RESOLUTION` in `map_merge.launch.py` (currently `0.05`).

### `/map_merged` not appearing

- Needs at least 2 maps discovered. Check that both `/ausra_1/map_fixed` and `/ausra_2/map_fixed` topics exist.
- Check map_merge logs for: `adding robot [/ausra_1] to system`
- If you see `Couldn't get initial position for robot [/ausra_1]` → check `map_merge_swarm_params.yaml`.

### Only 1 Jetson available

Edit `start_base.sh` and uncomment the fake publisher section. This launches a `fake_robot_pub` to simulate the missing robot.

---

## Files Summary

### What lives on the Jetsons (`ausra_comms`)

| File | Purpose |
|------|---------|
| `ausra_comms/relay_node.py` | Throttles map, relays pose, publishes heartbeat |
| `launch/hardware_with_comms.launch.py` | **THE ONE YOU RUN** — hardware stack + relay_node |

### What lives on the Laptop (`ausra_comms_base`)

| File | Purpose |
|------|---------|
| `ausra_comms_base/fake_robot_pub.py` | Dummy robot publisher (fallback for testing) |
| `launch/map_merge.launch.py` | Map expansion nodes + merge engine |
| `launch/base_station.launch.py` | Map merge + RViz2 |
| `config/map_merge_swarm_params.yaml` | Merge params (init poses, namespace config) |
| `scripts/start_base.sh` | Convenience script — ping + merge + RViz2 |

### Also needed on the Laptop (separate packages)

| Package | Purpose |
|---------|---------|
| `ausra_map_merge_HW` | Provides `map_expansion_node` |
| `m-explore-ros2` → `multirobot_map_merge` | Provides the merge engine |
