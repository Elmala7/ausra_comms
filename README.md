# AUSRA: Autonomous Unified Swarm Robotics Architecture

This repository contains the complete ROS 2 multi-robot communication, navigation, and map-merging stack for the **AUSRA** swarm system.

The system is designed with a hybrid high-level/low-level architecture. Each robot (e.g., **Jetson Orin Nano**) runs ROS 2 Humble for SLAM and local navigation, while an **ESP32-S3** runs micro-ROS for real-time motor control and hardware interfacing.

---

## 📡 Network & Communication Layer (DDS + Zenoh)

To handle challenging, bandwidth-constrained wireless environments in swarm robotics, the communication layer uses a hybrid **DDS + Zenoh** architecture:
* **Local DDS Isolation**: Each robot and the base station runs ROS 2 with `ROS_LOCALHOST_ONLY=1`. High-bandwidth internal topics (like raw `/scan`, `/odom`, and `/tf` trees) are restricted to local loopback to avoid flooding the WiFi network.
* **Zenoh Bridging**: A `zenoh-bridge-ros2dds` daemon runs on each Jetson and the laptop to tunnel only allowlisted, low-bandwidth topics across the WiFi network.
* **Map Compression**: A custom Python relay node compresses occupancy grids using `zlib` (`enable_compression:=true`), reducing transmitted map sizes from ~1MB down to ~1KB (a >99% reduction).
* **Domain Segregation**: Swarm communication is bound to `ROS_DOMAIN_ID=0`.
* **Zero-Configuration Time Sync**: Because Zenoh is highly sensitive to clock skew, a custom time synchronization script (`sync_time.sh`) is used to sync Jetson clocks to the laptop's clock before every session (since the Jetsons lack RTC battery backups).

---

## 📁 Repository Structure

```
AUSRA-Autonomous-System-hardware_with_nav2/
├── ausra_comms/               # [Jetson] Throttles/compresses maps, publishes heartbeat
│   └── launch/
│       ├── hardware_with_comms.launch.py   # Main robot launch (SLAM + Hardware + Comms)
│       └── decentralized_robot.launch.py   # Launch configuration for decentralized swarm nodes
├── ausra_comms_base/          # [Laptop] Map merge, decompressor, RViz, scripts, docs
│   ├── launch/
│   │   ├── base_station.launch.py          # Full laptop stack launch (merger + decompressor)
│   │   └── map_merge.launch.py             # Canvas expansion and merge node
│   ├── scripts/
│   │   └── start_base.sh                   # Wrapper script (ping check + base launch)
│   └── docs/                               # Detailed design & setup documentation
│       ├── DEPLOYMENT_2JETSONS.md          # 2-Jetson + Laptop step-by-step runbook
│       ├── DEPLOYMENT_DECENTRALIZED.md     # Decentrailzed map merging guides
│       ├── SYSTEM_ARCHITECTURE.md          # Swarm data flows and namespaces
│       ├── COMMUNICATION_ARCHITECTURE.md   # Hybrid comms approach & network design
│       └── FUTURE_WORK.md                  # Roadmap for decentralized merging
├── ausra_map_merge_HW/        # [Laptop] Stamps local maps onto fixed-size canvases
├── m-explore-ros2/            # [Laptop] Multi-robot map merging engine (m-explore)
├── docs/                      # Global documentation folder
│   └── TIME_SYNC_GUIDE.md     # Setup and operational guide for clock synchronization
└── HARDWARE_NAVIGATION_SETUP.md # Original hardware/navigation guide
```

---

## 🚀 Quick Start Guide

### 1. Sync Jetson Clocks (Per-Session Ritual)
Since Jetson Orin Nanos have no RTC battery, you must sync their clocks to the laptop's time before launching ROS/Zenoh:
```bash
# On the laptop
cd ~
./sync_time.sh
```
For the one-time sudoers configuration setup, refer to [Time Sync Guide](docs/TIME_SYNC_GUIDE.md).

### 2. Run on the Robots (Jetsons)
SSH into each Jetson, source your workspace, and launch the decentralized robot stack:

```bash
# Terminal on Jetson 1 (ausra_1)
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
ros2 launch ausra_comms decentralized_robot.launch.py robot_name:=ausra_1 robot_config:="ausra_1:0.0:0.0 ausra_2:0.0:1.2"

# Terminal on Jetson 2 (ausra_2)
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
ros2 launch ausra_comms decentralized_robot.launch.py robot_name:=ausra_2 robot_config:="ausra_1:0.0:0.0 ausra_2:0.0:1.2"
```

### 3. Run on the Base Station (Laptop)
Ensure you are connected to the same WiFi network, source your workspace, and launch the base station using the automated script:
```bash
cd ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms_base/scripts
./start_base.sh robot_config:="ausra_1:0.0:0.0 ausra_2:0.0:1.2"
```
Or launch directly:
```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
ros2 launch ausra_comms_base base_station.launch.py
```

---

## 🛠️ Detailed Documentation

Deep-dive documentation is located in the `docs/` and `ausra_comms_base/docs/` directories:
* 📖 [Time Sync Guide (Jetson Clocks without Internet)](docs/TIME_SYNC_GUIDE.md)
* 📖 [Zenoh Integration Guide (Transport Setup)](ausra_comms_base/docs/ZENOH_GUIDE.md)
* 📖 [Step-by-step Deployment Guide (2 Jetsons + Laptop)](ausra_comms_base/docs/DEPLOYMENT_2JETSONS.md)
* 📖 [Decentralized Deployment Guide](ausra_comms_base/docs/DEPLOYMENT_DECENTRALIZED.md)
* 📖 [System Architecture & Namespace Data Flows](ausra_comms_base/docs/SYSTEM_ARCHITECTURE.md)
* 📖 [Communication Architecture & Network Tuning](ausra_comms_base/docs/COMMUNICATION_ARCHITECTURE.md)
* 📖 [Swarm Roadmap & Decentralized Merging Plan](ausra_comms_base/docs/FUTURE_WORK.md)

---

## 💻 Sharing via GitHub

To upload this workspace to GitHub so your team can access it:

1. **Create a new repository** on GitHub named `ausra_comms`.
2. Run the following commands in the root of this repository to link and push:
   ```bash
   # Add your GitHub remote origin
   git remote add origin https://github.com/Elmala7/ausra_comms.git

   # Push files to main branch
   git push -u origin main
   ```