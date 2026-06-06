# AUSRA: Autonomous Unified Swarm Robotics Architecture

This repository contains the complete ROS 2 multi-robot communication, navigation, and map-merging stack for the **AUSRA** swarm system.

The system is designed with a hybrid high-level/low-level architecture. Each robot (e.g., **Jetson Orin Nano**) runs ROS 2 Humble for SLAM and local navigation, while an **ESP32-S3** runs micro-ROS for real-time motor control and hardware interfacing.

---

## 📡 Network & Discovery (DDS)

The communication layer uses native ROS 2 **DDS (Data Distribution Service)** with UDP multicast.
* **No Hardcoded IPs**: Robots find each other and the base station automatically over WiFi.
* **Zero Master Node**: There is no central ROS master. The connection is peer-to-peer.
* **Domain Segregation**: Swarm communication is bound to `ROS_DOMAIN_ID=0`.

---

## 📁 Repository Structure

```
AUSRA-Autonomous-System-hardware_with_nav2/
├── ausra_comms/               # [Jetson] Throttles map/pose, publishes heartbeat
│   └── launch/
│       └── hardware_with_comms.launch.py   # Main robot launch (SLAM + Hardware + Comms)
├── ausra_comms_base/          # [Laptop] Map merge, RViz, scripts, docs
│   ├── launch/
│   │   ├── base_station.launch.py          # Full laptop stack launch
│   │   └── map_merge.launch.py             # Canvas expansion and merge node
│   ├── scripts/
│   │   └── start_base.sh                   # Wrapper script (ping check + base launch)
│   └── docs/                               # Detailed design & setup documentation
│       ├── DEPLOYMENT_2JETSONS.md          # 2-Jetson + Laptop step-by-step runbook
│       ├── SYSTEM_ARCHITECTURE.md          # Swarm data flows and namespaces
│       ├── COMMUNICATION_ARCHITECTURE.md   # Hybrid comms approach & network design
│       └── FUTURE_WORK.md                  # Roadmap for decentralized merging
├── ausra_map_merge_HW/        # [Laptop] Stamps local maps onto fixed-size canvases
├── m-explore-ros2/            # [Laptop] Multi-robot map merging engine (m-explore)
└── HARDWARE_NAVIGATION_SETUP.md # Original hardware/navigation guide
```

---

## 🚀 Quick Start Guide

### 1. Run on the Robots (Jetsons)
SSH into each Jetson, source your workspace, set the domain environment, and run the hardware + comms stack:

```bash
# Terminal on Jetson 1 (ausra_1)
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch ausra_comms hardware_with_comms.launch.py robot_name:=ausra_1

# Terminal on Jetson 2 (ausra_2)
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch ausra_comms hardware_with_comms.launch.py robot_name:=ausra_2
```

### 2. Run on the Base Station (Laptop)
Ensure you are connected to the same WiFi network, source your workspace, and launch the base station:

**Option A (Using the automated script with network ping checks):**
```bash
cd ~/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms_base/scripts
./start_base.sh
```

**Option B (Direct launch):**
```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch ausra_comms_base base_station.launch.py
```

---

## 🛠️ Detailed Documentation

Deep-dive documentation is located in the `ausra_comms_base/docs/` directory:
* 📖 [Step-by-step Deployment Guide (2 Jetsons + Laptop)](ausra_comms_base/docs/DEPLOYMENT_2JETSONS.md)
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