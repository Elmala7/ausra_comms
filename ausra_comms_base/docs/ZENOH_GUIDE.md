# ZENOH_GUIDE.md

Cross-WiFi transport for the AUSRA swarm using `zenoh-bridge-ros2dds`.

This guide replaces the original "all DDS over WiFi" model. After applying this change, **Zenoh is the only channel that crosses WiFi**. DDS is pinned to loopback on every machine via `ROS_LOCALHOST_ONLY=1`. Only an explicit allowlist of topics is bridged: `/ausra_*/map_compressed` and `/ausra_*/heartbeat` (the default compressed map path). `/ausra_*/map` is only bridged when `enable_compression:=false`.

---

## Why Zenoh

The previous setup ran DDS multicast over WiFi. Even though `relay_node` only publishes three topics intended for cross-WiFi use, DDS itself doesn't know about that intent — it gossips discovery for every node, every topic, on every machine in the same `ROS_DOMAIN_ID`. With slam_toolbox + Nav2 + lifecycle nodes on each Jetson, that's hundreds of endpoints announcing themselves. Routers throttle multicast under load, and the discovery storm starves the actual map and pose messages. Symptom: `/map` and `/pose` arrive late or are dropped; Nav2 reports disconnects.

`zenoh-bridge-ros2dds` solves this by:
- Replacing cross-machine DDS with a single Zenoh session per machine.
- Bridging only an explicit allowlist of topics (the rest stay local).
- Preserving QoS (so `transient_local + reliable` for `/map` survives end-to-end).
- Reconnecting cleanly when WiFi blips.

The on-Jetson DDS still runs (loopback only) so SLAM, Nav2, the costmaps, TF, scans — everything you don't want on WiFi — keeps working at full rate locally.

---

## Decisions baked into this setup

These were chosen on your behalf when you said "do the right thing." Override anywhere noted.

| Decision | Value | Where to change |
|---|---|---|
| Bridge listening port | `tcp/7447` (Zenoh default) | `*.json5` `listen.endpoints` |
| Bridge install path | `/opt/zenoh-bridge/zenoh-bridge-ros2dds` | env var `ZENOH_BRIDGE_BIN` (per launch) |
| Per-robot Zenoh namespace | derived from `robot_name` launch arg (`/ausra_1`, `/ausra_2`) | already automatic |
| Bridge mode | peer mesh on all 3 machines | `*.json5` `mode` |
| Allowlist | `/ausra_*/map_compressed`, `/ausra_*/heartbeat` (Jetson also publishes; laptop subscribes). `/ausra_*/map` only when `enable_compression:=false` | `*.json5` `plugins.ros2dds.allow.*` |
| DDS scope | loopback only when `use_zenoh:=true` (the default) | env `ROS_LOCALHOST_ONLY` |
| Bridge crash behavior | respawn with 2s delay; relay/SLAM/Nav2 unaffected | `respawn=True` in launch files |

**Pinned bridge version: `zenoh-plugin-ros2dds v1.2.1`** (latest stable Humble-compatible release at time of writing). Bump by editing the URL in the install step below.

---

## 1. Install (do this on each machine — one time)

You need internet for this step. Do it on each Jetson and the laptop while you still have internet access. After that, the binary lives on disk; deployment can be offline.
<!-- ZENOH_GUIDE_PLACEHOLDER -->

### 1.1 Pick the right binary

Both Jetsons run **JetPack 6.x** (Ubuntu 22.04, arm64). The laptop is x86_64 Ubuntu.

| Machine | Architecture | Tarball |
|---|---|---|
| Jetson 1, Jetson 2 | `aarch64` | `zenoh-plugin-ros2dds-1.2.1-aarch64-unknown-linux-gnu-standalone.zip` |
| Laptop | `x86_64` | `zenoh-plugin-ros2dds-1.2.1-x86_64-unknown-linux-gnu-standalone.zip` |

Releases live at `https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases`. Pick the tag `1.2.1`.

### 1.2 Download + install (run on each machine)

```bash
ZENOH_VER="1.2.1"
ARCH="$(uname -m)"   # aarch64 on Jetson, x86_64 on laptop
TARBALL="zenoh-plugin-ros2dds-${ZENOH_VER}-${ARCH}-unknown-linux-gnu-standalone.zip"
URL="https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_VER}/${TARBALL}"

cd /tmp
curl -LO "${URL}"

sudo mkdir -p /opt/zenoh-bridge
sudo unzip -o "${TARBALL}" -d /opt/zenoh-bridge
sudo chmod +x /opt/zenoh-bridge/zenoh-bridge-ros2dds

# Verify
/opt/zenoh-bridge/zenoh-bridge-ros2dds --version
```

You should see `zenoh-bridge-ros2dds v1.2.1` (or whatever you pinned).

### 1.3 Optional: put a non-default install location somewhere else

If `/opt` is tight or requires sudo you don't want, install anywhere and point at it via env var before launching:

```bash
export ZENOH_BRIDGE_BIN=$HOME/zenoh-bridge/zenoh-bridge-ros2dds
```

Both `hardware_with_comms.launch.py` and `base_station.launch.py` honor this env var.

### 1.4 Step-by-Step Installation on Laptop (x86_64)

To install the Zenoh bridge on your laptop:

1. Open a terminal and download the `x86_64` standalone bridge release:
   ```bash
   cd /tmp
   curl -LO https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/1.2.1/zenoh-plugin-ros2dds-1.2.1-x86_64-unknown-linux-gnu-standalone.zip
   ```
2. Create the installation directory and extract the binary:
   ```bash
   sudo mkdir -p /opt/zenoh-bridge
   sudo unzip -o zenoh-plugin-ros2dds-1.2.1-x86_64-unknown-linux-gnu-standalone.zip -d /opt/zenoh-bridge
   sudo chmod +x /opt/zenoh-bridge/zenoh-bridge-ros2dds
   ```
3. Test that the binary is installed correctly and check its version:
   ```bash
   /opt/zenoh-bridge/zenoh-bridge-ros2dds --version
   ```
   *Expected Output:*
   ```text
   zenoh-bridge-ros2dds v1.2.1
   ```

---

## 2. Run the full system after this change

Three machines, three terminals. Order matters: start the Jetsons first so they're ready when the laptop's bridge comes up.

### 2.1 Jetson 1 (`ausra_1`)

```bash
cd ~/ausra_NM_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1     # <-- new, mandatory with Zenoh

ros2 launch ausra_comms decentralized_robot.launch.py robot_name:=ausra_1 robot_config:="ausra_1:0.0:0.0 ausra_2:0.0:1.2" nudge_robot:=true
```

### 2.3 Laptop (base station)

To start the base station with the default robot configurations (`ausra_1` and `ausra_2` at `0.0:0.0` offsets):

```bash
cd ~/ausra_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

cd src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms_base/scripts
./start_base.sh robot_config:="ausra_1:0.0:0.0 ausra_2:0.0:1.2"
```
#### To Kill Zenoh

```bash
ssh ausranano@192.168.0.129 'pkill -f zenoh-bridge'
ssh ausranano@192.168.0.165 'pkill -f zenoh-bridge'
# then relaunch the stack on each Jetson
```

#### Specifying Robot Configurations & Namespaces
If you need to change the namespaces or initial poses of the robots to merge, you can pass the `robot_config` argument directly to the script:

```bash
./start_base.sh robot_config:="ausra_1:0.0:0.0 ausra_2:-3.0:4.8"
```

The script automatically sets `ROS_LOCALHOST_ONLY=1` (when using Zenoh), pings the Jetson IPs to verify network connection, and forwards any CLI arguments to the underlying launch file.

If you prefer to call the launch file directly:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
ros2 launch ausra_comms_base base_station.launch.py robot_config:="ausra_1:0.0:0.0 ausra_2:1.5:-2.0"
```

---

## 3. How Zenoh is configured

Two JSON5 files, one per machine class. Both are installed by colcon into `share/<pkg>/config/`.

| File | Used by | Purpose |
|---|---|---|
| `ausra_comms/config/zenoh_bridge_jetson.json5` | each Jetson | Publishes `/ausra_X/{map,pose,heartbeat}` over Zenoh; subscribes to peers' equivalents |
| `ausra_comms_base/config/zenoh_bridge_laptop.json5` | laptop | Subscribes to all `/ausra_*/{map,pose,heartbeat}`; publishes nothing across WiFi by default |

The `/ausra_X/` namespace on each Jetson is set at launch via a `-n /ausra_X` CLI override, derived from the `robot_name` launch arg. So a single JSON5 file works for `ausra_1`, `ausra_2`, etc. — you don't fork it per robot.

### Key config sections (Jetson side)

```json5
{
  plugins: { ros2dds: {
    domain: 0,
    namespace: "/ausra_X_OVERRIDE_AT_LAUNCH",   // overridden by -n
    allow: {
      publishers:  ["/ausra_.*/map", "/ausra_.*/map_compressed", "/ausra_.*/heartbeat"],
      subscribers: ["/ausra_.*/map", "/ausra_.*/map_compressed", "/ausra_.*/heartbeat"],
      // services & actions explicitly empty — pub/sub only across WiFi
    },
    reliable_routes_blocking: true,    // preserves /map's reliable+transient_local QoS
  }},
  mode: "peer",
  listen:  { endpoints: ["tcp/0.0.0.0:7447"] },
  scouting: {
    multicast: { enabled: true, address: "224.0.0.224:7446", interface: "auto" },
    gossip:    { enabled: true },
  },
}
```

Peer discovery uses Zenoh's own scouting multicast on `224.0.0.224:7446` — **not** the DDS multicast group. It's one cheap packet to find peers, not a topic-discovery storm. If your router blocks multicast, switch to static peers (see §6).

---

## 4. Verify only `/map`, `/map_compressed`, `/heartbeat` cross WiFi

Three checks: name list, traffic capture, QoS sanity.

### 4.1 Topic list looks right on the laptop

```bash
source /opt/ros/humble/setup.bash && source ~/ausra_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1

ros2 topic list | grep -E '/ausra_[12]/'
```

Expected — exactly these and nothing else:
```
/ausra_1/heartbeat
/ausra_1/map
/ausra_1/map_compressed
/ausra_2/heartbeat
/ausra_2/map
/ausra_2/map_compressed
```

If you see `/ausra_1/scan`, `/ausra_1/odom`, or `/ausra_1/tf` in the list, the allowlist is leaking. Check that the Zenoh config installed correctly and that you actually launched with `use_zenoh:=true`.

### 4.2 Traffic capture on the WiFi interface

The strict check. Run on the laptop while the system is up:

```bash
# Find your WiFi iface name (wlan0, wlp3s0, ...)
ip -br addr show

IFACE=wlan0   # adjust

# Capture only traffic to/from a Jetson on the Zenoh port
sudo tcpdump -nni "$IFACE" "host 192.168.1.3 and tcp port 7447" -c 200
```

Expected: a steady stream of packets on TCP port 7447 (Zenoh) and **nothing else** between the laptop and Jetsons (other than whatever else is on the network — SSH, ping). If you see any of these, there's a leak:

| If you see... | Diagnosis |
|---|---|
| UDP `7400`–`7500` traffic | DDS is escaping. `ROS_LOCALHOST_ONLY=1` not set on one of the machines. |
| UDP multicast `239.255.0.x` | Same as above — DDS discovery on WiFi. |
| TCP traffic on ports other than `7447` between robot↔laptop | Another service (or wrong port in JSON5). |

Counterpart sanity check on the Jetson:

```bash
# On the Jetson, while running:
ss -tnlp | grep 7447     # bridge listening on tcp/7447
ss -tn   | grep 7447     # established peer connections to other machines
```

### 4.3 QoS preservation on `/ausra_*/map`

Critical because `transient_local + reliable` is load-bearing in this stack:

```bash
ros2 topic info /ausra_1/map -v
```

Both publisher (Zenoh bridge endpoint on the laptop) and subscriber (map_expansion_node) must show:
```
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (1) [or 5 — depth doesn't matter, kind does]
  Durability: TRANSIENT_LOCAL
```

If durability shows `VOLATILE`, map_expansion_node will silently fail to receive maps. The bridge auto-detects QoS from the publisher; if it fails, force it explicitly per topic in JSON5 (see Zenoh docs for `pub_max_frequencies` / `transient_local_cache`).

---

## 5. Adjust which topics cross WiFi

The allowlist is in **two** JSON5 files (Jetson + laptop) — they must stay in sync. To add a topic:

1. Add the regex to **both** files, in `plugins.ros2dds.allow.publishers` and/or `subscribers` depending on direction. Examples already commented in both files for `/ausra_*/active_target`, `/ausra_*/swarm_obstacles_scan`, and `/map_merged`.

2. Re-build and re-source on each machine that hosts the file you changed:
   ```bash
   cd ~/ausra_ws && colcon build --packages-select ausra_comms_base   # laptop
   cd ~/ausra_NM_ws && colcon build --packages-select ausra_comms     # each Jetson
   source install/setup.bash
   ```

3. Restart the bridge (or the whole launch). The bridge reads JSON5 once at startup; there's no live reload.

### Naming conventions to keep in mind

- The Jetson side prefixes everything with the per-robot namespace (`/ausra_1/...`). If you publish locally on `/foo`, the bridge re-keys it as `/ausra_1/foo` before sending. Receivers see it under `/ausra_1/foo`.
- Use **regex**, not glob. `"/ausra_.*/map"` matches `/ausra_1/map` and `/ausra_2/map`. `"/ausra_*/map"` does **not** — that's a literal asterisk.
- Service & action allowlists are deliberately empty. Cross-WiFi RPC is out of scope; if you ever need it, add specific entries (don't open a wildcard).

### Bandwidth budgeting

Every entry costs WiFi. Rule of thumb:
- Heartbeat-class (1 Hz, <100 B): free, add freely.
- Pose-class (~10 Hz, <1 KB): cheap, add when needed.
- Map-class (occasional, ~1 MB): expensive, throttle on the publisher side first (like `relay_node` does for `/map`).
- Scan/TF/odom: don't.

---

## 6. Troubleshooting

### `zenoh-bridge-ros2dds: not found`

Either the binary isn't installed, isn't at `/opt/zenoh-bridge/zenoh-bridge-ros2dds`, or isn't executable. See §1.

```bash
ls -la /opt/zenoh-bridge/zenoh-bridge-ros2dds
file /opt/zenoh-bridge/zenoh-bridge-ros2dds
```

### Bridge starts but peers don't see each other

Most likely: router blocks multicast. Switch to static peers. In **all three** JSON5 files, comment out the `scouting.multicast` block and uncomment the `connect.endpoints` section with the real WiFi IPs:

```json5
connect: {
  endpoints: [
    "tcp/192.168.1.3:7447",   // Jetson 1
    "tcp/192.168.1.4:7447",   // Jetson 2
    "tcp/192.168.1.5:7447",   // Laptop
  ],
},
```

A bridge connects to itself harmlessly — the same list can go in all three files.

### Topic appears but `ros2 topic echo` shows nothing

QoS mismatch. See §4.3. The map case is the common one.

### Bridge dies and the robot stops responding

It shouldn't — `respawn=True` is set, and the relay/SLAM/Nav2 are decoupled from the bridge process tree. If a bridge crash takes down the whole launch, check that you didn't change `respawn` to `False` somewhere.

### `start_base.sh` complains about Jetson IPs

Edit `JETSON1_IP` / `JETSON2_IP` near the top of the script with the real WiFi addresses (find with `ip addr show` on each Jetson). The ping check is for connectivity sanity; you can answer `y` to skip it during testing.

---

## 7. Revert (if Zenoh causes more pain than it solves)

The change is fully reversible — every Zenoh-aware part is gated on the `use_zenoh` launch arg, and the relay/map_expansion/merge pipeline does not depend on Zenoh.

### 7.1 Disable per-launch (temporary)

```bash
# Jetson
ros2 launch ausra_comms hardware_with_comms.launch.py robot_name:=ausra_1 use_zenoh:=false

# Laptop
USE_ZENOH=false ./start_base.sh
# OR
ros2 launch ausra_comms_base base_station.launch.py use_zenoh:=false
```

You also need to unset `ROS_LOCALHOST_ONLY` so DDS multicast can cross WiFi again:

```bash
unset ROS_LOCALHOST_ONLY
```

`start_base.sh` does this automatically when `USE_ZENOH=false`.

### 7.2 Disable permanently (revert in source)

Two options:

**(a) Revert the relevant commits.** All Zenoh-related changes are isolated to:
- `ausra_comms/launch/hardware_with_comms.launch.py` (Stage C block + `use_zenoh` arg)
- `ausra_comms/config/zenoh_bridge_jetson.json5` (delete)
- `ausra_comms/setup.py` (json5 install line)
- `ausra_comms_base/launch/base_station.launch.py` (Zenoh ExecuteProcess)
- `ausra_comms_base/config/zenoh_bridge_laptop.json5` (delete)
- `ausra_comms_base/setup.py` (json5 install line)
- `ausra_comms_base/scripts/start_base.sh` (USE_ZENOH block + base_station.launch.py invocation)
- `ausra_comms_base/docs/DEPLOYMENT_2JETSONS.md` (LOCALHOST_ONLY values)

`git revert` on the commit that introduces them is the clean way.

**(b) Flip the default.** Change `default_value='true'` → `default_value='false'` for the `use_zenoh` arg in both launch files, and change `USE_ZENOH:-true` → `USE_ZENOH:-false` in `start_base.sh`. The Zenoh code stays in tree but is dormant.

### 7.3 What you go back to

DDS multicast over WiFi, `ROS_LOCALHOST_ONLY=0`, all the original symptoms (flooded discovery, late/dropped maps under load). The relay's application-layer throttle still helps but doesn't solve the discovery storm. If you find yourself reverting, the next thing to try is **FastDDS Discovery Server** (see `ausra_comms_base/docs/FUTURE_WORK.md` §4 DDS Tuning) — it kills the multicast storm without the operational complexity of Zenoh.

---

## Appendix: How this layer relates to the rest of the stack

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       ON-JETSON (loopback DDS only)                      │
│                                                                          │
│  slam_toolbox ──► /map ──┐                                              │
│  Nav2, EKF, costmaps     │                                              │
│  /scan /tf /odom etc.    │                                              │
│                          ▼                                              │
│                     relay_node ──► /ausra_1/map (throttled)        │
│                                ──► /ausra_1/map_compressed (optional)   │
│                                ──► /ausra_1/heartbeat                   │
│                                            │                             │
│                                            ▼                             │
│                              zenoh-bridge-ros2dds (allowlist)            │
└────────────────────────────────────│─────────────────────────────────────┘
                                     │ tcp/7447 (Zenoh, peer mesh)
                                     │ ── ONLY allowlisted topics ──
                                     ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                       ON-LAPTOP (loopback DDS only)                      │
│                                                                          │
│   zenoh-bridge-ros2dds (allowlist) ──► /ausra_1/map (transient)   │
│                                    ──► /ausra_1/map_compressed          │
│                                    ──► /ausra_1/heartbeat               │
│                                            │                             │
│                                            ▼                             │
│                            map_expansion_node ──► /ausra_1/map_fixed     │
│                            multirobot_map_merge ──► /map_merged          │
│                            RViz2                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

What changed in the architecture:
- `relay_node`: **unchanged.** Still does namespacing + throttling. Still useful — Zenoh doesn't throttle.
- DDS scope: **loopback only.** Each machine's DDS no longer crosses WiFi.
- New cross-WiFi transport: **Zenoh,** with explicit allowlist.
- QoS contract on `/map` (transient_local + reliable): **preserved end-to-end.**
- Phantom robot, spawn-offset invariant, frame names: **unchanged** — all downstream of the bridge.

For the original design rationale (why three topics, why throttled, etc.) see `ausra_comms_base/docs/COMMUNICATION_ARCHITECTURE.md`. For deployment runbook see `ausra_comms_base/docs/DEPLOYMENT_2JETSONS.md`.

---

## 9. Quick Reference Cheat Sheet

### 9.1 Running Jetson with Zenoh
To run a Jetson (e.g. `ausra_1`) with Zenoh enabled:
1. Ensure loopback mode is active in the terminal:
   ```bash
   export ROS_LOCALHOST_ONLY=1
   export ROS_DOMAIN_ID=0
   ```
2. Launch the hardware with comms (Zenoh is enabled by default):
   ```bash
   ros2 launch ausra_comms hardware_with_comms.launch.py robot_name:=ausra_1 use_zenoh:=true
   ```

### 9.2 Running Laptop (Base Station) with Zenoh
To launch the Laptop Base Station with Zenoh:
1. Ensure loopback mode is active in the terminal:
   ```bash
   export ROS_LOCALHOST_ONLY=1
   export ROS_DOMAIN_ID=0
   ```
2. Run the startup script or launch file directly:
   ```bash
   USE_ZENOH=true ./start_base.sh
   # OR
   ros2 launch ausra_comms_base base_station.launch.py use_zenoh:=true
   ```

### 9.3 Fully Terminating & Running WITHOUT Zenoh (e.g., direct DDS over WiFi)
If you want to test without Zenoh (running raw DDS over WiFi, e.g., for hardware full stack):
1. **Kill all Zenoh bridge processes** on all machines:
   ```bash
   pkill -f zenoh-bridge
   ```
2. **Reset the ROS Localhost environment variable** (this is critical so that nodes on different machines can discover each other over WiFi DDS):
   ```bash
   unset ROS_LOCALHOST_ONLY
   ```
3. **Launch the nodes with `use_zenoh:=false`**:
   * **On Jetson**:
     ```bash
     ros2 launch ausra_comms hardware_with_comms.launch.py robot_name:=ausra_1 use_zenoh:=false
     ```
   * **On Laptop**:
     ```bash
     USE_ZENOH=false ./start_base.sh
     # OR
     ros2 launch ausra_comms_base base_station.launch.py use_zenoh:=false
     ```

