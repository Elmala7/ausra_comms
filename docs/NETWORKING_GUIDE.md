# Networking Guide — Router vs IBSS (Ad-Hoc) vs SoftAP

> Reference for choosing and configuring the WiFi layer under the AUSRA Zenoh
> comms stack. Pairs with
> [`ZENOH_GUIDE.md`](../ausra_comms_base/docs/ZENOH_GUIDE.md) (transport) and
> [`DEPLOYMENT_DECENTRALIZED.md`](../ausra_comms_base/docs/DEPLOYMENT_DECENTRALIZED.md)
> (topology).
>
> **The golden rule:** the ROS/Zenoh layer does not care *how* the IP link is
> formed. DDS is pinned to loopback (`ROS_LOCALHOST_ONLY=1`); Zenoh is the only
> thing crossing machines, and it only needs **reachable IP addresses**. So the
> link choice is an IP-connectivity + reliability decision, not a ROS one.

---

## 1. Quick answer: switching from the Archer C7 router to IBSS

**Do I need to change anything besides the IPs in the json5 files?**
Almost — but **one more thing matters: multicast scouting.** Two changes:

1. **IP addresses** in every `connect.endpoints` list (all `*.json5`). Required.
2. **Disable multicast scouting** and rely on static `connect` endpoints + gossip.
   In IBSS, multicast/broadcast delivery is unreliable and often silently
   dropped by drivers (AX210/AX211 included), so Zenoh's default multicast
   discovery at `224.0.0.224:7446` cannot be trusted to find peers.

Everything else (`mode: "peer"`, `listen` on `0.0.0.0:7447`, the allowlist,
`reliable_routes_blocking`) stays identical.

### Zenoh scouting behaviour in IBSS

| Mechanism | In an infrastructure network (router/SoftAP) | In IBSS (ad-hoc) |
|---|---|---|
| **Multicast scouting** (`scouting.multicast`) | Works — peers auto-discover via the `224.0.0.224:7446` group | **Unreliable.** IBSS multicast/broadcast frames are often not relayed between stations; discovery silently fails |
| **Gossip** (`scouting.gossip`) | Works — peers learn of each other through already-connected peers | Works **only after** a static connection exists to seed it |
| **Static `connect.endpoints`** | Works | **Works — this is the reliable path.** Explicit TCP to each peer's IP |

**Recommendation for IBSS: disable multicast, keep gossip on, list every peer
statically.** Multicast off removes the dependence on a feature IBSS doesn't
deliver; static endpoints guarantee the initial mesh; gossip then fills in any
peers you forgot to list.

```json5
// json5 changes for IBSS (apply to BOTH jetson + laptop configs)
scouting: {
  multicast: { enabled: false },   // <-- was true; IBSS can't be trusted with it
  gossip:    { enabled: true  },   // keep — propagates peers once one link is up
},
connect: {
  endpoints: [
    "tcp/10.0.0.1:7447",   // every OTHER peer's static IBSS IP
    "tcp/10.0.0.2:7447",
    "tcp/10.0.0.3:7447",
  ],
},
listen: { endpoints: ["tcp/0.0.0.0:7447"] },   // unchanged
mode: "peer",                                   // unchanged
```

> IBSS gives no DHCP, so you must assign **static IPs** on each card (e.g. via
> `ip addr add`). Put those exact IPs in `connect.endpoints`. There is no
> gateway handing out addresses the way the Archer C7 does.

---

## 2. Router vs IBSS vs SoftAP — comparison

### TP-Link Archer C7 (infrastructure router) — *current setup*
**Pros**
- DHCP, DNS, stable addressing out of the box.
- Multicast scouting works → zero static-peer config needed.
- Highest aggregate throughput (dedicated radios, beamforming, good antennas).
- Decouples robots from each other: any node can reboot without disturbing the link.
- Easiest to debug (web UI, client list, signal stats).

**Cons**
- A **single point of failure** for connectivity — if the router dies or you roam
  out of its range, the whole swarm loses cross-machine comms (on-board SLAM/Nav2
  keep running; only the merged-map sharing drops — this is the WiFi-independence
  guarantee in `CLAUDE.md` §8).
- Requires carrying/powering an extra device; ties you to its coverage area.
- Extra latency hop (robot → AP → robot) vs a direct link.

**Best for:** bench testing, demos, and any fixed-area deployment. **This is the
recommended default** for the AUSRA stack today.

### IBSS / Ad-Hoc (AX210/AX211 peer-to-peer)
**Pros**
- **No infrastructure** — robots form the network themselves; nothing extra to power.
- Truly decentralized: matches the decentralized map-merge topology where there
  is no privileged node.
- Direct robot-to-robot frames (one hop), potentially lower latency.
- Survives loss of any single node — the mesh persists among survivors.

**Cons**
- **Unreliable multicast/broadcast** → must disable Zenoh multicast scouting and
  manage static IPs + static peer lists (see §1).
- Modern driver/firmware support for IBSS is uneven; AX210/AX211 IBSS can need
  specific `iw`/regulatory setup and may not negotiate high MCS rates → **lower
  throughput** than infrastructure mode.
- No DHCP/DNS — manual static IP assignment per node.
- Harder to debug; no central client list.

**Best for:** field operation with no infrastructure where full decentralization
is the goal and you accept manual network config + reduced throughput.

### SoftAP / Hotspot (one device is the access point)
**Pros**
- Infrastructure-mode reliability (multicast scouting works) **without a separate
  router** — one Jetson or the laptop hosts the AP, others join as clients.
- DHCP available from the AP host (`hostapd` + `dnsmasq`).
- Better throughput and rate negotiation than IBSS on the same cards.
- Quick to stand up in the field.

**Cons**
- The **AP host becomes a single point of failure** — if that node dies, the
  network collapses (worse than a router, since the AP is also doing robot work).
- The AP host's CPU/radio is shared between AP duties and its own SLAM/merge load.
- One card in AP mode usually can't also be a station — that node can't roam.

**Best for:** field deployment without a router when you want infrastructure-mode
reliability and can designate one node (ideally the laptop, which has spare CPU)
as the AP.

### Summary matrix

| Criterion | Router (C7) | IBSS | SoftAP |
|---|---|---|---|
| Setup effort | Low | High (static IP + static peers) | Medium |
| Multicast scouting works | ✅ | ❌ (disable it) | ✅ |
| Throughput | Highest | Lowest | High |
| Single point of failure | Router | None | AP host |
| Needs extra hardware | Yes | No | No |
| Matches decentralized ethos | No | ✅ | Partly |
| Recommended for | Bench / demos | Infra-free field | Field w/o router |

**Overall recommendation:** keep the **router for development**. For untethered
field runs, prefer **SoftAP on the laptop** (reliability + spare CPU) unless you
specifically need a no-single-point-of-failure mesh, in which case use **IBSS**
with multicast disabled and static peers.

---

## 3. Config-tuning opportunities found in the current setup

Independent of the link choice, two settings are worth tuning before a run:

### 3.1 `merging_rate` vs map cadence (laptop/Jetson CPU) — APPLIED
`relay_node` ships a map at most every `map_interval_sec = 5.0` s (0.2 Hz),
stretching to 15–30 s under adaptive throttling. The merger previously ran at
`merging_rate: 1.0` Hz — ~5× more often than the data changes, recompositing
identical canvases. This is now tuned to match the map flow in both active
configs (`map_merge_swarm_params.yaml` and `map_merge_HW_params.yaml`):

```yaml
# map_merge_swarm_params.yaml / map_merge_HW_params.yaml  (current default)
merging_rate:   0.2     # matches the 0.2 Hz relay cadence; was 1.0
```

This is a free CPU win with no loss of freshness (the merger can't produce a
newer result than the newest input map). It matters most on the Jetson in the
decentralized topology, where SLAM and merge share one CPU.

### 3.2 Multicast scouting (link-dependent)
Leave `scouting.multicast.enabled: true` on the router/SoftAP. Set it `false` for
IBSS (§1). Do not leave it `true` on IBSS and hope — it will appear to work on the
bench (where you also have the router up) and then fail in the field.

> Both are config-only; no code change. The Python/C++ nodes are already
> optimized (`UInt8MultiArray`, `TRANSIENT_LOCAL`+`RELIABLE` depth 2,
> `MultiThreadedExecutor`). See
> [`ZENOH_GUIDE.md`](../ausra_comms_base/docs/ZENOH_GUIDE.md) for the allowlist
> two-files-in-sync rule when you edit the json5 configs.

---

## 4. Verifying IP reachability before launching ROS

Regardless of link type, confirm plain IP connectivity first — it isolates link
problems from Zenoh/ROS problems:

```bash
ping -c 3 <peer_ip>                 # link + IP layer
nc -vz <peer_ip> 7447               # is the peer's Zenoh listener reachable (TCP)
ros2 topic hz /ausra_2/map_compressed   # only meaningful once Zenoh is up
```

If `ping` works but `nc` fails → Zenoh bridge not running or firewall on 7447.
If `ping` fails → pure network-layer problem (wrong IP, wrong channel/SSID, IBSS
cells not merged). Fix that before touching any json5.
