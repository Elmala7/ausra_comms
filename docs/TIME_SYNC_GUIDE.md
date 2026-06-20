# Time Sync Guide — Jetson Clocks Without Internet

## Why this exists

The two Jetson Orin Nanos have **no RTC battery**, so their system clocks reset
to a default time on every reboot. A wrong clock **breaks Zenoh communication**
(and ROS time-stamped messages in general).

There is **no internet** on the robot network, so NTP (`pool.ntp.org`) is not an
option. Instead, the **laptop is the single source of truth**: before each
session we push the laptop's clock to both Jetsons over SSH.

This is a **per-boot ritual** — the clock is lost every time a Jetson powers off,
so you sync every session, the same cadence NTP would have used.

| Item | Value |
|---|---|
| Jetson 1 | `ausranano@192.168.0.129` |
| Jetson 2 | `ausranano@192.168.0.165` |
| SSH auth | password login (you type the login password) |
| sudo auth | passwordless for `/usr/bin/date` only (sudoers rule, set up once) |
| Laptop script | `~/sync_time.sh` |

---

## Part A — One-time setup

Do this **once per Jetson**. After this, the per-session sync needs no sudo
password.

### A1. On EACH Jetson: allow passwordless `date`

SSH into each Jetson and add a narrow sudoers rule (passwordless **only** for the
`date` command — nothing else). You'll type the sudo password this one time.

```bash
# From the laptop, SSH into Jetson 1
ssh ausranano@192.168.0.129

# --- now on the Jetson ---
echo "ausranano ALL=(ALL) NOPASSWD: /usr/bin/date" | sudo tee /etc/sudoers.d/sync_time_date
sudo chmod 440 /etc/sudoers.d/sync_time_date

# Verify the rule is valid (must print no errors)
sudo visudo -c

# Confirm date now needs no password
sudo -n date && echo "OK: passwordless date works"

exit
```

Repeat for Jetson 2:

```bash
ssh ausranano@192.168.0.165
echo "ausranano ALL=(ALL) NOPASSWD: /usr/bin/date" | sudo tee /etc/sudoers.d/sync_time_date
sudo chmod 440 /etc/sudoers.d/sync_time_date
sudo visudo -c
sudo -n date && echo "OK: passwordless date works"
exit
```

> **Why `/usr/bin/date`?** Check the path with `which date` on the Jetson — it
> should be `/usr/bin/date`. If it differs, use that path in the sudoers rule.

### A2. On the laptop: confirm the sync script exists

The script lives at `~/sync_time.sh` and is already created. Verify and make
sure it's executable:

```bash
ls -l ~/sync_time.sh
chmod +x ~/sync_time.sh
```

If it's missing, recreate it from **Appendix: sync_time.sh** at the bottom.

### A3. (Optional but recommended) Disable NTP on the Jetsons

Since there's no internet, `systemd-timesyncd` will waste time trying to reach
NTP servers and can fight your manual `date -s`. Disable it on each Jetson:

```bash
ssh ausranano@192.168.0.129 'sudo timedatectl set-ntp false'
ssh ausranano@192.168.0.165 'sudo timedatectl set-ntp false'
```

---

## Part B — Every session (the routine)

Run these **in this order** each time you power on the robots.

### B1. Boot the Jetsons
Power them on and wait for them to join the WiFi.

### B2. Sync clocks from the laptop — BEFORE launching anything

```bash
cd ~
./sync_time.sh
```

You'll be prompted for each Jetson's **SSH login password**. Expected output:

```
=== ausranano@192.168.0.129 ===
  set -> Sat Jun 20 10:50:01 AM EEST 2026
=== ausranano@192.168.0.165 ===
  set -> Sat Jun 20 10:50:02 AM EEST 2026

All Jetson clocks synced from laptop.
```

### B3. Verify the clocks match the laptop

```bash
# Laptop time
date

# Jetson times (should be within ~1-2 seconds of the laptop)
ssh ausranano@192.168.0.129 date
ssh ausranano@192.168.0.165 date
```

### B4. NOW launch ROS / Zenoh on the Jetsons

Only after clocks are synced:

```bash
# On each Jetson (per CLAUDE.md)
ros2 launch ausra_comms hardware_with_comms.launch.py robot_name:=ausra_1
```

> **Order matters.** `date -s` causes a time jump. If the `zenoh-bridge` is
> already running when you sync, a backward jump confuses it — **kill and restart
> the bridge** after syncing (see B5).

### B5. If you synced AFTER the bridge was already running

Kill stale bridges on both Jetsons, then relaunch:

```bash
ssh ausranano@192.168.0.129 'pkill -f zenoh-bridge'
ssh ausranano@192.168.0.165 'pkill -f zenoh-bridge'
# then relaunch the stack on each Jetson
```

---

## One-liner cheat sheet (per session)

```bash
./sync_time.sh                          # 1. sync clocks (before launch)
date && ssh ausranano@192.168.0.129 date && ssh ausranano@192.168.0.165 date   # 2. verify
# 3. launch ROS/Zenoh on the Jetsons
```

---

## Troubleshooting

| Symptom | Cause / Fix |
|---|---|
| `sudo: a password is required` during sync | Sudoers rule missing/wrong on that Jetson. Redo **A1**; check `which date` path. |
| `FAILED to reach ... ` | Jetson off, not on WiFi, or wrong IP. Check power/WiFi, `ping <ip>`. |
| Clocks still drift mid-session | Normal — Jetson clock only drifts slowly while powered. Only the **reboot** resets it. Re-run `sync_time.sh` if needed. |
| Zenoh still broken after sync | Bridge was running during the time jump. Do **B5** (kill + relaunch bridge). |
| `zenoh ... exceeding delta 500ms is rejected ... Replace timestamp` (log flood) | Clocks are skewed >500 ms. Non-fatal (Zenoh replaces the timestamp and still delivers — maps flow), but means sync isn't tight. The script pushes **sub-second epoch time** (`date -s @<epoch.ns>`) for exactly this; re-run `sync_time.sh`. If it persists, the SSH round-trip itself exceeds 500 ms — sync each Jetson in its own terminal, or raise tolerance with `timestamping: { drop_future_timestamp: false }` in both JSON5 bridge configs. |
| `visudo -c` reports an error | Bad sudoers file — fix `/etc/sudoers.d/sync_time_date`. A malformed file can lock sudo; keep a root shell open until `visudo -c` passes. |

---

## Appendix: `sync_time.sh`

If you need to recreate the laptop script (`~/sync_time.sh`):

```bash
#!/usr/bin/env bash
# Push THIS laptop's clock to both Jetson Orin Nanos. No internet needed.
set -u

JETSONS=("ausranano@192.168.0.129" "ausranano@192.168.0.165")

fail=0
for host in "${JETSONS[@]}"; do
  echo "=== $host ==="
  # Sub-second epoch sync: keeps skew under Zenoh's 500 ms HLC window.
  NOW="$(date '+%s.%N')"
  if ssh -o ConnectTimeout=5 "$host" "sudo date -s '@$NOW' >/dev/null && echo \"  set -> \$(date '+%Y-%m-%d %H:%M:%S.%N')\""; then
    :
  else
    echo "  FAILED to reach or set time on $host"
    fail=1
  fi
done

echo
if [ "$fail" -eq 0 ]; then
  echo "All Jetson clocks synced from laptop."
else
  echo "One or more Jetsons failed — check power / WiFi / IP and rerun."
fi
exit "$fail"
```

Then: `chmod +x ~/sync_time.sh`
