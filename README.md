# Autonomous Robotic Inspection with a Quadruped Robot

Code, logs, and setup notes for autonomous inspection with a **Unitree Go2 EDU** quadruped robot in GPS-denied, cluttered indoor environments. The robot navigates on a saved LiDAR SLAM map and uses QR codes as named inspection waypoints. Each QR detection is paired with the robot's SLAM pose, so the same targets can be revisited and the consistency of the logged pose can be measured.

This repository supports the paper:

> L. Protcheva, M. I. Radaideh, *Autonomous Inspection on a Quadruped Robot with QR-Anchored Waypoints and SLAM Map Reuse*, Robotics and Autonomous Systems (in preparation).

---

## Contents

- [Paper data](#paper-data)
- [Repository structure](#repository-structure)
- [Hardware](#hardware)
- [System overview](#system-overview)
- [Building keyDemo](#building-keydemo)
- [Running the system](#running-the-system)
- [Key bindings](#key-bindings)
- [Troubleshooting](#troubleshooting)
- [Connecting to the dock Jetson over Wi-Fi](#connecting-to-the-dock-jetson-over-wi-fi)
- [Planned work](#planned-work)
- [License](#license)
- [Acknowledgments](#acknowledgments)

---

## Paper data

The experiments in the paper were run at two University of Michigan laboratories: the Irradiated Materials Testing Laboratory (IMTL) and the Thermal Hydraulics Laboratory (THL). Each run produced two logs from the same physical run:

- **Operation layer** (`keyDemo` console output): relocalization check, QR detections and poses, goal dispatch, and arrivals.
- **Navigation layer** (`unitree_slam` console output): A* planning, QP motion planning, and obstacle constraint boxes.

| Campaign | Operation-layer log | Navigation-layer log |
|---|---|---|
| IMTL Trial 1 | `research/logs/imtl_trial1_operation.log` *(to be added)* | `research/logs/imtl_trials_navigation_firstrun.log` |
| IMTL Trial 2 | `research/logs/imtl_trial2_operation.log` *(to be added)* | covered by the Trials 1–2 capture *(to be added)* |
| IMTL Trial 3 | `research/logs/imtl_trial3_operation.log` | `research/logs/imtl_trial3_navigation.log` *(to be added)* |
| IMTL patrol | `research/logs/imtl_patrol_operation.log` *(to be added)* | `research/logs/imtl_patrol_navigation.log` *(to be added)* |
| THL patrol | `research/logs/thl_patrol_operation.log` | `research/logs/thl_patrol_navigation.log` |

Notes on the logs:

- The SLAM service ran continuously through IMTL Trials 1 and 2, so their navigation-layer figures come from splitting one capture at the boundary between the two waypoint sets. The Trial 3 navigation capture starts partway through the run, so its planner counts are lower bounds.
- Every run ended while one goal was still in progress. These goals are excluded from all counts in the paper.
- All reported values in the paper are taken directly from these logs.

Other paper materials:

| Item | Location |
|---|---|
| Final `keyDemo.cpp` used in all experiments | `research/actualDog/src/keyDemo.cpp` *(to be added)* |
| Saved maps | `research/maps/imtl_map.pcd`, `research/maps/thl_map.pcd` *(to be added)* |
| Log-parsing and plotting scripts | `research/scripts/` *(to be added)* |
| Provenance sheets (each reported value traced to a log line) | `research/provenance/` *(to be added)* |

---

## Repository structure

```
.
├── README.md
└── research/
    ├── actualDog/
    │   ├── src/          # C++ operation-layer code (keyDemo and earlier versions)
    │   └── resources/    # CMakeLists for keyDemo, example QR pose file, example map
    ├── logs/             # Operation-layer and navigation-layer logs for the paper
    ├── demos/            # Demo recording
    ├── reports/          # Progress reports
    ├── detect_qr.py, qr_detection_standing.py   # Standalone QR detection tests
    └── get_position.cpp, move_test.cpp           # Small robot test programs
```

The files in `research/actualDog/src/` other than `keyDemo.cpp` are earlier development versions and were not used for the experiments in the paper.

---

## Hardware

| Component | Specification |
|---|---|
| Unitree Go2 EDU | Quadruped mobility platform |
| Jetson Orin Nano dock | IP `192.168.123.18` (ethernet) or `10.42.0.1` (Wi-Fi hotspot), Ubuntu 20.04, kernel `5.10.104-tegra` |
| Hesai XT16 LiDAR | IP `192.168.123.20`, 16-channel rotating LiDAR for SLAM |
| Front RGB camera | H.264 UDP multicast stream, used for QR detection |
| ALFA AWUS036ACM Wi-Fi adapter | MediaTek MT7612U chipset, configured in AP/hotspot mode |

---

## System overview

The software has two layers that run at the same time:

- **Navigation layer:** the robot's onboard SLAM and navigation service (`unitree_slam`). It handles mapping, relocalization, A* global planning, and QP-based local motion planning. We modified it to change how maps are saved and to check that a map is detailed enough before it is used.
- **Operation layer:** our modified `keyDemo` program. It:
  - checks relocalization before anything else starts: after relocalization succeeds, it waits 2 s and requires the reported pose to lie within 0.5 m of the map origin, or QR scanning and navigation stay disabled
  - decodes QR codes from the live H.264 camera stream with OpenCV
  - records the SLAM pose for each marker, keeping the pose from the last frame in which the marker was visible
  - builds and locks the waypoint list after a teaching pass
  - dispatches goals in a repeating circuit and confirms arrival when the robot stays within 0.2 m of a waypoint for 3 s
  - keeps detecting QR codes during patrol in observation-only mode, so new sightings are logged but never change the locked list
  - logs every detection, pose, and arrival

```
Camera → QR detection → SLAM pose snapshot → waypoint list → goal dispatch → arrival check
                              ↑
                    unitree_slam (localization, A*, QP planning)
```

Before lab experiments, obstacle avoidance, robot movement, and object recognition were tested in the Gazebo simulator.

---

## Building keyDemo

`keyDemo` is built inside the Unitree SLAM module on the Jetson:

```bash
cp research/actualDog/src/keyDemo.cpp /unitree/module/unitree_slam/example/src/keyDemo.cpp
cp research/actualDog/resources/CMakeLists.txt /unitree/module/unitree_slam/example/CMakeLists.txt

cd /unitree/module/unitree_slam/example
rm -rf build
mkdir build && cd build
cmake ..
make -j4
```

Do not run `apt upgrade` or `apt full-upgrade` on the Jetson. Many packages are pending upgrade but should not be touched.

---

## Running the system

The system needs two terminals: the SLAM service and `keyDemo`. `unitree_slam` handles the LiDAR internally, so a separate `xt16_driver` process is not needed.

### Prerequisites

- Jetson connected to the robot through the dock
- LiDAR connected to the Jetson via RJ45 (never touch the LiDAR connector while the battery is inserted)
- Network interface name confirmed (usually `eth0`)

### Stop old processes

```bash
pkill unitree_slam
pkill keyDemo
```

### Terminal 1: SLAM service

```bash
cd /unitree/module/unitree_slam/bin
export LD_LIBRARY_PATH=/unitree/module/unitree_slam/third_party_lib:/unitree/module/unitree_slam/lib:$LD_LIBRARY_PATH
./unitree_slam
```

Expected output:

```
xt16 lidar ysn check success!
...
server start ...
server started. name:slam_operate, enable proirity queue:1
```

Keep this terminal running for the whole session.

### Terminal 2: keyDemo

```bash
unset ROS_DOMAIN_ID
export CYCLONEDDS_URI=file:///unitree/module/unitree_slam/config/slam_interfaces_server_config/cyclonedds.xml
source /opt/ros/foxy/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib/dds_backup:/usr/local/lib:/unitree/module/unitree_slam/lib:$LD_LIBRARY_PATH
cd /unitree/module/unitree_slam/bin
./keyDemo eth0
```

Replace `eth0` with your interface name if it is different.

### Running a patrol (as in the paper)

1. Map the area once: press `q`, drive the robot through the area, then press `w` to save the map.
2. Place the robot on the starting mark at the map origin and press `a` to relocalize. Wait for `Relocation verified`.
3. Drive the robot past each QR marker. Each new marker is added as a waypoint.
4. Press `d` to lock the list and start autonomous patrol.
5. Stop the run when finished. The goal in progress at that moment is excluded from analysis.

---

## Key bindings

| Key | Action |
|---|---|
| `q` | Start mapping |
| `w` | End mapping and save the map |
| `a` | Relocalize, run the relocalization check, and start QR scanning (unlocks the waypoint list) |
| `s` | Manually add the current pose to the task list |
| `d` | Lock the waypoint list and start autonomous execution |
| `f` | Clear the task list and QR records (unlocks the list) |
| `z` | Pause navigation |
| `x` | Resume navigation |
| `Ctrl+C` | Exit |

---

## Troubleshooting

### LiDAR clock source error

**Symptom:** the LiDAR pipeline crashes or logs "Abnormal time information, delta time = 0.017..."

**Cause:** the LiDAR clock source is set to GPS but has no GPS signal.

**Fix:**
1. Open the Pandar web UI at `http://192.168.123.20` (`curl` from the Jetson is faster than a laptop browser)
2. Go to Settings → Clock Source
3. Change from GPS to PTP or Free Run
4. Save and restart the SLAM service

### LiDAR serial number mismatch

**Symptom:** `unitree_slam` reports "xt16 lidar ysn check fail!"

**Cause:** `param.yaml` contains the wrong LiDAR serial number.

**Fix:**
1. Edit `/unitree/module/unitree_slam/param.yaml`
2. Set the `lidar_ysn` field to the serial printed on your LiDAR
3. Check: `grep lidar_ysn /unitree/module/unitree_slam/param.yaml`

### keyDemo binary missing

**Symptom:** `./keyDemo` is not found.

**Fix:** build it as described in [Building keyDemo](#building-keydemo), then check `ls -la /unitree/module/unitree_slam/bin/keyDemo`.

### Symbol lookup error: `iceoryx_header_from_chunk`

**Symptom:** `./keyDemo: symbol lookup error: ./keyDemo: undefined symbol: iceoryx_header_from_chunk`

**Cause:** DDS library version conflict.

**Fix:** put the DDS backup libraries first in the library path:

```bash
export LD_LIBRARY_PATH=/usr/local/lib/dds_backup:/usr/local/lib:/unitree/module/unitree_slam/lib:$LD_LIBRARY_PATH
./keyDemo eth0
```

### Relocalization rejected

**Symptom:** `[WARN] Relocalization suspect: pose is ... m from expected dock origin`

**Cause:** the relocalized pose is more than 0.5 m from the map origin.

**Fix:** return the robot to the starting mark and press `a` again.

### LiDAR protection circuit tripped

**Symptom:** the LiDAR does not power on and the web UI is unreachable.

**Cause:** the LiDAR connector was touched while the battery was inserted.

**Fix:**
1. Power off the robot completely
2. Remove the battery and wait 5–10 minutes
3. Reinsert the battery and power on

Never touch the LiDAR connector with the battery inserted, even if the robot is off.

### Jetson network issues

**Symptom:** `eth0` stops working, or Wi-Fi fails.

**Cause:** switching the Jetson to client-mode Wi-Fi (`nmcli connection up` to an external network) disrupts ethernet.

**Fix:** keep the Jetson in AP/hotspot mode. If eth0 is already stuck, reboot the robot.

### SSH into the Jetson

```bash
ssh unitree@192.168.123.18   # ethernet
ssh unitree@10.42.0.1        # Wi-Fi hotspot
```

The default password is the one in the Unitree documentation. Change it if the robot is used outside a controlled lab.

The Jetson has no internet access by design. Do not add external routing.

---

## Connecting to the dock Jetson over Wi-Fi

The dock Jetson broadcasts a hotspot (default SSID `DroneBlocks-Go2-001`), so you can SSH in without an ethernet cable.

### One-time setup (already done on our robot)

- ALFA AWUS036ACM Wi-Fi adapter (MediaTek MT7612U chipset)
- In-kernel `mt76x2u` driver (no install needed). Check the chipset with `lsusb` before any driver work.
- NetworkManager configured with `managed=true` in `/etc/NetworkManager/NetworkManager.conf`
- Hotspot profile `Go2-Hotspot` saved in NetworkManager with autoconnect enabled

### Each time

1. Plug the Wi-Fi adapter into the dock Jetson's USB port.
2. Wait about 10 seconds. The hotspot should come up automatically.
3. From your laptop, connect to the hotspot SSID with the password from the DroneBlocks guide.
4. SSH in: `ssh unitree@10.42.0.1`

### If the hotspot is not up

Connect by ethernet (`ssh unitree@192.168.123.18`) and check:

```bash
nmcli device status          # wlan0 should show 'connected' to Go2-Hotspot
ip addr show wlan0           # should have 10.42.0.1/24
sudo nmcli connection up Go2-Hotspot   # bring it up manually if needed
lsusb | grep -i mediatek     # should show '0e8d:7612 MediaTek Inc. Wireless'
```

### Notes

- The hotspot is local only. Your laptop loses internet while connected to it.
- Ethernet (`192.168.123.18`) and Wi-Fi (`10.42.0.1`) can both be used at the same time.
- Do not replace the hotspot profile with a client-mode connection to an external router. It disrupts eth0 and requires a reboot.

### Changing the hotspot credentials

The default credentials are public. To change them:

```bash
sudo nmcli connection modify Go2-Hotspot 802-11-wireless.ssid "your_new_ssid"
sudo nmcli connection modify Go2-Hotspot wifi-sec.psk "your_new_password"
sudo nmcli connection down Go2-Hotspot
sudo nmcli connection up Go2-Hotspot
```

---

## Planned work

- **Radiation sensing:** mount a compact gamma spectrometer and record counts at each QR-anchored station, so each measurement is tied to a named location in SLAM coordinates
- **Change detection:** compare radiation measurements across repeated patrols at the same stations, using the station-grouped pose repeatability as the positioning noise floor
- **Longer patrols:** fifty goals or more per session
- **Testing at an operating nuclear facility**

---

## License

[Add a license before release. Check with the AIMS Lab which license to use.]

---

## Related publications and presentations

- L. Protcheva, M. I. Radaideh, *Autonomous Inspection on a Quadruped Robot with QR-Anchored Waypoints and SLAM Map Reuse*, Robotics and Autonomous Systems (in preparation)
- ANS Winter Conference 2026: accepted summary
- STARFIRE Workshop presentations: 2025 and 2026

---

## Acknowledgments

This work was supported by the University of Michigan Department of Nuclear Engineering and Radiological Sciences and the Fastest Path to Zero Initiative. We thank the staff of the Irradiated Materials Testing Laboratory and the Thermal Hydraulics Laboratory for access to their facilities.

Contact: Lada Protcheva (lprotche@umich.edu), AIMS Lab, University of Michigan. Advisor: Prof. Majdi I. Radaideh.
