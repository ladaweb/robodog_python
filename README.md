# Autonomous Robotic Inspection Platform for Nuclear and Hazardous Environments

A research platform for autonomous semantic inspection, LiDAR-based mapping, and future radiation-aware navigation using the **Unitree Go2 EDU** quadruped robot.

---

## Research Overview

This project develops an AI-assisted autonomous quadruped robotic system for semantic inspection, spatial mapping, and radiation-aware navigation in nuclear and hazardous indoor environments.

The system combines LiDAR-based SLAM, vision-based semantic localization (QR checkpoints), and ROS2 autonomy to enable a robot that can:

- build and reuse accurate indoor maps
- localize itself reliably within those maps
- navigate autonomously to predefined inspection points
- avoid obstacles in real time
- record and revisit semantic waypoints
- (planned) sense and spatially map radiation levels

---

## Hardware

| Component | Role |
|---|---|
| Unitree Go2 EDU | Quadruped mobility platform |
| Hesai XT16 LiDAR | 3D environment perception and SLAM |
| Front RGB camera | QR detection and visual inspection |
| Radiacode 103G *(planned)* | Radiation sensing |

---

## System Architecture

```
Camera → QR detection → Pose snapshot → QR database → Task list → Navigation
                     ↑             ↑
                SLAM pose      JSON storage

Navigation → Local planner → Obstacle avoidance → Motor control
```

Localization, perception, and navigation run concurrently in separate threads.

### Core modules

- **SLAM navigation client** — map building, relocation, pose-based goal execution
- **SLAM pose subscriber** — continuous localization from the LiDAR SLAM service
- **Video stream module** — UDP multicast H264 stream decoded via GStreamer + OpenCV
- **QR detection thread** — real-time detection using `cv::QRCodeDetector`
- **Pose recording module** — last-visible-frame strategy for accurate waypoint capture
- **QR pose database** — persistent JSON storage of inspection waypoints
- **Navigation state manager** — pauses QR detection during autonomous traversal
- **Obstacle avoidance / local planner** — real-time replanning around dynamic obstacles

---

## Current Capabilities

- Autonomous indoor navigation using saved LiDAR maps
- SLAM-based localization and map reuse
- Real-time obstacle avoidance (LiDAR + depth camera)
- QR code-based semantic waypoint acquisition
- Last-frame pose recording (stores the closest reachable pose to each marker)
- Persistent waypoint storage to JSON for repeatable inspection routes
- Bidirectional task list execution (forward and reverse traversal)
- QR detection isolation during navigation (prevents pose corruption)

---

## Key Finding from Experiments

Repeated navigation trials in an indoor basement environment showed significant improvement across runs:

| Run | Time to first checkpoint | Battery used |
|---|---|---|
| Training run 1 | 55 sec | ~12% |
| Training run 2 | 22 sec | ~10% |
| Final evaluation | 10 sec | ~4% |

**Important:** localization and navigation quality degrade when the map becomes outdated or insufficiently detailed. Periodic remapping is required when:
- navigation performance degrades noticeably
- localization becomes unstable
- the environment layout changes

---

## Research Domains

| Area | Role |
|---|---|
| Robotics | Autonomous quadruped mobility |
| ROS2 | Communication and system integration |
| SLAM | Mapping and localization |
| LiDAR perception | 3D environment understanding |
| Semantic mapping | Associating meaning with spatial locations |
| Nuclear engineering | Inspection and radiation monitoring applications |
| AI / autonomy | Future intelligent inspection behavior |

---

## Planned Capabilities

- **Radiation sensing** — integration with Radiacode 103G detector
- **Radiation heatmaps** — 2D/3D spatial radiation maps aligned to SLAM coordinates
- **Semantic object recognition** — camera-based identification of inspection targets
- **Autonomous patrol routines** — scheduled inspection without human intervention
- **Hazard-aware navigation** — routing that avoids high-radiation zones
- **Advanced LiDAR SLAM** — improved localization in repetitive or featureless environments

---

## Repository Structure

```
.
├── unitree_sdk2py/          # Python SDK wrapping Unitree SDK2 (DDS/ROS2 interface)
│   ├── core/                # DDS channel setup
│   ├── go2/                 # Go2-specific clients (sport, video, obstacle avoidance, VUI)
│   ├── rpc/                 # RPC client/server infrastructure
│   └── idl/                 # DDS IDL message definitions
│
└── example/
    └── go2/
        ├── research/
        │   ├── actualDog/
        │   │   ├── src/     # C++ autonomous navigation implementations
        │   │   └── resources/  # Maps, QR pose database, build config
        │   └── demos/       # Experiment reports and recordings
        ├── high_level/      # Python high-level sport API examples
        ├── low_level/       # Python low-level motor control examples
        └── front_camera/    # Camera stream examples
```

---

## Installation

### Dependencies

- Python >= 3.8
- cyclonedds == 0.10.2
- numpy
- opencv-python

### Install from source

```bash
sudo apt install python3-pip
git clone <this-repo>
cd robodog_python
pip3 install -e .
```

### CycloneDDS (if not found automatically)

```bash
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install

export CYCLONEDDS_HOME="~/cyclonedds/install"
pip3 install -e .
```

---

## Building the C++ Navigation System

```bash
cd example/go2/research/actualDog
mkdir build && cd build
cmake ..
make
```

### Running autonomous navigation

```bash
./autonomous_nav_qr_detection_final <network_interface>
```

**Key bindings:**

| Key | Action |
|---|---|
| `q` | Start mapping |
| `w` | End mapping and save map |
| `a` | Start relocation (begins QR detection) |
| `s` | Manually add current pose to task list |
| `d` | Execute task list (navigate to all waypoints) |
| `f` | Clear task list and QR pose database |
| `z` | Pause navigation |
| `x` | Resume navigation |
| `Ctrl+C` | Exit |

---

## Network Setup

Before running any examples, configure the robot's network connection per the [Unitree quick start guide](https://support.unitree.com/home/en/developer/Quick_start). Replace `enp2s0` in all commands with your actual network interface name.

---

## SDK Examples

```bash
# DDS pub/sub communication test
python3 example/helloworld/publisher.py
python3 example/helloworld/subscriber.py

# High-level sport control
python3 example/go2/high_level/go2_sport_client.py enp2s0

# Front camera stream (requires display)
python3 example/go2/front_camera/camera_opencv.py enp2s0

# Obstacle avoidance toggle
python3 example/obstacles_avoid/obstacles_avoid_switch.py enp2s0
```

---

## Connecting to the Dock Jetson via WiFi

The dock Jetson broadcasts a WiFi hotspot named `DroneBlocks-Go2-001`. Connecting your laptop to this hotspot lets you SSH into the Jetson without an ethernet cable.

### Prerequisites (one-time, already done)
- BrosTrend / ALFA AC1L WiFi adapter (MediaTek MT7612U chipset)
- `mt76x2u` driver (in-kernel on Ubuntu 20.04 for JetPack — no install needed)
- NetworkManager configured with `managed=true` in `/etc/NetworkManager/NetworkManager.conf`
- Hotspot profile `Go2-Hotspot` saved in NetworkManager (autoconnect enabled)

### Every-time usage

1. **Plug the WiFi adapter into the dock Jetson's USB port.**
2. Wait ~10 seconds for the adapter to enumerate. The `Go2-Hotspot` profile is set to autoconnect, so the hotspot should come up automatically.
3. **From your laptop**, look at available WiFi networks and connect to:
   - **SSID:** `DroneBlocks-Go2-001`
   - **Password:** `00000000`
4. **SSH into the Jetson over WiFi:**
   ```bash
   ssh unitree@10.42.0.1
   ```
   Password: `123`

### Verifying the hotspot is up (from the Jetson)

If SSH over WiFi isn't working, plug in ethernet, SSH via `192.168.123.18`, and check:

```bash
nmcli device status          # wlan0 should show 'connected' to Go2-Hotspot
ip addr show wlan0           # should have 10.42.0.1/24
```

If `wlan0` is present but disconnected, bring the hotspot up manually:

```bash
sudo nmcli connection up Go2-Hotspot
```

If `wlan0` doesn't appear at all, the adapter isn't being detected. Check:

```bash
lsusb | grep -i mediatek     # should show '0e8d:7612 MediaTek Inc. Wireless'
ip link show wlan0
```

### Important notes

- The `DroneBlocks-Go2-001` hotspot is **local-only** — it does not provide internet access. Your laptop will lose internet while connected to it. If you need internet on your laptop simultaneously, use a wired ethernet connection or USB tether for internet, and reserve WiFi for the Jetson hotspot.
- The dock Jetson's ethernet interface (`eth0`, `192.168.123.18`) continues to work independently. You can SSH via either `10.42.0.1` (WiFi) or `192.168.123.18` (ethernet) — both will work when both interfaces are up.
- Do **not** replace the hotspot profile with a client-mode WiFi connection to an external router. Doing so has been observed to disrupt eth0 routing on this Jetson and requires a reboot to recover.

### Hotspot credentials — change from defaults if used outside a controlled lab

Default credentials (`DroneBlocks-Go2-001` / `00000000`) are public knowledge from the DroneBlocks guide. To change them:

```bash
sudo nmcli connection modify Go2-Hotspot 802-11-wireless.ssid "your_new_ssid"
sudo nmcli connection modify Go2-Hotspot wifi-sec.psk "your_new_password"
sudo nmcli connection down Go2-Hotspot
sudo nmcli connection up Go2-Hotspot
```
