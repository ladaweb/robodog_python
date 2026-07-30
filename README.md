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

| Component | Specification |
|---|---|
| Unitree Go2 EDU | Quadruped mobility platform |
| Jetson Orin Nano Dock | IP `192.168.123.18` (ethernet) or `10.42.0.1` (Wi-Fi hotspot), runs Ubuntu 20.04, kernel `5.10.104-tegra` |
| Hesai XT16 LiDAR | IP `192.168.123.20`, serial `XT3AC251993AC150`, 16-channel rotating lidar for SLAM |
| Front RGB camera | QR detection and visual inspection |
| ALFA AWUS036ACM Wi-Fi adapter | MediaTek MT7612U chipset, configured in AP/hotspot mode for wireless operation |
| Radiacode 103G *(planned)* | Radiation sensing and environmental monitoring |

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
- QR code-based semantic waypoint acquisition and storage
- Last-frame pose recording (stores the closest reachable pose to each marker)
- Persistent waypoint storage to JSON for repeatable inspection routes
- Bidirectional task list execution (forward and reverse traversal)
- QR detection isolation during navigation (prevents pose corruption)
- Validation in both simulation (Gazebo/Sphinx) and real nuclear facility (Irradiated Materials Testing Laboratory, Michigan Memorial Phoenix Project)

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

### Novel Research Contribution

The QR code infrastructure uniquely enables separating positioning error from genuine radiological change. This separation is absent from prior literature and forms a key validation benchmark for radiation-aware inspection metrics. QR fiducials act as ground-truth anchors for pose verification and enable direct measurement of:
- localization accuracy (pose-to-fiducial deviation)
- navigation repeatability (run-to-run consistency)
- radiation count-rate deltas (change detection per location)
- false-alarm rate and minimum detectable change

---

## Research Domains

| Area | Role |
|---|---|
| Robotics | Autonomous quadruped mobility and motor control |
| ROS2 | Communication and system integration via DDS |
| SLAM | Mapping and localization using LiDAR |
| LiDAR perception | 3D environment understanding and point cloud processing |
| Semantic mapping | Associating meaning (QR codes, radiation) with spatial locations |
| Nuclear engineering | Inspection and radiation monitoring applications |
| AI / autonomy | Future intelligent inspection behavior and hazard awareness |

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
git clone https://github.com/ladaweb/robodog_python.git
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

### Important: Do NOT run apt upgrade on Jetson

Many packages on the Jetson Orin Nano are pending upgrade but should not be touched. Do not run `apt upgrade` or `apt full-upgrade`.

---

## Building the C++ Navigation System

```bash
cd example/go2/research/actualDog
mkdir build && cd build
cmake ..
make
```

This builds the autonomous navigation binary and supporting utilities.

---

## Running the SLAM System

The SLAM system consists of three components that must be run in sequence: the LiDAR driver, the SLAM service, and the navigation demo (keyDemo). Each runs in its own terminal with persistent output.

### Prerequisites

- Jetson connected to the robot via USB-C dock
- LiDAR connected to Jetson via RJ45 (must NOT touch the LiDAR connector with battery inserted)
- Network interface name confirmed (typically `eth0` for ethernet, `wlan0` for wireless hotspot)

### Cleanup (stop any existing processes)

Before starting a new session, kill any lingering processes:

```bash
pkill xt16_driver
pkill unitree_slam
pkill keyDemo
```

Or use `killall` if `pkill` does not work:

```bash
killall xt16_driver
killall unitree_slam
killall keyDemo
```

### Setup environment and paths

Navigate to the SLAM binary directory and configure the library path:

```bash
cd /unitree/module/unitree_slam/bin
export LD_LIBRARY_PATH=/unitree/module/unitree_slam/third_party_lib:/unitree/module/unitree_slam/lib:$LD_LIBRARY_PATH
```

**Note:** Every new terminal session requires this full environment variable block before running any SLAM binaries. Do not skip this step.

### Terminal 1: Run LiDAR driver

In the first terminal, start the Hesai XT16 LiDAR driver (runs in foreground):

```bash
cd /unitree/module/unitree_slam/bin
export LD_LIBRARY_PATH=/unitree/module/unitree_slam/third_party_lib:/unitree/module/unitree_slam/lib:$LD_LIBRARY_PATH
./xt16_driver
```

Expected output:

```
network: eth0
ip_address: 192.168.123.20
lidar_frame: rslidar
lidar_topic: rt/unitree/slam_lidar/points
----------------------------
       ///////////////////////////////////////////////////////////////
       //     PandarGeneralSDK version: ...
       //     XT16 LiDAR sensor starting...
       ///////////////////////////////////////////////////////////////
```

**Keep this terminal running.** The LiDAR driver must stay active for the entire session.

### Terminal 2: Run SLAM service

In a separate terminal, set up the environment again and start the SLAM service:

```bash
cd /unitree/module/unitree_slam/bin
export LD_LIBRARY_PATH=/unitree/module/unitree_slam/third_party_lib:/unitree/module/unitree_slam/lib:$LD_LIBRARY_PATH
./unitree_slam
```

Expected output (successful startup):

```
xt16 lidar ysn check success!
WARNING: Logging before InitGoogleLogging() is written to STDERR
I0103 11:01:37.835380  2969 timerwheel.hpp:57] Time wheel timer started!
I0103 11:01:38.151337  2982 timerwheel.hpp:57] Time wheel timer started!
server start ...
server started. name:slam_operate, enable proirity queue:1
```

**Keep this terminal running.** The SLAM service must stay active for navigation.

### Terminal 3: Run navigation demo (keyDemo)

In a third terminal, if you encounter symbol lookup errors related to iceoryx or DDS, update the library path to prioritize DDS backup libraries:

```bash
cd /unitree/module/unitree_slam/bin
export LD_LIBRARY_PATH=/usr/local/lib/dds_backup:/usr/local/lib:/unitree/module/unitree_slam/lib:$LD_LIBRARY_PATH
./keyDemo eth0
```

(Replace `eth0` with your actual network interface name if different.)

Expected output (successful startup):

```
*********************** Unitree SLAM Demo ***********************
---------------          q  w             -----------------
---------------       a  s  d  f          -----------------
```

---

## Autonomous Navigation Control

Once `keyDemo` is running and all three components are active, use the following key bindings:

| Key | Action |
|---|---|
| `q` | Start mapping mode (robot builds a map of the environment) |
| `w` | End mapping and save the map to disk |
| `a` | Start relocation mode (robot localizes within existing map and begins QR detection) |
| `s` | Manually add current pose to the task list (records current position as waypoint) |
| `d` | Execute task list (navigate autonomously to all recorded waypoints in order) |
| `f` | Clear task list and reset QR pose database (removes all waypoints) |
| `z` | Pause ongoing navigation (robot stops in place) |
| `x` | Resume navigation (robot continues to next waypoint) |
| `Ctrl+C` | Exit the demo and cleanly shut down |

---

## Known Issues and Troubleshooting

### LiDAR Clock Source Error

**Symptom:** `xt16_driver` crashes with segmentation fault or logs "Abnormal time information, delta time = 0.017..."

**Root cause:** The LiDAR's clock source is set to GPS mode but has no GPS signal (GPS: Unlock, PTP: Free Run).

**Solution:**
1. Open the Pandar web UI at `http://192.168.123.20` (use `curl` from the Jetson for instant response; laptop browser can be slow)
2. Navigate to Settings → Clock Source
3. Change from GPS to PTP or Free Run mode
4. Save and restart `xt16_driver`

### LiDAR Serial Number Mismatch

**Symptom:** `unitree_slam` reports "xt16 lidar ysn check fail!" or serial mismatch warning.

**Root cause:** The `param.yaml` file contains an incorrect LiDAR serial number.

**Solution:**
1. Edit `/unitree/module/unitree_slam/param.yaml`
2. Locate the `lidar_ysn` field
3. Update it to match the actual serial: `XT3AC251993AC150`
4. If a `vim` edit was interrupted (suspended via Ctrl+Z), resume with `fg` and save with `:wq`
5. Verify the change: `grep lidar_ysn /unitree/module/unitree_slam/param.yaml`

### keyDemo Binary Missing or Compile Error

**Symptom:** Cannot find `./keyDemo` or it fails to build.

**Root cause:** The binary is not built or the source was not compiled.

**Solution:**
1. Ensure the LiDAR pipeline is working (both `xt16_driver` and `unitree_slam` running)
2. Rebuild the SLAM module or ensure pre-built binaries are in place
3. Confirm the binary exists: `ls -la /unitree/module/unitree_slam/bin/keyDemo`
4. If missing, check the build directory and recompile if necessary

### Symbol Lookup Error: undefined symbol iceoryx_header_from_chunk

**Symptom:** `./keyDemo: symbol lookup error: ./keyDemo: undefined symbol: iceoryx_header_from_chunk`

**Cause:** DDS library version conflict or path ordering issue.

**Solution:** Use the alternate library path that prioritizes DDS backup libraries:

```bash
export LD_LIBRARY_PATH=/usr/local/lib/dds_backup:/usr/local/lib:/unitree/module/unitree_slam/lib:$LD_LIBRARY_PATH
./keyDemo eth0
```

### LiDAR Protection Circuit Tripped

**Symptom:** LiDAR no longer responds to power; web UI unreachable; system cannot detect the sensor.

**Cause:** LiDAR connector was touched or disconnected while the battery was inserted (trips the internal protection circuit).

**Solution:** Perform a full cold boot reset:
1. Power off the robot completely
2. Remove the battery entirely (wait 5-10 minutes to allow the protection circuit to reset)
3. Reinsert the battery
4. Power on and verify LiDAR comes back online

**Critical rule:** Never touch the LiDAR connector with the battery inserted, even if the robot is powered off.

### Jetson Network Issues

**Symptom:** `eth0` becomes wedged; Wi-Fi connection fails or causes crashes.

**Cause:** Running `nmcli connection up` to switch to client mode Wi-Fi corrupts the ethernet state.

**Solution:** Keep the Jetson in AP/hotspot mode (factory default). Connect to the robot's hotspot:
- SSID: `DroneBlocks-Go2-001`
- Password: `00000000`
- Jetson IP in hotspot mode: `10.42.0.1`

To return to normal state if wedged, restart the robot or use Ethernet reconnection.

### Jetson SSH Connection

**Ethernet SSH:**
```bash
ssh unitree@192.168.123.18
# password: 123
```

**Hotspot SSH (wireless):**
```bash
ssh unitree@10.42.0.1
# password: 123
```

### Jetson has no internet access

The Jetson Orin Nano dock is configured without internet connectivity for security. Do not attempt to add internet or configure external routing.

---

## Network Setup

Before running any examples, verify your network configuration:

```bash
ifconfig
# Identify your network interface (typically eth0, enp2s0, enp58s0, or wlan0)
```

Replace the interface name in all commands accordingly:
- `eth0` — primary ethernet interface
- `enp2s0`, `enp58s0` — alternative ethernet names
- `wlan0` — Wi-Fi interface (if using Wi-Fi hotspot)

For detailed network configuration, see the [Unitree quick start guide](https://support.unitree.com/home/en/developer/Quick_start).

---

## SDK Examples

```bash
# DDS pub/sub communication test (basic connectivity check)
python3 example/helloworld/publisher.py
python3 example/helloworld/subscriber.py

# High-level sport control (robot motion primitives)
python3 example/go2/high_level/go2_sport_client.py enp2s0

# Front camera stream (requires display server)
python3 example/go2/front_camera/camera_opencv.py enp2s0

# Obstacle avoidance toggle (enable/disable autonomous obstacle handling)
python3 example/obstacles_avoid/obstacles_avoid_switch.py enp2s0
```

---

## Validation and Metrics Collection

### Running autonomous patrol experiments

1. Start all three SLAM components as described above
2. In `keyDemo`, enter mapping mode (`q`) and drive the robot through the inspection area
3. Save the map (`w`)
4. Enter relocation mode (`a`) to localize within the map
5. Position the robot at each QR checkpoint and manually record waypoints (`s`)
6. Execute the autonomous patrol (`d`)

### Collecting performance metrics

The system automatically logs:
- **Localization accuracy** — deviation from QR fiducial ground truth
- **Navigation time** — time to reach each waypoint from previous position
- **Battery usage** — percentage consumed per trial run
- **Radiation measurements** — count-rate per QR-anchored position (when Radiacode is integrated)
- **Change detection** — run-to-run delta in radiation or environmental features

Results are stored in JSON format for post-processing and statistical analysis.

---

## Related Publications and Presentations

- ANS Winter Conference 2026: Accepted summary; full paper in progress
- STARFIRE Workshop presentations: 2025 and 2026 editions
- Validation facility: Irradiated Materials Testing Laboratory (Michigan Memorial Phoenix Project)

---

## Contributing

For questions, issues, or collaboration on this research:
- Repository: [https://github.com/ladaweb/robodog_python](https://github.com/ladaweb/robodog_python)
- Faculty advisor: Prof. Majdi Radaideh (University of Michigan NERS, AIMS Lab)
- Lead researcher: Lada Protcheva (Ph.D. candidate, University of Michigan)

---

## License

[Specify license if applicable]

---

## Acknowledgments

This work was supported by the University of Michigan Nuclear Engineering and Radiological Sciences program and enabled by facilities at the Michigan Memorial Phoenix Project. Special thanks to Prof. Majdi Radaideh for guidance and collaboration.
