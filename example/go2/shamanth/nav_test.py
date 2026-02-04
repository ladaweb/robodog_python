#!/usr/bin/env python3
"""
Waypoint navigation test for Unitree Go2.

Pure Unitree SDK2 over DDS — no ROS 2.
Reads waypoints from a CSV file (x,y,label per line), navigates to each
sequentially using a proportional controller, and logs whether each was
reached or timed out.

Position is read from the rt/sportmodestate DDS topic.
Motion commands are sent via SportClient.Move().

Usage:
    python3 nav_test.py waypoints.csv                     # auto-detect interface
    python3 nav_test.py waypoints.csv --interface wlan0   # specify interface
"""

import math
import sys
import time
import argparse
import collections
import threading

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import SportClient

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
REACH_THRESHOLD = 0.3       # meters
WAYPOINT_TIMEOUT = 30.0     # seconds
MAX_VX = 0.3                # m/s
MAX_VYAW = 0.5              # rad/s
KP_LINEAR = 0.5             # proportional gain for distance
KP_ANGULAR = 1.0            # proportional gain for heading error
HEADING_THRESHOLD = 0.3     # rad — rotate in place when error exceeds this
CONTROL_DT = 0.1            # seconds (10 Hz control loop)

Waypoint = collections.namedtuple('Waypoint', ['x', 'y', 'label'])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))


def parse_csv(csv_path):
    """Parse a CSV file with lines of ``x,y,label``. Skips blanks and # comments."""
    waypoints = []
    with open(csv_path, 'r') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',', maxsplit=2)
            if len(parts) != 3:
                raise ValueError(
                    f"Line {line_num}: expected 'x,y,label', got: {line}"
                )
            x, y, label = float(parts[0]), float(parts[1]), parts[2].strip()
            waypoints.append(Waypoint(x, y, label))
    return waypoints


# ---------------------------------------------------------------------------
# Navigation
# ---------------------------------------------------------------------------
class NavTest:
    def __init__(self, csv_path, interface):
        # --- DDS setup ---
        if interface:
            ChannelFactoryInitialize(0, interface)
        else:
            ChannelFactoryInitialize(0)
        print(f"[nav] DDS initialized" + (f" on {interface}" if interface else ""))

        # --- SportClient for motion commands ---
        self.sport = SportClient()
        self.sport.SetTimeout(10.0)
        self.sport.Init()
        print("[nav] SportClient initialized")

        # --- Subscribe to rt/sportmodestate for position ---
        self.state_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.state_sub.Init(self._state_handler, 10)
        print("[nav] Subscribed to rt/sportmodestate")

        # --- Waypoints ---
        self.waypoints = parse_csv(csv_path)
        if not self.waypoints:
            print("[nav] ERROR: no waypoints found in CSV")
            sys.exit(1)
        print(f"[nav] Loaded {len(self.waypoints)} waypoints from {csv_path}")

        # --- State (updated by DDS callback thread) ---
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_yaw = 0.0
        self.state_received = False
        self._lock = threading.Lock()

        # --- Results ---
        self.results = []

    def _state_handler(self, msg: SportModeState_):
        """DDS callback — runs on subscriber thread."""
        with self._lock:
            self.cur_x = msg.position[0]
            self.cur_y = msg.position[1]
            self.cur_yaw = msg.imu_state.rpy[2]  # yaw from IMU
            self.state_received = True

    def _get_pose(self):
        with self._lock:
            return self.cur_x, self.cur_y, self.cur_yaw, self.state_received

    def run(self):
        """Main control loop — runs on the main thread."""

        # Wait for first state message
        print("[nav] Waiting for rt/sportmodestate ...")
        while True:
            _, _, _, received = self._get_pose()
            if received:
                break
            time.sleep(0.1)

        x, y, yaw, _ = self._get_pose()
        print(f"[nav] First state: x={x:.2f}  y={y:.2f}  yaw={math.degrees(yaw):.1f}°")

        # Navigate to each waypoint
        for i, wp in enumerate(self.waypoints):
            print(f"[nav] [{i+1}/{len(self.waypoints)}] Navigating to {wp.label} ({wp.x:.2f}, {wp.y:.2f})")
            result = self._go_to_waypoint(wp)
            self.results.append(result)

        # Done
        self.sport.StopMove()
        self._print_summary()

    def _go_to_waypoint(self, wp):
        start = time.monotonic()

        while True:
            x, y, yaw, _ = self._get_pose()
            dx = wp.x - x
            dy = wp.y - y
            distance = math.sqrt(dx * dx + dy * dy)
            elapsed = time.monotonic() - start

            # Reached?
            if distance < REACH_THRESHOLD:
                self.sport.StopMove()
                print(f"  -> REACHED {wp.label} in {elapsed:.1f}s (dist={distance:.2f}m)")
                return {'label': wp.label, 'status': 'reached', 'elapsed': elapsed}

            # Timeout?
            if elapsed > WAYPOINT_TIMEOUT:
                self.sport.StopMove()
                print(f"  -> TIMEOUT on {wp.label} after {elapsed:.1f}s (dist remaining={distance:.2f}m)")
                return {'label': wp.label, 'status': 'timeout', 'elapsed': elapsed}

            # Proportional controller
            target_yaw = math.atan2(dy, dx)
            heading_error = math.atan2(
                math.sin(target_yaw - yaw),
                math.cos(target_yaw - yaw),
            )

            if abs(heading_error) > HEADING_THRESHOLD:
                vx = 0.0
                vyaw = clamp(KP_ANGULAR * heading_error, -MAX_VYAW, MAX_VYAW)
            else:
                vx = clamp(KP_LINEAR * distance, 0.0, MAX_VX)
                vyaw = clamp(KP_ANGULAR * heading_error, -MAX_VYAW, MAX_VYAW)

            self.sport.Move(vx, 0.0, vyaw)
            time.sleep(CONTROL_DT)

    def _print_summary(self):
        print("\n========== NAVIGATION SUMMARY ==========")
        reached = 0
        for r in self.results:
            tag = 'OK' if r['status'] == 'reached' else 'FAIL'
            print(f"  [{tag}] {r['label']}: {r['status']} ({r['elapsed']:.1f}s)")
            if r['status'] == 'reached':
                reached += 1
        print(f"Result: {reached}/{len(self.results)} waypoints reached")
        print("=========================================\n")

    def stop(self):
        """Emergency stop — call on Ctrl+C."""
        self.sport.Move(0.0, 0.0, 0.0)
        self.sport.StopMove()


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='Go2 waypoint navigation test')
    parser.add_argument('csv_file', help='CSV file with x,y,label waypoints')
    parser.add_argument(
        '--interface', default='wlan0',
        help='Network interface for DDS (default: wlan0)',
    )
    args = parser.parse_args()

    nav = NavTest(args.csv_file, args.interface)

    try:
        nav.run()
    except KeyboardInterrupt:
        print("\n[nav] Ctrl+C — stopping robot")
    finally:
        nav.stop()


if __name__ == '__main__':
    main()
