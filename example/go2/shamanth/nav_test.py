#!/usr/bin/env python3
"""
Waypoint navigation test for Unitree Go2.

Reads waypoints from a CSV file (x,y,label per line), navigates to each
sequentially using a proportional controller, and logs whether each was
reached or timed out.

Usage:
    python3 nav_test.py waypoints.csv --interface wlan0       # real robot
    python3 nav_test.py waypoints.csv --sim                   # Gazebo sim
"""

import math
import sys
import argparse
import collections

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

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

Waypoint = collections.namedtuple('Waypoint', ['x', 'y', 'label'])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))


def quaternion_to_yaw(q):
    """Extract yaw from a geometry_msgs Quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


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
# ROS 2 Node
# ---------------------------------------------------------------------------
class NavTestNode(Node):
    def __init__(self, csv_path, interface, sim=False):
        super().__init__('nav_test_node')

        self.sim = sim

        if self.sim:
            # --- Sim mode: publish Twist on /cmd_vel ---
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.sport = None
            self.get_logger().info('Running in SIMULATION mode (/cmd_vel)')
        else:
            # --- Real robot: Unitree SDK setup ---
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.go2.sport.sport_client import SportClient

            ChannelFactoryInitialize(0, interface)
            self.get_logger().info(f'DDS initialized on {interface}')

            self.sport = SportClient()
            self.sport.SetTimeout(10.0)
            self.sport.Init()
            self.get_logger().info('SportClient initialized')

        # --- ROS 2 odometry subscription ---
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # --- Waypoints ---
        self.waypoints = parse_csv(csv_path)
        if not self.waypoints:
            self.get_logger().error('No waypoints found in CSV — exiting')
            raise SystemExit(1)
        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints from {csv_path}'
        )

        # --- State ---
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_yaw = 0.0
        self.odom_received = False

        self.wp_index = 0
        self.wp_start_time = None
        self.results = []  # list of dicts {label, status, elapsed}
        self.finished = False

        # --- Control loop at 10 Hz ---
        self.timer = self.create_timer(0.1, self._control_loop)
        self.get_logger().info('Navigation test started — waiting for /odom')

    # ---- callbacks --------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        self.cur_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(
                f'First odom received: x={self.cur_x:.2f}  '
                f'y={self.cur_y:.2f}  yaw={math.degrees(self.cur_yaw):.1f}°'
            )

    # ---- control loop -----------------------------------------------------
    def _control_loop(self):
        if not self.odom_received:
            return

        if self.finished:
            return

        if self.wp_index >= len(self.waypoints):
            self._finish()
            return

        wp = self.waypoints[self.wp_index]

        # First tick for this waypoint — record start time
        if self.wp_start_time is None:
            self.wp_start_time = self.get_clock().now()
            self.get_logger().info(
                f'[{self.wp_index + 1}/{len(self.waypoints)}] '
                f'Navigating to {wp.label} ({wp.x:.2f}, {wp.y:.2f})'
            )

        dx = wp.x - self.cur_x
        dy = wp.y - self.cur_y
        distance = math.sqrt(dx * dx + dy * dy)
        elapsed = (self.get_clock().now() - self.wp_start_time).nanoseconds / 1e9

        # --- Reached? ---
        if distance < REACH_THRESHOLD:
            self._send_cmd(0.0, 0.0)
            self.get_logger().info(
                f'  -> REACHED {wp.label} in {elapsed:.1f}s '
                f'(dist={distance:.2f}m)'
            )
            self.results.append(
                {'label': wp.label, 'status': 'reached', 'elapsed': elapsed}
            )
            self._advance()
            return

        # --- Timeout? ---
        if elapsed > WAYPOINT_TIMEOUT:
            self._send_cmd(0.0, 0.0)
            self.get_logger().warn(
                f'  -> TIMEOUT on {wp.label} after {elapsed:.1f}s '
                f'(dist remaining={distance:.2f}m)'
            )
            self.results.append(
                {'label': wp.label, 'status': 'timeout', 'elapsed': elapsed}
            )
            self._advance()
            return

        # --- Proportional controller ---
        target_yaw = math.atan2(dy, dx)
        heading_error = math.atan2(
            math.sin(target_yaw - self.cur_yaw),
            math.cos(target_yaw - self.cur_yaw),
        )

        if abs(heading_error) > HEADING_THRESHOLD:
            # Rotate in place
            vx = 0.0
            vyaw = clamp(KP_ANGULAR * heading_error, -MAX_VYAW, MAX_VYAW)
        else:
            # Move forward with yaw correction
            vx = clamp(KP_LINEAR * distance, 0.0, MAX_VX)
            vyaw = clamp(KP_ANGULAR * heading_error, -MAX_VYAW, MAX_VYAW)

        self._send_cmd(vx, vyaw)

    # ---- helpers ----------------------------------------------------------
    def _advance(self):
        self.wp_index += 1
        self.wp_start_time = None

    def _send_cmd(self, vx, vyaw):
        """Send velocity command via /cmd_vel (sim) or SportClient (real)."""
        if self.sim:
            msg = Twist()
            msg.linear.x = vx
            msg.angular.z = vyaw
            self.cmd_pub.publish(msg)
        else:
            if vx == 0.0 and vyaw == 0.0:
                self.sport.StopMove()
            else:
                self.sport.Move(vx, 0.0, vyaw)

    def _finish(self):
        self.finished = True
        self._send_cmd(0.0, 0.0)
        self.timer.cancel()

        self.get_logger().info('========== NAVIGATION SUMMARY ==========')
        reached = 0
        for r in self.results:
            tag = 'OK' if r['status'] == 'reached' else 'FAIL'
            self.get_logger().info(
                f"  [{tag}] {r['label']}: {r['status']} ({r['elapsed']:.1f}s)"
            )
            if r['status'] == 'reached':
                reached += 1
        self.get_logger().info(
            f'Result: {reached}/{len(self.results)} waypoints reached'
        )
        self.get_logger().info('=========================================')

    def stop_robot(self):
        """Ensure robot stops — called on shutdown."""
        self._send_cmd(0.0, 0.0)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description='Go2 waypoint navigation test'
    )
    parser.add_argument('csv_file', help='CSV file with x,y,label waypoints')
    parser.add_argument(
        '--interface', default='wlan0',
        help='Network interface for DDS (default: wlan0)',
    )
    parser.add_argument(
        '--sim', action='store_true',
        help='Simulation mode: publish Twist on /cmd_vel instead of using SportClient',
    )
    parsed = parser.parse_args()

    rclpy.init()
    node = NavTestNode(parsed.csv_file, parsed.interface, sim=parsed.sim)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C — stopping robot')

    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
