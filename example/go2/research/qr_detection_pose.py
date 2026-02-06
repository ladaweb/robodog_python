import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient


class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.latest_pose = None
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        self.latest_pose = msg.pose.pose


def main():
    # ROS2 init
    rclpy.init()
    odom_node = OdomListener()

    # Connect to Go2 video stream
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    print("[INFO] ChannelFactoryInitialize done")

    client = VideoClient()
    client.SetTimeout(3.0)
    client.Init()
    print("[INFO] VideoClient initialized")

    qr = cv2.QRCodeDetector()
    last_seen_text = None

    print("[INFO] Press ESC to quit.")
    poses = {}

    try:
        while rclpy.ok():
            rclpy.spin_once(odom_node, timeout_sec=0.01)

            code, data = client.GetImageSample()
            if code != 0 or data is None:
                continue

            img_bytes = np.frombuffer(bytes(data), dtype=np.uint8)
            frame = cv2.imdecode(img_bytes, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            text, points, _ = qr.detectAndDecode(frame)

            if text:
                if text != last_seen_text:
                    print(f"[QR DETECTED] {text}")
                    last_seen_text = text

                    if odom_node.latest_pose:
                        pose = odom_node.latest_pose
                        poses[text] = pose
                        print(f"[POSE SAVED] QR '{text}' at x={pose.position.x:.2f}, y={pose.position.y:.2f}")

                if points is not None:
                    pts = points.reshape(-1, 2).astype(int)
                    for i in range(4):
                        p1 = tuple(pts[i])
                        p2 = tuple(pts[(i + 1) % 4])
                        cv2.line(frame, p1, p2, (0, 255, 0), 2)

            cv2.imshow("Go2 QR Detection", frame)
            if cv2.waitKey(10) == 27:
                break

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C pressed. Exiting...")

    cv2.destroyAllWindows()
    odom_node.destroy_node()
    rclpy.shutdown()

    print("\n[INFO] Final QR → Pose mapping:")
    for tag, pose in poses.items():
        print(f"{tag}: x={pose.position.x:.2f}, y={pose.position.y:.2f}, z={pose.position.z:.2f}")


if __name__ == "__main__":
    main()