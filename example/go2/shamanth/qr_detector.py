#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient

import cv2
import numpy as np
import sys


class QRDetector(Node):
    def __init__(self):
        super().__init__('qr_detector')

        # Publisher
        self.qr_pub = self.create_publisher(String, '/qr_detected', 10)

        # Init Unitree video
        if len(sys.argv) > 1:
            ChannelFactoryInitialize(0, sys.argv[1])
        else:
            ChannelFactoryInitialize(0)

        self.get_logger().info("ChannelFactoryInitialize done")

        self.client = VideoClient()
        self.client.SetTimeout(3.0)
        self.client.Init()
        self.get_logger().info("VideoClient initialized")

        self.qr = cv2.QRCodeDetector()
        self.last_seen_text = None

        # Timer loop (10 Hz)
        self.timer = self.create_timer(0.1, self.process_frame)

        self.get_logger().info("QR Detector node started")

    def process_frame(self):
        code, data = self.client.GetImageSample()
        if code != 0 or data is None:
            return

        img_bytes = np.frombuffer(bytes(data), dtype=np.uint8)
        frame = cv2.imdecode(img_bytes, cv2.IMREAD_COLOR)
        if frame is None:
            return

        text, points, _ = self.qr.detectAndDecode(frame)

        if text and text != self.last_seen_text:
            self.last_seen_text = text

            msg = String()
            msg.data = text
            self.qr_pub.publish(msg)

            self.get_logger().info(f"QR detected: {text}")

        # Optional visualization (safe to keep)
        if points is not None:
            pts = points.reshape(-1, 2).astype(int)
            if pts.shape[0] >= 4:
                for i in range(4):
                    p1 = tuple(pts[i])
                    p2 = tuple(pts[(i + 1) % 4])
                    cv2.line(frame, p1, p2, (0, 255, 0), 2)

        cv2.imshow("Go2 Front Camera - QR Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = QRDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()