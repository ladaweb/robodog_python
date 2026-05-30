from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient

import cv2
import numpy as np
import sys
import os

def main():
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

    while True:
        code, data = client.GetImageSample()
        if code != 0 or data is None:
            continue

        img_bytes = np.frombuffer(bytes(data), dtype=np.uint8)
        frame = cv2.imdecode(img_bytes, cv2.IMREAD_COLOR)
        if frame is None:
            continue

        text, points, _ = qr.detectAndDecode(frame)  # <-- Make sure this is inside the loop

        if text:
            if text != last_seen_text:
                print(f"[QR DETECTED] {text}")
                last_seen_text = text

                with open("/tmp/qr_detected.txt", "w") as f:
                    f.write(text)

            if points is not None:
                pts = points.reshape(-1, 2).astype(int)
                for i in range(4):
                    pt1 = tuple(pts[i])
                    pt2 = tuple(pts[(i + 1) % 4])
                    cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

        cv2.imshow("QR Detection", frame)
        if cv2.waitKey(10) == 27:
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C pressed. Exiting cleanly.")
        cv2.destroyAllWindows()
