#!/usr/bin/env python3
import os, cv2, rclpy, datetime
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np

class VideoRecorder(Node):
    def __init__(self):
        super().__init__("video_recorder")

        folder = os.path.expanduser("~/videos")
        os.makedirs(folder, exist_ok=True)
        ts      = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.fn = os.path.join(folder, f"video_{ts}.avi")

        self.writer   = None
        self.prev_sec = None
        self.fps_est  = 30  

        self.create_subscription(
            CompressedImage, "/image_raw/compressed",
            self.cb_frame, 10)

        self.get_logger().info(f"Grabando a {self.fn}")

    def cb_frame(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if self.writer is None:
            h, w, _ = frame.shape
            fourcc  = cv2.VideoWriter_fourcc(*"XVID")
            self.writer = cv2.VideoWriter(self.fn, fourcc,
                                          self.fps_est, (w, h))
            self.prev_sec = msg.header.stamp.sec

        dt = msg.header.stamp.sec - self.prev_sec
        if dt > 0:
            self.fps_est = 1.0 / dt
            self.prev_sec = msg.header.stamp.sec

        self.writer.write(frame)

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
            self.get_logger().info(f"Vídeo guardado en {self.fn}")
        super().destroy_node()

def main():
    rclpy.init()
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
