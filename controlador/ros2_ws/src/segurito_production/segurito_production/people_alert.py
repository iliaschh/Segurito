#!/usr/bin/env python3
"""
Nodo ROS 2 que lanza una alerta cuando detecta personas y graba
un clip de vídeo del evento.  El clip se guarda como MP4 (H.264)
mediante una conversión automática con FFmpeg.

Autor: neverdiedooms
"""

import os
import cv2
import shutil
import subprocess
import tempfile
import datetime
import numpy as np

import rclpy
from rclpy.node       import Node
from rclpy.duration   import Duration
from rclpy.time       import Time

from std_msgs.msg     import String
from sensor_msgs.msg  import CompressedImage
from vision_msgs.msg  import Detection2DArray


class PeopleAlert(Node):
    RECORD_SECONDS = 10
    BASE_FOLDER    = os.path.expanduser("~/videos")
    FPS_ESTIMATE   = 15.0

    def __init__(self):
        super().__init__("people_alert")
        self.create_subscription(Detection2DArray,
                                 "/people_detections",
                                 self.cb_detections, 10)
        self.create_subscription(CompressedImage,
                                 "/image_raw/compressed",
                                 self.cb_camera, 10)
        self.alert_pub  = self.create_publisher(String, "/person_alert", 10)
        self.cooldown   = Duration(seconds=3)
        self.last_alert = Time(seconds=0, nanoseconds=0,
                               clock_type=self.get_clock().clock_type)

        self.recording      = False
        self.record_ends_at = Time(seconds=0, nanoseconds=0,
                                   clock_type=self.get_clock().clock_type)
        self.writer      = None   
        self.raw_path    = None    
        self.final_path  = None      
        self.thumb_path  = None   

        os.makedirs(self.BASE_FOLDER, exist_ok=True)

    def cb_detections(self, msg: Detection2DArray):
        if not msg.detections:
            return
        now = self.get_clock().now()
        if (now - self.last_alert) < self.cooldown:
            return
        self.alert_pub.publish(String(data="Persona detectada"))
        self.get_logger().info("Persona detectada")
        self.last_alert = now
        if not self.recording:
            self.start_recording()

    def cb_camera(self, msg: CompressedImage):
        if not self.recording:
            return
        frame = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if self.writer is None:
            self._open_writer(frame)
        if self.writer:
            self.writer.write(frame)

            if self.get_clock().now() > self.record_ends_at:
                self.stop_recording()

    def start_recording(self):
        self.recording = True
        self.writer    = None  

    def _open_writer(self, frame):
        h, w = frame.shape[:2]
        ts   = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        tmp_dir = tempfile.mkdtemp(prefix="alert_", dir=self.BASE_FOLDER)
        self.raw_path   = os.path.join(tmp_dir, f"raw_{ts}.avi")
        self.final_path = os.path.join(self.BASE_FOLDER, f"alert_{ts}.mp4")
        self.thumb_path = self.final_path.replace(".mp4", ".jpg")

        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        self.writer = cv2.VideoWriter(self.raw_path, fourcc,
                                      self.FPS_ESTIMATE, (w, h))
        
        self.get_logger().info(f"Grabando clip: {self.raw_path}")

        # miniatura = primer frame
        cv2.imwrite(self.thumb_path, frame)
        self.get_logger().info(f"Miniatura guardada: {self.thumb_path}")

        self.record_ends_at = self.get_clock().now() + \
                              Duration(seconds=self.RECORD_SECONDS)

    def stop_recording(self):
        if self.writer:
            self.writer.release()
            self.writer = None
            self._convert_with_ffmpeg()
        self.recording = False

    def _convert_with_ffmpeg(self):
        cmd = [
            "ffmpeg", "-y", "-i", self.raw_path,
            "-c:v", "libx264", "-preset", "veryfast",
            "-pix_fmt", "yuv420p", self.final_path
        ]
        try:
            subprocess.run(cmd, check=True,
                           stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)

            shutil.rmtree(os.path.dirname(self.raw_path), ignore_errors=True)
        except subprocess.CalledProcessError:
            self.get_logger().error("FFmpeg")

    def destroy_node(self):
        self.stop_recording()
        super().destroy_node()

def main():
    rclpy.init()
    node = PeopleAlert()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == "__main__":
    main()
