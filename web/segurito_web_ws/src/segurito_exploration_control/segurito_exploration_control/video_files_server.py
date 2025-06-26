#!/usr/bin/env python3
import os, glob, threading, http.server, socketserver, rclpy
from rclpy.node import Node
from segurito_interfaces.srv import ListVideos
from functools import partial

VIDEO_DIR = os.path.expanduser("~/videos")
PORT      = 8002

class VideoFilesServer(Node):
    def __init__(self):
        super().__init__("video_files_server")
        self.srv = self.create_service(
            ListVideos, "/videos/list", self.cb_list)

        handler = partial(http.server.SimpleHTTPRequestHandler,
                          directory=VIDEO_DIR)
        self.httpd = socketserver.TCPServer(("", PORT), handler)
        threading.Thread(target=self.httpd.serve_forever,
                         daemon=True).start()
        self.get_logger().info(f"Sirviendo vídeos en http://<robot>:{PORT}/")

    def cb_list(self, req, res):
        files = glob.glob(os.path.join(VIDEO_DIR, "*.mp4"))
        res.basenames = [os.path.splitext(os.path.basename(f))[0] for f in files]
        self.get_logger().info(f"Listados {len(res.basenames)} vídeos")
        return res

def main():
    rclpy.init()
    rclpy.spin(VideoFilesServer())

if __name__ == "__main__":
    main()
