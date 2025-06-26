import os, threading, http.server, socketserver, rclpy
from rclpy.node import Node
from segurito_interfaces.srv import ListMaps
from functools import partial

MAP_DIR = os.path.expanduser("~/maps")
PORT    = 8001 

class MapFilesServer(Node):
    def __init__(self):
        super().__init__("map_files_server")

        self.srv = self.create_service(
            ListMaps, "/maps/list", self.cb_list_maps)

        handler = partial(http.server.SimpleHTTPRequestHandler,
                          directory=MAP_DIR)
        self.httpd = socketserver.TCPServer(("", PORT), handler)

        th = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        th.start()
        self.get_logger().info(f"Sirviendo mapas en http://<robot>:{PORT}/")

    def cb_list_maps(self, request, response):
        bases = []
        for entry in os.listdir(MAP_DIR):
            if entry.endswith(".yaml"):
                bases.append(os.path.splitext(entry)[0])  
        response.basenames = sorted(bases)
        return response

def main():
    rclpy.init()
    rclpy.spin(MapFilesServer())

if __name__ == "__main__":
    main()
