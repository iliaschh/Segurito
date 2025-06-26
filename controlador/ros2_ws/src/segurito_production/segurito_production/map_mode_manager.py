#!/usr/bin/env python3
import os, subprocess, rclpy, signal
from rclpy.node   import Node
from std_srvs.srv import Trigger, SetBool
from nav2_msgs.srv import LoadMap
from std_msgs.msg  import Bool, String

PKG = "segurito_production"
SLAM_CMD = ["ros2", "launch", PKG, "mapping_launch.py"]
LOC_CMD  = ["ros2", "launch", PKG, "localization_launch.py"]

class MapModeManager(Node):
    def __init__(self):
        super().__init__("map_mode_manager")

        # servicios
        self.create_service(Trigger,  "/map_mode/set_explore", self.cb_set_explore)
        self.create_service(LoadMap,  "/map_mode/load_map",    self.cb_load_map)
        self.create_service(Trigger,  "/map_mode/stop",        self.cb_stop_all)
        self.create_service(SetBool,  "/robot/power",          self.cb_power)

        # publishers
        self.power_pub = self.create_publisher(Bool,   "/robot/powered", 10)
        self.mode_pub  = self.create_publisher(String, "/robot/mode",    10)

        self.launch_proc = None
        self.set_power(True)      
        self.set_mode("idle")

    def set_power(self, on: bool):
        self.power_pub.publish(Bool(data=on))

    def set_mode(self, mode: str):
        self.mode_pub.publish(String(data=mode))

    def kill_launch(self):
        if self.launch_proc and self.launch_proc.poll() is None:
            self.get_logger().info("Deteniendo proceso Nav2…")
            os.killpg(os.getpgid(self.launch_proc.pid), signal.SIGINT)
            self.launch_proc.wait()
        self.launch_proc = None

    def cb_set_explore(self, req, resp):
        self.kill_launch()
        self.launch_proc = subprocess.Popen(SLAM_CMD, preexec_fn=os.setsid)
        self.set_mode("exploring")
        resp.success, resp.message = True, "Exploración iniciada"
        return resp

    def cb_load_map(self, req, resp):
        yaml_path = req.map_url if req.map_url.endswith(".yaml") else f"{req.map_url}.yaml"
        if not os.path.exists(yaml_path):
            resp.success, resp.error_string = False, "Mapa no encontrado"
            return resp
        self.kill_launch()
        cmd = LOC_CMD + [f"map:={yaml_path}"]
        self.launch_proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.set_mode("patrolling")  
        resp.success, resp.message = True, f"Mapa cargado: {yaml_path}"
        return resp

    def cb_stop_all(self, req, resp):
        self.kill_launch()
        self.set_mode("idle")
        resp.success, resp.message = True, "Procesos Nav2 detenidos"
        return resp

    def cb_power(self, req, resp):
        if req.data:                         
            self.set_power(True)
            self.set_mode("idle")
            resp.success, resp.message = True, "Robot encendido"
        else:                                 # apagar
            self.kill_launch()
            self.set_power(False)
            self.set_mode("off")
            resp.success, resp.message = True, "Robot apagado"
        return resp

def main():
    rclpy.init()
    node = MapModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.kill_launch()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
