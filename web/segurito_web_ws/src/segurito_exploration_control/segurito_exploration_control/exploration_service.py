#!/usr/bin/env python3
import rclpy, subprocess, signal, os
from rclpy.node   import Node
from std_srvs.srv import SetBool

CMD = ["ros2", "launch", "segurito_production", "mapping_launch.py"]

class ExplorationService(Node):
    def __init__(self):
        super().__init__("exploration_service")
        self.srv = self.create_service(SetBool,"/explore/is_exploring",
                                       self.callback)
        self.proc = None  
    def callback(self, req, res):
        if req.data:                                
            if self.proc is None or self.proc.poll() is not None:
                self.get_logger().info("INiciando exploración...")
                self.proc = subprocess.Popen(CMD,stdout=subprocess.DEVNULL,
                    stderr=subprocess.STDOUT,preexec_fn=os.setsid) 
                res.success, res.message = True, "Exploración iniciada"
            else:
                res.success, res.message = False, "Ya está explorando"
        else:                                       
            if self.proc and self.proc.poll() is None:
                self.get_logger().info("Deteniendo exploración...")
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
                self.proc.wait(timeout=5)
                self.proc = None
                res.success, res.message = True, "Exploración detenida"
            else:
                res.success, res.message = False, "No está explorando"
        return res

def main():
    rclpy.init()
    node = ExplorationService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
