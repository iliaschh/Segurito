import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from vision_msgs.msg import Detection2DArray
from std_srvs.srv import SetBool

class PersonStop(Node):
    def __init__(self):
        super().__init__("person_stop")

        self.sub = self.create_subscription(
            Detection2DArray, "/people_detections",
            self.cb_detections, 10)

        #self.cli = self.create_client(SetBool,"/controller_server/set_paused")

        self.declare_parameter("pause_time",   3.0)  
        self.declare_parameter("cooldown",     1.0)  

        self.paused      = False
        self.last_seen   = self.get_clock().now()

        self.create_timer(0.2, self.check_resume)

    def cb_detections(self, msg: Detection2DArray):
        if msg.detections:
            self.last_seen = self.get_clock().now()
            if not self.paused:
                self.pause_nav(True)

    def check_resume(self):
        if not self.paused:
            return
        t_no_person = (self.get_clock().now() - self.last_seen).nanoseconds * 1e-9
        if t_no_person >= self.get_parameter("cooldown").value:
            self.pause_nav(False)
    """
    def pause_nav(self, value: bool):
        if not self.cli.service_is_ready():
            self.get_logger().warn("controller_server/set_paused no disponible")
            return
        req = SetBool.Request(data=value)
        future = self.cli.call_async(req)
        future.add_done_callback(
            lambda _: self._after_pause(value))
    """
    def _after_pause(self, paused: bool):
        state = "PAUSADO" if paused else "REANUDADO"
        self.paused = paused
        self.get_logger().info(f"Nav2 {state}")

def main():
    rclpy.init()
    rclpy.spin(PersonStop())

if __name__ == "__main__":
    main()
