#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# drive_base.py  —  ROS 2 Humble
# Escucha /cmd_vel y gobierna un puente TB6612FNG por GPIO.
# Incluye "trim" por rueda para compensar motores desiguales.
# ─────────────────────────────────────────────────────────────────────────────

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class DiffDriveGPIO(Node):
    def __init__(self):
        super().__init__("diff_drive_gpio")
        self.declare_parameter("wheel_radius",   0.0155)  
        self.declare_parameter("track_width",    0.108)   
        self.declare_parameter("pwm_frequency",  1000)     
        self.declare_parameter("max_speed_mps",  0.25)   
        self.PWMA = 13
        self.PWMB = 12
        self.AIN1 = 6
        self.AIN2 = 26
        self.BIN1 = 16
        self.BIN2 = 20
        self.STBY = 5
        # motores descompensados
        self.declare_parameter("left_trim",  -1.00) 
        self.declare_parameter("right_trim", 1.00)
        p = self.get_parameter
        self.track     = p("track_width").value
        self.max_v     = p("max_speed_mps").value
        self.lt        = p("left_trim").value
        self.rt        = p("right_trim").value

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(
            [self.AIN1, self.AIN2, self.BIN1, self.BIN2, self.STBY],
            GPIO.OUT,
            initial=GPIO.LOW,
        )
        GPIO.setup([self.PWMA, self.PWMB], GPIO.OUT, initial=GPIO.LOW)
        # controlar la velocidad máxima
        self.k_pwm = 100.0 / self.max_v
        self.pwmA = GPIO.PWM(self.PWMA, p("pwm_frequency").value)
        self.pwmB = GPIO.PWM(self.PWMB, p("pwm_frequency").value)
        self.pwmA.start(0)
        self.pwmB.start(0)
        GPIO.output(self.STBY, GPIO.HIGH) 

        self.create_subscription(Twist, "cmd_vel", self.cmd_event, 10)

        self.get_logger().info(
            "Diff-drive listo."
        )

    def cmd_event(self, msg: Twist):
        v  = msg.linear.x
        w  = msg.angular.z
        v_l = v - (w * self.track / 2.0)
        v_r = v + (w * self.track / 2.0)

        pwm_l = self.lt * max(min(v_l * self.k_pwm, 100.0), -100.0)
        pwm_r = self.rt * max(min(v_r * self.k_pwm, 100.0), -100.0)
        print("PWM A: ", pwm_l)
        print("PWM B: ", pwm_r)
        self.set_motor(self.AIN1, self.AIN2, self.pwmA, pwm_l)
        self.set_motor(self.BIN1, self.BIN2, self.pwmB, pwm_r)

    @staticmethod
    def set_motor(in1, in2, pwm, value):
        GPIO.output(in1, value > 0)
        GPIO.output(in2, value < 0)
        pwm.ChangeDutyCycle(abs(value))

    def destroy_node(self):
        self.set_motor(self.AIN1, self.AIN2, self.pwmA, 0)
        self.set_motor(self.BIN1, self.BIN2, self.pwmB, 0)
        self.pwmA.stop()
        self.pwmB.stop()
        GPIO.output(self.STBY, GPIO.LOW)
        GPIO.cleanup()
        super().destroy_node()

def main():
    rclpy.init()
    node = DiffDriveGPIO()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
