#!/usr/bin/env python3
import math
import threading
import time

import RPi.GPIO as GPIO     
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


class QuadratureEncoder:
    _lookup = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0]

    def __init__(self, pin_a: int, pin_b: int):
        self._pin_a = pin_a
        self._pin_b = pin_b
        self._ticks = 0
        self._last_state = 0
        self._lock = threading.Lock()
        GPIO.cleanup((pin_a, pin_b))
        for p in (pin_a, pin_b):
            GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(p, GPIO.BOTH, callback=self._callback)

        self._last_state = (GPIO.input(pin_a) << 1) | GPIO.input(pin_b)

    @property
    def ticks(self) -> int:
        with self._lock:
            return self._ticks

    def reset(self) -> int:
        with self._lock:
            t = self._ticks
            self._ticks = 0
            return t

    def _callback(self, channel):
        a = GPIO.input(self._pin_a)
        b = GPIO.input(self._pin_b)
        curr = (a << 1) | b
        idx = (self._last_state << 2) | curr
        delta = self._lookup[idx]
        if delta:
            with self._lock:
                self._ticks += delta
        self._last_state = curr


class OdomNode(Node):
    def __init__(self):
        super().__init__('odometria')

        GPIO.setwarnings(False)        
        GPIO.setmode(GPIO.BCM)
        
        self.R = self.declare_parameter('wheel_radius', 0.0168).value # diametro rueda   1.096 factor de correccion
        self.L = self.declare_parameter('track_width', 0.117).value  # distancia entre centros de la rueda
        self.N = self.declare_parameter('ticks_per_rev', 900).value    
        period_ms = self.declare_parameter('period_ms', 20).value
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        self.dt = period_ms / 1000.0

        GPIO.setmode(GPIO.BCM)
        self.enc_left  = QuadratureEncoder(27, 22) 
        self.enc_right = QuadratureEncoder(17, 25)

        qos = QoSProfile(
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None

        # pose
        self.x = self.y = self.th = 0.0
        self.timer = self.create_timer(self.dt, self.update)

    def update(self):
        dl_ticks = self.enc_left.reset()
        dr_ticks = self.enc_right.reset()

        m_per_tick = (2 * math.pi * self.R) / self.N
        #  80 horas más tarde me arrepiento de no haber invertido los cables directamente...
        dl = dl_ticks * m_per_tick * -1 #invertir
        dr = dr_ticks * m_per_tick * -1 #invertir
        ds = 0.5 * (dl + dr)
        dth = (dl - dr) / self.L 

        if dl_ticks == dr_ticks == 0:
            # si no hay movimiento solo actualizamos el campo de tiempo
            vx = 0.0
            wz = 0.0
        else:
            self.x += ds * math.cos(self.th + 0.5 * dth)
            self.y += ds * math.sin(self.th + 0.5 * dth)
            self.th += dth
            vx = ds / self.dt
            wz = dth / self.dt

        stamp = self.get_clock().now().to_msg()
        #stamp = (self.get_clock().now() - rclpy.duration.Duration(seconds=0.05)).to_msg()
        # formato que pide ros2
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        # odom --> base_link
        if self.tf_pub:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_pub.sendTransform(tf)

    def destroy_node(self):
        super().destroy_node()
        GPIO.cleanup()     


def main():
    rclpy.init()
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
