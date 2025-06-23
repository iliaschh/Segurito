#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def main():
    rclpy.init()
    node = rclpy.create_node('static_laser_tf')

    broadcaster = StaticTransformBroadcaster(node)

    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = 'base_link'
    t.child_frame_id  = 'laser'

    t.transform.translation.x = 0.00
    t.transform.translation.y = 0.00
    t.transform.translation.z = 0.1928

    # rotación: giro de 180° en Z <-- ya no... 
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 1.0  
    t.transform.rotation.w = 0.0   

    broadcaster.sendTransform(t)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
