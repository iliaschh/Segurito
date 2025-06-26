#!/usr/bin/env python3
"""
imu_node.py  – Publica sensor_msgs/Imu compensando bias

• Sensor   : MPU-6050 (I²C, dir. 0x68)
• Frecuencia: 50 Hz (param. «rate_hz»)
• Calibración: promedio los primeros N muestras (param. «calib_sec»)
"""

import rclpy, math, time
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import smbus2

MPU_ADDR            = 0x68
REG_PWR_MGMT_1      = 0x6B
REG_ACCEL_CONFIG    = 0x1C  
REG_GYRO_CONFIG     = 0x1B  
REG_ACCEL_START     = 0x3B   
ACC_LSB   = 16384.0         
GYRO_LSB  = 131.0         
DEG2RAD   = math.pi / 180.0
G         = 9.80665

bus = smbus2.SMBus(1)

# devuelve ax ay az t gx gy gz como enteros
def _read_vector14():
    raw = bus.read_i2c_block_data(MPU_ADDR, REG_ACCEL_START, 14)
    def s16(i):
        v = (raw[i] << 8) | raw[i+1]
        return v - 65536 if v > 32767 else v
    return tuple(s16(i) for i in range(0, 14, 2))

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu')

        self.rate_hz = 50.0 
        self.calib_sec = 2.0
        bus.write_byte_data(MPU_ADDR, REG_PWR_MGMT_1, 0x00)    
        bus.write_byte_data(MPU_ADDR, REG_ACCEL_CONFIG, 0x00)  
        bus.write_byte_data(MPU_ADDR, REG_GYRO_CONFIG,  0x00)  
        time.sleep(0.1)

        # calculo necesario ya que la imu no está 100% centrada
        # restamos el bias de cada eje
        n = int(self.calib_sec * self.rate_hz)
        sums = [0.0]*6   # Ax Ay Az Gx Gy Gz
        self.get_logger().info(f'Calibrando IMU ({n} muestras)…')
        for _ in range(n):
            ax, ay, az, _t, gx, gy, gz = _read_vector14()
            for i, v in enumerate((ax, ay, az, gx, gy, gz)):
                sums[i] += v
            time.sleep(1.0 / self.rate_hz)

        self.bias_ax = sums[0] / n
        self.bias_ay = sums[1] / n
        self.bias_az = sums[2] / n - ACC_LSB  
        self.bias_gx, self.bias_gy, self.bias_gz = (s/n for s in sums[3:])

        self.pub   = self.create_publisher(Imu, '/imu', 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

    def tick(self):
        ax, ay, az, _t, gx, gy, gz = _read_vector14()

        ax_mps2 = (ax - self.bias_ax) / ACC_LSB * G
        ay_mps2 = (ay - self.bias_ay) / ACC_LSB * G
        az_mps2 = (az - self.bias_az) / ACC_LSB * G

        gx_rads = (gx - self.bias_gx) / GYRO_LSB * DEG2RAD
        gy_rads = (gy - self.bias_gy) / GYRO_LSB * DEG2RAD
        gz_rads = (gz - self.bias_gz) / GYRO_LSB * DEG2RAD

        # formato que pide ros2
        msg = Imu()
        msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id='imu_link'
        )
        msg.linear_acceleration.x = ax_mps2
        msg.linear_acceleration.y = ay_mps2
        msg.linear_acceleration.z = az_mps2
        msg.angular_velocity.x = gx_rads
        msg.angular_velocity.y = gy_rads
        msg.angular_velocity.z = gz_rads
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 0.0        
        msg.orientation_covariance[0] = -1.0

        accel_var = (0.03 * G)**2     
        gyro_var  = (0.5 * DEG2RAD)**2 
        for i in (0, 4, 8):
            msg.linear_acceleration_covariance[i] = accel_var
            msg.angular_velocity_covariance[i]    = gyro_var

        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(ImuNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
