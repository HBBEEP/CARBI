#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float16MultiArray
from mpu9250_i2c import *

class CarbiIMU(Node):
    def __init__(self):
        super().__init__('carbi_imu_interface')

        self.imu_publisher = self.create_publisher(Float16MultiArray, '/imu/raw', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            ax,ay,az,wx,wy,wz = mpu6050_conv() 
        except:
            self.get_logger().info("Wait for connection")

        msg = Float16MultiArray()
        msg.data = [ax, ay, az, wx, wy, wx]
        self.imu_publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = CarbiIMU()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
