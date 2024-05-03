#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
from cv_bridge import CvBridge
from kinematics import inverse_kinematics, forward_kinematics

class CarbiBridge(Node):
    def __init__(self):
        super().__init__('carbi_bridge')
        self.create_subscription(Float32MultiArray, '/imu_raw', self.imu_raw_callback, 10)
        self.image_sub = self.create_subscription(Image, '/image/rgb', self.image_rgb_callback,10)
        self.cv_bridge = CvBridge()

    def imu_raw_callback(self, msg):
        self.imu_raw = msg.data 

    def image_rgb_callback(self, msg):
        current_frame = self.br.imgmsg_to_cv2(msg)

        cv2.imshow("face detection", current_frame)
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    node = CarbiBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()