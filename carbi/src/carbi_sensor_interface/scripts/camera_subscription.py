#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CarbiCameraSubcriber(Node):
    def __init__(self):
        super().__init__('carbi_camera_sub')
        self.subscription_rgb = self.create_subscription(Image, "img/rgb", self.rgb_frame_callback, 10)
        self.subscription_rgb = self.create_subscription(Image, "img/depth", self.depth_frame_callback, 10)
        self.depth_publisher = self.create_publisher(Image, '/camera/depth_registered/image_raw', 10)
        self.rgb_publisher = self.create_publisher(Image, '/camera/rgb/image_rect_color', 10)
        self.br_rgb = CvBridge()

    def rgb_frame_callback(self, data):
        self.get_logger().warning("Receiving RGB frame")
        current_frame = self.br_rgb.imgmsg_to_cv2(data)
        cv2.imshow("RGB", current_frame)
        cv2.waitKey(1)

    def depth_frame_callback(self, data):
        self.get_logger().warning("Receiving Depth frame")
        current_frame = self.br_rgb.imgmsg_to_cv2(data)
        cv2.imshow("DEPTH", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CarbiCameraSubcriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()