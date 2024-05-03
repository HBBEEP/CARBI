#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import time


class CarbiCamera(Node):
    def __init__(self):
        super().__init__("intel_publisher")
        self.intel_publisher_rgb = self.create_publisher(Image, "rgb_frame", 10)

        timer_period = 0.05
        self.br_rgb = CvBridge()

        try:
            # ctx = rs.context()
            # devices = ctx.query_devices()
            # for dev in devices:
            #     dev.hardware_reset()
            self.pipe = rs.pipeline()
            self.cfg  = rs.config()
            # self.cfg.enable_stream(rs.stream.depth, 640,480, rs.format.z16, 15)
            self.cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 15)

            self.pipe.start(self.cfg)
            self.timer = self.create_timer(timer_period, self.timer_callback)
        except Exception as e:
            print(e)
            self.get_logger().error("INTEL REALSENSE IS NOT CONNECTED")

    def timer_callback(self):
        frames = self.pipe.wait_for_frames()
        # color_frame = frames.get_depth_frame()
        color_frame = frames.get_depth_frame()

        color_image = np.asanyarray(color_frame.get_data())

        self.intel_publisher_rgb.publish(self.br_rgb.cv2_to_imgmsg(color_image))
        self.get_logger().info("Publishing rgb frame")
        cv2.imshow("image",color_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    image_publisher = CarbiCamera()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


