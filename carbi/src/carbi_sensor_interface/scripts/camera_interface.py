#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import time


class CarbiCameraPubisher(Node):
    def __init__(self):
        super().__init__("carbi_camera_pub")
        self.rgb_publisher = self.create_publisher(Image, "img/rgb", 10)
        self.depth_publisher = self.create_publisher(Image, "img/depth", 10)


        self.rtab_depth_publisher = self.create_publisher(Image, '/camera/depth_registered/image_raw', 10)
        self.rtab_rgb_publisher = self.create_publisher(Image, '/camera/rgb/image_rect_color', 10) 
        timer_period = 0.01
        self.br_rgb = CvBridge()

        try:
            self.pipe = rs.pipeline()
            self.cfg  = rs.config()
            self.cfg.enable_stream(rs.stream.depth, 640,480, rs.format.z16, 30)
            self.cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30)

            self.pipe.start(self.cfg)
            self.timer = self.create_timer(timer_period, self.timer_callback)
        except Exception as e:
            print(e)
            self.get_logger().error("INTEL REALSENSE IS NOT CONNECTED")

    def timer_callback(self):
        frames = self.pipe.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,
                                     alpha = 0.5), cv2.COLORMAP_JET)

        self.rgb_publisher.publish(self.br_rgb.cv2_to_imgmsg(color_image))
        self.depth_publisher.publish(self.br_rgb.cv2_to_imgmsg(depth_cm))    


        self.rtab_rgb_publisher.publish(self.br_rgb.cv2_to_imgmsg(color_image))
        self.rtab_depth_publisher.publish(self.br_rgb.cv2_to_imgmsg(depth_cm))    





def main(args=None):
    rclpy.init(args=args)
    image_publisher = CarbiCameraPubisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


