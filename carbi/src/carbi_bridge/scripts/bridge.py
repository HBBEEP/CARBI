#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import numpy as np
from kinematics import forward_kinematics
import tf_transformations
import tf2_ros
from tf2_ros import TransformBroadcaster

class CarbiBridge(Node):
    def __init__(self):
        super().__init__('carbi_bridge')
        # self.create_subscription(Float32MultiArray, '/imu_raw', self.imu_raw_callback, 10)

        self.create_subscription(Float32MultiArray, '/wheel_vel', self.wheel_vel_callback, 10)
    
        self.odom_publisher = self.create_publisher(Odometry,'/wheel/odom',10) 

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publish_transform = TransformBroadcaster(self)

        self.time_step = 0.01
        self.timer_ = self.create_timer(self.time_step, self.update)


        self.robot_twist = [0.0, 0.0, 0.0]
        self.robot_position = [0.0, 0.0, 0.0] 
        self.wheel_vel = [0.0, 0.0, 0.0, 0.0] 

    def update(self):
        self.calculate_wheel_odometry()
        self.publish_odometry()
    
    # def imu_raw_callback(self, msg):
    #     self.imu_raw = msg.data 

    def wheel_vel_callback(self, msg):
        wheel_vel =  msg.data        
        self.robot_twist = forward_kinematics(wheel_vel)
        
    def calculate_wheel_odometry(self):
        dx = self.robot_twist[0] * self.time_step
        dy = self.robot_twist[1] * self.time_step
        dtheta = self.robot_twist[2] * self.time_step

        self.robot_position[0] += dx
        self.robot_position[1] += dy
        self.robot_position[2] += dtheta
        self.robot_position[2] = np.arctan2(np.sin(self.robot_position[2]), np.cos(self.robot_position[2]))
        
    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.robot_position[0] 
        odom.pose.pose.position.y = self.robot_position[1] 
        odom.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.robot_position[2])

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = self.robot_twist[0]
        odom.twist.twist.linear.y = self.robot_twist[1]
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.robot_twist[2]
        # self.get_logger().info(f" {self.robot_position[0]}, {self.robot_position[1]}")

        self.odom_publisher.publish(odom)

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"

        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.publish_transform.sendTransform(t)

        

def main(args=None):
    rclpy.init(args=args)
    node = CarbiBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()