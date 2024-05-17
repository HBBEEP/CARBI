#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32MultiArray
import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf_transformations
from geometry_msgs.msg import PointStamped

class CarbiGoToGoal(Node):
    def __init__(self):
        super().__init__('carbi_go_to_goal')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.time_step = 0.01
        self.timer_ = self.create_timer(self.time_step, self.update)
        
        self.target_pos = [0.0, 0.0]
        self.pose = Pose()

        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.set_target_subscription = self.create_subscription(Float32MultiArray, '/target_pos', self.set_target_pos, 10)

        self.clicked_point_subscription = self.create_subscription(PointStamped,'/clicked_point',self.point_callback,10)

    def point_callback(self, msg):
        self.get_logger().info(f"Received point: [{msg.point.x}, {msg.point.y}]")
        self.target_pos[0] = msg.point.x
        self.target_pos[1] = msg.point.y

    def update(self):
        try:
            trans = self.tf_buffer.lookup_transform('base_footprint', 'base_link', rclpy.time.Time())
            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            self.pose = pose
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error('Could not transform from base_link to map: %s' % str(e))
            return None

        msg = self.controller()
        self.twist_publisher.publish(msg)


    def controller(self):
        msg = Twist()

        current_pos = np.array([self.pose.position.x, self.pose.position.y])
        dp = self.target_pos - current_pos
        _, _, theta = tf_transformations.euler_from_quaternion([self.pose.orientation.x,
                                                        self.pose.orientation.y,
                                                        self.pose.orientation.z,
                                                        self.pose.orientation.w])
        e = np.arctan2(dp[1], dp[0]) - theta
        K = 0.50
        cal_w = K * np.arctan2(np.sin(e), np.cos(e))

        opp_x = -1 if dp[0] < 0.0 else 1
        opp_y = -1 if dp[1] < 0.0 else 1
        opp_w = -1 if cal_w < 0.0 else 1

        vx = dp[0] if abs(dp[0]) < 0.2 else 0.2 * opp_x
        vy = dp[1] if abs(dp[1]) < 0.2 else 0.2 * opp_y
        w = cal_w if abs(cal_w) < 0.1 else 0.1 * opp_w

        if np.linalg.norm(dp) < 0.1:
            vx = 0.0
            vy = 0.0
            w = 0.0

        print(vx,vy,w)
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = w
        return msg

    def set_target_pos(self, msg):
        self.target_pos = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = CarbiGoToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()