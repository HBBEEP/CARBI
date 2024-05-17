#!/usr/bin/python3
# Import necessary ROS 2 and other Python libraries
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped, Twist
import tf_transformations
import math
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import Pose, TransformStamped
import numpy as np

class CarbiPurePursuit(Node):
    def __init__(self):
        super().__init__('differential_drive_pure_pursuit')
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_marker_publisher = self.create_publisher(PoseStamped, 'goal_marker', 10)
        self.lookahead_distance = 0.6
        self.goal_threshold = 0.6
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self.pure_pursuit_controller)
        self.path = None
        self.current_pose_index = 0
        self.total_path = None

    def pure_pursuit_controller(self):
        current_pose = self.get_robot_pose()
        if (current_pose != None):
            target_index = self.calculate_goal_point(self.path, current_pose, self.current_pose_index)
            linear_velocity, angular_velocity = self.calculate_velocities(current_pose, target_index)
            self.publish_velocity(linear_velocity, angular_velocity)

    def send_goal(self, pose):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = pose
        self.action_client.wait_for_server()
        self.future = self.action_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.follow_path(result.path.poses)

    def follow_path(self, path):
        self.path = path
        print(self.path)
        self.total_path = len(self.path)


    def calculate_goal_point(self, path, robot_pose, start_index):
        temp_pose_index = start_index
        target_point = path[temp_pose_index].pose.position
        curr_dist = self.distance_between_points(target_point, robot_pose.position)
        if curr_dist < self.lookahead_distance and temp_pose_index  < (self.total_path - 1):
            temp_pose_index += 1
            target_point = path[temp_pose_index].pose.position
            curr_dist = self.distance_between_points(target_point, robot_pose.position)
            self.get_logger().info(f'>> current_pose_index {self.current_pose_index}')

        self.current_pose_index = temp_pose_index
        return self.current_pose_index
            
    def calculate_velocities(self, robot_pose, goal_point):

        dp = self.path[goal_point].pose.position - robot_pose.position
        _, _, theta = tf_transformations.euler_from_quaternion([robot_pose.orientation.x,
                                                               robot_pose.orientation.y,
                                                               robot_pose.orientation.z,
                                                               robot_pose.orientation.w])
        e = np.arctan2(dp[1] , dp[0] ) - theta
        K = 0.50
        w = K * np.arctan2(np.sin(e), np.cos(e))

        opp_x = -1 if dp[0] < 0.0 else 1
        opp_y = -1 if dp[1] < 0.0 else 1

        vx = dp[0] if abs(dp[0]) < 0.5 else 0.5 * opp_x
        vy = dp[1] if abs(dp[1]) < 0.5 else 0.5 * opp_y

        if np.linalg.norm(dp) < 0.1:
            vx = 0.0
            vy = 0.0
            w = 0.0

        linear_velocity_x= vx
        linear_velocity_y = vy
        angular_velocity_z = w

        return linear_velocity_x, linear_velocity_y, angular_velocity_z
    
    def publish_velocity(self, linear_vel_x, linear_vel_y, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel_x
        twist.linear.y = linear_vel_y
        twist.angular.z = angular_vel
        self.velocity_publisher.publish(twist)

    def is_goal_reached(self, robot_pose, goal_pose):
        return self.distance_between_points(robot_pose.position, goal_pose.pose.position) <= self.goal_threshold

    def distance_between_points(self, point1, point2):
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

    def normalize_angle(self, angle):
        if angle > math.pi:
            angle -= 2.0 * math.pi
        if angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('base_footprint', 'base_link', rclpy.time.Time())
            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            return pose
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error('Could not transform from base_link to map: %s' % str(e))
            return None
        
def main(args=None):
    rclpy.init(args=args)
    node = CarbiPurePursuit()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "base_footprint"
    goal_pose.pose.position.x = 2.0
    goal_pose.pose.position.y = -3.0
    node.send_goal(goal_pose)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
