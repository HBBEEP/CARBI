#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from kinematics import inverse_kinematics

class CarbiSim(Node):
    def __init__(self):
        super().__init__('carbi_sim')
        self.create_subscription(Twist, '/cmd_vel', self.sim_carbi, 10)
        self.wheel_vel_publisher = self.create_publisher(Float32MultiArray, '/wheel_vel', 10)

    def sim_carbi(self, msg):
        wheel_vel = inverse_kinematics([msg.linear.x, msg.linear.y, msg.angular.z])
        wheel_vel_msg = Float32MultiArray()
        wheel_vel_msg.data = [wheel_vel[0], wheel_vel[1], wheel_vel[2], wheel_vel[3]]

        self.wheel_vel_publisher.publish(wheel_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CarbiSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()