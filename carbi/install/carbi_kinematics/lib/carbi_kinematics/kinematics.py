#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import numpy as np

class CarbiKinematics(Node):
    def __init__(self):
        super().__init__('carbi_kinematics')

        self.ROBOT_H_WIDTH = 0.340 
        self.ROBOT_H_LENGTH = 0.160 # not sure
        self.WHEEL_RADIUS = 0.076 

        self.FK_CONSTANT = (self.WHEEL_RADIUS / 4) * np.array([[1, 1, 1, 1],
                                                               [1, 1, 1, 1],
                                                               [1, -1, -1, 1],
                                                               [-1 / (self.ROBOT_H_LENGTH + self.ROBOT_H_WIDTH),
                                                                 1 / (self.ROBOT_H_LENGTH + self.ROBOT_H_WIDTH),
                                                                -1 / (self.ROBOT_H_LENGTH + self.ROBOT_H_WIDTH),
                                                                 1 / (self.ROBOT_H_LENGTH + self.ROBOT_H_WIDTH)]])
                                
        self.IK_CONSTANT = (1 / self.WHEEL_RADIUS) * np.array([[1, 1, -(self.ROBOT_H_LENGTH + self.ROBOT_H_WIDTH)],
                                                               [1, -1, (self.ROBOT_H_LENGTH + self.ROBOT_H_WIDTH)],
                                                               [1, -1, -(self.ROBOT_H_LENGTH + self.ROBOT_H_WIDTH)],
                                                               [1, 1, (self.ROBOT_H_LENGTH + self.ROBOT_H_WIDTH)]])

    def forward_kinematics(self, wheel_velocity:np.ndarray)->np.ndarray:
        robot_twist = np.dot(self.FK_CONSTANT, wheel_velocity)
        return robot_twist

    def inverse_kinematics(self, robot_twist:np.ndarray)->np.ndarray:
        wheel_velocity = np.dot(self.IK_CONSTANT, robot_twist)
        return wheel_velocity

    
def main(args=None):
    rclpy.init(args=args)
    node = CarbiKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()