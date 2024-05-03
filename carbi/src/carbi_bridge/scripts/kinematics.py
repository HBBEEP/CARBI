#!/usr/bin/python3
import numpy as np

ROBOT_WIDTH = 0.340 
ROBOT_LENGTH = 0.160 
WHEEL_RADIUS = 0.076 

FK_CONSTANT = (WHEEL_RADIUS / 4) * np.array([[1, 1, 1, 1],
                                                       [1, 1, 1, 1],
                                                       [1, -1, -1, 1],
                                                       [-1 / (ROBOT_LENGTH + ROBOT_WIDTH),
                                                         1 / (ROBOT_LENGTH + ROBOT_WIDTH),
                                                        -1 / (ROBOT_LENGTH + ROBOT_WIDTH),
                                                         1 / (ROBOT_LENGTH + ROBOT_WIDTH)]])
                                
IK_CONSTANT = (1 / WHEEL_RADIUS) * np.array([[1, 1, -(ROBOT_LENGTH + ROBOT_WIDTH)],
                                                       [1, -1, (ROBOT_LENGTH + ROBOT_WIDTH)],
                                                       [1, -1, -(ROBOT_LENGTH + ROBOT_WIDTH)],
                                                       [1, 1, (ROBOT_LENGTH + ROBOT_WIDTH)]])

def forward_kinematics(self, wheel_velocity:np.ndarray)->np.ndarray:
    robot_twist = np.dot(FK_CONSTANT, wheel_velocity)
    return robot_twist

def inverse_kinematics(self, robot_twist:np.ndarray)->np.ndarray:
    wheel_velocity = np.dot(IK_CONSTANT, robot_twist)
    return wheel_velocity

