#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from kinematics import inverse_kinematics
import serial
import time

MOTOR_A = 0
MOTOR_B = 1
MOTOR_C = 2
MOTOR_D = 3

MOTOR_PWM = 0
MOTOR_VALOCITY = 1
RESET = 2

class CarbiController(Node):
    def __init__(self):
        super().__init__('carbi_controller')

        self.timer_ = self.create_timer(0.01, self.timer_callback)
        self.wheel_vel_publisher = self.create_publisher(Float32MultiArray, '/wheel_vel', 50)
        self.create_subscription(Twist, '/cmd_vel', self.set_speed_cmd, 10)

        self.wheel_vel_msg = Float32MultiArray()

        self.mode = RESET

        self.prev = time.time()
        self.setup = False
        self.port = None
        self.connecting()

    def serial_velocity_control(self,wheel_vel):
        cmd = 'v'+' '+str(round(wheel_vel[0],2))+' '+str(round(wheel_vel[1],2))+' '+str(round(wheel_vel[2],2))+' '+str(round(wheel_vel[3],2))+'\n'
        self.port.write(cmd.encode())

    def set_speed_cmd(self, msg):
        wheel_vel = inverse_kinematics([msg.linear.x, msg.linear.y, msg.angular.z])
        self.mode = MOTOR_VALOCITY
        self.serial_velocity_control(wheel_vel)


    def timer_callback(self):
        serialSub = self.port.readline().decode('ascii')
        print(serialSub)

    def connecting(self):
        while(not self.setup):
            try:
                self.port = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)

            except:
                if(time.time() - self.prev > 2):
                    print("No serial detected, please plug your uController")
                    self.prev = time.time()

            if(self.port is not None):
                print("OK")
                self.setup = True

        
def main(args=None):
    rclpy.init(args=args)
    node = CarbiController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()