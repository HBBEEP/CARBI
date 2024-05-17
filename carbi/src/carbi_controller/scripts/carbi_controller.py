#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Bool
from geometry_msgs.msg import Twist 
# from std_msgs.msg import Float32MultiArray
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

        self.timer_ = self.create_timer(0.025, self.timer_callback)
        self.wheel_vel_publisher = self.create_publisher(Float32MultiArray, '/wheel_vel', 50)
        self.create_subscription(Twist, '/cmd_vel', self.set_speed_cmd, 10)
        self.create_subscription(Bool, '/Emergency', self.emergency_cmd, 10)

        self.wheel_vel_msg = Float32MultiArray()
        # self.wheel_vel_msg.data = [0.0,0.0,0.0,0.0]

        self.mode = RESET
        self.emergencyStatus = False

        self.prev = time.time()
        self.setup = False
        self.port = None
        self.readStatus = True
        self.connecting()

    def serial_velocity_control(self,wheel_vel):
        cmd = '<'+'v'+','+str(round(wheel_vel[0],2))+','+str(round(wheel_vel[1],2))+','+str(round(wheel_vel[2],2))+','+str(round(wheel_vel[3],2))+'>'+'\n'
        print(cmd)
        self.port.write(cmd.encode())

    def set_speed_cmd(self, msg):
        wheel_vel = inverse_kinematics([msg.linear.x, msg.linear.y, msg.angular.z])
        # wheel_vel = [-2.0,-2.0,-2.0,-2.0]
        self.mode = MOTOR_VALOCITY
        # self.serial_velocity_control([2.00, -2.00, -2.00, 0.00])
        if self.emergencyStatus == False:
            self.serial_velocity_control(wheel_vel)

    def emergency_cmd(self,msg):
        self.emergencyStatus = msg.data
        print("Emergency : ",self.emergencyStatus)
        if self.emergencyStatus == False:
            self.emergencyStatus=False
        else:
            cmd = "<r>\n"
            self.port.write(cmd.encode())

    def readSerial(self):
        try:
            serialSub = self.port.readline().decode('ascii')
            serialDecode = serialSub.split('/')
            if len(serialDecode[0])<=6:
                self.wheel_vel_msg.data = [float(serialDecode[0]),float(serialDecode[1]),float(serialDecode[2]),float(serialDecode[3])]
                self.wheel_vel_publisher.publish(self.wheel_vel_msg)
                self.readStatus = True
        except:
            self.readStatus = False

    def timer_callback(self):
            self.readSerial()
            while(self.readStatus == False):
                self.readSerial()

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