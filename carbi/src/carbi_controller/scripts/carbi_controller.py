#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time

class CarbiController(Node):
    def __init__(self):
        super().__init__('carbi_controller')
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.setup = False
        self.port = None
        self.prev = time.time()
        self.connecting()
        self.cmd = 'r'

    def read_ser(self,num_char):
        string = self.port.read(num_char)
        return string.decode()

    def write_ser(self,cmd):
        cmd = cmd + '\n'
        self.port.write(cmd.encode())

    def timer_callback(self):
        string = self.read_ser(4)
        if(len(string)):
            print(string)
        if self.cmd=='s':
            print(float(string))

        self.cmd = input()
        if(self.cmd):
            self.write_ser(self.cmd)

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
