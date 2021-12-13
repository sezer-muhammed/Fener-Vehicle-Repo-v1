#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray
import numpy as np
import time
import serial
from os import system

class MyNode(Node):
    def __init__(self):
        super().__init__("arduino_comm")
        self.get_logger().info("The Node has been started.")

        self.ard = serial.Serial(port = "/dev/ttyACM0", baudrate = 230400, timeout = 1)
        time.sleep(2)

        self.speed_mul = 0
        self.keyboard_packet = [0, 0, 0, 1]
        self.keyboard_commands = [0, 1, 2, 3]
        self.aim_angle = 0
        self.key_angle = 0
        self.key_speed = 0

        self.encoer = np.array([0, 0, 0, 0])
        self.bno055_heading = 0
        self.keys = [0, 0, 0, 0, 0, 0, 0]
 
        self.create_subscription(Float32MultiArray, "BNO055/all", self.bno055_callback, 1)
        self.create_subscription(Int8MultiArray, "/drive_command/keyboard", self.keyboard_callback, 1)

        self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        
        self.keyboard_packet[0:3] = [self.key_speed, min(3000, max(0, self.key_angle + 25 * (self.aim_angle - self.bno055_heading))), self.bno055_heading%360]

        self.encoder = self.send_to_arduino(self.keyboard_commands, self.keyboard_packet)
        data = self.ard.readline().decode()
#        print(data)
#        print(self.keyboard_packet)


    def send_to_arduino(self, commands, values):

            msg = bytearray()

            for command, value in zip(commands, values):
                    first_byte = value % 256
                    second_byte = value // 256
                    command_byte = command * 32
                    msg.append(first_byte)
                    msg.append(second_byte + command_byte)
            self.ard.write(msg)

            output = np.array(list(self.ard.read(size = 4))) - 1
            #print(output)
            return output

    def keyboard_callback(self, msg):
        #[w, a, s, d, space, ctrl_r, shift_r]
        keys = msg.data
        self.speed_mul = max(self.speed_mul - keys[5] + keys[6], 0)
        
        if (keys[1] - keys[3]) != 0:
            self.aim_angle = self.bno055_heading
        self.key_speed = int(self.speed_mul * 30 * keys[0] * (1 - keys[2]))
        self.key_angle = int(1500 - (keys[1] - keys[3]) * 400) # mid is 1500
        #self.encoder = self.send_to_arduino(self.keyboard_commands, self.keyboard_packet)
        #data = self.ard.readline() 

        #print(self.keyboard_packet)

    def bno055_callback(self, msg):
        self.bno055_heading = int(msg.data[3])
        #print(self.bno055_heading)


def main(args=None):
    system("sudo chmod 666 /dev/ttyACM0")
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if  __name__ == "__main__":
    main()
