#!usr/env/python3

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import numpy as np
print("Driver Node Başlıyor")
arduino = serial.Serial('/dev/ttyACM0', 38400, timeout=.1)
time.sleep(6)
arduino.write("1000700".encode())

class controller(Node):
    def __init__(self):
        self.hata = 0
        super().__init__('Driver_Node')
        self.subscription = self.create_subscription(Float32MultiArray, '/object/distance_angle', self.callback, 1)
        
    
    def calculate(self, distance, angle):
        #self.hata = self.hata + int((distance - 200) / 50)
        #print(self.hata)
        distance = max(0, distance - 30)
        vel = min(2000, int(np.sqrt(distance) * 9) + 1000) + int((distance - 120) / 10)
        vel = max(1000, vel)   
        angle = int(400 - angle/2) # 2.3 before
        print(vel)
        return vel, angle

    def callback(self, input):
        distance = input.data[0]
        angle = input.data[1]
        if distance != -999:
            brushless, steer = self.calculate(distance, angle)
            arduino.write((str(brushless) + str(steer)).encode())
            arduino.readline()           
            

def main():
    rclpy.init()
    kontrol = controller()
    rclpy.spin(kontrol)
    kontrol.destroy_mode()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
