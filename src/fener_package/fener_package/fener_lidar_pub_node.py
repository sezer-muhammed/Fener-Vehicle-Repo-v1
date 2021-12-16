#!/usr/bin/env python3
from rplidar import RPLidar
import os
import numpy as np

import rclpy

from std_msgs.msg import Float32MultiArray

def main(args=None):
    rclpy.init(args=args)
    scan_res = Float32MultiArray()
    os.system("sudo chmod 666 /dev/ttyUSB0")
    lidar = RPLidar('/dev/ttyUSB0')

    lidar_node = rclpy.create_node("RPLidar_sensor")
    publisher = lidar_node.create_publisher(Float32MultiArray, "Lidar/raw", 1)
    lidar_node.get_logger().info("RP Lidar Started!")
    
    for i, scan in enumerate(lidar.iter_scans()):
        scan = np.array(scan)[:, 1:]
    
        #bu veriler lidara göre, bunun aracın merkezine göre olacak hale çevirmek lazım
    
        scan = scan.flatten()
        scan_res.data = list(scan)
        publisher.publish(scan_res)
        
if __name__ == "__main__":
    main()
