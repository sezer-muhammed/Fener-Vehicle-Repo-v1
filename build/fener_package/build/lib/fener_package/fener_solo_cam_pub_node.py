#!usr/bin/python3
import rclpy
import jetson.inference
import jetson.utils
import time
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

RATIO = 0.35

def resize(img, resize_factor):
        resized_img = jetson.utils.cudaAllocMapped(width=img.width * resize_factor[0], height=img.height * resize_factor[1], format=img.format)
        jetson.utils.cudaResize(img, resized_img)
        return resized_img

def main():
    rclpy.init()
    node = rclpy.create_node("solo_cam_publisher")
    publisher = node.create_publisher(CompressedImage ,"cam/solo", 1)
    node.get_logger().info("This Node Stated")
    camera = jetson.utils.videoSource("csi://0")  

    br = CvBridge()

    while True:
        img = camera.Capture()
        img = resize(img, (RATIO, RATIO))
        np_img = jetson.utils.cudaToNumpy(img)
        jetson.utils.cudaDeviceSynchronize()
        msg = br.cv2_to_compressed_imgmsg(np_img)
        publisher.publish(msg)

if __name__ == '__main__':
    main()
