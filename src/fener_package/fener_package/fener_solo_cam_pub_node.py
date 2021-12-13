#!usr/bin/python3
import rclpy
import jetson.inference
import jetson.utils
import time
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

def resize(img, resize_factor):
        resized_img = jetson.utils.cudaAllocMapped(width=img.width * resize_factor[0], height=img.height * resize_factor[1], format=img.format)
        jetson.utils.cudaResize(img, resized_img)
        return resized_img

def main():
    rclpy.init()
    node = rclpy.create_node("solo_cam_publisher")
    publisher = node.create_publisher(CompressedImage ,"cam/solo", 1)
    node.get_logger().info("This Node Stated")
    #veri = CompressedImage()
    camera = jetson.utils.videoSource("csi://0")  

    br = CvBridge()
    while True:
        #start = time.time()
        img = camera.Capture()
        img = resize(img, (0.4, 0.4))
        np_img = jetson.utils.cudaToNumpy(img)
        jetson.utils.cudaDeviceSynchronize()
        msg = br.cv2_to_compressed_imgmsg(np_img)
        

        
        publisher.publish(msg)
        #fin = time.time()
        #print(1 / (fin - start))


if __name__ == '__main__':
    main()
