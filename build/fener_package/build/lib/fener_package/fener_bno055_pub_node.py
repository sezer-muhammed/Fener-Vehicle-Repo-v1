#!/usr/bin/env python3
import time
import board
import adafruit_bno055
import rclpy

from std_msgs.msg import Float32MultiArray

i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)


def main(args=None):
    rclpy.init(args=args)
    imu_res = Float32MultiArray()

    imu_node = rclpy.create_node("BNO055_sensor")
    publisher = imu_node.create_publisher(Float32MultiArray, "BNO055/all", 1)
    imu_node.get_logger().info("BNO055 Started! Code Generated on MASAUSTU PC")
    euler_1_old = sensor.euler[0]
    sensor_turn = 0
    while True:
        sensor_gyro = sensor.gyro
        sensor_euler = list(sensor.euler)
        
        euler_1_old_new = sensor_euler[0]
        if euler_1_old - sensor_euler[0] > 340:
            sensor_turn += 1
        if euler_1_old - sensor_euler[0] < -340:
            sensor_turn += -1
        sensor_euler[0] += sensor_turn * 360
        euler_1_old = euler_1_old_new
        
        imu_res.data = list(sensor_gyro + tuple(sensor_euler))
        publisher.publish(imu_res)
        time.sleep(0.02)
        
if __name__ == "__main__":
    main()
