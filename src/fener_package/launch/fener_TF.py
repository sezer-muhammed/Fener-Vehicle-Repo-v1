from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            name='static_TF',
            arguments = ["0", "0", "0", "0", "0", "0", "base_link", "RP_Lidar"]),
        Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            name='static_TF',
            arguments = ["0", "0", "0", "0", "0", "0", "base_link", "BNO055"]),
        Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            name='static_TF',
            arguments = ["0", "0", "0", "0", "0", "0", "base_link", "Camera Left"]),
        Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            name='static_TF',
            arguments = ["0", "0", "0", "0", "0", "0", "base_link", "Camera Right"]),
    ])
