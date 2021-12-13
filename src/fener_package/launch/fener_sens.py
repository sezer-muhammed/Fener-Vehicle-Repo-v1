from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fener_package',
            node_executable='bno055_pub',
            name='BNO055_sensor'
        ),
        Node(
            package='fener_package',
            node_executable='lidar_pub',
            name='RPLidar_sensor'
        ),
    ])

'''
        Node(
            package='fener_package',
            node_executable='driver',
            name='Driver_Node'
        ),
        Node(
            package='fener_package',
            node_executable='object_detector',
            name='distance_and_angle_of_object'
        ),
        Node(
            package='fener_package',
            node_executable='solo_cam_pub',
            name='solo_cam_publisher'
        ),
'''
