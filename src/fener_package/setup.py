from setuptools import setup

package_name = 'fener_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fener',
    maintainer_email='fener@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = fener_package.fener_object_detector_node:main',
            'lidar_pub = fener_package.fener_lidar_pub_node:main',
            'bno055_pub = fener_package.fener_bno055_pub_node:main',
            'solo_cam_pub = fener_package.fener_solo_cam_pub_node:main',
            'driver = fener_package.fener_arduino_comm:main',
        ],
    },
)
