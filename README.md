
# Fener Vehicle ROS2

Fener's ROS2 codes that runs on the vehicle. 
This node contains basic sensing and actuation nodes for vehicle control. Also example applications will be added.


## Related

This project is related to these project listed below.

[Fener's Arduino Code](https://github.com/sezer-muhammed/Fener-Ard-Repo-v1)

[Fener Console Ros2](https://github.com/sezer-muhammed/Fener-Console-Repo-v1)

## Installation

First be sure that you have ROS2 Eloquent on your processing unit.

# todo ADD ALL REQUIREMENTS 

```bash
  git clone https://github.com/sezer-muhammed/Fener-Vehicle-Repo-v1.git
  cd Fener-Vehicle-Repo-v1
  colcon build --symlink-install
```

## Nodes and Usage

| Node             | Command                                                                |
| ----------------- | ------------------------------------------------------------------ |
| BNO055 IMU Data Publisher | ```ros2 run fener_package bno055_pub``` |
| Lidar Data Publisher |  ```ros2 run fener_package lidar_pub``` |
| Driver and Encoder |  ```ros2 run fener_package driver``` |
| Solo Camera Publisher |  ```ros2 run fener_package solo_cam_pub``` |

### BNO055 IMU Data Publisher

This Node publishes vehicles **oriantation** and **angular velocity** as **Float32MultiArray**.

Example *subscriber callback*
```python
def bno055_callback(self, msg):
    self.bno055_data = msg.data
```

### Lidar Data Publisher

This Node publishes vehicles **Lidar data** as **Int32MultiArray**.

Example *subscriber callback*
```python
def lidar_callback(self, msg):
    self.lidar_data = np.array(msg.data).reshape(-1, 2)
```

### Driver and Encoder Node

This Node **drives the vehicle** according to related topics and publishes **encoder data** as **Int64MultiArray**.

Example *subscriber callback*
```python
def encoder_callback(self, msg): 
    self.encoder_data = msg.data
```

### Solo Camera Publisher

This Node publishes **left camera image** as **CompressedImage**
## Authors


### Thanks
- [@katherinepeterson](https://www.github.com/octokatherine)

