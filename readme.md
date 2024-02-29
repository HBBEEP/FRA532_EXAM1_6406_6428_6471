# FRA532 Mobile Robot: Exam1 Basic Mobile Robot

This Exam is part of the FRA532 Mobile Robot course for third-year students at the Institute of Field Robotics (FIBO), King Mongkut’s University of Technology Thonburi (KMUTT).

## Table of Contents

- [Diagram](#Diagram)
- [rqt_graph](#rqt_graph)
- [Installation](#Installation)
- [Usage](#Usage)
- [Experiment](#Experiment)
- [Our Team](#Our_Team)

## Diagram

![FRA532 Exam1 drawio](https://github.com/HBBEEP/FRA532_EXAM1_6406_6428_6471/assets/122891621/9b735f94-de3c-45a7-a07b-dd02f157af9f)

## rqt_graph

![image](https://github.com/HBBEEP/FRA532_EXAM1_6406_6428_6471/assets/122891621/5b69a3b2-95d8-4d0a-a0ce-278ab65adf61)

## Installation

Step 1: Clone the repository to the src directory of your workspace.
```
cd ~/[your_workspace]/src
git clone https://github.com/HBBEEP/FRA532_EXAM1_6406_6428_6471.git
```
Step 2: Run rosdep install to install dependencies
```
rosdep update
rosdep install --from-paths src -y --ignore-src
```

Step 3: Build a package in your workspace.
```
cd ~/[your_workspace]
colcon build 
source install/setup.bash
```
## Usage

### Serial Mode
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```
```
board_microros_transport = serial
```
### Wifi Mode
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
```
board_microros_transport = wifi
```

## Experiment

### 1. Experimenting to find relationships in Dynamixel motor control commands 

from MX-12W datasheet [https://emanual.robotis.com/docs/en/dxl/mx/mx-12w/?fbclid=IwAR2HMHfyoMVUpXsdOFKQmWoarIelAV0Ea9ONx6sfsGAt6oXsIt6a5hGgFXE] 

![image](https://github.com/HBBEEP/FRA532_EXAM1_6406_6428_6471/assets/122891621/b3046c6d-d512-4cfa-ab8d-4f6d01149191)

### 2. Experimenting to find the relationship between robot's real values and wheel odometry values

### 3. Experimenting to find the relationship between the command (cmd_vel) and the movement of the robot

### 4. Experimenting to adjust the covariance values of wheel odometry and IMU in the Extended Kalman Filter (EKF)

### 5. Experimenting with movement according to scenarios 1 and 2
#### 5.1 regtangle path (via points)
#### 5.2 circle path

### Conclusion

## Our Team

1. Kullakant Keawkallaya 64340500006
2. Thamakorn Thongyod 64340500028
3. Monsicha Sopitlaptana 64340500071
