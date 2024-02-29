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

### Terminal 1: Run ESP32

#### Serial Mode
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```
- Change board_microros_transport in platformio.ini
```
board_microros_transport = serial
```
- Add this code in Main.cpp
```
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif
```
#### Wifi Mode
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
- Change board_microros_transport in platformio.ini
```
board_microros_transport = wifi
```
### Terminal 2: Run robot_bridge.py 
Receiving various data from the robot for use in robot_localization (EKF)
```
ros2 run robot_bridge robot_bridge.py
```
### Terminal 3: Launch test.launch.py 
Use package robot_localization
```
ros2 launch robot_bridge test.launch.py
```
### Terminal 4: run robot_controller.py 
Send twist command to the robot
```
ros2 run robot_controller robot_controller.py
```
### Terminal 5: launch robot_description.py 
Visualize model in RVIZ
```
ros2 launch robot_description display.launch.py
```
### Terminal 6: 
Select a command to publish data in robot_controller

data: 1 = IDLE State (Not Move)

data: 2 = regtangle_path (via points)

data: 3 = circle_path

data: 4 = linear_path

```
ros2 topic pub /robot_command std_msgs/msg/Int8 'data: 1'
ros2 topic pub /robot_command std_msgs/msg/Int8 'data: 2'
ros2 topic pub /robot_command std_msgs/msg/Int8 'data: 3'
ros2 topic pub /robot_command std_msgs/msg/Int8 'data: 4'
```

## Experiment

## 1. Experimenting to find relationships in Dynamixel motor control commands 

from MX-12W datasheet [https://emanual.robotis.com/docs/en/dxl/mx/mx-12w/?fbclid=IwAR2HMHfyoMVUpXsdOFKQmWoarIelAV0Ea9ONx6sfsGAt6oXsIt6a5hGgFXE] 

![image](https://github.com/HBBEEP/FRA532_EXAM1_6406_6428_6471/assets/122891621/b3046c6d-d512-4cfa-ab8d-4f6d01149191)

**Motor Control**

In motor command, when the right motor moves forward, it rotates clockwise (CW), and the speed value is positive. When the left motor moves forward, it rotates counterclockwise (CCW), and the speed value is negative. These values can be used as booleans to command the motors. The motors require units in rpm, so commands must be converted from radians per second (rad/s) by multiplying by 60 / (2*pi), or approximately 9.5493.

```
void ROBOT_CONTROL::motorControl(float speedRight, float speedLeft)
{
    // Determine direction of each motor
    bool dirMotorRight = speedRight >= 0; // Right: True Forward CW
    bool dirMotorLeft = !(speedLeft >= 0); // Left: False Forward CCW

    // Control the motors
    Motor.turnWheel(MOTORLEFT, dirMotorLeft, abs(speedLeft *  9.5493));
    Motor.turnWheel(MOTORRIGHT, dirMotorRight, abs(speedRight *  9.5493));
}
```

**Read wheel velocity**

When reading motor values from the Dynamixel MX-12W datasheet, it is found that they can be used in the range of 0 to 2047 (0X7FF) with a unit of 0.916 rpm.

Bits 0-1023 represent counter-clockwise motor operation, while bits 1024-2048 represent clockwise motor operation.

If the left motor moves forward, it rotates counterclockwise (CCW), and if the right motor moves forward, it rotates clockwise (CW). Therefore, when the value exceeds 1024, it needs to be subtracted by 1024 and then multiplied by 0.916 to convert to rpm. After that, the rpm value is converted to radians per second (rad/s) by multiplying it by 2*pi / 60.

```
void ROBOT_CONTROL::readWheelVelocity(float* wheelVel){ 
    wheelVel[0] = Motor.readSpeed(MOTORRIGHT); // Right
    wheelVel[1] = Motor.readSpeed(MOTORLEFT); // Left

    if(wheelVel[0] > 1023){ // Right: Forward CW
        wheelVel[0] = wheelVel[0] - 1024;
    }
    else if(wheelVel[0] <= 1023){ // Right: Backward CCW
        wheelVel[0] = - wheelVel[0];
    }
    if(wheelVel[1] > 1023){ // Left: Backward CW
        wheelVel[1] = - (wheelVel[1] - 1024);
    }
    else if(wheelVel[1] <= 1023){ // Left: Forward CCW
        wheelVel[1] = wheelVel[1];
    }

    // rad/s
    wheelVel[0] = ((wheelVel[0] * 0.916) * 2 * PI) / 60; 
    wheelVel[1] = ((wheelVel[1] * 0.916) * 2 * PI) / 60;
}
```

## 2. Experimenting to find the relationship between robot's real values and wheel odometry values

## 3. Experimenting to find the relationship between the command (cmd_vel) and the movement of the robot

## 4. Experimenting to adjust the covariance values of wheel odometry and IMU in the Extended Kalman Filter (EKF)

## 5. Experimenting with movement according to scenarios 1 and 2

The purpose is to observe the movement trajectory of the actual robot compared to the odom frame of the robot obtained from wheel odometry calculations in the RVIZ program

Hypothesis: The movement trajectories may not align due to various factors such as friction forces that cannot be accurately calculated in the program, wheel slip, and other external factors

Two sets of command will be created

Set 1: Movement in a 1*1 meter square


### 5.1 regtangle path (via points)
https://github.com/HBBEEP/FRA532_EXAM1_6406_6428_6471/assets/75566343/bb18568b-762e-4abd-98b4-9f5bb295e32b

### 5.2 circle path
https://github.com/HBBEEP/FRA532_EXAM1_6406_6428_6471/assets/75566343/17a5f61e-d245-4abe-b685-1a071f6ed9c0




## Our Team

1. Kullakant Keawkallaya 64340500006
2. Thamakorn Thongyod 64340500028
3. Monsicha Sopitlaptana 64340500071
