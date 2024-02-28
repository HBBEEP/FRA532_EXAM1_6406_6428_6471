. ROS2_pkg_cpp_py/install_pkg.bash {YOUR_WORKSPACE} {PACKAGE_NAME}
<!-- Serial Mode -->

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
board_microros_transport = serial
<!-- WIFI Mode -->

ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
board_microros_transport = wifi

![FRA532 Exam1](https://github.com/HBBEEP/FRA532_EXAM1_6406_6428_6471/assets/122891621/c00d9b17-f4f5-4fac-8e88-12c7fab0b919)
