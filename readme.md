. ROS2_pkg_cpp_py/install_pkg.bash {YOUR_WORKSPACE} {PACKAGE_NAME}
<!-- Serial Mode -->

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
board_microros_transport = serial
<!-- WIFI Mode -->

ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
board_microros_transport = wifi
