#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include "MPU9250.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "robot_control.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t wheel_vel_pub;
rcl_publisher_t imu_data_pub;
rcl_subscription_t cmd_vel_sub;

geometry_msgs__msg__Twist cmd_vel;
std_msgs__msg__Float32MultiArray wheel_vel;
std_msgs__msg__Float32MultiArray imu_data;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void cmd_vel_callback(const void * msgin);

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
  
  }
}

bool malloc_wheel_flag = false;
bool malloc_imu_flag = false;

void cmd_vel_callback(const void * msgin)
{
  // Cast received message to used type
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  static float robot_wheel_vel[2];
  RobotControl.inverseKinematics(msg->linear.x, msg->angular.z, robot_wheel_vel); 
  
  RobotControl.motorControl( robot_wheel_vel[0], robot_wheel_vel[1]);

  static float wheel_vel_read[2];
  RobotControl.readWheelVelocity(wheel_vel_read);
  if (!malloc_wheel_flag)
  {
    wheel_vel.data.capacity = 2;
    wheel_vel.data.size = 2;
    wheel_vel.data.data = (float *)malloc(sizeof(float) * wheel_vel.data.capacity);
    malloc_wheel_flag = !malloc_wheel_flag;
  }
  wheel_vel.data.data[0] = wheel_vel_read[0];
  wheel_vel.data.data[1] = wheel_vel_read[1];

  static float imu_acc_read[3];
  static float imu_gyro_read[3];
  static float imu_data_read[6];
  RobotControl.getImuAcc(imu_acc_read);
  RobotControl.getImuGyro(imu_gyro_read);

  if (!malloc_imu_flag)
  {
    imu_data.data.capacity = 6;
    imu_data.data.size = 6;
    imu_data.data.data = (float *)malloc(sizeof(float) * imu_data.data.capacity);
    malloc_imu_flag = !malloc_imu_flag;
  }

  for (int i = 0; i < 3; i++)
  {
     imu_data.data.data[i] = imu_acc_read[i];
     imu_data.data.data[i+3] = imu_gyro_read[i];
  }

  RCSOFTCHECK(rcl_publish(&wheel_vel_pub, &wheel_vel, NULL));
  RCSOFTCHECK(rcl_publish(&imu_data_pub, &imu_data, NULL));


}


void setup() {
  // Configure serial transport

  // RobotControl.motorBegin();
  Serial.begin(115200);
  Wire.begin();
  // RobotControl.imuBegin();
  RobotControl.begin();
  
  set_microros_serial_transports(Serial); // for serial 

  // IPAddress agent_ip(172,20,10,2);
  // size_t agent_port = 8888;

  // char ssid[] = "BeepBeep";
  // char psk[] = "123456781234";
  // Serial.println("Initial Robotttt");
  // set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  delay(2000);


  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "BGK_microros_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &wheel_vel_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "BGK_wheel_vel"));

  RCCHECK(rclc_publisher_init_default(
    &imu_data_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "BGK_imu_data"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,  
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "BGK_cmd_vel"));

  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel, &cmd_vel_callback, ON_NEW_DATA))
  // msg.data = 0;

}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
