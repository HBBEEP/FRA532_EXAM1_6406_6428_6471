#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>

#include "robot_control.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

void cmd_vel_callback(const void * msgin);

// rcl_publisher_t publisher;
rcl_subscription_t cmd_vel_sub;

geometry_msgs__msg__Twist cmd_vel;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    // // Serial.println("Debug message");
    // msg.data++;
    // printf("Received: BEEPPPPPPPPPPPPP");
  }
}

void cmd_vel_callback(const void * msgin)
{
  // Cast received message to used type
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  float testIK[2];
  RobotControl.inverseKinematics(msg->linear.x, msg->angular.z, testIK); 
  RobotControl.motorControl( testIK[0], testIK[1]);
}


void setup() {
  // Configure serial transport
  RobotControl.begin();

  Serial.begin(115200);
  // RobotControl.motorControl(100,100);

  set_microros_serial_transports(Serial); // for serial 

//   IPAddress agent_ip(172,20,10,2);
//   size_t agent_port = 8888;

//   char ssid[] = "BeepBeep";
//   char psk[] = "123456781234";
//   Serial.println("Initial Robotttt");
//   set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "microros_node", "", &support));

  // create publisher
  // RCCHECK(rclc_publisher_init_default(
  //   &publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  //   "micro_ros_platformio_node_publisher"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,  
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "BGK_cmd_vel"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel, &cmd_vel_callback, ON_NEW_DATA))
  // msg.data = 0;

}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}