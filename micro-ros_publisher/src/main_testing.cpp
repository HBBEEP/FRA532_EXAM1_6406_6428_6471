// #include <Arduino.h>
// #include "robot_control.h"

// long preMilliseconds = 0;
// float dt = 0;



// void setup() 
// {
//   // Initialize motor control and serial communication
//   RobotControl.begin();
//   Serial.begin(115200);
//   RobotControl.motorControl(-10, -10);
//   Serial.print("Robot Init");
// }


// void loop() 
// {
//   if (millis() - preMilliseconds >= 30)
//   {
//     dt = (millis() - preMilliseconds) / 1000.0;
//     preMilliseconds = millis();

//     // int *wheelVel = RobotControl.getWheelVel();
//     // Serial.print("Wheel Velocity r: ");
//     // Serial.println(wheelVel[0]);
//     // Serial.print("Wheel Velocity l: ");
//     // Serial.println(wheelVel[1]);

//     // int *wheelPos = RobotControl.getWheelPos();
//     // Serial.print("Wheel Pos r: ");
//     // Serial.println(wheelVel[0]);
//     // Serial.print("Wheel Pos l: ");
//     // Serial.println(wheelVel[1]);
//     float testIK[2];
//     RobotControl.inverseKinematics(3, 4, testIK); 
//     Serial.print("testIK 0: ");
//     Serial.println(testIK[0]);
//     Serial.print("testIK 1: ");
//     Serial.println(testIK[1]);
//   }
// }


