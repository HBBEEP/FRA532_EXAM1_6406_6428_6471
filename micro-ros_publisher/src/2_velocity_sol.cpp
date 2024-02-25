// #include <Arduino.h>
// #include "Motor.h" // Include Arduino and Motor library headers

// #define DirectionPin 4 // Define pin for motor direction
// #define BaudRate 115200 // Define baud rate for serial communication
// #define MOTORLEFT 2
// #define MOTORRIGHT 1


// // Declare global variables for motor velocity, encoder counts, etc.
// int velocity = 20;
// int count_left = 0;
// int count_right = 0;
// long preMilliseconds = 0;
// float dt = 0;

// // Variables for phase, offset and position calculations
// float lastPhase_left = 0;
// int offset_left = 0;

// float lastPhase_right = 0;
// int offset_right = 0;



// void setup() 
// {
//   // Initialize motor control and serial communication
//   Motor.begin(BaudRate, DirectionPin, &Serial2);
//   Serial.begin(115200);
//   // Read initial motor positions to set offsets
//   offset_left = Motor.readPosition(MOTORLEFT);
//   offset_right = -Motor.readPosition(MOTORRIGHT);
//   lastPhase_left = offset_left;
//   lastPhase_right = offset_right;

//   Serial.print("Robot Init");
// }


// void loop() 
// {
//   // Execute code block every 30 milliseconds
//   if (millis() - preMilliseconds >= 30)
//   {
//     // Calculate time elapsed since last loop iteration
//     dt = (millis() - preMilliseconds) / 1000.0;
//     preMilliseconds = millis();

//     // Read current motor positions
//     count_left = Motor.readSpeed(MOTORLEFT);
//     count_right = Motor.readSpeed(MOTORRIGHT);

//     // Send command to Dynamixel
//     // Motor.turnWheel(1, LEFT, 0);  //
//     Motor.turnWheel(MOTORLEFT, LEFT, 140); // 
//     Motor.turnWheel(MOTORRIGHT, RIGHT, 100); // 


//     // Print Motor Speed
//     Serial.print("Count Left : ");
//     Serial.println(count_left);
//     Serial.print("Count Right : ");
//     Serial.println(count_right);
//   }
// }


