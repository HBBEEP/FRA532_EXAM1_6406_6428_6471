#include "robot_control.h"
#include <cmath>
#include "Motor.h" 
#include <Arduino.h>

void ROBOT_CONTROL::begin()
{
   motorBegin();
}

void ROBOT_CONTROL::end()
{
	
}

void ROBOT_CONTROL::motorBegin()
{

    Motor.begin(BaudRate, DirectionPin, &Serial2);

}

void ROBOT_CONTROL::motorControl(int speedRight, int speedLeft)
{
    // Determine direction of each motor
    bool dirMotorRight = speedRight >= 0;
    bool dirMotorLeft = !(speedLeft >= 0);

    // Control the motors
    Motor.turnWheel(MOTORLEFT, dirMotorLeft, abs(speedLeft));
    Motor.turnWheel(MOTORRIGHT, dirMotorRight, abs(speedRight));

}

int * ROBOT_CONTROL::getWheelVel()
{
    static int wheelVel[2];
    wheelVel[0] = Motor.readSpeed(MOTORRIGHT);
    wheelVel[1] = Motor.readSpeed(MOTORLEFT);
    return wheelVel;
}

int * ROBOT_CONTROL::getWheelPos()
{
    static int wheelPos[2];
    wheelPos[0] = Motor.readPosition(MOTORRIGHT);
    wheelPos[1] = Motor.readPosition(MOTORLEFT);
    return wheelPos;
}

void ROBOT_CONTROL::inverseKinematics(float linear_vel, float angular_vel, float* wheelVel) {
    float robotTwist[2] = {linear_vel, angular_vel};
    for (int i = 0; i < 2; i++) {
        wheelVel[i] = ikConstant[i][0] * robotTwist[0] + ikConstant[i][1] * robotTwist[1];
    }
}

void ROBOT_CONTROL::readWheelVelocity(float* wheelVel){ // pointer
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

ROBOT_CONTROL RobotControl; 