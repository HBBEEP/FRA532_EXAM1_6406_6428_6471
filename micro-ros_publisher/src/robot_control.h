#ifndef robot_control_h
#define robot_control_h
#include "config.h"
#include <Arduino.h>

class ROBOT_CONTROL{
private:
    void motorBegin();
    float ikConstant[2][2] = {{1/WHEEL_RADIUS, (0.5 * WHEEL_SEPARATION )/WHEEL_RADIUS}, 
                              {1/WHEEL_RADIUS, -(0.5 * WHEEL_SEPARATION )/WHEEL_RADIUS}};


public:
    void begin();
    void end(void); 

    void motorControl(int speedRight, int speedLeft);
    void robotOdometry();

    int *getWheelVel();
    int *getWheelPos();
    void inverseKinematics(float linear_vel, float angular_vel, float* wheelVel);
    int *ReadWheelVelocity();
};
extern ROBOT_CONTROL RobotControl;

#endif