#ifndef robot_control_h
#define robot_control_h
#include "config.h"
#include <Arduino.h>

class ROBOT_CONTROL{
private:

    float ikConstant[2][2] = {{1/WHEEL_RADIUS, (0.5 * WHEEL_SEPARATION )/WHEEL_RADIUS}, 
                              {1/WHEEL_RADIUS, -(0.5 * WHEEL_SEPARATION )/WHEEL_RADIUS}};


public:
    void begin();
    void end(void); 
    void motorBegin();
    void imuBegin();
    void motorControl(float speedRight, float speedLeft);

    int *getWheelVel();
    int *getWheelPos();
    
    void inverseKinematics(float linear_vel, float angular_vel, float* wheelVel);
    void readWheelVelocity(float* WheelVel);
    void getImuAcc(float * acc_in);
    void getImuGyro(float * gyro_in);

};
extern ROBOT_CONTROL RobotControl;

#endif