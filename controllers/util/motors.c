#include <webots/robot.h>
#include <webots/motor.h>

#include "motors.h"

#define MAX_SPEED 6.28

WbDeviceTag left_motor;
WbDeviceTag right_motor;

// returns a value clamped between min and max
double clamp(double value, double min, double max)
{
    if(value > max)
        return max;
    
    if(value < min)
        return min;
    
    return value;
}

void motors_set_speed(double left_speed, double right_speed)
{
    wb_motor_set_velocity(left_motor, clamp(left_speed, -MAX_SPEED, MAX_SPEED));
    wb_motor_set_velocity(right_motor, clamp(right_speed, -MAX_SPEED, MAX_SPEED));
}

void motors_stop()
{
    motors_set_speed(0, 0);
}

void motors_init()
{
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    
    motors_stop();
}
