#include <stdlib.h>
#include <math.h>

#include <webots/robot.h>

#include "../util/motors.h"
#include "../util/prox_sensors.h"
#include "../util/ground_sensors.h"
#include "../util/consts.h"

#include "move.h"

#define MOTOR_SPEED 3

/**
 * Returns the sum of a array of <b>doubles</b>.
 *
 * @param array An array containing <b>doubles</b>
 * @param length The size of the array
 * @return The sum of each items from the array
 */
double array_sum(const double array[], size_t length)
{
    double sum = 0;
    for(size_t i = 0; i < length; ++i)
        sum += array[i];
    
    return sum;
}

#define THRESHOLD_SPEED 400

void move_as(move_t behaviour)
{
    static const double lover_weights[PROX_COUNT] = {2, 2, 3, 4, 4, 3, 2, 2};
    static const double explorer_weights[PROX_COUNT] = {4, 3, 2, 1, 1, 2, 3, 4};
    
    const double lover_total_weight = array_sum(lover_weights, PROX_COUNT);
    const double explorer_total_weight = array_sum(explorer_weights, PROX_COUNT);
    
    double prox[2] = {0, 0};
    for(size_t i = 0; i < PROX_COUNT; ++i)
    {
        double weight = behaviour == MOVE_LOVER ? lover_weights[i] : explorer_weights[i];
        double total_weight = behaviour == MOVE_LOVER ? lover_total_weight : explorer_total_weight;
        prox[i * 2 / PROX_COUNT] +=
                weight * prox_get_value(i, true) / total_weight;
    }
    
    double speed_right = MOTOR_SPEED - prox[behaviour == MOVE_LOVER ? 0 : 1] / THRESHOLD_SPEED * MOTOR_SPEED;
    double speed_left = MOTOR_SPEED - prox[behaviour == MOVE_LOVER ? 1 : 0] / THRESHOLD_SPEED * MOTOR_SPEED;
    
    motors_set_speed(speed_left, speed_right);
}

#define PID_WALL_FOLLOW_TARGET 700
#define PID_K 0.0016
#define PID_T_I 9
#define PID_T_D 0.35

double calculate_pid(double proxLR)
{
    static double error = 0;
    static double deriv = 0;
    static double integ = 0;
    
    double prev_err = error;
    
    error = proxLR - PID_WALL_FOLLOW_TARGET;
    deriv = (error - prev_err) * 1000 / TIME_STEP;
    integ += error * TIME_STEP / 1000;
    
    return PID_K * error + PID_K * PID_T_D * deriv + PID_K / PID_T_I * integ;
}

#define WEIGHT_FRONT 3
#define WEIGHT_FRONT_SIDE 1
#define WEIGHT_SIDE 1
#define WEIGHT_BACK 0
#define WEIGHTS_COUNT 4

void follow_wall()
{
    static const double weights[WEIGHTS_COUNT] = {WEIGHT_FRONT, WEIGHT_FRONT_SIDE, WEIGHT_SIDE, WEIGHT_BACK};
    
    double prox_right = 0;
    for(size_t i = 0; i < WEIGHTS_COUNT; ++i)
        prox_right += weights[i] * prox_get_value(i, true);
    
    prox_right /= array_sum(weights, WEIGHTS_COUNT);
    
    double ds = calculate_pid(prox_right);
    
    double speed_right = (fabs(ds) > 1 ? 0 : MOTOR_SPEED) + ds;
    double speed_left = (fabs(ds) > 1 ? 0 : MOTOR_SPEED) - ds;
    
    motors_set_speed(speed_left, speed_right);
}

#define THRESHOLD_GROUND 300

bool detects_line()
{
    double average_ground =
            (ground_get_value(GROUND_LEFT) + ground_get_value(GROUND_CENTER) + ground_get_value(GROUND_RIGHT)) / 3;
    
    if(average_ground < THRESHOLD_GROUND)
        return true;
    else
        return false;
}

bool detects_wall()
{
    // TODO: implementation
    return false;
}
