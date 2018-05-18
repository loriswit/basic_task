#include <stdlib.h>
#include <math.h>

#include <webots/robot.h>

#include "../util/motors.h"
#include "../util/prox_sensors.h"
#include "../util/ground_sensors.h"
#include "../util/helpers.h"
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

#define PROX_THRESHOLD 400

void move_as(move_t behaviour)
{
    static const double lover_weights[PROX_COUNT] = {2, 2, 3, 4, 4, 3, 2, 2};
    static const double explorer_weights[PROX_COUNT] = {4, 3, 2, 1, 1, 2, 3, 4};
    
    const double total_weight = array_sum(behaviour == MOVE_LOVER ? lover_weights : explorer_weights, PROX_COUNT);
    
    double prox[2] = {0};
    for(size_t i = 0; i < PROX_COUNT; ++i)
    {
        double weight = behaviour == MOVE_LOVER ? lover_weights[i] : explorer_weights[i];
        prox[i * 2 / PROX_COUNT] += weight * prox_get_value(i, true) / total_weight;
    }
    
    double speed_right = MOTOR_SPEED - prox[behaviour == MOVE_LOVER ? 0 : 1] / PROX_THRESHOLD * MOTOR_SPEED;
    double speed_left = MOTOR_SPEED - prox[behaviour == MOVE_LOVER ? 1 : 0] / PROX_THRESHOLD * MOTOR_SPEED;
    
    motors_set_speed(speed_left, speed_right);
}

#define PID_WALL_FOLLOW_TARGET 700
#define PID_K 0.0016
#define PID_T_I 9
#define PID_T_D 0.35

double calculate_pid(double prox)
{
    static double error = 0;
    static double deriv = 0;
    static double integ = 0;
    
    double prev_err = error;
    
    error = prox - PID_WALL_FOLLOW_TARGET;
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

#define GROUND_THRESHOLD 600

bool detects_line()
{
    double ground_value =
            ground_get_value(GROUND_LEFT) +
            ground_get_value(GROUND_CENTER) +
            ground_get_value(GROUND_RIGHT);
    
    return (ground_value / GROUND_COUNT) < GROUND_THRESHOLD;
}

#define WALL_THRESHOLD 1100

bool detects_wall()
{
    return prox_get_value(0, true) + prox_get_value(7, true) / 2 > WALL_THRESHOLD;
}

void avoid_lines()
{
    static double cooldown = 0;
    
    if(detects_line())
        cooldown = now() + 1;
    
    if(cooldown > now())
        motors_set_speed(MOTOR_SPEED, -MOTOR_SPEED);
}
