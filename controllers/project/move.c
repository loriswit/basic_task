#include <stdlib.h>
#include <math.h>

#include <webots/robot.h>

#include "../util/motors.h"
#include "../util/prox_sensors.h"
#include "../util/ground_sensors.h"
#include "../util/consts.h"
#include "../util/leds.h"

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

void rotate(rotate_t direction)
{
    if(direction == ROTATE_LEFT)
        motors_set_speed(-MOTOR_SPEED, MOTOR_SPEED);
    else
        motors_set_speed(MOTOR_SPEED, -MOTOR_SPEED);
}

#define PROX_THRESHOLD 400

void move_as(move_t behaviour)
{
    if(behaviour == MOVE_EXPLORER)
        led_set(4, true);
    
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

double calculate_pid(double prox, double k, double ti, double td)
{
    static double error = 0;
    static double deriv = 0;
    static double integ = 0;
    
    double prev_err = error;
    
    error = prox - PID_WALL_FOLLOW_TARGET;
    deriv = (error - prev_err) * 1000 / TIME_STEP;
    integ += error * TIME_STEP / 1000;
    
    return k * error + k * td * deriv + k / ti * integ;
}

void follow_prox(double multiplier, side_t side, double k, double ti, double td)
{
    static const double left_weights[PROX_COUNT] = {0, 0, 0, 0, 0, 1, 1, 3};
    static const double right_weights[PROX_COUNT] = {3, 1, 1, 0, 0, 0, 0, 0};
    
    const double * weights = side == SIDE_LEFT ? left_weights : right_weights;
    
    double prox = 0;
    for(size_t i = 0; i < PROX_COUNT; ++i)
        prox += weights[i] * prox_get_value(i, true);
    
    prox /= array_sum(weights, PROX_COUNT);
    
    double ds = calculate_pid(prox, k, ti, td);
    if(side == SIDE_LEFT)
        ds *= -1;
    
    double speed_right = (fabs(ds) > 1 ? 0 : MOTOR_SPEED * multiplier);
    double speed_left = (fabs(ds) > 1 ? 0 : MOTOR_SPEED * multiplier);
    
    speed_right += ds;
    speed_left -= ds;
    
    motors_set_speed(speed_left, speed_right);
}

#define GROUND_MAX 1000
#define GROUND_FACTOR 250

void follow_ground(double multiplier)
{
    static const double right_weights[GROUND_COUNT] = {0, 0, 1};
    static const double left_weights[GROUND_COUNT] = {1, 0, 0};
    
    double ground_right = GROUND_MAX - ground_get_value(GROUND_RIGHT);
    double ground_center = GROUND_MAX - ground_get_value(GROUND_CENTER);
    double ground_left = GROUND_MAX - ground_get_value(GROUND_LEFT);
    
    double right_factor =
            ground_right * right_weights[GROUND_RIGHT] +
            ground_center * right_weights[GROUND_CENTER] +
            ground_left * right_weights[GROUND_LEFT];
    double left_factor =
            ground_right * left_weights[GROUND_RIGHT] +
            ground_center * left_weights[GROUND_CENTER] +
            ground_left * left_weights[GROUND_LEFT];
    
    right_factor /= array_sum(right_weights, GROUND_COUNT);
    left_factor /= array_sum(left_weights, GROUND_COUNT);
    
    double speed_right = (MOTOR_SPEED * multiplier) + left_factor / GROUND_FACTOR * (MOTOR_SPEED * multiplier);
    double speed_left = (MOTOR_SPEED * multiplier) + right_factor / GROUND_FACTOR * (MOTOR_SPEED * multiplier);
    
    motors_set_speed(speed_left, speed_right);
}

#define GROUND_THRESHOLD 500

void avoid_lines()
{
    typedef enum
    {
        SENSOR_LEFT, SENSOR_RIGHT, SENSOR_NONE
    } sensor_t;
    
    static sensor_t sensor = SENSOR_NONE;
    
    if(sensor == SENSOR_LEFT && ground_get_value(GROUND_LEFT) > GROUND_THRESHOLD)
        sensor = SENSOR_NONE;
    if(sensor == SENSOR_RIGHT && ground_get_value(GROUND_RIGHT) > GROUND_THRESHOLD)
        sensor = SENSOR_NONE;
    
    if(sensor == SENSOR_NONE && ground_get_value(GROUND_LEFT) < GROUND_THRESHOLD)
        sensor = SENSOR_LEFT;
    if(sensor == SENSOR_NONE && ground_get_value(GROUND_RIGHT) < GROUND_THRESHOLD)
        sensor = SENSOR_RIGHT;
    
    if(sensor == SENSOR_LEFT)
        rotate(ROTATE_RIGHT);
    if(sensor == SENSOR_RIGHT)
        rotate(ROTATE_LEFT);
}
