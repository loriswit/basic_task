#ifndef ROBOTICS_MOTORS_H
#define ROBOTICS_MOTORS_H

/**
 * Initializes both motors.
 */
void motors_init();

/**
 * Sets both motors individual speed.
 * If speed is greater than 2*pi or lower than -2*pi, it is clamped to those values.
 *
 * @param left_speed The speed of the left motor (in rad/s)
 * @param right_speed The speed of the right motor (in rad/s)
 */
void motors_set_speed(double left_speed, double right_speed);

/**
 * Sets both motors speed to zero.
 */
void motors_stop();

#endif //ROBOTICS_MOTORS_H
