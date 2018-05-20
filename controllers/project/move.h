#ifndef BASIC_TASK_MOVE_H
#define BASIC_TASK_MOVE_H

#include <stdbool.h>

/**
 * Enum constants representing the move behaviour of the robot.
 * <ul>
 *   <li><b>MOVE_LOVER</b>: the robot is attracted by nearby objects.</li>
 *   <li><b>MOVE_EXPLORER</b>: the robot avoids nearby objects.</li>
 * </ul>
 */
typedef enum
{
    MOVE_LOVER, MOVE_EXPLORER
} move_t;

/**
 * Moves the robot according to a specific behaviour.
 *
 * @param behaviour The behaviour of the robot, either <b>MOVE_LOVER</b> or <b>MOVE_EXPLORER</b>
 */
void move_as(move_t behaviour);

/**
 * Enum constants representing rotation direction.
 */
typedef enum
{
    ROTATE_LEFT, ROTATE_RIGHT
} rotate_t;

/**
 * Rotates in a specific direction.
 *
 * @param direction The rotation direction
 */
void rotate(rotate_t direction);

/**
 * Enum constants indicating on which side the robot should follow the wall.
 */
typedef enum
{
    SIDE_LEFT, SIDE_RIGHT
} side_t;

/**
 * Moves the robot such as it sticks to walls detected by proximity sensors along its path.
 * This function uses a PID algorithm.
 *
 * @param side The side which the robot should to follow, either WALL_LEFT or WALL_RIGHT
 * @param multiplier The speed multiplier.
 * @param k Proportional parameter
 * @param ti Integral parameter
 * @param td Differential parameter
 */
void follow_prox(double multiplier, side_t side, double k, double ti, double td);

/**
 * Moves the robot such as it sticks to lines detected by ground sensors along its path.
 *
 * @param multiplier The speed multiplier.
 */
void follow_ground(double multiplier);

/**
 * Changes the motors speed in order to avoid crossing ground lines.
 */
void avoid_lines();

#endif //BASIC_TASK_MOVE_H
