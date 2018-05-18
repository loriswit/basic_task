#ifndef PROJECT_MOVE_H
#define PROJECT_MOVE_H

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
 * Enum constants indicating on which side the robot should detect a wall.
 */
typedef enum
{
    WALL_LEFT, WALL_RIGHT, WALL_FRONT, WALL_BACK
} wall_t;

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
 * Moves the robot according to a specific behaviour.
 *
 * @param behaviour The behaviour of the robot, either <b>MOVE_LOVER</b> or <b>MOVE_EXPLORER</b>
 */
void move_as(move_t behaviour);

/**
 * Moves the robot such as it sticks to a wall along its path.
 */
void follow_wall();

/**
 * Moves the robot such as it sticks to a line along its path.
 */
void follow_line();

/**
 * Tells if the robot detects a line on the ground.
 *
 * @return <b>true</b> if a line is detected, <b>false</b> if not
 */
bool detects_line();

/**
 * Tells if the robot detects a wall.
 *
 * @param side The side which the robot should detect a wall
 * @return <b>true</b> if a wall is detected, <b>false</b> if not
 */
bool detects_wall(wall_t side);

/**
 * Changes the motors speed in order to avoid crossing ground lines.
 */
void avoid_lines();

#endif //PROJECT_MOVE_H
