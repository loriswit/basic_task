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
 * Tells if the robot detects a line on the ground.
 *
 * @return <b>true</b> if a line is detected, <b>false</b> if not
 */
bool detects_line();

/**
 * Tells if the robot detects a wall in front of it.
 *
 * @return <b>true</b> if a wall is detected, <b>false</b> if not
 */
bool detects_wall();

/**
 * Changes the motors speed in order to avoid crossing ground lines.
 */
void avoid_lines();

#endif //PROJECT_MOVE_H
