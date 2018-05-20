#ifndef BASIC_TASK_DETECTION_H
#define BASIC_TASK_DETECTION_H

#include <stdbool.h>

/**
 * Enum constants indicating on which side the robot should detect a wall.
 */
typedef enum
{
    WALL_LEFT, WALL_RIGHT, WALL_FRONT, WALL_BACK
} wall_t;

/**
  * Tells if the robot detects a line on the ground.
  * @param left_weight The weight of the left ground sensor
  * @param center_weight The weight of the central ground sensor
  * @param right_weight The weight of the right ground sensor
  * @return <b>true</b> if a line is detected, <b>false</b> if not
  */
bool detects_line(double left_weight, double center_weight, double right_weight);

/**
 * Tells if the robot detects a wall.
 *
 * @param side The side which the robot should detect a wall
 * @return <b>true</b> if a wall is detected, <b>false</b> if not
 */
bool detects_wall(wall_t side);

#endif //BASIC_TASK_DETECTION_H
