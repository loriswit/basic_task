#include "../util/ground_sensors.h"
#include "../util/prox_sensors.h"

#include "detection.h"

#define GROUND_THRESHOLD 500

bool detects_line(double left_weight, double center_weight, double right_weight)
{
    double ground_value =
            ground_get_value(GROUND_LEFT) * left_weight +
            ground_get_value(GROUND_CENTER) * center_weight +
            ground_get_value(GROUND_RIGHT) * right_weight;
    
    return (ground_value / (left_weight + center_weight + right_weight)) < GROUND_THRESHOLD;
}

#define WALL_THRESHOLD 800.0

bool detects_wall(wall_t side)
{
    static const size_t sensors[4][2] = {
            {5, 6}, // WALL_LEFT
            {1, 2}, // WALL_RIGHT
            {7, 0}, // WALL_FRONT
            {3, 4}  // WALL_BACK
    };
    
    double prox_value = prox_get_value(sensors[side][0], true) + prox_get_value(sensors[side][1], true);
    return prox_value / 2 > WALL_THRESHOLD / (side == WALL_BACK ? 2 : 1);
}
