#ifndef ROBOTICS_GROUND_SENSORS_H
#define ROBOTICS_GROUND_SENSORS_H

#include <stdlib.h>

#define GROUND_COUNT 3

#define GROUND_LEFT 0
#define GROUND_CENTER 1
#define GROUND_RIGHT 2

/**
 * Initializes all ground sensors.
 */
void ground_init();

/**
 * Returns the current value of a specific ground sensor.
 *
 * @param index The ground sensor index
 * @return The sensor value
 */
double ground_get_value(size_t index);

#endif //ROBOTICS_GROUND_SENSORS_H
