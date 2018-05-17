#ifndef ROBOTICS_PROX_SENSORS_H
#define ROBOTICS_PROX_SENSORS_H

#include <stdlib.h>
#include <stdbool.h>

#define PROX_COUNT 8

/**
 * Initializes all proximity sensors.
 */
void prox_init();

/**
 * Returns the current value of a specific proximity sensor.
 *
 * @param index The proximity sensor index
 * @param correct TRUE to return calibrated value, FALSE to return raw value
 * @return The sensor value
 */
double prox_get_value(size_t index, bool correct);

#endif //ROBOTICS_PROX_SENSORS_H
