#include <webots/robot.h>
#include <webots/distance_sensor.h>

#include "prox_sensors.h"
#include "leds.h"
#include "consts.h"

WbDeviceTag sensors[PROX_COUNT];
double corrections[PROX_COUNT] = {0, 0};

#define CALIB_COUNT 50
#define CALIB_OFFSET 5

// computes calibration values for proximity sensors
void prox_compute_corrections()
{
    for(unsigned i = 0; i < CALIB_COUNT + CALIB_OFFSET && wb_robot_step(TIME_STEP) != -1; ++i)
    {
        leds_flash();
        
        if(i >= CALIB_OFFSET)
            for(size_t j = 0; j < PROX_COUNT; ++j)
                corrections[j] += wb_distance_sensor_get_value(sensors[j]);
    }
    
    for(size_t i = 0; i < PROX_COUNT; ++i)
        corrections[i] /= CALIB_COUNT;
    
    leds_set(false);
}

void prox_init()
{
    static const char * prox_sensors_names[PROX_COUNT] =
            {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
    
    for(size_t i = 0; i < PROX_COUNT; ++i)
    {
        sensors[i] = wb_robot_get_device(prox_sensors_names[i]);
        wb_distance_sensor_enable(sensors[i], 4 * TIME_STEP);
    }
    
    prox_compute_corrections();
}

double prox_get_value(size_t index, bool correct)
{
    double sensor_value = wb_distance_sensor_get_value(sensors[index]);
    return correct ? sensor_value - corrections[index] : sensor_value;
}
