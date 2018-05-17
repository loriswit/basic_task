#include <webots/robot.h>
#include <webots/distance_sensor.h>

#include "ground_sensors.h"
#include "consts.h"

WbDeviceTag sensors[GROUND_COUNT];

void ground_init()
{
    static const char * ground_sensors_names[GROUND_COUNT] =
            {"gs0", "gs1", "gs2"};
    
    for(size_t i = 0; i < GROUND_COUNT; ++i)
    {
        sensors[i] = wb_robot_get_device(ground_sensors_names[i]);
        wb_distance_sensor_enable(sensors[i], TIME_STEP);
    }
}

double ground_get_value(size_t index)
{
    return wb_distance_sensor_get_value(sensors[index]);
}
