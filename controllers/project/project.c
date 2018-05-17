#include <stdio.h>

#include <webots/robot.h>

#include "../util/motors.h"
#include "../util/prox_sensors.h"
#include "../util/ground_sensors.h"
#include "../util/leds.h"
#include "../util/camera.h"
#include "../util/com.h"
#include "../util/consts.h"
#include "../util/helpers.h"

int main()
{
    wb_robot_init();
    motors_init();
    prox_init();
    ground_init();
    leds_init();
    camera_init();
    com_init();
    
    println("hello world");
    
    while(wb_robot_step(TIME_STEP) != -1)
    {
        // TODO
    }
    
    wb_robot_cleanup();
    
    return EXIT_SUCCESS;
}
