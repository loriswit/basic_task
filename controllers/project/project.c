#include <webots/robot.h>

#include "message.h"
#include "color.h"
#include "move.h"

#include "../util/motors.h"
#include "../util/prox_sensors.h"
#include "../util/ground_sensors.h"
#include "../util/leds.h"
#include "../util/camera.h"
#include "../util/com.h"
#include "../util/consts.h"

#define running wb_robot_step(TIME_STEP) != -1

int main()
{
    wb_robot_init();
    motors_init();
    prox_init();
    ground_init();
    leds_init();
    camera_init();
    com_init();
    
    color_t color_to_find = COLOR_ANY;
    bool ready_for_pid = false;
    
    while(running)
    {
        while(true)
            switch(message_receive())
            {
                case MESSAGE_FIND_RED:
                    color_to_find = COLOR_RED;
                    break;
                case MESSAGE_FIND_BLUE:
                    color_to_find = COLOR_BLUE;
                    break;
                case MESSAGE_DO_PID:
                    ready_for_pid = true;
                    break;
                case MESSAGE_NONE:
                    goto loop_end;
            }
        loop_end:;
        
        color_t current_color = get_color();
        
        if(color_matches(color_to_find, current_color))
        {
            if(detects_wall())
            {
                message_send(current_color == COLOR_RED ? MESSAGE_FIND_BLUE : MESSAGE_FIND_RED);
                
                if(current_color == COLOR_RED)
                    message_send(MESSAGE_DO_PID);
                
                else if(ready_for_pid)
                    break;
            }
            else
                move_as(MOVE_LOVER);
        }
        else
            move_as(MOVE_EXPLORER);
        
        // TODO: do not cross lines
    }
    
    camera_stop();
    
    // TODO: execute PID
    
    // TODO: phase 2
    
    message_send(MESSAGE_DO_PID);
    
    wb_robot_cleanup();
    
    return EXIT_SUCCESS;
}
