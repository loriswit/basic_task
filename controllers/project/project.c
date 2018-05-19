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
    leds_init();
    prox_init();
    ground_init();
    camera_init();
    com_init();
    
    color_t color_to_find = COLOR_ANY;
    color_t color_found = COLOR_NONE;
    
    while(running)
    {
        leds_set(false);
        
        while(true)
            switch(message_receive())
            {
                case MESSAGE_FIND_RED:
                    color_to_find = COLOR_RED;
                    break;
                case MESSAGE_FIND_BLUE:
                    color_to_find = COLOR_BLUE;
                    break;
                case MESSAGE_NONE:
                    goto loop_end;
            }
        loop_end:;
        
        if(color_found == COLOR_BLUE)
        {
            led_set(0, true);
            led_set(1, true);
            led_set(2, true);
        }
        else if(color_found == COLOR_RED)
        {
            led_set(0, true);
            led_set(6, true);
            led_set(7, true);
        }
        
        if(color_matches(color_to_find, color_found))
        {
            if(color_to_find == COLOR_BLUE)
                break;
            else
                continue;
        }
        else if(color_found != COLOR_NONE && color_found != color_to_find)
        {
            color_found = COLOR_NONE;
            camera_init();
        }
        
        color_t current_color = get_color();
        
        if(current_color == COLOR_RED)
            led_set(7, true);
        else if(current_color == COLOR_BLUE)
            led_set(1, true);
        
        if(color_matches(color_to_find, current_color))
        {
            if(detects_wall(WALL_FRONT))
            {
                color_found = current_color;
                
                motors_stop();
                camera_stop();
                
                message_send(current_color == COLOR_RED ? MESSAGE_FIND_BLUE : MESSAGE_FIND_RED);
            }
            else
                move_as(MOVE_LOVER);
        }
        else
            move_as(MOVE_EXPLORER);
        
        avoid_lines();
    }
    
    while(running)
    {
        if(!detects_wall(WALL_RIGHT))
            rotate(ROTATE_LEFT);
        else
            break;
    }
    
    while(running)
    {
        follow_wall(false);
        
        if(detects_line())
            break;
    }
    
    while(running)
    {
        follow_wall(false);
        
        if(!detects_line())
            break;
    }
    
    while(running)
    {
        if(ground_get_value(GROUND_RIGHT) > 500)
            rotate(ROTATE_LEFT);
        else
            break;
    }
    
    while(running)
    {
        follow_line();
        
        if(detects_wall(WALL_FRONT))
            break;
    }
    
    while(running)
    {
        if(!detects_wall(WALL_RIGHT))
            rotate(ROTATE_LEFT);
        else
            break;
    }
    
    while(running)
    {
        follow_wall(true);
        
        if(detects_line() || message_receive() == MESSAGE_STOP)
            break;
    }
    
    motors_stop();
    message_send(MESSAGE_FIND_BLUE);
    leds_set(true);
    
    while(running)
    {
        if(detects_wall(WALL_BACK))
            break;
    }
    
    message_send(MESSAGE_STOP);
    
    wb_robot_cleanup();
    
    return EXIT_SUCCESS;
}
