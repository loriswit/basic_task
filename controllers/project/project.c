#include <stdio.h>
#include <webots/robot.h>

#include "message.h"
#include "color.h"
#include "move.h"
#include "detection.h"

#include "../util/motors.h"
#include "../util/prox_sensors.h"
#include "../util/ground_sensors.h"
#include "../util/leds.h"
#include "../util/camera.h"
#include "../util/com.h"
#include "../util/consts.h"

#define println(format, ...) printf("[%s] " format "\n", wb_robot_get_name(), ##__VA_ARGS__)

#define running wb_robot_step(TIME_STEP) != -1

/**
 * First phase: find color blocks
 */
void find_color();

/**
 * Second phase: follow the blue block
 */
void follow_block();

/**
 * Third phase: follow the line
 */
void follow_line();

/**
 * Last phase: follow the arena walls
 */
void follow_wall();

int main()
{
    // initialize each component
    wb_robot_init();
    motors_init();
    leds_init();
    prox_init();
    ground_init();
    camera_init();
    com_init();
    
    // execute each phase
    find_color();
    follow_block();
    follow_line();
    follow_wall();
    
    println("Task completed.");
    
    // stop and tells the other robot to find blue
    leds_set(true);
    motors_stop();
    message_send(MESSAGE_FIND_BLUE);
    
    // wait until the next robot reaches its back
    while(running && !detects_wall(WALL_BACK));
    message_send(MESSAGE_STOP);
    
    wb_robot_cleanup();
    
    return EXIT_SUCCESS;
}

void find_color()
{
    println("Finding color blocks.");
    
    color_t color_to_find = COLOR_ANY;
    color_t color_found = COLOR_NONE;
    
    while(running)
    {
        // process received messages
        message_t msg = message_receive();
        if(msg == MESSAGE_FIND_RED)
            color_to_find = COLOR_RED;
        else if(msg == MESSAGE_FIND_BLUE)
            color_to_find = COLOR_BLUE;
        
        // if the color to find is blue and it has been found, go to the next phase
        if(color_to_find == COLOR_BLUE && color_found == COLOR_BLUE)
            break;
        
        // if the robot has already found a color, skip the rest
        if(color_matches(color_to_find, color_found))
            continue;
        
        // if the color to find has changed, turn the camera on again
        if(color_found != COLOR_NONE && color_found != color_to_find)
        {
            color_found = COLOR_NONE;
            camera_init();
        }
        
        // process the color currently in front of the robot
        color_t current_color = get_color();
        if(color_matches(color_to_find, current_color))
        {
            if(detects_wall(WALL_FRONT))
            {
                // send the color that the other robot has to find
                color_found = current_color;
                message_send(color_found == COLOR_RED ? MESSAGE_FIND_BLUE : MESSAGE_FIND_RED);
                
                motors_stop();
                camera_stop();
            }
            else
                move_as(MOVE_LOVER);
        }
        else
            move_as(MOVE_EXPLORER);
        
        // do not cross lines
        avoid_lines();
        
        // apply LEDs codes
        leds_set(false);
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
        else if(current_color == COLOR_RED)
            led_set(7, true);
        else if(current_color == COLOR_BLUE)
            led_set(1, true);
    }
}

void follow_block()
{
    println("Following blue block.");
    
    // rotate until a wall is detected on the right side
    while(running && !detects_wall(WALL_RIGHT))
        rotate(ROTATE_LEFT);
    
    // apply LEDs code (wall following on right side)
    leds_set(false);
    led_set(1, true);
    led_set(2, true);
    led_set(3, true);
    
    // follow the block until the robot has fully crossed the line
    bool over_line = false;
    while(running)
    {
        follow_prox(0.8, SIDE_RIGHT, 0.0015, 10, 0.2);
        
        if(detects_line(1, 1, 1))
            over_line = true;
        else if(over_line)
            break;
    }
}

void follow_line()
{
    println("Following line.");
    
    // rotate until a line is detected by the right ground sensor
    while(running && !detects_line(0, 0, 1))
        rotate(ROTATE_LEFT);
    
    // apply LEDs code (line following)
    leds_set(false);
    led_set(3, true);
    led_set(4, true);
    led_set(5, true);
    led_set(6, true);
    
    // follow the line until a wall is detected
    while(running && !detects_wall(WALL_FRONT))
        follow_ground(1.0 / 3);
}

void follow_wall()
{
    println("Following arena walls.");
    
    // rotate until a wall is detected on the left side
    while(running && !detects_wall(WALL_LEFT))
        rotate(ROTATE_RIGHT);
    
    // apply LEDs code (wall following on left side)
    leds_set(false);
    led_set(5, true);
    led_set(6, true);
    led_set(7, true);
    
    // follow the walls until a line is detected (or MESSAGE_STOP is received)
    while(running && !detects_line(1, 1, 1))
    {
        follow_prox(1.5, SIDE_LEFT, 0.0015, 6, 0.05);
        
        if(message_receive() == MESSAGE_STOP)
            break;
    }
}
