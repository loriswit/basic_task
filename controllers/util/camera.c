#include <stdlib.h>

#include <webots/camera.h>
#include <webots/robot.h>

#include "camera.h"
#include "consts.h"

WbDeviceTag camera;
int width;
int height;

void camera_init()
{
    camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, CAM_RATE * TIME_STEP);
    width = wb_camera_get_width(camera);
    height = wb_camera_get_height(camera);
}

rgb_color camera_get_average_color()
{
    const unsigned char * image = wb_camera_get_image(camera);
    
    unsigned red = 0;
    unsigned green = 0;
    unsigned blue = 0;
    
    for(size_t x = 0; x < width; ++x)
        for(size_t y = 0; y < height; ++y)
        {
            red += wb_camera_image_get_red(image, width, x, y);
            green += wb_camera_image_get_green(image, width, x, y);
            blue += wb_camera_image_get_blue(image, width, x, y);
        }
    
    int pixels_count = width * height;
    
    return (rgb_color) {
            (unsigned char) (red / pixels_count),
            (unsigned char) (green / pixels_count),
            (unsigned char) (blue / pixels_count)};
}
