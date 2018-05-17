#ifndef ROBOTICS_CAMERA_H
#define ROBOTICS_CAMERA_H

#define CAM_RATE 8

typedef struct
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} rgb_color;

/**
 * Initializes the camera.
 */
void camera_init();

/**
 * Computes and returns the average color of the current image.
 *
 * @return The average RGB color
 */
rgb_color camera_get_average_color();

#endif //ROBOTICS_CAMERA_H
