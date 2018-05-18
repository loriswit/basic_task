#ifndef BASIC_TASK_COLOR_H
#define BASIC_TASK_COLOR_H

#include <stdbool.h>

/**
 * Enum constants representing the color to find.
 * <ul>
 *   <li><b>COLOR_RED</b>: the red color.</li>
 *   <li><b>COLOR_BLUE</b>: the blue color.</li>
 *   <li><b>COLOR_ANY</b>: any of the red or blue color.</li>
 *   <li><b>COLOR_NONE</b>: neither the red or blue color.</li>
 * </ul>
 */
typedef enum
{
    COLOR_RED, COLOR_BLUE, COLOR_ANY, COLOR_NONE
} color_t;

/**
 * Returns the current color detected by the camera.
 *
 * @return The current color being one of <b>COLOR_BLUE</b>, <b>COLOR_RED</b> or <b>COLOR_NONE</b>
 */
color_t get_color();

/**
 * Tells if two colors (red/blue/none) are matching, such as:
 * <ul>
 *   <li><b>COLOR_NONE</b> does not match with any other color.</li>
 *   <li><b>COLOR_ANY</b> matches with <b>COLOR_BLUE/RED</b> or <b>COLOR_ANY</b>.</li>
 *   <li><b>COLOR_BLUE/RED</b> matches with <b>COLOR_BLUE/RED</b> respectively.</li>
 * </ul>
 *
 * @param a The first color
 * @param b The second color
 * @return <b>true</b> if both colors match, <b>false</b> if not
 */
bool color_matches(color_t a, color_t b);

#endif //BASIC_TASK_COLOR_H
