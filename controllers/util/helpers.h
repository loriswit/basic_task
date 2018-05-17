#ifndef BARCODE_HELPERS_H
#define BARCODE_HELPERS_H

/**
 * Returns the time elapsed until now.
 *
 * @return The elapsed time in seconds.
 */
#define now wb_robot_get_time

/**
 * Prints a formatted line, including the name of the robot.
 *
 * @param format The line to be formatted
 * @param ... The additional arguments
 *
 * @return The total number of characters written
 */
#define print(format, ...) printf("[%s] " format, wb_robot_get_name(), ##__VA_ARGS__)

/**
 * Prints a formatted line ending with a line break, including the name of the robot.
 *
 * @param format The line to be formatted
 * @param ... The additional arguments
 *
 * @return The total number of characters written
 */
#define println(format, ...) printf("[%s] " format "\n", wb_robot_get_name(), ##__VA_ARGS__)

#endif //BARCODE_HELPERS_H
