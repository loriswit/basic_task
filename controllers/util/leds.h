#ifndef ROBOTICS_LEDS_H
#define ROBOTICS_LEDS_H

#include <stdbool.h>

#define LEDS_COUNT 8

/**
 * Initializes all LEDs.
 */
void leds_init();

/**
 * Turns all LEDs on or off.
 *
 * @param value TRUE to turn LEDs on, FALSE to turn off
 */
void leds_set(bool value);

/**
 * Turns a specific LED on or off.
 *
 * @param index The LED index
 * @param value TRUE to turn the LED on, FALSE to turn off
 */
void led_set(size_t index, bool value);

/**
 * Turns all LEDs on or off according to the current step.
 */
void leds_flash();

/**
 * Turns each LED on then off one after another.
 */
void leds_spin();

#endif //ROBOTICS_LEDS_H
