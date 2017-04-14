/*
 * led_pattern.h
 *
 *	This code handles a LED indicator and can play various
 *	patterns. You can add your own custom patterns.
 *
 *	Insert the led_update() in any timer IRQ. This will update
 *	the pattern every time is called, so the faster the IRQ the
 *	faster the pattern will change. On each function call the
 *	next bit of the pattern is used on the LED and 0 means off
 *	and 1 means on.d
 *
 *  Created on: 1 Apr 2017
 *      Author: dimtass
 */

#ifndef LED_PATTERN_H_
#define LED_PATTERN_H_

#include "stm32f10x.h"

/**
 * LED patterns. Insert your custom patterns in here
 * 0: LED off
 * 1: LED on
 */
typedef enum {
	LED_PATTERN_ESP8266_INIT = 0b10101010,
	LED_PATTERN_HEARTBEAT = 0b00001111,
} en_led_pattern;

/**
 * @brief Initialize LED
 * @param[in] led_port The LED's port
 * @param[in] pin The port pin
 */
void led_init(GPIO_TypeDef * led_port, uint16_t pin);

/**
 * @brief Update the LED pattern
 */
void led_update(void);

/**
 * @brief Set the active pattern
 * @param[in] pattern The pattern to set as active
 */
void led_set_pattern(en_led_pattern pattern);

#endif /* LED_PATTERN_H_ */
