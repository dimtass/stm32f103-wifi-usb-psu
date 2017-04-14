/*
 * led_pattern.c
 *
 *  Created on: 1 Apr 2017
 *      Author: dimtass
 */
#include "led_pattern.h"

/* system */
static uint8_t m_led_pattern = 0;
static uint8_t m_led_pattern_index = 0;
static GPIO_TypeDef * m_gpio_port = (void*)0;
static uint16_t m_led_pin = 0;

void led_init(GPIO_TypeDef * led_port, uint16_t pin)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	if (led_port == GPIOA) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	}
	else if (led_port == GPIOB) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	}
	else if (led_port == GPIOC) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	}

	m_gpio_port = led_port;
	m_led_pin = pin;
	m_led_pattern = 0;
	m_led_pattern_index = 0;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

	/* Configure LED */
	GPIO_InitStructure.GPIO_Pin = m_led_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(m_gpio_port, &GPIO_InitStructure);
	m_gpio_port->ODR &= ~m_led_pin;
}


void led_set_pattern(en_led_pattern pattern)
{
	m_led_pattern = (uint16_t) pattern;
}

void led_update(void)
{
	/* Enable relay port */
	if (m_led_pattern & (1 << m_led_pattern_index) )
		m_gpio_port->ODR |= m_led_pin;
	else
		m_gpio_port->ODR &= ~m_led_pin;
	if ((++m_led_pattern_index) == 8)
		m_led_pattern_index = 0;
}

