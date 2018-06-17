/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : platform_config.h
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Evaluation board specific configuration file.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include "stm32f10x.h"
#include "debug_uart.h"

#define DEBUG_TRACE

/**
 * System options
 */
#define TMR_1MSEC
#define TMR_100MSEC
#define TMR_250MSEC
#define TMR_1000MSEC
#define USE_SPI_DMA		// If its defined then the DMA spi is used
#define USE_USB_IF		// If its set then USB interface is used instead of UART

/* Declare TX/RX buffers */
#ifdef USE_USB_IF
#define USB_RX_BUFF_SIZE	2048 //VIRTUAL_COM_PORT_DATA_SIZE
#define USB_TX_BUFF_SIZE	2048 //VIRTUAL_COM_PORT_DATA_SIZE
#define UART_RX_BUFF_SIZE	128
#define UART_TX_BUFF_SIZE	512
#else
#define USB_RX_BUFF_SIZE	64 //VIRTUAL_COM_PORT_DATA_SIZE
#define USB_TX_BUFF_SIZE	64 //VIRTUAL_COM_PORT_DATA_SIZE
#define UART_RX_BUFF_SIZE	2048
#define UART_TX_BUFF_SIZE	1024
#endif


#ifdef DEBUG_TRACE
#define TRACE(X) TRACEL(TRACE_LEVEL_DEFAULT, X)
#define TRACEL(TRACE_LEVEL, X) if (debug_uart_check_trace_level(TRACE_LEVEL)) printf X
#else
#define TRACE(X)
#endif

typedef struct {
	uint16_t ptr_in;
	uint16_t ptr_out;
	uint16_t length;
	uint8_t  int_en;
} tp_buff_pointers;

#define FLASH_PREAMBLE 		0xABCD
#define FLASH_CONF_VERSION	1
#define NUM_OF_POT_VALUES	5
typedef struct {
	uint16_t	preamble;			// a preamble to check that flash contains valid data
	uint8_t		version;			// configuration version
	/* ESP8266 specific params */
	char		ssid_name[40];		// SSID name that will be used for ESP8266
	char 		ssid_passwd[40];	// the password of the above AP
	/* pot specific params */
	uint8_t		pot_values[NUM_OF_POT_VALUES];	// the default POT values
} tp_conf;

typedef struct {
	/* Timers */
#ifdef TMR_1MSEC
	uint16_t tmr_1ms;
#endif
#ifdef TMR_100MSEC
	uint16_t tmr_100ms;
#endif
#ifdef TMR_250MSEC
	uint16_t tmr_250ms;
#endif
#ifdef TMR_1000MSEC
	uint16_t tmr_1sec;
#endif
	uint8_t tmr_boot_delay;
	uint16_t tmr_fpga_sm_tmr;
	uint16_t tmr_reset;	// When
	/* USB buffers */
	uint8_t usb_tx_ready;
	uint8_t	usb_rx_ready;
	uint8_t	usb_rx_ready_tmr;
	uint8_t usb_rx_buff[USB_RX_BUFF_SIZE];
	tp_buff_pointers usb_rx_buff_p;
	uint8_t usb_tx_buff[USB_TX_BUFF_SIZE];
	tp_buff_pointers usb_tx_buff_p;
	/* UART buffers */
	uint8_t uart_rx_buff[UART_RX_BUFF_SIZE];
	uint8_t uart_tx_buff[UART_TX_BUFF_SIZE];
	/* pot value */
	uint8_t * pot_value;

	tp_conf	conf;
} tp_glb;

extern tp_glb glb;

/* GPIOs */
#define GPIO_PIN_ADC 		GPIO_Pin_0
#define GPIO_PIN_ESP82266_TX GPIO_Pin_2
#define GPIO_PIN_ESP82266_RX GPIO_Pin_3
#define GPIO_PORT_SPI		GPIOA
#define GPIO_PIN_SPI_NSS 	GPIO_Pin_4
#define GPIO_PIN_SPI_SCK 	GPIO_Pin_5
//#define GPIO_PIN_SPI_MISO 	GPIO_Pin_6
#define GPIO_PIN_SPI_MOSI 	GPIO_Pin_7
#define GPIO_PIN_DEBUG_TX 	GPIO_Pin_9
#define GPIO_PIN_DEBUG_RX 	GPIO_Pin_10
#define GPIO_PIN_RELAY 		GPIO_Pin_12
#define GPIO_PORT_RELAY		GPIOB
#define GPIO_PIN_LED		GPIO_Pin_13
#define GPIO_PORT_LED		GPIOC

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#endif /* __PLATFORM_CONFIG_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
