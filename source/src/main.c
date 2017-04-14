/**
  ******************************************************************************
  * @file    main.c
  * @author  Dimitris Tassopoulos
  * @version V1.0
  * @date    29-Mar-2017
  * @brief   Default main function.
  *
  * This project if for implementing a USB/WIFI PSU using an LM2596S board,
  * an MPC41010 (10KOhm) resistor, an ESP-8266 wifi module and an output relay.
  * The maximum voltage output is depended on the input voltage of the LM2596S.
  * Every time the input voltage on the LM2596S or any other element that affects
  * the output voltage is changing then the circuit needs re-calibration.
  *
  * SPI1 	: MPC41010 resistor
  * UART1	: Debug port
  * UART2	: ESP-8266
  * A0		: Analog input
  * USB		: for USB input/output commands
  *
  * Pinout:
  * PA0		: ADC1_IN0
  * PA2		: USART2 TX (ESP8266)
  * PA3		: USART2 RX
  * PA5		: SPI1_SCK
  * PA6		: SPI1_MISO
  * PA7		: SPI1_MOSI
  * PA9		: USART1_TX (DEBUG)
  * PA10	: USART1_RX
  * PB12	: RELAY
  *
  * To create a bin file then in post-build steps in properties:
  * arm-none-eabi-objcopy -O binary "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.bin"; arm-none-eabi-size "${BuildArtifactFileName}"
  *
  * To create a hex file:
  * arm-none-eabi-objcopy -O ihex "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.hex"; arm-none-eabi-size "${BuildArtifactFileName}"
  *
  * To flash on windows:
  * "C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe" -c SWD -p stm32f103_wifi_usb_psu.hex -Rst
  ******************************************************************************
*/

#include <esp8266.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "platform_config.h"
#include "led_pattern.h"
#include "mcp41xxx.h"
#include "http_server.h"
#include "flash.h"

#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"

#define VERSION "0.1"	// firmware version

tp_glb glb;		// global data (defined here and exported in platform_config.h)

enum {
	CMD_SENDER_UART,
	CMD_SENDER_USB
};

enum {
	CMD_RESP_ERROR,
	CMD_RESP_OK
};

/* global functions */
void tcp_server_start(uint16_t port);
void tcp_handler(const uint8_t *data, size_t data_len, uint8_t mux_ch);
void cmd_handler(uint8_t *buffer, size_t bufferlen, uint8_t sender);
void remove_newline(char * buffer);

/**
 * @brief Start a TCP server on the ESP8266
 * @param[in] port The server's listening port
 */
void tcp_server_start(uint16_t port)
{
	char * str;

	/* reset esp8266 */
	eATRST();
	/* wait for esp8266 */
	while (!eAT());
	/* disable echo */
	eATE(0);
	/* connect to AP */
	sATCWJAP(glb.conf.ssid_name, glb.conf.ssid_passwd);
	/* get assigned IP address */
	esp8266_get_IP(&str);
	TRACE(("IP: %s\n", str));
	/* get AP mac address */
	esp8266_get_AP_MAC(&str);
	TRACE(("MAC: %s\n", str));
	/* enable MUX connections */
	sATCIPMUX(1);
	/* sest TCP timeout to 60 secs */
	sATCIPSTO(60);
	/* start TCP server */
	sATCIPSERVER(1, 80);
	/* register tcp_handler callback */
	esp8266_cb_register(tcp_handler);
	TRACE(("TCP registered\n"));
}

/**
 * @brief Handles incoming TCP data.
 * 		You need to register this callback with the esp8266_cb_register()
 * @param[in] data A pointer to the start of the incoming data
 * @param[in] datalen The length of the incoming data
 * @param[in] mux_ch The ESP8266 mux channel
 */
void tcp_handler(const uint8_t *data, size_t data_len, uint8_t mux_ch)
{
	TRACE(("tcp_data[%d]:%d\n", mux_ch, data_len));

	if (!strncmp((char*)data, "POT=", 4)) {
		uint8_t pot_val = strtoul((char*) &data[4], NULL, 10);
		TRACE(("Setting POT to: %d\n", pot_val));
		mcp41xxx_set_value(1, pot_val);
		sATCIPSENDMultiple(mux_ch, (uint8_t*) "\r\nOK\r\n", 6);
	}
	else {
		http_handler(mux_ch, (char*)data, data_len);
	}
	sATCIPCLOSEMulitple(mux_ch);
}

/**
 * @brief USB/Debug uart RX data handler
 * @param[in] buffer The incoming data buffer
 * @param[in] bufferlen The number of incoming bytes
 */
void cmd_handler(uint8_t *buffer, size_t bufferlen, uint8_t sender)
{
	uint8_t cmd_resp = CMD_RESP_ERROR;
#define RESP_STR_LEN 100
	char resp_str[RESP_STR_LEN] = {0};

	/* remove newline from incoming buffer */
	remove_newline((char*) buffer);

	TRACE(("cmd_handler[%d][%d]: %s~\n", sender, bufferlen, buffer));

	/* Set the POT value */
	if (!strncmp((char*)buffer, "POT=", 4)) {
		uint8_t pot_val = strtoul((char*) &buffer[4], NULL, 10);
		*glb.pot_value = pot_val;
		mcp41xxx_set_value(1, *glb.pot_value);
		cmd_resp = CMD_RESP_OK;
	}
	/* Save the pot value */
	else if (!strncmp((char*)buffer, "SETPOT=", 7)) {
		char * p = strtok((char*) &buffer[7], ",");
		uint8_t id = strtoul(p, NULL, 10);
		TRACE(("id: %d\n", id));
		p = strtok(NULL, ",");
		uint8_t value = 0xFF & strtoul(p, NULL, 10);
		TRACE(("value: %d\n", value));

		/* save new default pot value */
		if ((id > 0) && (id <= NUM_OF_POT_VALUES)) {
			glb.conf.pot_values[id-1] = value;
			FLASH_write((uint8_t*) &glb.conf, sizeof(tp_conf));
			cmd_resp = CMD_RESP_OK;
		}
	}
	else if (!strncmp((char*)buffer, "GETPOT=", 7)) {
		uint8_t id = strtoul((char*)&buffer[7], NULL, 10);
		/* save new default pot value */
		if ((id > 0) && (id <= NUM_OF_POT_VALUES)) {
			snprintf(resp_str, RESP_STR_LEN, "GETPOT:%d,%d\n", id, glb.conf.pot_values[id-1]);
			cmd_resp = CMD_RESP_OK;
		}
	}
	/* Save conf */
	else if (!strncmp((char*)buffer, "SAVE", 4)) {
		FLASH_write((uint8_t*) &glb.conf, sizeof(tp_conf));
		cmd_resp = CMD_RESP_OK;
	}
	/* Set SSID name */
	else if (!strncmp((char*)buffer, "SSID=", 5)) {
		strncpy(glb.conf.ssid_name, (char*) &buffer[5], sizeof(glb.conf.ssid_name)/sizeof(glb.conf.ssid_name[0]));
		FLASH_write((uint8_t*) &glb.conf, sizeof(tp_conf));
		cmd_resp = CMD_RESP_OK;
	}
	/* Set AP password */
	else if (!strncmp((char*)buffer, "PASS=", 5)) {
		strncpy(glb.conf.ssid_passwd, (char*) &buffer[5], sizeof(glb.conf.ssid_passwd)/sizeof(glb.conf.ssid_passwd[0]));
		FLASH_write((uint8_t*) &glb.conf, sizeof(tp_conf));
		cmd_resp = CMD_RESP_OK;
	}
	/* reconnect to AP */
	else if (!strncmp((char*)buffer, "RECONNECT", 9)) {
		tcp_server_start(80);
		cmd_resp = CMD_RESP_OK;
	}
	/* shut down POT */
	else if (!strncmp((char*)buffer, "POT_SHUTDN", 10)) {
		mcp41xxx_shut_down(1);
		cmd_resp = CMD_RESP_OK;
	}
	/* print firmware version */
	else if (!strncmp((char*)buffer, "VERSION", 7)) {
		snprintf(resp_str, RESP_STR_LEN, "Version: %s, %s, %s\n", VERSION, __DATE__, __TIME__);
		cmd_resp = CMD_RESP_OK;
	}
	/* send custom uart command */
	else if (!strncmp((char*)buffer, "AT=", 3)) {
		esp8266_uart_send_len((char*)&buffer[3], bufferlen-3);
		esp8266_uart_send("\r\n");
		char * resp = NULL;
		int resp_len = esp8266_uart_receive(&resp, 10000);
		if (resp_len) {
			snprintf(resp_str, RESP_STR_LEN, "RESP[%d]: %s", resp_len, resp);
		}
		else {
			snprintf(resp_str, RESP_STR_LEN, "RESP[%d]", resp_len);
		}
		cmd_resp = CMD_RESP_OK;
	}

	if (sender == CMD_SENDER_UART) {
		if (cmd_resp) {
			if (strlen(resp_str)) {
				TRACE(("%s", resp_str));
			}
			TRACE(("OK\n"));
		}
		else {
			TRACE(("ERROR\n"));
		}
	}
	else if (sender == CMD_SENDER_USB) {
		TRACE(("USB: %d, %d\n", cmd_resp, strlen(resp_str)));
		if (cmd_resp) {
			if (strlen(resp_str)) {
				TRACE(("Sending %d bytes to USB\n", strlen(resp_str)));
				CDC_Send_DATA((uint8_t*) resp_str, strlen(resp_str));
			}
			CDC_Send_DATA((uint8_t*) "OK\n", 3);
		}
		else {
			CDC_Send_DATA((uint8_t*) "ERROR\n", 6);
		}
	}
}

/**
 * @brief Main loop.
 */
static inline void main_loop(void)
{
	/* 1 ms timer */
	if (glb.tmr_1ms) {
		glb.tmr_1ms = 0;
	}

	/* 100 ms timer */
	if (glb.tmr_100ms >= 100) {
		glb.tmr_100ms = 0;
	}

	/* 1 second timer */
	if (glb.tmr_1sec >= 1000) {
		glb.tmr_1sec = 0;
	}

	/* poll for RX data in the debug interface */
	debug_uart_rx_poll();
	/* poll for RX data in the esp8266 interface */
	esp8266_poll_recv();

	/* USB data reception */
	if (bDeviceState == CONFIGURED) {
		CDC_Receive_DATA();
		if (glb.usb_rx_ready_tmr >= 2) {
			glb.usb_rx_ready = 0;
			glb.usb_rx_ready_tmr = 0;
//			TRACE(("usb:%d\n", glb.usb_rx_buff_p.length));
			/* Handle USB event */
			cmd_handler(glb.usb_rx_buff, glb.usb_rx_buff_p.ptr_in, CMD_SENDER_USB);
			glb.usb_rx_buff_p.ptr_in = 0;
			glb.usb_rx_buff_p.ptr_out = 0;
			glb.usb_rx_buff_p.length = 0;
		}
	}
}

int main(void)
{
	if (SysTick_Config(SystemCoreClock / 1000)) {
		/* Capture error */
		while (1);
	}

	/* GPIO Configuration */
	GPIO_Configuration();
	/* Init LED on PC13 */
	led_init(GPIO_PORT_LED, GPIO_PIN_LED);

	/* setup debug TRACE interface */
	debug_uart_init(USART1, 115200, glb.uart_rx_buff, UART_RX_BUFF_SIZE, glb.uart_tx_buff, UART_TX_BUFF_SIZE, &cmd_handler);
	debug_uart_set_trace_level(TRACE_LEVEL_DEFAULT, 1);
	debug_uart_set_trace_level(TRACE_LEVEL_USB, 1);
//	debug_uart_set_trace_level(TRACE_LEVEL_ESP, 1);
//	debug_uart_set_trace_level(TRACE_LEVEL_FLASH, 1);

	/* USB Configuration */
	TRACE(("Init USB\n"));
	glb.usb_rx_buff_p.length = glb.usb_rx_buff_p.ptr_in = glb.usb_rx_buff_p.ptr_out = 0;
	glb.usb_tx_buff_p.length = glb.usb_tx_buff_p.ptr_in = glb.usb_tx_buff_p.ptr_out = 0;
	glb.usb_tx_ready = 1;
	USB_Reset(1000);
	USB_Configuration();
	USB_Init();

	/* setup ESP8266 */
	TRACE(("Init ESP8266...\n"));
	esp8266_init(USART2, 115200);

	/* setup mcp41xxx */
	TRACE(("Init MCP41010...\n"));
	glb.pot_value = &glb.conf.pot_values[0];	// set default pot value
	mcp41xxx_init(*glb.pot_value);	// set the pot value

	TRACE(("Init Flash...\n"));
	/* Initialize STM32 flash */
	FLASH_Init(0x0800FC00, 1);	// start address=0x0800FC00 and 1 block is used

	/* read the configuration from the flash */
	tp_conf tmp_conf;
	FLASH_read((uint8_t*)&tmp_conf, sizeof(tp_conf));
	if (tmp_conf.preamble == FLASH_PREAMBLE) {
		/* valid configuration data */
		if (tmp_conf.version == FLASH_CONF_VERSION) {
			memcpy(&glb.conf, &tmp_conf, sizeof(tp_conf));
		}
		else {
			/* handle different flash versions */
		}
	}
	else {
		int pot_index = 0;
		/* invalid flash data found. Create defaults */
		glb.conf.preamble = FLASH_PREAMBLE;
		glb.conf.version = FLASH_CONF_VERSION;
		/* default ESP8266 params */
		strncpy(glb.conf.ssid_name, "SSID_NAME", sizeof(glb.conf.ssid_name)/sizeof(glb.conf.ssid_name[0]));
		strncpy(glb.conf.ssid_name, "SSID_PASSWD", sizeof(glb.conf.ssid_passwd)/sizeof(glb.conf.ssid_passwd[0]));
		/* default POT values */
		for (pot_index=0; pot_index<NUM_OF_POT_VALUES; pot_index++) {
			glb.conf.pot_values[pot_index] = 127;
		}
		/* save configuration in flash */
		FLASH_write((uint8_t*) &glb.conf, sizeof(tp_conf));
	}

	/* print conf values */
	TRACE(("Flash conf:\n"));
	TRACE(("\tpreamble: 0x%04X\n", glb.conf.preamble));
	TRACE(("\tversion: %d\n", glb.conf.version));
	TRACE(("\tssid: %s\n", glb.conf.ssid_name));
	TRACE(("\tpass: %s\n", glb.conf.ssid_passwd));

	TRACE(("[WiFi & USB PSU version %s]\n", VERSION));

	/* Start TCP server */
	led_set_pattern(LED_PATTERN_ESP8266_INIT);
	tcp_server_start(80);

	/* srever started */
	led_set_pattern(LED_PATTERN_HEARTBEAT);

	/* main loop */
	while (1) {
		main_loop();
	}
}

void remove_newline(char * buffer)
{
	char * pos = NULL;
	if ((pos=strchr(buffer, '\r')) != NULL) *pos = '\0';
	if ((pos=strchr(buffer, '\n')) != NULL) *pos = '\0';
}

/**
 * This is a (weak) function in syscalls.c and is used from printf
 * to print data to the UART1
 */
int __io_putchar(int ch)
{
	debug_uart_send(ch);
	return ch;
}
