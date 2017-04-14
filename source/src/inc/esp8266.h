/*
 * http_server.h
 *
 *  Created on: 6 Apr 2017
 *      Author: dimtass
 *
 *	Insert this line in the 1ms interrupt handler
 * 		if (esp8266_uart->rx_ready) esp8266_uart->rx_ready_tmr++;
 *
 * 	Insert this line in the UART IRQ handler
 * 		esp8266_uart_irq();
 *
 * 	Insert this line in the main loop
 * 		esp8266_poll_recv();
 *
 * 	Use esp8266_cb_register() to register a callback function for
 * 	the incoming TCP/UDP data from the ESP8266.
 */

#ifndef __ESP8266_H__
#define __ESP8266_H__

#include <stdint.h>
#include "stm32f10x.h"
#include "uart_buffer.h"

extern tp_uart * esp8266_uart;		// pointer to the debug tp_uart buffer
extern uint16_t esp8266_msec_cntr;	// msec timer for internal timeouts

typedef enum {
	ESP8266_FAIL = -1,
	ESP8266_TIMEOUT = 0,
	ESP8266_OK = 1,
} en_esp8266_resp;

typedef enum {
	ESP8266_MODE_STATION = 1,
	ESP8266_MODE_AP = 2,
	ESP8266_MODE_AP_STATION = 3,
} en_esp8266_mode;

typedef enum {
	ESP8266_AP_ECN_OPEN = 0,
	ESP8266_AP_ECN_WPA_PSK = 2,
	ESP8266_AP_ECN_WPA2_PSK = 3,
	ESP8266_AP_ECN_WPA_WPA2_PSK = 4
} en_esp8266_ap_ecn;

typedef void (*esp8266_rcv_callback)(const uint8_t *data, size_t data_len, uint8_t mux_ch);

/**
 * @brief Initialize the debug UART
 * @param[in] port The STM32 uart port
 * @param[in] baudrate The port baudrate
 */
void esp8266_init(USART_TypeDef * port, size_t baudrate);

/**
 * @brief The IRQ handler for the UART interface.
 * 		This should be called from the UART IRQ handler
 * 		of the application (e.g. stm32f10x_it.c)
 */
void esp8266_uart_irq(void);

/**
 * @brief Send a null terminated string to ESP866 module
 * @param[in] buffer The string to send
 * @return int The number of bytes sent
 */
int esp8266_uart_send(const char * buffer);

/**
 * @brief Sent a buffer to ESP8266
 * @param[in] buffer The buffer to sent
 * @param[in] buffer_len The length of the buffer
 * @return int The number of bytes sent
 */
int esp8266_uart_send_len(const char * buffer, size_t buffer_len);

/**
 * @brief Receive data from ESP8266 uart
 * @param[out] buffer A pointer to the incoming uart data
 * @param[in] timeout_ms Timeout for waiting data
 * @return int The length of incoming data
 */
int esp8266_uart_receive(char ** buffer, size_t timeout_ms);

/**
 * @brief Registers a callback function for incoming TCP/UDP data
 * @param[in] esp8266_rcv_callback The callback function
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
void esp8266_cb_register(esp8266_rcv_callback cb);

/**
 * @brief Polls for incoming data. Use this function only after you register a callback with esp8266_cb_register()
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int esp8266_poll_recv();

/**
 * @brief Get ESP8266 device IP
 * @param[out] ip The IP address
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int esp8266_get_IP(char ** ip);

/**
 * @brief Get MAC address
 * @param[out] mac_addr The AP mac address
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 *
 */
int esp8266_get_AP_MAC(char ** mac_addr);

/**
 * @brief Send AT to the module. This way you check if module is responsive
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int eAT(void);


int eATE(uint8_t echo);
/**
 * @brief Reset ESP module
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int eATRST(void);

/**
 * @brief Set the module operation mode
 * @param[in] mode This is the operation mode, see en_esp8266_mode
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int sATCWMODE(uint8_t mode);

/**
 * @brief Get the module's operation mode
 * @param[out] mode The current operation mode, see en_esp8266_mode
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int qATCWMODE(uint8_t * mode);

/**
 * @brief Joins a AP
 * @param[in] ssid The AP's SSID name
 * @param[in] pwd The AP's password
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int sATCWJAP(char * ssid, char * pwd);

/**
 * @brief Lists AP in range
 * @param[out] list A list of the APs in range
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int eATCWLAP(char ** list);

/**
 * @brief Gets software versions
 * @param[out] version A char string with the versions
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int eATGMR(char ** version);

/**
 * @brief Pings a URL
 * @param[in] url An IP or domain to ping
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int eATPING(char * url);

/**
 * @brief Configures the module's AP (WARNING: This is deprecated. See: sATCWSAP_CUR)
 * @param[in] ssid The SSID name to create
 * @param[in] pwd The AP's password
 * @param[in] chl The AP's channel
 * @param[in] ecn The encryption protocol
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int sATCWSAP(char * ssid, char * pwd, uint8_t chl, en_esp8266_ap_ecn ecn);

/**
 * @brief Same as sATCWSAP. Sets the config without save it in flash.
 * @param[in] ssid The SSID name to create
 * @param[in] pwd The AP's password
 * @param[in] chl The AP's channel
 * @param[in] ecn The encryption protocol
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int sATCWSAP_CUR(char * ssid, char * pwd, uint8_t chl, en_esp8266_ap_ecn ecn);

/**
 * @brief Same as sATCWSAP. Sets the AP config and also saves it in flash.
 * @param[in] ssid The SSID name to create
 * @param[in] pwd The AP's password
 * @param[in] chl The AP's channel
 * @param[in] ecn The encryption protocol
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int sATCWSAP_DEF(char * ssid, char * pwd, uint8_t chl, en_esp8266_ap_ecn ecn);

/**
 * @brief Get the IP of stations connected to ESP8266 softAP
 * @param[out] list The list of IPs
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int eATCWLIF(char ** list);

/**
 * @brief Check network connection status
 * @param[in] list The connection status
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int eATCIPSTATUS(char ** list);

/**
 * @brief Establish TCP connection, UDP transmition or SSL connection
 * @param[in] type Connection type [TCP,UDP,SSL]
 * @param[in] remote_addr Remote IP address
 * @param[in] remote_port Remote port number
 * @param[in] local_port Local port number
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int sATCIPSTARTSingle(char * type, char * remote_addr, uint16_t remote_port, uint16_t local_port);

/**
 * @brief Establish TCP connection, UDP transmition or SSL connection for multi-connection
 * @param[in] mux_id The ID of network connection
 * @param[in] type Connection type [TCP,UDP,SSL]
 * @param[in] remote_addr Remote IP address
 * @param[in] port Remote port number
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int sATCIPSTARTMultiple(uint8_t mux_id, char * type, char * addr, uint16_t port);

/**
 * @brief Sends data to a single connection
 * @param[in] buffer The data to send
 * @param[in] len The data length
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int sATCIPSENDSingle(const uint8_t *buffer, size_t len);

/**
 * @brief Sends data to a multi-connection
 * @param[in] mux_id The ID of network connection
 * @param[in] buffer The data to send
 * @param[in] len The data length
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int sATCIPSENDMultiple(uint8_t mux_id, const uint8_t *buffer, size_t len);

/**
 * @brief Close TCP, UDP or SSL connection
 * @param[in] mux_id The ID of network connection
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int sATCIPCLOSEMulitple(uint8_t mux_id);

/**
 * @brief Get local IP address
 * @param[out] list The IP address of ESP8266 softAP or station
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int eATCIFSR(char ** list);

/**
 * @brief Enable multiple connections
 * @param[in] mode 0: single connection, 1: multiple connection
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int sATCIPMUX(uint8_t mode);

/**
 * @brief Configure as TCP server
 * @param[in] mode 0: delete server, 1: Create server
 * @param[in] port Port number
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int sATCIPSERVER(uint8_t mode, uint16_t port);

/**
 * @brief Set TCP server timeout
 * @param[in] timeout TCP server timeout, range 0~7200 seconds. If 0, never timeouts (not recommended)
 * @return int Returns ESP8266_OK on success, otherwise a negative value
 */
int sATCIPSTO(size_t timeout);

#endif //__ESP8266_H__
