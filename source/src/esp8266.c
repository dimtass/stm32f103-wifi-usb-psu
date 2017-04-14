/*
 * ESP8266.cpp
 *
 *  Created on: 31 Mar 2017
 *      Author: dimtass
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp8266.h"
#include "platform_config.h"

#define STR_MAX_LEN	50

uint16_t esp8266_msec_cntr = 0;
tp_uart m_uart = {0};
tp_uart * esp8266_uart = (tp_uart*) &m_uart;
esp8266_rcv_callback m_cb_func = 0;

static size_t			m_baudrate;
static USART_TypeDef * m_port;

#define RSP8266_RX_BUFFER_SIZE	512
#define RSP8266_TX_BUFFER_SIZE	256

/* ESP buffers */
char rx_buffer[RSP8266_RX_BUFFER_SIZE];
char tx_buffer[RSP8266_TX_BUFFER_SIZE];

void rx_empty(void);
void esp8266_config(void);
int esp8266_uart_send(const char * buffer);
uint8_t esp8266_uart_send_byte(uint8_t byte);
int recvString(const char * target_str, size_t timeout_ms);
int recvString2(const char * target_str1, const char * target_str2, size_t timeout_ms);
int recvString3(const char * target_str1, const char * target_str2, const char * target_str3, size_t timeout_ms);
int recvFind(const char * str, size_t timeout_ms);
int recvFindAndFilter(const char * target, const char * begin, const char * end,
		char ** data, size_t timeout);

void esp8266_init(USART_TypeDef * port, size_t baudrate)
{
	m_port = port;
	m_baudrate = baudrate;
	/* Init rx buffer */
	m_uart.rx_buffer = (uint8_t*) rx_buffer;
	m_uart.rx_buffer_size = RSP8266_RX_BUFFER_SIZE;
	m_uart.rx_ptr_in = 0;
	m_uart.rx_ready = 0;
	m_uart.rx_ready_tmr = 0;

	/* init tx buffer */
	m_uart.tx_buffer = (uint8_t*) tx_buffer;
	m_uart.tx_buffer_size = RSP8266_TX_BUFFER_SIZE;
	m_uart.tx_ptr_in = m_uart.tx_ptr_out = 0;
	m_uart.tx_ready = 0;
	m_uart.tx_int_en = 0;
	m_uart.tx_length = 0;

	esp8266_config();
	TRACEL(TRACE_LEVEL_ESP,("ESP uart configured\n"));
}


void esp8266_cb_register(esp8266_rcv_callback cb)
{
	m_cb_func = cb;
}

int esp8266_get_IP(char ** ip)
{
	int ret = ESP8266_FAIL;
	char * str;

	eATCIFSR(&str);
	ret = recvFindAndFilter("OK", "+CIFSR:STAIP,\"", "\"\r\n+CIFSR:STAMAC", &str, 1000);
	if (ret == ESP8266_OK) {
		*ip = str;
	}
	else {
		*ip = NULL;
	}
	return ret;
}

int esp8266_get_AP_MAC(char ** mac_addr)
{
	int ret = ESP8266_FAIL;
	char * str;

	eATCIFSR(&str);
	ret = recvFindAndFilter("OK", "+CIFSR:STAMAC,\"", "\"", &str, 1000);
	if (ret == ESP8266_OK) {
		*mac_addr = str;
	}
	else {
		*mac_addr = NULL;
	}
	return ret;
}

int eAT(void)
{
	esp8266_uart_send("AT\r\n");
	return(recvFind("OK", 2000));
}

int eATE(uint8_t echo)
{
	char str[STR_MAX_LEN];
	snprintf(str, STR_MAX_LEN, "ATE%d\r\n", echo);
	esp8266_uart_send(str);
	return(recvFind("OK", 2000));
}

int eATGMR(char ** version)
{

	esp8266_uart_send("AT+GMR\r\n");
	return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", version, 1000);
}

int eATRST(void)
{
	esp8266_uart_send("AT+RST\r\n");
	return recvFind("OK", 1000);
}

int qATCWMODE(uint8_t * mode)
{
	int ret = ESP8266_FAIL;
	char * str_mode;

	if (!mode)
	{
		return ret;
	}
	esp8266_uart_send("AT+CWMODE?\r\n");
	ret = recvFindAndFilter("OK", "+CWMODE:", "\r\n\r\nOK", &str_mode, 1000);
	if (ret)
	{
		*mode = (uint8_t) strtoul(str_mode, NULL, 10);
	}
	return(ret);
}

int sATCWMODE(uint8_t mode)
{
	char str[STR_MAX_LEN];

	snprintf(str, STR_MAX_LEN, "AT+CWMODE=%d\r\n", mode);
	esp8266_uart_send(str);

	return recvString2("OK", "no change", 1000);
}

int sATCWJAP(char * ssid, char * pwd)
{
	esp8266_uart_send("AT+CWJAP=\"");
	esp8266_uart_send(ssid);
	esp8266_uart_send("\",\"");
	esp8266_uart_send(pwd);
	esp8266_uart_send("\"\r\n");

	return recvString("OK", 10000);
}

int eATCWLAP(char ** list)
{
	esp8266_uart_send("AT+CWLAP\r\n");
	return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", list, 10000);
}

int eATCWQAP(void)
{
	esp8266_uart_send("AT+CWQAP\r\n");
	return recvFind("OK", 500);
}

int eATPING(char * url)
{
	esp8266_uart_send("AT+PING=\"");
	esp8266_uart_send(url);
	esp8266_uart_send("\"\r\n");
	return recvString("OK", 5000);
}

int sATCWSAP(char * ssid, char * pwd, uint8_t chl, en_esp8266_ap_ecn ecn)
{
	char str[STR_MAX_LEN];

	esp8266_uart_send("AT+CWSAP=\"");
	esp8266_uart_send(ssid);
	esp8266_uart_send("\",\"");
	esp8266_uart_send(pwd);
	esp8266_uart_send("\",");
	snprintf(str, STR_MAX_LEN, "%d", chl);
	esp8266_uart_send(str);
	esp8266_uart_send(",");
	snprintf(str, STR_MAX_LEN, "%d\r\n", ecn);
	esp8266_uart_send(str);

	return recvString("OK", 5000);
}

int sATCWSAP_CUR(char * ssid, char * pwd, uint8_t chl, en_esp8266_ap_ecn ecn)
{
	char str[STR_MAX_LEN];

	esp8266_uart_send("AT+CWSAP_CUR=\"");
	esp8266_uart_send(ssid);
	esp8266_uart_send("\",\"");
	esp8266_uart_send(pwd);
	esp8266_uart_send("\",");
	snprintf(str, STR_MAX_LEN, "%d", chl);
	esp8266_uart_send(str);
	esp8266_uart_send(",");
	snprintf(str, STR_MAX_LEN, "%d\r\n", ecn);
	esp8266_uart_send(str);

	return recvString("OK", 5000);
}

int sATCWSAP_DEF(char * ssid, char * pwd, uint8_t chl, en_esp8266_ap_ecn ecn)
{
	char str[STR_MAX_LEN];

	esp8266_uart_send("AT+CWSAP_DEF=\"");
	esp8266_uart_send(ssid);
	esp8266_uart_send("\",\"");
	esp8266_uart_send(pwd);
	esp8266_uart_send("\",");
	snprintf(str, STR_MAX_LEN, "%d", chl);
	esp8266_uart_send(str);
	esp8266_uart_send(",");
	snprintf(str, STR_MAX_LEN, "%d\r\n", ecn);
	esp8266_uart_send(str);

	return recvString("OK", 5000);
}

int eATCWLIF(char ** list)
{
	esp8266_uart_send("AT+CWLIF\r\n");
	return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", list, 1000);
}

int eATCIPSTATUS(char ** list)
{
	esp8266_uart_send("AT+CIPSTATUS\r\n");
	return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", list, 1000);
}

int sATCIPSTARTSingle(char * type, char * remote_addr,
		uint16_t remote_port, uint16_t local_port)
{
	char str[STR_MAX_LEN];
	int ret = ESP8266_FAIL;


	esp8266_uart_send("AT+CIPSTART=\"");
	esp8266_uart_send(type);
	esp8266_uart_send("\",\"");
	esp8266_uart_send(remote_addr);
	esp8266_uart_send("\",");
	if (!strncmp(type, "UDP", 3))
	{
		snprintf(str, STR_MAX_LEN, "%d", remote_port);
		esp8266_uart_send(str);
		esp8266_uart_send(",");
		snprintf(str, STR_MAX_LEN, "%d", local_port);
		esp8266_uart_send(str);
		esp8266_uart_send(",2\r\n");
	}
	else
	{
		snprintf(str, STR_MAX_LEN, "%d", remote_port);
		esp8266_uart_send(str);
		esp8266_uart_send("\r\n");
	}

	ret = recvString3("OK", "ERROR", "ALREADY CONNECT", 500);
	if (ret == ESP8266_OK) {
		if (strstr((char*) m_uart.rx_buffer, "OK") || strstr((char*) m_uart.rx_buffer, "ALREADY CONNECT"))
		{
			ret = ESP8266_OK;
		}
		else {
			ret = ESP8266_FAIL;
		}
	}
	return ret;
}

int sATCIPSTARTMultiple(uint8_t mux_id, char * type, char * addr, uint16_t port)
{
	char str[STR_MAX_LEN];
	int ret = ESP8266_FAIL;

	esp8266_uart_send("AT+CIPSTART=");
	snprintf(str, STR_MAX_LEN, "%d", mux_id);
	esp8266_uart_send(str);

	esp8266_uart_send(",\"");
	esp8266_uart_send(type);
	esp8266_uart_send("\",\"");
	esp8266_uart_send(addr);
	esp8266_uart_send("\",");
	snprintf(str, STR_MAX_LEN, "%d\r\n", port);
	esp8266_uart_send(str);

	ret = recvString3("OK", "ERROR", "ALREADY CONNECT", 10000);
	if (ret == ESP8266_OK) {
		if (strstr((char*) m_uart.rx_buffer, "OK") || strstr((char*) m_uart.rx_buffer, "ALREADY CONNECT"))
		{
			ret = ESP8266_OK;
		}
		else {
			ret = ESP8266_FAIL;
		}
	}
	return ret;
}

int sATCIPSENDSingle(const uint8_t *buffer, size_t len)
{
	char str[STR_MAX_LEN];
	int ret = ESP8266_FAIL;

	esp8266_uart_send("AT+CIPSEND=");
	snprintf(str, STR_MAX_LEN, "%lu\r\n", len);
	esp8266_uart_send(str);

	if (recvFind(">", 5000))
	{
        size_t i = 0;
		for (; i < len; i++)
		{
			esp8266_uart_send_byte(buffer[i]);
		}
		ret = recvFind("SEND OK", 10000);
	}
	return ret;
}

int sATCIPSENDMultiple(uint8_t mux_id, const uint8_t *buffer, size_t len)
{
	char str[STR_MAX_LEN];
	int ret = ESP8266_FAIL;

	size_t bytes_to_send = 0;
	size_t remain_bytes = len;
	size_t max_bytes = 512;
	size_t index = 0;

	while (remain_bytes) {
		if (remain_bytes >= max_bytes) {
			bytes_to_send = max_bytes;
		}
		else {
			bytes_to_send = remain_bytes;
		}
		/* send packets with max length of 2048 */
		snprintf(str, STR_MAX_LEN, "AT+CIPSEND=%d,%lu\r\n", mux_id, bytes_to_send);
		TRACEL(TRACE_LEVEL_ESP,(str));
		esp8266_uart_send(str);
		if (recvFind(">", 5000))
		{
			TRACEL(TRACE_LEVEL_ESP,("sending %d bytes\n", bytes_to_send));
            size_t i=0;
			for (; i<bytes_to_send; i++)
			{
				esp8266_uart_send_byte(buffer[i+index]);
			}
			ret = recvFind("SEND OK", 5000);
			TRACEL(TRACE_LEVEL_ESP,("sent[%d]\n", ret));
		}

		remain_bytes -= bytes_to_send;
		index += bytes_to_send;
	}
	return ret;

	return ret;
}


int sATCIPCLOSEMulitple(uint8_t mux_id)
{
	char str[STR_MAX_LEN];
	int ret = ESP8266_FAIL;

	TRACEL(TRACE_LEVEL_ESP,("closing tcp: %d\n", mux_id));
	esp8266_uart_send("AT+CIPCLOSE=");
	snprintf(str, STR_MAX_LEN, "%d\r\n", mux_id);
	esp8266_uart_send(str);

	ret = recvString2("OK", "link is not", 5000);
	if (ret == ESP8266_OK) {
		if (strstr((char*) m_uart.rx_buffer, "OK") || strstr((char*) m_uart.rx_buffer, "link is not"))
		{
			ret = ESP8266_OK;
		}
		else {
			ret = ESP8266_FAIL;
		}
	}
	return ret;
}

int eATCIPCLOSESingle(void)
{
	esp8266_uart_send("AT+CIPCLOSE\r\n");
	return recvFind("OK", 5000);
}

int eATCIFSR(char ** list)
{
	esp8266_uart_send("AT+CIFSR\r\n");
	return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", list, 1000);
}

int sATCIPMUX(uint8_t mode)
{
	char str[STR_MAX_LEN];
	int ret = ESP8266_FAIL;

	snprintf(str, STR_MAX_LEN, "AT+CIPMUX=%d\r\n", mode);
	esp8266_uart_send(str);

	ret = recvString2("OK", "Link is builded", 1000);
	if (ret == ESP8266_OK) {
		if (strstr((char*) m_uart.rx_buffer, "OK"))
		{
			ret = ESP8266_OK;
		}
		else {
			ret = ESP8266_FAIL;
		}
	}
	return ret;
}

int sATCIPSERVER(uint8_t mode, uint16_t port)
{
	char str[STR_MAX_LEN];
	int ret = ESP8266_FAIL;

	if (mode)
	{
		snprintf(str, STR_MAX_LEN, "AT+CIPSERVER=1,%d\r\n", port);
		esp8266_uart_send(str);

		ret = recvString2("OK", "no change", 3000);
		if (ret == ESP8266_OK) {
			if (strstr((char*) m_uart.rx_buffer, "OK") || strstr((char*) m_uart.rx_buffer, "no change"))
			{
				ret = ESP8266_OK;
			}
			else {
				ret = ESP8266_FAIL;
			}
		}
	}
	else
	{
		esp8266_uart_send("AT+CIPSERVER=0\r\n");
		return recvFind("\r\r\n", 1000);
	}
	return ret;
}

int sATCIPSTO(size_t timeout)
{
	char str[STR_MAX_LEN];

	snprintf(str, STR_MAX_LEN, "AT+CIPSTO=%lu\r\n", timeout);
	esp8266_uart_send(str);

	return recvFind("OK", 1000);
}

int sATCIPSEND(const char* url)
{
	char str[STR_MAX_LEN];
	int ret = ESP8266_FAIL;

	snprintf(str, STR_MAX_LEN, "AT+CIPSEND=%d\r\n", strlen(url));
	esp8266_uart_send(str);

	if (recvFind(">", 1000))
	{
		esp8266_uart_send(url);

		ret = recvFind("SEND OK", 500);
	}

	return ret;
}



int recvString2(const char * target_str1, const char * target_str2, size_t timeout_ms)
{
	int resp = ESP8266_TIMEOUT;
	/* wait for reception */
	esp8266_msec_cntr = timeout_ms;
	while (esp8266_msec_cntr) {
		/* wait for incoming data */
		if (m_uart.rx_ready && (m_uart.rx_ready_tmr >= 10)) {
			/* terminate string */
			m_uart.rx_buffer[m_uart.rx_ptr_in] = 0;
			if (strstr((char*) m_uart.rx_buffer, target_str1) ||
					strstr((char*) m_uart.rx_buffer, target_str2)) {
				resp = ESP8266_OK;
				TRACEL(TRACE_LEVEL_ESP,("ESP(%d): %s\n", m_uart.rx_ptr_in, m_uart.rx_buffer));
				break;
			}
		}
	}
	return(resp);
}

int recvString3(const char * target_str1, const char * target_str2, const char * target_str3, size_t timeout_ms)
{
	int resp = ESP8266_TIMEOUT;
	/* wait for reception */
	esp8266_msec_cntr = timeout_ms;
	while (esp8266_msec_cntr) {
		/* wait for incoming data */
		if (m_uart.rx_ready && (m_uart.rx_ready_tmr >= 10)) {
			resp = ESP8266_OK;
			/* terminate string */
			m_uart.rx_buffer[m_uart.rx_ptr_in] = 0;
			if (strstr((char*) m_uart.rx_buffer, target_str1) ||
					strstr((char*) m_uart.rx_buffer, target_str2) ||
					strstr((char*) m_uart.rx_buffer, target_str3) ) {
				resp = ESP8266_OK;
				TRACEL(TRACE_LEVEL_ESP,("ESP(%d): %s\n", m_uart.rx_ptr_in, m_uart.rx_buffer));
				break;
			}
		}
	}
	return(resp);
}

int recvString(const char * target_str, size_t timeout_ms)
{
	int resp = ESP8266_TIMEOUT;
	/* wait for reception */
	esp8266_msec_cntr = timeout_ms;
	while (esp8266_msec_cntr) {
		/* wait for incoming data */
		if (m_uart.rx_ready && (m_uart.rx_ready_tmr >= 10)) {
			/* terminate string */
			m_uart.rx_buffer[m_uart.rx_ptr_in] = 0;
			if (strstr((char*) m_uart.rx_buffer, target_str)) {
				resp = ESP8266_OK;
				TRACEL(TRACE_LEVEL_ESP,("ESP(%d): %s\n", m_uart.rx_ptr_in, m_uart.rx_buffer));
				break;
			}
		}
	}
	return(resp);
}

int recvFind(const char * str, size_t timeout_ms)
{
	return(recvString(str, timeout_ms));
}

int recvFindAndFilter(const char * target, const char * begin, const char * end, char ** data, size_t timeout)
{
	*data = NULL;
	if (recvString(target, timeout))
	{
		char * index1 = strstr((char*)m_uart.rx_buffer, begin);
		char * index2 = strstr(index1, end);
		if ((index1 != NULL) && (index2 != NULL))
		{
			index1 += strlen(begin);
			*data = index1;
			*index2 = 0;
			return ESP8266_OK;
		}
	}
	return ESP8266_FAIL;
}

int esp8266_poll_recv()
{
	size_t data_len = 0;
	uint8_t mux_ch = 0;

	if (m_cb_func) {
		if (m_uart.rx_ready && (m_uart.rx_ready_tmr >= 100)) {int i;
			int index1 = 0;
			int found = 0;
			char tmp_str[20] = {0};

			/* find +IPD */
			for (i=0; i<strlen((char*)m_uart.rx_buffer); i++) {
				if (m_uart.rx_buffer[i] == '+') {
					if (!strncmp((char*)&m_uart.rx_buffer[i], "+IPD,", 5)) {
						index1 = i+5;
						found = 1;
						break;
					}
				}
			}
			if (found) {
				found = 0;
				TRACEL(TRACE_LEVEL_ESP,("found +IPD,\n"));
				for (i=index1; i<strlen((char*)m_uart.rx_buffer);i++) {
					if (m_uart.rx_buffer[i] == ',') {
						strncpy(tmp_str, (char*)&m_uart.rx_buffer[index1], i-index1);
						mux_ch = strtoul(tmp_str, NULL, 10);
						index1 = i+1;
						found = 1;
						break;
					}
				}
			}
			if (found) {
				found = 0;
				TRACEL(TRACE_LEVEL_ESP,("found mux: %d\n", mux_ch));
				for (i=index1; i<strlen((char*)m_uart.rx_buffer);i++) {
					if (m_uart.rx_buffer[i] == ':') {
						strncpy(tmp_str, (char*)&m_uart.rx_buffer[index1], i-index1);
						data_len = strtoul(tmp_str, NULL, 10);
						found = 1;
						index1 = i+1;
						break;
					}
				}
			}
			if (found) {
				found = 0;
				TRACEL(TRACE_LEVEL_ESP,("found length: %d\n", data_len));
				m_cb_func((uint8_t*)&m_uart.rx_buffer[index1], data_len, mux_ch);
			}
		}
	}
	return data_len;
}

void rx_empty(void)
{
	m_uart.rx_ptr_in = 0;
	m_uart.rx_ready = 0;
	m_uart.rx_ready_tmr = 0;
	memset(m_uart.rx_buffer, 0, m_uart.rx_buffer_size);
}

int esp8266_uart_receive(char ** buffer, size_t timeout_ms)
{
	int resp = ESP8266_TIMEOUT;
	/* wait for reception */
	esp8266_msec_cntr = timeout_ms;
//	TRACEL(TRACE_LEVEL_ESP,("esp8266_msec_cntr: %u\n", esp8266_msec_cntr));
	while (esp8266_msec_cntr) {
		/* wait for incoming data */
		if (m_uart.rx_ready && (m_uart.rx_ready_tmr >= 50)) {
			/* terminate string */
			m_uart.rx_buffer[m_uart.rx_ptr_in] = 0;
			*buffer = (char*) m_uart.rx_buffer;
			resp = m_uart.rx_ptr_in;
			break;
		}
	}
//	TRACEL(TRACE_LEVEL_ESP,("%d:%d:%u:%d\n", m_uart.rx_ready, m_uart.rx_ready_tmr,
//			esp8266_msec_cntr, timeout_ms));
	return(resp);
}


uint8_t esp8266_uart_send_byte(uint8_t byte)
{
//	USART_SendData(m_port, (uint8_t) byte);
	m_port->DR = (byte & (uint16_t)0x01FF);
	while (USART_GetFlagStatus(m_port, USART_FLAG_TC) == RESET);
//	TRACEL(TRACE_LEVEL_ESP,("%c", byte));
	return byte;
}

int esp8266_uart_send_len(const char * buffer, size_t buffer_len)
{
	size_t remaining_bytes = buffer_len;
	size_t bytes_to_sent = buffer_len;
	size_t i = 0;
	size_t index = 0;

	rx_empty();

	while (remaining_bytes) {
		/* wait for previous tx to end */
		while(m_uart.tx_length);
//		TRACEL(TRACE_LEVEL_ESP,("remain: %d\n", remaining_bytes));

		if (remaining_bytes >= m_uart.tx_buffer_size) {
			bytes_to_sent = m_uart.tx_buffer_size;
		}
		else {
			bytes_to_sent = remaining_bytes;
		}
		for (i=0; i<bytes_to_sent; i++) {
			esp8266_uart_send_byte(buffer[i+index]);
		}
		remaining_bytes -= bytes_to_sent;
		index += bytes_to_sent;
	}

	return buffer_len;
}

int esp8266_uart_send(const char * buffer)
{
	return(esp8266_uart_send_len(buffer, strlen(buffer)));
}

void esp8266_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	if (m_port == USART1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	}
	else if (m_port == USART2) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	}

	/* Configure USART Tx as alternate function push-pull */
	if (m_port == USART1) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	}
	else if (m_port == USART2) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	}
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as input floating */
	if (m_port == USART1) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	}
	else if (m_port == USART2) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	}
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = m_baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init(m_port, &USART_InitStructure);

	/*
	 Jump to the USART1_IRQHandler() function
	 if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(m_port, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 	// enable the USART1 receive interrupt

	if (m_port == USART1) {
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;// we want to configure the USART1 interrupts
	}
	else if (m_port == USART2) {
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;// we want to configure the USART1 interrupts
	}
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	// the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);	// the properties are passed to the NVIC_Init function which takes care of the low level stuff

	/* Enable the USART */
	USART_Cmd(m_port, ENABLE);
}

void esp8266_uart_irq(void)
{
	if (USART_GetITStatus(m_port, USART_IT_RXNE) != RESET) {
		/* Read one byte from the receive data register */
		if (m_uart.rx_ptr_in == m_uart.rx_buffer_size)
			m_port->DR;	//discard data
		m_uart.rx_buffer[m_uart.rx_ptr_in++] = m_port->DR;

		/* Disable the USARTy Receive interrupt */
		//USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		/* flag the byte reception */
		m_uart.rx_ready = 1;
		/* reset receive expire timer */
		m_uart.rx_ready_tmr = 0;
		m_port->SR &= ~USART_FLAG_RXNE;	          // clear interrupt
	}
}
