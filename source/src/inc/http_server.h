/*
 * http_server.h
 *
 *	This a very simplistic HTTP server used to handle the HTTP request
 *	from an ESP8266 module.
 *
 *  Created on: 6 Apr 2017
 *      Author: dimtass
 */

#ifndef HTTP_SERVER_H_
#define HTTP_SERVER_H_

#include <stdint.h>
#include <string.h>

/**
 * Define the ESP8266 module address in here
 */
#define HTTP_IP_ADDRESS	"192.168.0.80"

/**
 * @brief Handles the incoming HTTP requests
 * @param[in] mux_ch The ESP8266 channel
 * @param[in] req The http req data
 * @param[in] req_size The http req data length
 */
void http_handler(uint8_t mux_ch, const char * req, size_t req_size);

#endif /* HTTP_SERVER_H_ */
