/*
 * mcp41xxx.h
 *
 * Handles both MCP41XXX and MCP42XXX SPI pots
 *
 *  Created on: 1 Apr 2017
 *      Author: dimtass
 */

#ifndef MCP41XXX_H_
#define MCP41XXX_H_

#include "stm32f10x.h"

void mcp41xxx_init(uint8_t pot_value);

/**
 * @brief Sets the pot value
 * @param[in] pot_id The pot id (1: for MCP41XXX, 1 or 2: for MCP42XXX
 * @param[in] pot_value The pot value from 0-255. If 0, then R[PA0:PW0]=10K & R[PB0:PW0]=0K. If 255, then R[PA0:PW0]=0K & R[PB0:PW0]=10K.
 */
void mcp41xxx_set_value(uint8_t pot_id, uint8_t pot_value);

/**
 * @brief Shuts down the POT
 * @param[in] pot_id The pot id (1: for MCP41XXX, 1 or 2: for MCP42XXX
 */
void mcp41xxx_shut_down(uint8_t pot_id);

#endif /* MCP41XXX_H_ */
