/*
 * mcp41xxx.c
 *
 *  Created on: 1 Apr 2017
 *      Author: dimtass
 */
#include "mcp41xxx.h"
#include "spi_driver.h"

typedef enum {
	CMD_NONE = 0b00,
	CMD_WRITE = 0b01,
	CMD_SHUT_DOWN = 0b10,
} en_command;


void mcp41xxx_init(uint8_t pot_value)
{
	spi_driver_init();
	mcp41xxx_set_value(1, pot_value);
//	spi_driver_start();
//	spi_driver_tx();
//	spi_driver_rx();
//	spi_driver_stop();
}

void mcp41xxx_set_value(uint8_t pot_id, uint8_t pot_value)
{
	uint8_t data[2] = {0};

	/* write command & pot id */
	data[0] = (CMD_WRITE << 4) | pot_id;

	/* insert pot value */
	data[1] = pot_value;

	spi_driver_start();
	spi_driver_tx_dma(data, 2);
	spi_driver_stop();
}

void mcp41xxx_shut_down(uint8_t pot_id)
{
	uint8_t data[2] = {0};

	/* write command & pot id */
	data[0] = (CMD_SHUT_DOWN << 4) | pot_id;

	/* insert pot value */
	data[1] = 0;

	spi_driver_start();
	spi_driver_tx_dma(data, 2);
	spi_driver_stop();
}

