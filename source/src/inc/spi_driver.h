/*
 * spi_driver.h
 *
 *  Created on: 29 Nov 2016
 *      Author: dimtass
 */

#ifndef SPI_DRIVER_H_
#define SPI_DRIVER_H_

#include "stm32f10x.h"
#include "platform_config.h"

#define SPI_IF	SPI1
#define SPI_PORT		GPIO_PORT_SPI		// defined in platform_config.h
#define SPI_PIN_NSS		GPIO_PIN_SPI_NSS	// defined in platform_config.h
#define SPI_PIN_SCK		GPIO_PIN_SPI_SCK	// defined in platform_config.h
//#define SPI_PIN_MISO	GPIO_PIN_SPI_MISO	// defined in platform_config.h
#define SPI_PIN_MOSI	GPIO_PIN_SPI_MOSI	// defined in platform_config.h


/**
 * @brief Initialise the SPI interface
 */
void spi_driver_init(void);

/**
 * @brief Start the SPI interface
 */
void spi_driver_start(void);

/**
 * @brief Stop the SPI interface
 */
void spi_driver_stop(void);

/**
 * @brief Send a buffer to the SPI bus (blocking)
 * @param[in] buff The data buffer to send
 * @param[in] buff_len The length of the data buffer
 * @return uint16_t The number of bytes sent
 */
uint16_t spi_driver_tx(const uint8_t * buff, uint16_t buff_len);

/**
 * @brief Receive data from SPI (blocking)
 * @param[out] in A buffer to store the data
 * @param[in] on_len The buffer length
 * @return uint16_t The number of bytes received
 */
uint16_t spi_driver_rx(uint8_t * in, uint16_t in_len);

/**
 * @brief Send and then receive data from the SPI bus (DMA)
 * @param[in] buff The data buffer to send
 * @param[in] buff_len The length of the data buffer
 * @param[out] in A buffer to store the data
 * @param[in] on_len The buffer length
 * @return uint16_t The number of bytes received
 */
uint16_t spi_driver_txrx_dma(uint8_t * out, uint16_t out_len, uint8_t * in, uint16_t in_len);

/**
 * @brief Send a buffer to the SPI bus (DMA)
 * @param[in] buff The data buffer to send
 * @param[in] buff_len The length of the data buffer
 * @return uint16_t The number of bytes sent
 */
uint16_t spi_driver_tx_dma(const uint8_t * buff, uint16_t buff_len);

/**
 * @brief Receive data from SPI (DMA)
 * @param[out] in A buffer to store the data
 * @param[in] on_len The buffer length
 * @return uint16_t The number of bytes received
 */
uint16_t spi_driver_rx_dma(uint8_t * in, uint16_t in_len);

#endif /* SPI_DRIVER_H_ */
