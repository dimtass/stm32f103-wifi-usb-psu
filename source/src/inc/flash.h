/*
 * flash.h
 *
 *  Created on: 11 Apr 2017
 *      Author: dimtass
 */

#ifndef FLASH_H_
#define FLASH_H_
#include <stdint.h>
#include "stm32f10x.h"

/**
 * @brief Initialize STM32 FLASH
 */
void FLASH_Init(uint32_t flash_start_address, uint32_t blocks_to_use);

/**
 * @brief Write a data buffer to Flash and also verifies the write
 * @param[in] data The data buffer to store in flash
 * @param[in] datalen The length of the data buffer
 * @return int 0: failed to write the data, 1: success w/ verification
 */
int FLASH_write(const uint8_t * data, uint16_t datalen);

/**
 * @brief Read from Flash in a buffer
 * @param[out] data The buffer to store the data from flash
 * @param[in] datalen The buffer length
 */
void FLASH_read(uint8_t * data, uint16_t datalen);

/**
 * @brief Verify that a buffer is written in the flash
 * @param[in] data The buffer to verify
 * @parma[in] datalen The length of the data buffer
 * @return int 0: verification failed, 1: verification succeded
 */
int FLASH_verify(const uint8_t * data, uint16_t datalen);

#endif /* FLASH_H_ */
