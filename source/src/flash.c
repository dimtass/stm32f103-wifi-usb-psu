/*
 * flash.c
 *
 *  Created on: 11 Apr 2017
 *      Author: dimtass
 */
#include <string.h>
#include "platform_config.h"
#include "flash.h"

#define MAX_BYTES_PER_WRITE 4

typedef enum {
	FAILED = 0,
	PASSED = !FAILED
} TestStatus;

#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
  #define FLASH_PAGE_SIZE    ((uint16_t)0x800)
#else
  #define FLASH_PAGE_SIZE    ((uint16_t)0x400)
#endif

uint32_t _flash_start_address = 0;
uint32_t _flash_end_address = 0;

#ifdef STM32F10X_XL
volatile TestStatus MemoryProgramStatus2 = PASSED;
#endif /* STM32F10X_XL */

void FLASH_Init(uint32_t flash_start_address, uint32_t blocks_to_use)
{
	FLASH_SetLatency(FLASH_Latency_2);
	_flash_start_address = flash_start_address;
	_flash_end_address = _flash_start_address + (blocks_to_use * FLASH_PAGE_SIZE);
	TRACEL(TRACE_LEVEL_FLASH,("FLASH END ADDR: 0x%04X%04X\n", (uint16_t)(_flash_end_address >> 16), (uint16_t)(0xFFFF & _flash_end_address)));
}

int FLASH_write(const uint8_t * data, uint16_t datalen)
{
	volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
	uint32_t NbrOfPage = 0x00;
	uint32_t EraseCounter = 0x00;
	uint32_t Address = 0x00;

	uint32_t tmp = 0;
	uint16_t i = 0;
	uint16_t remain_bytes = 0;
	uint16_t bytes_to_write = MAX_BYTES_PER_WRITE;
	int resp = 0;

	/* Unlock the Flash Bank1 Program Erase controller */
	FLASH_UnlockBank1();

	/* Define the number of page to be erased */
	NbrOfPage = (_flash_end_address - _flash_start_address) / FLASH_PAGE_SIZE;

	/* Clear All pending flags */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

	/* Erase the FLASH pages */
	for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
		FLASHStatus = FLASH_ErasePage(_flash_start_address + (FLASH_PAGE_SIZE * EraseCounter));
	}

	/* Program Flash Bank1 */
	Address = _flash_start_address;
	remain_bytes = datalen;
	i = 0;
	while (remain_bytes) {
		tmp = 0;
		if (remain_bytes >= MAX_BYTES_PER_WRITE) {
			bytes_to_write = MAX_BYTES_PER_WRITE;
			memcpy(&tmp, &data[MAX_BYTES_PER_WRITE*i], bytes_to_write);
		}
		else {
			bytes_to_write = remain_bytes;
			memcpy(&tmp, &data[MAX_BYTES_PER_WRITE*i], bytes_to_write);
			tmp <<= 8*(MAX_BYTES_PER_WRITE-remain_bytes);
		}
		FLASH_ProgramWord(Address, tmp);
		remain_bytes -= bytes_to_write;
		Address += MAX_BYTES_PER_WRITE;
		i++;
	}

	FLASH_LockBank1();

	if (FLASH_verify(data, datalen)) {
		resp = 1;
	}
	TRACEL(TRACE_LEVEL_FLASH,("FLASH saved: %d\n", resp));
	return resp;
}

void FLASH_read(uint8_t * data, uint16_t datalen)
{
	uint32_t tmp = 0;
	uint16_t i = 0;
	uint16_t remain_bytes = 0;
	uint16_t bytes_to_read = MAX_BYTES_PER_WRITE;
	uint32_t address = 0;

	/* Program Flash Bank1 */
	address = _flash_start_address;
	remain_bytes = datalen;
	i = 0;
	while (remain_bytes) {
		tmp = 0;
		if (remain_bytes >= MAX_BYTES_PER_WRITE) {
			bytes_to_read = MAX_BYTES_PER_WRITE;
			tmp = (*(__IO uint32_t*) address);
			memcpy(&data[MAX_BYTES_PER_WRITE*i], &tmp, bytes_to_read);
		}
		else {
			bytes_to_read = remain_bytes;
			tmp = (*(__IO uint32_t*) address);
			tmp >>= 8*(MAX_BYTES_PER_WRITE-remain_bytes);
			memcpy(&data[MAX_BYTES_PER_WRITE*i], &tmp, bytes_to_read);
		}
		remain_bytes -= bytes_to_read;
		address += MAX_BYTES_PER_WRITE;
		i++;
	}
}

int FLASH_verify(const uint8_t * data, uint16_t datalen)
{
	uint32_t tmp = 0;
	uint16_t i = 0;
	uint16_t remain_bytes = 0;
	uint16_t bytes_to_read = MAX_BYTES_PER_WRITE;
	uint32_t address = 0;
	int ret = 0;

	/* Program Flash Bank1 */
	address = _flash_start_address;
	remain_bytes = datalen;
	i = 0;
	while (remain_bytes) {
		tmp = 0;
		if (remain_bytes >= MAX_BYTES_PER_WRITE) {
			bytes_to_read = MAX_BYTES_PER_WRITE;
			memcpy(&tmp, &data[MAX_BYTES_PER_WRITE*i], bytes_to_read);
		}
		else {
			bytes_to_read = remain_bytes;
			memcpy(&tmp, &data[MAX_BYTES_PER_WRITE*i], bytes_to_read);
			tmp <<= 8*(MAX_BYTES_PER_WRITE-remain_bytes);
		}
		if((*(__IO uint32_t*) address) != tmp) {
			goto _exit;
		}
		remain_bytes -= bytes_to_read;
		address += MAX_BYTES_PER_WRITE;
		i++;
	}
	ret = 1;

_exit:
	TRACEL(TRACE_LEVEL_FLASH,("FLASH verify: %d\n", ret));
	return ret;
}
