/*
 * spi_driver.c
 *
 *  Created on: 29 Nov 2016
 *      Author: dimtass
 */
#include "spi_driver.h"

/* define the NSS high/low */
#define NSS_L		SPI_PORT->ODR &= ~GPIO_PIN_SPI_NSS
#define NSS_H		SPI_PORT->ODR |= GPIO_PIN_SPI_NSS

#define DMA_RX_CH	DMA1_Channel2
#define DMA_TX_CH	DMA1_Channel3

DMA_InitTypeDef  DMA_Initstruct;

void spi_driver_init(void) {

	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Set SPI GPIOs for Master */
	GPIO_InitStructure.GPIO_Pin = SPI_PIN_SCK | SPI_PIN_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_PIN_NSS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SPI_PORT, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = SPI_PIN_MISO;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(SPI_PORT, &GPIO_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 10;
	SPI_CalculateCRC(SPI1, DISABLE);
	SPI_Init(SPI_IF, &SPI_InitStructure);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* Configure DMA for TX/RX */
	DMA_DeInit(DMA_RX_CH);
	DMA_DeInit(DMA_TX_CH);
	DMA_Initstruct.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI1->DR));
	DMA_Initstruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_Initstruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_Initstruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_Initstruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_Initstruct.DMA_Priority = DMA_Priority_Medium;
	DMA_Initstruct.DMA_M2M = DMA_M2M_Disable;
	SPI_IF->DR;
	SPI_IF->CR1 |= ((uint16_t)0x0040);
}

void spi_driver_start(void)
{
	NSS_L;
}

void spi_driver_stop(void)
{
	NSS_H;
}

uint16_t spi_driver_tx(const uint8_t * buff, uint16_t buff_len)
{
	__IO uint16_t TxIdx = 0;

	/* Enable SPI */
	SPI1->CR1 |= ((uint16_t)0x0040);
	/* Transfer procedure */
	  while (TxIdx < buff_len)
	  {
	    /* Wait for SPIy Tx buffer empty */
	    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	    /* Send SPIz data */
	    SPI_I2S_SendData(SPI1, buff[TxIdx++]);
	    /* Send SPIy data */
	    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	    /* Read SPIy received data */
	    SPI_I2S_ReceiveData(SPI1);

	  }
	  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	return(buff_len);
}

uint16_t spi_driver_rx(uint8_t * buff, uint16_t buff_len)
{
	uint8_t dummy = 0xff;
	int i;

	for (i=0; i<buff_len; i++) {
		/*!< Loop while DR register in not emplty */
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		/*!< Send byte through the SPI1 peripheral */
		SPI1->DR = dummy;
		/*!< Wait to receive a byte */
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		/*!< Return the byte read from the SPI bus */
		buff[i] = SPI1->DR;
	}
	return(buff_len);
}

uint16_t spi_driver_tx_dma(const uint8_t * buff, uint16_t buff_len)
{
	/* Configure Tx DMA */
	DMA_Initstruct.DMA_MemoryBaseAddr = (uint32_t)(buff);
	DMA_Initstruct.DMA_BufferSize = buff_len;
	DMA_Initstruct.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_Initstruct.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(DMA_TX_CH, &DMA_Initstruct);

	/* Enable the DMA channel */
	DMA_TX_CH->CCR |= DMA_CCR1_EN;
	/* Enable the SPI Rx/Tx DMA request */
	SPI1->CR2 |= SPI_I2S_DMAReq_Tx;

	/* Enable SPI */
	SPI1->CR1 |= ((uint16_t)0x0040);

	/* Wait write to finish */
	while (DMA_GetFlagStatus(DMA1_FLAG_TC3)==RESET);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == RESET);

	/* Clear interrupt flag */
    DMA1->IFCR = DMA1_FLAG_TC3;

    /* Disable DMA */
	DMA_TX_CH->CCR &= (uint16_t)(~DMA_CCR1_EN);

	/* Disable SPI Tx DMA request */
	SPI1->CR2 &= (uint16_t)~(SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx);

	/* Empty the I2C register */
	SPI1->DR;

	return(buff_len);
}


uint16_t spi_driver_txrx_dma(uint8_t * buff, uint16_t buff_len, uint8_t * in, uint16_t in_len)
{
	uint16_t ret = 0;

	/* Configure Tx DMA */
	DMA_Initstruct.DMA_MemoryBaseAddr = (uint32_t)(buff);
	DMA_Initstruct.DMA_BufferSize = buff_len;
	DMA_Initstruct.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_Initstruct.DMA_Mode = DMA_Mode_Circular;
	DMA_Init(DMA_TX_CH, &DMA_Initstruct);

	/* Configure Rx DMA */
	DMA_Initstruct.DMA_MemoryBaseAddr = (uint32_t)(in);
	DMA_Initstruct.DMA_BufferSize = in_len;
	DMA_Initstruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_Initstruct.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(DMA_RX_CH, &DMA_Initstruct);

	/* Enable the DMA channel */
//	DMA_Cmd(DMA_RX_CH, ENABLE);
//	DMA_Cmd(DMA_TX_CH, ENABLE);
	DMA_RX_CH->CCR |= DMA_CCR1_EN;
	DMA_TX_CH->CCR |= DMA_CCR1_EN;

	/* Enable the SPI Rx/Tx DMA request */
//	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);
	SPI1->CR2 |= SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx;

//	SPI_Cmd(SPI1, ENABLE);
	SPI1->CR1 |= ((uint16_t)0x0040);

	while (DMA_GetFlagStatus(DMA1_FLAG_TC3)==RESET);
	while (DMA_GetFlagStatus(DMA1_FLAG_TC2)==RESET);

//	DMA_ClearFlag(DMA1_FLAG_TC3);
//	DMA_ClearFlag(DMA1_FLAG_TC2);
    DMA1->IFCR = DMA1_FLAG_TC3 | DMA1_FLAG_TC2;

//	DMA_Cmd(DMA_RX_CH, DISABLE);
//	DMA_Cmd(DMA_TX_CH, DISABLE);
	DMA_RX_CH->CCR &= (uint16_t)(~DMA_CCR1_EN);
	DMA_TX_CH->CCR &= (uint16_t)(~DMA_CCR1_EN);

//	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
	SPI1->CR2 &= (uint16_t)~(SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx);

//	SPI_Cmd(SPI1, DISABLE);
//	SPI1->CR1 &= ((uint16_t)0xFFBF);

	/* Empty the I2C register */
	SPI1->DR;

	return (ret);
}

uint16_t spi_driver_rx_dma(uint8_t * in, uint16_t in_len)
{
	uint16_t ret = 0;

	/* Configure Tx DMA */
	DMA_Initstruct.DMA_MemoryBaseAddr = (uint32_t)NULL;
	DMA_Initstruct.DMA_BufferSize = 1;
	DMA_Initstruct.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_Initstruct.DMA_Mode = DMA_Mode_Circular;
	DMA_Init(DMA_TX_CH, &DMA_Initstruct);

	/* Configure Rx DMA */
	DMA_Initstruct.DMA_MemoryBaseAddr = (uint32_t)(in);
	DMA_Initstruct.DMA_BufferSize = in_len;
	DMA_Initstruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_Initstruct.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(DMA_RX_CH, &DMA_Initstruct);

	/* Enable the DMA channel */
//	DMA_Cmd(DMA_RX_CH, ENABLE);
//	DMA_Cmd(DMA_TX_CH, ENABLE);
	DMA_RX_CH->CCR |= DMA_CCR1_EN;
	DMA_TX_CH->CCR |= DMA_CCR1_EN;

	/* Enable the SPI Rx/Tx DMA request */
//	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);
	SPI1->CR2 |= SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx;

//	SPI_Cmd(SPI1, ENABLE);
	SPI1->CR1 |= ((uint16_t)0x0040);

	while (DMA_GetFlagStatus(DMA1_FLAG_TC3)==RESET);
	while (DMA_GetFlagStatus(DMA1_FLAG_TC2)==RESET);

//	DMA_ClearFlag(DMA1_FLAG_TC3);
//	DMA_ClearFlag(DMA1_FLAG_TC2);
    DMA1->IFCR = DMA1_FLAG_TC3 | DMA1_FLAG_TC2;

//	DMA_Cmd(DMA_RX_CH, DISABLE);
//	DMA_Cmd(DMA_TX_CH, DISABLE);
	DMA_RX_CH->CCR &= (uint16_t)(~DMA_CCR1_EN);
	DMA_TX_CH->CCR &= (uint16_t)(~DMA_CCR1_EN);

//	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
	SPI1->CR2 &= (uint16_t)~(SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx);

//	SPI_Cmd(SPI1, DISABLE);
//	SPI1->CR1 &= ((uint16_t)0xFFBF);

	/* Empty the I2C register */
	SPI1->DR;

	return (ret);
}
