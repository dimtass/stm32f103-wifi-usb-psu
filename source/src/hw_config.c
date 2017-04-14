/**
 ******************************************************************************
 * @file    hw_config.c
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   Hardware Configuration & Setup
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "platform_config.h"
#include "stm32f10x_it.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"

#define         ID1          (0x1FFFF7E8)
#define         ID2          (0x1FFFF7EC)
#define         ID3          (0x1FFFF7F0)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;
EXTI_InitTypeDef EXTI_InitStructure;

static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len);
/* Extern variables ----------------------------------------------------------*/

extern LINE_CODING linecoding;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
 * Function Name  : Enter_LowPowerMode
 * Description    : Power-off system clocks and power while entering suspend mode
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
void Enter_LowPowerMode(void)
{
	/* Set the device state to suspend */
	bDeviceState = SUSPENDED;
}

/*******************************************************************************
 * Function Name  : Leave_LowPowerMode
 * Description    : Restores system clocks and power while exiting suspend mode
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
void Leave_LowPowerMode(void)
{
	DEVICE_INFO *pInfo = &Device_Info;

	/* Set the device state to the correct state */
	if (pInfo->Current_Configuration != 0) {
		/* Device configured */
		bDeviceState = CONFIGURED;
	} else {
		bDeviceState = ATTACHED;
	}
	/*Enable SystemCoreClock*/
	SystemInit();
}

/*******************************************************************************
 * Function Name  : Get_SerialNum.
 * Description    : Create the serial number string descriptor.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Get_SerialNum(void)
{
	uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

	Device_Serial0 = *(uint32_t*) ID1;
	Device_Serial1 = *(uint32_t*) ID2;
	Device_Serial2 = *(uint32_t*) ID3;

	Device_Serial0 += Device_Serial2;

	if (Device_Serial0 != 0) {
		IntToUnicode(Device_Serial0, &Virtual_Com_Port_StringSerial[2], 8);
		IntToUnicode(Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
	}
}

/*******************************************************************************
 * Function Name  : HexToChar.
 * Description    : Convert Hex 32Bits value into char.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len)
{
	uint8_t idx = 0;

	for (idx = 0; idx < len; idx++) {
		if (((value >> 28)) < 0xA) {
			pbuf[2 * idx] = (value >> 28) + '0';
		} else {
			pbuf[2 * idx] = (value >> 28) + 'A' - 10;
		}

		value = value << 4;

		pbuf[2 * idx + 1] = 0;
	}
}

/*******************************************************************************
 * Function Name  : Send DATA .
 * Description    : send the data received from the STM32 to the PC through USB
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
uint32_t CDC_Send_DATA(const uint8_t *ptrBuffer, uint16_t bufflen)
{
	uint8_t send_len = 0;

	if ((glb.usb_tx_buff_p.ptr_in + bufflen)%USB_TX_BUFF_SIZE == glb.usb_tx_buff_p.ptr_out) {
		return 0;
	}

	glb.usb_tx_buff_p.length += bufflen;
	int i = 0;
	for (i=0; i<bufflen; i++) {
		glb.usb_tx_buff[glb.usb_tx_buff_p.ptr_in] = ptrBuffer[i];
		glb.usb_tx_buff_p.ptr_in = (glb.usb_tx_buff_p.ptr_in + 1)%USB_TX_BUFF_SIZE;
	}
	/* Check if usb buffer is empty */
	if (glb.usb_tx_ready) {
		if (bufflen < VIRTUAL_COM_PORT_DATA_SIZE)
			send_len = bufflen;
		else
			send_len = VIRTUAL_COM_PORT_DATA_SIZE;
//		TRACE(("send_len:%d\n", send_len));
		/* send  packet to PMA*/
		UserToPMABufferCopy((unsigned char*) &glb.usb_tx_buff[glb.usb_tx_buff_p.ptr_out],
				ENDP1_TXADDR, send_len);
		SetEPTxCount(ENDP1, send_len);
		SetEPTxValid(ENDP1);
		glb.usb_tx_buff_p.ptr_out += send_len;
		/*Sent flag*/
		glb.usb_tx_ready = 0;
	}
	else {
		/* Do nothing? */
	}

	return 1;
}

/*******************************************************************************
 * Function Name  : Receive DATA .
 * Description    : receive the data from the PC to STM32 and send it through USB
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
uint32_t CDC_Receive_DATA(void)
{
	/*Receive flag*/
//  glb.usb_rx_ready = 0;
	SetEPRxValid(ENDP3);
	return 1;
}

void USB_Reset(uint16_t delay)
{
//	GPIOA->ODR &= ~USB_RESET_Pin;
//	while(delay--);
//	GPIOA->ODR |= USB_RESET_Pin;
}

void SPI_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Set SPI GPIOs for Master */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 10;
	SPI_CalculateCRC(SPI1, DISABLE);
	SPI_Init(SPI1, &SPI_InitStructure);
}

void USB_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	/* Select USBCLK source */
	RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);

	/* Enable the USB clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the USB Wake-up interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	/* Configure the EXTI line 18 connected internally to the USB IP */
	EXTI_ClearITPendingBit(EXTI_Line18);
	EXTI_InitStructure.EXTI_Line = EXTI_Line18;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

void UART2_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init(USART2, &USART_InitStructure);

	/*
	 Jump to the USART1_IRQHandler() function
	 if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 	// enable the USART1 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;// we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	// the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);	// the properties are passed to the NVIC_Init function which takes care of the low level stuff

	/* Enable the USART1 */
	USART_Cmd(USART1, ENABLE);
}

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;


	/* Enable relay port */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* Configure Relay */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_RELAY;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIO_PORT_RELAY, &GPIO_InitStructure);
	GPIO_PORT_RELAY->ODR &= ~GPIO_PIN_RELAY;

}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
