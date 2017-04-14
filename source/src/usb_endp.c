/**
  ******************************************************************************
  * @file    usb_endp.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Endpoint routines
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
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "platform_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL             5
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/

void EP1_IN_Callback(void)
{

	if (glb.usb_tx_buff_p.ptr_out != glb.usb_tx_buff_p.ptr_in) {
		uint16_t remain_len = glb.usb_tx_buff_p.ptr_in - glb.usb_tx_buff_p.ptr_out;
		uint8_t send_len = 0;

		if (remain_len < VIRTUAL_COM_PORT_DATA_SIZE)
			send_len = remain_len;
		else
			send_len = VIRTUAL_COM_PORT_DATA_SIZE;

		/* send  packet to PMA*/
		UserToPMABufferCopy((unsigned char*) &glb.usb_tx_buff[glb.usb_tx_buff_p.ptr_out], ENDP1_TXADDR,
				send_len);
		SetEPTxCount(ENDP1, send_len);
		SetEPTxValid(ENDP1);
		glb.usb_tx_buff_p.ptr_out += send_len;
	}
	else {
		glb.usb_tx_ready = 1;
		glb.usb_tx_buff_p.ptr_out = 0;
		glb.usb_tx_buff_p.ptr_in = 0;
		glb.usb_tx_buff_p.length = 0;
	}
}

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
	uint16_t rx_count = GetEPRxCount(ENDP3);
	TRACEL(TRACE_LEVEL_USB, ("usb_in:%d\n", rx_count));
	PMAToUserBufferCopy((unsigned char*) &glb.usb_rx_buff[glb.usb_rx_buff_p.ptr_in],
			ENDP3_RXADDR, rx_count);
	glb.usb_rx_buff_p.ptr_in += rx_count;
	glb.usb_rx_buff_p.length = glb.usb_rx_buff_p.ptr_in - glb.usb_rx_buff_p.ptr_out;
	glb.usb_rx_ready = 1;
	glb.usb_rx_ready_tmr = 0;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
