/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN
STM32 Port: CHEN BING

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

//#define DEBUG_WAR_CONSOLE_ON
//#define DEBUG_ERR_CONSOLE_ON
#include "stm32f2xx.h"
#include "stm32f2xx_can.h"

#include "canfestival.h"

//  #define CANx                       CAN1
  #define CAN1_CLK                    RCC_APB1Periph_CAN1
  #define CAN1_RX_PIN                 GPIO_Pin_8
  #define CAN1_TX_PIN                 GPIO_Pin_9
  #define CAN1_GPIO_PORT              GPIOB
  #define CAN1_GPIO_CLK               RCC_AHB1Periph_GPIOB
  #define CAN1_AF_PORT                GPIO_AF_CAN1
  #define CAN1_RX_SOURCE              GPIO_PinSource8
  #define CAN1_TX_SOURCE              GPIO_PinSource9       

void can_irq_handler(void);

CAN_InitTypeDef        CAN_InitStructure;
CAN_FilterInitTypeDef  CAN_FilterInitStructure;


unsigned char canInit(unsigned int bitrate)
/******************************************************************************
Initialize the hardware to receive CAN messages and start the timer for the
CANopen stack.
INPUT	
OUTPUT	
******************************************************************************/
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* CAN GPIOs configuration **************************************************/

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(CAN1_GPIO_CLK, ENABLE);

  /* Connect CAN pins to AF9 */
  GPIO_PinAFConfig(CAN1_GPIO_PORT, CAN1_RX_SOURCE, CAN1_AF_PORT);
  GPIO_PinAFConfig(CAN1_GPIO_PORT, CAN1_TX_SOURCE, CAN1_AF_PORT); 
  
  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN | CAN1_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);

  /* CAN configuration ********************************************************/  
  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE);
  
  /* CAN register init */
  CAN_DeInit(CAN1);

  CAN_StructInit(&CAN_InitStructure);
  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;//125Kbps
  CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  CAN_InitStructure.CAN_Prescaler =16; 


  /* CAN Baudrate = 1MBps (CAN clocked at 30 MHz) */
  //CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  //CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  //CAN_InitStructure.CAN_Prescaler = 2;
  /* CAN Baudrate = 500KBps (CAN clocked at 30 MHz) */
  //CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  //CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  //CAN_InitStructure.CAN_Prescaler = 4;
  /* CAN Baudrate = 250KBps (CAN clocked at 30 MHz) */
  //CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  //CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  //CAN_InitStructure.CAN_Prescaler = 8;
  /* CAN Baudrate = 125KBps (CAN clocked at 30 MHz) */
//  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
//  CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
//  CAN_InitStructure.CAN_Prescaler = 16;
//  
	
	CAN_Init(CAN1, &CAN_InitStructure);
	
	CAN_FilterInitStructure.CAN_FilterNumber = 1;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x6420;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
  /* CAN filter init */


/*
  CAN_FilterInitStructure.CAN_FilterNumber = 0;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
*/  
  
  /* Enable FIFO 0 message pending Interrupt */
//  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
  return 1;
}

//unsigned char canInit(unsigned int bitrate)
/******************************************************************************
Initialize the hardware to receive CAN messages and start the timer for the
CANopen stack.
INPUT	
OUTPUT	
//******************************************************************************/
//{
//  CAN_InitTypeDef        CAN_InitStructure;
//  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
// /* CANs configuration */
//  GPIO_InitTypeDef  GPIO_InitStructure;

//  /* Configure CAN1 and CAN2 IOs **********************************************/
//  /* GPIOB, GPIOD and AFIO clocks enable */
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOB, ENABLE);
//     
//  /* Configure CAN1 RX pin */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//  GPIO_Init(GPIOD, &GPIO_InitStructure);
//   
//  /* Configure CAN2 RX pin */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);

//  /* Configure CAN1 TX pin */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOD, &GPIO_InitStructure);

//  /* Configure CAN2 TX pin */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);

//  /* Remap CAN1 and CAN2 GPIOs */
//  GPIO_PinRemapConfig(GPIO_Remap2_CAN1 , ENABLE);
//  //GPIO_PinRemapConfig(GPIO_Remap_CAN2, ENABLE);

//  /* Configure CAN1 and CAN2 **************************************************/  
//  /* CAN1 and CAN2 Periph clocks enable */
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);  
//  
//   /* CAN1 and CAN2 register init */
//  CAN_DeInit(CAN1);
//  CAN_DeInit(CAN2);

//  /* Struct init*/
//  CAN_StructInit(&CAN_InitStructure);

//  /* CAN1 and CAN2  cell init */
//  CAN_InitStructure.CAN_TTCM = DISABLE;
//  CAN_InitStructure.CAN_ABOM = DISABLE;
//  CAN_InitStructure.CAN_AWUM = DISABLE;
//  CAN_InitStructure.CAN_NART = DISABLE;
//  CAN_InitStructure.CAN_RFLM = DISABLE;
//  CAN_InitStructure.CAN_TXFP = ENABLE;
//  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
//  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
//  CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;//125Kbps
//  CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
//  CAN_InitStructure.CAN_Prescaler =18;
//  
//  /*Initializes the CAN1  and CAN2 */
//  CAN_Init(CAN1, &CAN_InitStructure);
//  CAN_Init(CAN2, &CAN_InitStructure);

//  /* CAN1 filter init */
//  CAN_FilterInitStructure.CAN_FilterNumber = 1;
//  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
//  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
//  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x6420;
//  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
//  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
//  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
//  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
//  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
//  CAN_FilterInit(&CAN_FilterInitStructure);
//  
//  /* CAN2 filter init */
//  CAN_FilterInitStructure.CAN_FilterIdHigh =0x2460;
//  CAN_FilterInitStructure.CAN_FilterNumber = 15;
//  CAN_FilterInit(&CAN_FilterInitStructure);
//  
//  

//  /* IT Configuration for CAN1 */  
//  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

//  /* IT Configuration for CAN2 */  
//  CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);

//  return 1;
//}


unsigned char canSend(UNS8 canHandle, Message *m)
/******************************************************************************
The driver send a CAN message passed from the CANopen stack
INPUT	CAN_PORT is not used (only 1 avaiable)
	Message *m pointer to message to send
OUTPUT	1 if  hardware -> CAN frame
******************************************************************************/
{

  CanTxMsg TxMessage;
  u8 i=0;
  u8 TransmitMailbox; 
  /* transmit */
  TxMessage.StdId=m->cob_id;
  //TxMessage.ExtId = 0x01;
  TxMessage.RTR=m->rtr;
  TxMessage.IDE=CAN_ID_STD;
  TxMessage.DLC=m->len;
  for(i=0;i<m->len;i++)
  {
    TxMessage.Data[i]=m->data[i];
  }
  if(canHandle==1)
    CAN_Transmit(CAN1, &TxMessage);
  else if(canHandle==2)
    CAN_Transmit(CAN2, &TxMessage);
  return 0;	// successful

}

unsigned char canReceive_1(Message *m)
/******************************************************************************
The driver passes a received CAN message to the stack
INPUT	Message *m pointer to received CAN message
OUTPUT	1 if a message received
******************************************************************************/
{
  CanRxMsg RxMessage;
  u8 i=0;  
  
/* receive */
  //¼Ó×´Ì¬ÅÐ¶Ï
  RxMessage.StdId=0x00;
  RxMessage.IDE=CAN_ID_STD;
  RxMessage.DLC=0;
  
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
  
  m->cob_id=RxMessage.StdId;
  m->len=RxMessage.DLC; 
  m->rtr=RxMessage.RTR;
  
  for(i=0;i<m->len;i++)
  {
    m->data[i]=RxMessage.Data[i];
  }
  return 1;		// message received
}

unsigned char canReceive_2(Message *m)
/******************************************************************************
The driver passes a received CAN message to the stack
INPUT	Message *m pointer to received CAN message
OUTPUT	1 if a message received
******************************************************************************/
{
  CanRxMsg RxMessage;
  u8 i=0;  
  
/* receive */
  //¼Ó×´Ì¬ÅÐ¶Ï
  RxMessage.StdId=0x00;
  RxMessage.IDE=CAN_ID_STD;
  RxMessage.DLC=0;
  
  CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);
  
  m->cob_id=RxMessage.StdId;
  m->len=RxMessage.DLC; 
  m->rtr=RxMessage.RTR;
  
  for(i=0;i<m->len;i++)
  {
    m->data[i]=RxMessage.Data[i];
  }
  return 1;		// message received
}

/******************************************************************************

 ******************************* CAN INTERRUPT  *******************************/
 void CAN1_RX0_IRQHandler(void)
{
  volatile unsigned int status;
  static Message m = Message_Initializer;		// contain a CAN message
  u8 i;		
  // Rx Interrupt  
   if (canReceive_1(&m))			// a message received
   {
      canDispatch(&ObjDict1_Data, &m);         // process it
   }
}
//void can_irq_handler(void)
void CAN2_RX0_IRQHandler(void)
{

  volatile unsigned int status;
  static Message m = Message_Initializer;		// contain a CAN message
  u8 i;		
  // Rx Interrupt  
   if (canReceive_2(&m))			// a message received
   {
      canDispatch(&ObjDict2_Data, &m);         // process it
   }
}