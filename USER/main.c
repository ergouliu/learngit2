/**
  ******************************************************************************
  * @file    main.c 
  * @author  Chen Bing
  * @version V1.1.0
  * @date    08-Nov.-2015
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f2xx.h"

#include "MODBUS.h"
#include "MBSlave.h"
#include "modbus_config.h"

#include "canfestival.h"
#include "objdict1.h"

static __IO uint32_t TimingDelay;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nTime);



/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CanTxMsg TxMessage;
CanRxMsg RxMessage;

uint8_t KeyNumber = 0x0;

/* Private function prototypes -----------------------------------------------*/
void NVIC_Config(void);
void CAN1_Config(void);
void CAN2_Config(void);
void Init_RxMes(CanRxMsg *RxMessage);
/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
uint16_t PrescalerValue = 0;

/* Private function prototypes -----------------------------------------------*/
void TIM_Config(void);

//static void TIM_Configuration(void);
static void USART3_Config(void); 
static void SPI_Config(void);

uint8_t	ADUBuff[MAX_ADU_LENGTH];  
SLAVE_INFORMATION 	 SlaveDevice = {
	0x04,					// 地址
	0x00,					// 链路层协议
	1152,					// 波特率 = BaudRate * 100	
	0,						// 奇偶校验
	2,						// 停止位
	ADUBuff					// 主机请求从帧指针
};
//uint32 T15TC_Value;
//uint32 T35TC_Value;
uint8_t mb_handle_event_flag;

void StartCountT15(void)
{
  TIM_SetCounter(TIM2,0x0000);
  TIM_Cmd(TIM2, ENABLE);
}

//********************************************************************************************************
// 函数名称：StartCountT35
// 输入参数：无
// 输出参数：无
// 功能描述：使能T3.5计数。T3.5的时间值为UART传输3.5个字符的时间。
//			 注意：该函数只是使能计时，不允许在函数中产生任何延时。
//			       如果T3.5时间结束执行T35EndHandle函数。
//********************************************************************************************************
void StartCountT35(void)
{
  TIM_SetCounter(TIM3,0x0000);
  TIM_Cmd(TIM3, ENABLE);
}
void SendResponse(uint8_t *buff,uint16_t len)
{	
  uint16_t i;
  GPIO_SetBits (GPIOC, GPIO_Pin_4); 
	Delay(1);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
  for(i=0;i<len;i++)
  {
    USART_SendData(USART3,buff[i]);
	  while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
  }
	Delay(1);
	GPIO_ResetBits (GPIOC, GPIO_Pin_4);
}

uint8_t MB_GetDiscrete(uint16_t Address)
{
  uint8_t i;
  if(Address<4)
    i=1;
  else
    i=0;
  return i;
}
uint8_t MB_GetCoils(uint16_t Address)
{
  uint8_t i;
  if(Address<4)
    i=1;
  else
    i=0;
  return i;
}
uint8_t MB_SetCoil(uint16_t Address,uint8_t CoilValue)
{	
	return TRUE;
}

/***************************************************
** 配置输入寄存器的参数。
**
****************************************************/
uint16_t InputRegBuffer[END_INPUT_REG_ADDR]={0,0};
//**************************************************
// 函数名称：MB_GetInputRegValue
// 输入参数：Address，寄存器地址
// 输出参数：返回寄存器值
// 功能描述：读输入寄存器值函数，访函数由用户编写
//**************************************************
uint16_t MB_GetInputRegValue(uint16_t Address)
{
	return InputRegBuffer[Address];

}
//**************************************************
// 函数名称：MB_SetInputRegValue
// 输入参数：Address，寄存器地址
//			 Value，设置输入寄存器的值
// 输出参数：无
// 功能描述：读输入寄存器值函数，访函数由用户编写
//**************************************************
void MB_SetInputRegValue(uint16_t Address,uint16_t Value)
{
//	InputRegBuffer[Address] = Value;
}

///***************************************************************
///
///  Modbus与CANopen的映射
///
///***************************************************************

void MB_GetRegValue_8( UNS16 index, UNS8 subIndex,UNS32 len,UNS8 dataType);
void MB_GetRegValue_16( UNS16 index, UNS8 subIndex,UNS32 len,UNS8 dataType);
void MB_GetRegValue_32( UNS16 index, UNS8 subIndex,UNS32 len,UNS8 dataType);
void MB_GetRegValue_bp( UNS8 subIndex);

uint16_t HoldRegBuffer[8];
UNS8 rdata[5];
UNS8 rdata_bp[77];
UNS16 rdata16[55];
UNS32 rdata32[1]; 
UNS16 wdata16[32]; 
UNS8  wdata8[32]; 
UNS32 size;
UNS32 abortCode = 0;

uint16_t MB_GetRegValue(uint16_t Address)
{

	if(Address<4)
{
	MB_GetRegValue_8( 0x2000,1,4,0x05);
  return rdata[Address];
}  
else if((Address>3)&&(Address<7))
	{
		MB_GetRegValue_16( 0x2001, 1,4,0x06);
		return rdata16[(Address-4)/2]; 	
	}
else if(Address==8)
	{
		MB_GetRegValue_16( 0x2002, 1,2,0x06);
		return rdata16[0]; 	 
	}
else if(Address==10)
	{
		MB_GetRegValue_16( 0x2003, 1,2,0x03);  
		return rdata16[0]; 	 
	}	
else if((Address>11)&&(Address<15))
	{
		MB_GetRegValue_16( 0x2004, 1,4,0x06);
	  return rdata16[(Address-12)/2];	
	}	
else if((Address>15)&&(Address<19))
	{
			MB_GetRegValue_16( 0x2005, 1,4,0x06);
	   return rdata16[(Address-16)/2];
	}	
else if(Address==20)
	{
		MB_GetRegValue_16( 0x2006, 1,2,0x03);	
		return rdata16[0]; 	 
	}
else if((Address>21)&&(Address<131))
	{
		MB_GetRegValue_16( 0x2007, 1,110,0x03);
		return (rdata16[(Address-22)/2]);
	}	
else if((Address>131)&&(Address<241))
	{
		MB_GetRegValue_16( 0x2008, 1,110,0x03);
		return (rdata16[(Address-132)/2]);
	}	
else if((Address>241)&&(Address<351))
	{
		MB_GetRegValue_16( 0x2009, 1,110,0x03);
		return (rdata16[(Address-242)/2]);
	}	
else if((Address>351)&&(Address<415))
	{
		MB_GetRegValue_16( 0x200A, 1,64,0x03);
		return (rdata16[(Address-352)/2]);
	}	
else if((Address>415)&&(Address<493))
	{
    MB_GetRegValue_bp(0);
		return ( rdata_bp[Address-416]);
	}	
else if((Address>492)&&(Address<570))
	{
		MB_GetRegValue_bp(1);
		return ( rdata_bp[Address-493]);
	}	
else if((Address>569)&&(Address<647))
	{
		MB_GetRegValue_bp(2);
		return ( rdata_bp[Address-570]);
	}	
else if((Address>646)&&(Address<724))
	{
		MB_GetRegValue_bp(3);
		return ( rdata_bp[Address-647]);
	}
else if((Address>723)&&(Address<801))
	{
		MB_GetRegValue_bp(4);
		return ( rdata_bp[Address-724]);	
	}
else if((Address>800)&&(Address<878))
	{
		MB_GetRegValue_bp(5);
		return ( rdata_bp[Address-801]);
	}
else if((Address>877)&&(Address<955))
	{
		MB_GetRegValue_bp(6);
		return ( rdata_bp[Address-878]);	
	}
else if((Address>954)&&(Address<1032))
	{
			MB_GetRegValue_bp(7);
		return ( rdata_bp[Address-955]);		
	}
else if((Address>1031)&&(Address<1109))
	{
			MB_GetRegValue_bp(8);
		return ( rdata_bp[Address-1032]);		
	}	
else if((Address>1108)&&(Address<1186))
	{
			MB_GetRegValue_bp(9);
		return ( rdata_bp[Address-1109]);		
	}
else if((Address>1185)&&(Address<1263))
	{
			MB_GetRegValue_bp(10);
		return ( rdata_bp[Address-1186]);		
	}
else if((Address>1262)&&(Address<1340))
	{
			MB_GetRegValue_bp(11);
		return ( rdata_bp[Address-1263]);		
	}
else if((Address>1339)&&(Address<1417))
	{
			MB_GetRegValue_bp(12);
		return ( rdata_bp[Address-1340]);		
	}
else if((Address>1416)&&(Address<1494))
	{
			MB_GetRegValue_bp(13);
		return ( rdata_bp[Address-1417]);		
	}
else if((Address>1493)&&(Address<1571))
	{
			MB_GetRegValue_bp(14);
		return ( rdata_bp[Address-1494]);		
	}
else if((Address>1570)&&(Address<1648))
	{
			MB_GetRegValue_bp(15);
		return ( rdata_bp[Address-1571]);		
	}
else if((Address>1647)&&(Address>1657))
	{
			MB_GetRegValue_16( 0x200C, 1,2,0x03);  
		return rdata16[0]; 	 
	}
else if(Address==1658)
	{
		 MB_GetRegValue_16( 0x200D, 1,2,0x03);  
		return rdata16[0]; 		
	}
else if(Address==1660)
	{
		MB_GetRegValue_8( 0x200E,1,1,0x05);
		return rdata[0]; 	 
	}
else if(Address==1661)
	{
		MB_GetRegValue_8( 0x200F,1,1,0x05);
		return rdata[0]; 
	}
else if(Address==1662)
	{
		MB_GetRegValue_16( 0x2010, 1,2,0x06);  
		return rdata16[0]; 
	}	
else if(Address==1664)
	{
		MB_GetRegValue_16( 0x2011, 1,2,0x06);  
		return rdata16[0]; 	
	}	
else if(Address==1666)
	{
		MB_GetRegValue_16( 0x2012, 1,2,0x06);  
		return rdata16[0]; 	
	}	
else if(Address==1668)
	{
		MB_GetRegValue_16( 0x2013, 1,2,0x06);  
		return rdata16[0]; 			 
	}	
else if(Address==1676)
	{
	  MB_GetRegValue_32( 0x2017, 1,4,0x07);
		return rdata32[0]; 	 
	}		
else if(Address==1684)
	{
		MB_GetRegValue_32( 0x2019, 0,4,0x07);
		return rdata32[0]; 	
	}	
else if(Address==1688)
	{
		MB_GetRegValue_32( 0x2019,1,4,0x07);
		return rdata32[0]; 
	}
else if(Address==1692)
	{
		MB_GetRegValue_32( 0x2019,2,4,0x07);
		return rdata32[0]; 
	}
else if(Address==1696)
	{
	  MB_GetRegValue_16( 0x201A, 1,2,0x06);  
		return rdata16[0]; 
	}
else if(Address==1700)
	{
		MB_GetRegValue_8( 0x200C,1,1,0x05);
		return rdata[0]; 	 
	}
else if(Address==1703)
	{
		MB_GetRegValue_16( 0x201F, 1,2,0x06);  
		return rdata16[0];  	 
	}
else if(Address==1707)
	{
		MB_GetRegValue_16( 0x2021, 1,2,0x03);  
		return rdata16[0];  	 		
	}
else if(Address==1709)
	{
		MB_GetRegValue_8( 0x2022,1,1,0x05);
		return rdata[0];
	}
else if(Address==1713)
	{
		MB_GetRegValue_8( 0x2026,1,1,0x05);
		return rdata[0];	 	 
	}	
	
////////////////////////////////////////////
////////////////////////////////////////////
else if(Address==1715)
	{
		size = 4;
	  readNetworkDict(&ObjDict1_Data, 0x04, 0x2028, 1, 0x03);  
    while (getReadResultNetworkDict (&ObjDict1_Data, 0x04, &rdata16, &size,&abortCode) == SDO_UPLOAD_IN_PROGRESS);
    closeSDOtransfer (&ObjDict1_Data, 0x04, SDO_CLIENT);	  
		return rdata16[0]; 	 
	}	
else if(Address==1719)
	{
		size = 4;
	  readNetworkDict(&ObjDict1_Data, 0x04, 0x2029, 1, 0x03);  
    while (getReadResultNetworkDict (&ObjDict1_Data, 0x04, &rdata16, &size,&abortCode) == SDO_UPLOAD_IN_PROGRESS);
    closeSDOtransfer (&ObjDict1_Data, 0x04, SDO_CLIENT);	  
		return rdata16[0]; 	 
	}	
///////////////////////////////////////////
////////////////////////////////////////////
else if(Address==1723)
	{
	  MB_GetRegValue_8( 0x202A,1,1,0x05);
		return rdata[0];	 	 			 
	}	
else if(Address==1725)
	{
		MB_GetRegValue_8( 0x202C,1,1,0x05);
		return rdata[0];				 
	}	
else if(Address==1727)
	{
		MB_GetRegValue_8( 0x202E,1,1,0x05);
		return rdata[0];	 	 
	}	
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
else if(Address==1728)
	{
		MB_GetRegValue_16( 0x202F, 1,3,0x06);  
		return rdata16[0];  		
	}	
else if(Address==1731)
	{
  	MB_GetRegValue_16( 0x202F, 1,2,0x06);  
		return rdata16[0]; 
		size = 4;
	  readNetworkDict(&ObjDict1_Data, 0x04, 0x2030, 1, 0x06);  
    while (getReadResultNetworkDict (&ObjDict1_Data, 0x04, &rdata16, &size,&abortCode) == SDO_UPLOAD_IN_PROGRESS);
    closeSDOtransfer (&ObjDict1_Data, 0x04, SDO_CLIENT);	  
		return rdata16[0]; 	 
	}	
else if(Address==1735)
	{
		size = 4;
	  readNetworkDict(&ObjDict1_Data, 0x04, 0x2030, 2, 0x06);  
    while (getReadResultNetworkDict (&ObjDict1_Data, 0x04, &rdata16, &size,&abortCode) == SDO_UPLOAD_IN_PROGRESS);
    closeSDOtransfer (&ObjDict1_Data, 0x04, SDO_CLIENT);	  
		return rdata16[0]; 	 
	}	
else if(Address==1739)
	{
		size = 4;
	  readNetworkDict(&ObjDict1_Data, 0x04, 0x2030, 3, 0x06);  
    while (getReadResultNetworkDict (&ObjDict1_Data, 0x04, &rdata16, &size,&abortCode) == SDO_UPLOAD_IN_PROGRESS);
    closeSDOtransfer (&ObjDict1_Data, 0x04, SDO_CLIENT);	  
		return rdata16[0]; 	 
	}		
////////////////////////////////////////////////////
///////////////////////////////////////////////////
else if(Address==1743)
	{
		MB_GetRegValue_8( 0x2031,1,1,0x05);
		return rdata[0];				 
	}	
else if(Address==1744)
	{
		MB_GetRegValue_16( 0x2032, 1,2,0x03);  
		return rdata16[0]; 	
	}	
else if(Address==1746)
	{
		MB_GetRegValue_8( 0x2033,1,1,0x05);
		return rdata[0];	
	}		
else if(Address==1747)
	{
		MB_GetRegValue_16( 0x2034, 1,2,0x03);  
		return rdata16[0]; 		 
	}	
else if(Address==1748)
	{
		MB_GetRegValue_8( 0x2035,1,1,0x05);
		return rdata[0];		
	}	
else if(Address==1753)
	{
	 	 MB_GetRegValue_16( 0x2038, 1,2,0x06);  
		return rdata16[0]; 
	}		
else if(Address==1755)
	{
		 MB_GetRegValue_16( 0x2038,2,2,0x06);  
		return rdata16[0]; 
	}		
else if(Address==1757)
	{
		MB_GetRegValue_16( 0x2038,3,2,0x06);  
		return rdata16[0]; 
	}	
else if(Address==1759)
	{
	  MB_GetRegValue_8( 0x203A,1,1,0x05);
		return rdata[0];		
		
	}		
else if(Address==1760)
	{
		MB_GetRegValue_16( 0x2040,1,2,0x06);  
		return rdata16[0]; 
	}		
else if(Address==1762)
	{
		 MB_GetRegValue_8( 0x2041,1,1,0x05);
		return rdata[0];
		
	}		
else if(Address==1765)
	{
		MB_GetRegValue_16( 0x2050, 1,2,0x03);  
		return rdata16[0]; 
	}		
else if(Address==1767)
	{
		MB_GetRegValue_16( 0x2050, 1,2,0x03);  
		return rdata16[0]; 
	}		
else if(Address==1781)
	{
	  MB_GetRegValue_32( 0x20DF, 1,4,0x07);
		return rdata32[0]; 
	}	
else if(Address==1789)
	{
		MB_GetRegValue_16( 0x20E1,1,2,0x06);  
		return rdata16[0]; 
		
	}	
	else if(Address==1803)
	{
		MB_GetRegValue_32( 0x2AAA, 1,4,0x07);
		return rdata32[0]; 	
	}	
	
//	return HoldRegBuffer[Address];
	//return REG_RWs[Address*2+1]*256+REG_RWs[Address*2];
}

void MB_GetRegValue_8( UNS16 index, UNS8 subIndex,UNS32 len,UNS8 dataType)
{
    size = len;
	  readNetworkDict(&ObjDict1_Data, 0x04, index, subIndex, dataType);  
    while (getReadResultNetworkDict (&ObjDict1_Data, 0x04, &rdata, &size,&abortCode) == SDO_UPLOAD_IN_PROGRESS);
    closeSDOtransfer (&ObjDict1_Data, 0x04, SDO_CLIENT);	   
}
void MB_GetRegValue_16( UNS16 index, UNS8 subIndex,UNS32 len,UNS8 dataType)
{
	  size = len;
	  readNetworkDict(&ObjDict1_Data, 0x04, index, subIndex, dataType);  
    while (getReadResultNetworkDict (&ObjDict1_Data, 0x04, &rdata16, &size,&abortCode) == SDO_UPLOAD_IN_PROGRESS);
    closeSDOtransfer (&ObjDict1_Data, 0x04, SDO_CLIENT);	  	 
}
void MB_GetRegValue_32( UNS16 index, UNS8 subIndex,UNS32 len,UNS8 dataType)
{
    size = len;
	  readNetworkDict(&ObjDict1_Data, 0x04, index, subIndex, dataType);  
    while (getReadResultNetworkDict (&ObjDict1_Data, 0x04, &rdata32, &size,&abortCode) == SDO_UPLOAD_IN_PROGRESS);
    closeSDOtransfer (&ObjDict1_Data, 0x04, SDO_CLIENT);	 
}
void MB_GetRegValue_bp( UNS8 subIndex)
{
    size = 77;
	  readNetworkDict(&ObjDict1_Data, 0x04, 0x200B, subIndex,0x05);  
    while (getReadResultNetworkDict (&ObjDict1_Data, 0x04, &rdata, &size,&abortCode) == SDO_UPLOAD_IN_PROGRESS);
    closeSDOtransfer (&ObjDict1_Data, 0x04, SDO_CLIENT);
}

void MB_SetRegValue(uint16_t Address,uint16_t Value)
{
//	  switch(Address)
//		{
//		  case 1670:
//			  MB_SetRegValue16(uint16_t Address,uint16_t Value) 
//			wdata[0] = 0x09;
//	wdata[1] = 0x03;
//	wdata[2] = 0x04;
//  res=writeNetworkDict (&ObjDict1_Data, 0x04, 0x202f,1,3,0x05,&wdata);
//  while(getWriteResultNetworkDict (&ObjDict1_Data, 0x04, &abortCode)!= SDO_FINISHED);
//  closeSDOtransfer (&ObjDict1_Data, 0x04, SDO_CLIENT);	 
//		}
	  HoldRegBuffer[Address] = Value;
//  REG_RWs[Address*2+1]=(u8)(Value/256);
 // REG_RWs[Address*2]=(u8)(Value%256);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */



int main(void)
{
  UNS32 data=0;
  UNS16 data1=0;
  UNS32 i;
  UNS32 tpdo_cob;
	
	GPIO_InitTypeDef  GPIO_InitStructure1;
	GPIO_InitTypeDef  GPIO_InitStructure2;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);
	
  /* NVIC Configuration */
  NVIC_Config();
  
  /* Enable the GPIO_LED Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* Configure the GPIO_LED and RS485_DE pin */
  GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4;
  GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure1.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure1);

  GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure2);

 /* Enable the GPIO_INPUT Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* Configure the GPIO_Input pin */
  GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure2);

/* USART3 configuration */
  USART3_Config();
	
	if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  }
	TIM_Config();  
	GPIO_ResetBits (GPIOC, GPIO_Pin_4); 
  /* TIM2 enable counter */
  //TIM_Cmd(TIM2, ENABLE);
	//TIM_Cmd(TIM2, ENABLE);
  MBSlaveIni(&SlaveDevice);
  mb_handle_event_flag=0;

/* SPI2 configuration ---------------------------------------*/
  SPI_Config();


	GPIO_SetBits (GPIOC, GPIO_Pin_0); // RUN_LED OFF
	GPIO_SetBits (GPIOC, GPIO_Pin_1); //COM_LED OFF
  
//	HoldRegBuffer[0] = 1990;
//  HoldRegBuffer[1] = 1701;
//	HoldRegBuffer[2] = 1512;
//	HoldRegBuffer[3] = 1392;
	
	/* CAN configuration */
  //CAN1_Config();
	//CAN2_Config();
  
	canInit();
	initTimer();

//debug begin
		
  /* Transmit Structure preparation */
//  TxMessage.StdId = 0x602;
//  TxMessage.ExtId = 0x00;
//  TxMessage.RTR = CAN_RTR_DATA;
//  TxMessage.IDE = CAN_ID_STD;
//  TxMessage.DLC = 8;
//  TxMessage.Data[0] = 0x40;
//	TxMessage.Data[1] = 0x00;
//	TxMessage.Data[2] = 0x10;
//	TxMessage.Data[3] = 0x20;
//	TxMessage.Data[4] = 0x00;
//	TxMessage.Data[5] = 0x00;
//	TxMessage.Data[6] = 0x00;
//	TxMessage.Data[7] = 0x00;
//	CAN_Transmit(CAN1, &TxMessage);
	//debug end
	

  ObjDict1_Data.canHandle=1;
//  ObjDict2_Data.canHandle=2;

  setNodeId (&ObjDict1_Data, 0x07);//Node ID = 7
  setState(&ObjDict1_Data, Initialisation);	// Init the state
	setState(&ObjDict1_Data, Pre_operational);
	while(1)
  {
		IdleModbus();		
  }
	
}

/**
  * @brief  Configures the NVIC.
  * @param  None
  * @retval None
  */
void NVIC_Config(void)
{
  NVIC_InitTypeDef  NVIC_InitStructure;

    /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	/* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
//  NVIC_InitTypeDef  NVIC_InitStructure;

//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
 
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
  NVIC_Init(&NVIC_InitStructure);

   /* Enable the TIM4 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
 
}

/**
  * @brief  Initializes the Rx Message.
  * @param  RxMessage: pointer to the message to initialize
  * @retval None
  */
void Init_RxMes(CanRxMsg *RxMessage)
{
  uint8_t i = 0;

  RxMessage->StdId = 0x00;
  RxMessage->ExtId = 0x00;
  RxMessage->IDE = CAN_ID_STD;
  RxMessage->DLC = 0;
  RxMessage->FMI = 0;
  for (i = 0;i < 8;i++)
  {
    RxMessage->Data[i] = 0x00;
  }
}


static void USART3_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	/* USART IOs configuration ***********************************/
  /* Enable GPIOC clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  /* Connect PC10 to USART3_Tx */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
  /* Connect PC11 to USART3_Rx*/
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
  /* Configure USART3_Tx and USART3_Rx as alternate function */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	/* USART configuration ***************************************/
  /* USART3 configured as follow:
  - BaudRate = 115200 baud
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  /* Enable USART3 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure);
  
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);   
	
	/* Enable USART3 */
  USART_Cmd(USART3, ENABLE);

}

static void SPI_Config(void)
{
	
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;

	/* Enable the SPI clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* Enable the GPIOB clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Connect PB13 to SPI2_SCK */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);

	/* Connect PB14 to SPI2_MISO */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);

	/* Connect PB15 to SPI2_MOSI */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* SPI configuration *****************************************/
  SPI_InitStructure.SPI_Direction =
  SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);
  SPI_Cmd(SPI2, ENABLE);
}	
	
/**
  * @brief  Configure the TIM IRQ Handler.
  * @param  None
  * @retval None
  */
void TIM_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;  
	/* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  /* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
    /* Compute the prescaler value */
  //PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 6000000) - 1;  
	PrescalerValue = 59;//1us
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 750;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);
   TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 5750;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);
   TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
