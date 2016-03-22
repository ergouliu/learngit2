/****************************************Copyright (c)**************************************************
**                               广州周立功单片机发展有限公司
**                                     研    究    所
**                                        产品一部 
**
**                                 http://www.zlgmcu.com
**
**--------------文件信息--------------------------------------------------------------------------------
**文   件   名: MBCommon.c
**创   建   人: 周立山
**最后修改日期: 2005年7月29日
**描        述: MODBUS RTU 协议栈
**
**--------------历史版本信息----------------------------------------------------------------------------
** 创建人: 周立山
** 版  本: 1.0a
** 日　期: 2005年7月29日
** 描　述: 原始版本
**
**------------------------------------------------------------------------------------------------------
** 修改人: 
** 版  本: 
** 日　期: 
** 描　述: 
**
**--------------当前版本修订------------------------------------------------------------------------------
** 修改人: 
** 日　期: 
** 描　述: 
**
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
#include "modbus_config.h"
#include "modbus.h"

PDU_HANDLE 				PDUData;
volatile ADU_CONTROL 	ADUData;

#ifdef UCOSII
extern	OS_EVENT *mb_handle_event_ptr;
#endif
extern uint8_t mb_handle_event_flag;
extern uint16_t Modbus_CRC16(uint8_t *Buff_addr,uint16_t len);

//********************************************************************************************************
// 函数名称：StartCountT15
// 输入参数：无
// 输出参数：无
// 功能描述：使能T1.5计数。T1.5的时间值为UART传输1.5个字符的时间。
//			 注意：该函数只是使能计时，不允许在函数中产生任何延时。
//			       如果T1.5时间结束执行T15EndHandle函数。
//********************************************************************************************************
extern void StartCountT15(void); 

//********************************************************************************************************
// 函数名称：StartCountT35
// 输入参数：无
// 输出参数：无
// 功能描述：使能T3.5计数。T3.5的时间值为UART传输3.5个字符的时间。
//			 注意：该函数只是使能计时，不允许在函数中产生任何延时。
//			       如果T3.5时间结束执行T35EndHandle函数。
//********************************************************************************************************
extern void StartCountT35(void);


volatile uint32_t g_10mS_count;
volatile uint8_t 	g_TimeEnd;				// 应答超时
//******************************************************************************************
// 函数名称：Get_10mS
// 输入参数：无
// 输出参数：10mS计数寄存器
// 功能描述：获取10mS计数值，该值每10毫秒加1
//******************************************************************************************
void Waite10mS(uint32_t time)
{
	g_TimeEnd = FALSE;
	g_10mS_count=time;
}



//*****************************************************************************
// 函数名称：OverTimeHandle
// 输入参数：无
// 输出参数：无
// 功能描述：
//*****************************************************************************
void Time10mSHandle(void)
{
	if(g_10mS_count==0)	return; // g_10mS_count,值为0表示没有计数事件,直接返回
	g_10mS_count--;				// 计数减1,直接到0为直
	if(g_10mS_count==0)
	{	g_TimeEnd = TRUE;
	#ifdef UCOSII
		OSSemPost(mb_handle_event_ptr);				// 发报信号量,使MODBUS服务任务处理传输
	#endif
	}
}

//********************************************************************************************************
// 函数名称：T15EndHandle
// 输入参数：无
// 输出参数：无
// 功能描述：T1.5超时处理
//********************************************************************************************************
void T15EndHandle(void)
{
	uint16_t RedundancyCheck;
	if(ADUData.EndT15)
		return;
	ADUData.EndT15 = TRUE;		// T1.5时间结束	

	if((*ADUData.ADUBuffPtr==BROADCAST_ADDRESS)||		// 验证地址是否有效
	   (*ADUData.ADUBuffPtr==ADUData.Address))
	{
		RedundancyCheck = *(ADUData.ADUBuffPtr+ADUData.ADULength-2)|
								  (*(ADUData.ADUBuffPtr+ADUData.ADULength-1)<<8);
		if(RedundancyCheck == Modbus_CRC16(ADUData.ADUBuffPtr,ADUData.ADULength-2))	// CRC校验
		{
			ADUData.FrameOK = TRUE;
			return;
		}
	}
	ADUData.FrameOK = FALSE;
	
}

//********************************************************************************************************
// 函数名称：T35EndHandle
// 输入参数：无
// 输出参数：无
// 功能描述：T3.5结束处理
//********************************************************************************************************
void T35EndHandle(void)
{

	if(ADUData.FrameOK)
	{
		PDUData.PDUBuffPtr = ADUData.ADUBuffPtr+1;
		PDUData.PDULength = ADUData.ADULength-3;
		PDUData.FunctionCode = *PDUData.PDUBuffPtr; // 保存功能代码
		PDUData.FrameOK=TRUE;
	}
	else
	{
		PDUData.FrameOK=FALSE;
	}
   	ADUData.EndT15 = FALSE;		// 允许收到字符
	ADUData.ADULength = 0;
#ifdef UCOSII
	OSSemPost(mb_handle_event_ptr);				// 发报信号量,使MODBUS服务任务处理传输
#endif
    mb_handle_event_flag=1;
}

 
//********************************************************************************************************
// 函数名称：ReceOneChar
// 输入参数：ReceCharacter
// 输出参数：无
// 功能描述：接收一个字符处理
//           要求:接收到一个字符后立即传入该函数处理。
//********************************************************************************************************
void ReceOneChar(uint8_t ReceCharacter)
{

	if(ADUData.EndT15==TRUE)
	{	StartCountT35();			// 起动T3.5计时
		return;
	}
	ADUData.FrameOK = FALSE;
	StartCountT15();			// 起动T1.5计时
	StartCountT35();			// 起动T3.5计时

   	*(ADUData.ADUBuffPtr + ADUData.ADULength) = ReceCharacter;
	ADUData.ADULength++; 
}

//********************************************************************************************************
// 函数名称：PutCharInPUD
// 输入参数：ch,写入的字符
//			 EndChar，
// 输出参数：无
// 功能描述：接收一个字符处理
//           要求:接收到一个字符后立即传入该函数处理。
//********************************************************************************************************
uint8_t PutDataInPUD(uint8_t *CharPtr,uint16_t Length)
{
	uint16_t i;
	if(Length<=MAX_PDU_DATA_LENGTH)
	{
		for(i=0;i<Length;i++)
			*(PDUData.PDUBuffPtr+i)=*(CharPtr+i);
		PDUData.PDULength = Length ;
		PDUData.FrameOK = TRUE;
		return TRUE;
	}
	else
		return FALSE;
}


/*********************************************************************************************************
**                            OSModbusServe MODBUS服务任务
********************************************************************************************************/
#ifdef UCOSII
	void OSModbusServe	(void *pdata)
{	pdata = pdata;
	while (1)
	{	IdleModbus();
	}
}   
#endif