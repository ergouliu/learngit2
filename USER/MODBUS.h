/****************************************Copyright (c)**************************************************
**                               广州周立功单片机发展有限公司
**                                     研    究    所
**                                        产品一部 
**
**                                 http://www.zlgmcu.com
**
**--------------文件信息--------------------------------------------------------------------------------
**文   件   名: MODBUS.h
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
#ifndef __MODBUS_H
#define __MODBUS_H

#include "modbus_config.h"



#define	MAX_PDU_DATA_LENGTH		 253
#define	MAX_PDU_LENGTH		 	(MAX_PDU_DATA_LENGTH+1)
#define MAX_ADU_LENGTH  		(MAX_PDU_LENGTH+2)

//#define MAX_FUNCTION			10
#define EXCEPTION_CODE_1		1
#define EXCEPTION_CODE_2		2
#define EXCEPTION_CODE_3		3
#define EXCEPTION_CODE_4		4
#define EXCEPTION_CODE_5		5
#define EXCEPTION_CODE_6		6

#define MB_NO_ERR					0x00
#define ILLEGAL_FUNCTION			0x01
#define ILLEGAL_DATA_ADDR			0x02
#define ILLEGAL_DATA_VALUE			0x03	
#define SLAVE_DEVICE_FAILURE		0x04
#define ACKNOWLEDGE					0x05
#define SLAVE_DEVICE_BUSY			0x06
#define MEMORY_PARITY_ERR			0x08
#define GATEWAY_PATH_UNABLE			0x0A
#define GATEWAY_TARGET_DEV_FAIL		0x0B

#define MB_FUN_NOT_DEF				0x80
#define MB_MASTER_BUSY				0x81
#define MB_MASTER_IDLE				0x82
#define MB_RESPONSE_TIME_OUT		0x82
#define MB_PARAMETER_ERR			0x83

#define	BROADCAST_ADDRESS	0


	


typedef struct __PDU_RESPONSE{
	uint8_t*	PDUDataPtr; 	// 请求数据指针,功能代码处理函数必需将处理的数据包存在该指针的开始位始
  uint8_t	PDUByteLength; 		// 请求数据数长度
	uint8_t	ExceptionCode;	// 仅为输出错异常代码，正常操作设为0
 }PDU_RESPONSE;

/* PDU描述符 */
typedef struct __PDU_HANDLE{
	uint8_t*			PDUBuffPtr;					// PUD数据指针,为NULL时表示无效的PDU

	uint8_t	FrameOK;

	uint8_t 			FunctionCode;				// 请求功能代码
	uint8_t			PDULength;					// PDU字节长度
				   	
 //   PDU_RESPONSE   	Response;
	uint8_t	ExceptionCode;
}PDU_HANDLE;

extern PDU_HANDLE PDUData;


typedef struct __ADU_CONTROL{
	uint8_t* 	ADUBuffPtr;	  				// ADU缓冲区指针

	uint8_t	Address;
	uint16_t	ADULength;					// ADU字节长度

//	uint16	RedundancyCheck;			// 冗余码校验,CRC/LRC

	uint8_t	EndT15;
	uint8_t	FrameOK;
//	uint8   TimeOut;					// 超时标志
			
}ADU_CONTROL;

extern volatile ADU_CONTROL ADUData;

extern volatile uint8_t g_TimeEnd;

void Start10mS(void);
void StartCountT35(void);
void StartCountT15(void);

void ReceOneChar(uint8_t ReceCharacter);
uint8_t PutDataInPUD(uint8_t *CharPtr,uint16_t Length);


void T15EndHandle(void);
void T35EndHandle(void);
void Waite10mS(uint32_t time);
void Time10mSHandle(void);
// 基于UC/OSII服务任务
void OSModbusServe	(void *pdata);
#endif

