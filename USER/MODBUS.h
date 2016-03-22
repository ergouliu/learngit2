/****************************************Copyright (c)**************************************************
**                               ������������Ƭ����չ���޹�˾
**                                     ��    ��    ��
**                                        ��Ʒһ�� 
**
**                                 http://www.zlgmcu.com
**
**--------------�ļ���Ϣ--------------------------------------------------------------------------------
**��   ��   ��: MODBUS.h
**��   ��   ��: ����ɽ
**����޸�����: 2005��7��29��
**��        ��: MODBUS RTU Э��ջ
**
**--------------��ʷ�汾��Ϣ----------------------------------------------------------------------------
** ������: ����ɽ
** ��  ��: 1.0a
** �ա���: 2005��7��29��
** �衡��: ԭʼ�汾
**
**------------------------------------------------------------------------------------------------------
** �޸���: 
** ��  ��: 
** �ա���: 
** �衡��: 
**
**--------------��ǰ�汾�޶�------------------------------------------------------------------------------
** �޸���: 
** �ա���: 
** �衡��: 
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
	uint8_t*	PDUDataPtr; 	// ��������ָ��,���ܴ��봦�������轫��������ݰ����ڸ�ָ��Ŀ�ʼλʼ
  uint8_t	PDUByteLength; 		// ��������������
	uint8_t	ExceptionCode;	// ��Ϊ������쳣���룬����������Ϊ0
 }PDU_RESPONSE;

/* PDU������ */
typedef struct __PDU_HANDLE{
	uint8_t*			PDUBuffPtr;					// PUD����ָ��,ΪNULLʱ��ʾ��Ч��PDU

	uint8_t	FrameOK;

	uint8_t 			FunctionCode;				// �����ܴ���
	uint8_t			PDULength;					// PDU�ֽڳ���
				   	
 //   PDU_RESPONSE   	Response;
	uint8_t	ExceptionCode;
}PDU_HANDLE;

extern PDU_HANDLE PDUData;


typedef struct __ADU_CONTROL{
	uint8_t* 	ADUBuffPtr;	  				// ADU������ָ��

	uint8_t	Address;
	uint16_t	ADULength;					// ADU�ֽڳ���

//	uint16	RedundancyCheck;			// ������У��,CRC/LRC

	uint8_t	EndT15;
	uint8_t	FrameOK;
//	uint8   TimeOut;					// ��ʱ��־
			
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
// ����UC/OSII��������
void OSModbusServe	(void *pdata);
#endif

