/****************************************Copyright (c)**************************************************
**                               ������������Ƭ����չ���޹�˾
**                                     ��    ��    ��
**                                        ��Ʒһ�� 
**
**                                 http://www.zlgmcu.com
**
**--------------�ļ���Ϣ--------------------------------------------------------------------------------
**��   ��   ��: MBSlave.c
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
#include	"modbus_config.h"
#include	"modbus.h"
#include    "MBSlave.h"


#ifdef UCOSII
	OS_EVENT *mb_handle_event_ptr;
#endif
extern uint8_t mb_handle_event_flag;


extern uint16_t Modbus_CRC16(uint8_t *Buff_addr,uint16_t len);

extern SendResponse(uint8_t *buff,uint16_t len);

// ���ܴ��봦��������
extern AT_CODE FUNCTION_ARRAY	FCodeHandleArray[MAX_FUNCTION];			  


//********************************************************************************************************
// �������ƣ�MBSlaveIni
// ���������DevicInfPtr���ӻ�������Ϣ���ݽṹָ��
// �����������
// ����������MODBUS�ӻ���ʼ��
//********************************************************************************************************
void MBSlaveIni(SLAVE_INFORMATION *DevicInfPtr)
{
	
	ADUData.Address = DevicInfPtr->Address;
	ADUData.ADUBuffPtr = DevicInfPtr->RequestPtr;
	ADUData.ADULength = 0;

//	ADUData.RedundancyCheck = 0;
	ADUData.EndT15 = FALSE;
	ADUData.FrameOK = FALSE;
	
	PDUData.PDUBuffPtr = NULL;
//	PDUData.Response.ExceptionCode = 0;

#ifdef UCOSII
	mb_handle_event_ptr = OSSemCreate(0);			// MODBUS�����ź���
#endif
}


//********************************************************************************************************
// �������ƣ�FunctionHandle
// �����������
// �����������
// ��������������������
//********************************************************************************************************
void FunctionHandle(void)
{
	uint8_t i;
	PDU_RESPONSE Response;
	for(i=0;i<MAX_FUNCTION;i++)
	{
		if(FCodeHandleArray[i].Code==PDUData.FunctionCode)
		{
			Response.PDUDataPtr = PDUData.PDUBuffPtr;
			Response.PDUByteLength = PDUData.PDULength;

			FCodeHandleArray[i].Function( &Response );		// ִ�д��������ݽṹָ��
			
		//	PDUData.PDUBuffPtr = Response.PDUDataPtr;
			PDUData.PDULength = Response.PDUByteLength;
			PDUData.ExceptionCode = Response.ExceptionCode;
			return;
		}	
	}
	PDUData.ExceptionCode = EXCEPTION_CODE_1;	//	���ܴ�����Ч
}

void PackPDU2ADU(void)
{
	if(PDUData.ExceptionCode)	// ������ʧ��
	{
		*PDUData.PDUBuffPtr = PDUData.FunctionCode|0x80;
		*(PDUData.PDUBuffPtr+1) = PDUData.ExceptionCode;
		PDUData.PDULength = 2;
	}
	ADUData.ADULength = PDUData.PDULength + 3;		// ADU = Adress + PDU + CRC/LRC = 1 + PDU + 2;
	
}

//********************************************************************************************************
// �������ƣ�TransfersReplyHandle
// �����������
// �����������
// ��������������Ӧ��֡����
//********************************************************************************************************
void TransfersReplyHandle(void)
{
	uint16_t	CRC_OR_LRC;
	*ADUData.ADUBuffPtr = ADUData.Address;	  								// �豸��ַ
	CRC_OR_LRC = Modbus_CRC16(ADUData.ADUBuffPtr,ADUData.ADULength-2);		// ����CRC (ADU��CRCǰ������)
	*(ADUData.ADUBuffPtr+ADUData.ADULength-1) = (uint8_t)(CRC_OR_LRC/0x100);	// CRC Hi or LRC Hi 
	*(ADUData.ADUBuffPtr+ADUData.ADULength-2) = (uint8_t)CRC_OR_LRC;			// CRC Hi or LRC Hi 

	SendResponse(ADUData.ADUBuffPtr,ADUData.ADULength);
}


//********************************************************************************************************
// �������ƣ�FunctionHandle
// �����������
// �����������
// ����������MODBUS������
//********************************************************************************************************
void IdleModbus(void)
{
	uint8_t err;

	// ������Ч֡
//	OSSemPend(mb_handle_event_ptr,0,&err);

    if(mb_handle_event_flag==1)
	  mb_handle_event_flag=0;
	else
	  return;
	if(PDUData.FrameOK==FALSE)		// ����֡δ��������
		return;
	PDUData.FrameOK = FALSE;
	 
	FunctionHandle();				// ���ܴ��봦����
	PackPDU2ADU();					// ��PDU֡��װ��ADU֡
	TransfersReplyHandle();			// ����Ӧ��֡
	// ׼���յ���һ֡

//	ADUData.ADUBuffPtr = SlaveDevice.RequestPtr;
	ADUData.ADULength = 0;
	PDUData.PDUBuffPtr = NULL;
}








