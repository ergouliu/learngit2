/****************************************Copyright (c)**************************************************
**                               ������������Ƭ����չ���޹�˾
**                                     ��    ��    ��
**                                        ��Ʒһ�� 
**
**                                 http://www.zlgmcu.com
**
**--------------�ļ���Ϣ--------------------------------------------------------------------------------
**��   ��   ��: MB_PFunction.c
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


//**************************************************
// �������ƣ�MB_GetDiscrete
// ���������Address����Ȧ��ַ
// ���������������Ȧֵ��0\1��
// ������������ȡ��ɢֵ�������ú������û���д
//**************************************************
uint8_t MB_GetDiscrete(uint16_t Address);

//**************************************************
// �������ƣ�MB_GetCoils
// ���������Address����Ȧ��ַ
// ���������������Ȧֵ��0\1��
// ������������ȡ��Ȧֵ�������ú������û���д
//**************************************************
uint8_t MB_GetCoils(uint16_t Address);

//**************************************************
// �������ƣ�MB_SetCoil
// ���������Address����Ȧ��ַ
//			 CoilValue,��Ȧֵ��0\1��
// ������������ؼĴ���ֵ
// ����������������Ȧֵ�������ú������û���д
//**************************************************
uint8_t MB_SetCoil(uint16_t Address,uint8_t CoilValue);


//**************************************************
// �������ƣ�MB_GetInputRegValue
// ���������Address���Ĵ�����ַ
// ������������ؼĴ���ֵ
// ����������������Ĵ���ֵ�������ú������û���д
//**************************************************
uint16_t MB_GetInputRegValue(uint16_t Address);

//**************************************************
// �������ƣ�MB_GetRegValue
// ���������Address���Ĵ�����ַ
// ������������ؼĴ���ֵ
// ���������������ּĴ���ֵ�������ú������û���д
//**************************************************
uint16_t MB_GetRegValue(uint16_t Address);

//**************************************************
// �������ƣ�MB_SetRegValue
// ���������Address���Ĵ�����ַ
//			 Value��д���ֵ
// �����������
// ����������д���ּĴ���ֵ�������ú������û���д
//**************************************************
void MB_SetRegValue(uint16_t Address,uint16_t Value);

/****************************************************************************************/
// �������ƣ�FReadCoils
// ���������Response
// �����������
// �������������������Ȧָ��01
/***************************************************************************************/
void FReadCoils(PDU_RESPONSE *Response);
/****************************************************************************************/
// �������ƣ�FReadDiscreteInputs
// ���������Response
// �����������
// �����������������ɢ����ָ��02
/***************************************************************************************/
void FReadDiscreteInputs(PDU_RESPONSE *Response);
/****************************************************************************************/
// �������ƣ�FReadHoldingReg
// ���������Response
// �����������
// ������������������ּĴ���ָ��03
/****************************************************************************************/
void FReadHoldingReg(PDU_RESPONSE *Response);
/****************************************************************************************/
// �������ƣ�FReadInputReg
// ���������Response
// �����������
// �������������������Ĵ���ָ��04
/****************************************************************************************/
void FReadInputReg(PDU_RESPONSE *Response);
/****************************************************************************************/
// �������ƣ�FWriteSingleCoil
// ���������Response
// �����������
// ��������������д����Ȧָ��05
/****************************************************************************************/
void FWriteSingleCoil(PDU_RESPONSE *Response);
/****************************************************************************************/
// �������ƣ�FWriteSingleReg
// ���������Response
// �����������
// ��������������д����Ȧָ��06
/****************************************************************************************/
void FWriteSingleReg(PDU_RESPONSE *Response);
/****************************************************************************************/
// �������ƣ�FWriteMultipleCoils
// ���������Response
// �����������
// ��������������д����Ȧָ��15
/****************************************************************************************/
void FWriteMultipleCoils(PDU_RESPONSE *Response);
/****************************************************************************************/
// �������ƣ�FWriteMultipleReg
// ���������Response
// �����������
// ��������������д��Ĵ���ָ��16
/****************************************************************************************/
void FWriteMultipleReg(PDU_RESPONSE *Response);
/****************************************************************************************/
// �������ƣ�FMaskWriteReg
// ���������Response
// �����������
// �����������������μĴ���ָ��22
/****************************************************************************************/
void FMaskWriteReg(PDU_RESPONSE *Response);
/****************************************************************************************/
// �������ƣ�FRWMultipleReg
// ���������Response
// �����������
// ���������������д��Ĵ���ָ��23
/****************************************************************************************/
void FRWMultipleReg(PDU_RESPONSE *Response);

void FTest(PDU_RESPONSE *Response){}

AT_CODE FUNCTION_ARRAY	FCodeHandleArray[MAX_FUNCTION]={
#if READ_COILS_EN > 0 
	{0x01,FReadCoils},			// ����Ȧ
#endif
#if READ_DIS_INPUT_EN > 0
	{0x02,FReadDiscreteInputs},	// ����ɢ������
#endif
#if READ_HOLD_REG_EN > 0
	{0x03,FReadHoldingReg},		// �����ּĴ���
#endif
#if READ_INPUT_REG_EN > 0
	{0x04,FReadInputReg},		// ������Ĵ���
#endif
#if WRITE_SING_COIL_EN > 0
	{0x05,FWriteSingleCoil},	// д������Ȧ
#endif
#if WRITE_SING_REG_EN > 0
	{0x06,FWriteSingleReg},		// д�����Ĵ���
#endif
#if WRITE_MULT_COIL_EN > 0
	{0x0f,FWriteMultipleCoils},	// д�����Ȧ
#endif
#if WRITE_MULT_REG_EN > 0
	{0x10,FWriteMultipleReg},	// д����Ĵ���
#endif
#if MASK_WRITE_REG_EN > 0
	{0x16,FMaskWriteReg},		// �������μĴ���ָ��
#endif
#if READ_WRITE_REG_EN > 0
	{0x17,FRWMultipleReg},		// ��д����Ĵ���
#endif
	{0x00,FTest}				// ���Թ��ܴ���
};


#if READ_DIS_INPUT_EN
//************************************************************************************
// �������ƣ�ReadDiscreteInputs
// ���������CoilsDataPtr�������ɢ��������ָ��
//			 StAddr,��ɢ����ʼ��ַ
//			 Quantity,��ɢ������
// �����������������TRUE,������FALSE��
// ��������������ɢ�����������
//************************************************************************************	
uint8_t ReadDiscreteInputs(uint8_t *CoilsDataPtr,uint16_t StAddr ,uint16_t Quantity)
{
	uint8_t *CoilsInByte;
	uint8_t offset;
	uint16_t i=0 ;
	CoilsInByte = CoilsDataPtr;
	while(i<Quantity)
	{	
		*CoilsInByte = 0;
		for(offset=0;offset<8;offset++,i++)		// ����Ȧֵд��һ���ֽ�
		{
			if(i<Quantity)
			{
				*CoilsInByte |= MB_GetDiscrete(StAddr+i)<<offset;
			}
		}
		CoilsInByte++;
	}
	return TRUE;
}
#endif

#if READ_COILS_EN
//************************************************************************************
// �������ƣ�ReadCoils
// ���������CoilsDataPtr�������Ȧ����ָ��
//			 StAddr,��Ȧ����ʼ��ַ
//			 Quantity,��Ȧ������
// �����������������TRUE,������FALSE��
// ����������������Ȧ��������
//************************************************************************************	
uint8_t ReadCoils(uint8_t *CoilsDataPtr,uint16_t StAddr ,uint16_t Quantity)
{
	uint8_t *CoilsInByte;
	uint8_t offset;
	uint16_t i=0 ;
	CoilsInByte = CoilsDataPtr;
	while(i<Quantity)
	{	
		*CoilsInByte = 0;
		for(offset=0;offset<8;offset++,i++)		// ����Ȧֵд��һ���ֽ�
		{
			if(i<Quantity)
			{
				*CoilsInByte |= MB_GetCoils(StAddr+i)<<offset;
			}
		}
		CoilsInByte++;
	}
	return TRUE;
}
#endif


#if WRITE_SING_COIL_EN
//************************************************************************************
// �������ƣ�WriteSingleCoil
// ���������DataPtr,����ָ�룬
//			 StAddr���Ĵ�����ʼ��ַ
//			 Quantity���Ĵ���������
// �����������������TRUE,������FALSE��
// ����������д����Ȧ������
//************************************************************************************	
uint8_t WriteSingleCoil(uint16_t OutputAddr,uint16_t OutputValue)
{
	return MB_SetCoil(OutputAddr,(OutputValue==0xff00)?1:0);
}
#endif


#if WRITE_MULT_COIL_EN
//************************************************************************************
// �������ƣ�WriteMultipleCoils
// ���������DataPtr,����ָ�룬
//			 StAddr���Ĵ�����ʼ��ַ
//			 Quantity���Ĵ���������
// �����������������TRUE,������FALSE��
// ����������д����Ȧ������
//************************************************************************************	
uint8_t WriteMultipleCoils(uint8_t *DataPtr,uint16_t StAddr ,uint16_t Quantity)
{
	uint16_t i;
	uint8_t 	CoilInByte;
	for(i=0;i<Quantity;i++)
	{
		if((i%8)==0)	
			CoilInByte = *(DataPtr+(i/8));
		if( FALSE == MB_SetCoil(StAddr+i,(CoilInByte>>(i%8))&0x01))
			return FALSE;	
	}
	return TRUE;
}
#endif

#if READ_INPUT_REG_EN
//************************************************************************************
// �������ƣ�ReadInputReg
// ���������DataPtr,����ָ�룬
//			 StAddr���Ĵ�����ʼ��ַ
//			 Quantity���Ĵ���������
// �����������������TRUE,������FALSE��
// ����������������Ĵ���
//************************************************************************************	
uint8_t ReadInputReg(uint8_t *DataPtr,uint16_t StAddr ,uint16_t Quantity)
{
	uint8_t i;
	uint16_t	Value;
	for(i=0;i<Quantity;i++)
	{
		Value = MB_GetInputRegValue(StAddr+i);
		*(DataPtr+i*2)   = (uint8_t)(Value>>8); 		// �ȴ���λ
		*(DataPtr+i*2+1) = (uint8_t)Value;  			// �󴫵�λ
	}
	return TRUE;
}
#endif

#if (READ_HOLD_REG_EN | READ_WRITE_REG_EN)
//************************************************************************************
// �������ƣ�ReadHoldingReg
// ���������DataPtr,����ָ�룬16λ���ݸ�8λ�ڵ��ֽڣ���8λ�ڸ��ֽ�
//			 StAddr���Ĵ�����ʼ��ַ
//			 Quantity���Ĵ���������
// �����������������TRUE,������FALSE��
// �������������Ĵ���������
//************************************************************************************	
uint8_t ReadHoldingReg(uint8_t *DataPtr,uint16_t StAddr ,uint16_t Quantity)
{
	uint8_t i;
	uint16_t	Value;
	for(i=0;i<Quantity;i++)
	{
		Value = MB_GetRegValue(StAddr+i);
		*(DataPtr+i*2)   = (uint8_t)(Value>>8); 	// �ȴ���λ
		*(DataPtr+i*2+1) = (uint8_t)Value;  		// �󴫵�λ
	}
	return TRUE;
}
#endif

#if (WRITE_SING_REG_EN | WRITE_MULT_REG_EN  |\
	 MASK_WRITE_REG_EN | READ_WRITE_REG_EN  )
//************************************************************************************
// �������ƣ�WriteHoldingReg
// ���������DataPtr,����ָ�룬16λ���ݸ�8λ�ڵ��ֽڣ���8λ�ڸ��ֽ�
//			 StAddr���Ĵ�����ʼ��ַ
//			 Quantity���Ĵ���������
// �����������������TRUE,������FALSE��
// ����������д�Ĵ���������
//************************************************************************************	
uint8_t WriteHoldingReg(uint8_t *KeepDataPtr,uint16_t StAddr ,uint16_t Quantity)
{
	uint8_t i;
	uint16_t	Value;
	for(i=0;i<Quantity;i++)
	{
		Value = (*(KeepDataPtr+i*2)<<8)|(*(KeepDataPtr+i*2+1));
		MB_SetRegValue(StAddr+i,Value);
	}
	return TRUE;
}
#endif











/****************************************************************************************/
// �������ƣ�FReadCoils
// ���������Response��ΪPDU_RESPONSE���͵�ָ�룬PDU_RESPONSE��
//			 typedef struct __PDU_RESPONSE{
//				uint8*	PDUDataPtr;   ��������ָ��,���ܴ��봦�������轫��������ݰ����ڸ�ָ��Ŀ�ʼλʼ
//			  	uint8	PDUByteLength;��������������
//				uint8	ExceptionCode;��Ϊ������쳣���룬����������Ϊ0
//			 }PDU_RESPONSE
// �����������
// �������������������Ȧָ��01
/***************************************************************************************/
#if READ_COILS_EN >0
void FReadCoils(PDU_RESPONSE *Response)
{
	uint8_t * PDUPtr;
	uint16_t StAddr ,Quantity;
	// ������PDU����ȡ��ز���
	PDUPtr = Response->PDUDataPtr;
	StAddr = PDUPtr[1]<<8|PDUPtr[2];
	Quantity =  PDUPtr[3]<<8|PDUPtr[4];
	
	Response->PDUByteLength = 2 + Quantity/8+((Quantity%8)?1:0) ;	// �������ص����ݳ���
	
	if((Quantity>=0x001)&&(Quantity<=0x07d0))
	{
		if((StAddr<END_COILS_ADDR)&&((StAddr+Quantity)<=END_COILS_ADDR))
		{			
			if(ReadCoils(PDUPtr+2,StAddr,Quantity))
			{
				*(PDUPtr+1) = Quantity/8+(Quantity%8)?1:0;	// �ֽ���
				Response->ExceptionCode = 0x00;
			}
			else
				Response->ExceptionCode = 0x04;
		}
		else
			Response->ExceptionCode = 0x02;
	}
	else
		Response->ExceptionCode = 0x03;
}
#endif

/****************************************************************************************/
// �������ƣ�FReadDiscreteInputs
// ���������Response
// �����������
// �����������������ɢ����ָ��02
/***************************************************************************************/
#if READ_DIS_INPUT_EN >0
void FReadDiscreteInputs(PDU_RESPONSE *Response)
{
	uint8_t * PDUPtr;
	uint16_t StAddr ,Quantity;
	// ������PDU����ȡ��ز���
	PDUPtr = Response->PDUDataPtr;
	StAddr = PDUPtr[1]<<8|PDUPtr[2];
	Quantity =  PDUPtr[3]<<8|PDUPtr[4];
	
	Response->PDUByteLength = 2 + Quantity/8+((Quantity%8)?1:0) ;	// �������ص����ݳ���
	
	if((Quantity>=0x001)&&(Quantity<=0x07d0))
	{
		if((StAddr<END_DISC_INPUT_ADDR)&&((StAddr+Quantity)<=END_DISC_INPUT_ADDR))
		{			
			if(ReadDiscreteInputs(PDUPtr+2,StAddr,Quantity))
			{
				*(PDUPtr+1) = Quantity/8+(Quantity%8)?1:0;	// �ֽ���
				Response->ExceptionCode = 0x00;
			}
			else
				Response->ExceptionCode = 0x04;
		}
		else
			Response->ExceptionCode = 0x02;
	}
	else
		Response->ExceptionCode = 0x03;
}
#endif

/****************************************************************************************/
// �������ƣ�FReadHoldingReg
// ���������Response
// �����������
// ������������������ּĴ���ָ��03
/****************************************************************************************/
#if READ_HOLD_REG_EN >0
void FReadHoldingReg(PDU_RESPONSE *Response)
{
	uint8_t * PDUPtr;
	uint16_t StAddr ,Quantity;
	// ������PDU����ȡ��ز���
	PDUPtr = Response->PDUDataPtr;
	StAddr = PDUPtr[1]<<8|PDUPtr[2];
	Quantity =  PDUPtr[3]<<8|PDUPtr[4];
	
	Response->PDUByteLength = 2 + Quantity*2 ;	// �������ص����ݳ���
	
	if((Quantity>=0x001)&&(Quantity<=0x007d))
	{
		if((StAddr<END_HOLDING_REG_ADDR)&&((StAddr+Quantity)<=END_HOLDING_REG_ADDR))
		{			
			if(ReadHoldingReg(PDUPtr+2,StAddr,Quantity))
			{
				*(PDUPtr+1) = Quantity*2;
				Response->ExceptionCode = 0x00;
			}
			else
				Response->ExceptionCode = 0x04;
		}
		else
			Response->ExceptionCode = 0x02;
	}
	else
		Response->ExceptionCode = 0x03;
}
#endif

/****************************************************************************************/
// �������ƣ�FReadInputReg
// ���������Response
// �����������
// �������������������Ĵ���ָ��04
/****************************************************************************************/
#if READ_INPUT_REG_EN >0
void FReadInputReg(PDU_RESPONSE *Response)
{
	uint8_t * PDUPtr;
	uint16_t StAddr ,Quantity;
	// ������PDU����ȡ��ز���
	PDUPtr = Response->PDUDataPtr;
	StAddr = PDUPtr[1]<<8|PDUPtr[2];
	Quantity =  PDUPtr[3]<<8|PDUPtr[4];
	
	Response->PDUByteLength = 2 + Quantity*2 ;	// �������ص����ݳ���
	
	if((Quantity>=0x001)&&(Quantity<=0x007d))
	{
		if((StAddr<END_INPUT_REG_ADDR) && ((StAddr+Quantity)<=END_INPUT_REG_ADDR))
		{			
			if(ReadInputReg(PDUPtr+2,StAddr,Quantity))
			{
				*(PDUPtr+1) = Quantity*2;
				Response->ExceptionCode = 0x00;
			}
			else
				Response->ExceptionCode = 0x04;
		}
		else
			Response->ExceptionCode = 0x02;
	}
	else
		Response->ExceptionCode = 0x03;
}
#endif

/****************************************************************************************/
// �������ƣ�FWriteSingleCoil
// ���������Response
// �����������
// ��������������д����Ȧָ��05
/****************************************************************************************/
#if WRITE_SING_COIL_EN >0
void FWriteSingleCoil(PDU_RESPONSE *Response)
{
	uint8_t * PDUPtr;
	uint16_t OutputAddr ,OutputValue;
	// ������PDU����ȡ��ز���
	PDUPtr = Response->PDUDataPtr;
	OutputAddr = PDUPtr[1]<<8|PDUPtr[2];
	OutputValue =  PDUPtr[3]<<8|PDUPtr[4];
	
	Response->PDUByteLength = 5 ;	// �������ص����ݳ���
	
	if((OutputValue==0x0000)||(OutputValue==0xff00))
	{
		if( OutputAddr < END_COILS_ADDR )
		{			
			if(WriteSingleCoil(OutputAddr,OutputValue))
			{
				*(PDUPtr+1) = OutputAddr>>8;
				*(PDUPtr+2) = OutputAddr;
				*(PDUPtr+3)	= OutputValue>>8;
				*(PDUPtr+4)	= OutputValue;
				Response->ExceptionCode = 0x00;
			}
			else
				Response->ExceptionCode = 0x04;
		}
		else
			Response->ExceptionCode = 0x02;
	}
	else
		Response->ExceptionCode = 0x03;
}
#endif

/****************************************************************************************/
// �������ƣ�FWriteSingleReg
// ���������Response
// �����������
// ��������������д����Ȧָ��06
/****************************************************************************************/
#if WRITE_SING_REG_EN >0
void FWriteSingleReg(PDU_RESPONSE *Response)
{
	uint8_t * PDUPtr;
	uint16_t OutputAddr ,OutputValue;
	// ������PDU����ȡ��ز���
	PDUPtr = Response->PDUDataPtr;
	OutputAddr = PDUPtr[1]<<8|PDUPtr[2];
	OutputValue =  PDUPtr[3]<<8|PDUPtr[4];
	
	//Response->PDUByteLength = 2 + 5 ;	// �������ص����ݳ���
	Response->PDUByteLength = 5 ;	// �������ص����ݳ���
	if(OutputValue<=0xffff)
	{
		if(OutputAddr<END_HOLDING_REG_ADDR)
		{			
			if(WriteHoldingReg(&PDUPtr[3],OutputAddr,1))
			{
				*(PDUPtr+1) = OutputAddr>>8;
				*(PDUPtr+2) = OutputAddr;
				*(PDUPtr+3)	= OutputValue>>8;
				*(PDUPtr+4)	= OutputValue;
				Response->ExceptionCode = 0x00;
			}
			else
				Response->ExceptionCode = 0x04;
		}
		else
			Response->ExceptionCode = 0x02;
	}
	else
		Response->ExceptionCode = 0x03;
}
#endif

/****************************************************************************************/
// �������ƣ�FWriteMultipleCoils
// ���������Response
// �����������
// ��������������д����Ȧָ��15
/****************************************************************************************/
#if WRITE_MULT_COIL_EN >0
void FWriteMultipleCoils(PDU_RESPONSE *Response)
{
	uint8_t * PDUPtr;
	uint16_t StAddr ,Quantity;
	uint8_t 	ByteCount;
	// ������PDU����ȡ��ز���
	PDUPtr = Response->PDUDataPtr;
	StAddr = PDUPtr[1]<<8|PDUPtr[2];
	Quantity =  PDUPtr[3]<<8|PDUPtr[4];
	ByteCount = PDUPtr[5];
	
	Response->PDUByteLength = 5 ;	// �������ص����ݳ���

	if((Quantity>=0x001)&&(Quantity<=0x07d0)&&(ByteCount==(Quantity/8+((Quantity%8)?1:0))))
	{
		if((StAddr<END_COILS_ADDR)&&((StAddr+Quantity)<=(END_COILS_ADDR+1)))
		{			
			if(WriteMultipleCoils(PDUPtr+6,StAddr,Quantity))
			{
				*(PDUPtr+1) = StAddr>>8;
				*(PDUPtr+2) = StAddr;
				*(PDUPtr+3) = Quantity>>8;
				*(PDUPtr+4) = Quantity;
				Response->ExceptionCode = 0x00;
			}
			else
				Response->ExceptionCode = 0x04;
		}
		else
			Response->ExceptionCode = 0x02;
	}
	else
		Response->ExceptionCode = 0x03;
}
#endif

/****************************************************************************************/
// �������ƣ�FWriteMultipleReg
// ���������Response
// �����������
// ��������������д��Ĵ���ָ��16
/****************************************************************************************/
#if WRITE_MULT_REG_EN >0
void FWriteMultipleReg(PDU_RESPONSE *Response)
{
	uint8_t * PDUPtr;
	uint16_t StAddr ,Quantity;
	uint8_t 	ByteCount;
	// ������PDU����ȡ��ز���
	PDUPtr = Response->PDUDataPtr;
	StAddr = PDUPtr[1]<<8|PDUPtr[2];
	Quantity =  PDUPtr[3]<<8|PDUPtr[4];
	ByteCount = PDUPtr[5];
	
	Response->PDUByteLength = 5 ;	// �������ص����ݳ���

	if((Quantity>=0x001)&&(Quantity<=0x007d)&&(ByteCount==(Quantity*2)))
	{
		if((StAddr<END_HOLDING_REG_ADDR) && ((StAddr+Quantity)<=(END_HOLDING_REG_ADDR+1)))
		{			
			if(WriteHoldingReg(PDUPtr+6,StAddr,Quantity))
			{
				*(PDUPtr+1) = StAddr>>8;
				*(PDUPtr+2) = StAddr;
				*(PDUPtr+3) = Quantity>>8;
				*(PDUPtr+4) = Quantity;
				Response->ExceptionCode = 0x00;
			}
			else
				Response->ExceptionCode = 0x04;
		}
		else
			Response->ExceptionCode = 0x02;
	}
	else
		Response->ExceptionCode = 0x03;
}
#endif

/****************************************************************************************/
// �������ƣ�FMaskWriteReg
// ���������Response
// �����������
// �����������������μĴ���ָ��22
/****************************************************************************************/
#if MASK_WRITE_REG_EN >0
void FMaskWriteReg(PDU_RESPONSE *Response)
{
	uint16_t RefeAddr,And_Mask,Or_Mask,RegValue;
	uint8_t  *PDUPtr;
	uint8_t  ValueInByte[2];		
	PDUPtr = Response->PDUDataPtr;
	RefeAddr = (PDUPtr[1]<<8)|(PDUPtr[2]);
	And_Mask = (PDUPtr[3]<<8)|(PDUPtr[4]);
	Or_Mask  = (PDUPtr[5]<<8)|(PDUPtr[6]);
	
	Response->PDUByteLength = 7;
	
	if(RefeAddr<END_HOLDING_REG_ADDR)
	{
		if(ReadHoldingReg(ValueInByte,RefeAddr,1))
		{	
			RegValue = (ValueInByte[0]<<8)|(ValueInByte[1]);
			RegValue = (RegValue & And_Mask)|(Or_Mask & (~And_Mask));
			ValueInByte[0] = RegValue >> 8;
			ValueInByte[1] = RegValue ;
			if(WriteHoldingReg(ValueInByte,RefeAddr,1))
			{
				*(PDUPtr+1) = RefeAddr>>8;
				*(PDUPtr+2) = RefeAddr;
				*(PDUPtr+3) = And_Mask>>8;
				*(PDUPtr+4) = And_Mask;
				*(PDUPtr+5) = Or_Mask>>8;
				*(PDUPtr+6) = Or_Mask;
				Response->ExceptionCode = 0x00;
			}
			else
				Response->ExceptionCode = 0x04;
		}
		else
			Response->ExceptionCode = 0x03;
	}
	else
		Response->ExceptionCode = 0x02;
}
#endif

/****************************************************************************************/
// �������ƣ�FRWMultipleReg
// ���������Response
// �����������
// ���������������д��Ĵ���ָ��23
/****************************************************************************************/
#if READ_WRITE_REG_EN > 0
void FRWMultipleReg(PDU_RESPONSE *Response)
{
	uint16_t ReadStaAddr,	ReadQuantity;
	uint16_t WriteStaAddr,WriteQuantity;
	uint8_t  WriteBytes;
	uint8_t  *WriteValuePtr;
	uint8_t  *PDUPtr;
	PDUPtr = Response->PDUDataPtr;
	ReadStaAddr   = (PDUPtr[1]<<8)|(PDUPtr[2]);
	ReadQuantity  = (PDUPtr[3]<<8)|(PDUPtr[4]);
	WriteStaAddr  = (PDUPtr[5]<<8)|(PDUPtr[6]);
	WriteQuantity = (PDUPtr[7]<<8)|(PDUPtr[8]);
	WriteBytes    = PDUPtr[9];
	WriteValuePtr = &PDUPtr[10];

	Response->PDUByteLength = ReadQuantity*2 + 2;
	
	if((ReadQuantity>=0x001)&&(ReadQuantity<=0x007d)&&		// ���Ĵ����������Ƿ�����Ч��Χ��
	   (WriteQuantity>=0x001)&&(WriteQuantity<=0x0079)&&	// д�Ĵ����������Ƿ�����Ч��Χ��
	   (WriteBytes==(WriteQuantity*2)))						// ���յ����ֽ����Ƿ���д��ļĴ�������һ��
	{
		if((ReadStaAddr<END_HOLDING_REG_ADDR)&&((ReadStaAddr+ReadQuantity)<=END_HOLDING_REG_ADDR)&&
		   (WriteStaAddr<END_HOLDING_REG_ADDR)&&((WriteStaAddr+WriteQuantity)<=END_HOLDING_REG_ADDR))
		{
			if(WriteHoldingReg(WriteValuePtr,WriteStaAddr,WriteQuantity))	// д�Ĵ���
			{
			   if(ReadHoldingReg(PDUPtr+2,ReadStaAddr,ReadQuantity))		// ���Ĵ���
				{
					*(PDUPtr+1) = ReadQuantity*2;
					Response->ExceptionCode = 0x00;
				}
				else 
					Response->ExceptionCode = 0x04;
			}
			else
				Response->ExceptionCode = 0x04;
		}
		else
			Response->ExceptionCode = 0x02;
	}
	else
		Response->ExceptionCode = 0x03;
}
#endif












