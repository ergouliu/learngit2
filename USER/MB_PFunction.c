/****************************************Copyright (c)**************************************************
**                               广州周立功单片机发展有限公司
**                                     研    究    所
**                                        产品一部 
**
**                                 http://www.zlgmcu.com
**
**--------------文件信息--------------------------------------------------------------------------------
**文   件   名: MB_PFunction.c
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
#include	"modbus_config.h"
#include	"modbus.h"
#include    "MBSlave.h"


//**************************************************
// 函数名称：MB_GetDiscrete
// 输入参数：Address，线圈地址
// 输出参数：返回线圈值（0\1）
// 功能描述：获取离散值函数，访函数由用户编写
//**************************************************
uint8_t MB_GetDiscrete(uint16_t Address);

//**************************************************
// 函数名称：MB_GetCoils
// 输入参数：Address，线圈地址
// 输出参数：返回线圈值（0\1）
// 功能描述：获取线圈值函数，访函数由用户编写
//**************************************************
uint8_t MB_GetCoils(uint16_t Address);

//**************************************************
// 函数名称：MB_SetCoil
// 输入参数：Address，线圈地址
//			 CoilValue,线圈值（0\1）
// 输出参数：返回寄存器值
// 功能描述：设置线圈值函数，访函数由用户编写
//**************************************************
uint8_t MB_SetCoil(uint16_t Address,uint8_t CoilValue);


//**************************************************
// 函数名称：MB_GetInputRegValue
// 输入参数：Address，寄存器地址
// 输出参数：返回寄存器值
// 功能描述：读输入寄存器值函数，访函数由用户编写
//**************************************************
uint16_t MB_GetInputRegValue(uint16_t Address);

//**************************************************
// 函数名称：MB_GetRegValue
// 输入参数：Address，寄存器地址
// 输出参数：返回寄存器值
// 功能描述：读保持寄存器值函数，访函数由用户编写
//**************************************************
uint16_t MB_GetRegValue(uint16_t Address);

//**************************************************
// 函数名称：MB_SetRegValue
// 输入参数：Address，寄存器地址
//			 Value，写入的值
// 输出参数：无
// 功能描述：写保持寄存器值函数，访函数由用户编写
//**************************************************
void MB_SetRegValue(uint16_t Address,uint16_t Value);

/****************************************************************************************/
// 函数名称：FReadCoils
// 输入参数：Response
// 输出参数：无
// 功能描述：处理读单线圈指令01
/***************************************************************************************/
void FReadCoils(PDU_RESPONSE *Response);
/****************************************************************************************/
// 函数名称：FReadDiscreteInputs
// 输入参数：Response
// 输出参数：无
// 功能描述：处理读离散输入指令02
/***************************************************************************************/
void FReadDiscreteInputs(PDU_RESPONSE *Response);
/****************************************************************************************/
// 函数名称：FReadHoldingReg
// 输入参数：Response
// 输出参数：无
// 功能描述：处理读保持寄存器指令03
/****************************************************************************************/
void FReadHoldingReg(PDU_RESPONSE *Response);
/****************************************************************************************/
// 函数名称：FReadInputReg
// 输入参数：Response
// 输出参数：无
// 功能描述：处理读输入寄存器指令04
/****************************************************************************************/
void FReadInputReg(PDU_RESPONSE *Response);
/****************************************************************************************/
// 函数名称：FWriteSingleCoil
// 输入参数：Response
// 输出参数：无
// 功能描述：处理写单线圈指令05
/****************************************************************************************/
void FWriteSingleCoil(PDU_RESPONSE *Response);
/****************************************************************************************/
// 函数名称：FWriteSingleReg
// 输入参数：Response
// 输出参数：无
// 功能描述：处理写单线圈指令06
/****************************************************************************************/
void FWriteSingleReg(PDU_RESPONSE *Response);
/****************************************************************************************/
// 函数名称：FWriteMultipleCoils
// 输入参数：Response
// 输出参数：无
// 功能描述：处理写多线圈指令15
/****************************************************************************************/
void FWriteMultipleCoils(PDU_RESPONSE *Response);
/****************************************************************************************/
// 函数名称：FWriteMultipleReg
// 输入参数：Response
// 输出参数：无
// 功能描述：处理写多寄存器指令16
/****************************************************************************************/
void FWriteMultipleReg(PDU_RESPONSE *Response);
/****************************************************************************************/
// 函数名称：FMaskWriteReg
// 输入参数：Response
// 输出参数：无
// 功能描述：处理屏蔽寄存器指令22
/****************************************************************************************/
void FMaskWriteReg(PDU_RESPONSE *Response);
/****************************************************************************************/
// 函数名称：FRWMultipleReg
// 输入参数：Response
// 输出参数：无
// 功能描述：处理读写多寄存器指令23
/****************************************************************************************/
void FRWMultipleReg(PDU_RESPONSE *Response);

void FTest(PDU_RESPONSE *Response){}

AT_CODE FUNCTION_ARRAY	FCodeHandleArray[MAX_FUNCTION]={
#if READ_COILS_EN > 0 
	{0x01,FReadCoils},			// 读线圈
#endif
#if READ_DIS_INPUT_EN > 0
	{0x02,FReadDiscreteInputs},	// 读离散量输入
#endif
#if READ_HOLD_REG_EN > 0
	{0x03,FReadHoldingReg},		// 读保持寄存器
#endif
#if READ_INPUT_REG_EN > 0
	{0x04,FReadInputReg},		// 读输入寄存器
#endif
#if WRITE_SING_COIL_EN > 0
	{0x05,FWriteSingleCoil},	// 写单个线圈
#endif
#if WRITE_SING_REG_EN > 0
	{0x06,FWriteSingleReg},		// 写单个寄存器
#endif
#if WRITE_MULT_COIL_EN > 0
	{0x0f,FWriteMultipleCoils},	// 写多个线圈
#endif
#if WRITE_MULT_REG_EN > 0
	{0x10,FWriteMultipleReg},	// 写多个寄存器
#endif
#if MASK_WRITE_REG_EN > 0
	{0x16,FMaskWriteReg},		// 处理屏蔽寄存器指令
#endif
#if READ_WRITE_REG_EN > 0
	{0x17,FRWMultipleReg},		// 读写多个寄存器
#endif
	{0x00,FTest}				// 测试功能代码
};


#if READ_DIS_INPUT_EN
//************************************************************************************
// 函数名称：ReadDiscreteInputs
// 输入参数：CoilsDataPtr，存放离散输入数据指针
//			 StAddr,离散的起始地址
//			 Quantity,离散的数量
// 输出参数：正常返回TRUE,出错返回FALSE。
// 功能描述：读离散输入操作函数
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
		for(offset=0;offset<8;offset++,i++)		// 将线圈值写入一个字节
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
// 函数名称：ReadCoils
// 输入参数：CoilsDataPtr，存放线圈数据指针
//			 StAddr,线圈的起始地址
//			 Quantity,线圈的数量
// 输出参数：正常返回TRUE,出错返回FALSE。
// 功能描述：读多线圈操作函数
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
		for(offset=0;offset<8;offset++,i++)		// 将线圈值写入一个字节
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
// 函数名称：WriteSingleCoil
// 输入参数：DataPtr,数据指针，
//			 StAddr，寄存器起始地址
//			 Quantity，寄存器数量。
// 输出参数：正常返回TRUE,出错返回FALSE。
// 功能描述：写单线圈处理函数
//************************************************************************************	
uint8_t WriteSingleCoil(uint16_t OutputAddr,uint16_t OutputValue)
{
	return MB_SetCoil(OutputAddr,(OutputValue==0xff00)?1:0);
}
#endif


#if WRITE_MULT_COIL_EN
//************************************************************************************
// 函数名称：WriteMultipleCoils
// 输入参数：DataPtr,数据指针，
//			 StAddr，寄存器起始地址
//			 Quantity，寄存器数量。
// 输出参数：正常返回TRUE,出错返回FALSE。
// 功能描述：写多线圈处理函数
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
// 函数名称：ReadInputReg
// 输入参数：DataPtr,数据指针，
//			 StAddr，寄存器起始地址
//			 Quantity，寄存器数量。
// 输出参数：正常返回TRUE,出错返回FALSE。
// 功能描述：读输入寄存器
//************************************************************************************	
uint8_t ReadInputReg(uint8_t *DataPtr,uint16_t StAddr ,uint16_t Quantity)
{
	uint8_t i;
	uint16_t	Value;
	for(i=0;i<Quantity;i++)
	{
		Value = MB_GetInputRegValue(StAddr+i);
		*(DataPtr+i*2)   = (uint8_t)(Value>>8); 		// 先传高位
		*(DataPtr+i*2+1) = (uint8_t)Value;  			// 后传低位
	}
	return TRUE;
}
#endif

#if (READ_HOLD_REG_EN | READ_WRITE_REG_EN)
//************************************************************************************
// 函数名称：ReadHoldingReg
// 输入参数：DataPtr,数据指针，16位数据高8位在低字节，低8位在高字节
//			 StAddr，寄存器起始地址
//			 Quantity，寄存器数量。
// 输出参数：正常返回TRUE,出错返回FALSE。
// 功能描述：读寄存器处理函数
//************************************************************************************	
uint8_t ReadHoldingReg(uint8_t *DataPtr,uint16_t StAddr ,uint16_t Quantity)
{
	uint8_t i;
	uint16_t	Value;
	for(i=0;i<Quantity;i++)
	{
		Value = MB_GetRegValue(StAddr+i);
		*(DataPtr+i*2)   = (uint8_t)(Value>>8); 	// 先传高位
		*(DataPtr+i*2+1) = (uint8_t)Value;  		// 后传低位
	}
	return TRUE;
}
#endif

#if (WRITE_SING_REG_EN | WRITE_MULT_REG_EN  |\
	 MASK_WRITE_REG_EN | READ_WRITE_REG_EN  )
//************************************************************************************
// 函数名称：WriteHoldingReg
// 输入参数：DataPtr,数据指针，16位数据高8位在低字节，低8位在高字节
//			 StAddr，寄存器起始地址
//			 Quantity，寄存器数量。
// 输出参数：正常返回TRUE,出错返回FALSE。
// 功能描述：写寄存器处理函数
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
// 函数名称：FReadCoils
// 输入参数：Response，为PDU_RESPONSE类型的指针，PDU_RESPONSE：
//			 typedef struct __PDU_RESPONSE{
//				uint8*	PDUDataPtr;   请求数据指针,功能代码处理函数必需将处理的数据包存在该指针的开始位始
//			  	uint8	PDUByteLength;请求数据数长度
//				uint8	ExceptionCode;仅为输出错异常代码，正常操作设为0
//			 }PDU_RESPONSE
// 输出参数：无
// 功能描述：处理读多线圈指令01
/***************************************************************************************/
#if READ_COILS_EN >0
void FReadCoils(PDU_RESPONSE *Response)
{
	uint8_t * PDUPtr;
	uint16_t StAddr ,Quantity;
	// 从请求PDU中提取相关参数
	PDUPtr = Response->PDUDataPtr;
	StAddr = PDUPtr[1]<<8|PDUPtr[2];
	Quantity =  PDUPtr[3]<<8|PDUPtr[4];
	
	Response->PDUByteLength = 2 + Quantity/8+((Quantity%8)?1:0) ;	// 正常返回的数据长度
	
	if((Quantity>=0x001)&&(Quantity<=0x07d0))
	{
		if((StAddr<END_COILS_ADDR)&&((StAddr+Quantity)<=END_COILS_ADDR))
		{			
			if(ReadCoils(PDUPtr+2,StAddr,Quantity))
			{
				*(PDUPtr+1) = Quantity/8+(Quantity%8)?1:0;	// 字节数
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
// 函数名称：FReadDiscreteInputs
// 输入参数：Response
// 输出参数：无
// 功能描述：处理读离散输入指令02
/***************************************************************************************/
#if READ_DIS_INPUT_EN >0
void FReadDiscreteInputs(PDU_RESPONSE *Response)
{
	uint8_t * PDUPtr;
	uint16_t StAddr ,Quantity;
	// 从请求PDU中提取相关参数
	PDUPtr = Response->PDUDataPtr;
	StAddr = PDUPtr[1]<<8|PDUPtr[2];
	Quantity =  PDUPtr[3]<<8|PDUPtr[4];
	
	Response->PDUByteLength = 2 + Quantity/8+((Quantity%8)?1:0) ;	// 正常返回的数据长度
	
	if((Quantity>=0x001)&&(Quantity<=0x07d0))
	{
		if((StAddr<END_DISC_INPUT_ADDR)&&((StAddr+Quantity)<=END_DISC_INPUT_ADDR))
		{			
			if(ReadDiscreteInputs(PDUPtr+2,StAddr,Quantity))
			{
				*(PDUPtr+1) = Quantity/8+(Quantity%8)?1:0;	// 字节数
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
// 函数名称：FReadHoldingReg
// 输入参数：Response
// 输出参数：无
// 功能描述：处理读保持寄存器指令03
/****************************************************************************************/
#if READ_HOLD_REG_EN >0
void FReadHoldingReg(PDU_RESPONSE *Response)
{
	uint8_t * PDUPtr;
	uint16_t StAddr ,Quantity;
	// 从请求PDU中提取相关参数
	PDUPtr = Response->PDUDataPtr;
	StAddr = PDUPtr[1]<<8|PDUPtr[2];
	Quantity =  PDUPtr[3]<<8|PDUPtr[4];
	
	Response->PDUByteLength = 2 + Quantity*2 ;	// 正常返回的数据长度
	
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
// 函数名称：FReadInputReg
// 输入参数：Response
// 输出参数：无
// 功能描述：处理读输入寄存器指令04
/****************************************************************************************/
#if READ_INPUT_REG_EN >0
void FReadInputReg(PDU_RESPONSE *Response)
{
	uint8_t * PDUPtr;
	uint16_t StAddr ,Quantity;
	// 从请求PDU中提取相关参数
	PDUPtr = Response->PDUDataPtr;
	StAddr = PDUPtr[1]<<8|PDUPtr[2];
	Quantity =  PDUPtr[3]<<8|PDUPtr[4];
	
	Response->PDUByteLength = 2 + Quantity*2 ;	// 正常返回的数据长度
	
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
// 函数名称：FWriteSingleCoil
// 输入参数：Response
// 输出参数：无
// 功能描述：处理写单线圈指令05
/****************************************************************************************/
#if WRITE_SING_COIL_EN >0
void FWriteSingleCoil(PDU_RESPONSE *Response)
{
	uint8_t * PDUPtr;
	uint16_t OutputAddr ,OutputValue;
	// 从请求PDU中提取相关参数
	PDUPtr = Response->PDUDataPtr;
	OutputAddr = PDUPtr[1]<<8|PDUPtr[2];
	OutputValue =  PDUPtr[3]<<8|PDUPtr[4];
	
	Response->PDUByteLength = 5 ;	// 正常返回的数据长度
	
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
// 函数名称：FWriteSingleReg
// 输入参数：Response
// 输出参数：无
// 功能描述：处理写单线圈指令06
/****************************************************************************************/
#if WRITE_SING_REG_EN >0
void FWriteSingleReg(PDU_RESPONSE *Response)
{
	uint8_t * PDUPtr;
	uint16_t OutputAddr ,OutputValue;
	// 从请求PDU中提取相关参数
	PDUPtr = Response->PDUDataPtr;
	OutputAddr = PDUPtr[1]<<8|PDUPtr[2];
	OutputValue =  PDUPtr[3]<<8|PDUPtr[4];
	
	//Response->PDUByteLength = 2 + 5 ;	// 正常返回的数据长度
	Response->PDUByteLength = 5 ;	// 正常返回的数据长度
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
// 函数名称：FWriteMultipleCoils
// 输入参数：Response
// 输出参数：无
// 功能描述：处理写多线圈指令15
/****************************************************************************************/
#if WRITE_MULT_COIL_EN >0
void FWriteMultipleCoils(PDU_RESPONSE *Response)
{
	uint8_t * PDUPtr;
	uint16_t StAddr ,Quantity;
	uint8_t 	ByteCount;
	// 从请求PDU中提取相关参数
	PDUPtr = Response->PDUDataPtr;
	StAddr = PDUPtr[1]<<8|PDUPtr[2];
	Quantity =  PDUPtr[3]<<8|PDUPtr[4];
	ByteCount = PDUPtr[5];
	
	Response->PDUByteLength = 5 ;	// 正常返回的数据长度

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
// 函数名称：FWriteMultipleReg
// 输入参数：Response
// 输出参数：无
// 功能描述：处理写多寄存器指令16
/****************************************************************************************/
#if WRITE_MULT_REG_EN >0
void FWriteMultipleReg(PDU_RESPONSE *Response)
{
	uint8_t * PDUPtr;
	uint16_t StAddr ,Quantity;
	uint8_t 	ByteCount;
	// 从请求PDU中提取相关参数
	PDUPtr = Response->PDUDataPtr;
	StAddr = PDUPtr[1]<<8|PDUPtr[2];
	Quantity =  PDUPtr[3]<<8|PDUPtr[4];
	ByteCount = PDUPtr[5];
	
	Response->PDUByteLength = 5 ;	// 正常返回的数据长度

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
// 函数名称：FMaskWriteReg
// 输入参数：Response
// 输出参数：无
// 功能描述：处理屏蔽寄存器指令22
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
// 函数名称：FRWMultipleReg
// 输入参数：Response
// 输出参数：无
// 功能描述：处理读写多寄存器指令23
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
	
	if((ReadQuantity>=0x001)&&(ReadQuantity<=0x007d)&&		// 读寄存器的数量是否在有效范围内
	   (WriteQuantity>=0x001)&&(WriteQuantity<=0x0079)&&	// 写寄存器的数量是否在有效范围内
	   (WriteBytes==(WriteQuantity*2)))						// 接收到的字节数是否与写入的寄存器数量一至
	{
		if((ReadStaAddr<END_HOLDING_REG_ADDR)&&((ReadStaAddr+ReadQuantity)<=END_HOLDING_REG_ADDR)&&
		   (WriteStaAddr<END_HOLDING_REG_ADDR)&&((WriteStaAddr+WriteQuantity)<=END_HOLDING_REG_ADDR))
		{
			if(WriteHoldingReg(WriteValuePtr,WriteStaAddr,WriteQuantity))	// 写寄存器
			{
			   if(ReadHoldingReg(PDUPtr+2,ReadStaAddr,ReadQuantity))		// 读寄存器
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












