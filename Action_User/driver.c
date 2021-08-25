#include "driver.h"
#include "dma.h"

/*******************************控制驱动器命令************************************/

/**
* @brief  驱动器状态(使能或失能)
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  state：状态, 范围: 
							ENABLE:使能
							DISABLE:失能
* @author ACTION
* @note 
*/
void DriverState(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, FunctionalState state)
{
	uint16_t identifier = IDENTIFIER_DRIVER_STATE;
	uint32_t data = state;
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
		
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;	  
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 2;
	TxMessage.Data[0] = (identifier>>0)&0xFF;
	TxMessage.Data[1] = (data>>0)&0xFF;

	mbox= CAN_Transmit(CANx, &TxMessage);
	
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;	
		}
	}
}


/**
* @brief  电机转矩控制
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  torque: 转矩，单位：毫牛米, 范围：-100 * 1000 ~ 100 * 1000
* @author ACTION
*/
void TorqueCtrl(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, int32_t torque)
{ 
	uint16_t identifier = IDENTIFIER_TORQUE_CTRL;
	uint32_t data = abs(torque);
	uint16_t timeout = 0;
	uint8_t mbox;
	CanTxMsg TxMessage;
	
	/*限幅*/
	if(data >= 100 * 1000)
	{
		data = 100 * 1000;
	}
					 
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 4;
	TxMessage.Data[0] = (identifier>>0)&0xFF;
	TxMessage.Data[1] = (data>>0)&0xFF;
	TxMessage.Data[2] = (data>>8)&0xFF;
	
	/*发送符号位*/
	if(torque >= 0)
	{
		TxMessage.Data[3] = (data>>16)&0xFF; 
	}
	else if(torque < 0)
	{
		TxMessage.Data[3] = ((data>>16)&0xFF) | 0x80;
	}

	mbox = CAN_Transmit(CANx, &TxMessage);
	
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;	
		
		}
	}
}


/**
* @brief  电机速度控制
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  vel: 速度，单位：脉冲每秒, 范围：-1024 * 4096 ~ 1024 * 4096
* @author ACTION
*/
void VelCtrl(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, int32_t vel)
{ 
	uint16_t identifier = IDENTIFIER_VEL_CTRL;
	uint32_t data = abs(vel);
	uint16_t timeout = 0;
	uint8_t mbox;
	CanTxMsg TxMessage;
	
	/*限幅*/
	if(data >= 1024 * 4096)
	{
		data = 1024 * 4096;
	}
					 
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 4;
	TxMessage.Data[0] = (identifier>>0)&0xFF;
	TxMessage.Data[1] = (data>>0)&0xFF;
	TxMessage.Data[2] = (data>>8)&0xFF;
	
	/*发送符号位*/
	if(vel >= 0)
	{
		TxMessage.Data[3] = (data>>16)&0xFF; 
	}
	else if(vel < 0)
	{
		TxMessage.Data[3] = ((data>>16)&0xFF) | 0x80;
	}

	mbox = CAN_Transmit(CANx, &TxMessage);
	
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;	
		
		}
	}
}

/**
* @brief  电机位置控制
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  posMode: 位置环运行模式，范围：
				ABSOLUTE_MODE: 绝对位置模式
				RELATIVE_MODE: 相对位置模式
* @param  pos:位置命令，单位：脉冲, 范围：-1024 * 4096 ~ 1024 * 4096
* @author ACTION
*/
void PosCtrl(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, PosLoopMode posMode, int32_t pos)
{
	uint16_t identifier = posMode;
	uint32_t data = abs(pos);
	uint16_t timeout = 0;
	uint8_t mbox;
	CanTxMsg TxMessage;
	
	/*限幅*/
	if(data >= 1024 * 4096)
	{
		data = 1024 * 4096;
	}

	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 4;
	TxMessage.Data[0] = (identifier>>0)&0xFF;
	TxMessage.Data[1] = (data>>0)&0xFF;
	TxMessage.Data[2] = (data>>8)&0xFF;
	
	/*发送符号位*/
	if(pos >= 0)
	{
		TxMessage.Data[3] = (data>>16)&0xFF; 
	}
	else if(pos < 0)
	{
		TxMessage.Data[3] = ((data>>16)&0xFF) | 0x80;
	}

	mbox = CAN_Transmit(CANx, &TxMessage);
	
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;	
		
		}
	}
}

/**
* @brief  配置驱动器控制模式
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  ctrlMode：控制模式，范围：
			SPD_CURR_CTRL_MODE：速度-电流双闭环模式
			POS_SPD_CURR_CTRL_MODE：位置-速度-电流三闭环模式
			POS_CURR_CTRL_MODE：位置-电流双闭环模式
* @author ACTION
* @note
*/
void SetCtrlMode(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, ControlMode ctrlMode)
{
	uint16_t identifier = IDENTIFIER_SET_CTRL_MODE;
	uint32_t data = ctrlMode;
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 4;
	TxMessage.Data[0] = (identifier>>0)&0xFF;
	TxMessage.Data[1] = (data>>0)&0xFF;
	TxMessage.Data[2] = (data>>8)&0xFF;
	TxMessage.Data[3] = (data>>16)&0xFF;
	
	mbox = CAN_Transmit(CANx, &TxMessage);
	
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			
			break;	
		}
	}
}



/**
* @brief  配置电机加速度与减速度
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  acc：加速度，单位：脉冲每二次方秒, 范围：0 ~ 8192 * 512(512代表一转)
* @param  dec：减速度，单位：脉冲每二次方秒, 范围：0 ~ 8192 * 512(512代表一转)
* @author ACTION
* @note
*/
void SetAccDec(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t acc, uint32_t dec)
{
	uint16_t identifier[2] = {IDENTIFIER_SET_ACC, IDENTIFIER_SET_DEC};
	uint32_t data[2] = {acc, dec};
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;

	/*限幅*/
	for(uint8_t i = 0; i < 2; i++)
	{
		if(data[i] > 1024 * 4096)
		{
			data[i] = 1024 * 4096;
		}
	}
	
	for(uint8_t i = 0; i < 2; i++)
	{
		TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
		TxMessage.ExtId = 0;
		TxMessage.IDE = CAN_Id_Standard;
		TxMessage.RTR = CAN_RTR_Data;
		TxMessage.DLC = 4;
		TxMessage.Data[0] = (identifier[i]>>0)&0xFF;
		TxMessage.Data[1] = (data[i]>>0)&0xFF;
		TxMessage.Data[2] = (data[i]>>8)&0xFF;
		TxMessage.Data[3] = (data[i]>>16)&0xFF;
		
		mbox = CAN_Transmit(CANx, &TxMessage);
		
		/*等待发送成功*/
		while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
		{
			timeout++;
			if(timeout > 6000)
			{
				if(CANx == CAN1)
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)("CAN1 send error !!!"));
				else
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)("CAN2 send error !!!"));
				/*在这里应加入异常处理*/
				
				break;	
			}
		}
	}
}


/**
* @brief  配置电机最大转矩
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  limit：最大转矩, 单位: 毫牛米, 范围：0 ~ 100 * 1000
* @author ACTION
* @note
*/
void SetTorqueLimit(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t limit)
{
	uint16_t identifier = IDENTIFIER_SET_TORQUE_LIMIT;
	uint32_t data = limit;	
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
	
	/*限幅*/
	if(data > 100 * 1000)
	{
		data = 100 * 1000;
	}
	
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 4;
	TxMessage.Data[0] = (identifier>>0)&0xFF;
	TxMessage.Data[1] = (data>>0)&0xFF;
	TxMessage.Data[2] = (data>>8)&0xFF;
	TxMessage.Data[3] = (data>>16)&0xFF;
		
	mbox = CAN_Transmit(CANx, &TxMessage);
		
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;	
			
		}
	}
}

/**
* @brief  配置速度环最大期望速度
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  limit：速度限制，单位：脉冲每秒, 范围：0 ~ 1024 * 4096
* @author ACTION
* @note
*/
void SetVelLimit(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t limit)
{
	uint16_t identifier = IDENTIFIER_SET_VEL_LIMIT;
	uint32_t data = limit;	
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
	
	/*限幅*/
	if(data > 1024 * 4096)
	{
		data = 1024 * 4096;
	}
	
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 4;
	TxMessage.Data[0] = (identifier>>0)&0xFF;
	TxMessage.Data[1] = (data>>0)&0xFF;
	TxMessage.Data[2] = (data>>8)&0xFF;
	TxMessage.Data[3] = (data>>16)&0xFF;
		
	mbox = CAN_Transmit(CANx, &TxMessage);
		
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			
			break;	
			
		}
	}
}


/**
* @brief  配置电机位置限制
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  upperLimit：正方向位置限制，单位：脉冲, 范围：-1024 * 4096 ~ 1024 * 4096
* @param  lowerLimit：反方向位置限制，单位：脉冲, 范围：-1024 * 4096 ~ upperLimit
* @author ACTION
* @note
*/
void SetPosLimit(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, int32_t upperLimit, int32_t lowerLimit)
{
	uint16_t identifier[2] = {IDENTIFIER_SET_POS_LIMIT_UP, IDENTIFIER_SET_POS_LIMIT_LOW};
	int32_t limit[2] = {upperLimit, lowerLimit};
	uint32_t data[2] = {0};
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
	
	/*upperLimit必须大于或等于lowerLimit*/
	if(limit[1] > limit[0])
	{
		limit[1] = limit[0];
	}
	
	data[0] = abs(limit[0]);
	data[1] = abs(limit[1]);
	
	/*限幅*/
	for(uint8_t i = 0; i < 2; i++)
	{
		if(data[i] > 1024 * 4096)
		{
			data[i] = 1024 * 4096;
		}
	}
	
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;	
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 4;
	
	for(uint8_t i = 0; i < 2; i++)
	{
		TxMessage.Data[0] = (identifier[i]>>0)&0xFF;
		TxMessage.Data[1] = (data[i]>>0)&0xFF;
		TxMessage.Data[2] = (data[i]>>8)&0xFF;
			
		/*发送符号位*/
		if(limit[i] >= 0)
		{
			TxMessage.Data[3] = (data[i]>>16)&0xFF;
		}
		else if(limit[i] < 0)
		{
			TxMessage.Data[3] = ((data[i]>>16)&0xFF) | 0x80;
		}
			
		mbox= CAN_Transmit(CANx, &TxMessage);
			
		/*等待发送成功*/
		while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
		{
			timeout++;
			if(timeout > 6000)
			{
				if(CANx == CAN1)
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)("CAN1 send error !!!"));
				else
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)("CAN2 send error !!!"));
				/*在这里应加入异常处理*/
				
				break;	
			}
		}
	}

}


/**
* @brief  配置电流环kp
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  p_current：电流环kp值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetCurrentKP(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t p_current)
{
	uint16_t identifier = IDENTIFIER_CURR_KP_Q;
	uint32_t data = p_current;	
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
	
	/*限幅*/
	//fix me 添加正确的限幅
//	if(data > 100 * 1000)
//	{
//		data = 100 * 1000;
//	}
	
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 4;
	TxMessage.Data[0] = (identifier>>0)&0xFF;
	TxMessage.Data[1] = (data>>0)&0xFF;
	TxMessage.Data[2] = (data>>8)&0xFF;
	TxMessage.Data[3] = (data>>16)&0xFF;
		
	mbox = CAN_Transmit(CANx, &TxMessage);
		
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;	
			
		}
	}
}

/**
* @brief  配置电流环ki
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  i_current：电流环ki值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetCurrentKI(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t i_current)
{
	uint16_t identifier = IDENTIFIER_CURR_KI_Q;
	uint32_t data = i_current;	
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
	
	/*限幅*/
	//fix me 添加正确的限幅
//	if(data > 100 * 1000)
//	{
//		data = 100 * 1000;
//	}
	
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 4;
	TxMessage.Data[0] = (identifier>>0)&0xFF;
	TxMessage.Data[1] = (data>>0)&0xFF;
	TxMessage.Data[2] = (data>>8)&0xFF;
	TxMessage.Data[3] = (data>>16)&0xFF;
		
	mbox = CAN_Transmit(CANx, &TxMessage);
		
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;	
			
		}
	}
}

/**
* @brief  配置速度环kp
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  p_speed：速度环kp值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetSpeedKP(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t p_speed)
{
	uint16_t identifier = IDENTIFIER_SPD_KP;
	uint32_t data = p_speed;	
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
	
	/*限幅*/
	//fix me 添加正确的限幅
//	if(data > 100 * 1000)
//	{
//		data = 100 * 1000;
//	}
	
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 4;
	TxMessage.Data[0] = (identifier>>0)&0xFF;
	TxMessage.Data[1] = (data>>0)&0xFF;
	TxMessage.Data[2] = (data>>8)&0xFF;
	TxMessage.Data[3] = (data>>16)&0xFF;
		
	mbox = CAN_Transmit(CANx, &TxMessage);
		
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;	
			
		}
	}
}

/**
* @brief  配置速度环ki
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  i_speed：速度环ki值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetSpeedKI(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t i_speed)
{
	uint16_t identifier = IDENTIFIER_SPD_KI;
	uint32_t data = i_speed;	
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
	
	/*限幅*/
	//fix me 添加正确的限幅
//	if(data > 100 * 1000)
//	{
//		data = 100 * 1000;
//	}
	
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 4;
	TxMessage.Data[0] = (identifier>>0)&0xFF;
	TxMessage.Data[1] = (data>>0)&0xFF;
	TxMessage.Data[2] = (data>>8)&0xFF;
	TxMessage.Data[3] = (data>>16)&0xFF;
		
	mbox = CAN_Transmit(CANx, &TxMessage);
		
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;	
			
		}
	}
}

/**
* @brief  配置位置环kp
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  p_pos：位置环kp值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetPosKP(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t p_pos)
{
	uint16_t identifier = IDENTIFIER_POS_KP;
	uint32_t data = p_pos;	
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
	
	/*限幅*/
	//fix me 添加正确的限幅
//	if(data > 100 * 1000)
//	{
//		data = 100 * 1000;
//	}
	
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 4;
	TxMessage.Data[0] = (identifier>>0)&0xFF;
	TxMessage.Data[1] = (data>>0)&0xFF;
	TxMessage.Data[2] = (data>>8)&0xFF;
	TxMessage.Data[3] = (data>>16)&0xFF;
		
	mbox = CAN_Transmit(CANx, &TxMessage);
		
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;	
			
		}
	}
}

/**
* @brief  配置位置环kd
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  d_pos：位置环kd值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetPosKD(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t d_pos)
{
	uint16_t identifier = IDENTIFIER_POS_KD;
	uint32_t data = d_pos;	
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
	
	/*限幅*/
	//fix me 添加正确的限幅
//	if(data > 100 * 1000)
//	{
//		data = 100 * 1000;
//	}
	
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 4;
	TxMessage.Data[0] = (identifier>>0)&0xFF;
	TxMessage.Data[1] = (data>>0)&0xFF;
	TxMessage.Data[2] = (data>>8)&0xFF;
	TxMessage.Data[3] = (data>>16)&0xFF;
		
	mbox = CAN_Transmit(CANx, &TxMessage);
		
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;	
			
		}
	}
}

/**
* @brief  抱死模式（轮子/俯仰）
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式 （因为是只给轮子发的，所以就算是广播模式航向也不会收到）
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  ControlMode：
*               1：切换成位置模式/抱死
*               2：切换成速度模式/解除抱死
* @author ACTION
* @note
*/
void SetWheelControlMode(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint8_t ControlMode)
{
	uint16_t identifier = IDENTIFIER_SET_CONTROL_MODE;
	uint8_t data = ControlMode;
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;

	/*限幅*/
	//fix me 添加正确的限幅
//	if(data > 100 * 1000)
//	{
//		data = 100 * 1000;
//	}

	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 2;
	TxMessage.Data[0] = (identifier >> 0) & 0xFF;
	TxMessage.Data[1] = (data >> 0) & 0xFF;

	mbox = CAN_Transmit(CANx, &TxMessage);

	/*等待发送成功*/
	while ((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if (timeout > 6000)
		{
			if (CANx == CAN1)
				USARTDMAOUT(DEBUG_USART, DebugUSARTSendBuf, &DebugUSARTSendBuffCnt, DebugUSARTDMASendBuf, DEBUG_USART_SEND_BUF_CAPACITY, \
					(uint8_t*)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART, DebugUSARTSendBuf, &DebugUSARTSendBuffCnt, DebugUSARTDMASendBuf, DEBUG_USART_SEND_BUF_CAPACITY, \
					(uint8_t*)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;

		}
	}
}



/**
* @brief  配置电机最大电流限幅
* @param  CANx: 所使用的CAN通道编号
* @param  DriverNum：驱动器ID号, 范围：1~64
* @param  limit：最大转矩电流, 单位:毫安, 范围：0 ~ 150 * 1000(mA)
* @author ACTION
* @note
*/
void SetCurrQLimit(CAN_TypeDef* CANx, uint8_t DriverNum, uint32_t CurrQLimit)
{
	uint16_t identifier = IDENTIFIER_SET_CurrQ_Limit;
	uint32_t data = CurrQLimit;
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;

	/*限幅*/
	if(data > 150 * 1000)
	{
		data = 150 * 1000;
	}

	TxMessage.StdId = DRIVER_SERVER_BASE_ID + 0x00 + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 4;
	TxMessage.Data[0] = (identifier >> 0) & 0xFF;
	TxMessage.Data[1] = (data >> 0) & 0xFF;
	TxMessage.Data[2] = (data >> 8) & 0xFF;
	TxMessage.Data[3] = (data >> 16) & 0xFF;

	mbox = CAN_Transmit(CANx, &TxMessage);

	/*等待发送成功*/
	while ((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if (timeout > 6000)
		{
			if (CANx == CAN1)
				USARTDMAOUT(DEBUG_USART, DebugUSARTSendBuf, &DebugUSARTSendBuffCnt, DebugUSARTDMASendBuf, DEBUG_USART_SEND_BUF_CAPACITY, \
					(uint8_t*)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART, DebugUSARTSendBuf, &DebugUSARTSendBuffCnt, DebugUSARTDMASendBuf, DEBUG_USART_SEND_BUF_CAPACITY, \
					(uint8_t*)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;

		}
	}
}



/**
* @brief  清除速度环积分
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note
*/
void SetClearIntegral(CAN_TypeDef* CANx, uint8_t DriverNum)
{
	uint16_t identifier = IDENTIFIER_SET_CLEAR_INTEGRAL;
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;

	/*限幅*/
	//fix me 添加正确的限幅
//	if(data > 100 * 1000)
//	{
//		data = 100 * 1000;
//	}

	TxMessage.StdId = DRIVER_SERVER_BASE_ID + 0x00 + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 1;
	TxMessage.Data[0] = (identifier >> 0) & 0xFF;

	mbox = CAN_Transmit(CANx, &TxMessage);

	/*等待发送成功*/
	while ((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if (timeout > 6000)
		{
			if (CANx == CAN1)
				USARTDMAOUT(DEBUG_USART, DebugUSARTSendBuf, &DebugUSARTSendBuffCnt, DebugUSARTDMASendBuf, DEBUG_USART_SEND_BUF_CAPACITY, \
					(uint8_t*)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART, DebugUSARTSendBuf, &DebugUSARTSendBuffCnt, DebugUSARTDMASendBuf, DEBUG_USART_SEND_BUF_CAPACITY, \
					(uint8_t*)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;

		}
	}
}

/**
* @brief  位置设置为零，设置保险的位置，设置期望的最终速度
* @param  CANx: 所使用的CAN通道编号
* @param  DriverNum：驱动器ID号，范围：1~64
* @param  
          SecurePos ：
		            含义：保险位置
					单位：脉冲
					范围：-1024 * 4096~1024 * 4096
          ExptFinalVel
                    含义：期望最终速度
					单位：脉冲每秒
					范围：-1024 * 4096~1024 * 4096
* @author ACTION
* @note
*/
void SetPosZero(CAN_TypeDef* CANx, uint8_t DriverNum, int32_t SecurePos, int32_t ExptFinalVel)
{
	uint16_t identifier[2] = { IDENTIFIER_SET_SECURE_POS, IDENTIFIER_SET_EXPTFINAL_VEL};
	int32_t Value[2] = { SecurePos, ExptFinalVel};
	uint32_t data[2] = { 0 };
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
    
	/*限幅*/
	for (uint8_t i = 0; i < 2; i++)
	{
		data[i] = abs(Value[i]);
		if (data[i] > 1024 * 4096)
		{
			data[i] = 1024 * 4096;
		}
	}

	for (uint8_t i = 0; i < 2; i++)
	{
		TxMessage.StdId = DRIVER_SERVER_BASE_ID + 0x00 + DriverNum;
		TxMessage.ExtId = 0;
		TxMessage.IDE = CAN_Id_Standard;
		TxMessage.RTR = CAN_RTR_Data;
		TxMessage.DLC = 4;
		TxMessage.Data[0] = (identifier[i] >> 0) & 0xFF;
		TxMessage.Data[1] = (data[i] >> 0) & 0xFF;
		TxMessage.Data[2] = (data[i] >> 8) & 0xFF;

		/*发送符号位*/
		if (Value[i] >= 0)
		{
			TxMessage.Data[3] = (data[i] >> 16) & 0xFF;
		}
		else if (Value[i] < 0)
		{
			TxMessage.Data[3] = ((data[i] >> 16) & 0xFF) | 0x80;
		}

		mbox = CAN_Transmit(CANx, &TxMessage);

		/*等待发送成功*/
		while ((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
		{
			timeout++;
			if (timeout > 6000)
			{
				if (CANx == CAN1)
					USARTDMAOUT(DEBUG_USART, DebugUSARTSendBuf, &DebugUSARTSendBuffCnt, DebugUSARTDMASendBuf, DEBUG_USART_SEND_BUF_CAPACITY, \
						(uint8_t*)("CAN1 send error !!!"));
				else
					USARTDMAOUT(DEBUG_USART, DebugUSARTSendBuf, &DebugUSARTSendBuffCnt, DebugUSARTDMASendBuf, DEBUG_USART_SEND_BUF_CAPACITY, \
						(uint8_t*)("CAN2 send error !!!"));
				/*在这里应加入异常处理*/

				break;
			}
		}
	}
}

/**********************************读取驱动器数据命令*************************************/

/**
* @brief  读取电机电磁转矩
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫牛米
*/
void ReadTorque(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_TORQUE;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 1;
	TxMessage.Data[0] = (identifier>>0)&0xFF;

	mbox= CAN_Transmit(CANx, &TxMessage);

	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;	
		}
	}
}

/**
* @brief  读取电机速度
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲每秒
*/
void ReadVel(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_VEL;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;

	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 1;
	TxMessage.Data[0] = (identifier>>0)&0xFF;

	mbox= CAN_Transmit(CANx, &TxMessage);
	
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			
			break;	
		}
	}
}


/**
* @brief  读取电机速度环输出
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲每秒
*/
void ReadVelLoopOut(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_SPD_LOOP_OUTPUT;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;

	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 1;
	TxMessage.Data[0] = (identifier>>0)&0xFF;

	mbox= CAN_Transmit(CANx, &TxMessage);
	
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			
			break;	
		}
	}
}


/**
* @brief  读取电机位置
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲
*/
void ReadPos(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_POS;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 1;
	TxMessage.Data[0] = (identifier>>0)&0xFF;

	mbox= CAN_Transmit(CANx, &TxMessage);
						 
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			
			break;	
		}
	}
}

/**
* @brief  读取电机位置环输出
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲
*/
void ReadPosLoopOut(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_POS_LOOP_OUTPUT;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 1;
	TxMessage.Data[0] = (identifier>>0)&0xFF;

	mbox= CAN_Transmit(CANx, &TxMessage);
						 
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			
			break;	
		}
	}
}


/**
* @brief  读取编码器脉冲
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲
*/
void ReadEncoder(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_ENCODER_POS;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 1;
	TxMessage.Data[0] = (identifier>>0)&0xFF;

	mbox= CAN_Transmit(CANx, &TxMessage);
						 
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			
			break;	
		}
	}
}

/**
* @brief  读取电机交轴电压
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫伏
*/
void ReadVolQ(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_VOL_Q;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 1;
	TxMessage.Data[0] = (identifier>>0)&0xFF;

	mbox= CAN_Transmit(CANx, &TxMessage);

	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;	
		}
	}
}


/**
* @brief  读取电机转矩电流
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫安
*/
void ReadCurrQ(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_CURR_Q;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 1;
	TxMessage.Data[0] = (identifier>>0)&0xFF;

	mbox= CAN_Transmit(CANx, &TxMessage);

	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			
			break;	
		}
	}
}

/**
* @brief  读取电机直轴电压
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫伏
*/
void ReadVolD(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_VOL_D;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 1;
	TxMessage.Data[0] = (identifier>>0)&0xFF;

	mbox= CAN_Transmit(CANx, &TxMessage);

	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;	
		}
	}
}


/**
* @brief  读取电机直轴电流
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫安
*/
void ReadCurrD(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_CURR_D;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 1;
	TxMessage.Data[0] = (identifier>>0)&0xFF;

	mbox= CAN_Transmit(CANx, &TxMessage);

	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			
			break;	
		}
	}
}



///**
//* @brief  读取四足小腿状态(包括速度/位置/触地)
//*         #该指令仅限于四足小腿电缸驱动器#
//* @param  CANx：所使用的CAN通道编号
//* @param  CMDmode: 指令模式，范围：
//				PTP_MODE: 点对点模式
//				BROADCAST_MODE: 广播模式
//* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
//* @author ACTION
//* @note   
//*         速度 单位：脉冲每秒  范围：-1024*4096 ~ 1024*4096
//		  位置 单位：脉冲  范围：-1024*4096 ~ 1024*4096
//		  触地 0/1
//*/
//void ReadShankStatus(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum)
//{
//	uint16_t identifier = 0x40 + IDENTIFIER_READ_SHANK_STATUS;
//	uint8_t mbox = 0;
//	uint16_t timeout = 0;
//	CanTxMsg TxMessage;

//	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
//	TxMessage.ExtId = 0;
//	TxMessage.IDE = CAN_Id_Standard;
//	TxMessage.RTR = CAN_RTR_Data;
//	TxMessage.DLC = 1;
//	TxMessage.Data[0] = (identifier >> 0) & 0xFF;

//	mbox = CAN_Transmit(CANx, &TxMessage);

//	/*等待发送成功*/
//	while ((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
//	{
//		timeout++;
//		if (timeout > 6000)
//		{
//			if (CANx == CAN1)
//				USARTDMAOUT(DEBUG_USART, DebugUSARTSendBuf, &DebugUSARTSendBuffCnt, DebugUSARTDMASendBuf, DEBUG_USART_SEND_BUF_CAPACITY, \
//					(uint8_t*)("CAN1 send error !!!"));
//			else
//				USARTDMAOUT(DEBUG_USART, DebugUSARTSendBuf, &DebugUSARTSendBuffCnt, DebugUSARTDMASendBuf, DEBUG_USART_SEND_BUF_CAPACITY, \
//					(uint8_t*)("CAN2 send error !!!"));
//			/*在这里应加入异常处理*/

//			break;
//		}
//	}


//}

/**
* @brief  读取转接板ADC_IN45\SPI2的值
* @param  CANx：所使用的CAN通道编号
* @param  DriverNum：转接板ID号，与驱动器同步, 范围：1~64
* @author ACTION
* @note   范围：
*         ADC_IN4 0-4095
		  ADC_IN5 0-4095
		  SPI2    0-32767
*/
void READ_TRANSPCB_ADC45_SPI2(CAN_TypeDef* CANx, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_TRANSPCB_ADC45_SPI2;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;

	TxMessage.StdId = DRIVER_SERVER_BASE_ID + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 1;
	TxMessage.Data[0] = (identifier >> 0) & 0xFF;

	mbox = CAN_Transmit(CANx, &TxMessage);

	/*等待发送成功*/
	while ((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if (timeout > 6000)
		{
			if (CANx == CAN1)
				USARTDMAOUT(DEBUG_USART, DebugUSARTSendBuf, &DebugUSARTSendBuffCnt, DebugUSARTDMASendBuf, DEBUG_USART_SEND_BUF_CAPACITY, \
					(uint8_t*)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART, DebugUSARTSendBuf, &DebugUSARTSendBuffCnt, DebugUSARTDMASendBuf, DEBUG_USART_SEND_BUF_CAPACITY, \
					(uint8_t*)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/

			break;
		}
	}


}


/**
* @brief  读取转接板ADC_IN67\SPI3的值
* @param  CANx：所使用的CAN通道编号
* @param  DriverNum：转接板ID号，与驱动器同步, 范围：1~64
* @author ACTION
* @note   范围：
*         ADC_IN6 0-4095
		  ADC_IN7 0-4095
		  SPI3    0-32767
*/
void READ_TRANSPCB_ADC67_SPI3(CAN_TypeDef* CANx,uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_TRANSPCB_ADC67_SPI3;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;

	TxMessage.StdId = DRIVER_SERVER_BASE_ID + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 1;
	TxMessage.Data[0] = (identifier >> 0) & 0xFF;

	mbox = CAN_Transmit(CANx, &TxMessage);

	/*等待发送成功*/
	while ((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if (timeout > 6000)
		{
			if (CANx == CAN1)
				USARTDMAOUT(DEBUG_USART, DebugUSARTSendBuf, &DebugUSARTSendBuffCnt, DebugUSARTDMASendBuf, DEBUG_USART_SEND_BUF_CAPACITY, \
					(uint8_t*)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART, DebugUSARTSendBuf, &DebugUSARTSendBuffCnt, DebugUSARTDMASendBuf, DEBUG_USART_SEND_BUF_CAPACITY, \
					(uint8_t*)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/

			break;
		}
	}
}


/**
* @brief  读取小腿霍尔的ADC值
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   范围：
*         HallA 0-4095
		  HallB 0-4095
		  
*/
void ReadHallAB(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_HALLAB;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;

	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 1;
	TxMessage.Data[0] = (identifier >> 0) & 0xFF;

	mbox = CAN_Transmit(CANx, &TxMessage);

	/*等待发送成功*/
	while ((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if (timeout > 6000)
		{
			if (CANx == CAN1)
				USARTDMAOUT(DEBUG_USART, DebugUSARTSendBuf, &DebugUSARTSendBuffCnt, DebugUSARTDMASendBuf, DEBUG_USART_SEND_BUF_CAPACITY, \
					(uint8_t*)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART, DebugUSARTSendBuf, &DebugUSARTSendBuffCnt, DebugUSARTDMASendBuf, DEBUG_USART_SEND_BUF_CAPACITY, \
					(uint8_t*)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/

			break;
		}
	}


}
/**
* @brief  获取电机消息的值
* @param  *driverMsg：对应电机消息结构体的地址
* @param  *data8: 接收驱动器消息数组的首地址
* @author ACTION
* @note   方便使用
*/
void GetDriverMsg(driverMsg_t* driverMsg, uint8_t* data8)
{
	union receive
	{
		uint8_t data8[4];
		int data32;
		float dataf;
	}receiveMsg,receiveMsg1;
	
	union received
	{
		uint8_t data8[2];
		uint16_t data16;
	}receiveMsg2,receiveMsg3;
	
	for (int i = 0; i < 4; i++)
	{
		receiveMsg.data8[i] = data8[i];
		receiveMsg1.data8[i] = data8[i+4];
	}
	for (int i = 0; i < 2; i++)
	{
		receiveMsg2.data8[i] = data8[i+1];
		receiveMsg3.data8[i] = data8[i+3];
	}
	if (receiveMsg.data8[0] == IDENTIFIER_READ_VEL)
	{
		//获取脉冲速度
		driverMsg->vel = TransformValue(receiveMsg.data32);
	}
	if (receiveMsg.data8[0] == IDENTIFIER_READ_POS)
	{
		//获取脉冲位置
		driverMsg->pos = TransformValue(receiveMsg.data32);
	}
	if (receiveMsg.data8[0] == IDENTIFIER_READ_ENCODER_POS)
	{
		//获取编码器脉冲
		driverMsg->encoder = TransformValue(receiveMsg.data32);
	}
	if (receiveMsg.data8[0] == IDENTIFIER_READ_TORQUE)
	{
		//获取电磁转矩
		driverMsg->torque = TransformValue(receiveMsg.data32);
	}
	if (receiveMsg.data8[0] == IDENTIFIER_READ_VOL_Q)
	{
		//获取交轴电压
		driverMsg->volQ = TransformValue(receiveMsg.data32);
	}
	if (receiveMsg.data8[0] == IDENTIFIER_READ_CURR_Q)
	{
		//获取转矩电流
		driverMsg->currQ = TransformValue(receiveMsg.data32);
	}
	if (receiveMsg.data8[0] == IDENTIFIER_READ_CURR_D)
	{
		//获取转矩电流
		driverMsg->currD = TransformValue(receiveMsg.data32);
	}
	if (receiveMsg.data8[0] == IDENTIFIER_READ_VOL_D)
	{
		//获取直轴电压
		driverMsg->volD = TransformValue(receiveMsg.data32);
	}
	if (receiveMsg.data8[0] == IDENTIFIER_READ_SPD_LOOP_OUTPUT)
	{
		//获取速度环输出
		driverMsg->velLoopOut = TransformValue(receiveMsg.data32);
	}
	if (receiveMsg.data8[0] == IDENTIFIER_READ_POS_LOOP_OUTPUT)
	{
		//获取位置环输出
		driverMsg->posLoopOut = TransformValue(receiveMsg.data32);
	}
	if (receiveMsg.data8[0] == IDENTIFIER_ENCODER_ERROR)
	{
		//获取转矩电流
		driverMsg->encoderErr = TransformValue(receiveMsg.data32);
	}
	if (receiveMsg.data8[0] == IDENTIFIER_HARD_FAULT)
	{
		//获取错误标志
		driverMsg->hardFault = TransformValue(receiveMsg.data32);
	}
//	if (receiveMsg.data8[0] == IDENTIFIER_READ_SHANK_STATUS)
//	{
//		//获取四足小腿状态
//		driverMsg->vel = TransformValue(receiveMsg.data32);
//		driverMsg->pos = TransformValue(receiveMsg1.data32);
//		driverMsg->travelSwitch_Status = receiveMsg1.data8[0];
//	}
	if (receiveMsg.data8[0] == IDENTIFIER_READ_HALLAB)
	{
		//获取四足小腿霍尔值
		driverMsg->thighHallValue[0] = receiveMsg2.data16;
		driverMsg->thighHallValue[1] = receiveMsg3.data16;
	}
    if (receiveMsg.data8[0] == IDENTIFIER_READ_TRANSPCB_ADC45_SPI2)
	{
		driverMsg->leftthighADCValue[0] = receiveMsg2.data16;
		driverMsg->leftthighADCValue[1] = receiveMsg3.data16;
		driverMsg->leftthighSPIValue = TransformValue(receiveMsg1.data32);
	}

	if (receiveMsg.data8[0] == IDENTIFIER_READ_TRANSPCB_ADC67_SPI3)
	{
		driverMsg->rightthighADCValue[0] = receiveMsg2.data16;
		driverMsg->rightthighADCValue[1] = receiveMsg3.data16;
		driverMsg->rightthighSPIValue = TransformValue(receiveMsg1.data32);
	}
  if (receiveMsg.data8[0] == IDENTIFIER_READ_VOLIN)
  {
    driverMsg->VolIn = TransformValue(receiveMsg.data32);
  }

}


/*将三个高字节的数据移到低三字节并提取符号位*/
int TransformValue(int data32)
{
	if((data32&0x80000000) == 0)
	{
		return ((data32>>8)&0x7FFFFF);
	}
	else
	{
		return(-((data32>>8)&0x7FFFFF)); 
	}
}


/**
* @brief  读取航向位置，大转盘位置
* @param  CANx：所使用的CAN通道编号
* @param  ID：驱动器ID号，范围：，
* @author ACTION
 * @note：
*/
void ReadActualPos(CAN_TypeDef* CANx, uint8_t ID)
 {
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=DRIVER_SERVER_BASE_ID + ID;					// standard identifier=0
	TxMessage.ExtId=DRIVER_SERVER_BASE_ID + ID;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


   TxMessage.Data[0] = 'A';
	TxMessage.Data[1] = 'T';
	TxMessage.Data[2] = 'g';
	TxMessage.Data[3] = 'e';
	TxMessage.Data[4] = 't';
	TxMessage.Data[5] = 'p';
	TxMessage.Data[6] = 'o';
	TxMessage.Data[7] = 's';

	mbox= CAN_Transmit(CANx, &TxMessage);
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			break;		
		}
	} 
 }
 
 /**
 * @brief  霍尔开关
* @param  CANx：所使用的CAN通道编号
* @param  ID：驱动器ID号，范围：，
* @author ACTION
 * @note：
*/
void Hall_Ctrl(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, HELL_MODE IDENTIFIER_HALL_TEST)
{
	uint16_t identifier = IDENTIFIER_HALL_TEST;
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 4;
	TxMessage.Data[0] = (identifier>>0)&0xFF;

	mbox = CAN_Transmit(CANx, &TxMessage);
	
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			
			break;	
		}
	}
}

/**
* @brief  读取电机母线电压
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫伏
*/
void ReadVolIN(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_VOLIN;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;

	TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 1;
	TxMessage.Data[0] = (identifier>>0)&0xFF;

	mbox= CAN_Transmit(CANx, &TxMessage);

	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;	
		}
	}
}

/**
* @brief  设置灯板颜色
* @param  CANx: 所使用的CAN通道编号
* @param  DriverNum：灯板ID号 15
* @param
		  color :
					含义：灯板颜色
					范围：
					0x01	红色
					0x02	蓝色
					0x03	绿色
					0x04	紫色
					0x05	黄色
					0x06	白色
					0x00	关灯
* @author ACTION
* @note
*/
 void SET_LIGHTING_STATUS(CAN_TypeDef* CANx, uint8_t DriverNum,uint8_t color)
{
	uint16_t identifier = IDENTIFIER_LIGHTING_STATUS;
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;
	TxMessage.StdId = DRIVER_SERVER_BASE_ID + 0x00 + DriverNum;
	TxMessage.ExtId = 0;	  
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 2;
	TxMessage.Data[0] = (identifier>>0)&0xFF;
	TxMessage.Data[1] = (color>>0)&0xFF;	
	

	mbox= CAN_Transmit(CANx, &TxMessage);
	
	/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!!"));
			else
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!!"));
			/*在这里应加入异常处理*/
			break;	
		}
	}
}

/**
* @brief  配置电机位置加速度与减速度
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  acc：加速度，单位：脉冲每二次方秒, 范围：0 ~ 8192 * 512(512代表一转)
* @param  dec：减速度，单位：脉冲每二次方秒, 范围：0 ~ 8192 * 512(512代表一转)
* @author ACTION
* @note
*/
void SetPosAccDec(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t acc, uint32_t dec)
{
	uint16_t identifier[2] = {IDENTIFIER_SET_PosAcc, IDENTIFIER_SET_PosDec};
	uint32_t data[2] = {acc, dec};
	uint8_t mbox;
	uint16_t timeout = 0;
	CanTxMsg TxMessage;

	/*限幅*/
	for(uint8_t i = 0; i < 2; i++)
	{
		if(data[i] > 1024 * 4096)
		{
			data[i] = 1024 * 4096;
		}
	}
	
	for(uint8_t i = 0; i < 2; i++)
	{
		TxMessage.StdId = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
		TxMessage.ExtId = 0;
		TxMessage.IDE = CAN_Id_Standard;
		TxMessage.RTR = CAN_RTR_Data;
		TxMessage.DLC = 4;
		TxMessage.Data[0] = (identifier[i]>>0)&0xFF;
		TxMessage.Data[1] = (data[i]>>0)&0xFF;
		TxMessage.Data[2] = (data[i]>>8)&0xFF;
		TxMessage.Data[3] = (data[i]>>16)&0xFF;
		
		mbox = CAN_Transmit(CANx, &TxMessage);
		
		/*等待发送成功*/
	while((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			if(CANx == CAN1)
			{
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN1 send error !!! %d \r\n"),(int)identifier);
			}
			else
			{
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)("CAN2 send error !!! %d \r\n"),(int)identifier);
			}
			/*在这里应加入异常处理*/
			break;	
		}
	}
}
}

