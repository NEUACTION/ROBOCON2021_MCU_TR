#include "pccomm.h"
#include "robot.h"

//spi发送消息缓冲区数组
uint8_t sendMsg[MSG_LEN];
//spi接收消息缓冲区数组
uint8_t receiveMsg[MSG_LEN];
uint8_t recAT = 0;
int blank[8] = {110,220,330,440,550,660,770,880};
uint8_t blankChar1, blankChar2, blankChar3, blankChar4;
uint8_t tempTest = 0;
extern int pccommCnt;
/**
  * @brief	得到上位机发送来的数据函数
  * @note		接收发送数据和通讯检测都在此函数
  * @param	None 
  * @retval	None
  */
void Talk2PC(void)
{
	//接收数据内容
	uint8_t rxData[MSG_LEN-5] = {0};
	//接收PC的消息
	uint8_t SPIIdentifierCorrect = 0; 
	SendMsg2PC();
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
	(uint8_t *)("gSPI %d %d %d "),(int)receiveMsg[0],(int)receiveMsg[1],(int)receiveMsg[2]);
	//心跳包
	//判断头尾是否正确
	if(receiveMsg[1] == 'H' && receiveMsg[2] == 'D' && receiveMsg[MSG_LEN-2] == '\r' && receiveMsg[MSG_LEN-1] == '\n')
	{
		SPIIdentifierCorrect = 1;
		//移位心跳包
		//将spi接收缓冲区的数据内容赋给rxData
		for(int i=0;i < MSG_LEN-5;i++)
		{
			rxData[i]=receiveMsg[i+3];
		}
	}
	else if(receiveMsg[0] == 'H' && receiveMsg[1] == 'D' && receiveMsg[MSG_LEN-3] == '\r' && receiveMsg[MSG_LEN-2] == '\n')
	{
		SPIIdentifierCorrect = 1;
		//移位心跳包
		//将spi接收缓冲区的数据内容赋给rxData
		for(int i=0;i < MSG_LEN-5;i++)
		{
			rxData[i]=receiveMsg[i+2];
		}
	}
	
	if(SPIIdentifierCorrect == 1)
	{
		pccommCnt++;
		if(pccommCnt > 10)
		{
			gRobot.spiState.startTalkFlag = 1;
			pccommCnt = 11;
		}
//		gRobot.spiState.startTalkFlag = 1;
		gRobot.spiState.spiHB = 0;
		gRobot.spiState.spiShiftHB = 0;
		SPIIdentifierCorrect = 0;
		//解析rxData赋给各个变量
		int8_t readRetval = ReadData(FLAG_NUM,DATAF_NUM,(MSG_LEN-5),rxData,&gRobot.walkStatus,&gRobot.pathPlanFlag,\
										&gRobot.archeryStruct.archeryStart,&gRobot.cvStruct.state,&blankChar1,&blankChar2,&blankChar3,&blankChar4,\
										&gRobot.steerStruct.one.wheelVelTarget.vel,&gRobot.steerStruct.two.wheelVelTarget.vel,&gRobot.steerStruct.thr.wheelVelTarget.vel,\
										&gRobot.steerStruct.one.wheelVelTarget.pos,&gRobot.steerStruct.two.wheelVelTarget.pos,&gRobot.steerStruct.thr.wheelVelTarget.pos,\
										&gRobot.cvStruct.cvDir,&gRobot.cvStruct.cvDis,&gRobot.ppsData.x,&gRobot.ppsData.y,&gRobot.ppsData.angle);
//		gRobot.cvStruct.cvDir = gRobot.cvStruct.cvMcuDir/10.0f;
		if(readRetval < 0)
		{
			//返回值小于0提示数组越界
			gRobot.spiState.spiOffArray = 1;
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("ERROR: spi readData Writing off the end of the array! \n"));
		}
		//发送消息给PC

	}
}
/**
  * @brief	SendMsg2PC 发送完成之后向DMA填值函数
  * @note		None
  * @param	None
  * @retval	None
  */
void SendMsg2PC(void)
{
	//发送数据内容
	uint8_t txData[MSG_LEN-5] = {0};
	//将需要发送的各个变量填入txData
	int8_t writeRetval = WriteData(FLAG_NUM,DATAF_NUM,(MSG_LEN-5),txData,gRobot.robotMode,gRobot.walkMode,gRobot.shotArrowsCnt,\
												gRobot.attackPotID[gRobot.shootOrder],gRobot.fieldCol,gRobot.archeryStruct.fetchMotorReady,gRobot.TRRetryFlag,gRobot.writeFlag,\
												(int)(Wtable*100),(int)(shootPara.shootVel*100),(int)(voltageTemp*100),(int)(shootPara.shootKp*100),(int)(shootPara.shootKi*100),\
												(int)(gRobot.archeryStruct.archeryPitch.Act.pos*100),blank[6],blank[7],blank[7],blank[7],blank[7]);
//记得改数据长度（进攻、防守的路径切换gRobot.walkMode）
//	int8_t writeRetval = WriteData(FLAG_NUM,DATAF_NUM,(MSG_LEN-5),txData,gRobot.robotMode,gRobot.walkMode,gRobot.shotArrowsCnt,\
//												gRobot.attackPotID[gRobot.shootOrder],gRobot.fieldCol,gRobot.archeryStruct.fetchMotorReady,gRobot.TRRetryFlag,\
//												(int)(Wtable * 100),blank[1],blank[2],blank[3],blank[4],blank[5],blank[6],blank[7],blank[7],blank[7],blank[7]);
	if(writeRetval < 0)
	{
		//返回值小于0提示数组越界
		gRobot.spiState.spiOffArray = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
			(uint8_t *)("ERROR: spi wirteData Writing off the end of the array! \r\n"));
	}
	//将txData与头尾写入spi发送缓冲区
	for(int i=0;i < MSG_LEN-5;i++)
	{
		sendMsg[i+2] = txData[i];
	}
	sendMsg[0]='H';
	sendMsg[1]='M';
	
	sendMsg[2]=tempTest;
	tempTest ++;
	if(tempTest > 100)
		tempTest = 0;
	sendMsg[MSG_LEN-3]='\r';
	sendMsg[MSG_LEN-2]='\n';
	sendMsg[MSG_LEN-1]=0;
	//开启DMA发送
	SPI_ReadWrite(SPI1,sendMsg,receiveMsg,MSG_LEN);
}
/**
	* @brief	ReadData 将spi读到的上位机数组内容解析给各个变量
  * @note		None
  * @param	n1：u8类型的标志位个数 
	* @param	n2：float类型的数据个数
	* @param	len: rx数组长度 必须等于n1+n2*4！！！
	* @param	rx：需要解析的数组首地址
	* @param	...：n1个u8类型变量取址，n2个float类型变量取址，注意个数！注意只能是u8和float类型变量的地址！！！
  * @retval	None
  */
int8_t ReadData(int n1,int n2,int len,uint8_t* rx, ...)
{
	va_list argptr;
	va_start(argptr,rx);
	
	union dataTrans
	{
		uint8_t data8[4];
		float dataf;
	}readDataTrans;
	
	if(n1 + n2 * 4 != len)
	{
		return -1;
	}
	
	if(n1 > 0)
	{
		for(int i = 0; i < n1; i++)
		{
			*va_arg(argptr,uint8_t*) = rx[i];
		}
	}
	if(n2 > 0)
	{
		for(int i = 0; i < n2; i++)
		{
			for(uint8_t j = 0; j < 4; j++)
			{
				readDataTrans.data8[j] = rx[n1 + i*4 + j];
			}
			*va_arg(argptr,float*) = readDataTrans.dataf;
		}
	}
	
	va_end(argptr);	
	return 1;
}
/**
	* @brief	WriteData 将各个变量存入spi需要给上位机发送的数据内容数组中
  * @note		None
  * @param	n1：u8类型的标志位个数 
	* @param	n2：float类型的数据个数
	* @param	len: rx数组长度 必须等于n1+n2*4！！！
	* @param	tx：需要存入的数组首地址
	* @param	...：n1个u8类型变量，n2个float类型变量，注意个数！注意只能是u8和float类型变量！！！
好像不能发送float,只能发送int
  * @retval	None
  */
int8_t WriteData(int n1,int n2,int len,uint8_t* tx, ...)
{
	va_list argptr;
	va_start(argptr,tx);
	
	union dataTrans
	{
		uint8_t data8[4];
		int data32;
	}writeDataTrans;
	
	if(n1 + n2 * 4 != len)
	{
		return -1;
	}
	
	if(n1 > 0)
	{
		for(int i = 0; i < n1; i++)
		{
			tx[i] = (uint8_t)(va_arg(argptr,int));
		}
	}
	if(n2 > 0)
	{
		for(int i = 0; i < n2; i++)
		{
			writeDataTrans.data32 = va_arg(argptr,int);
			for(uint8_t j = 0; j < 4; j++)
			{
				tx[n1 + i*4 + j] = writeDataTrans.data8[j];
			}
		}
	}
	
	va_end(argptr);
	return 1;
}

//检查spi通信状态
void CheckSpiState(void)
{
	//判断spi通信中断次数
	#define SPI_LOST_TIMES (20)
	
	if(gRobot.spiState.startTalkFlag == 1 && gRobot.pathPlanFlag == 0)
	{
		gRobot.spiState.spiHB++;
		gRobot.spiState.spiShiftHB++;
		
		if(gRobot.spiState.spiHB > SPI_LOST_TIMES)
		{
			gRobot.spiState.errorFlag = 1;
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
					(uint8_t *)("spiLOST! "));
		}
		if(gRobot.spiState.spiShiftHB > SPI_LOST_TIMES)
		{
			gRobot.spiState.errorFlag = 1;
			gRobot.spiState.errorShiftFlag = 1;
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
					(uint8_t *)("spiShiftLOST! "));
		}
		if(gRobot.spiState.spiOffArray == 1)
		{
			gRobot.spiState.errorFlag = 1;
		}
	}
}
uint8_t SPI_ReadWrite_SingleByte(SPI_TypeDef* SPIx,uint8_t writeData)
{

	/* Loop while DR register in not emplty */

	while(SPI_I2S_GetFlagStatus(SPIx,SPI_I2S_FLAG_TXE) == RESET);

	/* Send byte through the SPI1 peripheral */

	SPI_I2S_SendData(SPIx, writeData);

	/* Wait to receive a byte */

	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);

	/* Return the byte read from the SPI bus */

	return SPI_I2S_ReceiveData(SPIx);

}

void SPI_ReadWrite(SPI_TypeDef* SPIx,uint8_t* tx_data,uint8_t* rx_data,uint16_t length)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_4); 
	for(int i=0;i<length;i++)
		rx_data[i] = SPI_ReadWrite_SingleByte(SPIx,tx_data[i]);
	
	GPIO_SetBits(GPIOA,GPIO_Pin_4);

}






