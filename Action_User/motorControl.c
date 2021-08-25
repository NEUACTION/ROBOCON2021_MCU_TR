#include "motorcontrol.h"
#include "movebase.h"
#include "robot.h"

/**
  * @brief	电机控制函数
  * @note	None
  * @param	None
  * @retval	None
  */
void MotorControl(void)
{
//	if(gRobot.steerStruct.motorState.errorFlag)
	if(gRobot.steerStruct.motorState.errorFlag || gRobot.spiState.errorFlag\
			 || disableFlag == 1)//出错//急停)//未通信成功
//	if(gRobot.steerStruct.motorState.errorFlag || disableFlag == 1)//出错//急停)//未通信成功
	{
		AllMotorStop();
	} 
	else
	{
//		PitchPosSegmentStep(5.0f,gRobot.archeryStruct.archeryPitch.motorVelTarget.pos);
		SendCmd2Drivers();
	}
}

//检查驱动器状态
void CheckDriverState(void)
{
	//判断驱动器连续CAN通信中断次数
	#define MOTOR_LOST_TIMES (20)
	
	//驱动器CAN通信心跳包自加
	gRobot.steerStruct.motorState.oneHB++;
	gRobot.steerStruct.motorState.twoHB++;
	gRobot.steerStruct.motorState.thrHB++;
	gRobot.steerStruct.motorState.oneTurnHB++;
	gRobot.steerStruct.motorState.twoTurnHB++;
	gRobot.steerStruct.motorState.thrTurnHB++;
	
	gRobot.archeryStruct.archeryMotorState.shootArrowHB ++;	
	gRobot.archeryStruct.archeryMotorState.turnTableHB ++;	
	gRobot.archeryStruct.archeryMotorState.archeryPitchHB ++;	
	gRobot.archeryStruct.archeryMotorState.loadArrowHB ++;	
	
	if(gRobot.steerStruct.motorState.oneHB > MOTOR_LOST_TIMES)
	{
		gRobot.steerStruct.motorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("onewheelLOST!"));
	}
	if(gRobot.steerStruct.motorState.twoHB > MOTOR_LOST_TIMES)
	{
		gRobot.steerStruct.motorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("twowheelLOST!"));
	}
	if(gRobot.steerStruct.motorState.thrHB > MOTOR_LOST_TIMES)
	{
		gRobot.steerStruct.motorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("thrwheelLOST!"));
	}
	if(gRobot.steerStruct.motorState.oneTurnHB > MOTOR_LOST_TIMES)
	{
		gRobot.steerStruct.motorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("oneturnLOST!"));
	}
	if(gRobot.steerStruct.motorState.twoTurnHB > MOTOR_LOST_TIMES)
	{
		gRobot.steerStruct.motorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("twoturnLOST!"));
	}
	if(gRobot.steerStruct.motorState.thrTurnHB > MOTOR_LOST_TIMES)
	{
		gRobot.steerStruct.motorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("thrturnLOST!"));
	}

	if(gRobot.archeryStruct.archeryMotorState.shootArrowHB > MOTOR_LOST_TIMES)
	{
		gRobot.archeryStruct.archeryMotorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("shootMotorLOST!"));
	}
	if(gRobot.archeryStruct.archeryMotorState.loadArrowHB > MOTOR_LOST_TIMES)
	{
		gRobot.archeryStruct.archeryMotorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("loadArrowMotorLOST!"));
	}
	if(gRobot.archeryStruct.archeryMotorState.archeryPitchHB > MOTOR_LOST_TIMES)
	{
		gRobot.archeryStruct.archeryMotorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("archeryPitchMotorLost!"));
	}
	if(gRobot.archeryStruct.archeryMotorState.turnTableHB > MOTOR_LOST_TIMES)
	{
		gRobot.archeryStruct.archeryMotorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("turnTableMotorLOST!"));
	}
	
	if( 1 == oneWheelMsg.hardFault)
	{
		gRobot.steerStruct.motorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"onewheelHardFault! ");		
	}
	if( 1 == twoWheelMsg.hardFault)
	{
		gRobot.steerStruct.motorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"twowheelHardFault! ");		
	}
	if( 1 == thrWheelMsg.hardFault)
	{
		gRobot.steerStruct.motorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"thrwheelHardFault! ");		
	}
	if( 1 == oneTurnMsg.hardFault)
	{
		gRobot.steerStruct.motorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"oneturnHardFault! ");		
	}
	if( 1 == twoTurnMsg.hardFault)
	{
		gRobot.steerStruct.motorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"twoturnHardFault! ");		
	}
	if( 1 == thrTurnMsg.hardFault)
	{
		gRobot.steerStruct.motorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"thrturnHardFault! ");		
	}
	if( 1 == shootArrowMsg.hardFault)
	{
		gRobot.archeryStruct.archeryMotorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"shootMotorHardFault! ");		
	}
	if( 1 == archeryPitchMsg.hardFault)
	{
		gRobot.archeryStruct.archeryMotorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"archeryPitchMotorHardFault! ");		
	}
	if( 1 == loadArrowMsg.hardFault)
	{
		gRobot.archeryStruct.archeryMotorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"loadArrowMotorHardFault! ");		
	}
	if( 1 == turnTableMsg.hardFault)
	{
		gRobot.archeryStruct.archeryMotorState.errorFlag = 1;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"turnTableMotorHardFault! ");		
	}
	
}
//检查驱动器状态
void CheckDriverRec(void)
{
	static int wheel1Vel = 0,wheel2Vel = 0,wheel3Vel = 0,turn1Vel = 0,turn2Vel = 0,turn3Vel = 0,turn1Pos = 0,turn2Pos = 0,turn3Pos = 0;
	static int wheel1VelLast = 0,wheel2VelLast = 0,wheel3VelLast = 0,turn1VelLast = 0,turn2VelLast = 0,turn3VelLast = 0,turn1PosLast = 0,turn2PosLast = 0,turn3PosLast = 0;
	static int wheel1VelCnt = 0,wheel2VelCnt = 0,wheel3VelCnt = 0,turn1VelCnt = 0,turn2VelCnt = 0,turn3VelCnt = 0,turn1PosCnt = 0,turn2PosCnt = 0,turn3PosCnt = 0;
	//获得新的电机信息
	wheel1Vel = oneWheelMsg.vel;
	wheel2Vel = twoWheelMsg.vel;
	wheel3Vel = thrWheelMsg.vel;
	turn1Vel = oneTurnMsg.vel;
	turn2Vel = twoTurnMsg.vel;
	turn3Vel = thrTurnMsg.vel;
	turn1Pos = oneTurnMsg.pos;
	turn2Pos = twoTurnMsg.pos;
	turn3Pos = thrTurnMsg.pos;

	if(wheel1Vel == wheel1VelLast) //判断相等的次数
	{
		wheel1VelCnt ++;
	}
	else
	{
		wheel1VelCnt  = 0;
	}
	if(wheel2Vel == wheel2VelLast) //判断相等的次数
	{
		wheel2VelCnt ++;
	}
	else
	{
		wheel2VelCnt  = 0;
	}
	if(wheel3Vel == wheel3VelLast) //判断相等的次数
	{
		wheel3VelCnt ++;
	}
	else
	{
		wheel3VelCnt  = 0;
	}
	if(turn1Vel == turn1VelLast) //判断相等的次数
	{
		turn1VelCnt ++;
	}
	else
	{
		turn1VelCnt  = 0;
	}
	if(turn2Vel == turn2VelLast) //判断相等的次数
	{
		turn2VelCnt ++;
	}
	else
	{
		turn2VelCnt  = 0;
	}
	if(turn3Vel == turn3VelLast) //判断相等的次数
	{
		turn3VelCnt ++;
	}
	else
	{
		turn3VelCnt  = 0;
	}	
	if(turn1Pos == turn1PosLast) //判断相等的次数
	{
		turn1PosCnt ++;
	}
	else
	{
		turn1PosCnt  = 0;
	}
	if(turn2Pos == turn2PosLast) //判断相等的次数
	{
		turn2PosCnt ++;
	}
	else
	{
		turn2PosCnt  = 0;
	}
	if(turn3Pos == turn3PosLast) //判断相等的次数
	{
		turn3PosCnt ++;
	}
	else
	{
		turn3PosCnt  = 0;
	}	
	//记录电机信息
	wheel1VelLast = wheel1Vel;
	wheel2VelLast = wheel2Vel;
	wheel3VelLast = wheel3Vel;
	turn1VelLast = turn1Vel;
	turn2VelLast = turn2Vel;
	turn3VelLast = turn3Vel;
	turn1PosLast = turn1Pos;
	turn2PosLast = turn2Pos;
	turn3PosLast = turn3Pos;		
	if(wheel1VelCnt > 10)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("wheel1Vel rec same! "));
	}		
	if(wheel2VelCnt > 10)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("wheel2Vel rec same! "));
	}
			
	if(wheel3VelCnt > 10)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("wheel3Vel rec same! "));
	}
	if(turn1VelCnt > 10)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("turn1Vel rec same! "));
	}		
	if(turn2VelCnt > 10)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("turn2Vel rec same! "));
	}
	if(turn3VelCnt > 10)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("turn3Vel rec same! "));
	}
	if(turn1PosCnt > 10)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("turn1Pos rec same! "));
	}		
	if(turn2PosCnt > 10)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("turn2Pos rec same! "));
	}
	if(turn3PosCnt > 10)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("turn3Pos rec same! "));
	}
}



