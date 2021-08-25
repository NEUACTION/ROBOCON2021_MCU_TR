/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "string.h"
#include <ucos_ii.h>
#include "app_cfg.h"
#include <math.h>
#include "usart.h"
#include "timer.h"
#include "can.h"
#include "gpio.h"
#include "spi.h"
#include "timer.h"
#include "robot.h"
#include "movebase.h"
#include "archery.h"
#include "dma.h"
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

union CanReceive
	{
		uint8_t data8[8];
		int data32[2];
		float dataf[2];
	}canReceiveMsg;
	
	extern uint8_t getRobotModeFlag;
	//收到三个航向顺时针为正 轮子朝后转为正
void CAN1_RX0_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	uint8_t canNodeId = 0;
	uint32_t StdId=0;
	uint8_t receiveLength = 0;
	uint8_t data8[8]={0};
	
	CAN_RxMsg(CAN1, &StdId, data8, &receiveLength);

	canNodeId = StdId - DRIVER_CLIENT_BASE_ID;
	
	switch(canNodeId)
	{
		case ONE_WHEEL_ID:
		{
			//接收can消息，数据转换，存储
			GetDriverMsg(&oneWheelMsg,data8);
			gRobot.steerStruct.motorState.oneHB = 0;
			if(data8[0] == IDENTIFIER_READ_VEL)
			{
				//获取脉冲速度
				oneWheelMsg.vel = -oneWheelMsg.vel;
			}
			if(data8[0] == IDENTIFIER_READ_POS)
			{
				//获取脉冲速度
				oneWheelMsg.pos = -oneWheelMsg.pos;
			}
			break;
		}
		case TWO_WHEEL_ID:
		{
			GetDriverMsg(&twoWheelMsg,data8);
			gRobot.steerStruct.motorState.twoHB = 0;
			if(data8[0] == IDENTIFIER_READ_VEL)
			{
				//获取脉冲速度
				twoWheelMsg.vel = -twoWheelMsg.vel;
			}
			if(data8[0] == IDENTIFIER_READ_POS)
			{
				//获取脉冲速度
				twoWheelMsg.pos = -twoWheelMsg.pos;
			}
			break;
		}
		case THR_WHEEL_ID:
		{
			GetDriverMsg(&thrWheelMsg,data8);
			gRobot.steerStruct.motorState.thrHB = 0;
			if(data8[0] == IDENTIFIER_READ_VEL)
			{
				//获取脉冲速度
				thrWheelMsg.vel = -thrWheelMsg.vel;
			}
			if(data8[0] == IDENTIFIER_READ_POS)
			{
				//获取脉冲速度
				thrWheelMsg.pos = -thrWheelMsg.pos;
			}
			break;
		}
		case ONE_TURN_ID:
		{
			GetDriverMsg(&oneTurnMsg,data8);
			gRobot.steerStruct.motorState.oneTurnHB = 0;
			if(data8[0] == IDENTIFIER_READ_VEL)
			{
				//获取脉冲速度
				oneTurnMsg.vel = -oneTurnMsg.vel;
			}
			if(data8[0] == IDENTIFIER_READ_POS)
			{
				//获取脉冲速度
				oneTurnMsg.pos = -oneTurnMsg.pos;
			}
			break;
		}
		case TWO_TURN_ID:
		{
			GetDriverMsg(&twoTurnMsg,data8);
			gRobot.steerStruct.motorState.twoTurnHB = 0;
			if(data8[0] == IDENTIFIER_READ_VEL)
			{
				//获取脉冲速度
				twoTurnMsg.vel = -twoTurnMsg.vel;
			}
			if(data8[0] == IDENTIFIER_READ_POS)
			{
				//获取脉冲速度
				twoTurnMsg.pos = -twoTurnMsg.pos;
			}
			break;
		}
		case THR_TURN_ID:
		{
			GetDriverMsg(&thrTurnMsg,data8);
			gRobot.steerStruct.motorState.thrTurnHB = 0;
			if(data8[0] == IDENTIFIER_READ_VEL)
			{
				//获取脉冲速度
				thrTurnMsg.vel = -thrTurnMsg.vel;
			}
			if(data8[0] == IDENTIFIER_READ_POS)
			{
				//获取脉冲速度
				thrTurnMsg.pos = -thrTurnMsg.pos;
			}
			break;
		}
		
	}
	

	CAN_ClearFlag(CAN1, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN1, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN1, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN1, CAN_FLAG_LEC);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV1);
	OSIntExit();
}

/**
  * @brief  CAN2 receive FIFO0 interrupt request handler
  * @note
  * @param  None
  * @retval None
  */
void CAN2_RX0_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	uint32_t StdId=0;
	uint8_t receiveLength = 0;
	uint8_t data8[8]={0};
	uint8_t canNodeId = 0;
	
	
	CAN_RxMsg(CAN2, &StdId, data8, &receiveLength);
	
	canNodeId = StdId - DRIVER_CLIENT_BASE_ID;
	
	switch(canNodeId)
	{
		case SHOOT_ARROW_ID:
		{
			gRobot.archeryStruct.archeryMotorState.shootArrowHB = 0;		
			GetDriverMsg(&shootArrowMsg,data8);
			break;
		}
		case LOAD_ARROW_ID:
		{
			gRobot.archeryStruct.archeryMotorState.loadArrowHB = 0;	
	
			GetDriverMsg(&loadArrowMsg,data8);
			break;
		}
		case ARCHERY_PITCH_ID:
		{
			gRobot.archeryStruct.archeryMotorState.archeryPitchHB = 0;		
			GetDriverMsg(&archeryPitchMsg,data8);
			break;
		}
		
		case TURN_TABLE_ID :
		{
			gRobot.archeryStruct.archeryMotorState.turnTableHB = 0;	
			GetDriverMsg(&turnTableMsg,data8);
//			turnTableMsg.pos += 157312;//(int)(60*TURNTABLE_RATIO);
			break;
		}
	}
	CAN_ClearFlag(CAN2, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN2, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN2, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN2, CAN_FLAG_LEC);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV1);
	OSIntExit();
}



/*************定时器2******start************/
//每1ms调用一次

extern OS_EVENT *PeriodSem;
extern uint32_t timeCnt;
void TIM2_IRQHandler(void)//100 us
{
	#define PERIOD_COUNTER 100

	//用来计数100次，产生100*100us=10ms的定时器
	static uint16_t periodCounter = PERIOD_COUNTER;
	
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		//实现10ms 发送1次信号量
		periodCounter--;
		timeCnt++;
		if (periodCounter == 0)
		{
			OSSemPost(PeriodSem);
			periodCounter = PERIOD_COUNTER;
		}
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM8_UP_TIM13_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM8, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM5_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
	OSIntExit();
}

extern OS_EVENT *ResetPeriodSem;
void TIM3_IRQHandler(void)
{
	#define RESET_PERIOD_COUNTER 100
	
	//用来计数100次，产生10ms的定时器
	static uint16_t resetPeriodCounter = RESET_PERIOD_COUNTER;
	
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	

	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		//实现5ms 发送1次信号量
		resetPeriodCounter--;
		if (resetPeriodCounter == 0)
		{
			OSSemPost(ResetPeriodSem);
			resetPeriodCounter = RESET_PERIOD_COUNTER;
		}
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM4_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
	OSIntExit();
}


uint8_t recData[10];
int dataLen;
typedef union
{
  uint8_t data[4];
	float   dataFloat;
}trans_t;
trans_t dataTrans;
int loadArrowCnt = 0;
extern uint8_t pitchDone;
extern uint8_t operaRelease;
extern uint8_t turnDone;
uint16_t appRecDelay;
vChangeFlag_t vChanFlag;
//APP接收
//接收桶消息的时候，操作手先点完，把桶号保存下来，应在从调整转到射箭阶段切换的时候把消息发给DR，需要注意通信时间，要是要射同一个桶，慢的一方直接放弃该桶，执行下面的桶的指令
void UART4_IRQHandler(void)
{
	static uint8_t msg;
	static uint8_t count = 0;
	static uint8_t number = 0;
	extern uint8_t loadArrowCircleCnt ;
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		msg = USART_ReceiveData(UART4);
		
		switch(number)
		{
			case 0:
			{
				if(msg == 'A' || msg == 'S' || msg == 'P')
				{
					recData[dataLen++] = msg;
					number++;
				}
				else
				{
					number = 0;
					clearData(recData,10);
				}
				break;
			}
			case 1:
			{
				if(msg == 'T')
				{
					recData[dataLen++] = msg;
					number++;
				}
				else
				{
					clearData(recData,10);
					number = 0;
				}
				break;
			}
			case 2:
			{
				recData[dataLen++] = msg;
				break;
			}
		}

		switch (count)
		{
			case 0:
			{
				if(2 == dataLen)
				{
					if(('A' == recData[0]) && ('T' == recData[1]))
					{
						count = 1;
					}
					else if(('S' == recData[0]) && ('T' == recData[1]))//设置速度、角度指令
					{
						count = 2;
					}
					else if(('P' == recData[0]) && ('T' == recData[1]))//桶号指令
					{
						count = 3;
					}
					else
					{
						clearData(recData,10);
						dataLen = 0;
						number = 0;
					}
				}		
			}
			break;
			
			case 1://其他指令
			{
				if(6 == dataLen)		
				{
					if(('\r' == recData[4]) && ('\n' == recData[5]))
					{	
						if(recData[2] == 'O' && recData[3] == 'F' && gRobot.archeryStruct.potRevFlag == 0)//切换为进攻模式
						{
							gRobot.archeryStruct.potRevFlag = 1;
							gRobot.walkMode = ATTACK_MODE;			
              gRobot.TRRetryFlag = 0;
						}
						else if(recData[2] == 'D' && recData[3] == 'F' && gRobot.archeryStruct.potRevFlag == 0)//切换为防守模式
						{
							gRobot.archeryStruct.potRevFlag = 1;
							gRobot.walkMode = DEFENCE_MODE;    	
              gRobot.TRRetryFlag = 0;
						}
						else if(recData[2] == 'D' && recData[3] == 'T' && gRobot.archeryStruct.potRevFlag == 0)//取箭
						{
							gRobot.archeryStruct.potRevFlag = 1;
							gRobot.archeryStruct.fetchMotorReady = 2;
						}
						else if(recData[2] == 'T' && recData[3] == 'P' && gRobot.archeryStruct.potRevFlag == 0)
						{
							gRobot.archeryStruct.potRevFlag = 1;
							disableFlag = 1;//失能
						}
						else if(recData[2] == 'R' && recData[3] == 'Y' && gRobot.archeryStruct.potRevFlag == 0)
						{
							gRobot.archeryStruct.potRevFlag = 1;
							gRobot.fieldCol = RED_FIELD;
						}
						else if(recData[2] == 'B' && recData[3] == 'Y' && gRobot.archeryStruct.potRevFlag == 0)
						{
							gRobot.archeryStruct.potRevFlag = 1;
							gRobot.fieldCol = BLUE_FIELD;
						}
						else if(recData[2] == 'S' && recData[3] == 'S' && gRobot.archeryStruct.potRevFlag == 0)//比赛开始
						{
							gRobot.archeryStruct.potRevFlag = 1;
							gRobot.contestStart = 1;
						}			
						else if(recData[2] == 'S' && recData[3] == 'T' && gRobot.archeryStruct.potRevFlag == 0)//手动开始射箭
						{
							gRobot.archeryStruct.potRevFlag = 1;
							appShootFlag = 1;
						}			
						else if(recData[2] == 'P' && recData[3] == 'A' && gRobot.archeryStruct.potRevFlag == 0)//手动射箭
						{
							gRobot.archeryStruct.potRevFlag = 1;
							gRobot.operationMode = OPERATION_MODE;
						}
						else if(recData[2] == 'O' && recData[3] == 'U' && gRobot.archeryStruct.potRevFlag == 0)//自动射箭
						{
							gRobot.archeryStruct.potRevFlag = 1;
              gRobot.turnOpera = 0;
							gRobot.operationMode = AUTOMATIC_MODE;
						}
						else if(('R' == recData[2]) && ('T' == recData[3]) && gRobot.archeryStruct.potRevFlag == 0)//搬车重试
						{
							 gRobot.archeryStruct.potRevFlag = 1;
               gRobot.initialCnt = 5;
               loadStep = 5;
               gRobot.walkMode = 0;
							 gRobot.TRRetryFlag = 1;
							 gRobot.retryLoad = 1;
						}
						else if(('E' == recData[2]) && ('R' == recData[3]) && gRobot.archeryStruct.potRevFlag == 0)//取箭重试，停在原处，等箭取下
						{
							 gRobot.archeryStruct.potRevFlag = 1;
							 gRobot.TRRetryFlag = 2;
						}
						else if(('C' == recData[2]) && ('R' == recData[3]) && gRobot.archeryStruct.potRevFlag == 0)//取箭重试，回到出发区
						{
							 gRobot.archeryStruct.potRevFlag = 1;
               gRobot.walkMode = 0;
							 gRobot.TRRetryFlag = 3;
						}
            else if(('R' == recData[2]) && ('F' == recData[3]) && gRobot.archeryStruct.potRevFlag == 0)//重新取箭(或点击射1、射2去对应的射箭区
						{
							 gRobot.archeryStruct.potRevFlag = 1;
               loadStep = 5;
							 gRobot.TRRetryFlag = 0;
						}
            else if(('R' == recData[2]) && ('R' == recData[3]) && gRobot.archeryStruct.potRevFlag == 0)//搬车重取
						{
							 gRobot.archeryStruct.potRevFlag = 1;
               loadStep = 5;
							 gRobot.TRRetryFlag = 4;
						}
            else if(('L' == recData[2]) && ('L' == recData[3]) && gRobot.archeryStruct.potRevFlag == 0)//开始上箭失能
						{
							 gRobot.archeryStruct.potRevFlag = 1;
               gRobot.loadEnable = 1;
						}
            else if(('N' == recData[2]) && ('E' == recData[3]) && gRobot.archeryStruct.potRevFlag == 0)//9号电池
						{
							 gRobot.archeryStruct.potRevFlag = 1;
               gRobot.batteryNum = 9;
						}
            else if(('E' == recData[2]) && ('I' == recData[3]) && gRobot.archeryStruct.potRevFlag == 0)//18号电池
						{
							 gRobot.archeryStruct.potRevFlag = 1;
               gRobot.batteryNum = 18;
						}
						else if(('S' == recData[2]) && ('G' == recData[3]) && gRobot.archeryStruct.potRevFlag == 0)//过箭
						{
							gRobot.archeryStruct.potRevFlag = 1;
//              if(gRobot.walkStatus == 0 && gRobot.TRRetryFlag == 1)//搬车重试初始箭的个数
//              {
//                gRobot.initialCnt++;
//              }
//							if(gRobot.shotArrowsCnt == 3 || gRobot.shotArrowsCnt == 8 || gRobot.shotArrowsCnt == 13 || gRobot.shotArrowsCnt == 18)
//              if((gRobot.shotArrowsCnt - gRobot.initialCnt) % 5 == 0)
//							{
//								gRobot.shotArrowsCnt++;	
//							}
//							else
							{
								loadFlag ++;
								gRobot.shotArrowsCnt++;	
							}								
						}
					}			
				clearData(recData,6);
				count = 0;
				number = 0;
				dataLen = 0;						
				}
			}
      break;			
			
			case 2://设置速度、角度指令
			{
				if(10 == dataLen)		
				{
					if('\r' == recData[8] && '\n' == recData[9])
					{
						for(int i=0;i<4;i++)//手调参射箭速度设置
						{
							dataTrans.data[i] = recData[4+i];
						}
						if((recData[2] == 'S') && (recData[3] == 'V') && gRobot.archeryStruct.potRevFlag == 0)
						{
							gRobot.archeryStruct.potRevFlag = 1;
							shootPara.shootVel = dataTrans.dataFloat;
//							shootRecFlag = 1;
							
						}
						else if(recData[2] == 'P' && recData[3] == 'I' && gRobot.archeryStruct.potRevFlag == 0)//kp
						{				
							gRobot.archeryStruct.potRevFlag = 1;
//							shootPara.archeryPitchAngle = dataTrans.dataFloat;	
							shootPara.shootKp = dataTrans.dataFloat;	
//							ptOperaDone = 0;
						}
						else if(recData[2] == 'T' && recData[3] == 'U' && gRobot.archeryStruct.potRevFlag == 0)//ki
						{
							gRobot.archeryStruct.potRevFlag = 1;
//							shootPara.turnTableAngle = dataTrans.dataFloat;
							shootPara.shootKi = dataTrans.dataFloat;
//							turnFlag = 1;	
						}
						else if(recData[2] == 'P' && recData[3] == 'A' && gRobot.archeryStruct.potRevFlag == 0)//俯仰角
						{				
							gRobot.archeryStruct.potRevFlag = 1;
							shootPara.archeryPitchAngle = dataTrans.dataFloat;	
							pitchDone = 0;
							operaRelease = 1;//手动调整俯仰，俯仰解除抱死标志位
						}
						else if((recData[2] == 'C') && (recData[3] == 'V') && gRobot.archeryStruct.potRevFlag == 0)//自动射箭界面手动调节转速
						{
							gRobot.archeryStruct.potRevFlag = 1;
							gRobot.velChange = dataTrans.dataFloat;
							gRobot.vChangeFlag = 1;
							
						}
            else if((recData[2] == 'R') && (recData[3] == 'N') && gRobot.archeryStruct.potRevFlag == 0)//手动调节转盘
						{
							 gRobot.archeryStruct.potRevFlag = 1;
               gRobot.turnOperaAdjust = dataTrans.dataFloat;
               gRobot.turnOpera = 1;
               turnDone = 0;
						}
            else if((recData[2] == 'T') && (recData[3] == 'N') && gRobot.archeryStruct.potRevFlag == 0)//手动调节视觉补偿角
						{
							 gRobot.archeryStruct.potRevFlag = 1;
               gRobot.cvCompensate = dataTrans.dataFloat;
//               gRobot.cvComFlag = 1;
               turnDone = 0;
						}
					}
					clearData(recData,10);
					clearData(dataTrans.data,4);
					count = 0;
					number = 0;
					dataLen = 0;	
				}
			}
			break;
			
			case 3:
			{
				if(6 == dataLen)		
				{
					if(('\r' == recData[4]) && ('\n' == recData[5]))
					{
						////////////////////
						if(gRobot.potNum != 0)
						{
							//防止重复收到指令，让桶号数量增加错误
							if(recData[2] == '1' && recData[3] == 'A')
							{
								if(gRobot.archeryStruct.pot1ARevFlag == 0)
								{
//									if(gRobot.vChangeFlag == 1)
//									{
//										gRobot.vChangeFlag = 0;
//										vChanFlag.vel1A = 1;
//									}
									gRobot.archeryStruct.pot1ARevFlag = 1;
									gRobot.attackPotID[gRobot.potNum] = P1A;
									gRobot.potNum++;
								}
							}
							else if(recData[2] == '2' && recData[3] == 'A')
							{
								if(gRobot.archeryStruct.pot2ARevFlag == 0)
								{
//									if(gRobot.vChangeFlag == 1)
//									{
//										gRobot.vChangeFlag = 0;
//										vChanFlag.vel2A = 1;
//									}
									gRobot.archeryStruct.pot2ARevFlag = 1;
									gRobot.attackPotID[gRobot.potNum] = P2A;
									gRobot.potNum++;
								}
							}
							else if(recData[2] == '1' && recData[3] == 'B')
							{
								if(gRobot.archeryStruct.pot1BRevFlag == 0)
								{
//									if(gRobot.vChangeFlag == 1)
//									{
//										gRobot.vChangeFlag = 0;
//										vChanFlag.vel1B = 1;
//									}
									gRobot.archeryStruct.pot1BRevFlag = 1;
									gRobot.attackPotID[gRobot.potNum] = P1B;
									gRobot.potNum++;
								}
							}
							else if(recData[2] == '2' && recData[3] == 'B')
							{
								if(gRobot.archeryStruct.pot2BRevFlag == 0)
								{
//									if(gRobot.vChangeFlag == 1)
//									{
//										gRobot.vChangeFlag = 0;
//										vChanFlag.vel2B = 1;
//									}
									gRobot.archeryStruct.pot2BRevFlag = 1;
									gRobot.attackPotID[gRobot.potNum] = P2B;
									gRobot.potNum++;
								}
							}
							else if(recData[2] == '3' && recData[3] == '3')
							{
								if(gRobot.archeryStruct.pot3RevFlag == 0)
								{
//									if(gRobot.vChangeFlag == 1)
//									{
//										gRobot.vChangeFlag = 0;
//										vChanFlag.vel3 = 1;
//									}
									gRobot.archeryStruct.pot3RevFlag = 1;
									gRobot.attackPotID[gRobot.potNum] = P3;
									gRobot.potNum++;
									if(gRobot.potNum > 64)
									{
										gRobot.potNum = 64;
									}
								}
							}
						}
						else if(gRobot.potNum == 0)
						{
							if(recData[2] == '2' && recData[3] == 'B' && gRobot.archeryStruct.pot2BRevFlag == 0)
							{
								gRobot.attackPotID[gRobot.potNum] = P2B;//进攻2号桶
								gRobot.archeryStruct.pot2BRevFlag = 1;
								gRobot.potNum++;
							}
							else if(recData[2] == '1' && recData[3] == 'A' && gRobot.archeryStruct.pot1ARevFlag == 0)
							{
								gRobot.attackPotID[gRobot.potNum] = P1A;//进攻3号桶
								gRobot.archeryStruct.pot1ARevFlag = 1;
								gRobot.potNum++;
							}
							else if(recData[2] == '1' && recData[3] == 'B' && gRobot.archeryStruct.pot1BRevFlag == 0)
							{
								gRobot.attackPotID[gRobot.potNum] = P1B;//进攻4号桶
								gRobot.archeryStruct.pot1BRevFlag = 1;
								gRobot.potNum++;
							}
							else if(recData[2] == '3' && recData[3] == '3' && gRobot.archeryStruct.pot3RevFlag == 0)
							{
								gRobot.attackPotID[gRobot.potNum] = P3;//进攻5号桶	
								gRobot.archeryStruct.pot3RevFlag = 1;		
								gRobot.potNum++;
							}
							else if(recData[2] == '2' && recData[3] == 'A' && gRobot.archeryStruct.pot2ARevFlag == 0)
							{
								gRobot.attackPotID[gRobot.potNum] = P2A;//进攻5号桶	
								gRobot.archeryStruct.pot2ARevFlag = 1;		
								gRobot.potNum++;
							}
							
						}						
				}			
					clearData(recData,6);
					count = 0;
					number = 0;
					dataLen = 0;						
				}
			}
      break;			

			default:
					clearData(recData,10);
					count = 0;
					number = 0;
					dataLen = 0;	
				break;
		}
	}
	else
	{
		USART_ClearITPendingBit(UART4, USART_IT_PE);
		USART_ClearITPendingBit(UART4, USART_IT_TXE);
		USART_ClearITPendingBit(UART4, USART_IT_TC);
		USART_ClearITPendingBit(UART4, USART_IT_ORE_RX);
		USART_ClearITPendingBit(UART4, USART_IT_IDLE);
		USART_ClearITPendingBit(UART4, USART_IT_LBD);
		USART_ClearITPendingBit(UART4, USART_IT_CTS);
		USART_ClearITPendingBit(UART4, USART_IT_ERR);
		USART_ClearITPendingBit(UART4, USART_IT_ORE_ER);
		USART_ClearITPendingBit(UART4, USART_IT_NE);
		USART_ClearITPendingBit(UART4, USART_IT_FE);
		USART_ReceiveData(UART4);
	}
	OSIntExit();
}
/***************************试场调参数用蓝牙串口中断*****************************************************/
void USART1_IRQHandler(void)
{

	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
	
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	OSIntExit();
}

void USART2_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	static uint8_t msg;
	static uint8_t count = 0;

	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		msg = USART_ReceiveData(USART2);
		recData[dataLen++] = msg;
		switch (count)
		{
			case 0:
			{
				if(2 == dataLen)
				{
					if(('A' == recData[0]) && ('T' == recData[1]))
					{
						count = 1;
					}
				}
				else
				{
					clearData(recData,10);
					dataLen = 0;
				}
			}
			case 1:
			{
				if(6 == dataLen)		
				{
					if(('\r' == recData[4]) && ('\n' == recData[5]))
					{
						if((recData[2] == '2' && recData[3] == 'B'))
						{
							gRobot.drPotID = P2B;//进攻2号桶
							gRobot.drPotRecieve = 1;//记得每次判断过DR桶号就清零一次
						}
						else if((recData[2] == '1' && recData[3] == 'A'))
						{
							gRobot.drPotID = P1A;//进攻2号桶
							gRobot.drPotRecieve = 1;//记得每次判断过DR桶号就清零一次
						}
						else if((recData[2] == '1' && recData[3] == 'B'))
						{
							gRobot.drPotID = P1B;//进攻2号桶
							gRobot.drPotRecieve = 1;//记得每次判断过DR桶号就清零一次
						}
						else if((recData[2] == '3' && recData[3] == '3'))
						{
							gRobot.drPotID = P3;//进攻2号桶
							gRobot.drPotRecieve = 1;//记得每次判断过DR桶号就清零一次				
						}
						else if((recData[2] == '2' && recData[3] == 'A'))
						{
							gRobot.drPotID = P2A;//进攻2号桶
							gRobot.drPotRecieve = 1;//记得每次判断过DR桶号就清零一次		
						}					
				}			
				clearData(recData,6);
				count = 0;
				dataLen = 0;						
				}
			}
      break;	
		}
	}
	else
	{
		USART_ClearITPendingBit(USART2, USART_IT_PE);
		USART_ClearITPendingBit(USART2, USART_IT_TXE);
		USART_ClearITPendingBit(USART2, USART_IT_TC);
		USART_ClearITPendingBit(USART2, USART_IT_ORE_RX);
		USART_ClearITPendingBit(USART2, USART_IT_IDLE);
		USART_ClearITPendingBit(USART2, USART_IT_LBD);
		USART_ClearITPendingBit(USART2, USART_IT_CTS);
		USART_ClearITPendingBit(USART2, USART_IT_ERR);
		USART_ClearITPendingBit(USART2, USART_IT_ORE_ER);
		USART_ClearITPendingBit(USART2, USART_IT_NE);
		USART_ClearITPendingBit(USART2, USART_IT_FE);
		USART_ReceiveData(USART2);
	}
	OSIntExit();
}

//两车通信
void USART3_IRQHandler(void) //更新频率200Hz
{
	static int potType;
	static uint8_t ch;
	static uint8_t count = 0;
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		ch = USART_ReceiveData(USART3);
		switch (count)
		{
		case 0:
			if (ch == 'A')
				count++;
			else
				count = 0;
			break;

		case 1:
			if (ch == 'T')
				count++;
			else
				count = 0;
			break;

		case 2:
		{
			switch (ch)
			{
				case 1:
				{
					potType = 1;
					break;
				}
				case 2:
				{
					potType = 2;
					break;
				}
				case 3:
				{
					potType = 3;
					break;
				}
				case 4:
				{
					potType = 4;
					break;
				}
				case 5:
				{
					potType = 5;
					break;
				}
			}
			
			count++;
			break;
		}
		case 3:
			if (ch == '\r')
				count++;
			else
				count = 0;
			break;

		case 4:
			if (ch == '\n')
			{
				switch (potType)
				{
					case 1:
					{
						gRobot.drPotID = P2A;
						gRobot.drPotRecieve = 1;//记得每次判断过DR桶号就清零一次
						break;
					}
					case 2:
					{
						gRobot.drPotID = P2B;
						gRobot.drPotRecieve = 1;
						break;
					}
					case 3:
					{
						gRobot.drPotID = P1B;
						gRobot.drPotRecieve = 1;
						break;
					}
					case 4:
					{
						gRobot.drPotID = P1A;
						gRobot.drPotRecieve = 1;
						break;
					}
					case 5:
					{
						gRobot.drPotID = P3;
						gRobot.drPotRecieve = 1;
						break;
					}
				}
			}
			count = 0;
			break;
		
		default:
			count = 0;
			break;
		}
	}
	else
	{
		USART_ClearITPendingBit(USART3, USART_IT_PE);
		USART_ClearITPendingBit(USART3, USART_IT_TXE);
		USART_ClearITPendingBit(USART3, USART_IT_TC);
		USART_ClearITPendingBit(USART3, USART_IT_ORE_RX);
		USART_ClearITPendingBit(USART3, USART_IT_IDLE);
		USART_ClearITPendingBit(USART3, USART_IT_LBD);
		USART_ClearITPendingBit(USART3, USART_IT_CTS);
		USART_ClearITPendingBit(USART3, USART_IT_ERR);
		USART_ClearITPendingBit(USART3, USART_IT_ORE_ER);
		USART_ClearITPendingBit(USART3, USART_IT_NE);
		USART_ClearITPendingBit(USART3, USART_IT_FE);
		USART_ReceiveData(USART3);
	}
	OSIntExit();
}



void UART5_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(UART5, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
	}
	else
	{
		USART_ClearITPendingBit(UART5, USART_IT_PE);
		USART_ClearITPendingBit(UART5, USART_IT_TXE);
		USART_ClearITPendingBit(UART5, USART_IT_TC);
		USART_ClearITPendingBit(UART5, USART_IT_ORE_RX);
		USART_ClearITPendingBit(UART5, USART_IT_IDLE);
		USART_ClearITPendingBit(UART5, USART_IT_LBD);
		USART_ClearITPendingBit(UART5, USART_IT_CTS);
		USART_ClearITPendingBit(UART5, USART_IT_ERR);
		USART_ClearITPendingBit(UART5, USART_IT_ORE_ER);
		USART_ClearITPendingBit(UART5, USART_IT_NE);
		USART_ClearITPendingBit(UART5, USART_IT_FE);
		USART_ReceiveData(UART5);
	}
	OSIntExit();
}

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
	while (1)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"NMI Exception! ");
	}
}

void Hex_To_Str(uint8_t * pHex,char * s,float num)
{
  char        hex[] = "0123456789ABCDEF";
  char        *pStr = s;
  for (uint8_t i = 0; i < (int)(num/2.f+0.5f); i++)//(int)(x+0.5f)是把x四舍五入的意思
  {
    
    /*
    1.*pStr++右结合,并且*索引的是没有++之前的地址
    2.f.移位不会改变指针指向的那个空间的值
    3.对指针指向空间的移位也不会改变指针的指向
    */
    if (((num<((int)(num / 2.f + 0.5f))*2.f)&&i>0)|| (num==((int)(num / 2.f + 0.5f)) * 2.f))
      *pStr++ = hex[*(pHex + (int)(num / 2.f + 0.5f) - i - 1) >> 4];
    *pStr++ = hex[*(pHex + (int)(num / 2.f + 0.5f) - i - 1) & 0x0F];
  }
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{

	/*屏蔽掉的只是为了返回进硬件中断的错误语句，但是比赛的时候不可能出现错误，只可能有静电等突发因素，所以不用返回了*/
	static uint32_t r_sp ;

	AllMotorStop();

	/*判断发生异常时使用MSP还是PSP*/
	if(__get_PSP()!=0x00) //获取SP的值
		r_sp = __get_PSP(); 
	else
		r_sp = __get_MSP(); 
	
	/*因为经历中断函数入栈之后，堆栈指针会减小0x10，所以平移回来（可能不具有普遍性）*/
	r_sp = r_sp+0x10;
	
	/*串口发数通知*/
	char sPoint[2]={0};
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
	(uint8_t *)"%s","0x");
	
	/*获取出现异常时程序的地址*/
	for(int i=3;i>=-28;i--){
		Hex_To_Str((uint8_t*)(r_sp+i+28),sPoint,2);
		
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"%s",sPoint);
		
		if(i%4==0)
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"\r\n");		
	}
	/*发送回车符*/
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
	(uint8_t *)"\r\n");		
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"Hard Fault\r\n");
		
		AllMotorStop();
		
		delay_ms(10);
	}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"Memory Manage Exception! ");
	}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{

	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"Bus Fault Exception! ");	
	}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{

	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"Usage Fault Exception! ");
	}
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"SVCall Exception! ");
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"Debug Monitor Exception! ");
}

