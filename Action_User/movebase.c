#include "movebase.h"
#include "robot.h"
#include "driver.h"

driverMsg_t oneWheelMsg,twoWheelMsg,thrWheelMsg,oneTurnMsg,twoTurnMsg,thrTurnMsg,\
						turnTableMsg,loadArrowMsg,shootArrowMsg,archeryPitchMsg;

/**
* @brief  将轮子速度转换为动力电机的脉冲
* @note
* @param  vel:速度（mm/s）
* @param  turnMotorVelPulse:舵向电机速度脉冲
* @retval 脉冲速度
*/
int WheelVel2MotorPulse(float vel,float turnMotorVelPulse)
{
	int pulse = 0;
	pulse = Vel2Pulse(vel) + turnMotorVelPulse;
	return pulse;
}
/**
* @brief  将动力电机的脉冲转化为轮子速度
* @note
* @param  vel:速度（mm/s）
* @param  turnMotorVelPulse:舵向电机速度脉冲
* @retval 脉冲速度
*/
float MotorPulse2WheelVel(float velMotorVelPulse,float turnMotorVelPulse)
{
	float vel = 0;
	vel = Pulse2Vel(velMotorVelPulse) - Pulse2Vel(turnMotorVelPulse);
	return vel;
}
/**
* @brief  Vel2Pulse将速度转换为脉冲
* @note
* @param  vel:速度（mm/s）
* @retval 脉冲速度
*/
int Vel2Pulse(float vel)
{
	return (int)(vel/(PI*WHEEL_DIAMETER)*MOTOR_PULSE_PER_ROUND*WHEEL_RATIO);
}
/**
* @brief  Pulse2Vel将速度转换为脉冲
* @note
* @param  pulse:脉冲速度
* @retval 速度（mm/s）
*/
float Pulse2Vel(int pulse)
{
	return ((float)pulse/MOTOR_PULSE_PER_ROUND)/WHEEL_RATIO*PI*WHEEL_DIAMETER;
}
/**
* @brief 将航向角度转换为脉冲
* @note
* @param  angle:°
* @retval 
*/
int TurnAngle2Pulse(float angle,int rstPos)
{
	int pulse = 0;	
	pulse = angle/360.0f*MOTOR_PULSE_PER_ROUND*TURN_RATIO + rstPos;
	return pulse;
}
/**
* @brief	得到实际航向电机位置函数
* @note		None
* @param	pulse 脉冲
* @retval	返回此时的角度，单位：度
*/
float Pulse2TurnAngle(int pulse,int rstPos)
{
	float angle = 0;
	angle = (pulse - rstPos)/TURN_RATIO/MOTOR_PULSE_PER_ROUND*360.0f;
	return angle;
}

/**
* @brief	使能所有电机函数
  * @note	None
  * @param	None
  * @retval	None
  */
void AllMotorOn(void)
{
	DriverState(CAN1,PTP_MODE,ONE_TURN_ID,ENABLE);
	DriverState(CAN1,PTP_MODE,ONE_WHEEL_ID,ENABLE);
	
	DriverState(CAN1,PTP_MODE,TWO_TURN_ID,ENABLE);
	DriverState(CAN1,PTP_MODE,TWO_WHEEL_ID,ENABLE);
	
	DriverState(CAN1,PTP_MODE,THR_TURN_ID,ENABLE);
	DriverState(CAN1,PTP_MODE,THR_WHEEL_ID,ENABLE);
}
/**
* @brief	失能所有电机函数
  * @note	None
  * @param	None
  * @retval	None
  */
void AllMotorOff(void)
{
	DriverState(CAN1,PTP_MODE,ONE_TURN_ID,DISABLE);
	DriverState(CAN1,PTP_MODE,ONE_WHEEL_ID,DISABLE);
	
	DriverState(CAN1,PTP_MODE,TWO_TURN_ID,DISABLE);
	DriverState(CAN1,PTP_MODE,TWO_WHEEL_ID,DISABLE);
	
	DriverState(CAN1,PTP_MODE,THR_TURN_ID,DISABLE);
	DriverState(CAN1,PTP_MODE,THR_WHEEL_ID,DISABLE);
	
	DriverState(CAN2,PTP_MODE,SHOOT_ARROW_ID,DISABLE);
//	
	DriverState(CAN2,PTP_MODE,LOAD_ARROW_ID,DISABLE);
	
	DriverState(CAN2,PTP_MODE,TURN_TABLE_ID,DISABLE);
	
	DriverState(CAN2,PTP_MODE,ARCHERY_PITCH_ID,DISABLE);
}

/**
* @brief	停止所有电机函数
  * @note	None
  * @param	None
  * @retval	None
  */

void AllMotorStop(void)
{
	if(disableFlag == 1)//急停
	{
		static uint8_t disableCnt = 0;
		disableCnt++;
		if(disableCnt > 80)
		{
			GPIO_ResetBits(GPIOC, GPIO_Pin_12);
			disableCnt = 200;
		}
	}
	
//	gRobot.steerStruct.one.wheelVelTarget.pos = -60.0f;
//	gRobot.steerStruct.two.wheelVelTarget.pos = 0.0f;
//	gRobot.steerStruct.thr.wheelVelTarget.pos = 60.0f;
	
//	gRobot.steerStruct.one.motorVelTarget.pos = TurnAngle2Pulse(gRobot.steerStruct.one.wheelVelTarget.pos,RESET_POS_ONE);

//	gRobot.steerStruct.two.motorVelTarget.pos = TurnAngle2Pulse(gRobot.steerStruct.two.wheelVelTarget.pos,RESET_POS_TWO);

//	gRobot.steerStruct.thr.motorVelTarget.pos = TurnAngle2Pulse(gRobot.steerStruct.thr.wheelVelTarget.pos,RESET_POS_THR);
	
	VelCtrl(CAN1,PTP_MODE,ONE_WHEEL_ID,0);
//	PosCtrl(CAN1,PTP_MODE,ONE_TURN_ID,ABSOLUTE_MODE,-gRobot.steerStruct.one.motorVelTarget.pos);
			
	VelCtrl(CAN1,PTP_MODE,TWO_WHEEL_ID,0);
//	PosCtrl(CAN1,PTP_MODE,TWO_TURN_ID,ABSOLUTE_MODE,-gRobot.steerStruct.two.motorVelTarget.pos);
			
	VelCtrl(CAN1,PTP_MODE,THR_WHEEL_ID,0);
//	PosCtrl(CAN1,PTP_MODE,THR_TURN_ID,ABSOLUTE_MODE,-gRobot.steerStruct.thr.motorVelTarget.pos);
	
}

/**
* @brief	更新实际轮子速度
  * @note	None
  * @param	None
  * @retval	None
  */
void UpdataActWheelVel(void)
{
	gRobot.steerStruct.one.wheelVelAct.vel = MotorPulse2WheelVel(oneWheelMsg.vel,oneTurnMsg.vel);
	gRobot.steerStruct.one.wheelVelAct.pos = Pulse2TurnAngle(oneTurnMsg.pos,RESET_POS_ONE);
	
	gRobot.steerStruct.two.wheelVelAct.vel = MotorPulse2WheelVel(twoWheelMsg.vel,twoTurnMsg.vel);
	gRobot.steerStruct.two.wheelVelAct.pos = Pulse2TurnAngle(twoTurnMsg.pos,RESET_POS_TWO);
	
	gRobot.steerStruct.thr.wheelVelAct.vel = MotorPulse2WheelVel(thrWheelMsg.vel,thrTurnMsg.vel);
	gRobot.steerStruct.thr.wheelVelAct.pos = Pulse2TurnAngle(thrTurnMsg.pos,RESET_POS_THR);
}

/**
* @brief	发送速度和方向给电机
  * @note	None
  * @param	None
  * @retval	None
  */
#define LIMIT_VEL (10000.0f)
extern uint8_t disableFlag;
void SendCmd2Drivers(void)
{
	if(fabs(gRobot.steerStruct.one.wheelVelTarget.vel) > LIMIT_VEL)
	{
		gRobot.steerStruct.one.wheelVelTarget.vel = gRobot.steerStruct.one.wheelVelTarget.vel/(fabs(gRobot.steerStruct.one.wheelVelTarget.vel))*LIMIT_VEL;
	}
	if(fabs(gRobot.steerStruct.two.wheelVelTarget.vel) > LIMIT_VEL)
	{
		gRobot.steerStruct.two.wheelVelTarget.vel = gRobot.steerStruct.two.wheelVelTarget.vel/(fabs(gRobot.steerStruct.two.wheelVelTarget.vel))*LIMIT_VEL;
	}
	if(fabs(gRobot.steerStruct.thr.wheelVelTarget.vel) > LIMIT_VEL)
	{
		gRobot.steerStruct.thr.wheelVelTarget.vel = gRobot.steerStruct.thr.wheelVelTarget.vel/(fabs(gRobot.steerStruct.thr.wheelVelTarget.vel))*LIMIT_VEL;
	}
			
	if(gRobot.archeryStruct.shootArrow.motorVelTarget.vel > MAX_SHOOT_VEL*4096 )
	{
		gRobot.archeryStruct.shootArrow.motorVelTarget.vel = MAX_SHOOT_VEL*4096;
	}
	if(gRobot.archeryStruct.shootArrow.motorVelTarget.vel < -MAX_SHOOT_VEL*4096)
	{
		gRobot.archeryStruct.shootArrow.motorVelTarget.vel = -MAX_SHOOT_VEL*4096;
	}
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//	(uint8_t *)("TBU1 %d %d "),(int)(gRobot.archeryStruct.turnTable.Target.pos * 100),(int)(gRobot.archeryStruct.turnTable.motorVelTarget.pos));
	
	gRobot.steerStruct.one.motorVelTarget.vel = WheelVel2MotorPulse(gRobot.steerStruct.one.wheelVelTarget.vel,oneTurnMsg.vel);
	gRobot.steerStruct.one.motorVelTarget.pos = TurnAngle2Pulse(gRobot.steerStruct.one.wheelVelTarget.pos,RESET_POS_ONE);
	
	gRobot.steerStruct.two.motorVelTarget.vel = WheelVel2MotorPulse(gRobot.steerStruct.two.wheelVelTarget.vel,twoTurnMsg.vel);
	gRobot.steerStruct.two.motorVelTarget.pos = TurnAngle2Pulse(gRobot.steerStruct.two.wheelVelTarget.pos,RESET_POS_TWO);
	
	gRobot.steerStruct.thr.motorVelTarget.vel = WheelVel2MotorPulse(gRobot.steerStruct.thr.wheelVelTarget.vel,thrTurnMsg.vel);
	gRobot.steerStruct.thr.motorVelTarget.pos = TurnAngle2Pulse(gRobot.steerStruct.thr.wheelVelTarget.pos,RESET_POS_THR);
	
	gRobot.archeryStruct.turnTable.motorVelTarget.pos = gRobot.archeryStruct.turnTable.Target.pos * TURNTABLE_RATIO;
	gRobot.archeryStruct.archeryPitch.motorVelTarget.pos = gRobot.archeryStruct.archeryPitch.Target.pos * PITCH_RATIO;
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//	(uint8_t *)("TAU1 %d %d "),(int)(gRobot.archeryStruct.turnTable.Target.pos * 100),(int)(gRobot.archeryStruct.turnTable.motorVelTarget.pos));
	//射箭电机
	VelCtrl(CAN2,PTP_MODE,SHOOT_ARROW_ID, MINUS*gRobot.archeryStruct.shootArrow.motorVelTarget.vel);
	PosCtrl(CAN2, PTP_MODE, ARCHERY_PITCH_ID, ABSOLUTE_MODE, gRobot.archeryStruct.archeryPitch.motorVelTarget.pos);
//	PosCtrl(CAN2, PTP_MODE, TURN_TABLE_ID, ABSOLUTE_MODE, (gRobot.archeryStruct.turnTable.motorVelTarget.pos - (int)(60*TURNTABLE_RATIO)));
	PosCtrl(CAN2, PTP_MODE, TURN_TABLE_ID, ABSOLUTE_MODE, gRobot.archeryStruct.turnTable.motorVelTarget.pos);
	PosCtrl(CAN2, PTP_MODE, LOAD_ARROW_ID, ABSOLUTE_MODE, gRobot.archeryStruct.loadArrow.motorVelTarget.pos);
	
	//底盘电机
	VelCtrl(CAN1,PTP_MODE,ONE_WHEEL_ID,-gRobot.steerStruct.one.motorVelTarget.vel);
	PosCtrl(CAN1,PTP_MODE,ONE_TURN_ID,ABSOLUTE_MODE,-gRobot.steerStruct.one.motorVelTarget.pos);
	
	VelCtrl(CAN1,PTP_MODE,TWO_WHEEL_ID,-gRobot.steerStruct.two.motorVelTarget.vel);
	PosCtrl(CAN1,PTP_MODE,TWO_TURN_ID,ABSOLUTE_MODE,-gRobot.steerStruct.two.motorVelTarget.pos);
	
	VelCtrl(CAN1,PTP_MODE,THR_WHEEL_ID,-gRobot.steerStruct.thr.motorVelTarget.vel);
	PosCtrl(CAN1,PTP_MODE,THR_TURN_ID,ABSOLUTE_MODE,-gRobot.steerStruct.thr.motorVelTarget.pos);

}

/**
* @brief	运动时读取电机信息函数
  * @note	None
  * @param	None
  * @retval	None
  */
void ReadMotorMsg(void)
{

	ReadVel(CAN1, BROADCAST_MODE, 1);
	ReadPos(CAN1, BROADCAST_MODE, 1);
	ReadCurrQ(CAN1, BROADCAST_MODE, 1);
	
	ReadVel(CAN2,BROADCAST_MODE,1);
	ReadPos(CAN2,BROADCAST_MODE,1);
	ReadCurrQ(CAN2,BROADCAST_MODE,1);

  ReadVolIN(CAN2,PTP_MODE,SHOOT_ARROW_ID);
}


/**
  * @brief	位置初始化函数
  * @note	此函数为复位完成后转到初始位置
  * @param	None
  * @retval	到位置为1
			未到为0
  */
uint8_t PosInit(void)
{
	static uint16_t posInitStep = 0, posInitCnt = 0;
	
  gRobot.steerStruct.one.wheelVelTarget.vel = 0.0f;
	gRobot.steerStruct.two.wheelVelTarget.vel = 0.0f;
	gRobot.steerStruct.thr.wheelVelTarget.vel = 0.0f;

	if(posInitCnt > 100)
	{
		posInitCnt = 150;
		posInitStep = VEL_DIRECTION;
	}
	
	if(fabs(gRobot.steerStruct.one.wheelVelAct.pos-ONE_POS_INIT) < 5.0f && \
		fabs(gRobot.steerStruct.two.wheelVelAct.pos-TWO_POS_INIT) < 5.0f && \
		fabs(gRobot.steerStruct.thr.wheelVelAct.pos-THR_POS_INIT) < 5.0f )
		{
			posInitStep = VEL_DIRECTION;
		}

	
	switch (posInitStep)
	{
		case ZERO_RESET:
		{
			gRobot.steerStruct.one.wheelVelTarget.pos = ONE_ZERO_INIT;
			gRobot.steerStruct.two.wheelVelTarget.pos = TWO_ZERO_INIT;
			gRobot.steerStruct.thr.wheelVelTarget.pos = THR_ZERO_INIT;
			break;
		}
		case VEL_DIRECTION:
		{
			gRobot.steerStruct.one.wheelVelTarget.pos = ONE_POS_INIT;
			gRobot.steerStruct.two.wheelVelTarget.pos = TWO_POS_INIT;
			gRobot.steerStruct.thr.wheelVelTarget.pos = THR_POS_INIT;
			break;
		}
		default:
		{
			gRobot.steerStruct.one.wheelVelTarget.pos = ONE_ZERO_INIT;
			gRobot.steerStruct.two.wheelVelTarget.pos = TWO_ZERO_INIT;
			gRobot.steerStruct.thr.wheelVelTarget.pos = THR_ZERO_INIT;
			break;
		}
	}
		
	//待修正，转到电机一上电的初始位置
	gRobot.archeryStruct.shootArrow.motorVelTarget.vel = 0;
//	gRobot.archeryStruct.turnTable.Target.pos = 60;	
	gRobot.archeryStruct.turnTable.Target.pos = 0;	
//	gRobot.archeryStruct.archeryPitch.Target.pos = 55;
	gRobot.archeryStruct.archeryPitch.Target.pos = 45.0f;//上箭角度45
	
	gRobot.archeryStruct.turnTable.motorVelTarget.pos = gRobot.archeryStruct.turnTable.Target.pos * TURNTABLE_RATIO;
	gRobot.archeryStruct.archeryPitch.motorVelTarget.pos = gRobot.archeryStruct.archeryPitch.Target.pos * PITCH_RATIO;
  //TR初始身上有2支箭，期望射箭电机的脉冲,记得取消屏蔽
	#ifdef START_PLACE_ARCHERY
	gRobot.archeryStruct.loadArrow.motorVelTarget.pos = 0;
	#else
	gRobot.archeryStruct.loadArrow.motorVelTarget.pos = -(FIR_LOAD_POS - (33.0f/25.0f) * 4096 * 2);
	#endif
	
	
	//判断是否转到位置，角度小于1.0°，并且速度小于1000脉冲
	if(fabs(gRobot.steerStruct.one.wheelVelAct.pos-gRobot.steerStruct.one.wheelVelTarget.pos) < 1.0f && abs(oneTurnMsg.vel) < 2000 &&\
		fabs(gRobot.steerStruct.two.wheelVelAct.pos-gRobot.steerStruct.two.wheelVelTarget.pos) < 1.0f && abs(twoTurnMsg.vel) < 2000 &&\
		fabs(gRobot.steerStruct.thr.wheelVelAct.pos-gRobot.steerStruct.thr.wheelVelTarget.pos) < 1.0f && abs(thrTurnMsg.vel) < 2000)
//	if(fabs(gRobot.steerStruct.one.wheelVelAct.pos-gRobot.steerStruct.one.wheelVelTarget.pos) < 1.0f && abs(oneTurnMsg.vel) < 2000 )
		{
			posInitCnt++;
			if(posInitStep == VEL_DIRECTION)
			{
				return 1;
			}
		}
	
	return 0;
}

/**
* @brief 测试轮子和航向响应
  * @note	None
  * @param	None
  * @retval	None
  */
void WheelTurnTest(void)
{
  static int lightCnt = 0, lightMode = 0;
	static int wheelCnt = 0, turnTableCnt = 0;
	static int wheelMode = 0, turnTableMode = 0;

  lightCnt++;
  if(lightCnt > 100)
  {
    lightCnt = 0;
    lightMode++;
    lightMode %= 7; 
  }
  switch(lightMode)
  {
    case 0:
    {
      SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x01);//红
      break;
    }
    case 1:
    {
      SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x02);//蓝
      break;
    }
    case 2:
    {
      SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x03);//绿
      break;
    }
    case 3:
    {
      SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x04);//紫
      break;
    }
    case 4:
    {
      SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x05);//黄
      break;
    }
    case 5:
    {
      SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x06);//白
      break;
    }
    case 6:
    {
      SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x00);//关
      break;
    }
  }
	
	wheelCnt++;
	if(wheelCnt > 600)
	{
		wheelCnt = 0;
		wheelMode++;
		wheelMode %= 2;
	}
	switch(wheelMode)
	{
		case 0:
		{
			gRobot.steerStruct.one.wheelVelTarget.vel = 1000.0f;
			gRobot.steerStruct.two.wheelVelTarget.vel = 1000.0f;
			gRobot.steerStruct.thr.wheelVelTarget.vel = 1000.0f;
			break;
		}
		case 1:
		{
			gRobot.steerStruct.one.wheelVelTarget.vel = -1000.0f;
			gRobot.steerStruct.two.wheelVelTarget.vel = -1000.0f;
			gRobot.steerStruct.thr.wheelVelTarget.vel = -1000.0f;
			break;
		}
		default:
			break;
	}
	
	turnTableCnt++;
	if(turnTableCnt > 700)
	{
		turnTableCnt = 0;
		turnTableMode++;
		turnTableMode %= 4;
	}
	switch(turnTableMode)
	{
		case 0:
		{
			gRobot.steerStruct.one.wheelVelTarget.pos = 0.0f;
			gRobot.steerStruct.two.wheelVelTarget.pos = 0.0f;
			gRobot.steerStruct.thr.wheelVelTarget.pos = 0.0f;
			break;
		}
		case 1:
		{
			gRobot.steerStruct.one.wheelVelTarget.pos = 45.0f;
			gRobot.steerStruct.two.wheelVelTarget.pos = 45.0f;
			gRobot.steerStruct.thr.wheelVelTarget.pos = 45.0f;
			break;
		}
		case 2:
		{
			gRobot.steerStruct.one.wheelVelTarget.pos = 0.0f;
			gRobot.steerStruct.two.wheelVelTarget.pos = 0.0f;
			gRobot.steerStruct.thr.wheelVelTarget.pos = 0.0f;
			break;
		}
		case 3:
		{
			gRobot.steerStruct.one.wheelVelTarget.pos = -45.0f;
			gRobot.steerStruct.two.wheelVelTarget.pos = -45.0f;
			gRobot.steerStruct.thr.wheelVelTarget.pos = -45.0f;
			break;
		}
//		case 4:
//		{
//			gRobot.steerStruct.one.wheelVelTarget.pos = 225.0f;
//			gRobot.steerStruct.two.wheelVelTarget.pos = 225.0f;
//			gRobot.steerStruct.thr.wheelVelTarget.pos = 225.0f;
//			break;
//		}
//		case 5:
//		{
//			gRobot.steerStruct.one.wheelVelTarget.pos = 290.0f;
//			gRobot.steerStruct.two.wheelVelTarget.pos = 290.0f;
//			gRobot.steerStruct.thr.wheelVelTarget.pos = 290.0f;
//			break;
//		}
//		case 6:
//		{
//			gRobot.steerStruct.one.wheelVelTarget.pos = 360.0f;
//			gRobot.steerStruct.two.wheelVelTarget.pos = 360.0f;
//			gRobot.steerStruct.thr.wheelVelTarget.pos = 360.0f;
//			break;
//		}
//		case 7:
//		{
//			gRobot.steerStruct.one.wheelVelTarget.pos = 420.0f;
//			gRobot.steerStruct.two.wheelVelTarget.pos = 420.0f;
//			gRobot.steerStruct.thr.wheelVelTarget.pos = 420.0f;
//			break;
//		}
		default:
			break;
	}
}

/**
* @brief 测试转盘、上箭、俯仰、射箭响应
  * @note	None
  * @param	None
  * @retval	None
  */

int ArcheryTestMode = 0 ;
void ArcheryTest(void)
{
	static int turnTableCnt = 0, loadCnt = 0, pitchCnt = 0;
	static int turnTableMode = 0, loadMode = 0, pitchMode = 0;
	
	switch(	ArcheryTestMode )
	{
		case 0://转盘自检
		{
			turnTableCnt++;
			if( turnTableCnt > 150 )
			{
				turnTableCnt = 0;
				turnTableMode++;
			}
			if( turnTableMode >= 5 )
			{ 
				turnTableMode = 5;
			}
			switch( turnTableMode )
			{
				case 0:
				{
					gRobot.archeryStruct.turnTable.Target.pos = -30.0f;
					break;
				}
				case 1:
				{
					gRobot.archeryStruct.turnTable.Target.pos = 0.0f;
					break;
				}
				case 2:
				{
					gRobot.archeryStruct.turnTable.Target.pos = 30.0f;
					break;
				}
				case 3:
				{
					gRobot.archeryStruct.turnTable.Target.pos = 0.0f;
					break;
				}
				case 5:
				{
					ArcheryTestMode++;
					break;
				}
				default:
					break;
			}
			break;
		}
		case 1://俯仰角
		{			
			pitchCnt++;
			if(pitchCnt > 500)
			{
				pitchCnt = 0;
				pitchMode++;
				if(pitchMode >= 4 )
				{
					pitchMode = 4;
				}
			}
			switch(pitchMode)
			{
				case 0:
				{
					gRobot.archeryStruct.archeryPitch.Target.pos = 55.0f;//55
          if(fabs(gRobot.archeryStruct.archeryPitch.Act.pos - gRobot.archeryStruct.archeryPitch.Target.pos)<15.f)//俯仰接近射箭时的角度，抱死
          {
            SetWheelControlMode(CAN2,PTP_MODE,ARCHERY_PITCH_ID,1);
          }
          if(gRobot.archeryStruct.archeryPitch.Act.pos > 54.2f &&gRobot.archeryStruct.archeryPitch.Act.pos <= 55.f )
          {
            SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x01);//正常
          }
          else
          {
            SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x06);//未到位
          }
					break;
				}
				case 1:
				{
          SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x06);//未到位
          SetWheelControlMode(CAN2,PTP_MODE,ARCHERY_PITCH_ID,2);
					gRobot.archeryStruct.archeryPitch.Target.pos = 45.0f;//55
					break;
				}
				case 2:
				{
					gRobot.archeryStruct.archeryPitch.Target.pos = -105.0f;//55
          if(fabs(gRobot.archeryStruct.archeryPitch.Act.pos - gRobot.archeryStruct.archeryPitch.Target.pos)<4.5f)//俯仰接近射箭时的角度，抱死
          {
            SetWheelControlMode(CAN2,PTP_MODE,ARCHERY_PITCH_ID,1);
          }
          if(gRobot.archeryStruct.archeryPitch.Act.pos < -104.2f && gRobot.archeryStruct.archeryPitch.Act.pos >= -105.0f)
          {
            SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x01);//正常
          }
          else
          {
            SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x06);//未到位
          }
					break;
				}
        case 3:
				{
          SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x06);//未到位
          SetWheelControlMode(CAN2,PTP_MODE,ARCHERY_PITCH_ID,2);
					gRobot.archeryStruct.archeryPitch.Target.pos = 30.0f;//55
					break;
				}
				case 4:
				{
					ArcheryTestMode++;
					break;
				}
				default: 
					break;
			}
			break;
		}			
		case 2://上箭电机
		{
USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
			(uint8_t *)("L %d %d "),(int)(loadMode),(int)(loadCnt));
//			static float firstPos = -16661;
			loadCnt++;
			if(loadCnt > 100)
			{
				loadCnt = 0;
				loadMode++;
				if(loadMode >= 2)
				{
					loadMode = 2;
				}
			}
			switch(loadMode)
			{
//				case 0:
//				{
//					gRobot.archeryStruct.loadArrow.motorVelTarget.pos = -(firstPos - (33.0f/25.0f) * 4096 * 4);
//					break;
//				}
				case 0:
				{
          MAGNET_OFF;
					gRobot.archeryStruct.loadArrow.motorVelTarget.pos = (300.0f/25.f*4096);
					break;
				}
				case 1:
				{
					MAGNET_OFF;//关闭磁铁的吸力，为射箭准备
//          ArcheryTestMode++;
					break;
				}
				case 2:
				{
          MAGNET_OFF;
					ArcheryTestMode++;
					
					break;
				}
				default:
					break;
			}
			break;
		}
		case 3://射箭电机
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
			(uint8_t *)("DEBUG %d %d "),(int)(-shootArrowMsg.pos),(int)(FULL_CYCLE));
			if((-shootArrowMsg.pos) < (FULL_CYCLE - 100))
			{
				gRobot.archeryStruct.shootArrow.motorVelTarget.vel = 4096.0f * 2.0f;
			}
			else
			{
				gRobot.archeryStruct.shootArrow.motorVelTarget.vel = 0.f;
				ArcheryTestMode++;
			}
			break;	
		}
		case 4://自检完成
		{
			MAGNET_ON;//打开磁铁的吸力
		}
		default :ArcheryTestMode = 9;
			break;
	}
//	if(turnTableCnt > 300)
//	{
//		turnTableCnt = 0;
//		turnTableMode++;
//	}
//	if(turnTableMode >= 6)
//	{ 
//		turnTableMode = 6;
//	}
//	switch(turnTableMode)//转盘自检
//	{
//		case 0:
//		{
//			gRobot.archeryStruct.turnTable.Target.pos = 45.0f;
//			break;
//		}
//		case 1:
//		{
//			gRobot.archeryStruct.turnTable.Target.pos = 90.0f;
//			break;
//		}
//		case 2:
//		{
//			gRobot.archeryStruct.turnTable.Target.pos = 135.0f;
//			break;
//		}
//		case 3:
//		{
//			gRobot.archeryStruct.turnTable.Target.pos = 180.0f;
//			break;
//		}
//		case 4:
//		{
//			gRobot.archeryStruct.turnTable.Target.pos = 135.0f;
//			break;
//		}
//		case 5:
//		{
//			gRobot.archeryStruct.turnTable.Target.pos = 90.0f;
//			break;
//		}
//		case 6:
//		{
//			gRobot.archeryStruct.turnTable.Target.pos = 45.0f;
//			break;
//		}
//		default:
//			break;
//	}
	
//	static float firstPos = -16661;
//	static int fistCnt;
//	loadCnt++;
//	if(loadCnt > 300)
//	{
//		loadCnt = 0;
//		loadMode++;
//		if(loadMode >= 5)
//		{
//			loadMode = 5;
//		}
//	}
//	switch(loadMode)
//	{
//		case 0:
//		{
//			gRobot.archeryStruct.loadArrow.motorVelTarget.pos = -firstPos + (33.0f/25.0f) * 4096;
//			fistCnt++;
//			if(fistCnt > 150)
//			{
//				gRobot.archeryStruct.loadArrow.motorVelTarget.pos = -firstPos;
//			}
//			if(fistCnt >200)
//			{
//				fistCnt = 200;
//			}
//			gRobot.archeryStruct.loadArrow.motorVelTarget.pos = -(firstPos + (33.0f/25.0f) * 4096);
//			break;
//		}
//		case 1:
//		{
//			gRobot.archeryStruct.loadArrow.motorVelTarget.pos = -(firstPos - (33.0f/25.0f) * 4096);
//			break;
//		}
//		case 2:
//		{
//			gRobot.archeryStruct.loadArrow.motorVelTarget.pos = -(firstPos - (33.0f/25.0f) * 4096 * 2);
//			break;
//		}
//		case 3:
//		{
//			gRobot.archeryStruct.loadArrow.motorVelTarget.pos = -(firstPos - (33.0f/25.0f) * 4096 * 3);
//			break;
//		}
//		case 4:
//		{
//			gRobot.archeryStruct.loadArrow.motorVelTarget.pos = -(firstPos - (33.0f/25.0f) * 4096 * 4);
//			break;
//		}
//		case 5:
//		{
//			gRobot.archeryStruct.loadArrow.motorVelTarget.pos = -(-(300.0f/25.0f) * 4096);
//			break;
//		}
//		default:
//			break;
//	}
	
//	//俯仰角
//	pitchCnt++;
//	if(pitchCnt > 300)
//	{
//		pitchCnt = 0;
//		pitchMode++;
//		if(pitchMode >= 1)
//		{
//			pitchMode = 1;
//		}
//	}
//	switch(pitchMode)
//	{
//		case 0:
//		{
//			gRobot.archeryStruct.archeryPitch.Target.pos = 55.0f;
//			break;
//		}
//		case 1:
//		{
//			gRobot.archeryStruct.archeryPitch.Target.pos = 40.0f;
//			break;
//		}
//		default:
//			break;
//	}
	//射箭
//	shootCnt++;
//	if(shootCnt > 300)
//	{
//		shootCnt = 0;
//		shootMode++;
//		if(shootMode >= 1)
//		{
//			shootMode = 2;
//		}
//	}
//	switch(shootMode)
//	{
//		case 0:
//		{
//			gRobot.archeryStruct.shootArrow.motorVelTarget.vel = 0.0f;
//			break;
//		}
//		case 1:
//		{
//			gRobot.archeryStruct.shootArrow.motorVelTarget.vel = 4096.0f * 2.0f;
//			break;
//		}
//		default:
//		{
//			gRobot.archeryStruct.shootArrow.motorVelTarget.vel = 0.0f;
//			break;
//		}
//	}
}

/**
* @brief 俯仰电机位置环分每个周期阶跃给定
  * @note	None 
  * @param	tStep 阶跃角度(是大于0的值)  tPosTarget最后的目标角度
  * @retval	None
  */
void PitchPosSegmentStep(float tStep, float tPosTarget)
{
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("PPPP\n "));
	static float segmentTarget = 0;
	if(gRobot.archeryStruct.archeryPitch.Act.pos > gRobot.archeryStruct.archeryPitch.Target.pos)//目标位置比当前实际位置小，阶段值取负
	{
		if(tStep >= 0)
		{
			tStep = -tStep;
		}
	}
	if(fabs(gRobot.archeryStruct.archeryPitch.Act.pos - gRobot.archeryStruct.archeryPitch.Target.pos) > 1.0f)
	{
		segmentTarget = gRobot.archeryStruct.archeryPitch.Act.pos + tStep;
		if(fabs(gRobot.archeryStruct.archeryPitch.Act.pos - gRobot.archeryStruct.archeryPitch.Target.pos) < fabs(tStep))
		{
			segmentTarget = gRobot.archeryStruct.archeryPitch.Target.pos;
		}
		if(fabs(gRobot.archeryStruct.archeryPitch.Act.pos - segmentTarget) < fabs(tStep))
		{
//			PosCtrl(CAN2, PTP_MODE, ARCHERY_PITCH_ID, ABSOLUTE_MODE, (segmentTarget * PITCH_RATIO));
//			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("PI %d %d %d "),(int)(segmentTarget),(int)gRobot.archeryStruct.archeryPitch.Target.pos,(int)tStep);
		}
	}
}


