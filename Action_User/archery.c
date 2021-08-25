#include "movebase.h"
#include "archery.h"
#include "stdint.h"
#include "robot.h"
#include "usart.h"
#include "pot.h"
#include "pid.h"
#include "adc.h"
#include "APP.h"


extern int waitForFetchArrow;

uint8_t loadArrowCircleCnt = 0;
uint8_t loadFlag;
uint8_t enableState;

uint8_t countPro;
uint8_t loadStep;
uint8_t shootStep;
uint8_t visionStep;


uint8_t turnFlag;
uint8_t waveFlag;
uint8_t shootFlag;
uint8_t visionFlag;
uint8_t disableFlag;

int down;


int resetCount;
int visionCount;
int waitStableCount;
int waitUnstableCount;


int appShootFlag = 0;
int appLoadFlag = 0;
int resetCheckPos = 0;
int32_t errorPos;
int32_t lastPos;
int32_t aimPos;







////////////////////////////////////////////////////////////////修改后的变量
para_t shootPara={0};

float acc;
float accOne;

float voltageRead;
float voltageTemp;

uint8_t resetDone;//复位完成标志位
uint8_t potAssDone;//桶号的转盘、俯仰、转速赋值

uint8_t pitchDone = 0;//转盘、俯仰、视觉调整
uint8_t turnDone = 0;//转盘、俯仰、视觉调整
uint8_t comDone = 0;//通信发送成功

uint8_t operaRelease;
int recAppPitch;
uint8_t canShootFlag = 0;


float Wtable;
extern int shootRecFlag;

int loadDoneFlag;
int appSendOnce;

float temporaryValue = 0.0f;
uint8_t loadMotorDone;


void ShootArrow(void)
{
	//射箭个数限幅
	if(gRobot.shootOrder > 64)
	{
		gRobot.shootOrder = 64;
	}
	
	//计算转盘的世界坐标系角度（发给视觉，用遮挡桶程序）
	Wtable = angleLimit(gRobot.ppsData.angle + 120 + gRobot.archeryStruct.turnTable.Act.pos);//转盘在世界坐标系下的角度
	
	/*******************************************///	数据处理，ADC读取电池电压，发给平板
//	voltageRead = Get_Adc_Average(8, 20);
//	voltageTemp = voltageRead * (3.3/4096) * 9.86;
    voltageTemp = shootArrowMsg.VolIn/1000.f;
	
	//操作模式自动与手操的加速度计算赋值
	#ifdef BIG_SHOOT
		acc= shootPara.shootVel*4096 * shootPara.shootVel*4096 / 2.0f / (DOWN_POS - shootPara.shootVel*4096*0.025f);//复位时的减加速度
	#else
		acc= shootPara.shootVel*4096 * shootPara.shootVel*4096 / 2.0f / (FOWARD_POS);
	#endif
	
/*******************************************/
	switch(shootStep)//射箭的不同阶段
	{
		case COLLIMATION_STEP://瞄准
		{							
			//gRobot.potNum比点的桶数量多1
			
			//////////////////////////////////////////////////////////////////////////////////复位///////////////////////////////////////////////////////////////////////////////////////////////////////
              //滑块复位
        //开电第一支箭无需复位
     // if(gRobot.shotArrowsCnt == 0)
			if(gRobot.shotArrowsCnt == 0)
      {
        resetDone = 1;
      }
      else if(resetDone == 0)
      {
        resetDone = ResetArchery();
      }
			
			//只有车停在射箭区或出发区的时候射箭
			if(gRobot.archeryStruct.archeryStart == 1 && waitForFetchArrow == 1)
			{

        if(gRobot.attackPotID[gRobot.shootOrder] == 0)
        {
          gRobot.writeFlag = 0;
        }	
        else
        {
          gRobot.writeFlag = 2;
        }
        
				//完成上箭
				if(loadDoneFlag == 0)
				{
					loadDoneFlag = LoadAction();
				}
				
				//判断是否有指定桶，如无：等待，如有：调节俯仰、转盘参数
				if(gRobot.shootOrder <= gRobot.potNum - 1)
				{
					//为下一箭的桶号对应的转盘、俯仰、转速赋值
					if(potAssDone == 0)
					{
						potAssDone = PotAssignment();
//							ArcheryPitchSelect();
						temporaryValue = shootPara.archeryPitchAngle;
						shootPara.archeryPitchAngle = LOAD_PITCH_ANG;
						//设置Kp、Ki
						shootPara.shootKp = 1.5f;
						shootPara.shootKi = 5.0f;
					}
					
					//手动设置俯仰角度时，解除抱死
					if(operaRelease == 1)
					{
						SetWheelControlMode(CAN2,PTP_MODE,ARCHERY_PITCH_ID,2);
						operaRelease = 0;
					}
					
					//上箭完成后再进行俯仰调整
					if(pitchDone == 0 && loadDoneFlag == 1)
					{
						if(recAppPitch == 0)
						{
							shootPara.archeryPitchAngle = temporaryValue;
							recAppPitch = 1;
						}
						pitchDone = PitchAdjust();
					}
          
          //将要射的桶号发送DR
//          if(comDone == 0)
//          {
//            comDone = RobotComm();
//          }
					
					
					//转盘、视觉调整
					if(turnDone == 0)
					{
						turnDone = TurnAdjust();
					}
				}
				else;
			}
			
			/////////////////////////////////////////////////////////////////////////////进入射箭阶段///////////////////////////////////////////////////////////////////////////////////////////////
			//进入射箭前的准备
			if(gRobot.contestStart == 1 && pitchDone == 1 && turnDone == 1 && resetDone == 1 && loadMotorDone == 1)
			{
				
				if(gRobot.operationMode == AUTOMATIC_MODE)
				{
						if(gRobot.vChangeFlag == 1)
						{
								shootPara.shootVel = shootVelGet(voltageTemp,gRobot.attackPotID[gRobot.shootOrder]) + gRobot.velChange;
								gRobot.vChangeFlag = 0;
						}
						else
						{
								shootPara.shootVel = shootVelGet(voltageTemp,gRobot.attackPotID[gRobot.shootOrder]);
						}
						
						if(appShootFlag == 1 && gRobot.shotArrowsCnt < 3)
						{	
							canShootFlag = 1;
						}
						else if(gRobot.shotArrowsCnt >= 3)
						{
							canShootFlag = 1;
						}
				}
				else if(gRobot.operationMode == OPERATION_MODE)
				{
					if(appShootFlag == 1)
					{
						canShootFlag = 1;
					}
				}
        SendData2BlueTooth(voltageTemp,shootPara.shootVel,gRobot.cvStruct.cvDir,gRobot.cvStruct.cvDis);
				if(canShootFlag == 1)  
				{
					gRobot.writeFlag = 0;
					MAGNET_OFF;//关闭磁铁
					ShootMotorSet();//电机设置，自动模式需要选择
					ValueReset();
//					accOne= shootPara.shootVel*4096 * shootPara.shootVel*4096 / 2.0f / (CHANG_POS - shootPara.shootVel*4096*0.012f);
					accOne= shootPara.shootVel*4096 * shootPara.shootVel*4096 / 2.0f / (CHANG_POS);
					SetAccDec(CAN2, PTP_MODE, SHOOT_ARROW_ID, accOne/4096*512, accOne/4096*512);
          DriverState(CAN2,PTP_MODE,LOAD_ARROW_ID,DISABLE);
					shootStep = SHOOT_STEP;
				}
				
			}

			gRobot.archeryStruct.archeryPitch.Target.pos = shootPara.archeryPitchAngle;
			gRobot.archeryStruct.turnTable.Target.pos = shootPara.turnTableAngle;
//					gRobot.archeryStruct.shootArrow.motorVelTarget.vel = 0;	
		}
		break;
		
		case SHOOT_STEP: //射箭
		{
			gRobot.writeFlag = 1;
			MAGNET_OFF;
//			SetWheelControlMode(CAN2,PTP_MODE,ARCHERY_PITCH_ID,1);//给俯仰转换一个时间(需要每个周期都连续发才能保持稳定，只发一次会产生抖动）
			#ifdef BIG_SHOOT
					ShootBigStep();
			#else
					ShootSegmentStep();
			#endif
			if(shootArrowMsg.pos*MINUS > CHANG_POS)
			{ 
				#ifdef BIG_SHOOT
						SetSpeedKP(CAN2, PTP_MODE, SHOOT_ARROW_ID, 1.2*1000);//2
						SetSpeedKI(CAN2, PTP_MODE, SHOOT_ARROW_ID, 2.0*1000);//5;
						SetAccDec(CAN2, PTP_MODE, SHOOT_ARROW_ID, 8191*512, 8191*512);
				#else
						SetSpeedKP(CAN2, PTP_MODE, SHOOT_ARROW_ID, 0.85*1000);//2
						SetSpeedKI(CAN2, PTP_MODE, SHOOT_ARROW_ID, 1.3*1000);//5
				#endif
				MAGNET_ON;
				appShootFlag = 0;
        gRobot.archeryStruct.fetchMotorReady = 0;//防止误点射箭
				shootStep= RESETandWALK_STEP;
				
				#ifndef BIG_SHOOT
				gRobot.archeryStruct.shootArrow.motorVelTarget.vel=0;//复位时直接冲回来
				#endif
			}
		}
		break;
		
		case RESETandWALK_STEP:
		{
			static int enableCnt = 0;
			MAGNET_ON;
			enableCnt++;
			gRobot.writeFlag = 0;
			
			#ifdef BIG_SHOOT
					SetAccDec(CAN2, PTP_MODE, SHOOT_ARROW_ID, 8191*512, 8191*512);
					gRobot.archeryStruct.shootArrow.motorVelTarget.vel -= acc * 0.01f;
					if(gRobot.archeryStruct.shootArrow.motorVelTarget.vel <= 2*4096 || shootArrowMsg.pos*MINUS > FULL_CYCLE - 1800)
					{
						gRobot.archeryStruct.shootArrow.motorVelTarget.vel = 1*4096;
            SetClearIntegral(CAN2,LOAD_ARROW_ID);
					}
					if(enableCnt > 5 && enableCnt <=8)
					{
						SetWheelControlMode(CAN2,PTP_MODE,ARCHERY_PITCH_ID,2);
            SetClearIntegral(CAN2,LOAD_ARROW_ID);
            DriverState(CAN2,PTP_MODE,LOAD_ARROW_ID,ENABLE); 
					}
					if(enableCnt > 9)
					{
						motorSetAfterReset();//失能后三个周期使能射箭电机，俯仰解除抱死，轮子清除积分
						gRobot.shotArrowsCnt++;
						gRobot.shootOrder++;

						//保证取箭前不进行瞄准
            #ifdef SHOOT_TEST
            if(gRobot.shotArrowsCnt == gRobot.initialCnt)
            #else
            if((gRobot.shotArrowsCnt - gRobot.initialCnt) % 5 == 0)
            #endif
            {
              waitForFetchArrow = 0;
            }
						shootStep = COLLIMATION_STEP;
						enableCnt = 0;
					}
			#else
				if(enableCnt > 3 && enableCnt <=10)
				{
					DriverState(CAN2,PTP_MODE,SHOOT_ARROW_ID,ENABLE);
				}
				if(enableCnt > 8 && enableCnt <=10)
				{
					SetWheelControlMode(CAN2,PTP_MODE,ARCHERY_PITCH_ID,2);
				}
				if(enableCnt > 10)
				{
					motorSetAfterReset();//失能后三个周期使能射箭电机，俯仰解除抱死，轮子清除积分
					gRobot.shotArrowsCnt++;
					gRobot.shootOrder++;
					if((gRobot.shotArrowsCnt - gRobot.initialCnt) % 5 == 0)
					{
						waitForFetchArrow = 0;
					}
//					RobotComm();//将要射的桶号发送DR
					shootStep = COLLIMATION_STEP;
					enableCnt = 0;
				}
		#endif
		} 
		break;
		
		default:
		break;
	}
	//速度限幅
	if(gRobot.archeryStruct.shootArrow.motorVelTarget.vel > MAX_SHOOT_VEL*4096 )
	{
		gRobot.archeryStruct.shootArrow.motorVelTarget.vel = MAX_SHOOT_VEL*4096;
	}
	if(gRobot.archeryStruct.shootArrow.motorVelTarget.vel < -MAX_SHOOT_VEL*4096)
	{
		gRobot.archeryStruct.shootArrow.motorVelTarget.vel = -MAX_SHOOT_VEL*4096;
	}
} 

uint8_t fetchPre;
/*********************************************************************
* @brief 取箭前准备的动作及取完箭上箭动作函数
* @note  
* @param  
* @retval 
*********************************************************************/
void FetchAndLoadAction(void)
{
  static uint8_t retryMotorSet = 0;
	//收到APP的取箭指令
//	if(FetchArrowFlg == 1)
//	{
//		gRobot.archeryStruct.fetchMotorReady = 1;//告诉APP已经准备好取箭了
//	}

	//重试取箭时俯仰不抱死
	if(gRobot.TRRetryFlag == 2 && retryMotorSet < 3)
	{
    retryMotorSet++;
		DriverState(CAN2,PTP_MODE,LOAD_ARROW_ID,ENABLE);
		DriverState(CAN2,PTP_MODE,TURN_TABLE_ID,ENABLE);
		SetWheelControlMode(CAN2,PTP_MODE,ARCHERY_PITCH_ID,2);	
	}
  if(gRobot.TRRetryFlag != 2)
  {
     retryMotorSet = 0;
  }
	//开始取箭
	if((gRobot.walkStatus == 1 && gRobot.ppsData.y > 2000) || gRobot.walkStatus == 2 || gRobot.walkStatus == 5 || gRobot.walkStatus == 9 || gRobot.walkStatus == 21)
	{
		potAssDone = 0;
    if(gRobot.ppsData.y > 11350.0f && gRobot.ppsData.x < 1000.0f)
    {
      SetWheelControlMode(CAN2, PTP_MODE, ARCHERY_PITCH_ID, 1);//俯仰转到-104，抱死
    }
		if(FetchPre(1))//取箭姿态
		{
			if(gRobot.ppsData.y > 11380.0f && gRobot.ppsData.x < 900.0f)
			{
				SetWheelControlMode(CAN2, PTP_MODE, ARCHERY_PITCH_ID, 1);//俯仰转到-104，抱死
			}
			if(gRobot.ppsData.y > 11300.f && gRobot.ppsData.x < 1500.f)
			{
				DriverState(CAN2,PTP_MODE,LOAD_ARROW_ID,DISABLE);
				DriverState(CAN2,PTP_MODE,TURN_TABLE_ID,DISABLE);
			}
		}			
		gRobot.archeryStruct.fetchMotorReady = 0;
	}
	else
	{
		FetchPre(0);
	}
	
	//取箭完成，恢复
	if(gRobot.walkStatus == 3 || gRobot.walkStatus == 4 || gRobot.walkStatus == 7 || gRobot.walkStatus == 8 || gRobot.walkStatus == 11 || gRobot.walkStatus == 15 || gRobot.walkStatus == 32 || gRobot.walkStatus == 42 || gRobot.walkStatus == 40
	 || (gRobot.retryLoad == 1 && (gRobot.walkStatus == 40 || gRobot.walkStatus == 42)))
	{
		LoadPre(1);	
	}
	else
	{
		LoadPre(0);
	}
}

/*********************************************************************
* @brief 取箭俯仰、转盘角度变化函数
* @note  
* @param  
* @retval 
*********************************************************************/
uint8_t FetchPre(uint8_t startFetch)
{
	static uint8_t fetchMode = 0;
	//不进行取箭
	if(startFetch == 0)
	{
		fetchMode = 0;//将上箭阶段重新赋值为0，准备下一次取箭的准备动作
		return 0;
	}
	else
	{
USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("FM %d "),(int)fetchMode);
		MAGNET_ON;
		switch(fetchMode)//分两段上箭，不让俯仰电机转过大角度
		{
			case 0:
			{
				shootPara.turnTableAngle = -120.f;//240
				gRobot.archeryStruct.turnTable.Target.pos	= shootPara.turnTableAngle;
				
				shootPara.archeryPitchAngle = -105.5f;
				gRobot.archeryStruct.archeryPitch.Target.pos = shootPara.archeryPitchAngle;

				if(fabs(gRobot.archeryStruct.archeryPitch.Act.pos -shootPara.archeryPitchAngle) < 15.f &&\
				fabs( gRobot.archeryStruct.turnTable.Act.pos - shootPara.turnTableAngle) < 1.0f)	
				{
					fetchMode = 1;
				}				
				break;
			}
			case 1:
			{
					return 1;
			}	
	
		}
		return 0;
	}
}

uint8_t LoadPre(uint8_t startLoad)
{
	static uint8_t prepareMode = 0;
  if(loadStep != 5 && prepareMode == 0)
  {
    prepareMode = 2;
  }
	//不进行上箭
	if(startLoad == 0)
	{
		prepareMode = 0;//将上箭阶段重新赋值为0，准备下一次取箭的准备动作
		return 0;
	}
	else
	{
		switch(prepareMode)//分两段
		{
			case 0: //恢复转盘和俯仰
			{
				if(gRobot.ppsData.x <= 950)
				{
					SetWheelControlMode(CAN2,PTP_MODE,ARCHERY_PITCH_ID,2);
				}
				if(gRobot.ppsData.x >= 780 && gRobot.ppsData.x <= 1000)
				{
					DriverState(CAN2,PTP_MODE,LOAD_ARROW_ID,ENABLE);
					DriverState(CAN2,PTP_MODE,TURN_TABLE_ID,ENABLE);
					SetWheelControlMode(CAN2,PTP_MODE,ARCHERY_PITCH_ID,2);
				}
				if( gRobot.ppsData.y < 11350)
				{
					shootPara.archeryPitchAngle = LOAD_PITCH_ANG;
					gRobot.archeryStruct.archeryPitch.Target.pos = LOAD_PITCH_ANG;
          if(gRobot.walkStatus == 8 || gRobot.walkStatus == 7 || gRobot.walkStatus == 40)//1
          {
            shootPara.turnTableAngle = 0.0f;
            gRobot.archeryStruct.turnTable.Target.pos	= 0.0f;
          }
          else if(gRobot.walkStatus == 32 || gRobot.walkStatus == 15 || gRobot.walkStatus == 42)//2
          {
            shootPara.turnTableAngle = 120.0f;
            gRobot.archeryStruct.turnTable.Target.pos	= 120.0f;
          }
					
					
				}
				if(fabs(gRobot.archeryStruct.archeryPitch.Act.pos-LOAD_PITCH_ANG) < 2.5f && ((gRobot.walkStatus == 32 || gRobot.ppsData.y < 2800.0f) || (gRobot.walkStatus == 7 && gRobot.ppsData.y < 9600.f) || gRobot.walkStatus == 8))
				{
					prepareMode ++;
					loadFlag = 1;
					SetClearIntegral(CAN2,LOAD_ARROW_ID);
				}
				break;
			}
			case 1:
			{
				//为上箭留时间
        switch (gRobot.attackPotID[gRobot.shootOrder])
        {
          case P2B:
          {
            if(LoadDelay(LOAD_DELAY_2B))
            {
              prepareMode++;
              loadDoneFlag = 1;
            }
            break;
          }
          default:
          {
            if(LoadDelay(LOAD_DELAY))
            {
              prepareMode++;
              loadDoneFlag = 1;
            }
            break;
          }
        }
				
				break;
			}
			case 2:
			{				
				return 1;
			}
		}
		return 0;
	}
}



/*********************************************************************
* @brief 取箭电机位置函数
* @note  
* @param  
* @retval 
*********************************************************************/
extern int loadArrowCnt;

void LoadArrow(void)
{
	static float firstPos = 0;
	static int firstLoadCnt;
	static int lastLoadCnt;

//	if(fabs(gRobot.archeryStruct.loadArrow.motorVelTarget.pos - loadArrowMsg.pos) < 200 && abs(loadArrowMsg.vel) < 200 && enableState == 0)
//	{
//		DriverState(CAN2,PTP_MODE,LOAD_ARROW_ID,DISABLE);
//	}
//	else if(enableState == 1)
//	{
//		DriverState(CAN2,PTP_MODE,LOAD_ARROW_ID,ENABLE);
//	}
  

	if(loadStep > 5)//五次循环置零
	{
		loadStep = 1;
		loadArrowCircleCnt++;
	}
	
	//一开始TR身上只有三只箭，loadStep从4开始
	if(gRobot.walkStatus == 0 && loadStep == 5)
	{
		loadStep = 5;
	}
	
	switch(loadStep)
	{
		//上第一支箭
		case FIRST_LOAD:
		{
			
			
			lastLoadCnt = 0;
			firstPos = -16680;//第一次上完箭位置

			firstLoadCnt++;
			if(firstLoadCnt <= 100)
			{
//        SetVelLimit(CAN2, PTP_MODE, LOAD_ARROW_ID, 4*4096);
				gRobot.archeryStruct.loadArrow.motorVelTarget.pos = -(firstPos + (33.0f/25.0f) * 4096 - loadArrowCircleCnt * (300.0f/25.0f) * 4096);
			}
			if(firstLoadCnt > 100 && abs(loadArrowMsg.vel) < 500)
			{
        SetVelLimit(CAN2, PTP_MODE, LOAD_ARROW_ID, 2.9*4096);
				gRobot.archeryStruct.loadArrow.motorVelTarget.pos = -(firstPos - loadArrowCircleCnt * (300.0f/25.0f) * 4096);//保证每次上箭运行相同距离
				enableState = 1;
			}
			if(firstLoadCnt >110)
			{
				firstLoadCnt = 120;
			}
			
			if(loadFlag == 1 && fabs(loadArrowMsg.pos - gRobot.archeryStruct.loadArrow.motorVelTarget.pos) < 800)
			{
				SetClearIntegral(CAN2,LOAD_ARROW_ID);
				loadStep ++;
			}
			loadFlag = 0;
		}
		break;
		
		//上第二支箭
		case SECOND_LOAD:
		
		//上第三支箭
		case THIRD_LOAD:

		//上第四支箭
		case FOURTH_LOAD://2-4支箭
		{
			firstPos = -16661;//第一次上完箭位置
			firstLoadCnt = 0;
			gRobot.archeryStruct.loadArrow.motorVelTarget.pos = -(firstPos - (33.0f/25.0f) * 4096 * (loadStep  -1) - loadArrowCircleCnt * (300.0f/25.0f) *4096);//中间三次放箭
			if(loadFlag == 1 && fabs(loadArrowMsg.pos - gRobot.archeryStruct.loadArrow.motorVelTarget.pos) < 800)
			{
				SetClearIntegral(CAN2,LOAD_ARROW_ID);
				loadStep ++;
			}
			enableState = 1;
			loadFlag = 0;
			break;
		}
		//上第五支箭
		case FIFTH_LOAD:
		{
			
			gRobot.archeryStruct.loadArrow.motorVelTarget.pos = -(firstPos - (33.0f/25.0f) * 4096 * (loadStep  -1) - loadArrowCircleCnt * (300.0f/25.0f) *4096);
			lastLoadCnt++;
			if(lastLoadCnt > 120)
			{
				SetVelLimit(CAN2, PTP_MODE, LOAD_ARROW_ID, 4.6*4096);
				gRobot.archeryStruct.loadArrow.motorVelTarget.pos = -(-(300.0f/25.0f) * 4096 - loadArrowCircleCnt * (300.0f/25.0f) * 4096);
        enableState = 1;
			}
			if(lastLoadCnt >120)
			{
				lastLoadCnt = 121;
			}
			
//			gRobot.archeryStruct.loadArrow.motorVelTarget.pos = -(-(300.0f/25.0f) * 4096 - loadArrowCircleCnt * (300.0f/25.0f) * 4096);
			if(loadFlag == 1 && fabs(loadArrowMsg.pos - gRobot.archeryStruct.loadArrow.motorVelTarget.pos) < 800)
			{
				SetClearIntegral(CAN2,LOAD_ARROW_ID);
				
				loadStep ++;
			}
			loadFlag = 0;
		}break;
		
		default:
			break;
	}

  if(enableState == 1 && fabs(loadArrowMsg.pos - gRobot.archeryStruct.loadArrow.motorVelTarget.pos) < 120)
  {
    loadMotorDone = 1;
  }

//  //失能使能
//  switch(loadStep)
//  {
//    case FIRST_LOAD:
//    {
//      if(fabs(loadArrowMsg.pos + (firstPos - loadArrowCircleCnt * (300.0f/25.0f) * 4096))<800)
//      {
//         DriverState(CAN2,PTP_MODE,LOAD_ARROW_ID,DISABLE);
//      }
//      else if(fabs(loadArrowMsg.pos - gRobot.archeryStruct.loadArrow.motorVelTarget.pos) > 800)
//      {
//         DriverState(CAN2,PTP_MODE,LOAD_ARROW_ID,ENABLE);
//      }
//      break;
//    }
//    //上第二支箭
//		case SECOND_LOAD:
//		
//		//上第三支箭
//		case THIRD_LOAD:

//		//上第四支箭
//		case FOURTH_LOAD://2-4支箭
//    {
//      if(fabs(loadArrowMsg.pos + (firstPos - (33.0f/25.0f) * 4096 * (loadStep  -1) - loadArrowCircleCnt * (300.0f/25.0f) *4096))<800)
//      {
//         DriverState(CAN2,PTP_MODE,LOAD_ARROW_ID,DISABLE);
//      }
//      else if(fabs(loadArrowMsg.pos - gRobot.archeryStruct.loadArrow.motorVelTarget.pos) > 800)
//      {
//         DriverState(CAN2,PTP_MODE,LOAD_ARROW_ID,ENABLE);
//      }
//      break;
//    }
//    //上第五支箭
//		case FIFTH_LOAD:
//    {
//        if(fabs(loadArrowMsg.pos + (300.0f/25.0f) * 4096 - loadArrowCircleCnt * (300.0f/25.0f) * 4096)<800)
//        {
//           DriverState(CAN2,PTP_MODE,LOAD_ARROW_ID,DISABLE);
//        }
//        else if(fabs(loadArrowMsg.pos - gRobot.archeryStruct.loadArrow.motorVelTarget.pos) > 800)
//        {
//           DriverState(CAN2,PTP_MODE,LOAD_ARROW_ID,ENABLE);
//        }
//        break;
//    }
//  }
	
	

//	if(loadArrowMsg.currQ >130*1000 )
//	{
//	  countPro++;
//		if(countPro > 5)
//		{
//		  DriverState(CAN2, PTP_MODE, LOAD_ARROW_ID, DISABLE);
//		}
//	}
}

/*********************************************************************
* @brief 取箭俯仰、转盘角度变化函数
* @note  
* @param  
* @retval 
*********************************************************************/
uint8_t FetchArrow(uint8_t startFetch)
{
	static uint8_t fetchMode = 0;
	//不进行取箭
	if(startFetch == 0)
	{
		fetchMode = 0;//将上箭阶段重新赋值为0，准备下一次取箭的准备动作
		return 0;
	}
	else
	{
		MAGNET_ON;
		switch(fetchMode)//分两段上箭，不让俯仰电机转过大角度
		{
			
			case 0:
			{
				shootPara.archeryPitchAngle = -104.8f;//-104
				shootPara.turnTableAngle = 240.f;//120
				if(fabs(gRobot.archeryStruct.archeryPitch.Act.pos - shootPara.archeryPitchAngle) < 3.0f || \
				fabs(gRobot.archeryStruct.turnTable.Act.pos - shootPara.turnTableAngle) < 1.5f)
				{	
					fetchMode ++;
				}
				break;
			}
			case 2:
			{
					return 1;
			}		
		}
		SetArrowPitchPos(archeryPitchMsg,shootPara.archeryPitchAngle,1.0f);
		SetTurntablePos(turnTableMsg,shootPara.turnTableAngle,1.0f);
		return 0;
	}
//	shootPara.archeryPitchAngle = -104.0f;
//	shootPara.turnTableAngle = 179.99f;
	//设置俯仰期望位置

//	if(fabs(gRobot.archeryStruct.archeryPitch.Act.pos - shootPara.archeryPitchAngle) < 1.5f && fabs(gRobot.archeryStruct.turnTable.Act.pos - shootPara.turnTableAngle) < 1.5f)
//	{	
//		return 1;
//	}
//	if(fabs(gRobot.archeryStruct.archeryPitch.Act.pos - shootPara.archeryPitchAngle) < 1.5f && \
//		fabs(gRobot.archeryStruct.turnTable.Act.pos - shootPara.turnTableAngle) < 1.5f)
//	{	
//		return 1;
//	}
}

uint8_t PrepareForLoadFirstArrow(uint8_t startPrepare)
{
	static uint8_t prepareMode = 0;
	//不准备上箭
	if(startPrepare == 0)
	{
		prepareMode = 0;
		return 0;
	}
	else
	{
		switch(prepareMode)
		{
			case 0:
			{
				shootPara.archeryPitchAngle = 40.0f;//40
				shootPara.turnTableAngle = 60.f;//60
				if(fabs(gRobot.archeryStruct.archeryPitch.Act.pos - shootPara.archeryPitchAngle) < 3.0f || \
				fabs(gRobot.archeryStruct.turnTable.Act.pos - shootPara.turnTableAngle) < 1.5f)
				{	
					prepareMode ++;
				}
				break;
			}
//			case 1:
//			{
//				shootPara.archeryPitchAngle = 40.0f;//40
//				shootPara.turnTableAngle = 60.f;//60
//				if(fabs(gRobot.archeryStruct.archeryPitch.Act.pos - shootPara.archeryPitchAngle) < 1.5f && \
//				fabs(gRobot.archeryStruct.turnTable.Act.pos - shootPara.turnTableAngle) < 1.5f)
//				{	
//					prepareMode ++;
//					return 1;
//				}
//				break;
//			}
			case 1:
			{
					return 1;
			}		
		}
		SetArrowPitchPos(archeryPitchMsg,shootPara.archeryPitchAngle,1.0f);
		SetTurntablePos(turnTableMsg,shootPara.turnTableAngle,1.0f);
		return 0;
	}
}

/*********************************************************************
* @brief 转盘电机位置函数
* @note  
* @param  
* @retval 
*********************************************************************/
void SetTurntablePos (driverMsg_t archery,float aimPosAngle,float para)
{
//	float output = 0;
//	output = aimPosAngle*TURN_RATIO;//ArrowPosPid(&tTablePosPara,archery.motor.pos,aimPos*TURN_RATIO)*para;
	gRobot.archeryStruct.turnTable.Target.pos = aimPosAngle;
	//PosCtrl(CAN1, PTP_MODE, TURN_TABLE_ID, ABSOLUTE_MODE, gRobot.archeryStruct.turnTable.motorVelTarget.pos);
}


/*********************************************************************
* @brief 俯仰电机位置函数
* @note  
* @param  
* @retval 
**********************************************************************/
void  SetArrowPitchPos (driverMsg_t archery,float aimPosAngle,float para)
{
//	float output = 0;
//	//gRobot.archeryStruct.archeryPitch.motorVelTarget.pos = output;
//	output = aimPosAngle*PITCH_RATIO;//ArrowPosPid(&elevatorPosPara,archery.motor.pos,aimPos*PITCH_RATIO)*para;
	gRobot.archeryStruct.archeryPitch.Target.pos = aimPosAngle;
	//PosCtrl(CAN1, PTP_MODE, ARCHERY_PITCH_ID, ABSOLUTE_MODE, gRobot.archeryStruct.archeryPitch.motorVelTarget.pos);
}

/*********************************************************************
* @brief  配置射箭俯仰参数
* @note
* @param  potID 桶号
* @retval None
**********************************************************************/
void ArcheryPitchSelect(void)
{
	if(gRobot.walkStatus == 0)
	{
		switch(gRobot.attackPotID[gRobot.shootOrder])
		{
			case P2A://2A
			{				
				shootPara.archeryPitchAngle = 54.5f;//42
			}
			break;
			
			case P2B://2B
			{
			  shootPara.archeryPitchAngle = 45.0f;	//40
			}
			break;
			
			case P1A://1A
			{
			  shootPara.archeryPitchAngle = 48.0f;	//42	
			}
			break;
			
			case P1B://1B
			{
			  shootPara.archeryPitchAngle = 52.0f;			//42	
			}
			break;
			
			case P3://3
			{
			  shootPara.archeryPitchAngle = 47.0f;	//42
			}
			break;
			
			default:
			{
				shootPara.archeryPitchAngle = 48.0f;	//43
				break;
			}
		}
	}
	else
	{
		switch(gRobot.attackPotID[gRobot.shootOrder])
		{
			case P2A://2A
			{
				
				shootPara.archeryPitchAngle = 54.5f;//42
			}
			break;
			
			case P2B://2B
			{
			  shootPara.archeryPitchAngle = 52.0f;	//40
			}
			break;
			
			case P1A://1A
			{
			  shootPara.archeryPitchAngle = 54.5f;	//42	
			}
			break;
			
			case P1B://1B
			{
			  shootPara.archeryPitchAngle = 52.0f;			//42	
			}
			break;
			
			case P3://3
			{
			  shootPara.archeryPitchAngle = 54.5f;	//42
			}
			break;
			
			default:
			{
				shootPara.archeryPitchAngle = 54.5f;	//43
				break;
			}
		}
	}
}

/*********************************************************************
* @brief  配置射箭参数
* @note
* @param  potID 桶号
* @retval None
**********************************************************************/
para_t potCloocet(int potID)
{
	para_t pot;
	int shootPlace = 0;
	if(gRobot.walkStatus == 0 || gRobot.ppsData.y < 800.f)
	{
		shootPlace = 0;	//自己再区分射1射2区
	}
	else if(gRobot.walkStatus == 4 || gRobot.walkStatus == 8 || gRobot.ppsData.y > 8000.f)
	{
		shootPlace = 1;
	}
	else if(gRobot.walkStatus == 32 || (gRobot.ppsData.y > 1000.f && gRobot.ppsData.y < 3000.f)) 
	{
		shootPlace = 2;
	}
	else
	{
		shootPlace = 1;
	}
								
	pot.archeryPitchAngle = shootPitchAngSheet[potID-1][shootPlace];//因为potID范围是1-5，所以是potID-1
	//turnTable和车身角度有关，这里要改
	
  switch(potID)
	{
	  case P2A://2A
		{
			pot.turnTableAngle = turntableAngleCal(gRobot.ppsData.x,gRobot.ppsData.y,potCentGet(potID),gRobot.ppsData.angle,turnTableMsg.pos/TURNTABLE_RATIO);
			if(vChanFlag.vel2A == 1)
			{
				vChanFlag.vel2A = 0;
				pot.shootVel = shootVelGet(voltageTemp,potID) + gRobot.velChange;//需调试完善		
			}
			else
			{
				pot.shootVel = shootVelGet(voltageTemp,potID);
			}
			
		}
		break;
		
		case P2B://2B
		{
			pot.turnTableAngle = turntableAngleCal(gRobot.ppsData.x,gRobot.ppsData.y,potCentGet(potID),gRobot.ppsData.angle,turnTableMsg.pos/TURNTABLE_RATIO);
			if(vChanFlag.vel2B == 1)
			{
				vChanFlag.vel2B = 0;
				pot.shootVel = shootVelGet(voltageTemp,potID) + gRobot.velChange;//需调试完善		
			}
			else
			{
				pot.shootVel = shootVelGet(voltageTemp,potID);
			}
		}
		break;
		
		case P1A://1A
		{
			pot.turnTableAngle = turntableAngleCal(gRobot.ppsData.x,gRobot.ppsData.y,potCentGet(potID),gRobot.ppsData.angle,turnTableMsg.pos/TURNTABLE_RATIO);
			if(vChanFlag.vel1A == 1)
			{
				vChanFlag.vel1A = 0;
				pot.shootVel = shootVelGet(voltageTemp,potID) + gRobot.velChange;//需调试完善		
			}
			else
			{
				pot.shootVel = shootVelGet(voltageTemp,potID);
			}		
		}
		break;
		
		case P1B://1B
		{
			pot.turnTableAngle = turntableAngleCal(gRobot.ppsData.x,gRobot.ppsData.y,potCentGet(potID),gRobot.ppsData.angle,turnTableMsg.pos/TURNTABLE_RATIO);
			if(vChanFlag.vel1B == 1)
			{
				vChanFlag.vel1B = 0;
				pot.shootVel = shootVelGet(voltageTemp,potID) + gRobot.velChange;//需调试完善		
			}
			else
			{
				pot.shootVel = shootVelGet(voltageTemp,potID);
			}			
		}
		break;
		
		case P3://3
		{
			pot.turnTableAngle = turntableAngleCal(gRobot.ppsData.x,gRobot.ppsData.y,potCentGet(potID),gRobot.ppsData.angle,turnTableMsg.pos/TURNTABLE_RATIO);
			if(vChanFlag.vel3 == 1)
			{
				vChanFlag.vel3 = 0;
				pot.shootVel = shootVelGet(voltageTemp,potID) + gRobot.velChange;//需调试完善		
			}
			else
			{
				pot.shootVel = shootVelGet(voltageTemp,potID);
			}
		}
		break;
		
		default:
			break;
	}
	if(pot.shootVel > 135.0f)
	{
		pot.shootVel = 135.f;
	}
	return pot;
}





void UpdateActArcheryAngle(void)
{
					
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("TBU %d %d "),(int)(gRobot.archeryStruct.turnTable.Act.pos * 100),(int)(turnTableMsg.pos));
				
	//获得俯仰角和转盘角度
	gRobot.archeryStruct.turnTable.Act.pos = turnTableMsg.pos / TURNTABLE_RATIO; 
	gRobot.archeryStruct.archeryPitch.Act.pos = archeryPitchMsg.pos / PITCH_RATIO; 
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//	(uint8_t *)("TAU %d %d "),(int)(gRobot.archeryStruct.turnTable.Act.pos * 100),(int)(turnTableMsg.pos));
}


/*********************************************************************
* @brief  
* @note
* @param  
* @retval None
**********************************************************************/
void limit(float *data,float leftData,float rightData)
{
  if(*data < leftData)
	{
	  *data = leftData;
	}
	if(*data > rightData)
	{
	  *data = rightData;
	}
}

/*********************************************************************
* @brief  跟随底盘函数
* @note   为了适合底盘走行
* @param  
* @retval None
**********************************************************************/
float turntableFollow(void)
{
	//走行跟随可以是任意形式，但要有益于底盘走行
	//短距离可能不需要变化，长距离则需要跟随
	//可在从添加响应函数
  float output;
	output = 0.0f;
	return output;
}

/*********************************************************************
* @brief  转盘角度获取函数
* @note   计算终点该转多少度
* @param  
* @retval None
**********************************************************************/
float turntableAngleCal(float endX,float endY,pot_t pot,float turnAngle,float turnTable)
{
	float aimAngle;
	float Wtable;
	float output;

	aimAngle = atan2f(pot.pos.y - endY,pot.pos.x - endX) * (180.0f / PI);
	Wtable = angleLimit(turnAngle +120 + turnTable);
	//aimAngle = angleLimit(aimAngle);
	output = aimAngle - Wtable;
  output += turnTable;
	output = angleLimit(output);
  return output;	
}

void clearData(uint8_t*data,int num)
{
  for(int i;i<num;i++)
	{
	  data[i] = 0;
	}
}

float angleLimit(float angle)
{
  if(angle > 180)
	{
	  angle -= 360;
	}
	if(angle < -180)
	{
	  angle += 360;
	}
	return angle;
}

pot_t potCentGet(int potID)
{
	pot_t pot;
	
  if(potID == P1A)
	{
	  pot.pos.x = 5670.f;
		pot.pos.y = 7950.f;
	}
	else if(potID == P1B)
	{
	  pot.pos.x = 5670.f;
		pot.pos.y = 3950.f;
	}
	else if(potID == P2A)
	{
	  pot.pos.x = 3450.f;
		pot.pos.y = 5950.f;
	}
	else if(potID == P2B)
	{
	  pot.pos.x = 8450.f;
		pot.pos.y = 5950.f;
	}
	else if(potID == P3)
	{
	  pot.pos.x = 5950.f;
		pot.pos.y = 5950.f;
	}
	
	return pot;
}


//两车通信
uint8_t RobotComm(void)
{
  static int comCnt = 0;
  comCnt++;
  if(comCnt > 6)
  {
    //蓝牙
    USART_SendData(USART3,'A');
    USART_SendData(USART3,'T');
    USART_SendData(USART3,gRobot.attackPotID[gRobot.shootOrder]);
    USART_SendData(USART3,'\r');
    USART_SendData(USART3,'\n');
    //WIFI
    USART_SendData(USART3,'A');
    USART_SendData(USART3,'T');
    USART_SendData(USART3,gRobot.attackPotID[gRobot.shootOrder]);
    USART_SendData(USART3,'\r');
    USART_SendData(USART3,'\n');
    comCnt = 0;
    return 1;
  }
  return 0;
}

//视觉偏离角度选择
float cvAngleSelect(uint8_t trPot)
{
	float cvPotAngle;
	if(gRobot.walkStatus == 0 || gRobot.ppsData.y < 800.f)
	{
		switch (trPot)
		{
			case P3:
			{
        if(gRobot.fieldCol == BLUE_FIELD)
        {
          cvPotAngle = B_CV_ANGLE_30;
        }
				else if(gRobot.fieldCol == RED_FIELD)
        {
          cvPotAngle = R_CV_ANGLE_30;
        }
				break;
			}
			case P2B:
			{
        if(gRobot.fieldCol == BLUE_FIELD)
        {
          cvPotAngle = B_CV_ANGLE_2B0;
        }
				else if(gRobot.fieldCol == RED_FIELD)
        {
          cvPotAngle = R_CV_ANGLE_2B0;
        }
				break;
			}
      case P2A:
			{
        if(gRobot.fieldCol == BLUE_FIELD)
        {
          cvPotAngle = B_CV_ANGLE_2A0;
        }
				else if(gRobot.fieldCol == RED_FIELD)
        {
          cvPotAngle = R_CV_ANGLE_2A0;
        }
				break;
			}
			default:
				cvPotAngle = R_CV_ANGLE_2A0;
			break;
		}
	}
	else if(gRobot.walkStatus == 32 || (gRobot.ppsData.y > 1000.f && gRobot.ppsData.y < 3000.f))//射箭2区
	{
		switch (trPot)
		{
			case P1A:
			{
        if(gRobot.fieldCol == BLUE_FIELD)
        {
          cvPotAngle = B_CV_ANGLE_1A2;
        }
				else if(gRobot.fieldCol == RED_FIELD)
        {
          cvPotAngle = R_CV_ANGLE_1A2;
        }
				
				break;
			}
			case P2A:
			{
        if(gRobot.fieldCol == BLUE_FIELD)
        {
  				cvPotAngle = B_CV_ANGLE_2A2;
        }
				else if(gRobot.fieldCol == RED_FIELD)
        {
          cvPotAngle = R_CV_ANGLE_2A2;
        }
				break;
			}
			case P1B:
			{
        if(gRobot.fieldCol == BLUE_FIELD)
        {
  				cvPotAngle = B_CV_ANGLE_1B;
        }
				else if(gRobot.fieldCol == RED_FIELD)
        {
          cvPotAngle = R_CV_ANGLE_1B;
        }
				break;
			}
			case P2B:
			{
        if(gRobot.fieldCol == BLUE_FIELD)
        {
  				cvPotAngle = B_CV_ANGLE_2B2;
        }
				else if(gRobot.fieldCol == RED_FIELD)
        {
          cvPotAngle = R_CV_ANGLE_2B2;
        }

				break;
			}
			case P3:
			{
        if(gRobot.fieldCol == BLUE_FIELD)
        {
  				cvPotAngle = B_CV_ANGLE_32;
        }
				else if(gRobot.fieldCol == RED_FIELD)
        {
          cvPotAngle = R_CV_ANGLE_32;
        }
				break;
			}
			default:
				cvPotAngle = R_CV_ANGLE_1B;
			break;
		}
	}
	else if(gRobot.ppsData.y > 8000.f)//射箭1区
	{
		switch (trPot)
		{
			case P1A:
			{
        if(gRobot.fieldCol == BLUE_FIELD)
        {
  				cvPotAngle = B_CV_ANGLE_1A1;
        }
				else if(gRobot.fieldCol == RED_FIELD)
        {
          cvPotAngle = R_CV_ANGLE_1A1;
        }
				break;
			}
			case P2A:
			{
        if(gRobot.fieldCol == BLUE_FIELD)
        {
  				cvPotAngle = B_CV_ANGLE_2A1;
        }
				else if(gRobot.fieldCol == RED_FIELD)
        {
          cvPotAngle = R_CV_ANGLE_2A1;
        }
				break;
			}
			case P1B:
			{
        if(gRobot.fieldCol == BLUE_FIELD)
        {
  				cvPotAngle = B_CV_ANGLE_1B1;
        }
				else if(gRobot.fieldCol == RED_FIELD)
        {
          cvPotAngle = R_CV_ANGLE_1B1;
        }
				break;
			}
			case P2B:
			{
        if(gRobot.fieldCol == BLUE_FIELD)
        {
  				cvPotAngle = B_CV_ANGLE_2B1;
        }
				else if(gRobot.fieldCol == RED_FIELD)
        {
          cvPotAngle = R_CV_ANGLE_2B1;
        }
				break;
			}
			case P3:
			{
        if(gRobot.fieldCol == BLUE_FIELD)
        {
  				cvPotAngle = B_CV_ANGLE_31;
        }
				else if(gRobot.fieldCol == RED_FIELD)
        {
          cvPotAngle = R_CV_ANGLE_31;
        }
				break;
			}
			default:
				cvPotAngle = R_CV_ANGLE_1B;
			break;
		}
	}
	return cvPotAngle;
}

//复位后对射箭、轮子、俯仰电机的操作
void motorSetAfterReset(void)
{
	DriverState(CAN2,PTP_MODE,SHOOT_ARROW_ID,ENABLE);
	//俯仰解除抱死
	SetWheelControlMode(CAN2,PTP_MODE,ARCHERY_PITCH_ID,2);
	
	SetWheelControlMode(CAN1,BROADCAST_MODE,1,2);//速度环
	//轮子清除积分
	SetClearIntegral(CAN1,ONE_WHEEL_ID);
	SetClearIntegral(CAN1,TWO_WHEEL_ID);
	SetClearIntegral(CAN1,THR_WHEEL_ID);
	SetClearIntegral(CAN1,ONE_TURN_ID);
	SetClearIntegral(CAN1,TWO_TURN_ID);
	SetClearIntegral(CAN1,THR_TURN_ID);

  SetVelLimit(CAN1, PTP_MODE, ONE_WHEEL_ID , 100*4096);
	SetVelLimit(CAN1, PTP_MODE, TWO_WHEEL_ID , 100*4096);
	SetVelLimit(CAN1, PTP_MODE, THR_WHEEL_ID , 100*4096);
}

//接收DR射箭桶号，判断是否一致，若一致，转向下一个桶投箭
//DR只发一次，会有可能收不到的情况？
void JudgeDRArrow(void)
{
	if(gRobot.drPotRecieve == 1)
	{
		gRobot.drPotRecieve = 0;
		if(gRobot.drPotID == gRobot.attackPotID[gRobot.shootOrder] && (gRobot.shootOrder < gRobot.potNum - 1))//
		{
			gRobot.shootOrder++;
			//换桶瞄准前变量清0
			potAssDone = 0;
			pitchDone = 0;
			turnDone = 0;
		}
		else if(gRobot.drPotID == gRobot.attackPotID[gRobot.shootOrder] && (gRobot.shootOrder >= gRobot.potNum - 1))//无下一目标桶号
		{
			//待完善，等待0.2s。若收到app新的桶，则瞄准新桶，否则射旧桶
//			gRobot.shootOrder++;
//			ptAutoDone = 0;
//			ptOperaDone = 0
		}
	}
	else if(gRobot.drPotRecieve == 0)
	{
		if(gRobot.shootOrder > gRobot.potNum)//收到的桶号已打完，等待操作手新的指令
		{
			//待完善,应该等待新的桶号
//			ptAutoDone = 0;
//			ptOperaDone = 0;
		}
	}
}

//边瞄准边射箭复位
uint8_t ResetArchery(void)
{
	static int resetDoneCnt;
	#ifdef BIG_SHOOT
			gRobot.archeryStruct.shootArrow.motorVelTarget.vel -= acc * 0.01f;
			if(gRobot.archeryStruct.shootArrow.motorVelTarget.vel <= 2*4096 || shootArrowMsg.pos*MINUS > FULL_CYCLE - 1200)
			{
				gRobot.archeryStruct.shootArrow.motorVelTarget.vel = 1*4096;
			}
	#else
			static int stableCnt = 0;
			static int resetHalfPos = 0;
			resetHalfPos = FULL_CYCLE;
			gRobot.archeryStruct.shootArrow.motorVelTarget.vel\
			= ShootPosPid(&shootPosPara,resetHalfPos,shootArrowMsg.pos*MINUS);
			if(fabs(gRobot.archeryStruct.shootArrow.motorVelTarget.vel) > (50*4096))//float型限幅可能有问题！！！
			{
				gRobot.archeryStruct.shootArrow.motorVelTarget.vel = fabs(gRobot.archeryStruct.shootArrow.motorVelTarget.vel)/gRobot.archeryStruct.shootArrow.motorVelTarget.vel*50*4096;
			}
			if(shootArrowMsg.pos*MINUS > resetHalfPos - 1000)
			{
				stableCnt++;//只进一次
				if(stableCnt == 1)
				{
					SetClearIntegral(CAN2,SHOOT_ARROW_ID);
					stableCnt = 2;
				}
				gRobot.archeryStruct.shootArrow.motorVelTarget.vel = 1.4*4096;
			}
	#endif
	
	

	if(abs(shootArrowMsg.vel*MINUS2) < 300*1 && abs(FULL_CYCLE - shootArrowMsg.pos*MINUS2) < 300*1)
	{
    gRobot.archeryStruct.shootArrow.motorVelTarget.vel = 0.6*4096;
    SetClearIntegral(CAN2,SHOOT_ARROW_ID);//清除积分
		resetDoneCnt++;
		if(resetDoneCnt > 4)
		{
			//加一句让驱动器把位置置0
			SetPosZero(CAN2,SHOOT_ARROW_ID,-CHANG_POS	,200*4096);//复位完成将当前位置清零，重新设定失能位置
			SetClearIntegral(CAN2,SHOOT_ARROW_ID);//清除积分
			SetSpeedKP(CAN2, PTP_MODE, SHOOT_ARROW_ID, 0.4f*1000);//2.8
			SetSpeedKI(CAN2, PTP_MODE, SHOOT_ARROW_ID, 0.8f*1000);//20
			gRobot.archeryStruct.shootArrow.motorVelTarget.vel = 0;
//			MAGNET_OFF;	
			#ifndef BIG_SHOOT
			stableCnt = 0;
			#endif
			resetDoneCnt = 0;
			return 1;
		}
	}
  else if((shootArrowMsg.pos*MINUS2 - FULL_CYCLE) > 80*1)
//  if((shootArrowMsg.pos*MINUS2 - FULL_CYCLE) > 50*1)
  {
    //加一句让驱动器把位置置0
			SetPosZero(CAN2,SHOOT_ARROW_ID,-CHANG_POS	,200*4096);//复位完成将当前位置清零，重新设定失能位置
			SetClearIntegral(CAN2,SHOOT_ARROW_ID);//清除积分
			SetSpeedKP(CAN2, PTP_MODE, SHOOT_ARROW_ID, 0.4f*1000);//2.8
			SetSpeedKI(CAN2, PTP_MODE, SHOOT_ARROW_ID, 0.8f*1000);//20
			gRobot.archeryStruct.shootArrow.motorVelTarget.vel = 0;
//			MAGNET_OFF;	
			#ifndef BIG_SHOOT
			stableCnt = 0;
			#endif
			resetDoneCnt = 0;
			return 1;
  }
	return 0;
}

//上箭延时
uint16_t LoadDelay(int delayTime)
{
	static int shootLoadCnt = 0;
	
	shootLoadCnt++;
	if(shootLoadCnt > delayTime)
	{
		shootLoadCnt = 0;
		return 1;
	}
	return 0;
}

//转盘延时
uint16_t TurnDelay(int delayTime)
{
	static int shootTurnCnt = 0;
	
	shootTurnCnt++;
	if(shootTurnCnt > delayTime)
	{
		shootTurnCnt = 0;
		return 1;
	}
	return 0;
}

//为下一箭的桶号对应的转盘、俯仰、转速赋值
uint16_t PotAssignment(void)
{
	shootPara = potCloocet(gRobot.attackPotID[gRobot.shootOrder]);
	return 1;
}

//俯仰调整
uint16_t PitchAdjust(void)
{
	static int shootPitchFlag = 0;
	static int pitchCnt = 0;
  static int pitchMotorSet = 0;
	//俯仰调节
	if(fabs(gRobot.archeryStruct.archeryPitch.Act.pos - shootPara.archeryPitchAngle)<3.2f && pitchMotorSet < 3)//俯仰接近射箭时的角度，抱死
	{
    pitchMotorSet++;
		SetWheelControlMode(CAN2,PTP_MODE,ARCHERY_PITCH_ID,1);
	}
	if(fabs(gRobot.archeryStruct.archeryPitch.Act.pos - shootPara.archeryPitchAngle)<0.82f && loadDoneFlag == 1 )//俯仰可以射箭
	{
		pitchCnt++;
		if(pitchCnt >= 10)
		{
			pitchCnt = 20;
			shootPitchFlag = 1;
		}
	}
  else
  {
    pitchCnt = 0;
  }
	if(shootPitchFlag == 1)
	{
		pitchCnt = 0;
		shootPitchFlag = 0;
    pitchMotorSet = 0;
		return 1;
	}
	return 0;
}

 uint16_t TurnAdjust(void)
{
	float visionAngle = 0;
	static int aimStep = 0;
  static int operaStep = 0;
	static int averCnt = 0;
	static float sumAngleCV = 0.0f;
	float averAngleCV = 0.0f;
	//视觉识别到桶
//	if(gRobot.cvStruct.state == 1 && gRobot.cvAnomalFlag == 0 && gRobot.cvStruct.cvDir != 0)
  if(gRobot.cvStruct.state == 1 && gRobot.cvStruct.cvDir != 0 && gRobot.turnOpera == 0)
	{		
		visionAngle = gRobot.cvStruct.cvDir - (cvAngleSelect(gRobot.attackPotID[gRobot.shootOrder]) + gRobot.cvCompensate);	
    gRobot.cvAngleLiner = GetDisToVel(cvAngleToDisGet,gRobot.cvStruct.cvDis,ANGLE_DIS_NUM);
		//偏差角度限幅，防止视觉遮挡桶程序错误
		if(fabs(visionAngle) > 20.0f)
		{
			visionAngle = 0;
		}
		switch (aimStep)
		{
			case 0:
			{
        if(fabs(gRobot.archeryStruct.turnTable.Act.pos - shootPara.turnTableAngle) > 3.f)
        {
          //视觉遮挡桶程序没有桶及切换下一个桶的时候上层版应该发0桶号
          gRobot.writeFlag = 0;
        }
				if(fabs(gRobot.archeryStruct.turnTable.Act.pos - shootPara.turnTableAngle) < 0.3f && fabs(visionAngle) < 12.0f && fabs(gRobot.cvStruct.cvDir) < 12.f)
				{	
					shootPara.turnTableAngle = gRobot.archeryStruct.turnTable.Act.pos - visionAngle; 
					aimStep = 1;
				}
				break;
			}
			case 1:
			{
				if(fabs(gRobot.archeryStruct.turnTable.Act.pos - shootPara.turnTableAngle) < 0.15f)//0.3f
				{
					aimStep = 2;
				}
				break;
			}
			case 2:
			{
          //判断桶是否被乱转
          if(visionAngle > 6.0f)
          {
            gRobot.shootOrder++;
            potAssDone = 0;
            pitchDone = 0;
            turnDone = 0;
            return 0;
          }
//        if(gRobot.shotArrowsCnt == 0)
//        {
//          if(LoadDelay(1000) == 1)//delay for the stability of CV
//          {
//            aimStep = 3;
//          }
//        }
//        else
        {
          if(TurnDelay(30) == 1)//delay for the stability of CV
          {
            aimStep = 3;
          }
        }
				break;
			}
////////////////////////////////////////////////////////////////////////

			case 3:
			{
//        static float angleMax = 0;
//        static float angleMin = 0;
//        if(averCnt == 0)
//        {
//          angleMax = visionAngle;
//          angleMin = visionAngle;
//        }
//        else
//        {
//          
//        }
				averCnt++;
				sumAngleCV = sumAngleCV + visionAngle;
				if(averCnt >= 15)
				{
					averAngleCV = sumAngleCV / 15.0f;
					if(fabs(averAngleCV) < 0.06f)
					{
						averCnt = 0;
						sumAngleCV = 0.0f;
						aimStep = 0;
            gRobot.cvCompensate = 0;
						return 1;
					}
					else
					{
						shootPara.turnTableAngle = gRobot.archeryStruct.turnTable.Act.pos - averAngleCV;
						aimStep = 4;
					}
				}
				break;
			}
			case 4:
			{
				if(fabs(gRobot.archeryStruct.turnTable.Act.pos - shootPara.turnTableAngle)< 0.04f)
				{
//          if(gRobot.attackPotID[gRobot.shootOrder] == 5 && gRobot.shotArrowsCnt == 1)
//          {
//            aimStep = 5;
//          }
          if(TurnDelay(20) == 1)
          {
            averCnt = 0;
            sumAngleCV = 0.0f;
            aimStep = 0;
            gRobot.cvCompensate = 0;
            return 1;
          }
				}
				break;
			}

///////////////////////////////////////////////////////////////////////////
      case 5:
			{
          //判断桶是否被乱转
          if(visionAngle > 6.0f)
          {
            gRobot.shootOrder++;
            potAssDone = 0;
            pitchDone = 0;
            turnDone = 0;
            return 0;
          }
//        if(gRobot.shotArrowsCnt == 0)
//        {
//          if(LoadDelay(1000) == 1)//delay for the stability of CV
//          {
//            aimStep = 3;
//          }
//        }
//        else
        {
          if(TurnDelay(30) == 1)//delay for the stability of CV
          {
            aimStep = 6;
          }
        }
				break;
			}
////////////////////////////////////////////////////////////////////////

			case 6:
			{
				averCnt++;
				sumAngleCV = sumAngleCV + visionAngle;
				if(averCnt >= 15)
				{
					averAngleCV = sumAngleCV / 15.0f;
					if(fabs(averAngleCV) < 0.06f)
					{
						averCnt = 0;
						sumAngleCV = 0.0f;
						aimStep = 0;
            gRobot.cvCompensate = 0;
						return 1;
					}
					else
					{
						shootPara.turnTableAngle = gRobot.archeryStruct.turnTable.Act.pos - averAngleCV;
						aimStep = 7;
					}
				}
				break;
			}
			case 7:
			{
				if(fabs(gRobot.archeryStruct.turnTable.Act.pos - shootPara.turnTableAngle)< 0.04f)
				{
          if(TurnDelay(12) == 1)
          {
            averCnt = 0;
            sumAngleCV = 0.0f;
            aimStep = 0;
            gRobot.cvCompensate = 0;
            return 1;
          }
				}
				break;
			}
		}
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
		(uint8_t *)("cA %d %d %d "),(int)(shootPara.turnTableAngle*100),(int)(visionAngle*100),(int)aimStep);		
	}
//  else if(gRobot.cvAnomalFlag == 1)//视觉掉线，手动调整转盘
  else if(gRobot.turnOpera)
  {
    switch (operaStep)
		{
			case 0:
			{
				if(fabs(gRobot.archeryStruct.turnTable.Act.pos - shootPara.turnTableAngle) < 0.15f)
				{	
					shootPara.turnTableAngle = shootPara.turnTableAngle + gRobot.turnOperaAdjust;
					operaStep = 1;
				}
				break;
			}
			case 1:
			{
				if(fabs(gRobot.archeryStruct.turnTable.Act.pos - shootPara.turnTableAngle) < 0.15f)//0.3f
				{
					operaStep = 0;
          gRobot.cvCompensate = 0;
					return 1;
				}
				break;
			}
    }
    USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
		(uint8_t *)("tO %d %d %d "),(int)(operaStep),(int)(gRobot.cvCompensate*100),(int)(shootPara.turnTableAngle*100));		
  }
	return 0;	
}

//上箭
uint16_t LoadAction(void)
{
	static int loadDone = 0;
	static uint8_t delayStep = 0;

	if(gRobot.shotArrowsCnt == 0 )//第一支和到射箭区的第一支不上箭（已自动上箭）
	{
		//不需要上箭
		loadFlag = 0;
		return 1;
	}
	
//	#ifdef START_PLACE_ARCHERY
	if((gRobot.shotArrowsCnt - gRobot.initialCnt)%5 == 0 || gRobot.shotArrowsCnt == gRobot.initialCnt)//第一支和到射箭区的第一支不上箭（已自动上箭）
//	if(gRobot.shotArrowsCnt == 3)
	{
		//不需要上箭
		return 0;
	}
//	#endif
	
	//需要上箭
	shootPara.archeryPitchAngle = LOAD_PITCH_ANG;
	
	if(fabs(gRobot.archeryStruct.archeryPitch.Act.pos - LOAD_PITCH_ANG) < 2.0f)
	{
		switch(delayStep)
		{
			case 0:
			{
        if(gRobot.ppsData.y < 800.f)
        {
          if(LoadDelay(70)== 1)
          {
            delayStep = 1;
          }
        }
        else
        {
          if(LoadDelay(60)== 1)
          {
            delayStep = 1;
          }
        }
				
				break;
			}
			case 1:
			{
				loadFlag = 1;
				delayStep =2;
				break;
			}
			case 2:
			{
        enableState = 0;
        loadMotorDone = 0;
				//为上箭留时间
				switch (gRobot.attackPotID[gRobot.shootOrder])
				{
					case P2B:
					{
						loadDone = LoadDelay(LOAD_DELAY_2B);
						break;
					}
					default:
					{
						loadDone = LoadDelay(LOAD_DELAY);
						break;
					}
				}
				break;
			}
			default :
				break;
		}
		
	}
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("L %d %d "),(int)loadDone,(int)delayStep);
	if(loadDone == 1)
	{
		loadDone = 0;
		delayStep = 0;
		return 1;
	}
	
	return 0;
}

//射箭前的电机设置
void ShootMotorSet(void)
{
	//射箭电机参数
	SetSpeedKP(CAN2, PTP_MODE, SHOOT_ARROW_ID, shootPara.shootKp*1000);//2.8
	SetSpeedKI(CAN2, PTP_MODE, SHOOT_ARROW_ID, shootPara.shootKi*1000);//20
	
	//俯仰抱死
	SetWheelControlMode(CAN2,PTP_MODE,ARCHERY_PITCH_ID,1);
	
	//轮子抱死
	SetWheelControlMode(CAN1,BROADCAST_MODE,1,1);
	
	//射箭驱动器清除积分
	SetClearIntegral(CAN2,SHOOT_ARROW_ID);
}

//参数清零
void ValueReset(void)
{
	loadDoneFlag = 0;
	resetDone = 0;
	potAssDone = 0;
	appSendOnce = 0;
	recAppPitch = 0;
	operaRelease = 0;
	pitchDone = 0;
	turnDone = 0;
	appShootFlag = 0;
	canShootFlag = 0;
  comDone = 0;//两车通信
  loadMotorDone = 0;
  enableState = 0;
}

//射箭分段给阶跃
void ShootSegmentStep(void)
{
	gRobot.archeryStruct.shootArrow.motorVelTarget.vel += acc * 0.01f;//斜坡输入
	//需要阶段处理，让后几个周期缩小进行******
	if(gRobot.archeryStruct.shootArrow.motorVelTarget.vel > shootPara.shootVel*4096.0f)
	{
		gRobot.archeryStruct.shootArrow.motorVelTarget.vel = shootPara.shootVel*4096.0f;
	}
	if(gRobot.archeryStruct.shootArrow.motorVelTarget.vel * gRobot.archeryStruct.shootArrow.motorVelTarget.vel/2.0f/acc > CHANG_POS)
	{
		gRobot.archeryStruct.shootArrow.motorVelTarget.vel = shootPara.shootVel * 4096.0f;
	}
}

//射箭直接阶跃
void ShootBigStep(void)
{
	gRobot.archeryStruct.shootArrow.motorVelTarget.vel = shootPara.shootVel*4096.0f;	
}

//向APP发送数据
uint16_t AppSendData(void)
{
	static int appSendCnt = 0;
	appSendCnt++;
	if(appSendCnt < 6)
	{
		SendData2BlueTooth(voltageTemp,shootPara.shootVel,gRobot.cvStruct.cvDir,gRobot.cvStruct.cvDis);
	}
	if(appSendCnt>=6)
	{
		appSendCnt = 0;
		return 1;
	}
	return 0;
}

