#include "includes.h"
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "spi.h"
#include "pps.h"
#include "dma.h"
#include "key.h"
#include "robot.h"
#include "motorcontrol.h"
#include "adc.h"
#include "pccomm.h"
#include "pid.h"
#include "APP.h"
#include "archery.h"
#include "pot.h"
uint8_t getRobotModeFlag = 0;
uint8_t ResetAndCheck(void);
uint8_t TurnReset(void);
void DebugDataOut(void);
void SelfCheckDebugDataOut(void);
void ResetDebugDataOut(void);
void HardwareInit(void);
//复位限电流
void MotorConfigInit(void);
//行走时限流
void MotorWalkInit(void);
void SetOriginValue(void);
uint8_t CheckRetrySwitch(void);
uint8_t CheckKeySwitch(void);
void MotorSafeLimit(void);
void MotorHoldUp(void);
//防止重复接收APP指令
void AvoidRepeatRec(void);
//判断视觉是否掉线
void AbnormalJudge(void);
void SendData2App(void);
void ShootDebugDataOut(void);

int waitForFetchArrow = 1;

extern uint8_t resetDone;
extern int loadDoneFlag;
extern uint8_t pitchDone;
extern uint8_t turnDone;
extern int recAppPitch;
extern uint8_t canShootFlag;
extern uint8_t loadMotorDone;

/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;
OS_EVENT *ResetPeriodSem;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK ThrowTaskStk[Throw_TASK_STK_SIZE];
static OS_STK SelfCheckTaskStk[SelfCheck_TASK_STK_SIZE];

void App_Task()
{
	CPU_INT08U os_err;
	os_err = os_err; /*防止警告...*/

	/*创建信号量*/
	PeriodSem = OSSemCreate(0);
	
	ResetPeriodSem = OSSemCreate(0);


	/*创建任务*/
	os_err = OSTaskCreate((void (*)(void *))ConfigTask, /*初始化任务*/
						  (void *)0,
						  (OS_STK *)&App_ConfigStk[Config_TASK_START_STK_SIZE - 1],
						  (INT8U)Config_TASK_START_PRIO);

	os_err = OSTaskCreate((void (*)(void *))ThrowTask,
						  (void *)0,
						  (OS_STK *)&ThrowTaskStk[Throw_TASK_STK_SIZE - 1],
						  (INT8U)Throw_TASK_PRIO);
	os_err = OSTaskCreate((void (*)(void *))SelfCheckTask,
						  (void *)0,
						  (OS_STK *)&SelfCheckTaskStk[SelfCheck_TASK_STK_SIZE - 1],
						  (INT8U)SelfCheck_TASK_PRIO);
}

/*
   ===============================================================
   初始化任务
   ===============================================================
*/
extern float startP3disVelGet[3][2];
float velGetTest;
int photoLevel = 0;
uint8_t photoCnt = 0;
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	//单片机外设初始化
	HardwareInit();
	//电机初始化
	MotorConfigInit();
	//各项数值设置默认参数
	SetOriginValue(); 
//	AllMotorOff();
	
//	AllMotorOn();
	
	
	OSSemSet(ResetPeriodSem, 0, &os_err);
	while (1)
	{	
		OSSemPend(ResetPeriodSem, 0, &os_err);
		OSSemSet(ResetPeriodSem, 0, &os_err);			
//		USART_SendData(USART2,'A');
		//AllMotorOff();
//gRobot.cvStruct.cvDis = 6600.f;
//gRobot.attackPotID[gRobot.shootOrder] = 1;
//gRobot.walkStatus = 32;
//		shootPara.shootVel = shootVelGet(voltageTemp,gRobot.attackPotID[gRobot.shootOrder]);
//USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//	(uint8_t *)("cv %d "),(int)(shootPara.shootVel * 100));
		voltageRead = Get_Adc_Average(8, 20);
		voltageTemp = voltageRead * (3.3/4096) * 9.86;
//    gRobot.potNum = 3;
//    gRobot.attackPotID[0] = 1;
//    gRobot.attackPotID[1] = 2;
//    SendData2App();
    //和上位机通讯
//		Talk2PC();

    photoLevel = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5);
    SendData2App();

		//防止重复接收APP指令
		AvoidRepeatRec();

		//读电机信息
		ReadMotorMsg();
		//更新实际轮速
		UpdataActWheelVel();
		//检查驱动器状态
		//更新射箭机构实际角度
		UpdateActArcheryAngle();
		
		CheckDriverState();
		//CheckDriverRec();
		
		//测试走形，直接记为2支箭
//		gRobot.shotArrowsCnt = 2;
		
		if(gRobot.reset.isResetDone == IS_RESETING)
		{
			//执行复位程序
			gRobot.reset.isResetDone = ResetAndCheck();
		}
		else
		{
//			AllMotorOff();
			//检测按键
			if(CheckKeySwitch() == 1)
			{
				//根据按键设置机器人状态
				if(gRobot.robotMode == WASH_MODE)
				{
					gRobot.steerStruct.one.wheelVelTarget.vel = 500.0f;
					gRobot.steerStruct.two.wheelVelTarget.vel = -500.0f;
					gRobot.steerStruct.thr.wheelVelTarget.vel = 500.0f;
					gRobot.steerStruct.one.wheelVelTarget.pos = ONE_POS_INIT;
					gRobot.steerStruct.two.wheelVelTarget.pos = TWO_POS_INIT;
					gRobot.steerStruct.thr.wheelVelTarget.pos = THR_POS_INIT;
					
					SendCmd2Drivers();
				}
				else if(gRobot.robotMode == SELF_CHECK_MODE)
				{						
					OSTaskSuspend(Throw_TASK_PRIO);
					break;
				}
				else if(gRobot.robotMode == DEBUG_PARA_MODE)
				{
					
				}
				else if(gRobot.robotMode == WALK_MODE)  //比赛开始
				{
//          MotorWalkInit();
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
	(uint8_t *)("config "));
					OSTaskSuspend(SelfCheck_TASK_PRIO);
					break;
				}	
			}
		}
		
		ResetDebugDataOut();
	}
	
	//使能TIM2
	TIM_Cmd(TIM2, ENABLE); 
	//失能TIM3
	TIM_Cmd(TIM3, DISABLE); 
	
	delay_s(1);
	
	OSTaskSuspend(OS_PRIO_SELF);
}

uint32_t timeCnt = 0;
uint32_t timeCounter = 0;
uint16_t loadCnt = 0;
int pitchCnt = 0;
int magentHigh = 0;
int modeCnt = 0;
int pccommCnt = 0;
int maxMotorSet = 0, slowMotorSet = 0;
uint8_t disableCnt = 0;
uint8_t disableOne = 0;
uint8_t turnAble = 0;
void ThrowTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		OSSemSet(PeriodSem, 0, &os_err);		
//		static uint16_t retryStep = 0;
    static int motorSend = 0;
		
		//和上位机通讯
		Talk2PC();
		
		//负责查看程序有无超周期  单位100us
		timeCnt=0;
		//记录周期数 单位10ms
		timeCounter++;
		//读电机信息
		ReadMotorMsg();
		//更新实际轮子速度
		UpdataActWheelVel();
		//更新射箭机构实际角度
		UpdateActArcheryAngle();

    photoLevel = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5);

	if(fabs(gRobot.archeryStruct.turnTable.Act.pos - shootPara.turnTableAngle) > 15.f && gRobot.walkStatus == 8 )
	{
		SetVelLimit(CAN2, PTP_MODE, TURN_TABLE_ID, 80.0f*4096.0f);
	}
	else
	{
		SetVelLimit(CAN2, PTP_MODE, TURN_TABLE_ID, 150.0f*4096.0f);
	}
		
	
	

    if(photoLevel == 0)
    {
      photoCnt++;
      if(photoCnt >= 5)
      {
        photoCnt = 6;
        gRobot.contestStart = 1;
      }     
    }
    else
    {
      photoCnt = 0;
    }

		magentHigh = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);//磁铁引脚电平

		//在射箭区抱死调整轮子参数，出发时参数改回
		MotorHoldUp();
		
		/****APP按开始键，跳过射箭，开始走行****/
//		if(gRobot.contestStart == 1 && gRobot.walkStatus == 0)//在出发区按开始，开始走行
//		{			
//			gRobot.shotArrowsCnt = 3;
////			gRobot.contestStart = 0;
////			startCnt = 1;
//		}

    //发送给APP更新已接收到的桶号
    SendData2App();

		//防止重复接收APP指令
		AvoidRepeatRec();

    //检测车的状态指示灯
    AbnormalJudge();
	
	if(fabs(loadArrowMsg.pos - gRobot.archeryStruct.loadArrow.motorVelTarget.pos) > 1000 && loadArrowMsg.vel < 500)
    {
      
      disableCnt++;
      if(disableCnt > 80)
      {
        disableCnt = 85;
        DriverState(CAN2,PTP_MODE,LOAD_ARROW_ID,DISABLE);
      }
    }
    else
    { 
      disableCnt = 0;
    }

    if(motorSend == 0)
    {
      motorSend = 1;
      DriverState(CAN2,PTP_MODE,LOAD_ARROW_ID,DISABLE);
    }
	
	if(gRobot.TRRetryFlag == 1)
	{
		gRobot.potNum = 0;
		gRobot.attackPotID[0]=0;
		gRobot.attackPotID[1]=0;
		gRobot.attackPotID[2]=0;
		gRobot.attackPotID[3]=0;
		gRobot.attackPotID[4]=0;
		gRobot.attackPotID[5]=0;
		gRobot.attackPotID[6]=0;
		gRobot.attackPotID[7]=0;
	}
    
    if(gRobot.loadEnable == 1)
    {
      gRobot.loadEnable = 0;
		if(disableOne == 0)
		{
			gRobot.attackPotID[0] = P2B;
			gRobot.potNum = 3;
			disableOne = 1;
		}
      
		if(gRobot.TRRetryFlag == 1)
		{
			gRobot.potNum = 0;
			gRobot.attackPotID[0]=0;
			gRobot.attackPotID[1]=0;
			gRobot.attackPotID[2]=0;
			gRobot.attackPotID[3]=0;
			gRobot.attackPotID[4]=0;
			gRobot.attackPotID[5]=0;
			gRobot.attackPotID[6]=0;
			gRobot.attackPotID[7]=0;
		}
      SetClearIntegral(CAN2,LOAD_ARROW_ID);
      DriverState(CAN2,PTP_MODE,LOAD_ARROW_ID,ENABLE);
    }

//    if(gRobot.walkStatus != 0 && motorSend == 0)
    if(gRobot.steerStruct.one.wheelVelTarget.vel > 2000.f && gRobot.steerStruct.two.wheelVelTarget.vel > 2000.f && gRobot.steerStruct.thr.wheelVelTarget.vel > 2000.f)
    {
      if(maxMotorSet <= 3)
      {
        slowMotorSet = 0;
        maxMotorSet++;
        MotorWalkInit();
      }
    }
    else 
    {
      if(slowMotorSet <= 3)
      {
        slowMotorSet++;
        maxMotorSet = 0;
        SlowMotorWalkInit();
      }
    }

    
//    MAGNET_OFF;


		//重试(不能重开小电，等定位系统会花时间，给雷达一个重试标志位）
		/*
			身上有箭：用APP告知主控箭的个数，在初始上箭机构转到
		*/
//		if(retryStep == 0 && gRobot.robotMode == WALK_MODE)
//		{
//			if(gRobot.robotMode == RETRY_MODE)
//			{
//				gRobot.TRRetryFlag = 1;//给雷达的重试标志位置1
//				shootStep = 0;
//				retryStep ++;
//				//桶号数量也要清0
//				gRobot.potNum = 0;
//			}
//		}
		
		if(gRobot.spiState.startTalkFlag == 1)//spi通信成功
		{
			//射箭
			ShootArrow();
			
			if(gRobot.walkStatus == 3 || gRobot.walkStatus == 7 || gRobot.walkStatus == 15 || gRobot.walkStatus == 17)
			{
				waitForFetchArrow = 1;
			}
			
			//上箭                                             
			LoadArrow();
			
			//记得取消屏蔽
			
			//取箭前的一系列动作及取完箭的上箭动作
			FetchAndLoadAction();
		}

    //两射箭区切换时转盘的变化
    if(gRobot.walkStatus == 17)//1号到2号
    {
      gRobot.archeryStruct.turnTable.Target.pos = 120.f;
    } 
    else if(gRobot.walkStatus == 19)//2号到1号
    {
      gRobot.archeryStruct.turnTable.Target.pos = 0.f;
    }
	
	
	if(gRobot.walkStatus == 40)
	{
		pitchDone = 0;
		turnDone = 0;
	}
//    MAGNET_OFF;
		
		MotorSafeLimit();
		
		//电机控制
		MotorControl();
// SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x06);//正常
//    VelCtrl(CAN2,PTP_MODE,SHOOT_ARROW_ID, MINUS*10*4096);
		
		//检查spi通信状态
		CheckSpiState();
		
		//检查驱动器状态
		CheckDriverState();
		//CheckDriverRec();

    Talk2PC();

		//发送调试数据
		DebugDataOut();
	} 
}


//设置
void SetOriginValue(void)
{
  gRobot.initialCnt = 3;
  gRobot.batteryNum = 9;//设置默认电池
	gRobot.walkMode = DEFENCE_MODE;//默认2号射箭区

	//设置默认进攻桶号
	gRobot.attackPotID[0] = 0;
  gRobot.attackPotID[1] = P3;
	gRobot.attackPotID[2] = P2A;
//  gRobot.attackPotID[3] = P1A;
//	gRobot.attackPotID[4] = P3;
//  gRobot.attackPotID[5] = P1B;
//	gRobot.attackPotID[6] = P2A;
//  gRobot.attackPotID[7] = P1B;
  
	
	shootPara.archeryPitchAngle = 45.0f;//35
//	shootPara.turnTableAngle = 60.0f;
	shootPara.turnTableAngle = 0.0f;
	shootPara.shootKp = 1.5f;
	shootPara.shootKi = 5.f;

	gRobot.archeryStruct.shootArrow.motorVelTarget.pos = 0;
	//设置红蓝场
	gRobot.fieldCol = BLUE_FIELD;

	//上箭步骤从第三支箭开始，记得取消屏蔽
	#ifdef START_PLACE_ARCHERY
	loadStep = 0;
	#else
	loadStep = 3;
	#endif
	
	//射箭模式（自动射箭）
	gRobot.operationMode = AUTOMATIC_MODE;
	
//	gRobot.archeryStruct.turnTable.Target.pos = 60;
	gRobot.archeryStruct.turnTable.Target.pos = 0.0f;
	gRobot.archeryStruct.archeryPitch.Target.pos = 45.0f;//42.0f
	
	MAGNET_OFF;

}
void HardwareInit(void)
{
	//定时器2周期100us 用于主循环
	TIM_Init(TIM2, 99, 83, 1, 0);	
	//失能TIM2
	TIM_Cmd(TIM2, DISABLE); 	
	//定时器3周期100us 用于复位循环
	TIM_Init(TIM3, 99, 83, 1, 0);	
	//初始化调试蓝牙串口USART1
	USARTDMASendInit(DEBUG_USART,DebugUSARTDMASendBuf,&USART1_Init,921600);
	//初始化app通信蓝牙u4
	UART4_Init(921600);
	//初始化两车通信蓝牙u3
	USART3_Init(921600);
	//初始化两车通信WIFIu2
	USART2_Init(921600);
	//CAN1初始化
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	//CAN2初始化
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);
	//SPI3初始化，波特率5.25M
	//SPI3Init();	
	//SPI1初始化，波特率5.25M
	SPI1Init();
	SPI1_Tx_DMA_Configuration();
	SPI1_Rx_DMA_Configuration();
	
	GPIO_Init_Pins(GPIOC, GPIO_Pin_0, GPIO_Mode_IN);
	GPIO_Init_Pins(GPIOC, GPIO_Pin_1, GPIO_Mode_IN);
	GPIO_Init_Pins(GPIOC, GPIO_Pin_2, GPIO_Mode_IN);
	GPIO_Init_Pins(GPIOC, GPIO_Pin_3, GPIO_Mode_IN);
	GPIO_Init_Pins(GPIOC, GPIO_Pin_4, GPIO_Mode_IN);
	GPIO_Init_Pins(GPIOC, GPIO_Pin_5, GPIO_Mode_IN);
	GPIO_Init_Pins(GPIOB, GPIO_Pin_12, GPIO_Mode_OUT);
	GPIO_UP_Init_Pins(GPIOC, GPIO_Pin_5, GPIO_Mode_IN);

  GPIO_Init_Pins(GPIOC, GPIO_Pin_12, GPIO_Mode_OUT);
//	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	GPIO_SetBits(GPIOC, GPIO_Pin_12);
	Adc_Init();
	PidParaInit();
	
	delay_ms(50);
}

//复位限电流
void MotorConfigInit(void)
{
//       SetPosAccDec(CAN1, PTP_MODE, ONE_TURN_ID , 50/6*4096, 50/6*4096);
//       SetPosAccDec(CAN1, PTP_MODE, TWO_TURN_ID , 50/6*4096, 50/6*4096);
//       SetPosAccDec(CAN1, PTP_MODE, THR_TURN_ID , 50/6*4096, 50/6*4096);
//	SetTorqueLimit(CAN1,PTP_MODE,ONE_WHEEL_ID,1000);
//	SetTorqueLimit(CAN1,PTP_MODE,ONE_TURN_ID,1000);
//	               
//	SetTorqueLimit(CAN1,PTP_MODE,TWO_WHEEL_ID,1000);
//	SetTorqueLimit(CAN1,PTP_MODE,TWO_TURN_ID,1000);
//	               
//	SetTorqueLimit(CAN1,PTP_MODE,THR_WHEEL_ID,1000);
//	SetTorqueLimit(CAN1,PTP_MODE,THR_TURN_ID,1000);
	
//	SetPosKP(CAN1, PTP_MODE, ONE_TURN_ID , 40*1000);
//	SetPosKP(CAN1, PTP_MODE, TWO_TURN_ID , 40*1000);
//	SetPosKP(CAN1, PTP_MODE, THR_TURN_ID , 40*1000);
//	
//	SetPosKD(CAN1, PTP_MODE, ONE_TURN_ID , 0.1*4.0*1000);
//	SetPosKD(CAN1, PTP_MODE, TWO_TURN_ID , 0.1*1.5*1000);
//	SetPosKD(CAN1, PTP_MODE, THR_TURN_ID , 0.1*1.7*1000);
//	
//	SetSpeedKP(CAN1, PTP_MODE, ONE_TURN_ID , 2.0*1000);
//	SetSpeedKP(CAN1, PTP_MODE, TWO_TURN_ID , 2.0*1000);
////	SetSpeedKP(CAN1, PTP_MODE, THR_TURN_ID , 0.5*1000);
//	
//	SetSpeedKP(CAN1, PTP_MODE, ONE_WHEEL_ID , 1.5*1000);
//	SetSpeedKI(CAN1, PTP_MODE, ONE_WHEEL_ID , 12.0*1000);
//	
//	SetSpeedKP(CAN1, PTP_MODE, TWO_WHEEL_ID , 1.5*1000);
//	SetSpeedKI(CAN1, PTP_MODE, TWO_WHEEL_ID , 12.0*1000);
//	
//	SetSpeedKP(CAN1, PTP_MODE, THR_WHEEL_ID , 1.5*1000);
//	SetSpeedKI(CAN1, PTP_MODE, THR_WHEEL_ID , 12.0*1000);
////	
//	SetSpeedKI(CAN1, PTP_MODE, ONE_TURN_ID , 3.0f*1000);
//	SetSpeedKI(CAN1, PTP_MODE, TWO_TURN_ID , 2.5*1000);
////	SetSpeedKI(CAN1, PTP_MODE, THR_TURN_ID , 2.2*1000);
//	
//	SetPosKP(CAN1, PTP_MODE, ONE_TURN_ID , 2.0*60*1000);
//	SetPosKP(CAN1, PTP_MODE, TWO_TURN_ID , 2.0*60*1000);
////	SetPosKP(CAN1, PTP_MODE, THR_TURN_ID , 1.2*60*1000);
//	
////	SetSpeedKP(CAN1, PTP_MODE, THR_TURN_ID , 1.8*1000);

	
	
//	SetVelLimit(CAN2, PTP_MODE, TURN_TABLE_ID, 4.0f*4096.0f);
//	SetVelLimit(CAN2, PTP_MODE, ARCHERY_PITCH_ID, 3.0f*4096.0f);

	SetAccDec(CAN2, PTP_MODE, SHOOT_ARROW_ID, 3000*512, 3000*512);
	
	SetVelLimit(CAN2, PTP_MODE, LOAD_ARROW_ID,2.9*4096);

	SetSpeedKP(CAN2, PTP_MODE, SHOOT_ARROW_ID, 0.8f*1000);//8
	SetSpeedKI(CAN2, PTP_MODE, SHOOT_ARROW_ID, 1.0f*1000);//20
	
//	SetCurrQLimit(CAN2,TURN_TABLE_ID,90*1000);
//	SetVelLimit(CAN2, PTP_MODE, TURN_TABLE_ID, 90*4096);
	
//	SetPosZero(CAN2,SHOOT_ARROW_ID,-22550,300,-22850);
	//初始时第一次给射箭电机设置一个失能的位置和范围
	SetPosZero(CAN2,SHOOT_ARROW_ID,-23800,200*4096);
	delay_s(1);
}

//行走时限流
void MotorWalkInit(void)
{
//  SetSpeedKP(CAN1, PTP_MODE, ONE_TURN_ID , 2.0*1000);
//	SetSpeedKP(CAN1, PTP_MODE, TWO_TURN_ID , 2.0*1000);
//	SetSpeedKP(CAN1, PTP_MODE, THR_TURN_ID , 2.0*1000);
	
//	SetSpeedKP(CAN1, PTP_MODE, ONE_WHEEL_ID , 3.5*1000);
//	SetSpeedKI(CAN1, PTP_MODE, ONE_WHEEL_ID , 3.0*1000);
	
	SetSpeedKP(CAN1, PTP_MODE, TWO_WHEEL_ID , 2.5*1000);
	SetSpeedKI(CAN1, PTP_MODE, TWO_WHEEL_ID , 3.0*1000);
	
	SetSpeedKP(CAN1, PTP_MODE, THR_WHEEL_ID , 2.5*1000);
	SetSpeedKI(CAN1, PTP_MODE, THR_WHEEL_ID , 3.0*1000);

//	SetSpeedKI(CAN1, PTP_MODE, ONE_TURN_ID , 2.0*1000);
//	SetSpeedKI(CAN1, PTP_MODE, TWO_TURN_ID , 2.0*1000);
	SetSpeedKI(CAN1, PTP_MODE, THR_TURN_ID , 2.0*1000);

//	SetSpeedKP(CAN1, PTP_MODE, ONE_TURN_ID , 1.5*1000);
//	SetSpeedKP(CAN1, PTP_MODE, TWO_TURN_ID , 1.5*1000);
	SetSpeedKP(CAN1, PTP_MODE, THR_TURN_ID , 1.5*1000);

//  SetVelLimit(CAN1, PTP_MODE, ONE_WHEEL_ID , 100*4096);
//	SetVelLimit(CAN1, PTP_MODE, TWO_WHEEL_ID , 100*4096);
//	SetVelLimit(CAN1, PTP_MODE, THR_WHEEL_ID , 100*4096);

//	SetPosKP(CAN1, PTP_MODE, ONE_TURN_ID , 1.5*60*1000);
//	SetPosKP(CAN1, PTP_MODE, TWO_TURN_ID , 1.5*60*1000);
	SetPosKP(CAN1, PTP_MODE, THR_TURN_ID , 1.3*60*1000);



//	SetPosKD(CAN1, PTP_MODE, ONE_TURN_ID , 0.1*2.0*1000);
//	SetPosKD(CAN1, PTP_MODE, TWO_TURN_ID , 0.1*2.0*1000);
//	SetPosKD(CAN1, PTP_MODE, THR_TURN_ID , 0.1*2.0*1000);
//	
//	SetSpeedKP(CAN1, PTP_MODE, THR_TURN_ID , 1.8*1000);

//	SetVelLimit(CAN1, PTP_MODE, ONE_TURN_ID , 10*4096);
//	SetVelLimit(CAN1, PTP_MODE, TWO_TURN_ID , 10*4096);
//	SetTorqueLimit(CAN1,PTP_MODE,ONE_WHEEL_ID,2452);
//	SetTorqueLimit(CAN1,PTP_MODE,ONE_TURN_ID,2452);
//	               
//	SetTorqueLimit(CAN1,PTP_MODE,TWO_WHEEL_ID,2452);
//	SetTorqueLimit(CAN1,PTP_MODE,TWO_TURN_ID,2452);
//	               
//	SetTorqueLimit(CAN1,PTP_MODE,THR_WHEEL_ID,2452);
//	SetTorqueLimit(CAN1,PTP_MODE,THR_TURN_ID,2452);
	
//	delay_ms(50);
}

void SlowMotorWalkInit(void)
{
//  SetSpeedKP(CAN1, PTP_MODE, ONE_TURN_ID , 2.0*1000);
//	SetSpeedKP(CAN1, PTP_MODE, TWO_TURN_ID , 2.0*1000);
//	SetSpeedKP(CAN1, PTP_MODE, THR_TURN_ID , 2.0*1000);
	
	SetSpeedKP(CAN1, PTP_MODE, ONE_WHEEL_ID , 1.0*1000);
	SetSpeedKI(CAN1, PTP_MODE, ONE_WHEEL_ID , 2.0*1000);
	
	SetSpeedKP(CAN1, PTP_MODE, TWO_WHEEL_ID , 1.0*1000);//2.0
	SetSpeedKI(CAN1, PTP_MODE, TWO_WHEEL_ID , 2.0*1000);
	
	SetSpeedKP(CAN1, PTP_MODE, THR_WHEEL_ID , 1.0*1000);
	SetSpeedKI(CAN1, PTP_MODE, THR_WHEEL_ID , 2.0*1000);

	SetSpeedKI(CAN1, PTP_MODE, THR_TURN_ID , 1.0*1000);
	SetSpeedKP(CAN1, PTP_MODE, THR_TURN_ID , 1.0*1000);
	SetPosKP(CAN1, PTP_MODE, THR_TURN_ID , 0.8*60*1000);

//	SetSpeedKI(CAN1, PTP_MODE, ONE_TURN_ID , 1.5*1000);
//	SetSpeedKI(CAN1, PTP_MODE, TWO_TURN_ID , 1.5*1000);
//	SetSpeedKI(CAN1, PTP_MODE, THR_TURN_ID , 1.5*1000);

//	SetSpeedKP(CAN1, PTP_MODE, ONE_TURN_ID , 1.2*1000);
//	SetSpeedKP(CAN1, PTP_MODE, TWO_TURN_ID , 1.2*1000);
//	SetSpeedKP(CAN1, PTP_MODE, THR_TURN_ID , 1.2*1000);

//  SetVelLimit(CAN1, PTP_MODE, ONE_WHEEL_ID , 100*4096);
//	SetVelLimit(CAN1, PTP_MODE, TWO_WHEEL_ID , 100*4096);
//	SetVelLimit(CAN1, PTP_MODE, THR_WHEEL_ID , 100*4096);

//	SetPosKP(CAN1, PTP_MODE, ONE_TURN_ID , 1.2*60*1000);
//	SetPosKP(CAN1, PTP_MODE, TWO_TURN_ID , 1.2*60*1000);
//	SetPosKP(CAN1, PTP_MODE, THR_TURN_ID , 1.2*60*1000);
}

//航向复位
uint8_t ResetAndCheck(void)
{
	static uint8_t resetStep = 0;

	switch(resetStep)
	{
		case 0:
		{
			if(TurnReset() == 1)
			{
				resetStep = 1;
			}
			break;
		}
		case 1:
		{
			if(PosInit() == 1)
			{
				resetStep = 2;
			}
			SendCmd2Drivers();
			break;
		}
		case 2:
		{
			resetStep = 0;
			VelCtrl(CAN1,PTP_MODE,ONE_WHEEL_ID,0);
			VelCtrl(CAN1,PTP_MODE,TWO_WHEEL_ID,0);
			VelCtrl(CAN1,PTP_MODE,THR_WHEEL_ID,0);
			
			return RESET_DONE;
			//break;
		}
	}
	return IS_RESETING;
}			
//航向复位函数
uint8_t TurnReset(void)
{
	static uint8_t resetCnt=0;
	//0.5s检测到轮子脉冲小于1000 复位完成
	if(oneTurnMsg.vel < 1000 && twoTurnMsg.vel < 1000 && thrTurnMsg.vel < 1000)
	{
		resetCnt++;
	}
	if( resetCnt > 50)
	{		
		resetCnt = 0;
		return 1;
	}

	return 0;
}	


void ResetDebugDataOut(void)
{		
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("cv %d %d "),\
//				(int)(gRobot.cvStruct.cvDir * 10),(int)(gRobot.cvStruct.cvDis*10));
//	
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//	(uint8_t *)("CM %d "),(int)gRobot.drPotID);
//	
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("SS %d %d %d %d %d %d %d %d %d "),\
//				(int)gRobot.drPotID,(int)gRobot.shootOrder,(int)gRobot.potNum,(int)gRobot.attackPotID[0],(int)gRobot.attackPotID[1],\
//					(int)gRobot.attackPotID[2],(int)gRobot.attackPotID[3],(int)gRobot.attackPotID[4],(int)gRobot.attackPotID[5]);

	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
	(uint8_t *)("RE: W1 %d %d %d %d "),(int)gRobot.steerStruct.one.wheelVelTarget.vel,(int)gRobot.steerStruct.one.wheelVelAct.vel,\
					(int)gRobot.steerStruct.one.motorVelTarget.vel,(int)oneWheelMsg.vel);
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("T1 %d %d %d %d %d "),(int)(gRobot.steerStruct.one.wheelVelTarget.pos*10),(int)(gRobot.steerStruct.one.wheelVelAct.pos*10),\
					(int)gRobot.steerStruct.one.motorVelTarget.pos,(int)oneTurnMsg.pos,(int)oneTurnMsg.vel);			
				
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("W2 %d %d %d %d "),(int)gRobot.steerStruct.two.wheelVelTarget.vel,(int)gRobot.steerStruct.two.wheelVelAct.vel,\
					(int)gRobot.steerStruct.two.motorVelTarget.vel,(int)twoWheelMsg.vel);
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("T2 %d %d %d %d %d "),(int)(gRobot.steerStruct.two.wheelVelTarget.pos*10),(int)(gRobot.steerStruct.two.wheelVelAct.pos*10),\
					(int)gRobot.steerStruct.two.motorVelTarget.pos,(int)twoTurnMsg.pos,(int)twoTurnMsg.vel);		
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("W3 %d %d %d %d "),(int)gRobot.steerStruct.thr.wheelVelTarget.vel,(int)gRobot.steerStruct.thr.wheelVelAct.vel,\
					(int)gRobot.steerStruct.thr.motorVelTarget.vel,(int)thrWheelMsg.vel);
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("T3 %d %d %d %d %d "),(int)(gRobot.steerStruct.thr.wheelVelTarget.pos*10),(int)(gRobot.steerStruct.thr.wheelVelAct.pos*10),\
					(int)gRobot.steerStruct.thr.motorVelTarget.pos,(int)thrTurnMsg.pos,(int)thrTurnMsg.vel);	
				
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("Q %d %d %d %d %d %d "),(int)(oneWheelMsg.currQ),(int)(twoWheelMsg.currQ), (int)(thrWheelMsg.currQ),\
//					(int)(oneTurnMsg.currQ), (int)(twoTurnMsg.currQ), (int)(thrTurnMsg.currQ));		
//				
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("AS %d %d %d %d %d %d "),(int)magentHigh, (int)(gRobot.archeryStruct.shootArrow.motorVelTarget.pos),\
//					(int)(gRobot.archeryStruct.shootArrow.motorVelTarget.vel), (int)(gRobot.archeryStruct.shootArrow.Act.vel),\
//					(int)(-shootArrowMsg.pos), (int)(shootArrowMsg.vel));
//				
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("AT %d %d %d %d %d %d "),(int)(gRobot.archeryStruct.turnTable.Act.pos),(int)(gRobot.archeryStruct.turnTable.Target.pos),\
//					(int)(gRobot.archeryStruct.turnTable.motorVelTarget.pos), (int)(gRobot.archeryStruct.turnTable.Act.vel),\
//					(int)(turnTableMsg.pos), (int)(turnTableMsg.vel));
				
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("AP %d %d %d %d %d %d "),(int)(gRobot.archeryStruct.archeryPitch.Act.pos),(int)(gRobot.archeryStruct.archeryPitch.Target.pos),\
//					(int)(gRobot.archeryStruct.archeryPitch.motorVelTarget.pos), (int)(gRobot.archeryStruct.archeryPitch.Act.vel),\
//					(int)(archeryPitchMsg.pos), (int)(archeryPitchMsg.vel));
					
//		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("AP %d %d %d %d %d %d "),(int)(gRobot.archeryStruct.archeryPitch.Act.pos),(int)(gRobot.archeryStruct.archeryPitch.Target.pos),\
//					(int)(gRobot.archeryStruct.archeryPitch.motorVelTarget.pos), (int)(gRobot.archeryStruct.archeryPitch.Act.vel),\
//					(int)(archeryPitchMsg.pos), (int)(archeryPitchMsg.vel));
					
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("AP %d %d "),(int)(gRobot.archeryStruct.archeryPitch.Act.pos),(int)(gRobot.archeryStruct.archeryPitch.Target.pos));
				
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("AL %d %d %d %d %d %d "),(int)(gRobot.archeryStruct.loadArrow.Act.pos),(int)(gRobot.archeryStruct.loadArrow.Target.pos),\
//					(int)(gRobot.archeryStruct.loadArrow.motorVelTarget.pos), (int)(gRobot.archeryStruct.loadArrow.Act.vel),\
//					(int)(loadArrowMsg.pos), (int)(loadArrowMsg.vel));
					
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("SQ %d AL %d %d "),
					(int)shootArrowMsg.VolIn,(int)(gRobot.archeryStruct.loadArrow.motorVelTarget.pos), (int)(loadArrowMsg.pos));

  USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("E %d %d "),(int)(gRobot.steerStruct.motorState.errorFlag), (int)gRobot.spiState.errorFlag);				

	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
		(uint8_t *)("R %d "),(int)gRobot.robotMode);					
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
	(uint8_t *)("\r\n"));
}

extern int loadArrowCnt;
void DebugDataOut(void)
{	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("T %d %d "),(int)timeCounter,(int)timeCnt);
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("S %d %d %d %d "),(int)gRobot.walkStatus,(int)gRobot.pathPlanFlag,(int)gRobot.robotMode,(int)gRobot.contestStart);
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("S %d %d %d %d R %d %d "),(int)gRobot.walkStatus,(int)(gRobot.ppsData.x * 10), (int)(gRobot.ppsData.y * 10),(int)(gRobot.ppsData.angle * 10),\
					(int)gRobot.TRRetryFlag,(int)gRobot.initialCnt);

	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("SSS %d %d %d %d SF %d %d %d %d %d %d %d %d %d W %d "),\
				(int)gRobot.shotArrowsCnt,(int)gRobot.shootOrder,(int)gRobot.potNum,(int)gRobot.attackPotID[gRobot.shootOrder],\
					(int)resetDone,(int)loadDoneFlag,loadMotorDone,(int)pitchDone,(int)turnDone,\
					(int)gRobot.contestStart,(int)appShootFlag,(int)recAppPitch,(int)canShootFlag,\
						(int)waitForFetchArrow);

  USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
			(uint8_t *)("cv %d %d %d %d %d %d C %d %d F %d "),\
			(int)(gRobot.cvAngleLiner * 100),(int)gRobot.cvStruct.state,(int)(gRobot.cvStruct.cvDir * 100),(int)gRobot.cvCompensate,(int)(gRobot.cvStruct.cvDis*10),(int)gRobot.cvAnomalFlag,\
					(int)(shootPara.shootKp * 100),(int)(shootPara.shootKi * 100),\
					(int)gRobot.archeryStruct.fetchMotorReady);

  USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
			(uint8_t *)("K %d "),\
          (int)gRobot.archeryStruct.archeryStart);
//					
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("O %d %d %d C %d R %d "),(int)gRobot.archeryStruct.archeryStart,(int)gRobot.shotArrowsCnt,\
//					(int)gRobot.archeryStruct.fetchMotorReady,(int)gRobot.fieldCol,(int)gRobot.TRRetryFlag);
	
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("W1 %d %d %d %d "),(int)gRobot.steerStruct.one.wheelVelTarget.vel,(int)gRobot.steerStruct.one.wheelVelAct.vel,\
//					(int)gRobot.steerStruct.one.motorVelTarget.vel,(int)oneWheelMsg.vel);

  USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("AS %d %d %d %d %d %d o %d %d "),
					(int)shootStep,(int)(-shootArrowMsg.pos),(int)(gRobot.archeryStruct.shootArrow.motorVelTarget.vel), (int)(-shootArrowMsg.vel),(int)shootArrowMsg.currQ,\
            (int)(shootArrowMsg.VolIn*100),\
						(int)(shootPara.shootVel*100),(int)(gRobot.velChange*100));
////				
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("AT %d %d %d %d %d "),(int)(Wtable*100),(int)(gRobot.archeryStruct.turnTable.Target.pos * 100),(int)(gRobot.archeryStruct.turnTable.Act.pos * 100),\
					(int)(gRobot.archeryStruct.turnTable.motorVelTarget.pos), (int)(turnTableMsg.pos));
//				
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("AP %d %d "),(int)(gRobot.archeryStruct.archeryPitch.Target.pos * 10),(int)(gRobot.archeryStruct.archeryPitch.Act.pos *10));
				
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("AL %d %d "),(int)(gRobot.archeryStruct.loadArrow.motorVelTarget.pos),\
					(int)(loadArrowMsg.pos));
				
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("AF %d %d %d %d %d "),(int)(gRobot.attackPotID[gRobot.shootOrder]),(int)(gRobot.fieldCol),\
//					(int)(gRobot.archeryStruct.fetchMotorReady), (int)(gRobot.TRRetryFlag), (int)(voltageTemp*10));
//	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("E %d %d %d %d"),(int)gRobot.steerStruct.motorState.errorFlag,\
					(int)gRobot.spiState.errorFlag,(int)gRobot.spiState.errorShiftFlag, (int)gRobot.spiState.spiOffArray);	

   			
  if(gRobot.archeryStruct.archeryStart == 0)
  {
    USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
      (uint8_t *)("W1 %d %d "),
        (int)gRobot.steerStruct.one.motorVelTarget.vel,(int)oneWheelMsg.vel);

    USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
        (uint8_t *)("W2 %d %d "),
          (int)gRobot.steerStruct.two.motorVelTarget.vel,(int)twoWheelMsg.vel);
          
    USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
        (uint8_t *)("W3 %d %d "),
          (int)gRobot.steerStruct.thr.motorVelTarget.vel,(int)thrWheelMsg.vel);
    
    USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
        (uint8_t *)("T1 %d %d %d "),(int)(gRobot.steerStruct.one.wheelVelTarget.pos*10),(int)(gRobot.steerStruct.one.wheelVelAct.pos*10),(int)oneTurnMsg.pos);		

    USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
        (uint8_t *)("T2 %d %d %d "),(int)(gRobot.steerStruct.two.wheelVelTarget.pos*10),(int)(gRobot.steerStruct.two.wheelVelAct.pos*10),(int)twoTurnMsg.pos);

    USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
        (uint8_t *)("T3 %d %d %d "),(int)(gRobot.steerStruct.thr.wheelVelTarget.pos*10),(int)(gRobot.steerStruct.thr.wheelVelAct.pos*10),(int)thrTurnMsg.pos);	
      
  }
		
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("T1 %d %d %d %d %d "),(int)(gRobot.steerStruct.one.wheelVelTarget.pos*10),(int)(gRobot.steerStruct.one.wheelVelAct.pos*10),\
//					(int)gRobot.steerStruct.one.motorVelTarget.pos,(int)oneTurnMsg.pos,(int)oneTurnMsg.vel);			
//				
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("W2 %d %d %d %d "),(int)gRobot.steerStruct.two.wheelVelTarget.vel,(int)gRobot.steerStruct.two.wheelVelAct.vel,\
//					(int)gRobot.steerStruct.two.motorVelTarget.vel,(int)twoWheelMsg.vel);
//	
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("T2 %d %d %d %d %d "),(int)(gRobot.steerStruct.two.wheelVelTarget.pos*10),(int)(gRobot.steerStruct.two.wheelVelAct.pos*10),\
//					(int)gRobot.steerStruct.two.motorVelTarget.pos,(int)twoTurnMsg.pos,(int)twoTurnMsg.vel);		
//	
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("W3 %d %d %d %d "),(int)gRobot.steerStruct.thr.wheelVelTarget.vel,(int)gRobot.steerStruct.thr.wheelVelAct.vel,\
//					(int)gRobot.steerStruct.thr.motorVelTarget.vel,(int)thrWheelMsg.vel);
	
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("T3 %d %d %d %d %d "),(int)(gRobot.steerStruct.thr.wheelVelTarget.pos*10),(int)(gRobot.steerStruct.thr.wheelVelAct.pos*10),\
//					(int)gRobot.steerStruct.thr.motorVelTarget.pos,(int)thrTurnMsg.pos,(int)thrTurnMsg.vel);		
//				
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("Q %d %d %d %d %d %d "),(int)(oneWheelMsg.currQ),(int)(twoWheelMsg.currQ), (int)(thrWheelMsg.currQ),\
//					(int)(oneTurnMsg.currQ), (int)(twoTurnMsg.currQ), (int)(thrTurnMsg.currQ));		
				
	
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("AP %d %d %d %d "),(int)(gRobot.archeryStruct.archeryPitch.Target.pos * 100),(int)(gRobot.archeryStruct.archeryPitch.Act.pos *100),\
//					(int)(gRobot.archeryStruct.archeryPitch.motorVelTarget.pos), (int)(archeryPitchMsg.pos));
//				
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("AL %d %d  %d %d "),
//					(int)(gRobot.archeryStruct.loadArrow.motorVelTarget.pos), (int)(loadArrowMsg.pos), (int)gRobot.shotArrowsCnt,(int)loadArrowCnt);
//////				

				
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
	(uint8_t *)("\r\n"));
}

void ShootDebugDataOut(void)
{	
  USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
	(uint8_t *)("AS %d %d %d %d %d %d o %d %d "),
		(int)shootStep,(int)(-shootArrowMsg.pos),(int)(gRobot.archeryStruct.shootArrow.motorVelTarget.vel), (int)(-shootArrowMsg.vel),(int)shootArrowMsg.currQ,\
            (int)(shootArrowMsg.VolIn*100),(int)(shootPara.shootVel*100),(int)(gRobot.velChange*100));
}

void SelfCheckDebugDataOut(void)
{	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("T %d %d "),(int)timeCounter,(int)timeCnt);
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("W1 %d %d %d %d "),(int)gRobot.steerStruct.one.wheelVelTarget.vel,(int)gRobot.steerStruct.one.wheelVelAct.vel,\
					(int)gRobot.steerStruct.one.motorVelTarget.vel,(int)oneWheelMsg.vel);
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("T1 %d %d %d %d %d "),(int)(gRobot.steerStruct.one.wheelVelTarget.pos*10),(int)(gRobot.steerStruct.one.wheelVelAct.pos*10),\
					(int)gRobot.steerStruct.one.motorVelTarget.pos,(int)oneTurnMsg.pos,(int)oneTurnMsg.vel);			
				
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("W2 %d %d %d %d "),(int)gRobot.steerStruct.two.wheelVelTarget.vel,(int)gRobot.steerStruct.two.wheelVelAct.vel,\
					(int)gRobot.steerStruct.two.motorVelTarget.vel,(int)twoWheelMsg.vel);
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("T2 %d %d %d %d %d "),(int)(gRobot.steerStruct.two.wheelVelTarget.pos*10),(int)(gRobot.steerStruct.two.wheelVelAct.pos*10),\
					(int)gRobot.steerStruct.two.motorVelTarget.pos,(int)twoTurnMsg.pos,(int)twoTurnMsg.vel);		
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("W3 %d %d %d %d "),(int)gRobot.steerStruct.thr.wheelVelTarget.vel,(int)gRobot.steerStruct.thr.wheelVelAct.vel,\
					(int)gRobot.steerStruct.thr.motorVelTarget.vel,(int)thrWheelMsg.vel);
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("T3 %d %d %d %d %d "),(int)(gRobot.steerStruct.thr.wheelVelTarget.pos*10),(int)(gRobot.steerStruct.thr.wheelVelAct.pos*10),\
					(int)gRobot.steerStruct.thr.motorVelTarget.pos,(int)thrTurnMsg.pos,(int)thrTurnMsg.vel);		
				
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("Q %d %d %d %d %d %d "),(int)(oneWheelMsg.currQ),(int)(twoWheelMsg.currQ), (int)(thrWheelMsg.currQ),\
					(int)(oneTurnMsg.currQ), (int)(twoTurnMsg.currQ), (int)(thrTurnMsg.currQ));	
				
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("AP %d %d %d %d %d %d "),(int)(gRobot.archeryStruct.archeryPitch.Act.pos*100),(int)(gRobot.archeryStruct.archeryPitch.Target.pos),\
					(int)(gRobot.archeryStruct.archeryPitch.motorVelTarget.pos), (int)(gRobot.archeryStruct.archeryPitch.Act.vel),\
					(int)(archeryPitchMsg.pos), (int)(archeryPitchMsg.vel));
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
				(uint8_t *)("E %d %d "),(int)gRobot.steerStruct.motorState.errorFlag,(int)gRobot.spiState.errorFlag);		
				
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
	(uint8_t *)("\r\n"));
}

//自检程序
void SelfCheckTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);	
//	SetVelLimit(CAN2, PTP_MODE, ARCHERY_PITCH_ID, 3.f*4096.0f);
	MAGNET_ON;//打开磁铁吸力
	delay_s(2);
	int selfCheckCnt = 0;
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		OSSemSet(PeriodSem, 0, &os_err);
		
		//负责查看程序有无超周期  单位100us
		timeCnt=0;
		//记录周期数 单位10ms
		timeCounter++;
		selfCheckCnt ++;
		if(selfCheckCnt > 7500)
		{
			selfCheckCnt = 7500;
		}
		//读电机信息
		ReadMotorMsg();
		
//		//发送电调信息
//		C610RxData(CAN2); 
		
		//更新实际轮子速度
		UpdataActWheelVel();
		
		//更新射箭机构实际角度
		UpdateActArcheryAngle();
		if(selfCheckCnt < 2000)//20s
		{
			//测试轮子和航向响应
			WheelTurnTest();
		}
		else if(selfCheckCnt < 6000)//30s
		{
			gRobot.steerStruct.one.wheelVelTarget.vel = 0.0f;
			gRobot.steerStruct.two.wheelVelTarget.vel = 0.0f;
			gRobot.steerStruct.thr.wheelVelTarget.vel = 0.0f;
			//测试转盘、上箭、俯仰、射箭响应
			ArcheryTest();
		}	
    else if(selfCheckCnt > 6000)
    {
      photoLevel = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5);
      if(photoLevel == 1)//未触发光电
      {
        SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x00);//关
      }
      else if(photoLevel == 0)
      {
        SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x02);//蓝
      }
    }
    
		//发送速度和方向给底盘电机
//		SendCmd2Drivers();
		MotorControl();
		
		//检查驱动器状态
		CheckDriverState();
//		CheckDriverRec();
		
		//发送调试数据
		SelfCheckDebugDataOut();
	}
}

uint8_t CheckKeySwitch(void)
{	
	static int8_t keyCnt = 0;
	static uint8_t washTouchCnt, checkTouchCnt;	
	keyCnt++;
	if(keyCnt > 100)
	{
		keyCnt = 100;
	}
	if(keyCnt > 10)
		return 1;
	else
	{
		//默认按键不按下 为正常走形模式
		gRobot.robotMode = WALK_MODE;
		//擦轮按键是否被按下
		if(1 == KEY_WASH)
			washTouchCnt++;	
		else
			washTouchCnt = 0;
		if(washTouchCnt >= 8)
		{	
			gRobot.robotMode = WASH_MODE;
		}	
		//自检按键是否被按下
		if(1 == KEY_CHECK)
			checkTouchCnt++;
		else
			checkTouchCnt = 0;
		if(checkTouchCnt >= 8)
		{	
			gRobot.robotMode = SELF_CHECK_MODE;
		}	
		//调参按键是否被按下
//		if(1 == KEY_PARA)
//			paraTouchCnt++;
//		else
//			paraTouchCnt = 0;
//		if(paraTouchCnt >= 8)
//		{	
//			gRobot.robotMode = DEBUG_PARA_MODE;
//		}	
		
	}	
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//	(uint8_t *)("Cnt %d "),(int)washTouchCnt);
	return 0;
}	

//检测重试按键
uint8_t CheckRetrySwitch(void)
{	
	static int8_t keyCnt = 0;
	static uint8_t tetryTouchCnt;	
	keyCnt++;

	//重试按键是否被按下
	if(1 == KEY_RETRY)
	{
		tetryTouchCnt++;
	}
	else
	{
		tetryTouchCnt = 0;
	}
	if(tetryTouchCnt >= 8)
	{	
		gRobot.robotMode = RETRY_MODE;
	}	
	return 1;
}	

void MotorSafeLimit(void)
{
	if(gRobot.archeryStruct.archeryPitch.Act.pos < 15.f)//俯仰电机小于15°//15
	{
		gRobot.archeryStruct.shootArrow.motorVelTarget.vel = 0;
		MAGNET_ON;
	}
//	else
//	{
//		MAGNET_OFF;
//	}
}

void MotorHoldUp(void)
{

	if(gRobot.archeryStruct.archeryStart == 1 && gRobot.walkStatus > 2)
	{
		SetVelLimit(CAN1, PTP_MODE, ONE_TURN_ID , 20*4096);
		SetVelLimit(CAN1, PTP_MODE, TWO_TURN_ID , 20*4096);
		SetVelLimit(CAN1, PTP_MODE, THR_TURN_ID , 20*4096);
	}
	else if(gRobot.archeryStruct.archeryStart == 0)
	{
		SetVelLimit(CAN1, PTP_MODE, ONE_TURN_ID , 10*4096);
		SetVelLimit(CAN1, PTP_MODE, TWO_TURN_ID , 10*4096);
		SetVelLimit(CAN1, PTP_MODE, THR_TURN_ID , 10*4096);
	}
}

//防止重复接收APP指令
void AvoidRepeatRec(void)
{
	static int appRecCnt = 0;
  static int app1ARecCnt = 0;
  static int app1BRecCnt = 0;
  static int app2ARecCnt = 0;
  static int app2BRecCnt = 0;
  static int app3RecCnt = 0;
	if(gRobot.archeryStruct.potRevFlag == 1)
	{
		appRecCnt++;
	}
	if(appRecCnt > 30)
	{
		appRecCnt = 0;
		gRobot.archeryStruct.potRevFlag = 0;
	}
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,
//				(uint8_t *)("AR %d %d "),(int)gRobot.archeryStruct.potRevFlag,(int)appRecCnt);
	if(gRobot.archeryStruct.pot1ARevFlag == 1)//1A
	{
		app1ARecCnt++;
	}
	if(app1ARecCnt > 60)
	{
		app1ARecCnt = 0;
		gRobot.archeryStruct.pot1ARevFlag = 0;
	}

	if(gRobot.archeryStruct.pot1BRevFlag == 1)//1B
	{
		app1BRecCnt++;
	}
	if(app1BRecCnt > 60)
	{
		app1BRecCnt = 0;
		gRobot.archeryStruct.pot1BRevFlag = 0;
	}

	if(gRobot.archeryStruct.pot2ARevFlag == 1)//2A
	{
		app2ARecCnt++;
	}
	if(app2ARecCnt > 60)
	{
		app2ARecCnt = 0;
		gRobot.archeryStruct.pot2ARevFlag = 0;
	}

	if(gRobot.archeryStruct.pot2BRevFlag == 1)//2B
	{
		app2BRecCnt++;
	}
	if(app2BRecCnt > 60)
	{
		app2BRecCnt = 0;
		gRobot.archeryStruct.pot2BRevFlag = 0;
	}

	if(gRobot.archeryStruct.pot3RevFlag == 1)//3
	{
		app3RecCnt++;
	}
	if(app3RecCnt > 60)
	{
		app3RecCnt = 0;
		gRobot.archeryStruct.pot3RevFlag = 0;
	}
}

//使用灯条显示各种状态
void AbnormalJudge(void)
{
	static float cvPreDis = 0;
	static int cvAnomal = 0;
  static int ppsAbnormal = 0;
  if(gRobot.writeFlag != 0 && gRobot.contestStart == 1)
  {
    if(cvPreDis == gRobot.cvStruct.cvDis)
    {
      cvAnomal++;
    }
    else
    {
      cvAnomal = 0;
    }
    if(cvAnomal > 200)
    {
      SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x06);//视觉断开
      gRobot.cvAnomalFlag = 1;
      cvAnomal = 360;
    }
    else if(cvAnomal <= 200 && gRobot.spiState.startTalkFlag == 1)
    {
      switch (gRobot.fieldCol)
      {
        case BLUE_FIELD:
        {
          SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x02);//正常
          break;
        }
        case RED_FIELD:
        {
          SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x01);//正常
          break;
        }
      }
    }
  }
  else
  {
    switch (gRobot.fieldCol)
    {
      case BLUE_FIELD:
      {
        SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x02);//正常
        break;
      }
      case RED_FIELD:
      {
        SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x01);//正常
        break;
      }
    }
  }

  if(gRobot.archeryStruct.archeryMotorState.errorFlag == 1 || gRobot.steerStruct.motorState.errorFlag == 1)//上层机构丢失底盘丢失
  {
    SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x03);
  }
  if(gRobot.spiState.errorFlag == 1)//spi断开
  {
    SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x04);
  }
  else if(gRobot.walkStatus == 44)
  {
    ppsAbnormal++;
    ppsAbnormal = ppsAbnormal % 100;
    if(ppsAbnormal < 50)
    {
      SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x04);
    }
    else
    {
      SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x05);//黄
    }
  }

  if(gRobot.spiState.startTalkFlag == 0)//spi通信未成功
  {
    SET_LIGHTING_STATUS(CAN2,INDICATOR_LIGHT_ID,0x05);//黄
  }
	cvPreDis = gRobot.cvStruct.cvDis;
}

/****************************************************************
* @breaf 发数汇总函数
* @para  voltage--电压 vel--速度 ePos--俯仰位置 tPos--转盘位置
*****************************************************************/

int appCnt = 0;
uint8_t potLine[5];
void SendData2App(void)
{
	for(int i = 0; i < 5; i++)
	{
		potLine[i] = 0;
	}
	for(int i = gRobot.shootOrder, j = 0; ((i < gRobot.shootOrder + 5) && i < gRobot.potNum); i++ , j++ )
	{
		potLine[j] = gRobot.attackPotID[gRobot.shootOrder];
	}
	
	//发送数据给App 50ms 发一次
	appCnt++;
	appCnt = appCnt % 100;
	if(appCnt % 10 == 0)
	{ 
//		SendVoltageToApp(shootArrowMsg.VolIn);										//电压	ATE1
		SendVelToApp(shootPara.shootVel);									//转速	ATV1
//		SendEEAngleToApp(shootPara.archeryPitchAngle);						//俯仰	ATT1
//		SendTurnTableAngle2App(shootPara.turnTableAngle);					//转盘	ATP1
//		SendVisionAngleToApp(gRobot.cvStruct.cvDir);						//视觉误差	ATAA
//		SendDistanceToApp(gRobot.cvStruct.cvDis);							//视觉距离	ATDD

//		SendOKShootToApp(preForShootDone);									//手动可以按射箭	ATRA
//		SendOKStartToApp(preForStartDone);									//可以按开始	ATRB
		SendPotLineToApp(potLine);											//待射桶号	ATRC
	}
}
