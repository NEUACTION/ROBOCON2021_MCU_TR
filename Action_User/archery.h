#ifndef ARROW_H
#define ARROW_H
#include "stdint.h"
#include "driver.h"

#define BIG_SHOOT 
//#define SHOOT_TEST

extern float accRatioDis;//算加速度用的距离跟实际加速距离需要减去的距离（失能后加速阶段增加的距离算出来的比例）



#define FULL_CYCLE (57035)//57062
#define CHANG_POS (24500)//12280+4000//铝块脱离的脉冲//
//#define ACC_POS (30000)
//#define FOWARD_POS (CHANG_POS - accRatioDis)//CHANG_POS-4000
#define FOWARD_POS (CHANG_POS - 2800)
#define DOWN_POS (32262)
#define RESET_POS (43680)//43680
#define RESET_CHECK_POS (53235)//铝块反向撞到箭的位置
#define MAX_SHOOT_VEL (150.0f)
#define RATIO (2.0f/1) //这是什么的减速比

#define TURNTABLE_RATIO (2621.878f)//((43092.0f/187.0f)*(4096.0f/360.0f))
#define PITCH_RATIO (1024.0f/15.0f)  //(1310.573f) //(21540/187.0f)   


/*********************************************************************************红场********************************************************************************************/
#define R_CV_ANGLE_1A1 0.6f
#define R_CV_ANGLE_1A2 0.2f
#define R_CV_ANGLE_1B 0.2f
#define R_CV_ANGLE_1B1 0.9f
#define R_CV_ANGLE_2B0 1.1f
#define R_CV_ANGLE_2B1 1.2f
#define R_CV_ANGLE_2B2 0.1f
#define R_CV_ANGLE_2A0 0.7f
#define R_CV_ANGLE_2A1 1.15f
#define R_CV_ANGLE_2A2 0.15f
#define R_CV_ANGLE_30 1.05f
#define R_CV_ANGLE_31 1.05f
#define R_CV_ANGLE_32 0.1f

/*********************************************************************************蓝场********************************************************************************************/
#define B_CV_ANGLE_1A1 0.6f
#define B_CV_ANGLE_1A2 0.2f
#define B_CV_ANGLE_1B 0.2f
#define B_CV_ANGLE_1B1 1.1f
#define B_CV_ANGLE_2B0 0.8f
#define B_CV_ANGLE_2B1 0.9f
#define B_CV_ANGLE_2B2 0.1f
#define B_CV_ANGLE_2A0 0.9f
#define B_CV_ANGLE_2A1 1.35f
#define B_CV_ANGLE_2A2 0.15f
#define B_CV_ANGLE_30 0.5f
#define B_CV_ANGLE_31 0.75f
#define B_CV_ANGLE_32 0.1f

#define ACC_RATIO 11.58f


#define FIR_LOAD_POS (-16661)

//上箭步骤
#define LOAD_FLAG_DEAL 0
#define FIRST_LOAD 1
#define SECOND_LOAD 2
#define THIRD_LOAD 3
#define FOURTH_LOAD 4
 #define FIFTH_LOAD 5

//射箭步骤
#define COLLIMATION_STEP   (0) //瞄准
#define SHOOT_STEP         (1) //射箭
#define RESETandWALK_STEP  (2) //复位
//#define MINUS_RESET        (3) //转回去复位，减小累计误差


////////////////////////////////////////////////////////////////////////////整理后的变量
#define AUTOMATIC_MODE (1)
#define OPERATION_MODE (2)

//射箭地点
#define DEPART_POS 0

//上箭相关变量
#define INIT_ARROW_NUM 3
#define LOAD_PITCH_ANG 45.f
#define LOAD_DELAY_2B 140
#define LOAD_DELAY 120

typedef struct
{
  float shootVel;
	float turnTableAngle;
	float archeryPitchAngle;
	int takeFlag;
	float shootKp;
	float shootKi;
}para_t;

typedef struct
{
  float x;
	float y;
}pos_t;

typedef struct
{
	uint8_t vel2A;
	uint8_t vel2B;
	uint8_t vel1A;
	uint8_t vel1B;
	uint8_t vel3;
}vChangeFlag_t;

#define ResetRange (500)

#define RESETPOINT (20)



enum Step_e{RESET1=0,FUNCTION};
	
enum Pot2_e{P2A = 1,P2B,P1B,P1A,P3};


extern uint8_t waveFlag;
extern uint8_t shootFlag;
extern uint8_t turnFlag;

extern uint8_t pitchFlag;
extern uint8_t loadFlag;

extern float voltageTemp;
extern float voltageRead;

extern float Wtable;

extern para_t shootPara;

extern uint8_t loadStep;
extern uint8_t shootStep;
extern uint8_t disableFlag;


extern int appShootFlag;
extern int appLoadFlag;

extern vChangeFlag_t vChanFlag;

typedef struct
{
	float dir;
	float dis;
  pos_t pos;
}pot_t;



//typedef struct
//{
//	
//}app_t;


//�������
typedef struct
{
	//app_t app;
	pos_t oppos;
	pot_t potPPs;
	pot_t centerPPs;
}archery_t;


extern archery_t archery;



void archeryT(void);

para_t potCloocet(int potID);
void ShootArrow(void);
void LoadArrow(void);
uint8_t FetchArrow(uint8_t startFetch);
uint8_t PrepareForLoadFirstArrow(uint8_t startPrepare);
void UpdateActArcheryAngle(void);
float turntableFollow(void);
float turntableAngleCal(float endX,float endY,pot_t pot,float turnAngle,float turnTable);
float angleLimit(float angle);
void clearData(uint8_t*data,int num);
uint8_t RobotComm(void);//两车通信
pot_t potCentGet(int potID);
//视觉偏离角度选择
float cvAngleSelect(uint8_t trPot);
/*********************************************************************
* @brief  配置射箭俯仰参数
* @note
* @param  potID 桶号
* @retval None
**********************************************************************/
void ArcheryPitchSelect(void);

void limit(float *data,float leftData,float rightData);
/*********************************************************************
* @brief  
* @note
* @param  
* @retval 
**********************************************************************/
void SetTurntablePos (driverMsg_t archery,float aimPos,float para);

/*********************************************************************
* @brief  
* @note
* @param  
* @retval None
**********************************************************************/
void  SetArrowPitchPos (driverMsg_t archery,float aimPos,float para);

/*********************************************************************
* @brief 取箭前准备的动作及取完箭上箭动作函数
* @note  
* @param  
* @retval 
*********************************************************************/
void FetchAndLoadAction(void);
/*********************************************************************
* @brief  复位后对射箭、轮子、俯仰电机的操作
* @note  
* @param  
* @retval 
*********************************************************************/
void motorSetAfterReset(void);

/*********************************************************************
* @brief  接收DR射箭桶号，判断是否一致，若一致，转向下一个桶投箭
* @note  
* @param  
* @retval 
*********************************************************************/
void JudgeDRArrow(void);

/*********************************************************************
* @brief  边瞄准边射箭复位
* @note  
* @param  
* @retval 
*********************************************************************/
uint8_t ResetArchery(void);

/*********************************************************************
* @brief  上箭延时
* @note  
* @param  
* @retval 
*********************************************************************/
uint16_t LoadDelay(int delayTime);

/*********************************************************************
* @brief  转盘延时
* @note  
* @param  
* @retval 
*********************************************************************/
uint16_t TurnDelay(int delayTime);

/*********************************************************************
* @brief  上箭
* @note  
* @param  
* @retval 
*********************************************************************/
uint16_t LoadAction(void);

/*********************************************************************
* @brief  为下一箭的桶号对应的转盘、俯仰、转速赋值
* @note  
* @param  
* @retval 
*********************************************************************/
uint16_t PotAssignment(void);

/*********************************************************************
* @brief  俯仰调整
* @note  
* @param  
* @retval 
*********************************************************************/
uint16_t PitchAdjust(void);

/*********************************************************************
* @brief  转盘调整
* @note  
* @param  
* @retval 
*********************************************************************/
uint16_t TurnAdjust(void);

/*********************************************************************
* @brief  射箭前的电机设置
* @note  
* @param  
* @retval 
*********************************************************************/
void ShootMotorSet(void);

/*********************************************************************
* @brief  参数清零
* @note  
* @param  
* @retval 
*********************************************************************/
void ValueReset(void);

uint8_t LoadPre(uint8_t startLoad);

/*********************************************************************
* @brief 取箭俯仰、转盘角度变化函数
* @note  
* @param  
* @retval 
*********************************************************************/
uint8_t FetchPre(uint8_t startFetch);

/*********************************************************************
* @brief 射箭分段给阶跃
* @note  
* @param  
* @retval 
*********************************************************************/
void ShootSegmentStep(void);

/*********************************************************************
* @brief 射箭直接阶跃
* @note  
* @param  
* @retval 
*********************************************************************/
void ShootBigStep(void);

/*********************************************************************
* @brief 向APP发送数据
* @note  
* @param  
* @retval 
*********************************************************************/
uint16_t AppSendData(void);
#endif


