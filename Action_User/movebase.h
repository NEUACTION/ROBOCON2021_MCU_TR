#ifndef __MOVEBASE_H
#define __MOVEBASE_H

#include <stdint.h>
#include "driver.h"

//一号轮子电机的CAN ID号
#define ONE_WHEEL_ID          (1)
//二号轮子电机的CAN ID号
#define TWO_WHEEL_ID      		(3)
//三号轮子电机的CAN ID号
#define THR_WHEEL_ID          (5)
//一号转向电机的CAN ID号
#define ONE_TURN_ID           (2)
//二号转向电机的CAN ID号
#define TWO_TURN_ID      			(4)
//三号转向电机的CAN ID号
#define THR_TURN_ID           (6)
//射箭电机的CAN     ID号
#define SHOOT_ARROW_ID        (7)
//指示灯ID号
#define INDICATOR_LIGHT_ID    (15)
//上箭电机的CAN     ID号
#define LOAD_ARROW_ID         (8)
//转盘电机的CAN     ID号
#define TURN_TABLE_ID         (9)
//俯仰电机的CAN     ID号
#define ARCHERY_PITCH_ID           (10)
//减号，作为电机不同安装方向的公用符号
#define MINUS (-1.0f)
#define MINUS2 (-1)


#define PI (3.1415926535f)

//轮子直径
#define WHEEL_DIAMETER			(52.0f)
//驱动齿轮减速比
#define WHEEL_RATIO				(4.0f/3.0f)
//航向齿轮减速比
#define TURN_RATIO				(4.f/1.0f)
//电机转一圈的脉冲
#define MOTOR_PULSE_PER_ROUND			(4096.0f)

//复位阶段
#define ZERO_RESET 0
#define VEL_DIRECTION 1

//0°初始化位置
#define ONE_ZERO_INIT (0.f)
#define TWO_ZERO_INIT (0.f)
#define THR_ZERO_INIT (0.f)

////航向编码器复位位置，每换一次轮子电机要重测一次
//#define RESET_POS_ONE	((-6182.0f-5461.3f))//-120
//#define RESET_POS_TWO	((-1576.0f))
//#define RESET_POS_THR	((3843.0f+5461.3f))//+120

//航向编码器复位位置，每换一次轮子电机要重测一次
#define RESET_POS_ONE	((-4220.0f-5461.3f))//-120
#define RESET_POS_TWO	((-1546.0f))
#define RESET_POS_THR	((-1604.0f+5461.3f))//+120

//#define RESET_POS_ONE	((-1624.0f))//-120
//#define RESET_POS_TWO	((-1557.0f))
//#define RESET_POS_THR	((-1626.0f))//+120

//初始化位置
//#define ONE_POS_INIT (-60.f)
//#define TWO_POS_INIT (-60.f)
//#define THR_POS_INIT (-60.f)
//#define ONE_POS_INIT (-60.f)
//#define TWO_POS_INIT (0.f)
//#define THR_POS_INIT (60.f)

#define ONE_POS_INIT (-60.f)
#define TWO_POS_INIT (0.f)
#define THR_POS_INIT (60.f) 
//#define START_PLACE_ARCHERY 




//轮子或电机速度结构体
typedef struct
{
	float vel;
	float pos;
}vel_t;


//舵轮整体结构体
typedef struct
{
	//轮子的目标速度结构体
	vel_t wheelVelTarget;
	//轮子的实际速度结构体
	vel_t wheelVelAct;
	//电机的目标速度结构体（脉冲）
	vel_t motorVelTarget;
	//转盘的目标角度结构体
	
}steer_t;
//舵轮整体结构体
typedef struct
{
	//装置目标速度结构体
	vel_t Target;
	//装置实际速度结构体
	vel_t Act;
	//电机的目标速度结构体（脉冲）
	vel_t motorVelTarget;
	
}archeryDevice;


typedef struct
{
	//1号轮心跳包
	uint16_t oneHB;
	//2号轮心跳包
	uint16_t twoHB;
	//3号轮心跳包
	uint16_t thrHB;	
	//1号转向心跳包
	uint16_t oneTurnHB;
	//2号转向心跳包
	uint16_t twoTurnHB;
	//3号转向心跳包
	uint16_t thrTurnHB;
	
	//电机出错标志位
	uint8_t errorFlag;
}motorState_t;
typedef struct
{
	//转盘心跳包
	uint16_t turnTableHB;
	//射箭电机心跳包
	uint16_t shootArrowHB;
	//上箭心跳包
	uint16_t loadArrowHB;	
	//射箭俯仰
	uint16_t archeryPitchHB;	
	//电机出错标志位
	uint8_t errorFlag;
}archerymotorState_t;
typedef struct
{
	//1号舵轮
	steer_t one;
	//2号舵轮
	steer_t two;
	//3号舵轮
	steer_t thr;
	//电机检测结构体
	motorState_t motorState;
	
}steerState_t;
typedef struct
{
	//射箭机构模式
	uint8_t mode;
	//转盘电机
	archeryDevice turnTable;
	//俯仰电机
	archeryDevice archeryPitch;
	//上箭电机
	archeryDevice loadArrow;
	//射箭电机
	archeryDevice shootArrow;
	//电机检测结构体
	archerymotorState_t archeryMotorState;
	//取箭电机到位
	uint8_t fetchMotorReady;
	//射箭命令
	uint8_t archeryStart;
	//APP指令接收
	uint8_t potRevFlag;
  //APP指令接收
	uint8_t pot1ARevFlag;
  //APP指令接收
	uint8_t pot1BRevFlag;
  //APP指令接收
	uint8_t pot2ARevFlag;
  //APP指令接收
	uint8_t pot2BRevFlag;
  //APP指令接收
	uint8_t pot3RevFlag;
}archeryState_t;
extern driverMsg_t oneWheelMsg,twoWheelMsg,thrWheelMsg,oneTurnMsg,twoTurnMsg,thrTurnMsg, testWheelMsg, testTurnMsg,\
										turnTableMsg,loadArrowMsg,shootArrowMsg,archeryPitchMsg;


/**
* @brief  将轮子速度转换为动力电机的脉冲
* @note
* @param  vel:速度（mm/s）
* @param  turnMotorVelPulse:舵向电机速度脉冲
* @retval 脉冲速度
*/
int WheelVel2MotorPulse(float vel,float turnMotorVelPulse);

/**
* @brief  将动力电机的脉冲转化为轮子速度
* @note
* @param  vel:速度（mm/s）
* @param  turnMotorVelPulse:舵向电机速度脉冲
* @retval 脉冲速度
*/
float MotorPulse2WheelVel(float velMotorVelPulse,float turnMotorVelPulse);

/**
* @brief  Vel2Pulse将速度转换为脉冲
* @note
* @param  vel:速度（mm/s）
* @retval 脉冲速度
*/
int Vel2Pulse(float vel);

/**
* @brief  Pulse2Vel将速度转换为脉冲
* @note
* @param  pulse:脉冲速度
* @retval 速度（mm/s）
*/
float Pulse2Vel(int pulse);

/**
* @brief 将航向角度转换为脉冲
* @note
* @param  angle:°
* @retval 
*/
int TurnAngle2Pulse(float angle,int rstPos);

/**
* @brief	得到实际航向电机位置函数
* @note		None
* @param	pulse 脉冲
* @retval	返回此时的角度，单位：度
*/
float Pulse2TurnAngle(int pulse,int rstPos);

/**
* @brief	使能所有电机函数
  * @note	None
  * @param	None
  * @retval	None
  */
void AllMotorOn(void);

/**
* @brief	失能所有电机函数
  * @note	None
  * @param	None
  * @retval	None
  */
void AllMotorOff(void);

/**
* @brief	停止所有电机函数
  * @note	None
  * @param	None
  * @retval	None
  */
void AllMotorStop(void);

/**
* @brief	更新实际轮子速度
  * @note	None
  * @param	None
  * @retval	None
  */
void UpdataActWheelVel(void);

/**
* @brief	发送速度和方向给底盘电机
  * @note	None
  * @param	None
  * @retval	None
  */
void SendCmd2Drivers(void);

/**
* @brief	运动时读取电机信息函数
  * @note	None
  * @param	None
  * @retval	None
  */
void ReadMotorMsg(void);

/**
  * @brief	位置初始化函数
  * @note	此函数为复位完成后转到初始位置
  * @param	None
  * @retval	到位置为1
			未到为0
  */
uint8_t PosInit(void);

/**
* @brief 测试轮子和航向响应
  * @note	None
  * @param	None
  * @retval	None
  */
void WheelTurnTest(void);

/**
* @brief 测试转盘、上箭、俯仰、射箭响应
  * @note	None
  * @param	None
  * @retval	None
  */
void ArcheryTest(void);

/**
* @brief 射箭复位
  * @note	None
  * @param	None
  * @retval	None
  */
void archeryDebug(void);

/**
* @brief 俯仰电机位置环分每个周期阶跃给定
  * @note	None 
  * @param	tStep 阶跃角度(是大于0的值)  tPosTarget最后的目标角度
  * @retval	None
  */
void PitchPosSegmentStep(float tStep, float tPosTarget);

void SlowMotorWalkInit(void);

#endif
