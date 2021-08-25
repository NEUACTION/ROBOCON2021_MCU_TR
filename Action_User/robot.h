#ifndef __ROBOT_H
#define __ROBOT_H


#include "pccomm.h" 
#include "movebase.h"

//红蓝场
#define RED_FIELD (1)
#define BLUE_FIELD (2)
//复位
#define IS_RESETING (0)
#define RESET_DONE (1)

//机器人模式
#define WALK_MODE (0)//走形
#define WASH_MODE (1)//擦轮
#define SELF_CHECK_MODE (2)//自检
#define DEBUG_PARA_MODE (3)//调参
#define RETRY_MODE (4)//重试
//走形模式
#define ATTACK_MODE (2)//进攻
#define DEFENCE_MODE (1)//防守

//复位结构体
typedef struct
{
	//是否复位完成
	uint8_t isResetDone;
}reset_t;

typedef struct
{
  //收到视觉识别号的标志
	uint8_t vision;
	//壶接收标志
	uint8_t potRev;
	//阶段
	uint8_t step;
}flag_t;

typedef struct
{
//	//主控收到的小电脑的视觉角度（十倍）
//	int cvMcuDir;
  //视觉传过来的偏离角度
	float cvDir;
	//视觉传过来的距离
  float cvDis;
	//状态
	uint8_t state;
}cvState_t;

typedef struct
{
  //视觉传过来的偏离角度
	float x;
	//视觉传过来的距离
  float y;
	//状态
	float angle;
}ppsData_t;

typedef struct
{
  //初始射箭个数
  int arrowConfigCnt;
	float x;
	//视觉传过来的距离
  float y;
	//状态
	float angle;
}configData_t;

typedef struct
{
	//舵轮底盘结构体
	steerState_t steerStruct;
	//射箭机构结构体
	archeryState_t archeryStruct;
	//视觉结构体
	cvState_t cvStruct;
	//定位系统结构体
	ppsData_t ppsData;
	//复位变量结构体
	reset_t reset;
	//spi心跳包
	spiState_t spiState;
	//机器人正常走形状态
	uint8_t walkStatus;
	//轨迹规划ing 标志位
	uint8_t pathPlanFlag;
	//机器人模式
	uint8_t robotMode;
	//走形模式
	uint8_t walkMode;
	
	uint8_t fieldCol;
	
	int attackPotID[100];
	
	uint16_t potNum;//收到指令的桶的数量
	
	uint16_t shootOrder;
	
	uint16_t drPotID;
	
	uint16_t drPotRecieve;
	
	//TR重试
	uint8_t TRRetryFlag;
	
	//小电脑开始记录射箭数据标志位
	uint8_t writeFlag;
	
	flag_t flag;
	//比赛开始标志位
	uint8_t contestStart;
	
	uint8_t shotArrowsCnt;
	
	uint8_t arrowLeftCnt;
	
	uint8_t operationMode;
	
	uint16_t magentCnt;
	
	//自动射箭界面手动调节转速
	float velChange;
	uint8_t vChangeFlag;
	
	//根据距离获取的速度参数
	float velCalculate;
	
	//视觉掉线标志位
	uint8_t cvAnomalFlag;

  //手动调整转盘
  float turnOperaAdjust;

  //手动调剂视觉补偿角
  float cvCompensate;
  //手动调节补偿角度标志位
  uint8_t cvComFlag;

  //自检标志位
  uint8_t selfCheckFlag;

  //手动调整转盘标志位
  uint8_t turnOpera;

  //比赛开始上箭使能
  int loadEnable;

  int batteryNum;
  //最开始需要射的箭的数量
   uint8_t initialCnt;

  //视觉线性补偿角度
  float cvAngleLiner;
  
  uint8_t retryLoad;
	
}gRobot_t;

extern gRobot_t gRobot;




#endif
