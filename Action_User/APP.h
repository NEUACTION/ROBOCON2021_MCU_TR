#ifndef __APP_H
#define __APP_H

#include "stdint.h"

#define APP_USART UART4
#define ROBOT_USART USART2//两车通信串口

/****************************************************************
* @breaf 发数汇总函数
* @para  voltage--电压 vel--速度 ePos--俯仰位置 tPos--转盘位置
*****************************************************************/
void SendData2BlueTooth(float Voltage,float paraVel,float paraEpos,float paraTpos);

/**************
* @breaf 发送电压
* @para  
***************/
void SendVoltageToApp(float input);

/**************
* @breaf 发数转盘
* @para  
***************/
void SendTurnTableAngle2App(float input);

/**************
* @breaf 发送速度
* @para  
***************/
void SendVelToApp(float input);

/**************
* @breaf 发送俯仰
* @para  
**************/
void SendEEAngleToApp(float input);

void SendShootToApp(void);

//??????
void SendVisionAngleToApp(float input);

//??????
void SendDistanceToApp(float input);


void SendOKShootToApp(uint8_t input);


void SendOKStartToApp(uint8_t input);

void SendPotLineToApp(uint8_t* input);

#endif

