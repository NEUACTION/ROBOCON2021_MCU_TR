#ifndef __APP_H
#define __APP_H

#include "stdint.h"

#define APP_USART UART4
#define ROBOT_USART USART2//����ͨ�Ŵ���

/****************************************************************
* @breaf �������ܺ���
* @para  voltage--��ѹ vel--�ٶ� ePos--����λ�� tPos--ת��λ��
*****************************************************************/
void SendData2BlueTooth(float Voltage,float paraVel,float paraEpos,float paraTpos);

/**************
* @breaf ���͵�ѹ
* @para  
***************/
void SendVoltageToApp(float input);

/**************
* @breaf ����ת��
* @para  
***************/
void SendTurnTableAngle2App(float input);

/**************
* @breaf �����ٶ�
* @para  
***************/
void SendVelToApp(float input);

/**************
* @breaf ���͸���
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

