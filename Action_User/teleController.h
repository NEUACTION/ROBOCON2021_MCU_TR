#ifndef __TELECONTROLLER_H
#define __TELECONTROLLER_H
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include <ucos_ii.h>
#include "app_cfg.h"
#include "telecontroller.h"
#include "usart.h"
#include "string.h"
#include "dma.h"
#include "motorControl.h"

#define MAX_TELECMD_BYTE_LENGTH	3

#define TELENOCMD								0
#define TELENEXT								1
#define TELESTOP								2
#define TELECLOSE								3
#define TELETRNEXT							4

#define ANGLESHIFT_NONE					0
#define ANGLESHIFT_POSITIVE			1
#define ANGLESHIFT_NEGIATIVE		2


#define STOP	'S'


extern uint8_t teleController;
extern uint8_t teleClampClose;
extern uint8_t trNextFlag;
extern uint8_t trPressureFlag;
extern uint8_t beepOnFlag;



void TeleCmdBufferInput_BT(uint8_t data);
void TeleCmdPross_BT(char* teleMsg);
void TeleCmdBufferInput_WIFI(uint8_t data);
void TeleCmdPross_WIFI(char* teleMsg);
void TeleInit(void);
void MsgReceive_BT(uint8_t ch);
void MsgProcess_BT(char* data);
void MsgJudge_BT(void);

#endif









