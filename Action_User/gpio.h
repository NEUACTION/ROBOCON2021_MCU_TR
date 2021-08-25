#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f4xx_gpio.h"

#define KEY_WASH GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1)//按下擦轮模式按键
#define KEY_RETRY GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)//重试模式
#define KEY_CHECK GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)//自检模式
#define KEY_PARA GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)//调参模式
//#define KEY_PARA 0//调参模式

#define MAGNET_ON	GPIO_ResetBits(GPIOB, GPIO_Pin_12)//吸住磁铁
#define MAGNET_OFF	GPIO_SetBits(GPIOB, GPIO_Pin_12)//松开磁铁

#define MAGNET_ON1	GPIO_ResetBits(GPIOC, GPIO_Pin_10)//吸住磁铁
#define MAGNET_OFF1	GPIO_SetBits(GPIOC, GPIO_Pin_10)//松开磁铁

#define MAGNET_ON2	GPIO_ResetBits(GPIOC, GPIO_Pin_11)//吸住磁铁
#define MAGNET_OFF2	GPIO_SetBits(GPIOC, GPIO_Pin_11)//松开磁铁





//#define BEEP_ON          		 GPIO_SetBits(GPIOE, GPIO_Pin_7)
//#define BEEP_OFF         		 GPIO_ResetBits(GPIOE, GPIO_Pin_7)

void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,
					uint16_t GPIO_Pin,
					GPIOMode_TypeDef GPIO_Mode);

void GPIO_UP_Init_Pins(GPIO_TypeDef *GPIOx,
					uint16_t GPIO_Pin,
					GPIOMode_TypeDef GPIO_Mode);

void KeyInit(void);
void LEDInit(void);
void BeepInit(void);
void PhotoelectricityInit(void);
#endif
