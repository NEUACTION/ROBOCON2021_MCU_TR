#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f4xx_gpio.h"

#define KEY_WASH GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1)//���²���ģʽ����
#define KEY_RETRY GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)//����ģʽ
#define KEY_CHECK GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)//�Լ�ģʽ
#define KEY_PARA GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)//����ģʽ
//#define KEY_PARA 0//����ģʽ

#define MAGNET_ON	GPIO_ResetBits(GPIOB, GPIO_Pin_12)//��ס����
#define MAGNET_OFF	GPIO_SetBits(GPIOB, GPIO_Pin_12)//�ɿ�����

#define MAGNET_ON1	GPIO_ResetBits(GPIOC, GPIO_Pin_10)//��ס����
#define MAGNET_OFF1	GPIO_SetBits(GPIOC, GPIO_Pin_10)//�ɿ�����

#define MAGNET_ON2	GPIO_ResetBits(GPIOC, GPIO_Pin_11)//��ס����
#define MAGNET_OFF2	GPIO_SetBits(GPIOC, GPIO_Pin_11)//�ɿ�����





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
