#ifndef __ADC_H
#define __ADC_H	

#include "stdint.h"
#include "stm32f4xx_usart.h"

void Adc_Init(void); 				//ADCͨ����ʼ��
u16  Get_Adc(u8 ch); 				//���ĳ��ͨ��ֵ 
u16 Get_Adc_Average(u8 ch,u8 times);//�õ�ĳ��ͨ����������������ƽ��ֵ  

#endif 