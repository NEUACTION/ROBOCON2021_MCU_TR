#ifndef __SPI_H
#define __SPI_H

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "gpio.h"

#define SPI1_CS_HIGH	GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define SPI1_CS_LOW		GPIO_ResetBits(GPIOA, GPIO_Pin_4)

#define SPI2_CS_HIGH	GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define SPI2_CS_LOW		GPIO_ResetBits(GPIOB, GPIO_Pin_12)

#define SPI3_CS_HIGH	GPIO_SetBits(GPIOA, GPIO_Pin_15)
#define SPI3_CS_LOW		GPIO_ResetBits(GPIOA, GPIO_Pin_15)

void SPI1Init(void);
void SPI2Init(void);
void SPI3Init(void);
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler);
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler);
void SPI3_SetSpeed(u8 SPI_BaudRatePrescaler);
void SPIWRData(SPI_TypeDef* SPIx,uint8_t *send,uint8_t *read,uint8_t len);
#endif






