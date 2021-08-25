#ifndef __KEY_H
#define __KEY_H
#include "gpio.h"
#include <stdint.h>

#define ON	(1)
#define OFF	(0)
#define KEY_STATUS	(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2))




#endif

