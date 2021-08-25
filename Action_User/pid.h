#ifndef	__PID_H
#define __PID_H
#include "math.h"
#include "driver.h"
#include "motorControl.h"
#ifdef PI
#define PI						(3.1415926535f)
#endif
#define POS_INTEGRAL_LIMIT		(10.0f)
#define POS_OUT_INIT			(4096.0f*50.0f)

#define INTEGRAL_LIMIT			(10.0f)
#define WHEEL_RESET_LIMIT		(4096.0f*10.0f)

#define RESET_LIMIT				(4096.0f*15.0f)

typedef struct
{
	float err;
	float errLast;
	float errIntegral;
	float kp;
	float ki;
	float kd;
	float integral;
  
	float limit;
}pidPara_;

extern pidPara_ shootPosPara;
extern pidPara_ takePosPara;
extern pidPara_ tTablePosPara;
extern pidPara_ elevatorPosPara;

void PidParaInit(void);
int32_t ArrowPosPid(pidPara_ *pidPara,int32_t disierPos,int32_t actulPos);
int32_t ShootPosPid(pidPara_ *pidPara,int32_t disierPos,int32_t actulPos);
#endif
