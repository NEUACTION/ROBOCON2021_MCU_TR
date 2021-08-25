#include "pid.h"

pidPara_ shootPosPara={0};
pidPara_ takePosPara={0};
pidPara_ tTablePosPara={0};
pidPara_ elevatorPosPara={0};
/**
  * @brief  pid������ʼ��
  * @note
  * @param  
  * @retval None
  */
void PidParaInit(void)
{	
	//ȡ���������ת�̡������ǵ�PID����
	takePosPara.kp=3.0f;
	takePosPara.ki=0.0f;
	takePosPara.kd=1.5f;
	
	shootPosPara.kp=2.0f;
	shootPosPara.ki=0.0f;
	shootPosPara.kd=1.0f;
	
	tTablePosPara.kp=2.5f;
	tTablePosPara.ki=0.0f;
	tTablePosPara.kd=0.0f;
	
	elevatorPosPara.kp=2.5f;
	elevatorPosPara.ki=0.0f;
	elevatorPosPara.kd=0.0f;
}



/**
  * @brief  ���λ��Pid
  * @note
  * @param  
  * @retval None
  */
int32_t ShootPosPid(pidPara_ *pidPara,int32_t disierPos,int32_t actulPos)
{
	float pidOut=0; 
	(*pidPara).err=(float)disierPos-(float)actulPos;

	(*pidPara).errIntegral+=(*pidPara).err;
	(*pidPara).integral=(*pidPara).errIntegral*(*pidPara).ki;
	
	//�������޷�
	if(fabsf((*pidPara).integral) > POS_INTEGRAL_LIMIT)
		(*pidPara).integral=(*pidPara).integral/fabsf((*pidPara).integral)*POS_INTEGRAL_LIMIT;
	
	//λ�����ǳ�С��ʱ�����������
	if(fabsf((*pidPara).err) < 0.8f && fabs(((*pidPara).err-(*pidPara).errLast)/0.005f) < 50)
		(*pidPara).integral=0;
	
	pidOut=(*pidPara).err*(*pidPara).kp+(*pidPara).integral+((*pidPara).err-(*pidPara).errLast)*(*pidPara).kd;
	
	
	if(fabs(pidOut) > POS_OUT_INIT)
		pidOut=pidOut/fabs(pidOut)*POS_OUT_INIT;
	
	
	return (int32_t)pidOut;

}

int32_t ArrowPosPid(pidPara_ *pidPara,int32_t disierPos,int32_t actulPos)
{
	float pidOut=0; 
	(*pidPara).err=(float)disierPos-(float)actulPos;

	(*pidPara).errIntegral+=(*pidPara).err;
	(*pidPara).integral=(*pidPara).errIntegral*(*pidPara).ki;
	
	//�������޷�
	if(fabsf((*pidPara).integral) > POS_INTEGRAL_LIMIT)
		(*pidPara).integral=(*pidPara).integral/fabsf((*pidPara).integral)*POS_INTEGRAL_LIMIT;
	
	//λ�����ǳ�С��ʱ�����������
	if(fabsf((*pidPara).err) < 0.8f && fabs(((*pidPara).err-(*pidPara).errLast)/0.005f) < 50)
		(*pidPara).integral=0;
	
	pidOut=(*pidPara).err*(*pidPara).kp+(*pidPara).integral+((*pidPara).err-(*pidPara).errLast)*(*pidPara).kd;
	
	
	if(fabs(pidOut) > POS_OUT_INIT)
		pidOut=pidOut/fabs(pidOut)*POS_OUT_INIT;
	
	//��Ҫ����д�������
	return (int32_t)pidOut;

}
