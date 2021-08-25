#include "pot.h"
#include "archery.h"
#include "robot.h"
#include "blue.h"

/*****************************************************¸射箭俯仰角*******************************************************************/
float shootPitchAngSheet[5][3] = 
{
	{52.f,	54.5f,	54.5f},//2A
	{48.f,	52.f,	52.0f},//2B
	{52.f,	52.f,	54.5f},//1B
	{42.f,	54.5f,	52.f},//1A
	{50.f,	54.5f,	54.5f} //3
};

///******************************************************视觉角度差值******************************************************************/
//³ö·¢Ǹ3ЍͰµċٶȍ
float cvAngleToDisGet[ANGLE_DIS_NUM][VEL_NUM] = 
{
	{7536.f,	0.1f},
	{8337.f,	0.4f}
};

///******************************************************³出发区(9)******************************************************************/
//³ö·¢Ǹ3ЍͰµċٶȍ
float startB9P3disVelGet[SP3_DIS_NUM][VEL_NUM] = 
{
	{7940.f,	99.7f},
	{7980.f,	99.7f}
};

//³ö·¢Ǹ2Bµċٶȍ
float startB9P2BdisVelGet[SP2B_DIS_NUM][VEL_NUM] = 
{
	{9795.f,	111.3f},
	{9813.f,	111.50001f}
};

//³ö·¢Ǹ2Aµċٶȍ
float startB9P2AdisVelGet[SP2A_DIS_NUM][VEL_NUM] = 
{
	{6470.f,	88.2f},
	{6470.f,	88.2001f},
	{6511.f,	88.7f},
	{6530.f,	88.8f}
};



/*****************************************************射1（9）**********************************************************************/
//ɤ¼ýǸ1Aµċٶȍ
float ar1B9P1AdisVelGet[A1P1A_DIS_NUM_B9][VEL_NUM] = 
{
	{4363.f,	71.4f},
	{4375.f,	71.4f}
};

//ɤ¼ýǸ1Bµċٶȍ
float ar1B9P1BdisVelGet[A1P1B_DIS_NUM_B9][VEL_NUM] = 
{
	{6846.f,	90.5f},
	{6853.f,	90.5f}
};

//ɤ¼ýǸ2Aµċٶȍ
float ar1B9P2AdisVelGet[A1P2A_DIS_NUM_B9][VEL_NUM] = 
{
	{3669.f,	64.7f},
	{3675.f,	64.7f},
	{3885.f,	67.6f},
	{3911.f,	67.6f},
	{3972.f,	68.2f},//5
	{3986.f,	68.2f},
	{4062.f,	68.9f},
	{4074.f,	68.9f},
	{4105.f,	69.2f},
	{4118.f,	69.2f},//10
	{4175.f,	69.9f},
	{4186.f,	69.9f},
	{4301.f,	70.6f},
	{4315.f,	70.6f},
	{4371.f,	71.6f},//15
	{4386.f,	71.6f},
	{4444.f,	72.2f},
	{4452.f,	72.2f},
	{4541.f,	73.f},
	{4552.f,	73.f},//20
	{4573.f,	73.2f},
	{4585.f,	73.2f},
	{4609.f,	73.5f},
	{4624.f,	73.5f}
};

//ɤ¼ýǸ2Bµċٶȍ
float ar1B9P2BdisVelGet[A1P2B_DIS_NUM_B9][VEL_NUM] = 
{
	{7369.f,	97.5f},
	{7382.f,	97.5f},
	{7536.f,	98.2f},
	{7548.f,	98.2f},
	{7578.f,	98.9f},//5
	{7587.f,	98.9f},
	{7627.f,	99.4f},
	{7644.f,	99.4f},
	{7662.f,	99.8f},
	{7675.f,	99.8f},//10
	{7734.f,	100.0f},
	{7742.f,	100.0f},
	{7791.f,	100.4f},
	{7803.f,	100.4f},
	{7903.f,	101.5f},//15
	{7915.f,	101.5f},
	{8057.f,	102.5f},
	{8068.f,	102.5f},
	{8189.f,	103.4f},
	{8196.f,	103.4f},//20
	{8212.f,	103.8f},
	{8226.f,	103.8f},
	{8323.f,	104.3f},
	{8337.f,	104.3f}//mark
};

//ɤ¼ýǸ3ЍͰµċٶȍ
float ar1B9P3disVelGet[A1P3_DIS_NUM_B9][VEL_NUM] = 
{
  {5255.f,	81.7f},
  {5353.f,	81.7f},
  {5398.f,	81.7f},
  {5416.f,	83.16f},
  {5437.f,	83.6f},//5
  {5446.f,	83.6f},
  {5540.f,	85.f},
  {5547.f,	85.f},
  {5575.f,	85.2f},
  {5588.f,	85.3f},//10
  {5846.f,	87.3f},
  {5863.f,	87.3f},
  {6032.f,	89.5f},
  {6044.f,	89.5f},
  {6056.f,	89.7f},//15
  {6060.f,	89.7f},
  {6113.f,	89.9f},
  {6126.f,	90.0f}
};

/*****************************************************射2（9）**********************************************************************/
//ɤ¼ýǸ1Aµċٶȍ
float arB9P1AdisVelGet[AP1A_DIS_NUM_B9][VEL_NUM] = 
{
	{7131.f,	91.6f},
	{7146.f,	91.6f}
};

//ɤ¼ýǸ1Bµċٶȍ
float arB9P1BdisVelGet[AP1B_DIS_NUM_B9][VEL_NUM] = 
{
	{4581.f,	72.5f},
	{4592.f,	72.5f}
};

//ɤ¼ýǸ2Aµċٶȍ
float arB9P2AdisVelGet[AP2A_DIS_NUM_B9][VEL_NUM] = 
{
	{3669.f,	64.7f},
	{3675.f,	64.7f},
	{3885.f,	67.6f},
	{3911.f,	67.6f},
	{3972.f,	68.2f},
	{3986.f,	68.2f},
	{4062.f,	68.9f},
	{4074.f,	68.9f},
	{4105.f,	69.2f},
	{4118.f,	69.2f},
	{4175.f,	69.9f},
	{4186.f,	69.9f},
	{4371.f,	71.9f},
	{4386.f,	71.9f},
	{4444.f,	72.5f},
	{4452.f,	72.5f},
	{4541.f,	73.3f},
	{4552.f,	73.3f},
	{4573.f,	73.5f},
	{4585.f,	73.5f},
	{4609.f,	73.8f},
	{4624.f,	73.8f}
};

//ɤ¼ýǸ2Bµċٶȍ
float arB9P2BdisVelGet[AP2B_DIS_NUM_B9][VEL_NUM] = 
{
	{7536.f,	97.f},
	{7548.f,	97.f},
	{7578.f,	97.7f},
	{7587.f,	97.7f},
	{7627.f,	98.2f},
	{7644.f,	98.2f},
	{7662.f,	98.6f},
	{7675.f,	98.6f},
	{7734.f,	98.8f},
	{7742.f,	98.8f},
	{7791.f,	99.2f},
	{7803.f,	99.2f},
	{8057.f,	101.1f},
	{8068.f,	101.1f},
	{8189.f,	102.1f},
	{8196.f,	102.1f},
	{8212.f,	102.3f},
	{8226.f,	102.3f},
	{8323.f,	102.8f},
	{8337.f,	102.8f}
};

//ɤ¼ýǸ3ЍͰµċٶȍ
float arB9P3disVelGet[AP3_DIS_NUM_B9][VEL_NUM] = 
{
  {5437.f,	82.3f},
  {5446.f,	82.3f},
  {5540.f,	83.7f},
  {5547.f,	83.7f},
  {5575.f,	83.9f},
  {5588.f,	84.0f},
  {6032.f,	88.2f},
  {6044.f,	88.2f},
  {6056.f,	88.4f},
  {6060.f,	88.4f},
  {6113.f,	88.6f},
  {6126.f,	88.7f}
};

/******************************************************³出发区(18)******************************************************************/
//3
float startB18P3disVelGet[SP3_DIS_NUM][VEL_NUM] = 
{
	{7950.f,	100.5f},
	{7978.f,	100.6f},
};

//2B
float startB18P2BdisVelGet[SP2B_DIS_NUM][VEL_NUM] = 
{
	{9789.f,	112.2},
	{9822.f,	112.5f}
};

//2A
float startB18P2AdisVelGet[SP2A_DIS_NUM][VEL_NUM] = 
{
	{6497.f,	89.3f},
	{6504.f,	89.5f},
	{6511.f,	89.5f},
	{6530.f,	89.8f}
};
///*****************************************************射1（18）**********************************************************************/
//1A
float ar1B18P1AdisVelGet[A1P1A_DIS_NUM_B18][VEL_NUM] = 
{
	{4581.f,	72.6f},
	{4592.f,	72.6f}
};

//1B
float ar1B18P1BdisVelGet[A1P1B_DIS_NUM_B18][VEL_NUM] = 
{
	
	{7131.f,	92.2f},
	{7146.f,	92.2001f}
};

//2A
float ar1B18P2AdisVelGet[A1P2A_DIS_NUM_B18][VEL_NUM] = 
{
	{3669.f,	66.3f },
	{3675.f,	66.3f},
	{3792.f,	67.6f},
	{3803.f,	67.6f},
	{3885.f,	69.3f},//5
	{3911.f,	69.3f},
	{3972.f,	67.3f},
	{3986.f,	67.3f},
	{4062.f,	70.6f},
	{4074.f,	70.6f},//10
	{4105.f,	70.9f},
	{4118.f,	70.9f},
	{4175.f,	71.6f},
	{4186.f,	71.6f},
	{4248.f,	72.1f},//15
	{4261.f,	72.1f},
	{4301.f,	72.3f},
	{4315.f,	72.3f},
	{4371.f,	73.3f},
	{4386.f,	73.3f},//20
	{4444.f,	73.9f},
	{4452.f,	73.9f},
	{4541.f,	74.7f},
	{4552.f,	74.7f},
	{4573.f,	74.9f},//25
	{4585.f,	74.9f},
	{4609.f,	75.2f},
	{4624.f,	75.2f}

};

//2B
float ar1B18P2BdisVelGet[A1P2B_DIS_NUM_B18][VEL_NUM] = 
{
    {7369.f,	95.8f},
	{7382.f,	95.8f},
	{7536.f,	97.2f},
	{7548.f,	97.2f},
	{7578.f,	97.2f},//5
	{7587.f,	97.2f},
	{7627.f,	98.7f},
	{7644.f,	98.7f},
	{7662.f,	99.1f},
	{7675.f,	99.1f},//10
	{7734.f,	99.3f},
	{7742.f,	99.3f},
	{7791.f,	99.7f},
	{7803.f,	99.7f},
	{7903.f,	100.3f},//15
	{7915.f,	100.3f},
	{8057.f,	101.3f},
	{8068.f,	101.2f},
	{8189.f,	102.7f},
	{8196.f,	102.7f},//20
	{8212.f,	103.1f},
	{8226.f,	103.1f},
	{8323.f,	103.6f},
	{8337.f,	103.6f}//mark
};

//3
float ar1B18P3disVelGet[A1P3_DIS_NUM_B18][VEL_NUM] = 
{	
  {5255.f,	81.7f},
  {5353.f,	81.7f},
  {5398.f,	81.7f},
  {5416.f,	83.16f},
  {5437.f,	83.6f},//5
  {5446.f,	83.6f},
  {5540.f,	85.f},
  {5547.f,	85.f},
  {5575.f,	85.2f},
  {5588.f,	85.3f},//10
  {5846.f,	87.3f},
  {5863.f,	87.3f},
  {6032.f,	89.5f},
  {6044.f,	89.5f},
  {6056.f,	89.7f},//15
  {6060.f,	89.7f},
  {6113.f,	89.9f},
  {6126.f,	90.0f}
};




/*****************************************************射2（18）**********************************************************************/
//ɤ¼ýǸ1Aµċٶȍ
float arB18P1AdisVelGet[AP1A_DIS_NUM_B18][VEL_NUM] = 
{
	{7131.f,	92.2f},
	{7146.f,	92.2001f}
};

//ɤ¼ýǸ1Bµċٶȍ
float arB18P1BdisVelGet[AP1B_DIS_NUM_B18][VEL_NUM] = 
{
	{4581.f,	72.6f},
	{4592.f,	72.6f}
};

//ɤ¼ýǸ2Aµċٶȍ
float arB18P2AdisVelGet[AP2A_DIS_NUM_B18][VEL_NUM] = 
{
	{3669.f,	66.3f },
	{3675.f,	66.3f},
	{3792.f,	67.6f},
	{3803.f,	67.6f},
	{3885.f,	69.3f},//5
	{3911.f,	69.3f},
	{3972.f,	67.3f},
	{3986.f,	67.3f},
	{4062.f,	70.6f},
	{4074.f,	70.6f},//10
	{4105.f,	70.9f},
	{4118.f,	70.9f},
	{4175.f,	71.6f},
	{4186.f,	71.6f},
	{4248.f,	72.1f},//15
	{4261.f,	72.1f},
	{4301.f,	72.3f},
	{4315.f,	72.3f},
	{4371.f,	73.3f},
	{4386.f,	73.3f},//20
	{4444.f,	73.9f},
	{4452.f,	73.9f},
	{4541.f,	74.7f},
	{4552.f,	74.7f},
	{4573.f,	74.9f},//25
	{4585.f,	74.9f},
	{4609.f,	75.2f},
	{4624.f,	75.2f}
};

//ɤ¼ýǸ2Bµċٶȍ
float arB18P2BdisVelGet[AP2B_DIS_NUM_B18][VEL_NUM] = 
{
    {7369.f,	95.8f},
	{7382.f,	95.8f},
	{7536.f,	97.2f},
	{7548.f,	97.2f},
	{7578.f,	97.2f},//5
	{7587.f,	97.2f},
	{7627.f,	98.7f},
	{7644.f,	98.7f},
	{7662.f,	99.1f},
	{7675.f,	99.1f},//10
	{7734.f,	99.3f},
	{7742.f,	99.3f},
	{7791.f,	99.7f},
	{7803.f,	99.7f},
	{7903.f,	100.3f},//15
	{7915.f,	100.3f},
	{8057.f,	101.3f},
	{8068.f,	101.2f},
	{8189.f,	102.7f},
	{8196.f,	102.7f},//20
	{8212.f,	103.1f},
	{8226.f,	103.1f},
	{8323.f,	103.6f},
	{8337.f,	103.6f}//mark
};

//ɤ¼ýǸ3ЍͰµċٶȍ
float arB18P3disVelGet[AP3_DIS_NUM_B18][VEL_NUM] = 
{
  {5255.f,	81.7f},
  {5353.f,	81.7f},
  {5398.f,	81.7f},
  {5416.f,	83.16f},
  {5437.f,	83.6f},//5
  {5446.f,	83.6f},
  {5540.f,	85.f},
  {5547.f,	85.f},
  {5575.f,	85.2f},
  {5588.f,	85.3f},//10
  {5846.f,	87.3f},
  {5863.f,	87.3f},
  {6032.f,	89.5f},
  {6044.f,	89.5f},
  {6056.f,	89.7f},//15
  {6060.f,	89.7f},
  {6113.f,	89.9f},
  {6126.f,	90.0f}
};





//ֻ�о����ת�ٵĹ�ϵ
float GetDisToVel(float velChar[][VEL_NUM],float disAct,int disNum)
{
	static int i = 1;
	float velReturn;
	for(i=1; i<disNum; i++)
	{
		if(disAct < velChar[i][0] && disAct > velChar[i-1][0])
		{
			velReturn = velChar[i-1][1] + (disAct - velChar[i-1][0])*(velChar[i][1] - velChar[i-1][1])/(velChar[i][0] - velChar[i-1][0]);//��������м��ٶ�ֵ
			break;
		}
		else if(disAct == velChar[i-1][0])
		{
			velReturn = velChar[i-1][1];
			break;
		}
		else if(disAct < velChar[0][0])
		{
			velReturn = velChar[0][1] - (velChar[0][0]-disAct)*(velChar[1][1]-velChar[0][1])/(velChar[1][0]-velChar[0][0]);
			break;
		}
		else if(disAct > velChar[disNum - 1][0])
		{
			velReturn = velChar[disNum - 1][1] + (disAct-velChar[disNum - 1][0])*(velChar[disNum - 1][1]-velChar[disNum - 2][1])/(velChar[disNum - 1][0]-velChar[disNum - 2][0]);
			break;
		}
		else if(disAct == velChar[disNum - 1][0])
		{
			velReturn = velChar[disNum - 1][1];
			break;
		}
	}
	if(velReturn > 130.0f)
	{
		velReturn = velChar[disNum - 1][1];
	}
	if(velReturn < 0)
	{
		velReturn = 0;
	}
	return velReturn;
}


/*
 *�ٶ�ѡ��δȷ����ѹ��λ�ã�������ڼ��ٵľ����ϵ����������ƣ���������ѡ���ٶ�
 */
float shootVelGet(float Voltage,int potID)
{
	float vel;
	if(gRobot.walkStatus == 0 || gRobot.ppsData.y < 800.f)//�ڳ�����
	{
		switch (potID)
		{			
			case P2B://2B
			{
//				if(voltageTemp < 24.92f)
//        {
//          vel = 112.3f;
//        }
//        else
//        {
//          vel = 112.3f;
//        }
        if(gRobot.batteryNum == 9)
        {
          if(gRobot.fieldCol == BLUE_FIELD)
          {
            vel = GetDisToVel(startB9P2BdisVelGet_Blue,gRobot.cvStruct.cvDis,SP2B_DIS_NUM_BLUE);
          }
          else if(gRobot.fieldCol == RED_FIELD)
          {
            vel = GetDisToVel(startB9P2BdisVelGet,gRobot.cvStruct.cvDis,SP2B_DIS_NUM);
          }
           
        }
        else
        {
          if(gRobot.fieldCol == BLUE_FIELD)
          {
            vel = GetDisToVel(startB18P2BdisVelGet_Blue,gRobot.cvStruct.cvDis,SP2B_DIS_NUM_BLUE);
          }
          else if(gRobot.fieldCol == RED_FIELD)
          {
            vel = GetDisToVel(startB18P2BdisVelGet,gRobot.cvStruct.cvDis,SP2B_DIS_NUM);
          }

        }
				gRobot.velCalculate = vel;
			}
			break;

			case P3://3
			{
//				if(voltageTemp < 24.97f)
//        {
//          vel = 101.7f;
//        }
//        else
//        {
//          vel = 101.7f;
//        }
//				gRobot.velCalculate = GetDisToVel(startP3disVelGet,gRobot.cvStruct.cvDis,SP2B_DIS_NUM);
          if(gRobot.batteryNum == 9)
          {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
              vel = GetDisToVel(startB9P3disVelGet_Blue,gRobot.cvStruct.cvDis,SP3_DIS_NUM_BLUE);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
              vel = GetDisToVel(startB9P3disVelGet,gRobot.cvStruct.cvDis,SP3_DIS_NUM);
            }
          }
          else
          {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
              vel = GetDisToVel(startB18P3disVelGet_Blue,gRobot.cvStruct.cvDis,SP3_DIS_NUM_BLUE);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
              vel = GetDisToVel(startB18P3disVelGet,gRobot.cvStruct.cvDis,SP3_DIS_NUM);
            }
          }
          gRobot.velCalculate = vel;
          break;
			}
      case P2A:
      {
        if(gRobot.batteryNum == 9)
          {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
              vel = GetDisToVel(startB9P2AdisVelGet_Blue,gRobot.cvStruct.cvDis,SP2A_DIS_NUM_BLUE);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
              vel = GetDisToVel(startB9P2AdisVelGet,gRobot.cvStruct.cvDis,SP2A_DIS_NUM);
            }
          }
          else
          {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
              vel = GetDisToVel(startB18P2AdisVelGet_Blue,gRobot.cvStruct.cvDis,SP2A_DIS_NUM_BLUE);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
              vel = GetDisToVel(startB18P2AdisVelGet,gRobot.cvStruct.cvDis,SP2A_DIS_NUM);
            }

          }
          gRobot.velCalculate = vel;
      }
			break;
			
			default:
				vel = 109.6f;
				break;
    }
	
	}
  else if(gRobot.walkStatus == 32 || gRobot.walkStatus == 20 || (gRobot.ppsData.y > 1000.f && gRobot.ppsData.y < 3000.f))//2�������
	{
		switch (potID)
		{
			case P2A://2A
			{
//				vel = 73.0f;
        if(gRobot.batteryNum == 9)
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
               vel = GetDisToVel(arB9P2AdisVelGet_Blue,gRobot.cvStruct.cvDis,AP2A_DIS_NUM_BLUE_B9);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
               vel = GetDisToVel(arB9P2AdisVelGet,gRobot.cvStruct.cvDis,AP2A_DIS_NUM_B9);
            }
          
        }
        else
        {
             if(gRobot.fieldCol == BLUE_FIELD)
            {
                vel = GetDisToVel(arB18P2AdisVelGet_Blue,gRobot.cvStruct.cvDis,AP2A_DIS_NUM_BLUE_B18);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
                vel = GetDisToVel(arB18P2AdisVelGet,gRobot.cvStruct.cvDis,AP2A_DIS_NUM_B18);
            }
          
        }
				
				gRobot.velCalculate = vel;
			}
			break;
			
			case P2B://2B
			{
//				vel = 90.0f;
        if(gRobot.batteryNum == 9)
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
                vel = GetDisToVel(arB9P2BdisVelGet_Blue,gRobot.cvStruct.cvDis,AP2B_DIS_NUM_BLUE_B9);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
                vel = GetDisToVel(arB9P2BdisVelGet,gRobot.cvStruct.cvDis,AP2B_DIS_NUM_B9);
            }            

        }
        else
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
                vel = GetDisToVel(arB18P2BdisVelGet_Blue,gRobot.cvStruct.cvDis,AP2B_DIS_NUM_BLUE_B18);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
                vel = GetDisToVel(arB18P2BdisVelGet,gRobot.cvStruct.cvDis,AP2B_DIS_NUM_B18);
            }

        }
        
				gRobot.velCalculate = vel;
			}
			break;
			
			case P1B://1b
			{
//				vel = 90.0f;
        if(gRobot.batteryNum == 9)
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
                vel = GetDisToVel(arB9P1BdisVelGet_Blue,gRobot.cvStruct.cvDis,AP1B_DIS_NUM_BLUE_B9);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
                vel = GetDisToVel(arB9P1BdisVelGet,gRobot.cvStruct.cvDis,AP1B_DIS_NUM_B9);   
            }
        }
        else
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
                vel = GetDisToVel(arB18P1BdisVelGet_Blue,gRobot.cvStruct.cvDis,AP1B_DIS_NUM_BLUE_B18);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
                 vel = GetDisToVel(arB18P1BdisVelGet,gRobot.cvStruct.cvDis,AP1B_DIS_NUM_B18);
            }
          
        }
       
				gRobot.velCalculate = vel;
			}	
			break;
			
			case P1A://1A
			{
//				vel = 70.0f;
        if(gRobot.batteryNum == 9)
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
                vel = GetDisToVel(arB9P1AdisVelGet_Blue,gRobot.cvStruct.cvDis,AP1A_DIS_NUM_BLUE_B9);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
                vel = GetDisToVel(arB9P1AdisVelGet,gRobot.cvStruct.cvDis,AP1A_DIS_NUM_B9);
            }

        }
        else
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
                vel = GetDisToVel(arB18P1AdisVelGet_Blue,gRobot.cvStruct.cvDis,AP1A_DIS_NUM_BLUE_B18);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
                vel = GetDisToVel(arB18P1AdisVelGet,gRobot.cvStruct.cvDis,AP1A_DIS_NUM_B18);
            }

        }
        
				gRobot.velCalculate = vel;
			}
			break;
			
			case P3://3
			{
//				vel = 80.0f;
        if(gRobot.batteryNum == 9)
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
                vel = GetDisToVel(arB9P3disVelGet_Blue,gRobot.cvStruct.cvDis,AP3_DIS_NUM_BLUE_B9);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
                vel = GetDisToVel(arB9P3disVelGet,gRobot.cvStruct.cvDis,AP3_DIS_NUM_B9);
            }

        }
        else
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
                vel = GetDisToVel(arB18P3disVelGet_Blue,gRobot.cvStruct.cvDis,AP3_DIS_NUM_BLUE_B18);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
                vel = GetDisToVel(arB18P3disVelGet,gRobot.cvStruct.cvDis,AP3_DIS_NUM_B18);
            }

        }
        
				gRobot.velCalculate = vel;
			}
			break;
			
			default:
				vel = 80.0f;
				break;
		}
	}
	else if(gRobot.ppsData.y > 8000.f)
	{
		switch (potID)
		{
			case P2A://2A
			{
//				vel = 73.0f;
        if(gRobot.batteryNum == 9)
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
                vel = GetDisToVel(ar1B9P2AdisVelGet_Blue,gRobot.cvStruct.cvDis,A1P2A_DIS_NUM_BLUE_B9);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
                vel = GetDisToVel(ar1B9P2AdisVelGet,gRobot.cvStruct.cvDis,A1P2A_DIS_NUM_B9);
            }

        }
        else
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
                vel = GetDisToVel(ar1B18P2AdisVelGet_Blue,gRobot.cvStruct.cvDis,A1P2A_DIS_NUM_BLUE_B18);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
                vel = GetDisToVel(ar1B18P2AdisVelGet,gRobot.cvStruct.cvDis,A1P2A_DIS_NUM_B18);
            }

        }
				
				gRobot.velCalculate = vel;
			}
			break;
			
			case P2B://2B
			{
//				vel = 90.0f;
        if(gRobot.batteryNum == 9)
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
            vel = GetDisToVel(ar1B9P2BdisVelGet_Blue,gRobot.cvStruct.cvDis,A1P2B_DIS_NUM_BLUE_B9);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
            vel = GetDisToVel(ar1B9P2BdisVelGet,gRobot.cvStruct.cvDis,A1P2B_DIS_NUM_B9);
            }

        }
        else
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
           vel = GetDisToVel(ar1B18P2BdisVelGet_Blue,gRobot.cvStruct.cvDis,A1P2B_DIS_NUM_BLUE_B18);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
           vel = GetDisToVel(ar1B18P2BdisVelGet,gRobot.cvStruct.cvDis,A1P2B_DIS_NUM_B18);
            }

        }
        
				gRobot.velCalculate = vel;
			}
			break;
			
			case P1B://1B
			{
//				vel = 90.0f;
        if(gRobot.batteryNum == 9)
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
            vel = GetDisToVel(ar1B9P1BdisVelGet_Blue,gRobot.cvStruct.cvDis,A1P1B_DIS_NUM_BLUE_B9);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
            vel = GetDisToVel(ar1B9P1BdisVelGet,gRobot.cvStruct.cvDis,A1P1B_DIS_NUM_B9);
            }

        }
        else
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
           vel = GetDisToVel(ar1B18P1BdisVelGet_Blue,gRobot.cvStruct.cvDis,A1P1B_DIS_NUM_BLUE_B18);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
           vel = GetDisToVel(ar1B18P1BdisVelGet,gRobot.cvStruct.cvDis,A1P1B_DIS_NUM_B18);
            }

        }
        
				gRobot.velCalculate = vel;
			}	
			break;
			
			case P1A://1A
			{
//				vel = 70.0f;
        if(gRobot.batteryNum == 9)
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
            vel = GetDisToVel(ar1B9P1AdisVelGet_Blue,gRobot.cvStruct.cvDis,A1P1A_DIS_NUM_BLUE_B9);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
            vel = GetDisToVel(ar1B9P1AdisVelGet,gRobot.cvStruct.cvDis,A1P1A_DIS_NUM_B9);
            }

        }
        else
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
           vel = GetDisToVel(ar1B18P1AdisVelGet_Blue,gRobot.cvStruct.cvDis,A1P1A_DIS_NUM_BLUE_B18);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
           vel = GetDisToVel(ar1B18P1AdisVelGet,gRobot.cvStruct.cvDis,A1P1A_DIS_NUM_B18);
            }

        }
        
				gRobot.velCalculate = vel;
			}
			break;
			
			case P3://3
			{
//				vel = 80.0f;
        if(gRobot.batteryNum == 9)
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
            vel = GetDisToVel(ar1B9P3disVelGet_Blue,gRobot.cvStruct.cvDis,A1P3_DIS_NUM_BLUE_B9);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
            vel = GetDisToVel(ar1B9P3disVelGet,gRobot.cvStruct.cvDis,A1P3_DIS_NUM_B9);
            }

        }
        else
        {
            if(gRobot.fieldCol == BLUE_FIELD)
            {
           vel = GetDisToVel(ar1B18P3disVelGet_Blue,gRobot.cvStruct.cvDis,A1P3_DIS_NUM_BLUE_B18);
            }
            else if(gRobot.fieldCol == RED_FIELD)
            {
           vel = GetDisToVel(ar1B18P3disVelGet,gRobot.cvStruct.cvDis,A1P3_DIS_NUM_B18);
            }

        }
        
				gRobot.velCalculate = vel;
			}
			break;
			
			default:
				vel = 80.0f;
				break;
		}
	}
	return vel;
}

//
//float firstVelSelect(void)
float P3VelSelect(float perMinus)
{
	float vel;
	if(gRobot.cvStruct.cvDis < 7970.0f)
	{
		vel = 94.0f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 7970.0f && gRobot.cvStruct.cvDis < 7980.0f)
	{
		vel = 94.1f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 7980.0f && gRobot.cvStruct.cvDis < 7990.0f)
	{
		vel = 94.1f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 7990.0f && gRobot.cvStruct.cvDis < 8000.0f)
	{
		vel = 94.3f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 8000.0f && gRobot.cvStruct.cvDis < 8010.0f)
	{
		vel = 94.6f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 8020.0f && gRobot.cvStruct.cvDis < 8030.0f)
	{
		vel = 94.6f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 8030.0f && gRobot.cvStruct.cvDis < 8040.0f)
	{
		vel = 95.9f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 8040.0f && gRobot.cvStruct.cvDis < 8050.0f)
	{
		vel = 95.4f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 8050.0f && gRobot.cvStruct.cvDis < 8060.0f)
	{
		vel = 95.2f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 8060.0f && gRobot.cvStruct.cvDis < 8070.0f)
	{
		vel = 95.4f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 8070.0f && gRobot.cvStruct.cvDis < 8080.0f)
	{
		vel = 95.12f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 8080.0f && gRobot.cvStruct.cvDis < 8090.0f)
	{
		vel = 95.15f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 8090.0f && gRobot.cvStruct.cvDis < 8100.0f)
	{
		vel = 94.7f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 8100.0f)
	{
		vel = 95.2f + perMinus;
	}
	else 
	{
		vel = 95.0;
	}
	return vel;
}

float P2BVelSelect(float perMinus)
{
	float vel;
	if(gRobot.cvStruct.cvDis < 9775.0f)
	{
		vel = 109.1f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 9775.0f && gRobot.cvStruct.cvDis < 9785.0f)
	{
		vel = 108.7f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 9785.0f && gRobot.cvStruct.cvDis < 9795.0f)
	{
		vel = 108.4f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 9795.0f && gRobot.cvStruct.cvDis < 9805.0f)
	{
		vel = 108.8f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 9805.0f && gRobot.cvStruct.cvDis < 9815.0f)
	{
		vel = 108.9f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 9815.0f && gRobot.cvStruct.cvDis < 9825.0f)
	{
		vel = 108.7f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 9825.0f && gRobot.cvStruct.cvDis < 9835.0f)
	{
		vel = 108.4f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 9835.0f && gRobot.cvStruct.cvDis < 9845.0f)
	{
		vel = 108.4f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 9845.0f && gRobot.cvStruct.cvDis < 9855.0f)
	{
		vel = 108.4f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 9855.0f && gRobot.cvStruct.cvDis < 9865.0f)
	{
		vel = 108.1f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 9565.0f && gRobot.cvStruct.cvDis < 9875.0f)
	{
		vel = 108.2f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 9875.0f && gRobot.cvStruct.cvDis < 9885.0f)
	{
		vel = 108.3f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 9885.0f && gRobot.cvStruct.cvDis < 9895.0f)
	{
		vel = 108.4f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 9895.0f && gRobot.cvStruct.cvDis < 9905.0f)
	{
		vel = 108.5f + perMinus;
	}
	else if(gRobot.cvStruct.cvDis >= 9905.0f)
	{
		vel = 108.6f + perMinus;
	}
	else 
	{
		vel = 107.5;
	}
	return vel;
}
