#ifndef POT_H
#define POT_H

#define N_0_SHEET_DIS_INDEX  10
#define N_0_SHEET_VOL_INDEX  5

//�Ӿ�ƫ������Բ�ֵ����
#define ANGLE_DIS_NUM 2

//������
#define SP3_DIS_NUM 2
#define SP2B_DIS_NUM 2
#define SP2A_DIS_NUM 4

/*************************************************************************9*****************************************************************************************/
//射1
#define A1P1A_DIS_NUM_B9 2
#define A1P1B_DIS_NUM_B9 2
#define A1P2A_DIS_NUM_B9 24
#define A1P2B_DIS_NUM_B9 24
#define A1P3_DIS_NUM_B9 18
//射2
#define AP1A_DIS_NUM_B9 2
#define AP1B_DIS_NUM_B9 2
#define AP2A_DIS_NUM_B9 22
#define AP2B_DIS_NUM_B9 20
#define AP3_DIS_NUM_B9 12


/*************************************************************************18*****************************************************************************************/
//射1
#define A1P1A_DIS_NUM_B18 2
#define A1P1B_DIS_NUM_B18 2
#define A1P2A_DIS_NUM_B18 28
#define A1P2B_DIS_NUM_B18 24
#define A1P3_DIS_NUM_B18 18
//射2
#define AP1A_DIS_NUM_B18 2
#define AP1B_DIS_NUM_B18 2
#define AP2A_DIS_NUM_B18 28
#define AP2B_DIS_NUM_B18 24
#define AP3_DIS_NUM_B18 18


#define VEL_NUM 2

extern float cvAngleToDisGet[ANGLE_DIS_NUM][VEL_NUM];

typedef struct{
	float dis;
	float voltage;
	float shootVel;
	float kp;
	float ki;
}paraElement_t;
float CalculateShootVel(paraElement_t shootVelSheet[][N_0_SHEET_VOL_INDEX],int sheetDisIndex,int sheetVolIndex, int dis, int voltage);
extern float shootPitchAngSheet[5][3];
//�������ڱ��е�λ�ã�0-14��
float GetshootVel(int pitchNumCnt ,int dis ,int voltage);


float shootVelGet(float Voltage,int potID);

float P3VelSelect(float perMinus);

float P2BVelSelect(float perMinus);

float GetshootVel(int pitchNumCnt ,int dis ,int voltage);

//ֻ�о����ת�ٵĹ�ϵ
float GetDisToVel(float velChar[][VEL_NUM],float disAct,int disNum);

	
#endif
