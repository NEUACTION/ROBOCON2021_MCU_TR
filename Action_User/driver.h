#ifndef __DRIVER_H
#define __DRIVER_H

#include "can.h"
#include "math.h"
#include "stdlib.h"
#include "stm32f4xx.h"

/*驱动器发送ID基地址*/
#define DRIVER_CLIENT_BASE_ID	0x280

/*驱动器接收ID基地址*/
#define DRIVER_SERVER_BASE_ID	0x300

/*控制标识符*/
#define IDENTIFIER_DRIVER_STATE				0x01
#define IDENTIFIER_CURR_KP_Q				0x02
#define IDENTIFIER_CURR_KI_Q				0x03
#define IDENTIFIER_SPD_KP					0x04
#define IDENTIFIER_SPD_KI					0x05
#define IDENTIFIER_POS_KP					0x06
#define IDENTIFIER_POS_KD					0x07
#define IDENTIFIER_TORQUE_CTRL				0x08
#define IDENTIFIER_VEL_CTRL					0x09
#define IDENTIFIER_POS_CTRL_ABS				0x0A
#define IDENTIFIER_POS_CTRL_REL				0x0B
#define IDENTIFIER_SET_CTRL_MODE			0x0C
#define IDENTIFIER_SET_ACC					0x0D
#define IDENTIFIER_SET_DEC					0x0E
#define IDENTIFIER_SET_TORQUE_LIMIT			0x0F
#define IDENTIFIER_SET_VEL_LIMIT			0x10
#define IDENTIFIER_SET_POS_LIMIT_UP			0x11
#define IDENTIFIER_SET_POS_LIMIT_LOW		0x12
#define IDENTIFIER_CORRECT_POS_OFFSET		0x13
#define IDENTIFIER_SET_INTEGRAL_CLEARED		0x14
#define IDENTIFIER_SET_LOAD_GAIN_1        	0x17
#define IDENTIFIER_SET_LOAD_GAIN_2        	0x18
#define IDENTIFIER_SET_CONTROL_MODE		    0x19
#define IDENTIFIER_SET_CLEAR_INTEGRAL		0x1A
#define IDENTIFIER_SET_PosAcc    0x1B
#define IDENTIFIER_SET_PosDec    0X1C
#define IDENTIFIER_SET_SECURE_POS           0x1D
#define IDENTIFIER_SET_CurrQ_Limit          0x1E
#define IDENTIFIER_SET_EXPTFINAL_VEL        0x1F




/*读取标识符*/	
#define IDENTIFIER_READ_TORQUE				0x20
#define IDENTIFIER_READ_VEL					0x21
#define IDENTIFIER_READ_POS					0x22
#define IDENTIFIER_READ_ENCODER_POS			0x23
#define IDENTIFIER_READ_VOL_D				0x24
#define IDENTIFIER_READ_CURR_D				0x25
#define IDENTIFIER_READ_VOL_Q				0x26
#define IDENTIFIER_READ_CURR_Q				0x27
#define IDENTIFIER_READ_SPD_LOOP_OUTPUT		0x28
#define IDENTIFIER_READ_POS_LOOP_OUTPUT		0x29
#define IDENTIFIER_READ_TRANSPCB_ADC45_SPI2 0x2A
#define IDENTIFIER_READ_TRANSPCB_ADC67_SPI3 0x2B
#define IDENTIFIER_READ_HALLAB              0x2C
//#define IDENTIFIER_READ_SHANK_STATUS		0x30
#define IDENTIFIER_READ_VOLIN           0x30
#define IDENTIFIER_LIGHTING_STATUS			0x31


/*错误标识符*/
#define IDENTIFIER_ENCODER_ERROR		0xEE
#define IDENTIFIER_HARD_FAULT			0xFF


#define IDENTIFIER_HALL_TEST_ON     0x32         
#define IDENTIFIER_HALL_TEST_OFF     0x33         

/*驱动器指令模式*/
typedef enum{
				PTP_MODE = 0x00, 
				BROADCAST_MODE = 0x40
			} CommandMode;

/*驱动器控制模式*/
typedef enum{
			SPD_CURR_CTRL_MODE = 1, 
			POS_SPD_CURR_CTRL_MODE = 2, 
			POS_CURR_CTRL_MODE = 3,
			TORQUE_CTRL_MODE = 4
			} ControlMode;

/*位置环模式*/
typedef enum{
			ABSOLUTE_MODE = IDENTIFIER_POS_CTRL_ABS, 
			RELATIVE_MODE = IDENTIFIER_POS_CTRL_REL
			} PosLoopMode;

typedef enum{
			HELL_ON = IDENTIFIER_HALL_TEST_ON, 
			HELL_OFF = IDENTIFIER_HALL_TEST_OFF
			} HELL_MODE;
/*主控接收驱动器消息内容结构体*/
typedef struct
{
	int vel;
	int pos;
	int torque;
	int volQ;
	int currQ;
	int volD;
	int currD;
	int encoder;
	int velLoopOut;
	int posLoopOut;
  int32_t VolIn;
	uint8_t encoderErr;
	uint8_t hardFault;
	uint8_t travelSwitch_Status;
	uint16_t thighHallValue[2];
	
	uint16_t leftthighADCValue[2];
	uint32_t leftthighSPIValue;
	
	uint16_t rightthighADCValue[2];
	uint32_t rightthighSPIValue;
	
}driverMsg_t;

/*******************************控制驱动器命令************************************/

/**
* @brief  驱动器状态(使能或失能)
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  state：状态, 范围: 
							ENABLE:使能
							DISABLE:失能
* @author ACTION
* @note 
*/
void DriverState(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, FunctionalState state);

/**
* @brief  电机转矩控制
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  torque: 转矩，单位：毫牛米, 范围：-100 * 1000 ~ 100 * 1000
* @author ACTION
*/
void TorqueCtrl(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, int32_t torque);

/**
* @brief  电机速度控制
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  vel: 速度，单位：脉冲每秒, 范围：-1024 * 4096 ~ 1024 * 4096
* @author ACTION
*/
void VelCtrl(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, int32_t vel);

/**
* @brief  电机位置控制
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  posMode: 位置环运行模式，范围：
				ABSOLUTE_MODE: 绝对位置模式
				RELATIVE_MODE: 相对位置模式
* @param  pos:位置命令，单位：脉冲, 范围：-1024 * 4096 ~ 1024 * 4096
* @author ACTION
*/
void PosCtrl(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, PosLoopMode posMode, int32_t pos);

/**
* @brief  配置驱动器控制模式
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  ctrlMode：控制模式，范围：
			SPD_CURR_CTRL_MODE：速度-电流双闭环模式
			POS_SPD_CURR_CTRL_MODE：位置-速度-电流三闭环模式
			POS_CURR_CTRL_MODE：位置-电流双闭环模式
* @author ACTION
* @note
*/
void SetCtrlMode(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, ControlMode ctrlMode);

/**
* @brief  配置电机加速度与减速度
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  acc：加速度，单位：脉冲每二次方秒, 范围：0 ~ 8192 * 512(512代表一转)
* @param  dec：减速度，单位：脉冲每二次方秒, 范围：0 ~ 8192 * 512(512代表一转)
* @author ACTION
* @note
*/
void SetAccDec(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t acc, uint32_t dec);

/**
* @brief  配置电机最大转矩
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  limit：最大转矩, 单位: 毫牛米, 范围：0 ~ 100 * 1000
* @author ACTION
* @note
*/
void SetTorqueLimit(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t limit);
	
/**
* @brief  配置速度环最大期望速度
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  limit：速度限制，单位：脉冲每秒, 范围：0 ~ 1024 * 4096
* @author ACTION
* @note
*/
void SetVelLimit(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t limit);

/**
* @brief  配置电机位置限制
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  upperLimit：正方向位置限制，单位：脉冲, 范围：-1024 * 4096 ~ 1024 * 4096
* @param  lowerLimit：反方向位置限制，单位：脉冲, 范围：-1024 * 4096 ~ upperLimit
* @author ACTION
* @note
*/
void SetPosLimit(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, int32_t upperLimit, int32_t lowerLimit);

/**
* @brief  配置电流环kp
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  p_current：电流环kp值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetCurrentKP(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t p_current);

/**
* @brief  配置电流环ki
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  i_current：电流环ki值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetCurrentKI(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t i_current);

/**
* @brief  配置速度环kp
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  p_speed：速度环kp值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetSpeedKP(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t p_speed);

/**
* @brief  配置速度环ki
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  i_speed：速度环ki值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetSpeedKI(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t i_speed);

/**
* @brief  配置位置环kp
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  p_pos：位置环kp值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetPosKP(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t p_pos);

/**
* @brief  配置位置环kd
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  d_pos：位置环kd值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetPosKD(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t d_pos);



/**
* @brief  抱死模式（轮子/俯仰）
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式 （因为是只给轮子发的，所以就算是广播模式航向也不会收到）
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  ControlMode：
*               1：切换成位置模式/抱死
*               2：切换成速度模式/解除抱死
* @author ACTION
* @note
*/
void SetWheelControlMode(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint8_t ControlMode);

/**
* @brief  配置电机最大电流限幅
* @param  CANx: 所使用的CAN通道编号
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  limit：最大转矩电流, 单位:毫安, 范围：0 ~ 150 * 1000(mA)
* @author ACTION
* @note
*/
void SetCurrQLimit(CAN_TypeDef* CANx, uint8_t DriverNum, uint32_t CurrQLimit);


/**
* @brief  清除速度环积分
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note
*/
void SetClearIntegral(CAN_TypeDef* CANx, uint8_t DriverNum);

/**
* @brief  位置设置为零，设置保险的位置，设置期望的最终速度
* @param  CANx: 所使用的CAN通道编号
* @param  DriverNum：驱动器ID号，范围：1~64
* @param  
          SecurePos ：
		            含义：保险位置
					单位：脉冲
					范围：-1024 * 4096~1024 * 4096
          ExptFinalVel
                    含义：期望最终速度
					单位：脉冲每秒
					范围：-1024 * 4096~1024 * 4096
* @author ACTION
* @note
*/
void SetPosZero(CAN_TypeDef* CANx, uint8_t DriverNum, int32_t SecurePos, int32_t ExptFinalVel);

/**********************************读取驱动器数据命令*************************************/

/**
* @brief  读取电机电磁转矩
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫牛米
*/
void ReadTorque(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum);

/**
* @brief  读取电机速度
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲每秒
*/
void ReadVel(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum);

/**
* @brief  读取电机速度环输出
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲每秒
*/
void ReadVelLoopOut(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum);
	
/**
* @brief  读取电机位置
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲
*/
void ReadPos(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum);

/**
* @brief  读取电机位置环输出
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲
*/
void ReadPosLoopOut(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum);
	
/**
* @brief  读取编码器脉冲
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲
*/
void ReadEncoder(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum);
	
/**
* @brief  读取电机交轴电压
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫伏
*/
void ReadVolQ(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum);

/**
* @brief  读取电机转矩电流
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫安
*/
void ReadCurrQ(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum);


/**
* @brief  读取电机直轴电压
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫伏
*/
void ReadVolD(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum);

/**
* @brief  读取电机直轴电流
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫安
*/
void ReadCurrD(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum);

/**
* @brief  获取电机消息的值
* @param  *driverMsg：对应电机消息结构体的地址
* @param  *data8: 接收驱动器消息数组的首地址
* @author ACTION
* @note   方便用户使用 
*/
void GetDriverMsg(driverMsg_t *driverMsg, uint8_t *data8);

/*将三个高字节的数据移到低三字节并提取符号位*/
int TransformValue(int data32);

/**
* @brief  读取航向位置
* @param  CANx：所使用的CAN通道编号
* @param  ID：驱动器ID号，范围：，
* @author ACTION
 * @note：
*/
void ReadActualPos(CAN_TypeDef* CANx, uint8_t ID);
/**
* @brief  开启/关闭霍尔
* @param  CANx：所使用的CAN通道编号
* @param  ID：驱动器ID号，范围：
* @param  IDENTIFIER_HALL_TEST：开启或关闭的宏定义
* @author ACTION
 * @note：
*/
void Hall_Ctrl(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, HELL_MODE IDENTIFIER_HALL_TEST);
	

/**
* @brief  读取四足小腿状态(包括速度/位置/触地)
*         #该指令仅限于四足小腿电缸驱动器#
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   
*         速度 单位：脉冲每秒  范围：-1024*4096 ~ 1024*4096
		  位置 单位：脉冲  范围：-1024*4096 ~ 1024*4096
		  触地 0/1
*/
void ReadShankStatus(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum);

/**
* @brief  读取转接板ADC_IN45\SPI2的值
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：转接板ID号(广播模式下为组号)，与驱动器同步, 范围：1~64
* @author ACTION
* @note   单位：
*         ADC_IN4 0-4095
		  ADC_IN5 0-4095
		  SPI2    0-32767
*/
void READ_TRANSPCB_ADC45_SPI2(CAN_TypeDef* CANx, uint8_t DriverNum);

/**
* @brief  读取转接板ADC_IN67\SPI3的值
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：转接板ID号(广播模式下为组号)，与驱动器同步, 范围：1~64
* @author ACTION
* @note   单位：
*         ADC_IN6 0-4095
		  ADC_IN7 0-4095
		  SPI3    0-32767
*/
void READ_TRANSPCB_ADC67_SPI3(CAN_TypeDef* CANx, uint8_t DriverNum);

/**
* @brief  读取小腿霍尔的ADC值
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   范围：
*         HallA 0-4095
		  HallB 0-4095
		  
*/
void ReadHallAB(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum);


/**
* @brief  读取电机母线电压
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫伏
*/
void ReadVolIN(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum);


/**
* @brief  设置灯板颜色
* @param  CANx: 所使用的CAN通道编号
* @param  DriverNum：灯板ID号 15
* @param
		  color :
					含义：灯板颜色
					范围：
					0x01	红色
					0x02	蓝色
					0x03	绿色
					0x04	紫色
					0x05	黄色
					0x06	白色
					0x00	关灯
* @author ACTION
* @note
*/
void SET_LIGHTING_STATUS(CAN_TypeDef* CANx, uint8_t DriverNum,uint8_t color);

/**
* @brief  配置电机位置加速度与减速度
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  acc：加速度，单位：脉冲每二次方秒, 范围：0 ~ 8192 * 512(512代表一转)
* @param  dec：减速度，单位：脉冲每二次方秒, 范围：0 ~ 8192 * 512(512代表一转)
* @author ACTION
* @note
*/
void SetPosAccDec(CAN_TypeDef* CANx, CommandMode CMDmode, uint8_t DriverNum, uint32_t acc, uint32_t dec);

#endif

