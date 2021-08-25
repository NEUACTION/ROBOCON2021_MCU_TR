#ifndef __PCCOMM_H
#define __PCCOMM_H

#include <stdarg.h>
#include <stdint.h>
#include "dma.h"
#include "archery.h"
//spi发送接收消息字节长度
#define MSG_LEN (57)
//发送接收数据中标志位个数
#define FLAG_NUM (8)
//发送接收数据中浮点变量个数
#define DATAF_NUM (11)

typedef struct
{
	//spi心跳包
	uint8_t spiHB;
	//spi移位心跳包
	uint8_t spiShiftHB;
	//spi数组越界标志位
	uint8_t spiOffArray;
	//spi通信错误标志位
	uint8_t errorFlag;
	//spi通信错误标志位
	uint8_t errorShiftFlag;
	//spi通信开始标志位
	uint8_t startTalkFlag;
}spiState_t;

/**
  * @brief	得到上位机发送来的数据函数
  * @note		接收发送数据和通讯检测都在此函数
  * @param	None 
  * @retval	None
  */
void Talk2PC(void);
	
/**
  * @brief	SendMsg2PC 发送完成之后向DMA填值函数
  * @note		None
  * @param	None
  * @retval	None
  */
void SendMsg2PC(void);

/**
  * @brief	SendOK2PC 接收到上位机正确信息后填值OK\r\n
  * @note		None
  * @param	None
  * @retval	None
  */
//void SendOK2PC(void);

/**
	* @brief	ReadData 将spi读到的上位机数组内容解析给各个变量
  * @note		None
  * @param	n1：u8类型的标志位个数 
	* @param	n2：float类型的数据个数
	* @param	len: rx数组长度 必须等于n1+n2*4！！！
	* @param	tx：需要解析的数组首地址
	* @param	...：n1个u8类型变量取址，n2个float类型变量取址，注意个数！注意只能是u8和float类型变量的地址！！！
  * @retval	None
  */
int8_t ReadData(int n1,int n2,int len,uint8_t* rx, ...);

/**
	* @brief	WriteData 将各个变量存入spi需要给上位机发送的数据内容数组中
  * @note		None
  * @param	n1：u8类型的标志位个数 
	* @param	n2：float类型的数据个数
	* @param	len: rx数组长度 必须等于n1+n2*4！！！
	* @param	tx：需要存入的数组首地址
	* @param	...：n1个u8类型变量，n2个float类型变量，注意个数！注意只能是u8和float类型变量！！！
  * @retval	None
  */
int8_t WriteData(int n1,int n2,int len,uint8_t* tx, ...);

//检查spi通信状态
void CheckSpiState(void);
//更新过后的SPI
uint8_t SPI_ReadWrite_SingleByte(SPI_TypeDef* SPIx,uint8_t writeData);

void SPI_ReadWrite(SPI_TypeDef* SPIx,uint8_t* tx_data,uint8_t* rx_data,uint16_t length);


#endif
