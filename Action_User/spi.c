#include "spi.h"
#include "dma.h"

/**
  * @brief  SPI初始化
  * @param  None
  * @note  	发送使用软件片选，接收使用硬件片选
  * @retval None
  */
void SPI1Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);// 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);//
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1); 	//PA5 SCK SPI1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);	//PA6 MISO SPI1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);	//PA7 MOSI SPI1
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;// 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;// 
	GPIO_Init(GPIOA, &GPIO_InitStructure);// 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;// 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;// 
	GPIO_Init(GPIOA, &GPIO_InitStructure);//

	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI2
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI
	
	SPI_Cmd(SPI1,DISABLE);
	 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // 全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //设置SPI工作模式/；从模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//设置SPI数据大小：8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//串行同步时钟的闲状态为低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//数据捕获于第二个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //硬件片选
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//预分频16，波特率5.25M
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7; //
	SPI_Init(SPI1, &SPI_InitStructure); 

    SPI_Cmd(SPI1, ENABLE);

	
}
/**
  * @brief  SPI设置波特率
  * @param  SPI_BaudRatePrescaler：波特率
  * @note  	SPI速度 = fAPB1/分频系数
			入口参数范围：SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256
			fAPB1 时钟一般为 42Mhz
  * @retval None
  */
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI1->CR1&=0XFFC7;
	SPI1->CR1|=SPI_BaudRatePrescaler;
	SPI_Cmd(SPI1,ENABLE); 
}



/**
  * @brief  SPI初始化
  * @param  None
  * @note  	发送使用软件片选，接收使用硬件片选
  * @retval None
  */
void SPI2Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);// 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);//
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_SPI2); 	//PB12 NSS SPI2
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2); 	//PB13 SCK SPI2
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);	//PB14 MISO SPI2
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);	//PB15 MOSI SPI2
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;// 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;// 
	GPIO_Init(GPIOB, &GPIO_InitStructure);// 
	
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);//复位SPI2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);//停止复位SPI
	
	SPI_Cmd(SPI2,DISABLE);
	 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // 全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave; //设置SPI工作模式/；从模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//设置SPI数据大小：8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//串行同步时钟的闲状态为低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//数据捕获于第二个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard; //硬件片选
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//预分频8，波特率5.25M
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7; //
	SPI_Init(SPI2, &SPI_InitStructure); 

    SPI_Cmd(SPI2, ENABLE);

	
}


/**
  * @brief  SPI设置波特率
  * @param  SPI_BaudRatePrescaler：波特率
  * @note  	SPI速度 = fAPB1/分频系数
			入口参数范围：SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256
			fAPB1 时钟一般为 42Mhz
  * @retval None
  */
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI2->CR1&=0XFFC7;
	SPI2->CR1|=SPI_BaudRatePrescaler;
	SPI_Cmd(SPI2,ENABLE); 
}


/**
  * @brief  SPI初始化
  * @param  None
  * @note  	发送使用软件片选，接收使用硬件片选
  * @retval None
  */
void SPI3Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);// 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);// 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);// 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);//
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI3); 	//PC10 SCK SPI3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI3);	//PC11 MISO SPI3
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SPI3);	//PC12 MOSI SPI3
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;// 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;// 
	GPIO_Init(GPIOA, &GPIO_InitStructure);// 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;// 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;// 
	GPIO_Init(GPIOC, &GPIO_InitStructure);// 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3| GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;// 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;// 
	GPIO_Init(GPIOB, &GPIO_InitStructure);// 
	
	//
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);//复位SPI2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);//停止复位SPI
	
	SPI_Cmd(SPI3,DISABLE);
	 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // 全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //设置SPI工作模式/；从模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//设置SPI数据大小：8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//串行同步时钟的闲状态为低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//数据捕获于第二个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //软件片选
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//预分频8，波特率5.25M
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7; //
	SPI_Init(SPI3, &SPI_InitStructure); 

    SPI_Cmd(SPI3, ENABLE);
	
}

/**
  * @brief  SPI设置波特率
  * @param  SPI_BaudRatePrescaler：波特率
  * @note  	SPI速度 = fAPB1/分频系数
			入口参数范围：SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256
			fAPB1 时钟一般为 42Mhz
  * @retval None
  */
void SPI3_SetSpeed(u8 SPI_BaudRatePrescaler)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI3->CR1&=0XFFC7;
	SPI3->CR1|=SPI_BaudRatePrescaler;
	SPI_Cmd(SPI3,ENABLE); 
}

void SPIWRData(SPI_TypeDef* SPIx,uint8_t *send,uint8_t *read,uint8_t len)
{
	if(SPIx == SPI1)
		SPI1_CS_LOW;
	else if(SPIx == SPI2)
		SPI2_CS_LOW;
	else if(SPIx == SPI3)
		SPI3_CS_LOW;
	
	for(uint8_t i=0;i<len;i++)
	{	
		while (SPI_I2S_GetFlagStatus(SPIx , SPI_I2S_FLAG_TXE) == RESET){};	
			SPI_I2S_SendData(SPIx , send[i]);
		while (SPI_I2S_GetFlagStatus(SPIx , SPI_I2S_FLAG_RXNE) == RESET){};
			read[i]=SPI_I2S_ReceiveData(SPIx); 
	}
	if(SPIx == SPI1)
		SPI1_CS_HIGH;
	else if(SPIx == SPI2)
		SPI2_CS_HIGH;
	else if(SPIx == SPI3)
		SPI3_CS_HIGH;
}


