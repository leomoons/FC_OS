/**********************************************************************************************************
 * @文件     drv_spi.c
 * @说明     SPI驱动，SPI2用于和传感器交互,SPI1用于和flash交互
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.8
**********************************************************************************************************/
#include "drv_spi.h"

#define DUMMY_BYTE 0xFF

/**********************************************************************************************************
*函 数 名: Spi1_Init
*功能说明: SPI1 主机引脚(SCK, MOSI, MISO)初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Spi1_Init(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	//使能GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);	//使能SPI1时钟
	
	/***************GPIOA 5,6,7 初始化设置****************/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	
	/*!< SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*!< SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*!< SPI MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*************SPI1设置******************/
	SPI_I2S_DeInit ( SPI1 );													//SPI1配置恢复默认
    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;	//设置SPI单向或者双向的数据模式：SPI设置为双线双向全双工
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;					//设置SPI工作模式：设置为主SPI
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;					//设置SPI的数据大小：SPI发送接收8位帧结构
    SPI_InitStructure.SPI_CPOL              = SPI_CPOL_High;					//串行同步时钟的空闲状态为高电平
    SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;					//串行同步时钟的第二个跳边沿（上升或下降）数据被采样
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;						//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理：内部NSS信号有SSI位控制
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;			//定义波特率预分频的值：波特率分频值为16
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;					//指定数据传输从MSB位还是LSB位开始：数据传输从MSB位开始
    SPI_InitStructure.SPI_CRCPolynomial     = 7;								//CRC值计算的多项式
    SPI_Init ( SPI1, &SPI_InitStructure );										//根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
	//使能SPI外设
	SPI_Cmd ( SPI1, ENABLE );		
}

/**********************************************************************************************************
*函 数 名: Spi1_ReadWriteByte
*功能说明: SPI1单字节读取
*形    参: 写入的数据
*返 回 值: 读取到的数据
**********************************************************************************************************/
uint8_t Spi1_ReadWriteByte(uint8_t TxData)
{
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}	//等待发射区空
		SPI_I2S_SendData(SPI1, TxData);	//通过外设SPIx发送一个byte数据
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}//等待接收完一个byte
		return SPI_I2S_ReceiveData(SPI1);//返回通过SPIx最近接收的数据
}

/**********************************************************************************************************
*函 数 名: Spi1_Receive
*功能说明: SPI1接收指定长度字节
*形    参: 接收数据缓冲区指针 数据长度
*返 回 值: 无
**********************************************************************************************************/
void Spi1_Receive(uint8_t *pData, uint16_t len)
{
	for(u16 i=0; i<len; i++)
		pData[i] = Spi1_ReadWriteByte(0);	//可能是0xff而不是0
}


/**********************************************************************************************************
*函 数 名: Spi2_Init
*功能说明: SPI2 主机引脚(SCK, MOSI, MISO)初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Spi2_Init(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	//使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);	//使能SPI2时钟
	
	/***************GPIOB 3,4,5 初始化设置****************/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

	/*!< SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*!< SPI MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*!< SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*************SPI2设置******************/
	SPI_I2S_DeInit ( SPI2 );
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);
  
	/* Enable SPI2 */
	SPI_Cmd(SPI2, ENABLE);
}


/**********************************************************************************************************
*函 数 名: Spi2_ReadWriteByte
*功能说明: SPI2单字节读取
*形    参: 写入的数据
*返 回 值: 读取到的数据
**********************************************************************************************************/
uint8_t Spi2_ReadWriteByte(uint8_t TxData)
{
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}	//等待发射区空
		SPI_I2S_SendData(SPI2, TxData);	//通过外设SPIx发送一个byte数据
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){}//等待接收完一个byte
		return SPI_I2S_ReceiveData(SPI2);//返回通过SPIx最近接收的数据
}

/**********************************************************************************************************
*函 数 名: Spi2_Receive
*功能说明: SPI2接收指定长度字节
*形    参: 接收数据缓冲区指针 数据长度
*返 回 值: 无
**********************************************************************************************************/
void Spi2_Receive(uint8_t *pData, uint16_t len)
{
	for(u16 i=0; i<len; i++)
		pData[i] = Spi2_ReadWriteByte(0xff);	//可能是0xff而不是0
}
