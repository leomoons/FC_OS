/**********************************************************************************************************
 * @文件     w25qxx.c
 * @说明     板载Flash存储芯片驱动,一个page是256个字节，一个sector(扇区)是4096(4K)字节，
 *           一个block(块)是64K字节,flash使用24个Bit来表示地址
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.09
**********************************************************************************************************/
#include "w25qxx.h"
#include "boardConfig.h"

u16 W25QXX_TYPE;					//定义W25QXX芯片型号


/**********************************************************************************************************
*函 数 名: W25QXX_CS
*功能说明: W25QXX Flash CS脚使能或失能
*形    参: 1代表使能，0代表失能
*返 回 值: 无
**********************************************************************************************************/
static void W25QXX_CS(u8 ena)
{
	if(ena)
		GPIO_ResetBits(W25QXX_CS_GPIO, W25QXX_CS_PIN);
	else
		GPIO_SetBits(W25QXX_CS_GPIO, W25QXX_CS_PIN);
}

/**********************************************************************************************************
*函 数 名: W25QXXCSPin_Init
*功能说明: W25QXX CS使能引脚初始化
*形    参: 无
*返 回 值: 是否初始化成功
**********************************************************************************************************/
static void W25QXXCSPin_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;   //推挽输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init ( GPIOA, &GPIO_InitStructure );
	
	//失能FLASH
	GPIO_SetBits(W25QXX_CS_GPIO, W25QXX_CS_PIN);
}


/**********************************************************************************************************
*函 数 名: W25QXX_ReadID
*功能说明: 读取w25qxx信息
*形    参: void
*返 回 值: 芯片ID  0XEF13,表示芯片型号为W25Q80  0XEF14,表示芯片型号为W25Q16 
* 0XEF15,表示芯片型号为W25Q32  0XEF16,表示芯片型号为W25Q64 0XEF17,表示芯片型号为W25Q128 	 
**********************************************************************************************************/
u16 W25QXX_ReadID(void)
{
	u16 Temp = 0;	  
	W25QXX_CS(1);				    
	Spi1_ReadWriteByte(W25X_ManufactDeviceID);//发送读取ID命令	    
	Spi1_ReadWriteByte(0x00); 	    
	Spi1_ReadWriteByte(0x00); 	    
	Spi1_ReadWriteByte(0x00); 	 			   
	Temp|=Spi1_ReadWriteByte(0xFF)<<8;  
	Temp|=Spi1_ReadWriteByte(0xFF);	 
	W25QXX_CS(0);				    
	return Temp;
}

/**********************************************************************************************************
*函 数 名: W25QXX_WriteEnable
*功能说明: 使能w25qxx写
*形    参: void
*返 回 值: void
**********************************************************************************************************/
void W25QXX_WriteEnable(void)
{
	W25QXX_CS(1);
	Spi1_ReadWriteByte(W25X_WriteEnable);
	W25QXX_CS(0);
}

/**********************************************************************************************************
*函 数 名: W25QXX_WriteDisable
*功能说明: 失能w25qxx写
*形    参: void
*返 回 值: void
**********************************************************************************************************/
void W25QXX_WriteDisable(void)
{
	W25QXX_CS(1);
	Spi1_ReadWriteByte(W25X_WriteDisable);
	W25QXX_CS(0);
}

/**********************************************************************************************************
*函 数 名: W25QXX_WaitForEnd
*功能说明: 等待 flash读写结束
*形    参: void
*返 回 值: void
**********************************************************************************************************/
void W25QXX_WaitForEnd(void)
{
	u8 FLASH_STATUS = 0;
	/* Loop as long as the memory is busy with a write cycle */
    do
    {
        /* Select the FLASH: Chip Select low */
        W25QXX_CS(1);
        /* Send "Read Status Register" instruction */
        Spi1_ReadWriteByte( W25X_ReadStatusReg );
        /* Send a dummy byte to generate the clock needed by the FLASH
        and put the value of the status register in FLASH_Status variable */
        FLASH_STATUS = Spi1_ReadWriteByte( DUMMY_BYTE );
        /* Deselect the FLASH: Chip Select high */
        W25QXX_CS(0);
    }while(FLASH_STATUS & 0x01);
}



/**********************************************************************************************************
*函 数 名: W25QXX_Init
*功能说明: W25QXX寄存器配置初始化
*形    参: 无
*返 回 值: 是否初始化成功
**********************************************************************************************************/
u8 W25QXX_Init(void)
{
	W25QXXCSPin_Init();
	W25QXX_TYPE = W25QXX_ReadID();
	return 1;
}

/**********************************************************************************************************
*函 数 名: W25QXX_PageRead
*功能说明: W25QXX在一页（0~65535）内读取少于等于256个字节的数据
*形    参: pBuffer:数据存储区
*          ReadAddr:读取开始的地址(24bit)
*          NumByteToRead:要读取的字节数(最大256),该数不应该超过该页的剩余字节数!!!
*返 回 值: 
**********************************************************************************************************/
void W25QXX_PageRead(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)
{
	/* Select the FLASH: Chip Select low */
    W25QXX_CS(1);

    /* Send "Read from Memory " instruction */
    Spi1_ReadWriteByte ( W25X_ReadData );

    /* Send ReadAddr high nibble address byte to read from */
    Spi1_ReadWriteByte ( ( ReadAddr & 0xFF0000 ) >> 16 );
    /* Send ReadAddr medium nibble address byte to read from */
    Spi1_ReadWriteByte ( ( ReadAddr & 0xFF00 ) >> 8 );
    /* Send ReadAddr low nibble address byte to read from */
    Spi1_ReadWriteByte ( ReadAddr & 0xFF );

    while ( NumByteToRead-- ) /* while there is data to be read */
    {
        /* Read a byte from the FLASH */
        *pBuffer = Spi1_ReadWriteByte ( DUMMY_BYTE );
        /* Point to the next location where the byte read will be saved */
        pBuffer++;
    }

    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS(0);
}



/**********************************************************************************************************
*函 数 名: W25QXX_PageWrite
*功能说明: W25QXX在一页（0~65535）内写入少于等于256个字节的数据
*形    参: pBuffer:数据存储区
*          WriteAddr:开始写入的地址(24bit)
*          NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!
*返 回 值: 
**********************************************************************************************************/
void W25QXX_PageWrite(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
	u16 i;  
	W25QXX_WriteEnable();                  					//SET WRITE
	
	W25QXX_CS(1);                            				//使能器件   
	Spi1_ReadWriteByte(W25X_PageProgram);      				//发送写页命令   
	Spi1_ReadWriteByte((u8)((WriteAddr & 0xFF0000)>>16)); 	//发送24bit地址    
	Spi1_ReadWriteByte((u8)((WriteAddr & 0xFF00)>>8));   
	Spi1_ReadWriteByte((u8)WriteAddr);   
	for(i=0;i<NumByteToWrite;i++)	Spi1_ReadWriteByte(pBuffer[i]);//循环写数  
	W25QXX_CS(0);                            				//取消片选 
	W25QXX_WaitForEnd();					   				//等待写入结束
}

/**********************************************************************************************************
*函 数 名: W25QXX_SectorErase
*功能说明: 清空指定地址的扇区
*形    参: 地址
*返 回 值: void
**********************************************************************************************************/
void W25QXX_SectorErase(uint32_t address)
{
	W25QXX_WriteEnable();
    /* Select the FLASH: Chip Select low */
    W25QXX_CS(1);
    /* Send Sector Erase instruction */
    Spi1_ReadWriteByte ( W25X_SectorErase );
    /* Send SectorAddr high nibble address byte */
    Spi1_ReadWriteByte ( ( address & 0xFF0000 ) >> 16 );
    /* Send SectorAddr medium nibble address byte */
    Spi1_ReadWriteByte ( ( address & 0xFF00 ) >> 8 );
    /* Send SectorAddr low nibble address byte */
    Spi1_ReadWriteByte ( address & 0xFF );
    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS(0);
}

/**********************************************************************************************************
*函 数 名: W25QXX_Write_NoCheck
*功能说明: 无校验写SPI FLASH，必须确保所写地址范围内的数据全部为0xFF，否则在非0xFF处写入的数据将失败
* 		   具有自动换页功能，在指定地址开始写入指定长度的数据，但是要确保地址不越界
*形    参:  pBuffer:数据存储区
*           WriteAddr:开始写入的地址(24bit)
*           NumByteToWrite:要写入的字节数(最大65535)
*返 回 值: void
**********************************************************************************************************/
void W25QXX_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 			 		 
	u16 pageremain;	   
	pageremain=256-WriteAddr%256; //单页剩余的字节数		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//不大于256个字节
	while(1)
	{	   
		W25QXX_PageWrite(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//写入结束了
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //减去已经写入了的字节数
			if(NumByteToWrite>256)pageremain=256; //一次可以写入256个字节
			else pageremain=NumByteToWrite; 	  //不够256个字节了
		}
	};	    
} 


/**********************************************************************************************************
*函 数 名: W25QXX_Read
*功能说明: 在指定地址开始读取指定长度的数据
*形    参: pBuffer: 数据存储区
*		   ReadAddr: 开始读取的地址（24Bit)
*          length: 要读取的字节数（最大65525）
*返 回 值: void
**********************************************************************************************************/
void W25QXX_Read(u8* pBUffer, u32 ReadAddr, u16 length)
{
	u16 i;
	W25QXX_CS(1);							//CSPin使能设备
	Spi1_ReadWriteByte(W25X_ReadData);		//发送读取命令
	Spi1_ReadWriteByte((u8)(ReadAddr>>16)); //发送24Bit地址
	Spi1_ReadWriteByte((u8)(ReadAddr>>8));
	Spi1_ReadWriteByte((u8)(ReadAddr));
	for(i=0; i<length; i++)
	{
		pBUffer[i] = Spi1_ReadWriteByte(DUMMY_BYTE);
	}
	W25QXX_CS(0);
}

/**********************************************************************************************************
*函 数 名: W25QXX_Write
*功能说明: 在指定地址开始写入指定长度的数据
*形    参: WriteAddr: 开始写入的地址（24Bit)
*		   pBuffer: 数据存储区
*          length: 要写入的字节数（最大65525）
*返 回 值: void
**********************************************************************************************************/
u8 W25QXX_BUFFER[4096];
void W25QXX_Write(u8* pBuffer, u32 WriteAddr, u16 length)
{
	u32 secpos;
	u16 secoff;
	u16 secremain;	   
 	u16 i;    
	u8 * W25QXX_BUF;	  
   	W25QXX_BUF=W25QXX_BUFFER;	     
 	secpos=WriteAddr/4096;//扇区地址  
	secoff=WriteAddr%4096;//在扇区内的偏移
	secremain=4096-secoff;//扇区剩余空间大小   
 	//printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//测试用
 	if(length<=secremain)secremain=length;//不大于4096个字节
	while(1) 
	{	
		W25QXX_Read(W25QXX_BUF,secpos*4096,4096);//读出整个扇区的内容
		for(i=0;i<secremain;i++)//校验数据
		{
			if(W25QXX_BUF[secoff+i]!=0XFF)break;//需要擦除  	  
		}
		if(i<secremain)//需要擦除
		{
			W25QXX_SectorErase(secpos);//擦除这个扇区
			for(i=0;i<secremain;i++)	   //复制
			{
				W25QXX_BUF[i+secoff]=pBuffer[i];	  
			}
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);//写入整个扇区  

		}else W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);//写已经擦除了的,直接写入扇区剩余区间. 				   
		if(length==secremain)break;//写入结束了
		else//写入未结束
		{
			secpos++;//扇区地址增1
			secoff=0;//偏移位置为0 	 

		   	pBuffer+=secremain;  //指针偏移
			WriteAddr+=secremain;//写地址偏移	   
		   	length-=secremain;				//字节数递减
			if(length>4096)secremain=4096;	//下一个扇区还是写不完
			else secremain=length;			//下一个扇区可以写完了
		}	 
	};	
}


