#include	"config.h"
#include	"msic_device.h"

#define	OPENIWDG	0
static	volatile	__FP_ACC	lvfpacc_firstpress=0.0f,lvfpacc_heightadd=0.0f;//累次高度值
static	volatile	int	lvfpacc_getfirstpresscnt=0;


///////////////////////////////////基本功能操作///////////////////////////////////////////////////
//xn297,2.4g遥控
const unsigned char TEL_ADDRESS_DEF[2][5] = {{0xd1,0x2c,0x3,0x4,0xc1},{0xdb,0x2c,0x0,0x2,0x56}};
const unsigned char TX_ADDRESS_DEF[5] = {0xCC,0xCC,0xCC,0xCC,0xCC};             //RF 地址：接收端和发送端需一致
unsigned char rf_channel[6]={0x3c,0x02,0x21,0x41,0x0b,0x4b};//0x3c;0x02;0x21;0x41;0x0b;0x4b
static	int	rf_lost=0;
static	int	rf_index=0;
unsigned char ucCurrent_Channel;
static	volatile	int	rf_checkerrcnt=0;

/////stm32 inter flash
/*
static	uint16 STMFLASH_ReadHalfWord(uint32 faddr)
{
	return *(volatile	uint16*)faddr; 	
}
static	void STMFLASH_Read(uint32 ReadAddr,uint16 *pBuffer,uint32 NumToRead)   	
{
	uint32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//读取2个字节.
		ReadAddr+=2;//偏移2个字节.	
	}
}
static	void STMFLASH_Write_NoCheck(uint32 WriteAddr,uint16 *pBuffer,uint32 NumToWrite)   
{ 			 		 
	uint16 i;
	for(i=0;i<NumToWrite;i++)
	{
		FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
	    WriteAddr+=2;//地址增加2.
	}  
} 
static	void STMFLASH_Write(uint32 WriteAddr,uint16 *pBuffer,uint32 NumToWrite)	
{ 
	FLASH_Unlock();
	FLASH_ErasePage(WriteAddr);
	STMFLASH_Write_NoCheck(WriteAddr,pBuffer,NumToWrite);
	FLASH_Lock();//上锁
} */

static	unsigned char SPI_RW(unsigned char R_REG)
{
	unsigned char  i;
	for(i = 0; i < 8; i++)
	{
		SCK_LOW;
		if(R_REG & 0x80)
			MOSI_HIGH;

		else
			MOSI_LOW;
		
		R_REG = R_REG << 1;
		SCK_HIGH;
		if( MISO_STATUS )
			R_REG = R_REG | 0x01;
	}
	SCK_LOW;
	return R_REG;
}
static	void RF_WriteReg(unsigned char reg, unsigned char wdata)
{
	CSN_LOW;
	SPI_RW(reg);
	SPI_RW(wdata);
	CSN_HIGH;
}
static	unsigned char ucRF_ReadReg(unsigned char reg)
{
	unsigned char tmp;

	CSN_LOW;
	SPI_RW(reg);
	tmp = SPI_RW(0);
	CSN_HIGH;

	return tmp;
}
static	void RF_WriteBuf(unsigned char reg,unsigned char *pBuf,unsigned char length)
{
	unsigned char j;
	CSN_LOW;
	j = 0;
	SPI_RW(reg);
	for(j = 0;j < length; j++)
		SPI_RW(pBuf[j]);
	
	j = 0;
	CSN_HIGH;
}
static	void RF_ReadBuf(unsigned char reg, unsigned char *pBuf, unsigned char length)
{
	unsigned char byte_ctr;

	CSN_LOW;                    		                                // Set CSN low, init SPI tranaction
	SPI_RW(reg);       		                                                // Select register to write to and read status byte
	for(byte_ctr=0;byte_ctr<length;byte_ctr++)
		pBuf[byte_ctr] = SPI_RW(0);                                                 // Perform SPI_RW to read byte from XN24L01
	CSN_HIGH;                                                                   // Set CSN high again   
}
static	void RF_RxMode(void)
{
	CE_LOW;
	RF_WriteReg(W_REGISTER + CONFIG, 0x0f);                                     // 将RF设置成RX模式
	CE_HIGH;                                                                    // Set CE pin high 开始接收数据
}
#if	0//UNUSE
static	unsigned char RF_GetStatus(void)
{
	return ucRF_ReadReg(STATUS)&0x70;                                           //读取RF的状态 
}
static	void RF_ClearStatus(void)
{
	RF_WriteReg(W_REGISTER + STATUS,0x70);	                                //清除RF的IRQ标志 
}
static	void RF_ClearFIFO(void)
{
	RF_WriteReg(FLUSH_TX, 0);			                                //清除RF 的 TX FIFO		
	RF_WriteReg(FLUSH_RX, 0);                                                   //清除RF 的 RX FIFO	
}
#endif
/*
static	void RF_SetChannel(unsigned char Channel)
{    
	CE_LOW;
	ucCurrent_Channel = Channel;
	RF_WriteReg(W_REGISTER + RF_CH, Channel);
}*/
static	void XN297Initial(void)
{
	unsigned char BB_cal_data[]    = {0x53,0xA4,0x67,0x9C,0x20};                //请勿自行修改此参数 @ 2Mbps
	unsigned char RF_cal_data[]    = {0xDA,0x9A,0xB0,0x61,0xBB,0xAB,0x9C};      //请勿自行修改此参数 @ 2Mbps
	unsigned char Dem_cal_data[]   = {0x0B,0xDF,0xC4,0xA7,0x03};                //请勿自行修改此参数 @ 2Mbps    

	CE_LOW;
	CSN_HIGH;
	SCK_LOW;
	DelayMs(10);                                                               //延时10ms
	ucCurrent_Channel = DEFAULT_CHANNEL;

#if(TRANSMIT_TYPE == TRANS_ENHANCE_MODE)                                        //增强型模式，含有ACK
/*********************use xn297 Enhance*********************/
	RF_WriteReg(FLUSH_TX, 0);		    			
	RF_WriteReg(FLUSH_RX, 0);
	RF_WriteReg(W_REGISTER + STATUS, 0x70);	
	RF_WriteBuf(W_REGISTER + BB_CAL,  BB_cal_data,  5);
	RF_WriteBuf(W_REGISTER + RF_CAL,  RF_cal_data,  7);
	RF_WriteBuf(W_REGISTER + DEM_CAL, Dem_cal_data, 5);

	RF_WriteBuf(W_REGISTER + TX_ADDR,   (unsigned char*)TX_ADDRESS_DEF, 5);     // 写入发送的地址
	RF_WriteBuf(W_REGISTER + RX_ADDR_P0,(unsigned char*)TX_ADDRESS_DEF, 5); 	// 写入RX PIPO 0 的地址

	RF_WriteReg(W_REGISTER + EN_AA,     0x01);     	 			// 使能 Auto.Ack:Pipe0
	RF_WriteReg(W_REGISTER + EN_RXADDR, 0x01);  			        // 使能 Pipe0 RX
	RF_WriteReg(W_REGISTER + SETUP_AW,  0x03);					// 地址宽度：5 bytes
	RF_WriteReg(W_REGISTER + RF_CH,     DEFAULT_CHANNEL);                       //写入工作频率
	RF_WriteReg(W_REGISTER + SETUP_RETR,RETRY_MAX); 				// 设定为5次发送，即最多4次重传	
	RF_WriteReg(W_REGISTER + RX_PW_P0,  PAYLOAD_WIDTH);				// payload length
	RF_WriteReg(W_REGISTER + RF_SETUP,  RF_POWER);   				// TX_PWR:8dBm, Data rate:2Mbps, LNA:HCURR 

	RF_WriteReg(ACTIVATE, 0x73);
	RF_WriteReg(W_REGISTER + DYNPD, 0x01);	
	if(PAYLOAD_WIDTH <33)
		RF_WriteReg(W_REGISTER +FEATURE, 0x04);                                 //切换到32byte模式
	else
		RF_WriteReg(W_REGISTER +FEATURE, 0x1C);                                 //切换到64byte模式     
	RF_WriteReg(ACTIVATE, 0x73); 

#elif(TRANSMIT_TYPE == TRANS_BURST_MODE)                                        //普通模式,无ACK
/*********************use xn297 normal*********************/
	RF_WriteReg(FLUSH_TX, 0);					
	RF_WriteReg(FLUSH_RX, 0);
	RF_WriteReg(W_REGISTER + STATUS, 0x70);	
	RF_WriteBuf(W_REGISTER + BB_CAL,  BB_cal_data,  5);
	RF_WriteBuf(W_REGISTER + RF_CAL,  RF_cal_data,  7);
	RF_WriteBuf(W_REGISTER + DEM_CAL, Dem_cal_data, 5);

	RF_WriteBuf(W_REGISTER + TX_ADDR,   (unsigned char*)TX_ADDRESS_DEF, 5);     // 写入发送的地址
	RF_WriteBuf(W_REGISTER + RX_ADDR_P0,(unsigned char*)TX_ADDRESS_DEF, 5); 	// 写入RX PIPO 0 的地址

	RF_WriteReg(W_REGISTER + EN_AA,     0x00);     	 		        // Disable Auto.ACK
	RF_WriteReg(W_REGISTER + EN_RXADDR, 0x01);  			        // Enable Pipe0
	RF_WriteReg(W_REGISTER + SETUP_AW,  0x03);					// 地址宽度： 5 bytes
	RF_WriteReg(W_REGISTER + RF_CH,     DEFAULT_CHANNEL);                       // 写入工作频率
	RF_WriteReg(W_REGISTER + SETUP_RETR,0x00);                                  // 使用普通传输 	
	RF_WriteReg(W_REGISTER + RX_PW_P0,  PAYLOAD_WIDTH);				// 设定Payload长度
	RF_WriteReg(W_REGISTER + RF_SETUP,  RF_POWER);   				// TX_PWR:8dBm, Datarate:2Mbps, LNA:HCURR 

	RF_WriteReg(ACTIVATE, 0x73);
	RF_WriteReg(W_REGISTER + DYNPD, 0x00);					
	if(PAYLOAD_WIDTH <33)
		RF_WriteReg(W_REGISTER +FEATURE, 0x00);                                 //切换到32byte模式 
	
	else
		RF_WriteReg(W_REGISTER +FEATURE, 0x18);                                 //切换到64byte模式

#endif

	RF_RxMode();
	DelayMs(10);
}

static	void	CheckoutTel(void)
{//对码
	uint8	tmp_buf[10];
	//已成功对码即忽略
	if(ucRF_ReadReg(STATUS)!=STATUS_RX_DR)
		return;

	CE_LOW;
	RF_ReadBuf(R_RX_PAYLOAD,tmp_buf,9);// read receive payload from RX_FIFO buffer
	RF_WriteReg(W_REGISTER + STATUS, 0x70); 									//清除Status
	RF_WriteReg(FLUSH_RX,0);//flush Rx
	CE_HIGH;
/*
	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DOCK))
	{
		if(gvu8_TelTxAddr[0]==tmp_buf[0]&&	gvu8_TelTxAddr[1]==tmp_buf[1]&&	gvu8_TelTxAddr[2]==tmp_buf[2]&&	gvu8_TelTxAddr[3]==tmp_buf[3])
		{
			MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DROP);
		}
		else	
			return;
	}
	else if(TEL_ADDRESS_DEF[0][0]==tmp_buf[0]&&	TEL_ADDRESS_DEF[0][1]==tmp_buf[1]&&	TEL_ADDRESS_DEF[0][2]==tmp_buf[2]&&	TEL_ADDRESS_DEF[0][3]==tmp_buf[3])
	//else if(1)
	{
		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DROP);
	}
	else if(TEL_ADDRESS_DEF[1][0]==tmp_buf[0]&&	TEL_ADDRESS_DEF[1][1]==tmp_buf[1]&&	TEL_ADDRESS_DEF[1][2]==tmp_buf[2]&&	TEL_ADDRESS_DEF[1][3]==tmp_buf[3])
	{
		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DROP);
	}
	else
		{//printf("0x%x,0x%x,0x%x,0x%x,0x%x\r\n",tmp_buf[0],tmp_buf[1],tmp_buf[2],tmp_buf[3],tmp_buf[4]);
		return;}
	*/
	gvu8_TelTxAddr[0]=tmp_buf[0];
	gvu8_TelTxAddr[1]=tmp_buf[1];
	gvu8_TelTxAddr[2]=tmp_buf[2];
	gvu8_TelTxAddr[3]=tmp_buf[3];

	CE_LOW;
	CSN_HIGH;
	SCK_LOW;
	//DelayMs(10);															  //延时10ms
	RF_WriteBuf(W_REGISTER + TX_ADDR,	 (unsigned char*)gvu8_TelTxAddr, 5);	 // 写入发送的地址
	RF_WriteBuf(W_REGISTER + RX_ADDR_P0,(unsigned char*)gvu8_TelTxAddr, 5);   // 写入RX PIPO 0 的地址
	RF_RxMode(); //这句不能省略，否则无法收到数据

	MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DOCK);
	MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DROP);
//	printf("Telecontroller connect,and the brid's TX_Addr=[%x,%x,%x,%x,%x].\r\n",gvu8_TelTxAddr[0],gvu8_TelTxAddr[1],gvu8_TelTxAddr[2],gvu8_TelTxAddr[3],gvu8_TelTxAddr[4]);
}


//I2C,mpu6881 gyro&acc
static	void	I2C_delay(void){int i=20;while(i--);}
static	uint8	I2C_Start(void) 
{ 
	SDA_H; 
	SCL_H; 
	I2C_delay(); 
	if(!SDA_read)return 0; //SDA线为低电平则总线忙,退出 
	SDA_L; 
	I2C_delay(); 
	if(SDA_read) return 0; //SDA线为高电平则总线出错,退出 
	SDA_L; 
	I2C_delay(); 
	return 1; 
} 
static	void	I2C_Stop(void) 
{ 
	SCL_L; 
	I2C_delay(); 
	SDA_L; 
	I2C_delay(); 
	SCL_H; 
	I2C_delay(); 
	SDA_H; 
	I2C_delay(); 
} 
#if	1
static	void	I2C_Ack(void) 
{ 
	SCL_L; 
	I2C_delay(); 
	SDA_L; 
	I2C_delay(); 
	SCL_H; 
	I2C_delay(); 
	SCL_L; 
	I2C_delay(); 
} 
#endif
static	void I2C_NoAck(void) 
{ 
	SCL_L; 
	I2C_delay(); 
	SDA_H; 
	I2C_delay(); 
	SCL_H; 
	I2C_delay(); 
	SCL_L; 
	I2C_delay(); 
} 
static	uint8	I2C_WaitAck(void)   //返回为:=1有ACK,=0无ACK 
{ 
	SCL_L; 
	I2C_delay(); 
	SDA_H; 
	I2C_delay(); 
	SCL_H; 
	I2C_delay(); 
	if(SDA_read) 
	{ 
		SCL_L; 
		return 0; 
	} 
	SCL_L; 
	return 1; 
} 
static	void	I2C_SendByte(uint8 SendByte) //数据从高位到低位// 
{ 
	uint8 i=8; 
	while(i--) 
	{ 
		SCL_L; 
		//I2C_delay(); 
		if(SendByte&0x80) 
		SDA_H;   
		else  
		SDA_L;    
		SendByte<<=1; 
		I2C_delay(); 
		SCL_H; 
		I2C_delay(); 
	} 
	SCL_L; 
} 
static	uint8	I2C_ReceiveByte(void)  //数据从高位到低位// 
{  
	uint8 i=8; 
	uint8 ReceiveByte=0; 

	SDA_H; 
	while(i--) 
	{ 
		ReceiveByte<<=1;       
		SCL_L; 
		I2C_delay(); 
		SCL_H; 
		I2C_delay(); 
		if(SDA_read) 
		{ 
			ReceiveByte|=0x01; 
		} 
	} 
	SCL_L; 
	return ReceiveByte; 
} 
static	uint8	I2C_Write(uint8 sla_address, uint8 sub_address, uint8 data_w) 
{ 
	if (!I2C_Start()) return 0; 
	I2C_SendByte(sla_address<<1);  
	if (!I2C_WaitAck()) 
	{ 
		I2C_Stop();  
		return 0; 
	} 
	I2C_SendByte(sub_address);         
	I2C_WaitAck(); 

	I2C_SendByte(data_w); 
	I2C_WaitAck(); 
	I2C_Stop(); 

	return 1; 
} 


 /*        
static	uint8	I2C_Read(uint8 sla_address, uint8 sub_address) 
{ 
	uint8 data_r=0;

	if (!I2C_Start()) return 0; 
	I2C_SendByte(sla_address<<1);  
	if (!I2C_WaitAck())  
	{ 
		I2C_Stop();  
		return 0; 
	} 

	I2C_SendByte(sub_address);      
	I2C_WaitAck(); 
	I2C_Start(); 
	I2C_SendByte((sla_address<<1)+1); 
	I2C_WaitAck(); 
	data_r = I2C_ReceiveByte(); 
	I2C_NoAck(); 
	I2C_Stop(); 
	return data_r; 
} */
static  uint8_t i2c_len_Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!I2C_Start())
        return 0;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return 0;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte((addr << 1)+1);
    I2C_WaitAck();
    while (len) {
        *buf = I2C_ReceiveByte();
        if (len == 1)
            I2C_NoAck();
        else
            I2C_Ack();
        buf++;
        len--;
    }
    I2C_Stop();
    return 1;
}


static	void Mpu6881Initial(void)
{
//gyro init,16bit for (+-)2000der/s
	I2C_Write(MPU6050_ADDRESS, 0x6B, 0x80);			  //PWR_MGMT_1	  -- DEVICE_RESET 1
	DelayMs(200);
	I2C_Write(MPU6050_ADDRESS, 0x6B, 0x03);			  //PWR_MGMT_1	  -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
	I2C_Write(MPU6050_ADDRESS, 0x1A, 0x03);//低通42hz，300hz控制周期可以，也抖
	I2C_Write(MPU6050_ADDRESS, 0x1B, 0x18);
//	printf("Gyroscope sensor check out.\r\n");
//acc init,16bit for (+-)8g
	I2C_Write(MPU6050_ADDRESS, 0x1C, 0x10); 			//ACCEL_CONFIG	-- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0	//note something is wrong in the spec.
	I2C_Write(MPU6050_ADDRESS, 0x37, 0x00);
//	printf("Acceleration transducer sensor check out.\r\n");
	
//for stabilize
	DelayMs(500);
//	printf("All sensor check out.\r\n");
}
/*
static	int	FBM320ReadI2cValue(void)
{
	int	tmp=0;
	tmp|=I2C_Read(FBM320_ADDRESS,FBM320_DATA_MSB);
	tmp<<=8;
	tmp|=I2C_Read(FBM320_ADDRESS,FBM320_DATA_CSB);
	tmp<<=8;
	tmp|=I2C_Read(FBM320_ADDRESS,FBM320_DATA_LSB);

	return	tmp;
}*/
/*
static	void	FBM320Initial(void)
{
	uint32	i,l16_rx[10];
	DelayMs(1000);
	if(I2C_Read(FBM320_ADDRESS,FBM320_MYID) != FBM320_MYID_ITEM)
		{
		printf("FBM320 initial final.\r\n");
		return;
	}

	for(i=0;i<9;i++)
	{
		l16_rx[i]=0;
		l16_rx[i]|=I2C_Read(FBM320_ADDRESS,0xAA+2*i);
		l16_rx[i]<<=8;
		l16_rx[i]|=I2C_Read(FBM320_ADDRESS,0xAB+2*i);
	}
	l16_rx[9]=0;
	l16_rx[9]|=I2C_Read(FBM320_ADDRESS,0xD0);
	l16_rx[9]<<=8;
	l16_rx[9]|=I2C_Read(FBM320_ADDRESS,0xF1);

	gvst_ThisBird.Calc_coeff[0]= l16_rx[0] >> 4;
	gvst_ThisBird.Calc_coeff[1]= ((l16_rx[1] & 0xFF00) >> 5) | (l16_rx[2] & 7);
	gvst_ThisBird.Calc_coeff[2]= ((l16_rx[1] & 0xFF) << 1) | (l16_rx[4] & 1);
	gvst_ThisBird.Calc_coeff[3]= l16_rx[2] >> 3;
	gvst_ThisBird.Calc_coeff[4]= ((int)l16_rx[3] << 2) | (l16_rx[0] & 3);
	gvst_ThisBird.Calc_coeff[5]= l16_rx[4] >> 1;
	gvst_ThisBird.Calc_coeff[6]= l16_rx[5] >> 3;
	gvst_ThisBird.Calc_coeff[7]= ((int)l16_rx[6] << 3) | (l16_rx[5] & 7);
	gvst_ThisBird.Calc_coeff[8]= l16_rx[7] >> 3;
	gvst_ThisBird.Calc_coeff[9]= l16_rx[8] >> 2;
	gvst_ThisBird.Calc_coeff[10]= ((l16_rx[9] & 0xFF00) >> 6) | (l16_rx[8] & 3);
	gvst_ThisBird.Calc_coeff[11]= l16_rx[9] & 0xFF;
	gvst_ThisBird.Calc_coeff[12]= ((l16_rx[0] & 0x0C) << 1) | (l16_rx[7] & 7);

	DelayMs(10);
}*/

////////////////////////////////////分割线，以下为流程操作///////////////////////////////////

/*static	void	SystemClockSupplyInit(void)//系统时钟供应初始化
{
	
#if	MV_SYSTEM_CLOCK==36000000
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_9);
#elif	MV_SYSTEM_CLOCK==48000000
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);
#endif
	RCC_PLLCmd(ENABLE); 
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); 
	while(RCC_GetSYSCLKSource() != 0x08);
	
}*/

//static	void	SystemMemoryRemapInit(void)/*系统内存重映射初始化*/
//{;}

static	void	SystemPeripheralEnableInit(void)/*系统外设使能初始化*/
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);				//使能DMA时钟
}

static	void	SystemInterruputInit(void)/*系统中断初始化*/
{
	NVIC_InitTypeDef	NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
/*
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;		//选择DMA1通道中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//中断使能
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;				//优先级设为0
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                //选择TIM2中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                //使能TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;                //优先级为0
	NVIC_Init(&NVIC_InitStructure);*/
}

static	void	SystemGPIOInit(void)/*系统管脚初始化*/
{
	GPIO_InitTypeDef GPIO_InitStructure;
//motor
	//左上角电机1，编号B，对应PB8对应TIM16_CH1
	//左下角电机2，编号F，对应PA3对应TIM2_CH4
	//右下角电机3，编号R，对应PA8对应TIM1_CH1
	//右上角电机4，编号L，对应PA11对应TIM1_CH4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_8 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_2);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_2);  

//spi1->2.4g[XN297]
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//PA6->MISO
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_6);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7;//PA4,5,7->CSN,SCK, MOSI
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PB1->CE
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

//i2c1->gyro[MPU6050]
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
//ad1

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
//u1
/*
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//高速
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//开漏
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);

//led
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_5 ;//PB2对应LED1,PB5对应LED2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ;//PA15对应LED4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

//dv&photo
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 ;//PB3对应拍照,PB4对应录像
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//GPIOB->BRR	= GPIO_Pin_3;
	//GPIOB->BRR	= GPIO_Pin_4;
	gvst_ThisHal.SetIO(2,3,1);
	gvst_ThisHal.SetIO(2,4,1);

//free
}
static	void	ADC_Reset(void)
{
#if	1
	ADC_InitTypeDef	ADC_InitStructure;
	
	ADC_DeInit(ADC1);
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
	ADC_Init(ADC1, &ADC_InitStructure);

#else
	DMA_InitTypeDef dma_init_structure;
	ADC_InitTypeDef adc_init_structure;
	TIM_TimeBaseInitTypeDef timer_init_structure;
	
	ADC_DeInit(ADC1);												//复位ADC
	ADC_StructInit(&adc_init_structure);							//初始化ADC结构体
	adc_init_structure.ADC_ContinuousConvMode = DISABLE;			//禁用连续转换模式
	adc_init_structure.ADC_DataAlign = ADC_DataAlign_Right; 		//采样数据右对齐
	adc_init_structure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO; //外部触发设置为TIM2
	adc_init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;//上升沿触发
	adc_init_structure.ADC_Resolution = ADC_Resolution_12b; 		//12位分辨率
	adc_init_structure.ADC_ScanDirection = ADC_ScanDirection_Upward;//向上扫描0-18通道
	ADC_Init(ADC1, &adc_init_structure);

	ADC_OverrunModeCmd(ADC1, ENABLE);								//使能数据覆盖模式
	ADC_ChannelConfig(ADC1, ADC_Channel_0,ADC_SampleTime_239_5Cycles);
	//ADC_ChannelConfig(ADC1, ADC_Channel_0 | ADC_Channel_1 | ADC_Channel_2,ADC_SampleTime_239_5Cycles);//配置采样通道，采样时间125nS
	ADC_GetCalibrationFactor(ADC1); 								//使能前校准ADC
	ADC_Cmd(ADC1, ENABLE);											//使能ADC1
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN) == RESET); 		//等待ADC1使能完成

	ADC_DMACmd(ADC1, ENABLE);										//使能ADC_DMA
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);			//配置DMA请求模式为循环模式
	ADC_StartOfConversion(ADC1);									//开启一次转换（必须）

	DMA_DeInit(DMA1_Channel1);										//复位DMA1_channel1
	DMA_StructInit(&dma_init_structure);							//初始化DMA结构体
	dma_init_structure.DMA_BufferSize = 3;			//DMA缓存数组大小设置
	dma_init_structure.DMA_DIR = DMA_DIR_PeripheralSRC; 			//DMA方向：外设作为数据源
	dma_init_structure.DMA_M2M = DISABLE;							//内存到内存禁用
	dma_init_structure.DMA_MemoryBaseAddr = (uint32)&gvst_ThisBird.system_vol[0];//缓存数据数组起始地址
	dma_init_structure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//数据大小设置为word
	dma_init_structure.DMA_MemoryInc = DMA_MemoryInc_Enable;		//内存地址递增
	dma_init_structure.DMA_Mode = DMA_Mode_Circular;				//DMA循环模式，即完成后重新开始覆盖
	dma_init_structure.DMA_PeripheralBaseAddr = (uint32) &(ADC1->DR);//取值的外设地址
	dma_init_structure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设取值大小设置为Halfword
	dma_init_structure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址递增禁用
	dma_init_structure.DMA_Priority = DMA_Priority_High;			 //DMA优先级设置为高
	DMA_Init(DMA1_Channel1, &dma_init_structure);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); 				 //使能DMA中断
	DMA_ClearITPendingBit(DMA_IT_TC);								 //清除一次DMA中断标志
	DMA_Cmd(DMA1_Channel1, ENABLE); 								 //使能DMA1

	TIM_DeInit(TIM2);                                               //复位TIM2
	TIM_TimeBaseStructInit(&timer_init_structure);                  //初始化TIMBASE结构体

	timer_init_structure.TIM_ClockDivision = TIM_CKD_DIV1;          //系统时钟，不分频，36M
	timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;      //向上计数模式
	timer_init_structure.TIM_Period = 10;                          //每312 uS触发一次中断，开启ADC
	timer_init_structure.TIM_Prescaler = 36-1;                      //计数时钟预分频，f＝1M，systick＝1 uS
	timer_init_structure.TIM_RepetitionCounter = 0x00;              //发生0+1次update事件产生中断
	TIM_TimeBaseInit(TIM2, &timer_init_structure);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);                      //使能TIM2中断
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);           //选择TIM2的update事件更新为触发源

	TIM_Cmd(TIM2, ENABLE);                                          //使能TIM2
#endif
}

static	void	SystemMsicPeripheralInit(void)/*系统外设初始化*/
{
	
	USART_InitTypeDef USART_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	
//SYSTICK
	SysTick_Config(MV_SYSTEM_CLOCK/1000);

//ADC 
	ADC_Reset();

//SPI1_default

//I2C1_default

//UASRT1
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);

//TIM1_TIM2_TIM16
	TIM_TimeBaseStructure.TIM_Period = 1000;//计数1000
	TIM_TimeBaseStructure.TIM_Prescaler = 2;//48/n
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	//TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);//CH4可以正确输出，但CH1不行，全速转动，为什么？
	//TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);	
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);	

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	//   TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);	
	TIM_Cmd(TIM2, ENABLE);

	TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OC1Init(TIM16, &TIM_OCInitStructure);
	//   TIM_OC1PreloadConfig(TIM16, TIM_OCPreload_Enable);	
	TIM_Cmd(TIM16, ENABLE);
	TIM_CtrlPWMOutputs(TIM16, ENABLE);

	TIM16->CCR1=1000-0;//PB8
	TIM2->CCR4=0;//PA3
	TIM1->CCR1=1000-0;//PA8
	TIM1->CCR4=0;//PA11
	/*
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
  TIM_TimeBaseStructure.TIM_Period = 19200;//400us
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_ClearFlag(TIM3, TIM_FLAG_Update);
//    TIM IT enable 
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//TIM3用于主循环定时器，也可以用滴答定时器
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	TIM_Cmd(TIM3, ENABLE);*/

}
static	void Iwdg_Init(void)
{
#if	OPENIWDG==1
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_64);
	IWDG_SetReload(1999);
	IWDG_ReloadCounter();
	IWDG_Enable();
#endif
}

static	void Iwdg_Feed()
{
#if	OPENIWDG==1
	IWDG_ReloadCounter();
#endif
}

static	void	SystemRestart(void)/*系统重启*/
{NVIC_SystemReset();}
/*
static	void	SystemVersionPrint(void)//系统打印信息
{
	printf("%s\r\n%s %s.\r\n",gcu8_ThisVersion,__DATE__,__TIME__);
}*/

//static	int 	SystemWriteDisk(int* address,void* dat)/*系统存储信息*/
//{

//	return	0;
//}

//static	int 	SystemReadDisk(int* address,void* dat)/*系统读取信息*/
//{
//
//	return	0;
//}


static	uint32	ReadADValue(uint32	ch)
{
	ADC_Reset();
	ADC_ChannelConfig(ADC1, 1<<ch, ADC_SampleTime_239_5Cycles);
	ADC_GetCalibrationFactor(ADC1);
	ADC_Cmd(ADC1, ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));
	ADC_StartOfConversion(ADC1);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return	(uint32)ADC_GetConversionValue(ADC1);
}

static	void	SetIO(int gr,int ch,int va)/*设置IO数值*/
{
	GPIO_TypeDef *	_gr;
	if(1==gr)	_gr=GPIOA;
	else if(2==gr)	_gr=GPIOB;
	/*
	else if(3==gr)	_gr=GPIOC;
	else if(4==gr)	_gr=GPIOD;
	else if(5==gr)	_gr=GPIOE;
	else if(6==gr)	_gr=GPIOF;
	*/
	else	return;

	ch=1<<ch;

	if(va)	_gr->BSRR=ch;
	else	_gr->BRR=ch;
}

static	int		ReadIO(int gr,int ch)/*读取IO数值*/
{
	GPIO_TypeDef *	_gr;
	if(1==gr)	_gr=GPIOA;
	else if(2==gr)	_gr=GPIOB;
	/*
	else if(3==gr)	_gr=GPIOC;
	else if(4==gr)	_gr=GPIOD;
	else if(5==gr)	_gr=GPIOE;
	else if(6==gr)	_gr=GPIOF;
	*/
	else	return	MV_ERR_READ_INV_PIN;

	ch=1<<ch;
	return	GPIO_ReadInputDataBit(_gr ,ch);
}

static	void	AircraftInit(void)/*飞行功能模块初始化*/
{
//	int	i;
//motor init

//	printf("Syetem checkout motor,%d motors are ready.\r\n",MV_HOWMANY_MOTOR);

//2.4g
	XN297Initial();
//	printf("Syetem checkout telecontroller module,the def-TX_ADDRESS is [%x,%x,%x,%x,%x].\r\n",TX_ADDRESS_DEF[0],TX_ADDRESS_DEF[1],TX_ADDRESS_DEF[2],TX_ADDRESS_DEF[3],TX_ADDRESS_DEF[4]);

//gyro & acc
	Mpu6881Initial();

//mag

//baro
	//FBM320Initial();
//gps

//flow

}

static	int	ReadTel(void* dat)/*读取遥控器数值*/
{
	uint8	sta;
	if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DOCK))
	{
		CheckoutTel();
		return	FALSE;
	}
	else if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DROP))
	{
		CheckoutTel();
		return	FALSE;
	}
	
	if(dat==NULL)
		return	FALSE;
	
	CE_LOW;
	sta=ucRF_ReadReg(STATUS);	// read register STATUS's value
	if(sta&STATUS_RX_DR)				// if receive data ready (RX_DR) interrupt
	{
		rf_lost=0;
		rf_checkerrcnt=0;
		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK);
		CE_LOW;
		RF_ReadBuf(R_RX_PAYLOAD,(uint8*)dat,9);// read receive payload from RX_FIFO buffer
		RF_WriteReg(W_REGISTER + STATUS, 0x70); 									//清除Status
		RF_WriteReg(FLUSH_RX,0);//flush Rx
		CE_HIGH;

		if(TelDatCheck(dat,9)==TRUE)
			return	TRUE;
	}
	else//不加这段代码的话，在电机启动后容易通信失败
	{
		CE_LOW;
		RF_WriteReg(W_REGISTER + STATUS, 0x70); 									//清除Status
		RF_WriteReg(FLUSH_RX,0);//flush Rx
		CE_HIGH;

		if(rf_lost++>=2)   //once 20ms  原5
		{
			rf_lost=0;
			rf_index++;
			rf_index=rf_index%6;/*
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10))    //测试模式跳频
				RF_SetChannel(rf_channel[rf_index]);//跳频
			else
			{
				RF_SetChannel(0x3c);
			}*/
		}
	}

	if(rf_checkerrcnt++>MV_TEL_DROP_DELAY)
	{
		rf_checkerrcnt=0;
		MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK);    //遥控器掉线
		//XN297Initial();
//		printf("The brid has disconnect from telecontroller.\r\n");
//		printf("Syetem reset telecontroller module,the def-TX_ADDRESS is [%x,%x,%x,%x,%x].\r\n",TX_ADDRESS_DEF[0],TX_ADDRESS_DEF[1],TX_ADDRESS_DEF[2],TX_ADDRESS_DEF[3],TX_ADDRESS_DEF[4]);
	}
	return	FALSE;
}/*
void	ReadGyroandAcc(void)
{
	i2c_len_Read(MPU6050_ADDRESS,0x3B,12,mpu6050_buf);
  gvst_ThisBird.GyroDatNow.pitch=(int16)(mpu6050_buf[0]<<8+mpu6050_buf[1]);
	gvst_ThisBird.GyroDatNow.roll=(int16)(mpu6050_buf[2]<<8+mpu6050_buf[3]);
	gvst_ThisBird.GyroDatNow.yaw=(int16)(mpu6050_buf[4]<<8+mpu6050_buf[5]);
	
	tmp_acc.pitch=(int16)(mpu6050_buf[6]<<8+mpu6050_buf[7]);
	tmp_acc.roll=(int16)(mpu6050_buf[8]<<8+mpu6050_buf[9]);
	tmp_acc.yaw=(int16)(mpu6050_buf[10]<<8+mpu6050_buf[11]);
}*/

static	void	ReadAcc(void* dat)/*读取加速度计数值*/
{
	uint8_t mpu6050_acc[6];
	__AccDat*	__dat;
	if(dat==NULL)	return	;

	__dat=(__AccDat*)dat;
	i2c_len_Read(MPU6050_ADDRESS,0x3B,6,mpu6050_acc);
	__dat->pitch=(int16)((mpu6050_acc[0]<<8)+mpu6050_acc[1]);//acc_x
	__dat->roll=(int16)((mpu6050_acc[2]<<8)+mpu6050_acc[3]);//acc_y
	__dat->yaw=(int16)((mpu6050_acc[4]<<8)+mpu6050_acc[5]);//acc_z
}

static	void	ReadGyro(void* dat)//读取陀螺仪数值
{
	uint8_t mpu6050_gyro[6];
	__GyroDat*	__dat;
	if(dat==NULL)	return	;

	__dat=(__GyroDat*)dat;
	i2c_len_Read(MPU6050_ADDRESS,0x43,6,mpu6050_gyro);
	__dat->pitch=(int16)((mpu6050_gyro[0]<<8)+mpu6050_gyro[1]);
	__dat->roll=(int16)((mpu6050_gyro[2]<<8)+mpu6050_gyro[3]);
	__dat->yaw=(int16)((mpu6050_gyro[4]<<8)+mpu6050_gyro[5]);
}
/*
static	void	ReadBaro(void)//读取气压计数值
{	
	int RP=0;
	static	int int_temp,int_press;
	static	int	readbarocnt=0;
	int DT, DT2, X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26, X31, X32, CF, PP1, PP2, PP3, PP4;
	
	if(readbarocnt==0)
		{//read temp
		I2C_Write(FBM320_ADDRESS,FBM320_DATA_CFG,0x2E);//for 2.2ms
	}
	else if(readbarocnt==2)
		{
		int_temp=FBM320ReadI2cValue();
		I2C_Write(FBM320_ADDRESS,FBM320_DATA_CFG,FBM320_CFG_DEF);//osr 8192 for 9.8ms
	}
	else if(readbarocnt==6)
		{
		int_press=FBM320ReadI2cValue();
		I2C_Write(FBM320_ADDRESS,FBM320_DATA_CFG,0x2E);//for 2.2ms
		
		//real temp
		DT	=	((int_temp - 8388608) >> 4) + (gvst_ThisBird.Calc_coeff[0] << 4);
		X01	=	(gvst_ThisBird.Calc_coeff[1] + 4459) * DT >> 1;
		X02	=	((((gvst_ThisBird.Calc_coeff[2] - 256) * DT) >> 14) * DT) >> 4;
		X03	=	(((((gvst_ThisBird.Calc_coeff[3] * DT) >> 18) * DT) >> 18) * DT);
		//RT	=	((2500 << 15) - X01 - X02 - X03) >> 15;

		//real pressure
		DT2	=	(X01 + X02 + X03) >> 12;
		X11	=	((gvst_ThisBird.Calc_coeff[5] - 4443) * DT2);
		X12	=	(((gvst_ThisBird.Calc_coeff[6] * DT2) >> 16) * DT2) >> 2;
		X13	=	((X11 + X12) >> 10) + ((gvst_ThisBird.Calc_coeff[4] + 120586) << 4);
		X21	=	((gvst_ThisBird.Calc_coeff[8] + 7180) * DT2) >> 10;
		X22	=	(((gvst_ThisBird.Calc_coeff[9] * DT2) >> 17) * DT2) >> 12;
		if(X22 >= X21)
			X23	=	X22 - X21;
		else
			X23	=	X21 - X22;
		X24	=	(X23 >> 11) * (gvst_ThisBird.Calc_coeff[7] + 166426);
		X25	=	((X23 & 0x7FF) * (gvst_ThisBird.Calc_coeff[7] + 166426)) >> 11;
		if((X22 - X21) < 0)
			X26	=	((0 - X24 - X25) >> 11) + gvst_ThisBird.Calc_coeff[7] + 166426;
		else	
			X26	=	((X24 + X25) >> 11) + gvst_ThisBird.Calc_coeff[7] + 166426;
					
		PP1	=	((int_press - 8388608) - X13) >> 3;
		PP2	=	(X26 >> 11) * PP1;
		PP3	=	((X26 & 0x7FF) * PP1) >> 11;
		PP4	=	(PP2 + PP3) >> 10;
		CF	=	(2097152 + gvst_ThisBird.Calc_coeff[12] * DT2) >> 3;
		X31	=	(((CF * gvst_ThisBird.Calc_coeff[10]) >> 17) * PP4) >> 2;
		X32	=	(((((CF * gvst_ThisBird.Calc_coeff[11]) >> 15) * PP4) >> 18) * PP4);
		RP	=	((X31 + X32) >> 15) + PP4 + 99880;


		//Fylib_LpFilter(0.980f,(__FP_ACC)RP,(float*)(&gvst_ThisBird.barodat.pressure));
		//gvst_ThisBird.barodat.pressure=(__FP_ACC)RP;
		//gvst_ThisBird.barodat.temp=(__FP_ACC)RT/100.0f;
		//H=(RT/gM)*ln(p0/p)
		//lvfpacc_heightadd=(8.510f*(gvst_ThisBird.barodat.temp+273.15f)/286.8584166f)*log(101325.0f/gvst_ThisBird.barodat.pressure);
		//lvfpacc_heightadd*=100000.0f;//km->cm
		//lvfpacc_heightadd+=2000;
		lvfpacc_heightadd=(__FP_ACC)RP;
		Fylib_LpFilter(0.8f,lvfpacc_heightadd,(float*)(&gvst_ThisBird.barodat.pressure));//0.875-0.90
		gvfpacc_CurHeight=(MV_STAND_PRESS-gvst_ThisBird.barodat.pressure)*10.0f;
		//SendDatShow((int)gvfpacc_CurHeight);
		
		readbarocnt=0;
	}
	readbarocnt++;
}
*/
//static	void	ReadGps(void* dat)/*读取GPS数值*/
//{}
uint32	SystemGetMirco(void)
{//返回当前微秒数，即1/1000000s
#if	MV_SYSTEM_CLOCK==36000000
	return	gvu32_SystemRunTime*1000+(SysTick->LOAD - SysTick->VAL)/36;
#elif	MV_SYSTEM_CLOCK==48000000
	return	gvu32_SystemRunTime*1000+(SysTick->LOAD - SysTick->VAL)/48;
#endif
}

void	SystemHalInit(__ThisHal *hal)
{
	SystemInit();
//	SystemClockSupplyInit();
//	hal->SystemClockSupplyInit=SystemClockSupplyInit;
//	hal->SystemMemoryRemapInit=SystemMemoryRemapInit;
	hal->SystemPeripheralEnableInit=SystemPeripheralEnableInit;
	hal->SystemInterruputInit=SystemInterruputInit;
	hal->SystemGPIOInit=SystemGPIOInit;
	hal->SystemMsicPeripheralInit=SystemMsicPeripheralInit;
	//hal->SystemVersionPrint=SystemVersionPrint;
	hal->Iwdg_Init=Iwdg_Init;
	hal->Iwdg_Feed=Iwdg_Feed;
	hal->SystemRestart=SystemRestart;
//	hal->SystemWriteDisk=STMFLASH_Write;
//	hal->SystemReadDisk=STMFLASH_Read;
	hal->AircraftInit=AircraftInit;
	hal->ReadADValue=ReadADValue;
//	hal->SystemUpdateMotorPower=SystemUpdateMotorPower;
	hal->SetIO=SetIO;
	hal->ReadIO=ReadIO;
	hal->ReadTel=ReadTel;
	hal->ReadAcc=ReadAcc;
	hal->ReadGyro=ReadGyro;
//	hal->ReadMag=ReadMag;
	//hal->ReadBaro=ReadBaro;
//	hal->ReadGps=ReadGps;
	hal->SystemGetMirco=SystemGetMirco;
}


