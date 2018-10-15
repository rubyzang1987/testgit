#include	"config.h"
#include	"msic_device.h"

#define	OPENIWDG	0
static	volatile	__FP_ACC	lvfpacc_firstpress=0.0f,lvfpacc_heightadd=0.0f;//�۴θ߶�ֵ
static	volatile	int	lvfpacc_getfirstpresscnt=0;


///////////////////////////////////�������ܲ���///////////////////////////////////////////////////
//xn297,2.4gң��
const unsigned char TEL_ADDRESS_DEF[2][5] = {{0xd1,0x2c,0x3,0x4,0xc1},{0xdb,0x2c,0x0,0x2,0x56}};
const unsigned char TX_ADDRESS_DEF[5] = {0xCC,0xCC,0xCC,0xCC,0xCC};             //RF ��ַ�����ն˺ͷ��Ͷ���һ��
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
		pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//��ȡ2���ֽ�.
		ReadAddr+=2;//ƫ��2���ֽ�.	
	}
}
static	void STMFLASH_Write_NoCheck(uint32 WriteAddr,uint16 *pBuffer,uint32 NumToWrite)   
{ 			 		 
	uint16 i;
	for(i=0;i<NumToWrite;i++)
	{
		FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
	    WriteAddr+=2;//��ַ����2.
	}  
} 
static	void STMFLASH_Write(uint32 WriteAddr,uint16 *pBuffer,uint32 NumToWrite)	
{ 
	FLASH_Unlock();
	FLASH_ErasePage(WriteAddr);
	STMFLASH_Write_NoCheck(WriteAddr,pBuffer,NumToWrite);
	FLASH_Lock();//����
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
	RF_WriteReg(W_REGISTER + CONFIG, 0x0f);                                     // ��RF���ó�RXģʽ
	CE_HIGH;                                                                    // Set CE pin high ��ʼ��������
}
#if	0//UNUSE
static	unsigned char RF_GetStatus(void)
{
	return ucRF_ReadReg(STATUS)&0x70;                                           //��ȡRF��״̬ 
}
static	void RF_ClearStatus(void)
{
	RF_WriteReg(W_REGISTER + STATUS,0x70);	                                //���RF��IRQ��־ 
}
static	void RF_ClearFIFO(void)
{
	RF_WriteReg(FLUSH_TX, 0);			                                //���RF �� TX FIFO		
	RF_WriteReg(FLUSH_RX, 0);                                                   //���RF �� RX FIFO	
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
	unsigned char BB_cal_data[]    = {0x53,0xA4,0x67,0x9C,0x20};                //���������޸Ĵ˲��� @ 2Mbps
	unsigned char RF_cal_data[]    = {0xDA,0x9A,0xB0,0x61,0xBB,0xAB,0x9C};      //���������޸Ĵ˲��� @ 2Mbps
	unsigned char Dem_cal_data[]   = {0x0B,0xDF,0xC4,0xA7,0x03};                //���������޸Ĵ˲��� @ 2Mbps    

	CE_LOW;
	CSN_HIGH;
	SCK_LOW;
	DelayMs(10);                                                               //��ʱ10ms
	ucCurrent_Channel = DEFAULT_CHANNEL;

#if(TRANSMIT_TYPE == TRANS_ENHANCE_MODE)                                        //��ǿ��ģʽ������ACK
/*********************use xn297 Enhance*********************/
	RF_WriteReg(FLUSH_TX, 0);		    			
	RF_WriteReg(FLUSH_RX, 0);
	RF_WriteReg(W_REGISTER + STATUS, 0x70);	
	RF_WriteBuf(W_REGISTER + BB_CAL,  BB_cal_data,  5);
	RF_WriteBuf(W_REGISTER + RF_CAL,  RF_cal_data,  7);
	RF_WriteBuf(W_REGISTER + DEM_CAL, Dem_cal_data, 5);

	RF_WriteBuf(W_REGISTER + TX_ADDR,   (unsigned char*)TX_ADDRESS_DEF, 5);     // д�뷢�͵ĵ�ַ
	RF_WriteBuf(W_REGISTER + RX_ADDR_P0,(unsigned char*)TX_ADDRESS_DEF, 5); 	// д��RX PIPO 0 �ĵ�ַ

	RF_WriteReg(W_REGISTER + EN_AA,     0x01);     	 			// ʹ�� Auto.Ack:Pipe0
	RF_WriteReg(W_REGISTER + EN_RXADDR, 0x01);  			        // ʹ�� Pipe0 RX
	RF_WriteReg(W_REGISTER + SETUP_AW,  0x03);					// ��ַ��ȣ�5 bytes
	RF_WriteReg(W_REGISTER + RF_CH,     DEFAULT_CHANNEL);                       //д�빤��Ƶ��
	RF_WriteReg(W_REGISTER + SETUP_RETR,RETRY_MAX); 				// �趨Ϊ5�η��ͣ������4���ش�	
	RF_WriteReg(W_REGISTER + RX_PW_P0,  PAYLOAD_WIDTH);				// payload length
	RF_WriteReg(W_REGISTER + RF_SETUP,  RF_POWER);   				// TX_PWR:8dBm, Data rate:2Mbps, LNA:HCURR 

	RF_WriteReg(ACTIVATE, 0x73);
	RF_WriteReg(W_REGISTER + DYNPD, 0x01);	
	if(PAYLOAD_WIDTH <33)
		RF_WriteReg(W_REGISTER +FEATURE, 0x04);                                 //�л���32byteģʽ
	else
		RF_WriteReg(W_REGISTER +FEATURE, 0x1C);                                 //�л���64byteģʽ     
	RF_WriteReg(ACTIVATE, 0x73); 

#elif(TRANSMIT_TYPE == TRANS_BURST_MODE)                                        //��ͨģʽ,��ACK
/*********************use xn297 normal*********************/
	RF_WriteReg(FLUSH_TX, 0);					
	RF_WriteReg(FLUSH_RX, 0);
	RF_WriteReg(W_REGISTER + STATUS, 0x70);	
	RF_WriteBuf(W_REGISTER + BB_CAL,  BB_cal_data,  5);
	RF_WriteBuf(W_REGISTER + RF_CAL,  RF_cal_data,  7);
	RF_WriteBuf(W_REGISTER + DEM_CAL, Dem_cal_data, 5);

	RF_WriteBuf(W_REGISTER + TX_ADDR,   (unsigned char*)TX_ADDRESS_DEF, 5);     // д�뷢�͵ĵ�ַ
	RF_WriteBuf(W_REGISTER + RX_ADDR_P0,(unsigned char*)TX_ADDRESS_DEF, 5); 	// д��RX PIPO 0 �ĵ�ַ

	RF_WriteReg(W_REGISTER + EN_AA,     0x00);     	 		        // Disable Auto.ACK
	RF_WriteReg(W_REGISTER + EN_RXADDR, 0x01);  			        // Enable Pipe0
	RF_WriteReg(W_REGISTER + SETUP_AW,  0x03);					// ��ַ��ȣ� 5 bytes
	RF_WriteReg(W_REGISTER + RF_CH,     DEFAULT_CHANNEL);                       // д�빤��Ƶ��
	RF_WriteReg(W_REGISTER + SETUP_RETR,0x00);                                  // ʹ����ͨ���� 	
	RF_WriteReg(W_REGISTER + RX_PW_P0,  PAYLOAD_WIDTH);				// �趨Payload����
	RF_WriteReg(W_REGISTER + RF_SETUP,  RF_POWER);   				// TX_PWR:8dBm, Datarate:2Mbps, LNA:HCURR 

	RF_WriteReg(ACTIVATE, 0x73);
	RF_WriteReg(W_REGISTER + DYNPD, 0x00);					
	if(PAYLOAD_WIDTH <33)
		RF_WriteReg(W_REGISTER +FEATURE, 0x00);                                 //�л���32byteģʽ 
	
	else
		RF_WriteReg(W_REGISTER +FEATURE, 0x18);                                 //�л���64byteģʽ

#endif

	RF_RxMode();
	DelayMs(10);
}

static	void	CheckoutTel(void)
{//����
	uint8	tmp_buf[10];
	//�ѳɹ����뼴����
	if(ucRF_ReadReg(STATUS)!=STATUS_RX_DR)
		return;

	CE_LOW;
	RF_ReadBuf(R_RX_PAYLOAD,tmp_buf,9);// read receive payload from RX_FIFO buffer
	RF_WriteReg(W_REGISTER + STATUS, 0x70); 									//���Status
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
	//DelayMs(10);															  //��ʱ10ms
	RF_WriteBuf(W_REGISTER + TX_ADDR,	 (unsigned char*)gvu8_TelTxAddr, 5);	 // д�뷢�͵ĵ�ַ
	RF_WriteBuf(W_REGISTER + RX_ADDR_P0,(unsigned char*)gvu8_TelTxAddr, 5);   // д��RX PIPO 0 �ĵ�ַ
	RF_RxMode(); //��䲻��ʡ�ԣ������޷��յ�����

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
	if(!SDA_read)return 0; //SDA��Ϊ�͵�ƽ������æ,�˳� 
	SDA_L; 
	I2C_delay(); 
	if(SDA_read) return 0; //SDA��Ϊ�ߵ�ƽ�����߳���,�˳� 
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
static	uint8	I2C_WaitAck(void)   //����Ϊ:=1��ACK,=0��ACK 
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
static	void	I2C_SendByte(uint8 SendByte) //���ݴӸ�λ����λ// 
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
static	uint8	I2C_ReceiveByte(void)  //���ݴӸ�λ����λ// 
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
	I2C_Write(MPU6050_ADDRESS, 0x1A, 0x03);//��ͨ42hz��300hz�������ڿ��ԣ�Ҳ��
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

////////////////////////////////////�ָ��ߣ�����Ϊ���̲���///////////////////////////////////

/*static	void	SystemClockSupplyInit(void)//ϵͳʱ�ӹ�Ӧ��ʼ��
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

//static	void	SystemMemoryRemapInit(void)/*ϵͳ�ڴ���ӳ���ʼ��*/
//{;}

static	void	SystemPeripheralEnableInit(void)/*ϵͳ����ʹ�ܳ�ʼ��*/
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);				//ʹ��DMAʱ��
}

static	void	SystemInterruputInit(void)/*ϵͳ�жϳ�ʼ��*/
{
	NVIC_InitTypeDef	NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
/*
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;		//ѡ��DMA1ͨ���ж�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//�ж�ʹ��
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;				//���ȼ���Ϊ0
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                //ѡ��TIM2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                //ʹ��TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;                //���ȼ�Ϊ0
	NVIC_Init(&NVIC_InitStructure);*/
}

static	void	SystemGPIOInit(void)/*ϵͳ�ܽų�ʼ��*/
{
	GPIO_InitTypeDef GPIO_InitStructure;
//motor
	//���Ͻǵ��1�����B����ӦPB8��ӦTIM16_CH1
	//���½ǵ��2�����F����ӦPA3��ӦTIM2_CH4
	//���½ǵ��3�����R����ӦPA8��ӦTIM1_CH1
	//���Ͻǵ��4�����L����ӦPA11��ӦTIM1_CH4
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
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//����
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//����
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//��©
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);

//led
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_5 ;//PB2��ӦLED1,PB5��ӦLED2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ;//PA15��ӦLED4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

//dv&photo
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 ;//PB3��Ӧ����,PB4��Ӧ¼��
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
	
	ADC_DeInit(ADC1);												//��λADC
	ADC_StructInit(&adc_init_structure);							//��ʼ��ADC�ṹ��
	adc_init_structure.ADC_ContinuousConvMode = DISABLE;			//��������ת��ģʽ
	adc_init_structure.ADC_DataAlign = ADC_DataAlign_Right; 		//���������Ҷ���
	adc_init_structure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO; //�ⲿ��������ΪTIM2
	adc_init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;//�����ش���
	adc_init_structure.ADC_Resolution = ADC_Resolution_12b; 		//12λ�ֱ���
	adc_init_structure.ADC_ScanDirection = ADC_ScanDirection_Upward;//����ɨ��0-18ͨ��
	ADC_Init(ADC1, &adc_init_structure);

	ADC_OverrunModeCmd(ADC1, ENABLE);								//ʹ�����ݸ���ģʽ
	ADC_ChannelConfig(ADC1, ADC_Channel_0,ADC_SampleTime_239_5Cycles);
	//ADC_ChannelConfig(ADC1, ADC_Channel_0 | ADC_Channel_1 | ADC_Channel_2,ADC_SampleTime_239_5Cycles);//���ò���ͨ��������ʱ��125nS
	ADC_GetCalibrationFactor(ADC1); 								//ʹ��ǰУ׼ADC
	ADC_Cmd(ADC1, ENABLE);											//ʹ��ADC1
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN) == RESET); 		//�ȴ�ADC1ʹ�����

	ADC_DMACmd(ADC1, ENABLE);										//ʹ��ADC_DMA
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);			//����DMA����ģʽΪѭ��ģʽ
	ADC_StartOfConversion(ADC1);									//����һ��ת�������룩

	DMA_DeInit(DMA1_Channel1);										//��λDMA1_channel1
	DMA_StructInit(&dma_init_structure);							//��ʼ��DMA�ṹ��
	dma_init_structure.DMA_BufferSize = 3;			//DMA���������С����
	dma_init_structure.DMA_DIR = DMA_DIR_PeripheralSRC; 			//DMA����������Ϊ����Դ
	dma_init_structure.DMA_M2M = DISABLE;							//�ڴ浽�ڴ����
	dma_init_structure.DMA_MemoryBaseAddr = (uint32)&gvst_ThisBird.system_vol[0];//��������������ʼ��ַ
	dma_init_structure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//���ݴ�С����Ϊword
	dma_init_structure.DMA_MemoryInc = DMA_MemoryInc_Enable;		//�ڴ��ַ����
	dma_init_structure.DMA_Mode = DMA_Mode_Circular;				//DMAѭ��ģʽ������ɺ����¿�ʼ����
	dma_init_structure.DMA_PeripheralBaseAddr = (uint32) &(ADC1->DR);//ȡֵ�������ַ
	dma_init_structure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//����ȡֵ��С����ΪHalfword
	dma_init_structure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ��������
	dma_init_structure.DMA_Priority = DMA_Priority_High;			 //DMA���ȼ�����Ϊ��
	DMA_Init(DMA1_Channel1, &dma_init_structure);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); 				 //ʹ��DMA�ж�
	DMA_ClearITPendingBit(DMA_IT_TC);								 //���һ��DMA�жϱ�־
	DMA_Cmd(DMA1_Channel1, ENABLE); 								 //ʹ��DMA1

	TIM_DeInit(TIM2);                                               //��λTIM2
	TIM_TimeBaseStructInit(&timer_init_structure);                  //��ʼ��TIMBASE�ṹ��

	timer_init_structure.TIM_ClockDivision = TIM_CKD_DIV1;          //ϵͳʱ�ӣ�����Ƶ��36M
	timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;      //���ϼ���ģʽ
	timer_init_structure.TIM_Period = 10;                          //ÿ312 uS����һ���жϣ�����ADC
	timer_init_structure.TIM_Prescaler = 36-1;                      //����ʱ��Ԥ��Ƶ��f��1M��systick��1 uS
	timer_init_structure.TIM_RepetitionCounter = 0x00;              //����0+1��update�¼������ж�
	TIM_TimeBaseInit(TIM2, &timer_init_structure);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);                      //ʹ��TIM2�ж�
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);           //ѡ��TIM2��update�¼�����Ϊ����Դ

	TIM_Cmd(TIM2, ENABLE);                                          //ʹ��TIM2
#endif
}

static	void	SystemMsicPeripheralInit(void)/*ϵͳ�����ʼ��*/
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
	TIM_TimeBaseStructure.TIM_Period = 1000;//����1000
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
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);//CH4������ȷ�������CH1���У�ȫ��ת����Ϊʲô��
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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
  TIM_TimeBaseStructure.TIM_Period = 19200;//400us
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_ClearFlag(TIM3, TIM_FLAG_Update);
//    TIM IT enable 
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//TIM3������ѭ����ʱ����Ҳ�����õδ�ʱ��
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
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

static	void	SystemRestart(void)/*ϵͳ����*/
{NVIC_SystemReset();}
/*
static	void	SystemVersionPrint(void)//ϵͳ��ӡ��Ϣ
{
	printf("%s\r\n%s %s.\r\n",gcu8_ThisVersion,__DATE__,__TIME__);
}*/

//static	int 	SystemWriteDisk(int* address,void* dat)/*ϵͳ�洢��Ϣ*/
//{

//	return	0;
//}

//static	int 	SystemReadDisk(int* address,void* dat)/*ϵͳ��ȡ��Ϣ*/
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

static	void	SetIO(int gr,int ch,int va)/*����IO��ֵ*/
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

static	int		ReadIO(int gr,int ch)/*��ȡIO��ֵ*/
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

static	void	AircraftInit(void)/*���й���ģ���ʼ��*/
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

static	int	ReadTel(void* dat)/*��ȡң������ֵ*/
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
		RF_WriteReg(W_REGISTER + STATUS, 0x70); 									//���Status
		RF_WriteReg(FLUSH_RX,0);//flush Rx
		CE_HIGH;

		if(TelDatCheck(dat,9)==TRUE)
			return	TRUE;
	}
	else//������δ���Ļ����ڵ������������ͨ��ʧ��
	{
		CE_LOW;
		RF_WriteReg(W_REGISTER + STATUS, 0x70); 									//���Status
		RF_WriteReg(FLUSH_RX,0);//flush Rx
		CE_HIGH;

		if(rf_lost++>=2)   //once 20ms  ԭ5
		{
			rf_lost=0;
			rf_index++;
			rf_index=rf_index%6;/*
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10))    //����ģʽ��Ƶ
				RF_SetChannel(rf_channel[rf_index]);//��Ƶ
			else
			{
				RF_SetChannel(0x3c);
			}*/
		}
	}

	if(rf_checkerrcnt++>MV_TEL_DROP_DELAY)
	{
		rf_checkerrcnt=0;
		MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK);    //ң��������
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

static	void	ReadAcc(void* dat)/*��ȡ���ٶȼ���ֵ*/
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

static	void	ReadGyro(void* dat)//��ȡ��������ֵ
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
static	void	ReadBaro(void)//��ȡ��ѹ����ֵ
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
//static	void	ReadGps(void* dat)/*��ȡGPS��ֵ*/
//{}
uint32	SystemGetMirco(void)
{//���ص�ǰ΢��������1/1000000s
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


