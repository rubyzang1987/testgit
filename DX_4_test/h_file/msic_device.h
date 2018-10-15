#ifndef	__MSIC_DEVICE_H__
#define	__MSIC_DEVICE_H__

#include	"config.h"


//about xn297
/*
RF_ReadBuf(R_RX_PAYLOAD,(uint8*)dat,9);
dat[0]->油门，大小0-0xff
dat[1]->yaw，大小0-0xff，0x80为居中，0x80->0为依次逆时针旋转，0x80->0xff为依次顺时针旋转
dat[2]->yaw微调，目前保留，恒为中间值0x40
dat[3]->ptich，大小0-0xff，0x80为居中，0x80->0为依次下倾俯，0x80->0xff为依次上倾仰
dat[4]->roll，大小0-0xff，0x80为居中，0x80->0为依次左翻滚，0x80->0xff为依次右翻滚
dat[5]->ptich微调，中指为0x40，F增加，S减小
dat[6]->roll微调，中指为0x40，R增加，L减小
dat[7]->四通道控制时，正常为0，按住speed/stunt[翻转]为0x0f，按3ch[拍照]为0x40，按4ch[摄像]为0x20，且一直输出0x20，直到再次按下4ch[摄像]停止摄像，时恢复输出0
三通道时，正常为0x80，按住speed/stunt[翻转]为0x8f，按3ch[拍照]为0xc0，按4ch[摄像]为0xa0，且一直输出0xa0，直到再次按下4ch[摄像]停止摄像，时恢复输出0x80
dat[8]->校验值，为dat[0-8]累加后取反，最后一个字节即校验值。
遥控器上下左右可微调中间状态初始值

*/
/*
	#define 		CE_HIGH 		 GPIO->PB_DATABSR= 0x00000001
	#define 		CE_LOW			 GPIOB->BRR	= GPIO_Pin_0

#define         CSN_HIGH         GPIOA->BSRR= GPIO_Pin_4
#define         CSN_LOW          GPIOA->BRR = GPIO_Pin_4
#define         SCK_HIGH         GPIOA->BSRR= GPIO_Pin_5
#define         SCK_LOW          GPIOA->BRR = GPIO_Pin_5
#define         MOSI_HIGH        GPIOA->BSRR= GPIO_Pin_7
#define         MOSI_LOW         GPIOA->BRR = GPIO_Pin_7
#define         MISO_STATUS      GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)
*/
/******************Parameter define *******************/
#define         DEFAULT_CHANNEL                 0x3C                              //初始化时的频率： 2460 MHz           
#define         PAYLOAD_WIDTH                   9                               //Payload宽度： 9bytes
#define         RETRY_MAX                       5                               //最大传输次数（普通型只能为0）

#define         TRANSMIT_TYPE                   TRANS_ENHANCE_MODE              //使用增强型模式(TRANS_TYPE)
#define         RF_POWER                        RF_8dBm                         //发射功率8dBm (RF_SET_POWER)


////////////////////////////////////////////////////////////////////////////////
//                    以下为声明部分，不建议修改                              //
////////////////////////////////////////////////////////////////////////////////
enum  TRANS_RESULT                                                              //通信结果                                                  
{
    MAX_RT = 0x10,                                                              //发送超时（增强型）
    TX_DS  = 0x20,                                                              //发送完成（普通型）  发送成功（增强型）
    RX_DR  = 0x40                                                               //接收到数据
};

enum  TRANS_TYPE                                                                //传输类型
{
    TRANS_ENHANCE_MODE,                                                         //增强型
    TRANS_BURST_MODE                                                            //普通型
};

enum  RF_SET_POWER                                                              //RF发射功率设定
{
    RF_N10dBm = 0x09,                                                           // -10dBm                                                      
    RF_0dBm   = 0x0B,                                                           // 0dBm 
    RF_8dBm   = 0x05,                                                           // 8dBm,1Mbps
    RF_11dBm  = 0x0F                                                            // 11dBm 
};
/***************Read Write XN297 Register**************/
#define		R_REGISTER			0x00                            //spi read RF data
#define		W_REGISTER			0x20                            //spi write RF data

#define		R_RX_PAYLOAD		        0x61                            //Read RX Payload
#define		W_TX_PAYLOAD		        0xA0                            //Write TX Payload
#define		FLUSH_TX			0xE1                            //Flush RX FIFO
#define		FLUSH_RX			0xE2                            //Flush TX FIFO
#define		REUSE_TX_PL			0xE3                            //Reuse TX Payload
#define		ACTIVATE			0x50                            //ACTIVATE
#define		R_RX_PL_WID			0x60                            //Read width of RX data 
#define		W_ACK_PAYLOAD		        0xA8                            //data with ACK
#define		W_TX_PAYLOAD_NOACK	        0xB0                            //TX Payload no ACK Request

#define		CONFIG				0x00            
#define		EN_AA				0x01
#define		EN_RXADDR			0x02
#define		SETUP_AW			0x03
#define		SETUP_RETR			0x04
#define		RF_CH				0x05
#define		RF_SETUP			0x06
#define		STATUS				0x07
#define		OBSERVE_TX			0x08
#define		RPD			        0x09
#define		RX_ADDR_P0			0x0A
#define		RX_ADDR_P1			0x0B
#define		RX_ADDR_P2			0x0C
#define		RX_ADDR_P3			0x0D
#define		RX_ADDR_P4			0x0E
#define		RX_ADDR_P5			0x0F
#define		TX_ADDR				0x10
#define		RX_PW_P0			0x11
#define		RX_PW_P1			0x12
#define		RX_PW_P2			0x13
#define		RX_PW_P3			0x14
#define		RX_PW_P4			0x15
#define		RX_PW_P5			0x16
#define		FIFO_STATUS			0x17
#define		DYNPD				0x1C
#define		FEATURE				0x1D	
#define		DEM_CAL				0x19
#define		RF_CAL				0x1E
#define		BB_CAL				0x1F

//interrupt status
#define STATUS_RX_DR 0x40
#define STATUS_TX_DS 0x20
#define STATUS_MAX_RT 0x10

#define STATUS_TX_FULL 0x01

//FIFO_STATUS
#define FIFO_STATUS_TX_REUSE 0x40
#define FIFO_STATUS_TX_FULL 0x20
#define FIFO_STATUS_TX_EMPTY 0x10

#define FIFO_STATUS_RX_FULL 0x02
#define FIFO_STATUS_RX_EMPTY 0x01


//about MPU6881
#define MPU6050_ADDRESS		0x68 //使用7位地址 address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board
#define MPU6050_DLPF_CFG	0
#define SCL_H         GPIO->PA_DATABSR = 0x00040000
#define SCL_L         GPIO->PA_DATABCR = 0x00040000
#define SDA_H         GPIO->PA_DATABSR = 0x00080000
#define SDA_L         GPIO->PA_DATABCR = 0x00080000
#define SDA_IN         GPIO->PA_DIRBSR = 0x00080000
#define SDA_OUT         GPIO->PA_DIRBCR = 0x00080000
#define SCL_read      GPIO->PA_PORT  & 0x00040000
#define SDA_read      GPIO->PA_PORT  & 0x00080000
#define I2C_Direction_Transmitter       ((uint16_t)0x0000)
#define I2C_Direction_Receiver          ((uint16_t)0x0400)


//about FBM320
#define FBM320_ADDRESS		0x6D
#define	FBM320_MYID			0x6B
#define	FBM320_MYID_ITEM	0x42
#define	FBM320_DATA_LSB		0xF8
#define	FBM320_DATA_CSB		0xF7
#define	FBM320_DATA_MSB		0xF6
#define	FBM320_DATA_CFG		0xF4
#define	FBM320_CFG_8192		0xF4
#define	FBM320_CFG_4096		0xB4
#define	FBM320_CFG_2048		0x74
#define	FBM320_CFG_1024		0x34
#define	FBM320_CFG_DEF		FBM320_CFG_8192


#endif

