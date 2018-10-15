/******************************************************************************
 * @file     interrupt.c
 * @brief    interrupt function
 *           
 *
 * @version  1.0
 * @date     2015-12-02
 *
 * @author   Sundy
 *
 * @note
 * @Copyright (C) 2015 Shanghai Eastsoft Microelectronics C0., Ltd.
 ******************************************************************************/
#include <stdint.h>
#include "hr8p506.h"
#include	"config.h"
#include "hw2000.h"
#include	"flash.h"

volatile	static	int	lv32_ReadTelCnt=0;
volatile	static	int	lv32_ReadIMUCnt=0;
volatile	static	int	lv32_PhotoCnt=0;
volatile	static	int	lv32_DvCnt=0;
static uint16_t   oc_pwm_timer,reset_wifi_time=0,TX_waittimer=0;
volatile static uint8 low_battery_count=0;
int8_t  led=1,lowlvdtimer=0;
extern uint16 lost_timer;

u16 Power_value=0;
/******************************************************************************
 * @brief    interrupt initilaize
 *           
 * @note
 *
 * @param
 * @retval	 None
 *
 * @version  1.0
 * @date     2015-12-02
 * @author   sundy
 ******************************************************************************/
void hr8p506_interrupt_init(void)
{	
	SYSTICK->CSR=0X00000007;
	//SYSTICK->RVR=36800;
	SYSTICK->RVR=48000;
	
	NVIC->ICER = 0x00000008; //clear pint3 interrupt flag
	NVIC->ISER = 0x00000008; //enable pint3 interrupt
	NVIC->PR0 |= 0xC0000000; //lowerst priority	
	
	NVIC->ICER = 0x00000010; //clear pint4 interrupt flag
	NVIC->ISER = 0x00000010; //enable pint4 interrupt
	NVIC->PR1 |= 0x000000C0; //lowerst priority	
	
	NVIC->ICER = 0x00800000; //clear euart interrupt flag
	NVIC->ISER = 0x00800000; //enable euart interrupt
	NVIC->PR5 |= 0xC0000000; //lowerst priority
}

//void hw2181_t16n2_enable(uint16_t time)
//{
//	T16N2->CNT0.Word = 0x00000000; 
//	T16N2->IF.Word = 0x00000001; 
//	T16N2->IE.Word = 0x00000000;	
//	T16N2->CON0.Word = 0x00000200; 
//		
//	T16N2->MAT0.Word = time; //2.5ms x match
//	T16N2->CON0.Word = 0x00000201;  //clear, int again for mat0
//	T16N2->IE.Word = 0x00000001;	//enable t16n2 mat0 int
//}

/******************************************************************************
 * @brief    pint3_handler
 *           
 * @note
 *
 * @param
 * @retval	 None
 *
 * @version  1.0
 * @date     2015-12-02
 * @author   sundy
 ******************************************************************************/
void pint3_handler(void) 
{
	uint8 i;
	GPIOE->PINTIF  = 0x00000008; //clear int
	
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_SENDTX) || START_MODE==1)
		return;
	
	if (hw2000_rx_data(dataTel) == 0)
	  _hw2000_irq_request = 1;
	else
		_hw2000_irq_request = 0;
	
	if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DOCK))   //未对码
	{
		if (dataTel[1] == 0xF0) //对吗状态
		{
			link_select=LINK_2P4G;
			hw2000_rx_enable(MATCH_FRE);  //对码使能
		}
	}
	else
	{
		if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_SENDTX))
		{
			if (dataTel[1] == 0xF7 && quad_idf == (dataTel[3] << 24 | dataTel[4] << 16 | dataTel[5] << 8 | dataTel[6]) && _hw2000_irq_request==1)
			{
				if(TelDatCheck(dataTel+7,9)==TRUE)
				{
					for(i=0;i<=8;i++)
					{
						gvst_ThisBird.TelDat[i]=dataTel[i+7];
						if(i<=6)
							dataTX[i]=dataTel[i];
					}
					if(hold_high_statue==HOLD_HIGH_STOP)
						realthrottlesave=540;
					
					lost_timer=0;
					MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK);
					MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_LINKDOWN);
					if(indexcount != dataTel[2])
					  signal_strength++;  //有效数据累加
					indexcount = dataTel[2];
					
					//转TX
					if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S1) || hold_high_statue==HOLD_HIGH_STOP)
					{
						disable_irq();
						hw2000_write_reg(0x21, 0x0000);
						hw2000_write_reg(0x23, 0x431B); //Soft reset
						hw2000_write_reg(0x23, data1a); 
						hw2000_write_reg(0x3C, 0x1000); //ACK disable
						enable_irq();
						MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_RX);
						TX_waittimer=0;
					}
					
				}
				indexcount = dataTel[2];
				hw2000_rx_enable(channel[channel_num]);
			}
		}
	}
}

/******************************************************************************
 * @brief    pint4_handler
 *           
 * @note
 *
 * @param
 * @retval	 None
 *
 * @version  1.0
 * @date     2015-12-09
 * @author   sundy
 ******************************************************************************/
void pint4_handler(void) 
{
	GPIOE->PINTIF  = 0x00000010; //clear int
}

static uint8 gyro_calcount=0;
void SYSTICK_HANDLER(void)
{
	gvu32_SystemRunTime++;
	ms_count++;
	
	tx_rx_delayms++;
//	carrier_time_count++;
//	sleep_key();
	
		//TX
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_RX))
	{
		TX_waittimer++;
		if(TX_waittimer==3 || TX_waittimer==6)
		{
			#if APEX
				dataTX[0]=9;
			#else
				dataTX[0]=7;
			#endif		
//			dataTX[2]=FH_timer&0x07;
			if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S1))
				dataTX[7]|=0x55;
			else
				dataTX[7]&=0xaa;
			if((dataTX[7]&0xaa)==0xaa)
				gyro_calcount++;
			else
				gyro_calcount=0;
			if(gyro_calcount>=15)
				dataTX[7]&=0x55;
			#if APEX
				dataTX[2]+=2;
				dataTX[8]++;
				dataTX[9]=dataTX[2]+dataTX[8]+dataTX[7];
			#else
				dataTX[2]=dataTX[7];
				dataTX[8]=0x85;
			#endif
			hw2000_tx_data(dataTX,channel[channel_num]);
		}
		if(TX_waittimer==7)  //7
		{
			TX_waittimer=0;
			MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_RX);
			hw2000_rx_enable(channel[channel_num]);
//			radio_carrier_detect(1);   //跳频
		}
	}
	//U1
//	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_U1BUSY))
//	{
//		if(gvu8_U1ReceiveTimeCnt++>MV_U1_TIMEOUT)
//		{
//			MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_U1BUSY);
//			MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_U1DONE);
//			gvu8_U1ReceiveBuf[gvu8_U1ReceiveBufCnt]='\0';
//			gvu8_U1ReceiveTimeCnt=0;
//		}
//	}

	//led
	LedFlash();

	//loop
	if(lv32_ReadTelCnt++>MV_FREQ_READ_TEL)
		{
			if(reset_wifi_time<30)
			  reset_wifi_time++;
			if(reset_wifi_time==30)  //开机600ms后给WiFi上电，若电压低于2.95V则关机
			{
				reset_wifi_time=250;
				if(gvst_ThisBird.system_vol[System_Vol_PowerLow]>2150)
					GPIO->PB_DATABSR = 0x0000000C;
				else if(gvst_ThisBird.system_vol[System_Vol_PowerLow]>235)
					GPIO->PB_DATABCR = 0x0000000C;
			}
			if(throttle_stop_time>=1)
			  throttle_stop_time++;
			if(throttle_stop_time>250) throttle_stop_time=250;
		MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_READ_TEL);
		lv32_ReadTelCnt=0;}

	if(lv32_ReadIMUCnt++>=gvu32_ImuSamFeqCnt)
		{
			
		MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_READ_IMU);
		lv32_ReadIMUCnt=0;}
		


	//DV&PHOTO
//	if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S1))
	{
		if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_PHOTO))//MV_SYSTEM_FLG_GET_PHOTO
		{
//			GPIO->PA_DATABCR = 0x04000000;
			if(lv32_PhotoCnt++>MV_PHOTO_TIMEOUT)
			{
				MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_PHOTO);
//				GPIO->PA_DATABSR = 0x04000000;
				lv32_PhotoCnt=0;
			}
		}
//		if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_DV))//MV_SYSTEM_FLG_GET_PHOTO
//		{
//			GPIO->PA_DATABCR = 0x08000000;
//		}
//		else
//		{
//			GPIO->PA_DATABSR = 0x08000000;
//		}
	}
//pwm50msopen1msclose
	  oc_pwm_timer++;
		if(oc_pwm_timer>=50)
		{
			MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_50MSPWM);
			oc_pwm_timer=0;
    }
	//clock
	gvst_ThisBird.time.miro_sec++;
	if(gvst_ThisBird.time.miro_sec>=1000)
	{
		gvst_ThisBird.time.miro_sec=0;
		sleep_timer++;
		if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S1))
		{
			low_battery_count++;
			if(low_battery_count>=50)
			{
				low_battery_count=100;
				MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S2);
			}
		}
		else
		{
			low_battery_count=0;
		}
		
		if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S2))  //二级低压
	  {
			lowlvdtimer++;
			if(lowlvdtimer>=5)
			{
				lowlvdtimer=100;
				GPIO->PB_DATABCR = 0x00000004;   //关WIFI
			}
	  }

//		if(gvst_ThisBird.time.second>=60)
//		{
//			gvst_ThisBird.time.second=0;
//			gvst_ThisBird.time.minute++;
//			if(gvst_ThisBird.time.minute>=60)
//			{
//				gvst_ThisBird.time.minute=0;
//				gvst_ThisBird.time.hour++;
//			}
//		}     
	}
}

void INT_UART0(void) 
{
//	hr8p506_led_on();
	if(gvu8_U1ReceiveBufCnt>MV_U1_BUF-1)	gvu8_U1ReceiveBufCnt=0;
	gvu8_U1ReceiveBuf[gvu8_U1ReceiveBufCnt++] = UART0->RBR.Byte[0];
	if(gvu8_U1ReceiveBuf[0]!=0xcc)
		gvu8_U1ReceiveBufCnt=0;
	if(gvu8_U1ReceiveBuf[0]==0xcc && (gvu8_U1ReceiveBuf[10]==0x33))
	{
		lost_timer=0;
		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK);
		if(link_select==LINK_NOLINK)
		{
			MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DOCK);
			MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DROP);
			link_select=LINK_USART;
		}
	}
//	hr8p506_led_off();
}




