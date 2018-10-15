#include	"config.h"

const uint8 lcu8_ledaddr[5][2]={{0xff,0xff},{1,15},{2,5},{0xff,0xff},{0xff,0xff}};//本机led地址
volatile	static	uint32	lvu32_LedFlashCnt=0;
volatile	static	uint32	lvu32_Ledtestcount=0;

//void    usart_data_send(void)    //100ms
//{
////	static uint16 poweron_timer=0;
//	static uint8 usart_i=0;
//	static uint8 datau[14]={0xa1,0,0,0,0,0,0,0,0,0,0,0,0,0x52};
//	uint8 i;
//	if(EV_LED_2==gven_LedFlashKind)//未对码
//	{datau[1] &=0x1e;datau[1] |=0x80;}
//	else if(EV_LED_1==gven_LedFlashKind)//检测陀螺仪
//	{datau[1]&=0x1e;datau[1]|=0x40;}
//	else if(EV_LED_3==gven_LedFlashKind)  //正常量
//	{
//		if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK))
//		{
//			datau[1]&=0x1e;
//		  datau[1]|=0x20;
//		}
//	  else
//		{
//			datau[1]&=0x1e;
//		  datau[1]|=0x01;
//		}
//	}
//	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S2))
//	{datau[1]&=0xc7;}
//	else if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S1))
//	{datau[1]&=0xc7;datau[1]|=0x08;}
//	else if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S3))
//	{datau[1]&=0xe7;datau[1]|=0x10;}
//	else
//	{datau[1]&=0xe7;datau[1]|=0x18;}
////	else if(EV_LED_5==gven_LedFlashKind)    //EV_LED_5  低压   EV_LED_6卡死
////	{datau[1]&=0x16;datau[1]|=0x10;}
//	if(EV_LED_6==gven_LedFlashKind)
//	{datau[2]&=0xef;datau[2]|=0x10;datau[1]&=0xdf;}
//	else
//	{datau[2]&=0xef;}
//	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_PHOTO))
//	{datau[9]&=0xfb;datau[9]|=0x02;}	
////	{datau[9]&=0xfd;datau[9]|=0x04;}
//	else 	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_DV))
//		{datau[9]&=0xfd;datau[9]|=0x04;}
////	{datau[9]&=0xfb;datau[9]|=0x02;}
//	if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_PHOTO))
//	{
//		datau[9]&=0xfd;
////		datau[9]&=0xfb;
//	}
//	if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_DV))
//	{
//		datau[9]&=0xfb;
////		datau[9]&=0xfd;
//	}
//	else if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT))
//	{datau[2] &=0xf0;datau[2] |=0x02;}
//	else if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT))
//	{datau[2] &=0xf0;datau[2] |=0x04;}
//	if(EV_LED_2==gven_LedFlashKind)
//	{datau[2] &=0xf0;datau[2] |=0x00;}
//	
//	//发送信号强度数据
//	if(signal_display<=2)
//	{datau[2]&=0x1f;}
//	else if(signal_display<=4)
//	{datau[2]&=0x1f;datau[2]|=0x20;}
//	else if(signal_display<=6)
//	{datau[2]&=0x1f;datau[2]|=0x40;}
//	else if(signal_display<=8)
//	{datau[2]&=0x1f;datau[2]|=0x60;}
//	else if(signal_display<=10)
//	{datau[2]&=0x1f;datau[2]|=0x80;}
//	else if(signal_display<=12)
//	{datau[2]&=0x1f;datau[2]|=0xa0;}
//	else if(signal_display<=14)
//	{datau[2]&=0x1f;datau[2]|=0xc0;}
//	else
//	{datau[2]&=0x1f;datau[2]|=0xe0;}
//	
//  if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DOCK))
//	{datau[2]&=0x1f;datau[2]|=0xe0;}
//	
//	datau[12]=datau[1]+datau[2]+datau[3]+datau[4]+datau[5]+datau[6]+datau[7]+datau[8]+datau[9]+datau[10]+datau[11];
//	
//	hw2181_uart_tx(datau+usart_i,1);
//	usart_i++;
//	if(usart_i>=14)
//	{
//		for(i=3;i<=10;i++)
//		{
//			datau[i]=gvst_ThisBird.TelDat[i-3];
//		}
//		usart_i=0;
//	}
//}

void	SystemSoftwareInit(void)
{
	memset((void*)gvu8_U1ReceiveBuf,0,32);

	//SetLed(0xff,0);

	gvst_ThisBird.time.miro_sec=0;
	gvst_ThisBird.time.second=0;
	gvst_ThisBird.time.minute=0;
	gvst_ThisBird.time.hour=0;
//	printf("SystemSoftwareInit complete.\r\n");
}

void	LedFlash(void)
{
	//if(EV_LED_M==gven_LedFlashKind)
	//	{SetLed(1,0);SetLed(2,0);}
	/*else */if(EV_LED_1==gven_LedFlashKind)
	{//1.未校准陀螺仪=两灯交替闪烁，频率5hz
		lvu32_LedFlashCnt++;
//		if(lvu32_LedFlashCnt%8==0)
//			usart_data_send();
		if(lvu32_LedFlashCnt<32)
			{hr8p506_led_on();}
		else if(lvu32_LedFlashCnt<64)
			{hr8p506_led_off();}
		else
		{
			lvu32_LedFlashCnt=0;
		}
	}
	//else if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10))
	else if(START_MODE==1)
	{
		//MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DOCK);
	  //MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DROP);
		lvu32_LedFlashCnt++;
		if(lvu32_LedFlashCnt<32 && (lvu32_Ledtestcount<=10))
			{hr8p506_led_on();}
		else if(lvu32_LedFlashCnt<64)
			{hr8p506_led_off();}
		else
		{
			lvu32_Ledtestcount++;
			if(lvu32_Ledtestcount>=20)
				lvu32_Ledtestcount=0;
			lvu32_LedFlashCnt=0;
		}
	}
	else if(EV_LED_2==gven_LedFlashKind)
	{//2.遥控器未对码或丢失=两灯同时闪烁，频率4hz
		lvu32_LedFlashCnt++;
		if(lvu32_LedFlashCnt<=125)
			{hr8p506_led_on();}
		else if(lvu32_LedFlashCnt<=250)
			{hr8p506_led_off();}
		else
			lvu32_LedFlashCnt=0;
	}
	else if(EV_LED_3==gven_LedFlashKind)
	{//3.已对码=两灯全亮。
		hr8p506_led_on();
		lvu32_LedFlashCnt=0;
	}/*
	else if(EV_LED_4==gven_LedFlashKind)
	{//4.飞行器校零=两灯同时闪烁6次，频率2hz
		lvu32_LedFlashCnt++;
		if(lvu32_LedFlashCnt<50)
			{SetLed(1,1);SetLed(2,1);}
		else if(lvu32_LedFlashCnt<100)
			{SetLed(1,0);SetLed(2,0);}
		else if(lvu32_LedFlashCnt<150)
			{SetLed(1,1);SetLed(2,1);}
		else if(lvu32_LedFlashCnt<200)
			{SetLed(1,0);SetLed(2,0);}
		else if(lvu32_LedFlashCnt<250)
			{SetLed(1,1);SetLed(2,1);}
		else if(lvu32_LedFlashCnt<300)
			{SetLed(1,0);SetLed(2,0);}
		else if(lvu32_LedFlashCnt<350)
			{SetLed(1,1);SetLed(2,1);}
		else if(lvu32_LedFlashCnt<400)
			{SetLed(1,0);SetLed(2,0);}
		else if(lvu32_LedFlashCnt<450)
			{SetLed(1,1);SetLed(2,1);}
		else if(lvu32_LedFlashCnt<500)
			{SetLed(1,0);SetLed(2,0);}
		else if(lvu32_LedFlashCnt<550)
			{SetLed(1,1);SetLed(2,1);}
		else if(lvu32_LedFlashCnt<600)
			{SetLed(1,0);SetLed(2,0);}
		else
			{lvu32_LedFlashCnt=0;gven_LedFlashKind=EV_LED_NULL;}
	}*/
	else if(EV_LED_5==gven_LedFlashKind)
		{//5.飞行器低压=两灯同时闪烁，频率2hz
		lvu32_LedFlashCnt++;
		if(lvu32_LedFlashCnt<250)
			{hr8p506_led_on();}
		else if(lvu32_LedFlashCnt<500)
			{hr8p506_led_off();}
		else
			{lvu32_LedFlashCnt=0;}
	}
	else if(EV_LED_7==gven_LedFlashKind)
	{
		lvu32_LedFlashCnt++;
		if(lvu32_LedFlashCnt<500)
			{hr8p506_led_on();}
		else if(lvu32_LedFlashCnt<700)
			{hr8p506_led_off();}
		else
			{lvu32_LedFlashCnt=0;}
	}
	else if(EV_LED_6==gven_LedFlashKind)
		{//6.飞行器卡死=两灯同时双闪，频率0.5S/次
		lvu32_LedFlashCnt++;
		if(lvu32_LedFlashCnt<100)
			{hr8p506_led_on();}
		else if(lvu32_LedFlashCnt<200)
			{hr8p506_led_off();}
		else if(lvu32_LedFlashCnt<300)
			{hr8p506_led_on();}
		else if(lvu32_LedFlashCnt<400)
			{hr8p506_led_off();}
		else if(lvu32_LedFlashCnt<500)
			{hr8p506_led_on();}
		else if(lvu32_LedFlashCnt<1200)
			{hr8p506_led_off();}
		else
			{lvu32_LedFlashCnt=0;}
	}
}

