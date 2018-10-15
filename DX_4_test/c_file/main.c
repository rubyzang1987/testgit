#include	"config.h"
#include "hr8p506.h"
#include "msic_device.h"
#include "hw2000.h"
#include "flash.h"
#include	"state_estimation.h"



int	main()
{
	hr8p506_scu_init();
//	GPIO->PA_DIRBCR=0x00100400;
//	GPIO->PA_FUNC2=0x00020000;
	SystemGPIOInit();
	hr8p506_motor_init();
//		T16N0->MAT0 = 300;
//		T16N0->MAT2 = 300;
//		T16N3->MAT0 = 300;
//		T16N3->MAT2 = 300;
	SystemHalInit((__ThisHal*) &gvst_ThisHal);
	SystemSoftwareInit();
	hr8p506_adc_init();
//	hw2181_t32n0_init();
	hw2181_uart_init();
	hw2000_port_init();
	hw2000_init_250k(0x031B,0x3800);
	
	hr8p506_interrupt_init();
	while(Mpu6881Initial()==1);
	spl0601_init();
	MissionInit();
	
	gyro_acc_dataread();   //读取陀螺仪数据
	
	GPIO->PA_DATABSR = 0x20000000;     //PA29=1;  输出置位寄存器
	
	MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK);   //开机先失联，用于对码
  if(!GPIOA_READ(19))                //14脚载波模式  PA20
	{
		START_MODE=1;
		MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_TESTF);
		hw2000_port_init();
	  hw2000_init_250k(0x031B,0x3800);
    hw2000_write_reg(0x22, 0x1800 | 58); //channel set
		hw2000_power_test();
		//while(1);
	}
	else
	{
		MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_DMMODE);
		hw2000_rx_enable(MATCH_FRE); //waiting for unlock
	}
	initialize_system_parameter(0.004f,100,10);
  while(1)
  {
//		sleep();
		MissionLoop();
  }
}

//void   sleep_key(void)
//{
//	static  uint8_t  sleep_key_timer=100;
//	if(GPIOA_READ(8))//(GPIOB_READ(6))
//	{
//		sleep_key_timer=0;
//	}
//	if(!GPIOA_READ(8) && ((sleep_key_timer<=10)))
//	{
//		sleep_key_timer++;
//		if(sleep_key_timer>=5 && (sleep_key_timer<=10))
//			GPIO->PA_DATABCR = 0x00000240;
//	}
//	else
//	{
//		if(sleep_key_timer>=2)
//			sleep_key_timer-=2;
//	}
//}

#define START_UP_TIMER 1000
void   sleep_key(void)//ZFM  20180906
{
	static  uint8 kaiji=0;
    static  int8 sleep_key_timer1=0;
	static  uint16_t  sleep_key_timer=0;
	if(!GPIOA_READ(8))
	{
		if(sleep_key_timer1<10)
		  sleep_key_timer1++;
		if(sleep_key_timer1>9)
		{
			if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_START_FLAG))
			{
				if(sleep_key_timer>0)
				{
					if(kaiji==1)
						sleep_key_timer--;
				}
				else
					MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_START_FLAG);
			}
			else
			{
				if(sleep_key_timer<START_UP_TIMER)
				{
					if(kaiji==0)
						sleep_key_timer++;
				}
				else
				{
					MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_START_FLAG);
				}
			}
		}
	}
	else
	{
		if(sleep_key_timer1>-10)
			sleep_key_timer1--;
		if(sleep_key_timer>=START_UP_TIMER)
			kaiji=1;
		if(sleep_key_timer1<-9)
		{
			if(sleep_key_timer<10 && MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_START_FLAG))  //YIKAIJI 
				GPIO->PA_DATABCR = 0x00000240;
			if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_START_FLAG))
				GPIO->PA_DATABCR = 0x00000240;
		}
	}
}
void  sleep(void)
{
	if(sleep_timer>=600)
	{
		sleep_timer=600;
		SYSTICK->CSR=0X00000000;
		hr8p506_led_off();
		hw2000_write_reg(0x21, 0x0000);
		hr8p506_i2c_write(MPU6050_ADDRESS, 0x6B,1, 0x40);
		ADC->CON0.EN = 0;
		SCU->TIMER_EN = 0x00000000;
		
//    GPIO->PA_DATABCR = 0x40000000;
		GPIO->PA_DATABCR = 0x00000200;   //PA9=0;   输出清零寄存器  ZFM
		while(1)
		{
			
		}
	}
}

