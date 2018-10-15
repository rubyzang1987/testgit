/******************************************************************************
 * @file     scu.c
 * @brief    hr8p506 system control unit function
 *           
 *
 * @version  1.0
 * @date     2015-12-01
 *
 * @author   Sundy
 *
 * @note
 * @Copyright (C) 2015 Shanghai Eastsoft Microelectronics C0., Ltd.
 ******************************************************************************/
#include <stdint.h>
#include "hr8p506.h"
#include "msic_device.h"
#include "hw2000.h"
#include	"flash.h"

#define I2C0_WRITE()  	I2C->CON.RW = 0
#define I2C0_READ()  	I2C->CON.RW = 1

#define I2C0_START()    I2C->MODE.SRT = 1
#define I2C0_STOP()     I2C->MODE.SPT = 1
#define I2C0_RECEIVE()  I2C->MODE.RDT = 1
#define I2C0_RELEASE()  I2C->MODE.BLD = 1


//#define FREQUENCY_MAX	2483	//2.4G???????
//#define FREQUENCY_MIN	2457	//2.4G???????

//#define CHANNEL_MAXX	 (FREQUENCY_MAX - 2402)	 //?????????
//#define CHANNEL_MIN	 (FREQUENCY_MIN - 2402)	 //?????????
//#define CHANNEL_SPACE	((CHANNEL_MAXX - CHANNEL_MIN - 15)/3)	//????

uint16 lost_timer=0;

#define	CHANNEL_MAX		5
#define	REPEAT_MAX		5

//const uint8_t _hw2000_channel_table[40] = {74, 76, 79, 75, 77,
//										   74, 76, 79, 75, 77,
//										   74, 76, 79, 75, 77,
//										   74, 76, 79, 75, 77,
//										   74, 76, 79, 75, 77,
//										   74, 76, 79, 75, 77,
//										   74, 76, 79, 75, 77,
//										   74, 76, 79, 75, 77};

const uint8_t _hw2000_channel_table[40] = {56, 65, 70, 79, 74,
										   58, 68, 78, 72, 62,
										   55, 59, 63, 69, 75,
										   59, 63, 69, 74, 79,
										   58, 63, 68, 73, 78,
										   57, 62, 67, 72, 77,
										   56, 61, 66, 71, 76,
										   55, 60, 65, 70, 75};

/******************************************************************************
 * @brief    System control unit initialization
 *           Clock initializing for 48MHz
 * @note
 *
 * @param
 * @retval	 None
 *
 * @version  1.0
 * @date     2015-12-01
 * @author   sundy
 ******************************************************************************/
void hr8p506_scu_init(void)
{
	SCU->SCU_PROT.Word = 0x55AA6996; //Disable SCU protection	 ¹Ø±ÕSCUÐ´±£»¤
	
	SCU->CLKEN_SYS1.Word |= 0x00000002;	//HRC_EN = 1		Ê¹ÄÜÄÚ²¿Õñµ´Æ÷µçÂ·
	while (!SCU->CLKEN_SYS1.HRC_RDY); //wait for HRC_RDY	µÈ´ýÄÚ²¿Õñµ´Æ÷ÎÈ¶¨
	
//	SCU->CLKEN_SYS1.Word = 0x00001A02;	//48MHz PLL enable, 4MHz
//	while (!SCU->CLKEN_SYS1.PLL_RDY); //wait for PLL_RDY
//	SCU->CLKEN_SYS0.Word = 0x04550100;
	
	SCU->CLKEN_SYS1.Word = 0x00003A02;	//48MHz PLL enable, 16MHz
	while (!SCU->CLKEN_SYS1.PLL_RDY); //wait for PLL_RDY
	SCU->CLKEN_SYS0.Word = 0x04550100;
	
//	SCU->CLKEN_SYS0.Word = 0x04550000;	//HRC 

	SCU->SCU_PROT.Word = 0x00000000; //Enable SCU protection	¿ªÆôÐ´±£»¤
}

/******************************************************************************
 * @brief    led on function
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
void hr8p506_led_on(void)
{
		GPIO->PA_DATABSR = 0x00040200;
		GPIO->PB_DATABCR =0x00000D00;
//	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_START_FLAG))
//	{
//		GPIO->PA_DATABSR = 0x00040200;
//		GPIO->PB_DATABCR =0x00000D00;
//	}
//	else
//	{
//		GPIO->PA_DATABCR = 0x00040200;
//		GPIO->PA_DATABSR = 0x00000D00;
//	}
		
}

/******************************************************************************
 * @brief    led off function
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
void hr8p506_led_off(void)
{
	GPIO->PA_DATABCR = 0x00040200;
	GPIO->PB_DATABSR =0x00000D00;//PB8,PB10,PB11
}

void hw2181_t32n0_init(void)       //¶¨Ê±Æ÷³õÊ¼»¯
{	
	SCU->SCU_PROT.Word = 0x55AA6996; //Disable SCU protection
	SCU->CLKEN_PERI.CLKEN_T32N0 = 1; //enable t32n0 clk
	SCU->SCU_PROT.Word = 0x00000000; //Enable SCU protection
	
	T32N0->PREMAT = 47; //1MHz
}

void hw2181_uart_init(void)
{
	SCU->SCU_PROT.Word = 0x55AA6996; //Disable SCU protection
	SCU->CLKEN_PERI.CLKEN_UATR0 = 1; //enable euart0 clk
	SCU->SCU_PROT.Word = 0x00000000; //Enable SCU protection
	
	GPIO->PA_DIRBSR = 0x04000000;	//PA 26 is input uart
	GPIO->PA_FUNC3 &= 0xFFFFF0FF;
	GPIO->PA_FUNC3 |= 0x00000300;
	GPIO->PA_PUE |= 0x04000000;
	
	GPIO->PA_DIRBCR = 0x08000000;		//PA 27 is output uart
	GPIO->PA_FUNC3 &= 0xFFFF0FFF;
	GPIO->PA_FUNC3 |= 0x00003000;
	
	UART0->CON0.Word = 0x00060006;
	UART0->CON1.Word = 0x00000300;
	UART0->BRR.Word = 318;//104;//618;//104;////1236;  //¸Ä´®¿ÚÍ¨ÐÅ²¨ÌØÂÊÎª115200 ZFM  20180611
	UART0->IE.RB = 1;
	UART0->CON0.Word = 0x00010001;
}

void hw2181_uart_tx(uint8_t *data, uint8_t length)
{
	uint8_t i;
	
	for (i = 0; i < length; i++) {
		UART0->TBW.Byte[0] = data[i];
		while (!(UART0->IF.TB));
	}			
}

void	SystemGPIOInit(void)
{
	//I2C init
	SCU->SCU_PROT.Word = 0x55AA6996; //Disable SCU protection
	SCU->CLKEN_PERI.CLKEN_I2C = 1;  //enable i2c0 clk
	SCU->SCU_PROT.Word = 0x00000000; //Enable SCU protection
	
//	GPIO->PA_DIR &= 0xFFFBFFFF; //pa18 is output for scl
//	GPIO->PA_FUNC2 |= 0x00002200; //pa19 is sda, pa18 is scl   ¸´ÓÃFUN2  SCL0  SDA0
//	
//	GPIO->PA_PUE = 0x000C0000;    //¶Ë¿ÚÈõÉÏÀ­Ê¹ÄÜÎ»  PA18  PA19¾ùÎªÈõÉÏÀ­
	GPIO->PA_DIR &= 0xFFFFFFEF; //pa18 is output for scl
	GPIO->PA_FUNC0 |= 0x00110000; //pa19 is sda, pa18 is scl   ¸´ÓÃFUN2  SCL0  SDA0
	
	GPIO->PA_PUE = 0x00000030;    //¶Ë¿ÚÈõÉÏÀ­Ê¹ÄÜÎ»  PA18  PA19¾ùÎªÈõÉÏÀ­
	//SPI0,SPI1
	//SPI0  2.4GHZ COMMUNICATE
	SCU->SCU_PROT.Word = 0x55AA6996; //Disable SCU protection
	SCU->CLKEN_PERI.CLKEN_SPI0 = 1; //enable spi0 clk
	SCU->SCU_PROT.Word = 0x00000000; //Enable SCU protection
	
	GPIO->PA_DIRBSR = 0x00001000;	//PA 12 is input for MISO
	GPIO->PA_DIRBCR = 0x0000A000;	//PA 13 is output for MOSI, PA15 is output for SCK
	GPIO->PA_FUNC1 &= 0x0F00FFFF;
	GPIO->PA_FUNC1 |= 0x10330000;
	
	SPI0->CON.Word = 0xC7000002; //reset
	SPI0->CKS.CKS = 3; //8MHz
	SPI0->IE.Word = 0x00000000; //disable all int, byte int mode
	//SPI1
//	SCU->SCU_PROT.Word = 0x55AA6996; //Disable SCU protection
//	SCU->CLKEN_PERI.CLKEN_SPI1 = 1; //enable spi1 clk
//	SCU->SCU_PROT.Word = 0x00000000; //Enable SCU protection
//	
//	GPIO->PA_DIRBSR = 0x00000010;	//PA 25 is input for MISO
//	GPIO->PA_DIRBCR = 0x00000028;	//PA 24 is output for MOSI, PA26 is output for SCK
//	GPIO->PA_FUNC3 &= 0xFFFFF000;
//	GPIO->PA_FUNC3 |= 0x00000111;
//	
//	SPI1->CON.Word = 0xC7000042; //reset   Çå¿Õ»º³åÆ÷ ÏÂ½µÑØ·¢ËÍ£¨ÏÈ£      ÉÏÉýÑØ½ÓÊÕ©
//	SPI1->CKS.CKS = 24; //1MHz   Í¨Ñ¶Ê±ÖÓ²¨ÌØÂÊÉèÖÃ
//	SPI1->IE.Word = 0x00000000; //disable all int
	//PWM I/O init  ZFM
    SCU->SCU_PROT.Word = 0x55AA6996; //Disable SCU protection
	SCU->CLKEN_PERI.CLKEN_T16N0 = 1; //enable t16n0 clk
	SCU->CLKEN_PERI.CLKEN_T16N3 = 1; //enable t16n3 clk
	SCU->SCU_PROT.Word = 0x00000000; //Enable SCU protection

	GPIO->PB_DIRBCR = 0x0000000F; //PB01~03 output for pwm
	GPIO->PB_FUNC0 &= 0xFFFF0000;
	GPIO->PB_FUNC0 |= 0x00001133; //PB00~PB01 is t16n3, PB02~PB03 is t16n0
//	SCU->SCU_PROT.Word = 0x55AA6996; //Disable SCU protection
//	SCU->CLKEN_PERI.CLKEN_T16N0 = 1; //enable t16n0 clk
//	SCU->CLKEN_PERI.CLKEN_T16N3 = 1; //enable t16n3 clk
//	SCU->SCU_PROT.Word = 0x00000000; //Enable SCU protection

//	GPIO->PB_DIRBCR = 0x0000000F; //PB01~03 output for pwm
//	GPIO->PB_FUNC0 &= 0xFFFF0000;
//	GPIO->PB_FUNC0 |= 0x00001133; //PB00~PB01 is t16n3, PB02~PB03 is t16n0
	//LED init
    	//ÐÞ¸ÄLEDµÆ½ÅÎ» zfm 20180418
	GPIO->PA_DIRBCR = 0x0040200; //PA18,PA9 is ouput   PA9ÎªµçÔ´Æô¶¯½Å 
	GPIO->PA_DS = 0x0040200;   //PA18,PA9 Ç¿µçÁ÷Çý¶¯
	GPIO->PA_DATABSR=0x0040200;//PA9ÖÃÎ»
//	GPIO->PA_DATABCR = 0x0040200;   //PA18,PA9   Êä³öÇåÁã¼Ä´æÆ÷
  //Ôö¼Ó7²ÊµÆµÄ´¦Àí   20180904  ZFM 
	GPIO->PB_DIRBCR=0x00000D00;
	GPIO->PB_DS=0x00000D00;
//	GPIO->PA_DATABSR =0x00000D00;
	GPIO->PB_DATABCR =0x00000D00; //ÊäÈë¿Ú,µÍµçÆ½µãÁÁ
  
//	GPIO->PA_DIRBCR = 0x00800240; //PA29 is ouput
//	GPIO->PA_DS = 0x00800240;   //PA29 Ç¿µçÁ÷Çý¶¯
//	GPIO->PA_DATABCR = 0x00800000;   //PA9=0;   Êä³öÇåÁã¼Ä´æÆ÷
//	GPIO->PA_DATABSR = 0x00000040;     //PA28=1;  Êä³öÖÃÎ»¼Ä´æÆ÷
//	GPIO->PA_DATABCR = 0x00800200;
//	
//	GPIO->PB_DIRBCR = 0x00000800; //PB10  PB11 is ouput
//	GPIO->PB_DS = 0x0000800;   //PB10  PB11 Ç¿µçÁ÷Çý¶¯
//	GPIO->PB_DATABCR = 0x00000800;   //PB10  PB11=0;   Êä³öÇåÁã¼Ä´æÆ÷
	
	//ÔØ²¨·¢Éä°´¼ü¶Ë¿Ú
	GPIO->PA_DIRBSR = 0x00080100;  //PA8 PA9  ÉèÖÃÎªÊäÈë
	GPIO->PA_PUE = 0x00080100;    //PA8  PA9  ¶Ë¿ÚÉÏÀ­
}


int8_t  GPIOA_READ(uint32_t PIN)
{
	uint8_t bitstatus = 0x00;
	uint32_t GPIO_PIN=0x00000001 << PIN;
	if(GPIO->PA_PORT & GPIO_PIN)
		bitstatus=1;
	else
		bitstatus=0;
	return bitstatus;
}
//int8_t  GPIOB_READ(uint32_t PIN)
//{
//	uint8_t bitstatus = 0x00;
//	uint32_t GPIO_PIN=0x00000001 << PIN;
//	if(GPIO->PB_PORT & GPIO_PIN)
//		bitstatus=1;
//	else
//		bitstatus=0;
//	return bitstatus;
//}
//adc init  ZFM 
void hr8p506_adc_init(void)
{	
  SCU->SCU_PROT.Word = 0x55AA6996; //Disable SCU protection
	SCU->CLKEN_PERI.CLKEN_ADC = 1; //enable adc clk
	SCU->SCU_PROT.Word = 0x00000000; //Enable SCU protection
	
	GPIO->PA_INEB = 0x00000084;    //PA7  PA2¹Ø±ÕÊý×ÖÊäÈë¿Ú  Ê¹ÄÜÄ£ÄâÍ¨µÀ¶Ë¿Ú
	GPIO->PA_DIRBSR = 0x00000084;
	GPIO->PB_INEB = 0x000020E0;   //PB5,6,7¹Ø±ÕÊý×ÖÊäÈë¿Ú  Ê¹ÄÜÄ£ÄâÍ¨µÀ¶Ë¿Ú
	GPIO->PB_DIRBSR = 0x000020E0;
    ADC->VREF_CON.Word = 0x00000005;   //
	delay_ms(30);
	ADC->CON1.Word = 0x031F5804;	//3MHz AD CLK / 15 = 200KHz //bit9-8£º00±íÊ¾µÄÊÇÑ¡ÔñADµÄ¹¤×÷µçÑ¹
	delay_ms(30);
	ADC->CHS.VDD5_FLAG_EN = 1;
	ADC->CON0.EN = 1;
	ADC->IF.Word = 0x0000000F;
	delay_ms(120);
	
//	SCU->SCU_PROT.Word = 0x55AA6996; //Disable SCU protection
//	SCU->CLKEN_PERI.CLKEN_ADC = 1; //enable adc clk
//	SCU->SCU_PROT.Word = 0x00000000; //Enable SCU protection
//	
//	GPIO->PA_INEB = 0x00000084;    //PA7  PA2¹Ø±ÕÊý×ÖÊäÈë¿Ú  Ê¹ÄÜÄ£ÄâÍ¨µÀ¶Ë¿Ú
//	GPIO->PA_DIRBSR = 0x00000084;
//	GPIO->PB_INEB = 0x000020E0;   //PB5,6,7¹Ø±ÕÊý×ÖÊäÈë¿Ú  Ê¹ÄÜÄ£ÄâÍ¨µÀ¶Ë¿Ú
//	GPIO->PB_DIRBSR = 0x000020E0;
//	
//	ADC->VREF_CON.Word = 0x00003A1D;   //bit1£¬ÄÚ²¿VREFPµçÑ¹Ñ¡Ôñ¿ØÖÆÎ»Îª1.8V   ZFM  20180511
//	delay_ms(30);
//	ADC->CON1.Word = 0x031F5904;	//3MHz AD CLK / 15 = 200KHz²ÉÑùÂÊ
//	delay_ms(30);
//	ADC->CHS.VDD5_FLAG_EN = 1;
//	ADC->CON0.EN = 1;
//	ADC->IF.Word = 0x0000000F;
//	delay_ms(120);
}
uint16_t hr8p506_adc_sample(uint8_t ch)
{
	uint16_t sample = 0;
	
	ADC->CHS.CHS = ch;   //ADC×ª»»Í¨µÀÑ¡Ôñ
	ADC->CON0.TRG = 1;   //ÖÃ1Îª¿ªÆôADC×ª»»

	
	while (!ADC->IF.IF);
	sample = ADC->DR.Word;
	ADC->IF.Word = 0x0000000F;
	return sample;
}
void   all_adc_read(void)
{
	static  uint8  adc_ch=13;   //³õÊ¼»¯Îª¼ì²âµçÔ´µçÑ¹ad
	ADC->CHS.CHS = adc_ch;
	ADC->CON0.TRG = 1;   //ÖÃ1Îª¿ªÆôADC×ª»»
	ADC->IF.Word = 0x0000000F;
	while (!ADC->IF.IF);
	if(adc_ch==13)
	{
		gvst_ThisBird.system_vol[System_Vol_PowerLow]=ADC->DR.Word;
		adc_ch=2;
		GPIO->PA_DIRBCR = 0x00000084;
		GPIO->PB_DIRBCR = 0x000000E0;
		return;
	}
	else if(adc_ch==2)
	{
		gvst_ThisBird.system_vol[System_Vol_T1]=ADC->DR.Word;
		adc_ch=0;
		GPIO->PA_DIRBCR = 0x00000084;
		GPIO->PB_DIRBCR = 0x000000E0;
//		ADC->CHS.CHS = adc_ch;
		return;
	}
	else if(adc_ch==0)
	{
		gvst_ThisBird.system_vol[System_Vol_T2]=ADC->DR.Word;
		adc_ch=5;
		GPIO->PA_DIRBCR = 0x00000084;
		GPIO->PB_DIRBCR = 0x000000E0;
//		ADC->CHS.CHS = adc_ch;
		return;
	}
	else if(adc_ch==5)
	{
		gvst_ThisBird.system_vol[System_Vol_T3]=ADC->DR.Word;
		adc_ch=12;
		GPIO->PA_DIRBCR = 0x00000084;
		GPIO->PB_DIRBCR = 0x000000E0;
//		ADC->CHS.CHS = adc_ch;
		return;
	}
	else if(adc_ch==12)
	{
		gvst_ThisBird.system_vol[System_Vol_T4]=ADC->DR.Word;
		adc_ch=13;
		GPIO->PA_DIRBCR = 0x00000084;
		GPIO->PB_DIRBCR = 0x000000E0;
//		ADC->CHS.CHS = adc_ch;
		return;
	}
}
//motor
void hr8p506_motor_init(void)
{
    T16N0->CON2 = 0x00009903; //pwm config
	//bit15-14:10 T16N_MAT3Æ¥Åäºó¶Ë¿Ú1¹¤×÷Ä£Ê½Ñ¡ÔñÎ»±íÊ¾Îª¶Ë¿ÚÖÃ1
	//bit13-12:10 T16N_MAT2Æ¥ÅäºóµÄ¶Ë¿ÚÖÃ1    01±íÊ¾T16N_MAT2Æ¥Åä¶Ë¿ÚºóµÄ¶Ë¿Ú1Çå0
	//bit11-10:10 T16N_MAT1¶Ë¿ÚÖÃ1
	//bit9-8:01 T16N_MAT0Æ¥ÅäºóµÄ¶Ë¿ÚÇå0
	//bit5:0±íÊ¾PWM»¥²¹Ä£Ê½ËÀÇø½ûÖ¹
	//bit4:0±íÊ¾PWMÄ£Ê½¶ÀÁ¢
	//bit3:0±íÊ¾T16Nx¡ª¡ªOUT1Êä³ö¼«ÐÔÑ¡ÔñÎ»ÎªÕý¼«ÐÔ
	//bit2:0±íÊ¾T16Nx¡ª¡ªOUT0Êä³ö¼«ÐÔÑ¡ÔñÎ»ÎªÕý¼«ÐÔ
	//bit1:0Êä³ö¶Ë¿Ú1½ûÖ¹   1±íÊ¾Êä³ö¶Ë¿Ú1Ê¹ÄÜ
	//bit0:1Êä³ö¶Ë¿Ú0Ê¹ÄÜ
//    T16N0->PREMAT = 16; //1:2   Ô¤·ÖÆµÆ÷¼ÆÊý±ÈÀý¼Ä´æÆ÷
	T16N0->PREMAT = 2; //1:2   Ô¤·ÖÆµÆ÷¼ÆÊý±ÈÀý¼Ä´æÆ÷
	T16N0->MAT0 = 1000 - 1;    //¼ÆÊýÆ¥Åä¼Ä´æÆ÷
	T16N0->MAT1 = 1000 - 1;    //¼ÆÊýÆ¥Åä¼Ä´æÆ÷
	T16N0->MAT2 = 1000 - 1;    //¼ÆÊýÆ¥Åä¼Ä´æÆ÷
	T16N0->MAT3 = 1000 - 1;    //¼ÆÊýÆ¥Åä¼Ä´æÆ÷

	T16N3->CON2 = 0x00009903; //pwm config
	//bit15-14:10 T16N_MAT3Æ¥Åäºó¶Ë¿Ú1¹¤×÷Ä£Ê½Ñ¡ÔñÎ»±íÊ¾Îª¶Ë¿ÚÖÃ1
	//bit13-12:10 T16N_MAT2Æ¥ÅäºóµÄ¶Ë¿ÚÖÃ1    01±íÊ¾T16N_MAT2Æ¥Åä¶Ë¿ÚºóµÄ¶Ë¿Ú1Çå0
	//bit11-10:10 T16N_MAT1¶Ë¿ÚÖÃ1
	//bit9-8:01 T16N_MAT0Æ¥ÅäºóµÄ¶Ë¿ÚÇå0
	//bit5:0±íÊ¾PWM»¥²¹Ä£Ê½ËÀÇø½ûÖ¹
	//bit4:0±íÊ¾PWMÄ£Ê½¶ÀÁ¢
	//bit3:0±íÊ¾T16Nx¡ª¡ªOUT1Êä³ö¼«ÐÔÑ¡ÔñÎ»ÎªÕý¼«ÐÔ
	//bit2:0±íÊ¾T16Nx¡ª¡ªOUT0Êä³ö¼«ÐÔÑ¡ÔñÎ»ÎªÕý¼«ÐÔ
	//bit1:0Êä³ö¶Ë¿Ú1½ûÖ¹   1±íÊ¾Êä³ö¶Ë¿Ú1Ê¹ÄÜ
	//bit0:1Êä³ö¶Ë¿Ú0Ê¹ÄÜ
//    T16N3->PREMAT = 16; //1:2
	T16N3->PREMAT = 2; //1:2
	T16N3->MAT0 = 1000 - 1;
	T16N3->MAT1 = 1000 - 1;
	T16N3->MAT2 = 1000 - 1;
	T16N3->MAT3 = 1000 - 1;	
	
	T16N0->CON0 = 0x000088C1; //pwm mode enable
	// bit15-14:10±íÊ¾T16N_CNT0Çå0²¢ÖØÐÂ¼ÆÊý£¬²úÉúÖÐ¶Ï£¬Æ¥ÅäT16N¡ª¡ªMAT3
	//bit13-12:00±íÊ¾T16N_CNT0Æ¥ÅäT16N_MAT2ºóµÄ¹¤×÷Ä£Ê½Îª¼ÌÐø¼ÆËã£¬²»²úÉúÖÐ¶Ï
	//bit13-12:10±íÊ¾ T16N_CNT1Æ¥ÅäT16N¡ª¡ªMAT2µÄ¹¤×÷Ä£Ê½Çå0²¢ÖØÐÂ¼ÆÊý£¬²úÉúÖÐ¶Ï
	//bit11-10:10±íÊ¾T16N_CNT0Çå0²¢ÖØÐÂ¼ÆÊý£¬²úÉúÖÐ¶Ï£¬Æ¥ÅäT16N¡ª¡ªMAT1
	//bit11-10:00±íÊ¾T16N_CNT0Æ¥ÅäT16N¡ª¡ªMAT1¼ÌÐø¼ÆÊý£¬²»²úÉúÖÐ¶Ï
	//bit9-8:00±íÊ¾T16N_CNT0Æ¥ÅäT16N¡ª¡ªMAT0¼ÌÐø¼ÆÊý£¬²»²úÉúÖÐ¶Ï
	//bit9-8:10 T16N_CNT0Æ¥ÅäT16N¡ª¡ªMAT0£¬¹¤×÷Ä£Ê½Çå0²¢ÖØÐÂ¼ÆÊý£¬²úÉúÖÐ¶Ï
	//bit7-6:11±íÊ¾µ÷ÖÆÄ£Ê½
	//bit5-4:00±íÊ¾ÉÏÉýÑØ¼ÆÊý
	//bit3:0±íÊ¾Òì²½¼ÆÊý
	//bit2-bit1£º00±íÊ¾¼ÆÊýÊ±ÖÓÔ´Ñ¡ÔñÎªÄÚ²¿Ê±ÖÓ
	//bit0:1±íÊ¾T16NÊ¹ÄÜ
	T16N3->CON0 = 0x000088C1; //pwm mode enable
	
//	T16N0->CON2 = 0x00009903; //pwm config
//	T16N0->PREMAT = 2; //1:2   Ô¤·ÖÆµÆ÷¼ÆÊý±ÈÀý¼Ä´æÆ÷
//	T16N0->MAT0 = 1000 - 1;    //¼ÆÊýÆ¥Åä¼Ä´æÆ÷
//	T16N0->MAT1 = 1000 - 1;    //¼ÆÊýÆ¥Åä¼Ä´æÆ÷
//	T16N0->MAT2 = 1000 - 1;    //¼ÆÊýÆ¥Åä¼Ä´æÆ÷
//	T16N0->MAT3 = 1000 - 1;    //¼ÆÊýÆ¥Åä¼Ä´æÆ÷

//	T16N3->CON2 = 0x00009903; //pwm config
//	T16N3->PREMAT = 2; //1:2
//	T16N3->MAT0 = 1000 - 1;
//	T16N3->MAT1 = 1000 - 1;
//	T16N3->MAT2 = 1000 - 1;
//	T16N3->MAT3 = 1000 - 1;	
//	
//	T16N0->CON0 = 0x000088C1; //pwm mode enable
//	T16N3->CON0 = 0x000088C1; //pwm mode enable
	
	//T16N0->PTR = 0x00000004;  //adc sample
}

void close_pwm(void)
{
	T16N0->MAT0 = 0;
	T16N0->MAT2 = 0;
	T16N3->MAT0 = 0;
	T16N3->MAT2 = 0;
}

void open_pwm(void)
{
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_THR_DEBLOCK))
	{
		T16N0->MAT0 = 100;
		T16N0->MAT2 = 100;
		T16N3->MAT0 = 100;
		T16N3->MAT2 = 100;
	}
	else
	{
        T16N0->MAT0 = gvst_ThisBird.MotorValue_pwm[0];
		T16N0->MAT2 = gvst_ThisBird.MotorValue_pwm[1];
		T16N3->MAT0 = gvst_ThisBird.MotorValue_pwm[2];
		T16N3->MAT2 = gvst_ThisBird.MotorValue_pwm[3];
	}
}

//I2C
//I2C,mpu6881 gyro&acc
/*
static	void	I2C_delay(void){int i=20;while(i--);}
static	uint8	I2C_Start(void) 
{ 
	SDA_OUT;
	SDA_H;
	SCL_H;
	SDA_IN;
	I2C_delay();
	if(!SDA_read)return 0; //SDAÏßÎªµÍµçÆ½Ôò×ÜÏßÃ¦,ÍË³ö
	SDA_OUT;
	SDA_L; 
	SDA_IN;
	I2C_delay();
	if(SDA_read) return 0; //SDAÏßÎª¸ßµçÆ½Ôò×ÜÏß³ö´í,ÍË³ö 
	SDA_OUT;
	SDA_L; 
	I2C_delay(); 
	return 1; 
} 
static	void	I2C_Stop(void) 
{ 
	SDA_OUT;
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
	SDA_OUT;
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
	SDA_OUT;
	SCL_L; 
	I2C_delay(); 
	SDA_H; 
	I2C_delay(); 
	SCL_H; 
	I2C_delay(); 
	SCL_L; 
	I2C_delay(); 
} 
static	uint8	I2C_WaitAck(void)   //·µ»ØÎª:=1ÓÐACK,=0ÎÞACK 
{ 
	SDA_OUT;
	SCL_L; 
	I2C_delay(); 
	SDA_H; 
	I2C_delay(); 
	SCL_H; 
	SDA_IN;
	I2C_delay(); 
	if(SDA_read) 
	{ 
		SCL_L; 
		return 0; 
	} 
	SCL_L; 
	return 1; 
} 
static	void	I2C_SendByte(uint8 SendByte) //Êý¾Ý´Ó¸ßÎ»µ½µÍÎ»// 
{ 
	uint8 i=8; 
	SDA_OUT;
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
static	uint8	I2C_ReceiveByte(void)  //Êý¾Ý´Ó¸ßÎ»µ½µÍÎ»// 
{  
	uint8 i=8; 
	uint8 ReceiveByte=0; 

	SDA_OUT;
	SDA_H; 
	SDA_IN;
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
}*/
/******************************************************************************
 * @brief    i2c read function
 *           
 * @note
 *
 * @param	 addr
 *			 reg
 *			 len
 *			 data
 * @retval	 0 if success else failed
 *
 * @version  1.0
 * @date     2016-02-14
 * @author   sundy
 ******************************************************************************/
int8_t hr8p506_i2c_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	uint16_t i;
	uint16_t timer;
	
	I2C->CON.RST  = 1; //reset
	I2C->CON.Word  = 0x0000078D | addr << 17; //buard 375kbps, open drain
	I2C->MODE.Word = 0x00000000; //1 byte ack
	I2C->IF.Word  = 0xFFFFFFFF; //clear all int flag
	I2C->IE.Word = 0x00000000;
	
	I2C0_WRITE();
	I2C0_START();
	timer = 0;
	while (!I2C->IF.SR) {
		timer++;
		if (timer > 3000) {
			I2C0_STOP();
//			I2C->CON.RST  = 1; //reset
			return -1;
		}
	}
	I2C->IF.SR = 1;
	
	I2C->TBW.Byte[0] = reg;
	timer = 0;
	while (!I2C->IF.TB) {
		timer++;
		if (timer > 3000) {
			I2C0_STOP();
			return -1;
		}
	}
	I2C->IF.TB = 1;
	
	I2C0_READ();
	I2C0_START();
	timer = 0;
	while (!I2C->IF.SR) {
		timer++;
		if (timer > 3000) {
			I2C0_STOP();
			return -1;
		}
	}
	I2C->IF.SR = 1;
	
	I2C->MODE.Word = 0x00000000; //1 byte ack
	timer = 0;
	for (i = 0; i < len-1; i++) {
		I2C0_RECEIVE();
		while (!I2C->IF.RB) {
			timer++;
			if (timer > 3000) {
				I2C0_STOP();
				return -1;
			}
		}
		I2C->IF.RB = 1;
		data[i] = I2C->RBR.Byte[0];
	}
	
	I2C->MODE.Word = 0x00000002; //1 byte noack
	I2C0_RECEIVE();
	timer = 0;
	while (!I2C->IF.RB) {
		timer++;
		if (timer > 3000) {
			I2C0_STOP();
			return -1;
		}
	}
	I2C->IF.RB = 1;
	data[i] = I2C->RBR.Byte[0];
	
	I2C0_STOP();
	timer = 0;
	while (!I2C->IF.SP) {
		timer++;
		if (timer > 3000) {
			I2C0_STOP();
			return -1;
		}
	}
	I2C->IF.SP = 1;
	
	return 0;
}

/******************************************************************************
 * @brief    i2c write function
 *           
 * @note
 *
 * @param	 addr
 *			 reg
 *			 len
 *			 data
 * @retval	 0 if success else failed
 *
 * @version  1.0
 * @date     2016-02-14
 * @author   sundy
 ******************************************************************************/
int8_t hr8p506_i2c_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t data)
{
	uint8_t i;
	uint16_t timer;

	I2C->CON.RST  = 1; //reset
	//I2C->CON.Word  = 0x0000078D | addr << 17; //buard 375kbps, open drain
	I2C->CON.Word  = 0x0000078D | addr << 17; //buard 375kbps
	I2C->MODE.Word = 0x00000000; //1 byte ack
	I2C->IF.Word  = 0xFFFFFFFF; //clear all int flag
	I2C->IE.Word = 0x00000000;
	
	I2C0_WRITE();
	I2C0_START();
	timer = 0;
	while (!I2C->IF.SR) {
		timer++;
		if (timer > 3000) {
			I2C0_STOP();
			return -1;
		}
	}
	I2C->IF.SR = 1;
	
	I2C->TBW.Byte[0] = reg;
	timer = 0;
	while (!I2C->IF.TB) {
		timer++;
		if (timer > 3000) {
			I2C0_STOP();
			return -1;
		}
	}
	I2C->IF.TB = 1;
	
	for (i = 0; i < len; i++) {
		I2C->TBW.Byte[0] = data;
		timer = 0;
		while (!I2C->IF.TB) {
			timer++;
			if (timer > 3000) {
				I2C0_STOP();
				return -1;
			}
		}
		I2C->IF.TB = 1;
	}
	
//	timer = 0;
//	while (!I2C->IF.TB) {
//		timer++;
//		if (timer > 30000) {
//			I2C0_STOP();
//			return -1;
//		}
//	}
//	I2C->IF.TB = 1;
	
	timer = 0;
	while ((I2C->STA.Word & 0x00000F00) != 0x00000F00) {
		timer++;
		if (timer > 6000) {
			I2C0_STOP();
			return -1;
		}	
	}
	
	I2C0_STOP();
	timer = 0;
	while (!I2C->IF.SP) {
		timer++;
		if (timer > 3000) {
			I2C0_STOP();
			return -1;
		}
	}
	I2C->IF.SP = 1;
	
	return 0;
}
////20618
//int8 Mpu6881Initial(void)
//{
//	uint8 id[1]={0};
//	//uint8_t data[7]={0x80,0x03,0x03,0x18,0x10,0x00};
////gyro init,16bit for (+-)2000der/s
//	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x7f,1, 0x00)==-1)			  //bank 0
//		return 1;
//	do {
//		hr8p506_i2c_read(MPU6050_ADDRESS,0x00,1,id);
//	} while (id[0]!= 0xA0);
//	DelayMs(50);
//	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x06,1, 0x80)==-1)//reset
//		return 1;
//	DelayMs(50);
//	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x06,1, 0x0b)==-1)//wakeup, disable temperature sensor
//		return 1;
//	DelayMs(50);
//	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x7f,1, 0x20)==-1)			  //bank 2
//		return 1;
//	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x00,1, 0x00)==-1)//1125/(1+GYRO_SMPLRT_DIV[7:0])=ODR
//		return 1;
//	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x01,1, 0x2f)==-1)//50Hz, +-2000degree  0x27
//		return 1;
//	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x10,1, 0x00)==-1)
//	  return 1;
//	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x11,1, 0x00)==-1)//1125/(1+acc_SMPLRT_DIV[11:0])=ODR
//		return 1;
//	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x14,1, 0x35)==-1)//50Hz,+-8g   35
//		return 1;
//	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x7f,1, 0x00)==-1)//bank 0
//		return 1;
//	return 0;
////	hr8p506_i2c_write(MPU6050_ADDRESS, 0x0f,1, 0x12);//50us pulse, bypass i2c_master interface
////	hr8p506_i2c_write(MPU6050_ADDRESS, 0x11,1, 0x01);//raw data ready
////	hr8p506_i2c_write(MPU6050_ADDRESS, 0x10,1, 0x04);//pll rdy enable
//}

//static	void	ReadAcc(void* dat)/*¶ÁÈ¡¼ÓËÙ¶È¼ÆÊýÖµ*/
//{
//	static uint8_t mpu6050_acc[6];
//	__AccDat*	__dat;
//	if(dat==NULL)	return	;

//	__dat=(__AccDat*)dat;
//	if(hr8p506_i2c_read(MPU6050_ADDRESS,0x2d,6,mpu6050_acc)==0)
//	{
//		__dat->pitch=(int16)((mpu6050_acc[0]<<8)+mpu6050_acc[1]);//acc_x
//		__dat->roll=(int16)((mpu6050_acc[2]<<8)+mpu6050_acc[3]);//acc_y
//		__dat->yaw=(int16)((mpu6050_acc[4]<<8)+mpu6050_acc[5]);//acc_z
//	}
//}

//static	void	ReadGyro(void* dat)//¶ÁÈ¡ÍÓÂÝÒÇÊýÖµ
//{
//	static uint8_t mpu6050_gyro[6];
//	__GyroDat*	__dat;
//	if(dat==NULL)	return	;

//	__dat=(__GyroDat*)dat;
//	if(hr8p506_i2c_read(MPU6050_ADDRESS,0x33,6,mpu6050_gyro)==0)
//	{
//		__dat->pitch=(int16)((mpu6050_gyro[0]<<8)+mpu6050_gyro[1]);
//		__dat->roll=(int16)((mpu6050_gyro[2]<<8)+mpu6050_gyro[3]);
//		__dat->yaw=(int16)((mpu6050_gyro[4]<<8)+mpu6050_gyro[5]);
//	}
//}
//6050
int8 Mpu6881Initial(void)
{
	//uint8_t data[7]={0x80,0x03,0x03,0x18,0x10,0x00};
//gyro init,16bit for (+-)2000der/s
	gven_LedFlashKind=EV_LED_1;
	DelayMs(5);
	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x6B,1, 0x80)==-1)//PWR_MGMT_1	  -- DEVICE_RESET 1
	  return 1;
	DelayMs(200);
	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x6B,1, 0x03)==-1)	  //PWR_MGMT_1	  -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
		return 1;
	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x19,1, 0x00)==-1)
	  return 1;
	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x1A,1, 0x03)==-1)//µÍÍ¨42hz£¬300hz¿ØÖÆÖÜÆÚ¿ÉÒÔ£¬Ò²¶¶  gyro filter
	  return 1;
	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x1B,1, 0x18)==-1)
	  return 1;
	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x1C,1, 0x10)==-1)//ACCEL_CONFIG	-- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0	//note something is wrong in the spec.
	  return 1;
	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x1D,1, 0x05)==-1)   //acc filter
	  return 1;
	if(hr8p506_i2c_write(MPU6050_ADDRESS, 0x37,1, 0x00)==-1)
	  return 1;
	DelayMs(500);
	gven_LedFlashKind=EV_LED_2;
	return 0;
}

static	void	ReadAcc(void* dat)/*¶ÁÈ¡¼ÓËÙ¶È¼ÆÊýÖµ*/
{
	static uint8_t mpu6050_acc[6];
	__AccDat*	__dat;
	if(dat==NULL)	return	;

	__dat=(__AccDat*)dat;
	if(hr8p506_i2c_read(MPU6050_ADDRESS,0x3B,6,mpu6050_acc)==0)
	{
		__dat->pitch=(int16)((mpu6050_acc[0]<<8)+mpu6050_acc[1]);//acc_x
	  __dat->roll=(int16)((mpu6050_acc[2]<<8)+mpu6050_acc[3]);//acc_y
	  __dat->yaw=(int16)((mpu6050_acc[4]<<8)+mpu6050_acc[5]);//acc_z
	}
}

static	void	ReadGyro(void* dat)//¶ÁÈ¡ÍÓÂÝÒÇÊýÖµ
{
	static uint8_t mpu6050_gyro[6];
	__GyroDat*	__dat;
	if(dat==NULL)	return	;

	__dat=(__GyroDat*)dat;
	if(hr8p506_i2c_read(MPU6050_ADDRESS,0x43,6,mpu6050_gyro)==0)
	{
		__dat->pitch=(int16)((mpu6050_gyro[0]<<8)+mpu6050_gyro[1]);
	  __dat->roll=(int16)((mpu6050_gyro[2]<<8)+mpu6050_gyro[3]);
	  __dat->yaw=(int16)((mpu6050_gyro[4]<<8)+mpu6050_gyro[5]);
	}
}
uint32	SystemGetMirco(void)    //¾§ÕñÆµÂÊÎª36.8MHz
{//·µ»Øµ±Ç°Î¢ÃëÊý£¬¼´1/1000000s
	return	gvu32_SystemRunTime*1000+(SYSTICK->RVR - SYSTICK->CVR)/48;
}

void  send_tx(uint8* tx_data)
{
	static uint8 sendtx_count=0;
//	uint8 k;
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_SENDTX))
	{
		sendtx_count++;
		if(sendtx_count<16)
			return;
			hw2000_tx_data(tx_data,MATCH_FRE);
		if(sendtx_count>=66)
//		if(0)
		{
			
			MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_DMMODE);  //ÒÑ¶ÔÉÏÂë
			sendtx_count=0;
			_hw2000_irq_request = 0;
			if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DOCK))   //Ò¡»ÎÖØÐÂ¼ÇID
				quad_idf=quad_id;  //¼Ç×¡ID
			MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DOCK);
			MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DROP);
			
			data1a=0x0300+(((uint8)(tx_data[3]+tx_data[4]+tx_data[5]+tx_data[6]))&0x7f);
			data2a=0xf800+tx_data[6];
			hw2000_init_250k(data1a,data2a);
			hw2000_write_reg(0x36, 0x0080);
			hw2000_write_reg(0x3d, 0x0008);
			hw2000_write_reg(0x21, 0x0000);
			hw2000_rx_enable(channel[channel_num]);
//			gyro_acc_datasave();
			
			MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_SENDTX);   //·¢ËÍ½áÊøºóÇåÁã
			realthrottlesave=540;  //ÓÍÃÅ³õÊ¼Öµ
		}
	}
}
static	int	ReadTel(void)
{
  static uint8 signal_count1=0;
	static uint8 tel_timer=0;
	static uint16 signal_count=0;
	
	uint8_t i,j;//,k;
	if(START_MODE==1)
		return TRUE;
	if(link_select==LINK_USART)
	{
		for(i=0;i<10;i++)
		  gvst_ThisBird.TelDat[i]=gvu8_U1ReceiveBuf[i+1];
		lost_timer++;
		if(lost_timer>=500)
		{
			MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK);
		}
    signal_display=14;
		return TRUE;
	}
	
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_SENDTX))
	{
		send_tx(dataTel);
		return 0;
	}

//ÐÅºÅÇ¿¶ÈÅÐ¶Ï  200msÓÐ¶àÉÙÓÐÐ§ÕêÊý  
//  signal_count++;
//	if(signal_count>=300)
//	{
//		signal_count=0;
//		signal_display1=signal_strength;
//		signal_strength=0;
//		if(signal_display1<8)
//		{
//			signal_display=2;
//		}
//		if(signal_display1>=12)
//		{
//			if(signal_count1<250)
//			  signal_count1++;
//		}
//		else
//			signal_count1=0;
//		
//		if(signal_count1>=8)
//			signal_display=14;
//		
//		if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DOCK) || hold_high_statue==HOLD_HIGH_STOP)
//		{
//			signal_count1=12;
//			signal_display=14;
//		}
//	}

	if (_hw2000_irq_request == 0)
	{
	  if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DOCK))
		{
			if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK))  //ÒÑ¶ÔÉÏÂíÎ´µôÏß  Õý³£ÌøÆµ
			{
				carrier_time_count++;
				radio_carrier_detect(0);   //ÌøÆµ
			}
			else         //¶ÔÉÏÂíÒÑµôÏß
			{
				#if APEX
					{
							tel_timer++;
							if(tel_timer==1)
							{
								MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_DMMODE);
					
								hw2000_write_reg(0x23, data1a);
								hw2000_write_reg(0x29, data2a);
								radio_carrier_detect(2);   //ÌøÆµ
							}
							if(tel_timer==6)
							{
									MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_DMMODE);
					
									hw2000_write_reg(0x23, 0x031B);
									hw2000_write_reg(0x29, 0x3800);
									hw2000_rx_enable(MATCH_FRE);
							}
							if(tel_timer>=10)
							tel_timer=0;
					}
					#else
					{
							tel_timer++;
							if(tel_timer==10)
							{
								MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_DMMODE);
					
								hw2000_write_reg(0x23, data1a);
								hw2000_write_reg(0x29, data2a);
								radio_carrier_detect(2);   //ÌøÆµ
							}
							if(tel_timer==20)
							{
									MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_DMMODE);
					
									hw2000_write_reg(0x23, 0x031B);
									hw2000_write_reg(0x29, 0x3800);
									hw2000_rx_enable(MATCH_FRE);
							}
							if(tel_timer>=20)
							tel_timer=0;
					}
					#endif
			}
		}
		else
			hw2000_rx_enable(MATCH_FRE);
		lost_timer++;
		if(lost_timer>=500)
		{
			
			MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK);
		}
		return	FALSE;
	}
	else
	{
		tel_timer=0;
		_hw2000_irq_request = 0;
		carrier_time_count=0;
//		if (hw2000_rx_data(dataTel) == 0)
		{
			if (dataTel[1] == 0xF0 && MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK))
			{
				  indexcount=dataTel[2];
					hw2000_write_reg(0x21, 0x0000);  //½ûÖ¹·¢ËÍ½ÓÊÕÄ£Ê½
				
					quad_id = dataTel[3] << 24 | dataTel[4] << 16 | dataTel[5] << 8 | dataTel[6];   //ËÄ×Ö½ÚID
				if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DOCK))
				{
					if(quad_id!=quad_idf)
					  return 0;
				}
				
				
				i = (dataTel[3] + dataTel[4] + dataTel[5] + dataTel[6]) & 0x07;
				i *= CHANNEL_MAX;
				
				for (j = 0; j < REPEAT_MAX; j++) {
					channel[j] = _hw2000_channel_table[i+j];
				}
				
//				channel[0]=dataTel[6]&0x7f;
//			if(channel[0]>=80)
//			  channel[0]=channel[0]-80;
//		  channel[1]=(channel[0]+((dataTel[6]>>4)&0x0f)+10)&0x3f;
//			channel[2]=(channel[1]+(dataTel[5]&0x0f)+10)&0x3f;
//			channel[3]=(channel[2]+((dataTel[5]>>4)&0x0f)+10)&0x3f;
//			channel[4]=(channel[3]+(dataTel[4]&0x0f)+10)&0x3f;
				
				  quad_unlock = 0;
					channel_num = 0;
					life = 0;
				disable_irq();
				hw2000_write_reg(0x21, 0x0000);
				hw2000_write_reg(0x23, 0x431B); //Soft reset
				hw2000_write_reg(0x23, 0x031B); 
				hw2000_write_reg(0x3C, 0x1000); //ACK disable
				enable_irq();
				tx_rx_delayms=0;
//				while(tx_rx_delayms<=60);
				
				MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_SENDTX);
				send_tx(dataTel);
				
			}
			else
			{

			}
		}
	}
	
	return	TRUE;
}
void  delay_ms(int16 ms)
{
	uint16 us;
	//for(us=0;us<=48000;us++);
	ms--;
	if(ms<=0)
		return;
	else
		for(us=0;us<=48000;us++);
}
void	SystemHalInit(__ThisHal *hal)
{
	//SystemInit();

	//hal->SystemPeripheralEnableInit=SystemPeripheralEnableInit;
	//hal->SystemInterruputInit=SystemInterruputInit;
	hal->SystemGPIOInit=SystemGPIOInit;
	//hal->SystemMsicPeripheralInit=SystemMsicPeripheralInit;

	//hal->Iwdg_Init=Iwdg_Init;
	//hal->Iwdg_Feed=Iwdg_Feed;
	//hal->SystemRestart=SystemRestart;

	//hal->AircraftInit=AircraftInit;
	//hal->ReadADValue=ReadADValue;

	//hal->SetIO=SetIO;
	//hal->ReadIO=ReadIO;
	hal->ReadTel=ReadTel;
	hal->ReadAcc=ReadAcc;
	hal->ReadGyro=ReadGyro;
	
	hal->SystemGetMirco=SystemGetMirco;
}



