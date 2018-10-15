/******************************************************************************
 * @file     adc.c
 * @brief    hr8p506 adc function
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
#include "config.h"
//#include "fly.h"

/******************************************************************************
 * @brief    hr8p506_adc_init
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
void hr8p506_adc_init(void)
{	
	SCU->SCU_PROT.Word = 0x55AA6996; //Disable SCU protection
	SCU->CLKEN_PERI.CLKEN_ADC = 1; //enable adc clk
	SCU->SCU_PROT.Word = 0x00000000; //Enable SCU protection
	
	GPIO->PA_INEB = 0x00000040;
	GPIO->PA_DIRBSR = 0x00000040;
	GPIO->PB_INEB = 0x000001E0;
	GPIO->PA_DIRBSR = 0x000001E0;
	
	ADC->VREF_CON.Word = 0x00003A1D;
	ADC->CON1.Word = 0x03005904;	//3MHz AD CLK / 15 = 200KHz²ÉÑùÂÊ
	ADC->CHS.VDD5_FLAG_EN = 1;
	ADC->CON0.EN = 1;
	ADC->IF.Word = 0x0000000F;
}

/******************************************************************************
 * @brief    hr8p506_adc_sample
 *           
 * @note
 *
 * @param	 ch channel num
 * @retval	 adc value
 *
 * @version  1.0
 * @date     2015-12-02
 * @author   sundy
 ******************************************************************************/
uint16_t hr8p506_adc_sample(uint8_t ch)
{
	uint16_t sample = 0;
	
	ADC->CHS.CHS = ch;
	ADC->CON0.TRG = 1; 

	while (!ADC->IF.IF);
	sample = ADC->DR.Word;
	ADC->IF.Word = 0x0000000F;
	
	return sample;
}

/******************************************************************************
 * @brief    hr8p506_adc_sample
 *           
 * @note
 *
 * @param	 
 * @retval	 adc value
 *
 * @version  1.0
 * @date     2015-12-02
 * @author   sundy
 ******************************************************************************/
uint16_t hr8p506_adc_battery(void)
{
	return (4095 - hr8p506_adc_sample(BAT_ADC_CHAN));
}

/******************************************************************************
 * @brief    hr8p506_adc_update
 *           
 * @note
 *
 * @param	 fly
 *			 ch
 * @retval	 None
 *
 * @version  1.0
 * @date     2015-12-02
 * @author   sundy
 ******************************************************************************/
void hr8p506_adc_update(fly_object_t * fly)
{	
	static uint8_t i = 0;
	static uint8_t i0 = 0, i1 = 0, i2 = 0, i3 = 0, i4 = 0;
	static int32_t avg_vbat = 0, avg_check1 = 0, avg_check2 = 0, avg_check3 = 0, avg_check4 = 0;
	int16_t adc;
	
	i++;
	i %= 5;
	
	ADC->IF.Word = 0x0000000F;
	while (!ADC->IF.IF);
	adc = ADC->DR.Word;
	
	if (adc != 4095 && adc != 0) {
		switch (i) {
		case 0:
			ADC->CHS.CHS = MOTOR_CHECK1;
			i0++;
			if (i0 == 50) {
				i0 = 0;
				avg_vbat += adc;
				
				avg_vbat /= 50;
				fly->vbat = 4095 - avg_vbat;
				
				avg_vbat = 0;
			} else {
				avg_vbat += adc;
			}
			break;
		case 1:
			ADC->CHS.CHS = MOTOR_CHECK2;
			i1++;
			if (i1 == 50) {
				i1 = 0;
				avg_check1 += adc;
				
				avg_check1 /= 50;
				fly->check1 = 4095 - avg_check1;
				
				avg_check1 = 0;
			} else {
				avg_check1 += adc;
			}
			break;
		case 2:
			ADC->CHS.CHS = MOTOR_CHECK3;
			i2++;
			if (i2 == 50) {
				i2 = 0;
				avg_check2 += adc;
				
				avg_check2 /= 50;
				fly->check2 = 4095 - avg_check2;
				
				avg_check2 = 0;
			} else {
				avg_check2 += adc;
			}
			break;
		case 3:
			ADC->CHS.CHS = MOTOR_CHECK4;
			i3++;
			if (i3 == 50) {
				i3 = 0;
				avg_check3 += adc;
				
				avg_check3 /= 50;
				fly->check3 = 4095 - avg_check3;
				
				avg_check3 = 0;
			} else {
				avg_check3 += adc;
			}
			break;
		case 4:
			ADC->CHS.CHS = BAT_ADC_CHAN;
			i4++;
			if (i4 == 50) {
				i4 = 0;
				avg_check4 += adc;
				
				avg_check4 /= 50;
				fly->check4 = 4095 - avg_check4;
				
				avg_check4 = 0;
			} else {
				avg_check4 += adc;
			}
			break;
		default:
			break;
		}
	}
}






