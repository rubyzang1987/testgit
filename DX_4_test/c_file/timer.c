/******************************************************************************
 * @file     timer.c
 * @brief    hr8p506 timer function
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

/******************************************************************************
 * @brief    delay millisecond
 *           use timer t16n1
 * @note
 *
 * @param	 ms	millisecond 0~65536
 * @retval	 None
 *
 * @version  1.0
 * @date     2015-12-02
 * @author   sundy
 ******************************************************************************/
void hr8p506_delay_ms(uint16_t ms)
{
	uint16_t i;
	
	i=4800*ms;
  while(i--);
	
	
//	SCU->SCU_PROT.Word = 0x55AA6996; //Disable SCU protection
//	SCU->CLKEN_PERI.CLKEN_T16N1 = 1; //enable t16n1 clk
//	SCU->SCU_PROT.Word = 0x00000000; //Enable SCU protection
//	
//	T16N1->CNT0 = 0x00000000; //clear
//	T16N1->IF = 0x00000001; //clear
//	T16N1->PREMAT = 47; //1MHz
//	T16N1->MAT0 = 999; //1ms match
//	T16N1->CON0 = 0x00000201;  //clear, int again for mat0
//	
//	for (i = 0; i < ms; i++) {
//		while (!(T16N1->IF & 0x00000001));
//		T16N1->IF = 0x00000001; //clear
//	}
//	T16N1->CON0 = 0x00000200;  //disable
}

/******************************************************************************
 * @brief    wdt initialize
 *           
 * @note
 *
 * @param	 interval 32-bit wide
 * @retval	 None
 *
 * @version  1.0
 * @date     2015-12-02
 * @author   sundy
 ******************************************************************************/
/*
void hr8p506_wdt_init(uint32_t interval)
{
	WDT->LOCK = 0x1ACCE551;
	WDT->LOAD = interval;
	WDT->CON = 0x05;
}
*/

/******************************************************************************
 * @brief    wdt clear
 *           
 * @note
 *
 * @param	 enable
 * @retval	 None
 *
 * @version  1.0
 * @date     2015-12-02
 * @author   sundy
 ******************************************************************************/
/*
void hr8p506_wdt_clear(uint8_t enable)
{
	if (enable) {
		WDT->INTCLR = 0xDEADBEEF;
	}
}
*/


