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
#ifndef __HR8P506_TIMER_H
#define __HR8P506_TIMER_H
#include <stdint.h>
#include "hr8p506.h"

/******************************************************************************
 * @brief    delay millisecond
 *           use timer t16n1
 * @note
 *
 * @param	 ms	millisecond 0~65536
 * @retval	 None
 ******************************************************************************/
void hr8p506_delay_ms(uint16_t ms);

/******************************************************************************
 * @brief    wdt initialize
 *           
 * @note
 *
 * @param	 interval 32-bit wide
 * @retval	 None
 ******************************************************************************/
void hr8p506_wdt_init(uint32_t interval);

/******************************************************************************
 * @brief    wdt clear
 *           
 * @note
 *
 * @param	 enable
 * @retval	 None
 ******************************************************************************/
void hr8p506_wdt_clear(uint8_t enable);
	
#endif

