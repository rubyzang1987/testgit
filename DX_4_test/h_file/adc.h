/******************************************************************************
 * @file     adc.h
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
#ifndef __HR8P506_ADC_H
#define __HR8P506_ADC_H
#include <stdint.h>
#include "fly.h"

/******************************************************************************
 * @brief    hr8p506_adc_init
 *           
 * @note
 *
 * @param	 
 * @retval	 None
 ******************************************************************************/
void hr8p506_adc_init(void);

/******************************************************************************
 * @brief    hr8p506_adc_sample
 *           
 * @note
 *
 * @param	 ch channel num
 * @retval	 adc value
 ******************************************************************************/
uint16_t hr8p506_adc_sample(uint8_t ch);

/******************************************************************************
 * @brief    hr8p506_adc_sample
 *           
 * @note
 *
 * @param	 
 * @retval	 adc value
 ******************************************************************************/
uint16_t hr8p506_adc_battery(void);

/******************************************************************************
 * @brief    hr8p506_adc_update
 *           
 * @note
 *
 * @param	 fly
 *			 ch
 * @retval	 None
 ******************************************************************************/
void hr8p506_adc_update(fly_object_t * fly);

#endif