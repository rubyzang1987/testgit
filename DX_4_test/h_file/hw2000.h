/******************************************************************************
 * @file     hw2000.h
 * @brief    hw2000 2.4GHz wireless conmunication function
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
#ifndef __HR8P506_HW2000_H
#define __HR8P506_HW2000_H

#include <stdint.h>

/******************************************************************************
 * @brief    hw2000_write_reg
 *           
 * @note
 *
 * @param	 addr reg address
			 data
 * @retval	 None
 ******************************************************************************/
void hw2000_write_reg(uint8_t addr, uint16_t data);

/******************************************************************************
 * @brief    hw2000_read_reg
 *           
 * @note
 *
 * @param	 addr reg address
 *
 * @retval	 data
 ******************************************************************************/
uint16_t hw2000_read_reg(uint8_t addr);

/******************************************************************************
 * @brief    hw2000_write_fifo
 *           
 * @note
 *
 * @param	 addr reg address
			 data
			 length
 ******************************************************************************/
void hw2000_write_fifo(uint8_t addr, uint8_t *data, uint8_t length);

/******************************************************************************
 * @brief    hw2000_read_fifo
 *           
 * @note
 *
 * @param	 addr reg address
 *		     data
 *           length
 * @retval	 None
 ******************************************************************************/
void hw2000_read_fifo(uint8_t addr, uint8_t *data, uint8_t length);

/******************************************************************************
 * @brief    hw2000 port init
 *           
 * @note
 *
 * @param
 * @retval	 None
 ******************************************************************************/
void hw2000_port_init(void);

/******************************************************************************
 * @brief    hw2000_init_1m
 *           12M osc, maxim output power(default)
 * @note
 *
 * @param
 * @retval	 None
 ******************************************************************************/
void hw2000_init_1m(void);

/******************************************************************************
 * @brief    hw2000_init_250k
 *           12M osc, maxim output power(default)
 * @note
 *
 * @param
 * @retval	 None
 ******************************************************************************/
void hw2000_init_250k(uint16 data1,uint16 data2);

/******************************************************************************
 * @brief    hw2000_tx_data
 *           
 * @note
 *
 * @param	 data		data[0] indicate the length
 *			 channel    0~81 valid
 * @retval	 0 if success -1 failed
 ******************************************************************************/
int8_t hw2000_tx_data(uint8_t *data, uint8_t channel);

/******************************************************************************
 * @brief    hw2000_rx_enable
 *           
 * @note
 *
 * @param	 channel    0~81 valid
 * @retval	 0 if success -1 failed
 ******************************************************************************/
void hw2000_rx_enable(uint8_t channel);

/******************************************************************************
 * @brief    hw2000_rx_data
 *           
 * @note
 *
 * @param	 data
 * @retval	 0 if success -1 failed
 ******************************************************************************/
int8_t hw2000_rx_data(uint8_t *data);

/******************************************************************************
 * @brief    hw2000_power_test
 *           
 * @note
 *
 * @param	 
 * @retval	 None
 ******************************************************************************/
void hw2000_power_test(void);

void radio_carrier_detect(int8 rx_statue);

#endif

