/******************************************************************************
 * @file     hw2000.c
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
#include <stdint.h>
#include "hr8p506.h"
#include "config.h"
#include "spi.h"
#include "timer.h"

volatile uint8_t _hw2000_state; //0 idle 1 tx 2 rx
//volatile uint8_t _hw2000_irq_request;

/******************************************************************************
 * @brief    hw2000_write_reg
 *           
 * @note
 *
 * @param	 addr reg address
			 data
 * @retval	 None
 *
 * @version  1.0
 * @date     2015-12-03
 * @author   sundy
 ******************************************************************************/
void hw2000_write_reg(uint8_t addr, uint16_t data)
{	
	addr |= 0x80;
	
	SPI0->CON.Word = 0xC7000001; //disable spi rx
	GPIO->PA_DATABCR = 0x00004000; //CSN 0
	
	SPI0->TBW.Byte[0] = addr; //write addr
	while (!SPI0->IF.TBIF); //waiting for spi txb empty
	SPI0->TBW.Byte[0] = data >> 8;
	while (!SPI0->IF.TBIF); //waiting for spi txb empty
	SPI0->TBW.Byte[0] = data;
	while (!SPI0->STA.IDLE); //waiting for spi idle
	
	GPIO->PA_DATABSR = 0x00004000; //CSN 1
	SPI0->CON.Word = 0xC7000000; //disable spi
}

/******************************************************************************
 * @brief    hw2000_read_reg
 *           
 * @note
 *
 * @param	 addr reg address
 *
 * @retval	 data
 *
 * @version  1.0
 * @date     2015-12-03
 * @author   sundy
 ******************************************************************************/
uint16_t hw2000_read_reg(uint8_t addr)
{	
	uint8_t val_h, val_l;
	
	SPI0->CON.Word = 0xC7000009; //enable spi rx
	GPIO->PA_DATABCR = 0x00004000; //CSN 0

	SPI0->TBW.Byte[0] = addr; //write addr
	while (!SPI0->IF.RBIF); //waiting for spi rxb full
	val_h = SPI0->RBR.Byte[0];
	SPI0->TBW.Byte[0] = 0x00; 
	while (!SPI0->IF.RBIF); //waiting for spi rxb full
	val_h = SPI0->RBR.Byte[0];
	SPI0->TBW.Byte[0] = 0x00; 
	while (!SPI0->IF.RBIF); //waiting for spi rxb full
	val_l = SPI0->RBR.Byte[0];
	
	GPIO->PA_DATABSR = 0x00004000; //CSN 1
	SPI0->CON.Word = 0xC7000000; //disable spi
	
	return (val_h * 256 + val_l);
}

/******************************************************************************
 * @brief    hw2000_write_fifo
 *           
 * @note
 *
 * @param	 addr reg address
			 data
			 length
 * @retval	 None
 *
 * @version  1.0
 * @date     2015-12-03
 * @author   sundy
 ******************************************************************************/
void hw2000_write_fifo(uint8_t addr, uint8_t *data, uint8_t length)
{
	uint8_t i;
	
	addr |= 0x80;
	
	SPI0->CON.Word = 0xC7000001; //disable spi rx
	GPIO->PA_DATABCR = 0x00004000; //CSN 0
	
	SPI0->TBW.Byte[0] = addr; //write addr
	
	for (i = 0; i < length; i++) {
		while (!SPI0->IF.TBIF); //waiting for spi txb empty
		SPI0->TBW.Byte[0] = data[i]; //write  data
	}
	while (!SPI0->STA.IDLE); //waiting for spi idle
	
	GPIO->PA_DATABSR = 0x00004000; //CSN 1
	SPI0->CON.Word = 0xC7000000; //disable spi
}

/******************************************************************************
 * @brief    hw2000_read_fifo
 *           
 * @note
 *
 * @param	 addr reg address
 *		     data
 *           length
 * @retval	 None
 *
 * @version  1.0
 * @date     2015-12-03
 * @author   sundy
 ******************************************************************************/
void hw2000_read_fifo(uint8_t addr, uint8_t *data, uint8_t length)
{
	uint8_t i;
	
	SPI0->CON.Word = 0xC7000009; //enable spi rx
	GPIO->PA_DATABCR = 0x00004000; //CSN 0

	SPI0->TBW.Byte[0] = addr; //write addr
	while (!SPI0->IF.RBIF); //waiting for spi rxb full
	data[0] = SPI0->RBR.Byte[0];
	
	for (i = 0; i < length; i++) {
		SPI0->TBW.Byte[0] = 0x00; 
		while (!SPI0->IF.RBIF); //waiting for spi rxb full
		data[i] = SPI0->RBR.Byte[0];
	}
	
	GPIO->PA_DATABSR = 0x00004000; //CSN 1
	SPI0->CON.Word = 0xC7000000; //disable spi
}

/******************************************************************************
 * @brief    hw2000 port init
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
void hw2000_port_init(void)
{	
	GPIO->PA_DIRBCR = 0x00014000; //PA16 is CE, PA 14 is CSN
	GPIO->PA_DIRBSR = 0x00000800; //PA11 is IRQ
	
	GPIO->PA_DATABSR = 0x00004000; //CSN 1
	
	GPIO->PA_DATABCR = 0x00010000; //CE 0 for reset HW2000 reg to default value
	hr8p506_delay_ms(1);
	GPIO->PA_DATABSR = 0x00010000; //CE 1
	hr8p506_delay_ms(1);
	
	GPIOE->PINTSEL &= 0xFFFF0FFF; //PA11 selected
	GPIOE->PINTSEL |= 0x00001000;
	GPIOE->PINTCFG &= 0xFFFF0FFF; //rising edge for int		IRQ3上升沿中断
	
	GPIOE->PINTIE &= 0xFFFFF7FF;
	GPIOE->PINTIE |= 0x00000008;
		
	_hw2000_irq_request = 0;
	_hw2000_state = IDLE;
}

/******************************************************************************
 * @brief    hw2000_init_1m
 *           12M osc, maxim output power(default)
 * @note
 *
 * @param
 * @retval	 None
 *
 * @version  1.0
 * @date     2015-12-02
 * @author   sundy
 ******************************************************************************/
/*
void hw2000_init_1m(void)
{
    uint8_t i;
    
    uint16_t agcTab[18] = {0x0012, 0x0012, 0x0012, 0x0012, 0x0012, 0x0012,
                           0x0052, 0x0249, 0x0251, 0x0252, 0x0451, 0x0491,
                           0x04D1, 0x04D9, 0x06D8, 0x06E0, 0x06E1, 0x06E2
    };  
	
    hw2000_write_reg(0x4C, 0x55AA);
	for (i = 0; i < 18; i++) {
        hw2000_write_reg(0x50 + i, agcTab[i]);            
    }  
		 
    hw2000_write_reg(0x01, 0x4D55);
    hw2000_write_reg(0x02, 0x44CC);
    hw2000_write_reg(0x08, 0x7BC8);
    hw2000_write_reg(0x09, 0xCC00);
    hw2000_write_reg(0x28, 0x8401);
    hw2000_write_reg(0x2C, 0x918B); 

    hw2000_write_reg(0x1B, 0xE754);
    hw2000_write_reg(0x06, 0xB000);
    hw2000_write_reg(0x07, 0x54E0);
    hw2000_write_reg(0x1C, 0x51A0);
    hw2000_write_reg(0x19, 0x2084);
 
    hw2000_write_reg(0x20, 0x3000);
    hw2000_write_reg(0x2A, 0xC084);
    
//    hw2000_write_reg(0x3C, 0x1001); //ACK enable
//    hw2000_write_reg(0x23, 0x0300); //Repeat 3  
}
*/

/******************************************************************************
 * @brief    hw2000_init_250k
 *           12M osc, maxim output power(default)
 * @note
 *
 * @param
 * @retval	 None
 *
 * @version  1.0
 * @date     2015-12-02
 * @author   sundy
 ******************************************************************************/
void hw2000_init_250k(uint16 data1,uint16 data2)
{
    uint8_t i;
    
    uint16_t agcTab[18] = {0x0012, 0x0012, 0x0012, 0x0012, 0x0012, 0x0012,
                           0x0052, 0x0249, 0x0251, 0x0252, 0x0451, 0x0491,
                           0x04D1, 0x04D9, 0x06D8, 0x06E0, 0x06E1, 0x06E2
						  };  
	
	hw2000_write_reg(0x4C, 0x55AA);
	for (i = 0; i < 18; i++) {
        hw2000_write_reg(0x50 + i, agcTab[i]);            
    }  	 
	
	hw2000_write_reg(0x01, 0x4D55);  //5d57下功率更大  4D55
    hw2000_write_reg(0x02, 0x44CC);
	hw2000_write_reg(0x08, 0x7BC8);
	hw2000_write_reg(0x09, 0xCC00);
	hw2000_write_reg(0x19, 0x0884);
	hw2000_write_reg(0x1A, 0x0D31);
	hw2000_write_reg(0x20, 0x70A0); //8/10bit line code fec on
	hw2000_write_reg(0x23, data1);	//Scramble code 0b0011011
	hw2000_write_reg(0x28, 0x8404); //Max sync word error	
	hw2000_write_reg(0x29, data2);	//Scramble on
	hw2000_write_reg(0x2A, 0x4074);	//250Kbps
    hw2000_write_reg(0x2C, 0x918B); //CD threshold
	hw2000_write_reg(0x3C, 0x1000); //ACK Disable

	hw2000_write_reg(0x42, 0x8218); //High-16 bits addr
	hw2000_write_reg(0x41, 0xA7A3); //Middle-16 bits addr
	hw2000_write_reg(0x40, 0x92DD); //Low-16 bits addr
		
	hw2000_write_reg(0x12, 0x3043);
	hw2000_write_reg(0x13, 0x4000);
	hw2000_write_reg(0x14, 0x6032);
	hw2000_write_reg(0x15, 0xFC0c);   //8db
}

/******************************************************************************
 * @brief    hw2000_tx_data
 *           
 * @note
 *
 * @param	 data		data[0] indicate the length
 *			 channel    0~81 valid
 * @retval	 0 if success -1 failed
 *
 * @version  1.0
 * @date     2015-12-02
 * @author   sundy
 ******************************************************************************/
int8_t hw2000_tx_data(uint8_t *data, uint8_t channel)
{
//	if (_hw2000_state != TX) {
		disable_irq();
		hw2000_write_reg(0x21, 0x0000);
		hw2000_write_reg(0x23, 0x431B); //Soft reset
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_DMMODE))
		hw2000_write_reg(0x23, 0x031B); 
	else
		hw2000_write_reg(0x23, data1a); 
		hw2000_write_reg(0x22, 0x1800 | channel); //channel set
		
		hw2000_write_reg(0x21, 0x0100);
		hw2000_write_reg(0x21, 0x0100); //for delay 5us reason
		hw2000_write_reg(0x21, 0x0100); //for delay 5us reason
		hw2000_write_reg(0x21, 0x0100); //for delay 5us reason
		hw2000_write_reg(0x3B, 0x8000);
		hw2000_write_fifo(0x32, data, data[0]+1);
		hw2000_write_reg(0x36, 0x0081);
		enable_irq();
		
		_hw2000_state = TX;
	
		return 0;
//	} 
	
	return -1;
}

/******************************************************************************
 * @brief    hw2000_rx_enable
 *           
 * @note
 *
 * @param	 channel    0~81 valid
 * @retval	 None
 *
 * @version  1.0
 * @date     2015-12-02
 * @author   sundy
 ******************************************************************************/
void hw2000_rx_enable(uint8_t channel)
{	
	disable_irq();
	hw2000_write_reg(0x21, 0x0000);	
	hw2000_write_reg(0x23, 0x431B); //Soft reset
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_DMMODE))
		hw2000_write_reg(0x23, 0x031B); //软件复位不使能，MISO引脚输出不使能
	else
		hw2000_write_reg(0x23, data1a);  
		
	hw2000_write_reg(0x22, 0x1800 | channel); //channel set
	hw2000_write_reg(0x21, 0x0080);
	enable_irq();

	_hw2000_state = RX;
}

/******************************************************************************
 * @brief    hw2000_rx_data
 *           
 * @note
 *
 * @param	 data
 * @retval	 0 if success -1 failed
 *
 * @version  1.0
 * @date     2015-12-02
 * @author   sundy
 ******************************************************************************/
int8_t hw2000_rx_data(uint8_t *data)
{
	uint16_t reg;
	
	_hw2000_state = RX;
	
	reg = hw2000_read_reg(0x36);
	if( !(reg & 0x2000)) {
		hw2000_read_fifo(0x32, data, 1);
		if (data[0] != 15) {
			hw2000_write_reg(0x21, 0x0000);	
			hw2000_write_reg(0x23, 0x431B); //Soft reset
			hw2000_write_reg(0x23, 0x031B); 
			hw2000_write_reg(0x21, 0x0080);
			
			return -1;
		} else {
			hw2000_read_fifo(0x32, &data[1], data[0]);
			
			hw2000_write_reg(0x21, 0x0000);	
			hw2000_write_reg(0x23, 0x431B); //Soft reset
			hw2000_write_reg(0x23, 0x031B); 
			hw2000_write_reg(0x21, 0x0080);

			return 0;
		}
	} else {
		hw2000_write_reg(0x21, 0x0000);	
		hw2000_write_reg(0x23, 0x431B); //Soft reset
		hw2000_write_reg(0x23, 0x031B); 
		hw2000_write_reg(0x21, 0x0080);

		return -1;
	}
}

void radio_carrier_detect(int8 rx_statue)
{
	if(rx_statue==1)
	{
		carrier_time_count=0;
		channel_num++;
		channel_num %= CHANNEL_MAX1;
		hw2000_rx_enable(channel[channel_num]);
	}
	else if(rx_statue==0)
	{
		if(carrier_time_count>=NO_Data_TIME)   //连续64ms收不到信号跳频
		{
			carrier_time_count=0;
			channel_num++;
			channel_num %= CHANNEL_MAX1;
			hw2000_rx_enable(channel[channel_num]);
		}
	}
	else if(rx_statue==2)
	{
		carrier_time_count=0;
		channel_num++;
		channel_num %= CHANNEL_MAX1;
		hw2000_rx_enable(channel[channel_num]);
	}
//	uint8_t rssi;
//	
//	tick++;
//	if (tick >=CARRIER_TIME) {
//		tick = 0;
//		
//		if (_hw2000_state == RX) {
//			rssi = hw2000_read_reg(0x2D) & 0xFF;
////			if (rssi > 0xC4) {
//				channel_num++;
//				channel_num %= CHANNEL_MAX1;
//				hw2000_rx_enable(channel[channel_num]);
////			}	
//		}
//	}
}
/******************************************************************************
 * @brief    hw2000_power_test
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
void hw2000_power_test(void)
{
	hw2000_write_reg(0x1C, 0x501B);
	hw2000_write_reg(0x29, 0x0000);
	hw2000_write_reg(0x21, 0x0100);
	hw2000_write_reg(0x36, 0x0081);
}



