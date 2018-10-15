#ifndef	__APP_H__
#define	__APP_H__

#include	"config.h"

extern u16 Difference_value(void);
extern	void	MissionLoop(void);
extern	void	MissionInit(void);
void    close_pwm(void);
void    open_pwm(void);
void	  WatchVol(void);
int8 Mpu6881Initial(void);

	
	void hr8p506_scu_init(void);
	void hr8p506_led_on(void);
	void hr8p506_led_off(void);
	void hr8p506_motor_init(void);
	void	SystemGPIOInit(void);
	//void close_pwm(void);
	//void open_pwm(void);
	
	//interrupt
	void hr8p506_interrupt_init(void);
	void pint3_handler(void);
	void pint4_handler(void);
	void SYSTICK_HANDLER(void);
	//adc
	void hr8p506_adc_init(void);
	uint16_t hr8p506_adc_sample(uint8_t ch);
	
	void hw2181_t32n0_init(void);
	void hw2181_uart_init(void);
	void hw2181_uart_tx(uint8_t *data, uint8_t length);
	void    usart_data_send(void);
	
	//I2C
	int8_t hr8p506_i2c_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t data);
	int8_t hr8p506_i2c_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
	
	void pwm50msopen1msclose(void);
	void   all_adc_read(void);
	void  delay_ms(int16 ms);
	void	delayus(int us);
	void  sleep(void);
	
	int8_t  GPIOA_READ(uint32_t PIN);
	int8_t  GPIOB_READ(uint32_t PIN);
#endif
//
void	check_flymode(uint8 throttle_advalue,uint8 mode_sure);
int	hold_high(float real_acc_z);
float get_realaccz(int16 read_accz,float acc_zero,float rolly,float pitchx,float accz_com);
//ÆøÑ¹¼Æ
void spl0601_init(void);
void	ReadBaro(void);
void    baro_average_pid(void);

void  disable_irq(void);
void  enable_irq(void);

void   sleep_key(void);

void	SendDatShow(int dat1,int dat2,int dat3,int dat4);
 void DT_Send_Senser(int32 a_x,int32 a_y,int32 a_z,
										int32 g_x,int32 g_y,int32 g_z,
										int32 m_x,int32 m_y,int32 m_z,
										int32 bar,int32 end);

//altitude
void ahrs_update_R_bf_to_ef(float angle_pitch,float angle_roll,float angle_yaw);
//float ahrs_get_accel_ef_z_mss(void);
void accel_baro_mag_gps_calculator(int16 accel_x,int16 accel_y,int16 accel_z,int16 zero_accelz,int32 baro_alt);
void Data_To_APP(void);
void UxSend(uint8 data);
