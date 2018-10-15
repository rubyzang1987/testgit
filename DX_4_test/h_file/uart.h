#ifndef	__UART_H
#define	__UART_H

#include "config.h"

 #define  USE_UART  0



#if USE_UART 
/*主控芯片*/
#define  HW2181    1
#define  STM32F0   (~HW2181)

/*串口波特率*/
#define BaudRate  (115200U)               //串口初始化入口参数(波特率)

/*数据拆分宏定义，
 *在发送大于1字节的数据类型时
 *比如int16、float等，
 *需要把数据拆分成单独字节进行发送
*/
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

/*数据发送标志位*/
typedef struct t_flag_t {
	u8 send_version;

	u8 send_status;

	u8 send_senser;

	u8 send_rcdata;

	u8 send_motopwm;

	u8 send_power;

	u8 send_pid1;

	u8 send_pid2;

	u8 send_pid3;
}_t_flag_t;

#if HW2181
void Uart_Init(u32 BaudRate_BRR);
#elif STM32F0
void Uart_Init(uint32_t BaudRate_BRR);
#endif
void DT_Data_Exchange(void);
void DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32 alt, u8 fly_model, u8 armed);
void DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar);
void DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void DT_Send_Power(u16 votage, u16 current);
void DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
void DT_Send_Senser_Bar(int32_t bar,int16_t a_x);
void DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
#endif



#endif




