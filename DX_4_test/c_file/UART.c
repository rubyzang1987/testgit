#include "uart.h"
#include "hr8p506.h"
#include	"state_estimation.h"

/*Versions：V1.0.0
 *Author  : Frank
 *Data    : 2018.7.14 PM
 *适用 HW2181 和 STM32F0单片机
 *使用时只需直接将'uart.h'和'uart.c'添加到工程中即可
 *根据所使用的单片机和波特率在'uart.h' 修改宏定义
 *发送格式全部为飞控数据如需使用用户数据 只需修改data_to_send[3]为"0xf*"即可
 *使用STM32单片机时需要把"#include "hr8p506.h""注释掉
 *地面站使用的是匿名四轴地面站
*/

#if USE_UART 

_t_flag_t f;			     //需要发送数据的标志

u8 data_to_send[50];	//发送数据缓存


extern volatile float Power_average;
extern volatile float h_target,h_targetf;
extern volatile int16 dg_updown;
extern u16 Power_value;
extern struct vertical_information  baro_acc_alt;
/*串口初始化*/
// void Uart_Init(uint32_t BaudRate_BRR)
// {
// 	#if HW2181
// 	{
// 		SCU->SCU_PROT.Word = 0x55AA6996; //Disable SCU protection
// 		SCU->CLKEN_PERI.CLKEN_UATR0 = 1; //enable euart0 clk
// 		SCU->SCU_PROT.Word = 0x00000000; //Enable SCU protection
// 		
// 		GPIO->PA_DIRBSR = 0x04000000;	//PA 26 is input uart
// 		GPIO->PA_FUNC3 &= 0xFFFFF0FF;
// 		GPIO->PA_FUNC3 |= 0x00000300;
// 		GPIO->PA_PUE |= 0x04000000;
// 		
// 		GPIO->PA_DIRBCR = 0x08000000;		//PA 27 is output uart
// 		GPIO->PA_FUNC3 &= 0xFFFF0FFF;
// 		GPIO->PA_FUNC3 |= 0x00003000;
// 		
// 		UART0->CON0.Word = 0x00060006;
// 		UART0->CON1.Word = 0x00000300;  //PCLK/4
// 		UART0->BRR.Word  = (12000000/BaudRate_BRR) ;  
// 		UART0->IE.RB = 1;
// 		UART0->CON0.Word = 0x00010001;
// 	}
// 	#elif STM32F0
// 	{
// 		USART_InitTypeDef USART_InitStructure;
// 		USART_InitStructure.USART_BaudRate = BaudRate_BRR;
// 		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
// 		USART_InitStructure.USART_StopBits = USART_StopBits_1;
// 		USART_InitStructure.USART_Parity = USART_Parity_No;
// 		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
// 		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
// 		USART_Init(USART1, &USART_InitStructure);
// 		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
// 		USART_Cmd(USART1, ENABLE);
// 	}
// 	#endif
// }
/*发送数据*/
void Uart_Tx(u8 *data,u8 length)
{
	u8 i = 0;
	for (i = 0; i < length; i++) {
		#if HW2181
			UART0->TBW.Byte[0] =(*(data+i)&(u16)0x01FF);
			while (!(UART0->IF.TB));
		#elif STM32F0
			USART1->TDR = (*(data+i)&(u16)0x01FF);
			while (!(USART1->ISR&=0X000080));
		#endif
	}			
}

/*
*Data_Exchange函数处理各种数据发送请求
*比如想实现每10ms发送一次传感器数据至上位机
*即在此函数内实现
*此函数应由用户每1ms调用一次
*/
void DT_Data_Exchange(void)
{
	static u8 cnt = 0;
	static u8 senser_cnt 	= 10;
	static u8 status_cnt 	= 15;
	static u8 rcdata_cnt 	= 20;
	static u8 motopwm_cnt	= 20;
	static u8 power_cnt		= 50;

	/*---------------------------------------------------*/  
	/*传感器数据*/
	if((cnt % senser_cnt) == (senser_cnt-1))  
		f.send_senser = 1;	
	/*飞控状态*/
	if((cnt % status_cnt) == (status_cnt-1))
		f.send_status = 1;	
	/*接收机*/ 
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.send_rcdata = 1;	
	/*电机输出*/
	if((cnt % motopwm_cnt) == (motopwm_cnt-1))
		f.send_motopwm = 1;	
	/*电池电压 电流*/
	if((cnt % power_cnt) == (power_cnt-1))
		f.send_power = 1;		
	
	cnt++;
  /*---------------------------------------------------*/  
	if(f.send_senser)/*传感器数据*/ 
	{
		f.send_senser = 0;
//		DT_Send_Senser((int16)gvst_ThisBird.AccDatNow.pitch ,
// 									 (int16)gvst_ThisBird.AccDatNow.roll ,
// 									 (int16)gvst_ThisBird.AccDatNow.yaw ,
// 	                 (int16)gvst_ThisBird.GyroDatNow.pitch,
// 	                 (int16)gvst_ThisBird.GyroDatNow.roll,
// 			             (int16)gvst_ThisBird.GyroDatNow.yaw,
// 			              0,0,0,0);	
	}	
  else if(f.send_status)/*飞控状态*/ 
	{
		f.send_status = 0;
		DT_Send_Status(Power_average,gvst_ThisBird.time.second*100,Power_value*10,0,0,0);
	}	
	else if(f.send_rcdata)/*接收机*/ 
	{
		f.send_rcdata = 0;
		//DT_Send_RCData(Rc_Pwm_In[0],Rc_Pwm_In[1],Rc_Pwm_In[2],Rc_Pwm_In[3],Rc_Pwm_In[4],Rc_Pwm_In[5],Rc_Pwm_In[6],Rc_Pwm_In[7],0,0);
	}	
	else if(f.send_motopwm)/*电机输出*/
	{
		f.send_motopwm = 0;
		//DT_Send_MotoPWM(1,2,3,4,5,6,7,8);
	}	
	else if(f.send_power)/*电池电压 电流*/ 
	{
		f.send_power = 0;
		//DT_Send_Power(Power_average,gvst_ThisBird.time.second*100);
	}
	else if(f.send_pid1)/*PID1*/ 
	{
		f.send_pid1 = 0;
		/*DT_Send_PID(1,ctrl_1.PID[PIDROLL].kp,ctrl_1.PID[PIDROLL].ki,ctrl_1.PID[PIDROLL].kd,
											ctrl_1.PID[PIDPITCH].kp,ctrl_1.PID[PIDPITCH].ki,ctrl_1.PID[PIDPITCH].kd,
											ctrl_1.PID[PIDYAW].kp,ctrl_1.PID[PIDYAW].ki,ctrl_1.PID[PIDYAW].kd);*/
	}	
	else if(f.send_pid2)/*PID2*/  
	{
		f.send_pid2 = 0;
		/*DT_Send_PID(2,ctrl_1.PID[PID4].kp,ctrl_1.PID[PID4].ki,ctrl_1.PID[PID4].kd,
											ctrl_1.PID[PID5].kp,ctrl_1.PID[PID5].ki,ctrl_1.PID[PID5].kd,
											ctrl_1.PID[PID6].kp,ctrl_1.PID[PID6].ki,ctrl_1.PID[PID6].kd);*/
	}
	else if(f.send_pid3)/*PID3*/  
	{
		f.send_pid3 = 0;
		/*DT_Send_PID(3,ctrl_2.PID[PIDROLL].kp,ctrl_2.PID[PIDROLL].ki,ctrl_2.PID[PIDROLL].kd,
											ctrl_2.PID[PIDPITCH].kp,ctrl_2.PID[PIDPITCH].ki,ctrl_2.PID[PIDPITCH].kd,
											ctrl_2.PID[PIDYAW].kp,ctrl_2.PID[PIDYAW].ki,ctrl_2.PID[PIDYAW].kd);*/
	}			
}
/*飞控状态*/
void DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	s16 _temp;
	s32 _temp2 = alt;
	u8 sum = 0,i=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xf1;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	

	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Uart_Tx(data_to_send, _cnt);
}
/*传感器数据*/
void DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar)
{
	u8 _cnt=0;
	s16 _temp;
	u8 sum=0,i=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Uart_Tx(data_to_send, _cnt);
}
/*接收机*/
// void DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
// {
// 	u8 _cnt=0;
// 	u8 sum = 0,i=0;
// 	
// 	data_to_send[_cnt++]=0xAA;
// 	data_to_send[_cnt++]=0xAA;
// 	data_to_send[_cnt++]=0x03;
// 	data_to_send[_cnt++]=0;
// 	data_to_send[_cnt++]=BYTE1(thr);
// 	data_to_send[_cnt++]=BYTE0(thr);
// 	data_to_send[_cnt++]=BYTE1(yaw);
// 	data_to_send[_cnt++]=BYTE0(yaw);
// 	data_to_send[_cnt++]=BYTE1(rol);
// 	data_to_send[_cnt++]=BYTE0(rol);
// 	data_to_send[_cnt++]=BYTE1(pit);
// 	data_to_send[_cnt++]=BYTE0(pit);
// 	data_to_send[_cnt++]=BYTE1(aux1);
// 	data_to_send[_cnt++]=BYTE0(aux1);
// 	data_to_send[_cnt++]=BYTE1(aux2);
// 	data_to_send[_cnt++]=BYTE0(aux2);
// 	data_to_send[_cnt++]=BYTE1(aux3);
// 	data_to_send[_cnt++]=BYTE0(aux3);
// 	data_to_send[_cnt++]=BYTE1(aux4);
// 	data_to_send[_cnt++]=BYTE0(aux4);
// 	data_to_send[_cnt++]=BYTE1(aux5);
// 	data_to_send[_cnt++]=BYTE0(aux5);
// 	data_to_send[_cnt++]=BYTE1(aux6);
// 	data_to_send[_cnt++]=BYTE0(aux6);

// 	data_to_send[3] = _cnt-4;
// 	
// 	for(i=0;i<_cnt;i++)
// 		sum += data_to_send[i];
// 	
// 	data_to_send[_cnt++]=sum;
// 	
// 	Uart_Tx(data_to_send, _cnt);
// }
/*电池电压 电流*/
void DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt=0;
	u16 temp;
	u8 sum = 0,i=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xf1;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Uart_Tx(data_to_send, _cnt);
}
// /*电机输出*/
// void DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
// {
// 	u8 _cnt=0;
// 	u8 sum = 0,i=0;
// 	
// 	data_to_send[_cnt++]=0xAA;
// 	data_to_send[_cnt++]=0xAA;
// 	data_to_send[_cnt++]=0x06;
// 	data_to_send[_cnt++]=0;
// 	
// 	data_to_send[_cnt++]=BYTE1(m_1);
// 	data_to_send[_cnt++]=BYTE0(m_1);
// 	data_to_send[_cnt++]=BYTE1(m_2);
// 	data_to_send[_cnt++]=BYTE0(m_2);
// 	data_to_send[_cnt++]=BYTE1(m_3);
// 	data_to_send[_cnt++]=BYTE0(m_3);
// 	data_to_send[_cnt++]=BYTE1(m_4);
// 	data_to_send[_cnt++]=BYTE0(m_4);
// 	data_to_send[_cnt++]=BYTE1(m_5);
// 	data_to_send[_cnt++]=BYTE0(m_5);
// 	data_to_send[_cnt++]=BYTE1(m_6);
// 	data_to_send[_cnt++]=BYTE0(m_6);
// 	data_to_send[_cnt++]=BYTE1(m_7);
// 	data_to_send[_cnt++]=BYTE0(m_7);
// 	data_to_send[_cnt++]=BYTE1(m_8);
// 	data_to_send[_cnt++]=BYTE0(m_8);
// 	
// 	data_to_send[3] = _cnt-4;
// 	
// 	for(i=0;i<_cnt;i++)
// 		sum += data_to_send[i];
// 	
// 	data_to_send[_cnt++]=sum;
// 	
// 	Uart_Tx(data_to_send, _cnt);
// }
// /*飞控状态->高度*/
// void DT_Send_Senser_Bar(int32_t bar,int16_t a_x)
// {
// 	u8 _cnt=0;
// 	s16 _temp;
// 	s32 _temp2=0;
//   u8 sum = 0,i=0;
// 	
// 	data_to_send[_cnt++]=0xAA;
// 	data_to_send[_cnt++]=0xAA;
// 	data_to_send[_cnt++]=0x07;
// 	data_to_send[_cnt++]=0;
//   
// 	_temp2=bar;
// 	data_to_send[_cnt++]=BYTE3(_temp2);
// 	data_to_send[_cnt++]=BYTE2(_temp2);
// 	data_to_send[_cnt++]=BYTE1(_temp2);
// 	data_to_send[_cnt++]=BYTE0(_temp2);
// 	
// 	_temp = a_x;
// 	data_to_send[_cnt++]=BYTE1(_temp);
// 	data_to_send[_cnt++]=BYTE0(_temp);

// 	data_to_send[3] = _cnt-4;
// 	
// 	for(i=0;i<_cnt;i++)
// 		sum += data_to_send[i];
// 	data_to_send[_cnt++] = sum;
// 	
// 	Uart_Tx(data_to_send, _cnt);
// }
// /*PID*/
// void DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
// {
// 	u8 _cnt=0;
// 	s16 _temp;
// 	u8 sum=0,i=0;
// 	
// 	data_to_send[_cnt++]=0xAA;
// 	data_to_send[_cnt++]=0xAA;
// 	data_to_send[_cnt++]=0x10+group-1;
// 	data_to_send[_cnt++]=0;
// 	
// 	
// 	_temp = p1_p * 1000;
// 	data_to_send[_cnt++]=BYTE1(_temp);
// 	data_to_send[_cnt++]=BYTE0(_temp);
// 	_temp = p1_i  * 1000;
// 	data_to_send[_cnt++]=BYTE1(_temp);
// 	data_to_send[_cnt++]=BYTE0(_temp);
// 	_temp = p1_d  * 1000;
// 	data_to_send[_cnt++]=BYTE1(_temp);
// 	data_to_send[_cnt++]=BYTE0(_temp);
// 	_temp = p2_p  * 1000;
// 	data_to_send[_cnt++]=BYTE1(_temp);
// 	data_to_send[_cnt++]=BYTE0(_temp);
// 	_temp = p2_i  * 1000;
// 	data_to_send[_cnt++]=BYTE1(_temp);
// 	data_to_send[_cnt++]=BYTE0(_temp);
// 	_temp = p2_d * 1000;
// 	data_to_send[_cnt++]=BYTE1(_temp);
// 	data_to_send[_cnt++]=BYTE0(_temp);
// 	_temp = p3_p  * 1000;
// 	data_to_send[_cnt++]=BYTE1(_temp);
// 	data_to_send[_cnt++]=BYTE0(_temp);
// 	_temp = p3_i  * 1000;
// 	data_to_send[_cnt++]=BYTE1(_temp);
// 	data_to_send[_cnt++]=BYTE0(_temp);
// 	_temp = p3_d * 1000;
// 	data_to_send[_cnt++]=BYTE1(_temp);
// 	data_to_send[_cnt++]=BYTE0(_temp);
// 	
// 	data_to_send[3] = _cnt-4;
// 	
// 	for(i=0;i<_cnt;i++)
// 		sum += data_to_send[i];
// 	
// 	data_to_send[_cnt++]=sum;

// 	Uart_Tx(data_to_send, _cnt);
// }

#endif




