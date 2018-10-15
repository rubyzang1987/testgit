#include	"fylib.h"


volatile	static  __SPEED_STRUCT baro_calspeed={0,0,0,0};
#pragma import(__use_no_semihosting_swi)
struct __FILE
{
 int handle;
};
FILE __stdout;
FILE __stdin;/*
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1,ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	return ch;
}*/
int fgetc(FILE *f)
{return	(0x0FF);}
int	ferror(FILE *f)
{return	EOF;}
void	_sys_exit(int x)
{
label:
	goto	label;
}

void	DelayMs(int ms)
{
	static int mss;
	ms_count=0;
	if(ms>1000)
		ms=1000;
	if(ms<0)
		ms=0;
	mss=ms;
  
	while(ms>ms_count);
}
void	delayus(int us)
{
	while(us--);
}

// void	SendDatShow(int dat1,int dat2,int dat3,int dat4)
// {
// 	static uint8 usart_jj=0;
// 	uint8 i;
// 	static uint8 data_send[13]={0xaa,0xaa,0x06,0x08};
// 	
// 	hw2181_uart_tx(data_send+usart_jj,1);
// 	usart_jj++;
// 	if(usart_jj>=13)
// 	{
// 		data_send[4]=dat1>>8&0xff;
// 		data_send[5]=dat1&0xff;
// 		data_send[6]=dat2>>8&0xff;
// 		data_send[7]=dat2&0xff;
// 		data_send[8]=dat3>>8&0xff;
// 		data_send[9]=dat3&0xff;
// 		data_send[10]=dat4>>8&0xff;
// 		data_send[11]=dat4&0xff;
// 		data_send[12]=(0x15E+data_send[4]+data_send[5]+data_send[6]+data_send[7]+data_send[8]+data_send[9]+data_send[10]+data_send[11])&0xff;
// 		usart_jj=0;
// 	}
// }
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
 void DT_Send_Senser(int32 a_x,int32 a_y,int32 a_z,
										int32 g_x,int32 g_y,int32 g_z,
										int32 m_x,int32 m_y,int32 m_z,
										int32 bar,int32 end)
{
    uint8 data_to_send[70];
	uint8_t _cnt=0;
	int32_t _temp=0;
//	int32_t _temp2=0;
	uint8_t sum = 0;
	uint8_t i=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xf2;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
		data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
		data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
		data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp=bar;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
		_temp=end;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	hw2181_uart_tx(data_to_send,_cnt);
}
 /*
uint32	GetCommand(void)
{
	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_U1DONE))
	{//U1指令接收执行
		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_U1DONE);

		//AskSystemState((char*)(gvu8_U1ReceiveBuf));
		gvu8_U1ReceiveBuf[0]='\0';
			
	}

	return	MV_ERR_NO_ERR;
}
*/
int	TelDatCheck(uint8* dat,int cnt)
{
	uint8	i,j=0;
	if(dat==NULL || cnt<=2)
		return	FALSE;

	for(i=0;i<(cnt-1);i++)
		j+=dat[i];

	if(j+dat[cnt-1]==0xff)	return	TRUE;
	
	return	FALSE;
}


////////////////////////////////
/////////////FY_LIB/////////////
////////////////////////////////
void	Fylib_Delays(int dlys)
{
	if(dlys<0)	return;
	while(dlys--);
}
void	Fylib_DelayMs(volatile int *time,int ms)
{
	if(0>ms)	return;
	ms+=*time;
	while(ms>*time);
}

float Fylib_Qrsqrt( float x )
{
/*
	//old version
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y = number;
	i = * ( long * ) &y; // evil floating point bit level hacking
	i = 0x5f3759df - ( i >> 1 ); // what the fuck?
	y = * ( float * ) &i;
	y = y * ( threehalfs - ( x2 * y * y ) ); // 1st iteration

	return y;
*/
	//new version
	float xhalf = 0.5f*x;
	int i = *(int*)&x; // get bits for floating VALUE
	i = 0x5f375a86- (i>>1); // gives initial guess y0
	x = *(float*)&i; // convert bits BACK to float
	x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
	return x;
}
//void	Fylib_AmplitudeLimiting1(const float limit,float *output)
//{
//	if(fabs(*output)<limit)
//		*output =0.0f;
//	else if(*output>0)
//		*output-=limit;
//	else
//		*output+=limit;
//}
//void	Fylib_LpFilter(const float ratio,const float input,float *output)    //低通滤波
//{
//	if(input==NULL ||output==NULL)	
//		return;

//	*output=*output*ratio+input*(1.0f-ratio);
//}
// float	Fylib_KalmanFilter(void* par,float input)
// {
// 	__SingleKlmPar	*lst_klmpar=(__SingleKlmPar*)par;
// 	float	kg,x_pre,x_next,p_pre,p_next;
// /*        
// 	这里Q=1e-6，R=1e-1但也没有提如何选取
// 	Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
// 	R:测量噪声，R增大，动态响应变慢，收敛稳定性变好        
//  */
// 	//how is the R & Q & p_pre?
// 	x_pre=lst_klmpar->x_pre;p_pre=lst_klmpar->p_pre+(lst_klmpar->Q);//step1;2
// 	kg=p_pre/(p_pre+(lst_klmpar->R));//step3
// 	x_next=x_pre+kg*(input-x_pre);//step4
// 	p_next=(1-kg)*p_pre;//step5

// 	lst_klmpar->x_pre=x_next;
// 	lst_klmpar->p_pre=p_next;
// 	return	x_next;
// }
float	Fylib_ConstrainF(float dat,float min,float max)   //限制最大最小值  floating
{
	if(dat<=min)
		return	min;
	else if(dat>=max)
		return max;

	return dat;
}
int	Fylib_Constrain(int dat,int min,int max)    //限制最大最小值  int
{
	if(dat<=min)
		return	min;
	else if(dat>=max)
		return max;

	return dat;
}

// float my_cos(float xx)
// {
// float result;
//   result = 1 - xx * xx/2;
// return result; 
// }

// float my_sin(float yy)
// {
// float result;
//   result = yy - yy * yy * yy /6;
// return result; 
// }
float FL_ABS(float x)
{
	float data_x;
   if(x < 0)
		 data_x=-x;
	 else
		 data_x=x;
	 return data_x;
}

//int32 speed_calculate1(int32 error_my,uint16 KT_NUM,__SPEED_STRUCT* speed_par)
//{
//	speed_par->distance_my+=error_my-speed_par->errorf_my;
//	speed_par->errorf_my=error_my;
//	speed_par->KT_COUNT++;
//	if(speed_par->KT_COUNT>=KT_NUM)
//	{
//		speed_par->distance_myout=speed_par->distance_my;
//		speed_par->distance_my=0;
//		speed_par->KT_COUNT=0;
//	}
//	return speed_par->distance_myout;
//}

// float speed_calculatef(float error_my,uint16 KT_NUM,__SPEED_STRUCTF* speed_parf)
// {
// 	speed_parf->distance_my+=error_my-speed_parf->errorf_my;
// 	speed_parf->errorf_my=error_my;
// 	speed_parf->KT_COUNT++;
// 	if(speed_parf->KT_COUNT>=KT_NUM)
// 	{
// 		speed_parf->distance_myout=speed_parf->distance_my;
// 		speed_parf->distance_my=0;
// 		speed_parf->KT_COUNT=0;
// 	}
// 	return speed_parf->distance_myout;
// }
// #define baro_ka 9

// void inertial_filter_predict(double x[6],float acczerror,int32 baro_alt,uint16 count)  //2ms
// {
// //	static double dt=0.002f;//,accz_speed;
// 	static int16 baro_speed;
// 	static uint16 predict_count=0;
// //	static int16 zero_com=0;
// //	x[2]=acczerror*9.8f/1000;
// //	x[0] += x[1] * dt + x[2] * dt * dt / 2.0f;
// //	x[1] += x[2] * dt;
// 	baro_speed=(int16)speed_calculate1(baro_alt,count,(__SPEED_STRUCT*)&baro_calspeed);
// 	predict_count++;
// 	if(predict_count>=count)
// 	{
// 		predict_count=0;
// //		x[1]=0;
// 		speed_calculate1(baro_alt,0,(__SPEED_STRUCT*)&baro_calspeed);
// 		if(baro_speed<20 && baro_speed>-20)
// 		{
// 			x[0]+=baro_speed;
// 			if(x[0]>70) x[0]=70;
// 			if(x[0]<-70) x[0]=-70;
// 		}
// //		x[3]=-zero_com*0;
// 		x[4]=baro_speed>0?baro_speed:-baro_speed;
// 		if(x[4]>baro_ka-1) x[4]=baro_ka-1;
// 		x[5]=(baro_ka-x[4])*10;
// 	}
// 	
// }

// int32 average_cal(int32 in_data,int16 KT_NUM,__AVERAGE_DATA* average_par)
// {
// //	static uint8 average_flag=0;
// 	average_par->data_sum+=in_data;
// 	average_par->in_count++;
// 	if(average_par->in_count>=KT_NUM)
// 	{
// 		average_par->data_average=average_par->data_sum/KT_NUM;
// 		average_par->data_sum=0;
// 		average_par->in_count=0;
// //		average_flag=1;
// 	}
// //	if(average_flag==0)
// //		average_par->data_average=500;
// 	return average_par->data_average;
// }




