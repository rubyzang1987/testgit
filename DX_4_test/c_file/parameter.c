#include	"config.h"

const	char	gcu8_ThisVersion[8]="1.00.";

volatile	__ThisHal	gvst_ThisHal;
volatile	__ThisBird	gvst_ThisBird;

volatile	uint32	gvu32_SystemFlag=0;/*系统事件记录器*/
volatile	uint32	gvu32_SystemRunTime=0;/*系统运行时间记录器，单位ms*/

volatile	uint32	gvu32_ImuSamFeqCnt=MV_FREQ_READ_IMU;
volatile	uint8	gvu8_U1ReceiveBuf[MV_U1_BUF];
volatile	uint8	gvu8_TelTxAddr[6]={0,0,0,0,0xC1,0};
volatile	uint8	gvu8_TelRxAddr[5];
volatile	uint8	gvu8_U1ReceiveBufCnt=0;
volatile	uint8	gvu8_U1ReceiveTimeCnt=0;
//volatile  uint8 mpu6050_acc[6];
//volatile  uint8 mpu6050_gyro[6];
volatile	__FP_ACC	gvfpacc_CurHeight;//当前高度
volatile	__FP_ACC	gvfpacc_ZeroHeight=0.0f;//初始高度
volatile	enum	LED_FLASH_KIND	gven_LedFlashKind=EV_LED_2;//led灯状态

volatile uint8 _hw2000_irq_request=0;
volatile uint8_t indexcount=0;
//对码过程使用
volatile uint32_t quad_id;
volatile uint32   quad_idf;
volatile uint8_t  quad_unlock;
volatile uint8_t  channel[5];
volatile uint8_t  channel_num;
volatile uint16_t tick;
volatile uint16_t life;

volatile uint32_t FLAGA=0;
volatile int ms_count;

volatile	uint8  START_MODE=0;
volatile	uint16  START_TEST_TIMER=0;
volatile	uint16 sleep_timer=0;
uint32 gyro_acc_data[16]={0};
volatile  uint32 vol_pwm[8];
volatile  int32  realthrottlea=540;
volatile  int32	realthrottleb=540;
volatile  int32	realthrottlesave=540;

volatile  uint8 hold_high_statue=HOLD_HIGH_STOP;
volatile	int32	H_I;//当前高度
volatile	int32	H_I_initialvalue;//初始高度
volatile	int32 H_I_average;
volatile	int32 H_I_zero;
volatile	int32 H_I_start;
//volatile  int16 baro_out;
volatile  int16  dg_updown=0;

volatile  int16 baro_speed1=0;
volatile  int32 fbm320_error1=0;

volatile  uint8 fblr_state;
volatile  uint8 fblr_stateall;

volatile  uint16 tx_rx_delayms=0;

volatile  int16 accel_out;
volatile  int16 baro_dis;

volatile  float check_accz_vel=0;
volatile  int16 fblr_com=0;
volatile  int16 fblr_com1=0;
volatile  uint16 up_to_keep=0;
volatile  int16 up_down_speed=0;

volatile  float log_lz=0;
volatile  float acc_velf=0;

//volatile  float coefficient_acc=1;
//volatile  float gxgygz=0;

//volatile  uint16 thr_clock_count=0;
volatile  int16  real_accz;

volatile  float ak=0;
volatile  float spin_rate_rad_s=0;
//2.4G跳频
volatile  uint8 carrier_time_count=0;
uint8 dataTel[20];
volatile uint8 signal_strength=0;
volatile uint8 signal_display=0;
volatile uint8 signal_display1=0;
volatile uint8 throttle_stop_time=0;

volatile int16 accel_outhand=0;
//volatile int8 pitch_trim=0,roll_trim=0,pitch_trim1=0,roll_trim1=0;
volatile uint16 data1a,data2a;
uint8 dataTX[10];
volatile uint8 link_select=LINK_NOLINK;
volatile int baro_home;






