#include	"config.h"


#define M_PI 		    (double)3.1415926535
#define M_DEG_TO_RAD    (double)0.0174533
#define M_RAD_TO_DEG    (double)57.29578
#define math_max(x1,x2) (x1 > x2 ? x1 : x2)
#define math_min(x1,x2)	(x1 < x2 ? x1 : x2)

#define VELOCITY_KP 2.4f
#define POSITION_KP 1.0f

#define MPU6500_ACC_4G_SENSITIVITY	0.00239258   /* 注：该灵敏度单位是 m/s^2/LSB */   //9.8/8192

#define CONSTANTS_ONE_G					        9.80665f		/* m/s^2		*/


typedef struct {
//	hrt_abstime timestamp;	/**< in microseconds since system start */
	float roll;		        /**< Roll angle (degree, Tait-Bryan, NED)	*/
	float pitch;		    /**< Pitch angle (degree, Tait-Bryan, NED)	*/
	float yaw;		        /**< Yaw angle (degree, Tait-Bryan, NED)	*/
 	float R[3][3];		    /**< Rotation matrix body to world, (Tait-Bryan, NED)*/
	float q[4];		        /**< Quaternion (NED)	*/
	
}vehicle_attitude_s;

typedef struct {
	int16 x;
	int16 y;
	int16 z;
}filter_accel;

typedef struct {
	float x;
	float y;
	float z;
}filter_accel_F;

typedef struct  
{
	int16_t acc_raw[3];					/**< Raw acceleration in NED body frame           */
  float acc_calib_m_s2[3];
	float acc_m_s2[3];					/**< Acceleration in NED body frame, in m/s^2     */
	float acc_offset[3];
	
}sensor_s;

typedef struct 
{
    /*
    * Z axis weight for barometer
    *
    * Weight (cutoff frequency) for barometer altitude measurements.
    *
    * @min 0.0
    * @max 10.0
    */
	float w_z_baro;
    /**
    * Z axis weight for GPS
    *
    * Weight (cutoff frequency) for GPS altitude measurements. GPS altitude data is very noisy and should be used only as slow correction for baro offset.
    *
    * @min 0.0
    * @max 10.0
    */
	float w_z_gps_p;
    /**
    * XY axis weight for GPS position
    *
    * Weight (cutoff frequency) for GPS position measurements.
    *
    * @min 0.0
    * @max 10.0
    */
	float w_xy_gps_p;
    /**
    * XY axis weight for GPS velocity
    *
    * Weight (cutoff frequency) for GPS velocity measurements.
    *
    * @min 0.0
    * @max 10.0
    */
	float w_xy_gps_v;
    /**
    * XY axis weight for resetting velocity
    *
    * When velocity sources lost slowly decrease estimated horizontal velocity with this weight.
    *
    * @min 0.0
    * @max 10.0
    */
    float w_xy_reset_v;
    /**
    * Accelerometer bias estimation weight
    *
    * Weight (cutoff frequency) for accelerometer bias estimation. 0 to disable.
    *
    * @min 0.0
    * @max 0.1
    */
	float w_acc_bias;
    /**
    * GPS delay
    *
    * GPS delay compensation
    *
    * @min 0.0
    * @max 1.0
    * @unit s
    */
	float delay_gps;
   /**
    * Land detector time
    *
    * Vehicle assumed landed if no altitude changes happened during this time on low throttle.
    *
    * @min 0.0
    * @max 10.0
    * @unit s
    */
    float land_t;

    
}position_estimator_nav_params;


int16 altitude_out(void);
int16 altitude_out1(void);



