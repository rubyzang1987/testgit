#ifndef _STATE_ESTIMATION
#define _STATE_ESTIMATION


struct vertical_information
{
  // user variables converted from conresponding estimated variables 
	 int velocity_hat;			// from  velocity
	 int altitude_hat;			// from	altitude
};


void initialize_system_parameter(float sample_time,short Q_acc_temp,short R_barometer_temp);
void initialize_state_parameter(struct vertical_information *p,int altitude_temp,int velocity_temp);

void TimeUpdate(struct vertical_information *pAltitude,short accz);
void ObservationUpdate(struct vertical_information *p_altitude,int observation);


#endif
