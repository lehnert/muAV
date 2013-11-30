/*
 * control.c
 *
 * Created: 1/07/2013 10:16:10 AM
 *  Author: Chris
 */ 
#include "control.h"
#include "motorControl.h"
#include "imu3000.h"
#include "global.h"
#include "global_parameters.h"
#include "eeprom_param.h"

#include <avr/interrupt.h>

#define MAX_ROLL_SPEED 60
#define MAX_PITCH_SPEED 60
#define MAX_YAW_SPEED 60
#define MIN_SPEED IDLE_SPEED

volatile int32_t m1_power, m2_power, m3_power, m4_power;
volatile int32_t power_difference_yaw, power_difference_roll, power_difference_pitch;

void initControlLoop()
{
	// initialize timer 2
	TCNT2 = 0;								// reset TCNT2
	//sbi(TIMSK2, TOIE2);						// enable TCNT2 overflow
	TCCR2A |= (1 << WGM21) | (0 << WGM20);
	
	TCCR2B |= (0 << WGM22) | (1 << CS22) | (1 << CS21) | (0 << CS20);
	TIMSK2 |= (1 << OCIE2A);
		
	//249 = 250Hz for 256 prescale	
	//249 = 500Hz for 128 prescale
	//155 = 800Hz for 128 prescale
	//249 = 1Khz for 64 prescale
	OCR2A = 249; 
	sei();
}

inline void rollPDcontrol(int32_t desired_roll, int32_t base_speed, volatile IMU_t *imu, uint8_t armed)
{
	//PD scaling factors
	//Gains defined in global.h 

	int32_t Kp_num = (int32_t)volatile_param_array.param[PID_ROLL_P];
	int32_t Kd_num = (int32_t)volatile_param_array.param[PID_ROLL_D];
	const int32_t K_den = 60000;//12;
	int32_t proportional = 0;
	int32_t derivative = 0;
	int32_t power_difference = 0;
	
	derivative = -imu->gyro_x;
	proportional = desired_roll - imu->roll;
	
	power_difference = (Kp_num*proportional/K_den)  +  (Kd_num*derivative/K_den);
	power_difference_roll = power_difference;

	// Compute the actual motor settings.  We never set either motor
	// to a negative value.
	if(power_difference > 0)
	{
		//+ form
		m3_power = MIN(base_speed + power_difference,MAX_ROLL_SPEED);
		m2_power = MAX(base_speed - power_difference,MIN_SPEED);
				
		//only apply power if motors are armed
		if(armed){
			writePWMduty((uint8_t)m3_power,&(motors[2]));
			writePWMduty((uint8_t)m2_power,&(motors[1]));
		}else{
			writePWMduty(0,&(motors[2]));
			writePWMduty(0,&(motors[1]));
		}				
					
	}else{
		//+ form
		m3_power = MAX(base_speed + power_difference,MIN_SPEED);
		m2_power = MIN(base_speed - power_difference,MAX_ROLL_SPEED);
		
		//only apply power if motors are armed
		if(armed){
			writePWMduty((uint8_t)m3_power,&(motors[2]));
			writePWMduty((uint8_t)m2_power,&(motors[1]));
		}else{
			writePWMduty(0,&(motors[2]));
			writePWMduty(0,&(motors[1]));
			
			
		}		
	}	
	
}

inline void pitchPDcontrol(int32_t desired_pitch, int32_t base_speed, volatile IMU_t *imu, uint8_t armed)
{
	//PD scaling factors
	//Gains defined in global.h
	int32_t Kp_num = (int32_t)volatile_param_array.param[PID_PITCH_P];
	int32_t Kd_num = (int32_t)volatile_param_array.param[PID_PITCH_D];
	const int32_t K_den = 60000;//12;
	
	int32_t proportional = 0;
	int32_t derivative = 0;
	
	int32_t power_difference = 0;

	derivative = -imu->gyro_y;
	proportional = desired_pitch - imu->pitch;
	power_difference = (Kp_num*proportional/K_den) +  (Kd_num*derivative/K_den);
    power_difference_pitch = power_difference;
	
	// Compute the actual motor settings.  We never set either motor
	// to a negative value.
	if(power_difference > 0)
	{
		m1_power = MIN(base_speed + power_difference,MAX_PITCH_SPEED);
		m4_power = MAX(base_speed - power_difference,MIN_SPEED);
		
		//+ form
		if(armed){
			writePWMduty((uint8_t)m1_power,&(motors[0]));
			writePWMduty((uint8_t)m4_power,&(motors[3]));
		}else{
			writePWMduty(0,&(motors[0]));
			writePWMduty(0,&(motors[3]));
		}			
	}else{
		m1_power = MAX(base_speed + power_difference,MIN_SPEED);
		m4_power = MIN(base_speed - power_difference,MAX_PITCH_SPEED);
		
		//+ form
		if(armed){
			writePWMduty((uint8_t)m1_power,&(motors[0]));
			writePWMduty((uint8_t)m4_power,&(motors[3]));
		}else{
			writePWMduty(0,&(motors[0]));
			writePWMduty(0,&(motors[3]));
		}			
	}
}

inline void PDcontrol(int32_t desired_yaw, int32_t desired_pitch, int32_t desired_roll, int32_t base_speed, volatile IMU_t *imu, uint8_t armed)
{
	//PD scaling factors
	//Gains defined in global.h
	int32_t Kp_num = (int32_t)volatile_param_array.param[PID_YAW_P];
	int32_t Kd_num = (int32_t)volatile_param_array.param[PID_YAW_D];
	
	const int32_t K_den = 60000;
	
	int32_t proportional = 0;
	int32_t derivative = 0;
	int32_t power_difference = 0;
	int32_t base_speed_pitch = 0, base_speed_roll = 0;

	derivative = -imu->gyro_z;
	
	proportional = desired_yaw - imu->yaw;

	power_difference = (Kp_num*proportional/K_den) +  (Kd_num*derivative/K_den);
	power_difference_yaw = power_difference;

	// Compute the actual motor settings.  We never set either motor
	// to a negative value.
	if(power_difference > 0)
	{
		//+ form
		base_speed_pitch = MAX(base_speed - power_difference,MIN_SPEED);
		base_speed_roll =  MIN(base_speed + power_difference,MAX_YAW_SPEED);		
	}else{
		base_speed_pitch = MIN(base_speed - power_difference,MAX_YAW_SPEED);			
		base_speed_roll = MAX(base_speed + power_difference,MIN_SPEED);
	}
	
	//use yaw base speeds as set point for roll and pitch base speeds
	rollPDcontrol(desired_roll, base_speed_roll, imu, armed);
	pitchPDcontrol(desired_pitch, base_speed_pitch, imu, armed);
}	


inline void yawPDcontrol(int32_t desired_yaw, int32_t base_speed, volatile IMU_t *imu, uint8_t armed)
{
	//PD scaling factors
	//Gains defined in global.h
	int32_t Kp_num = (int32_t)volatile_param_array.param[PID_YAW_P];
	int32_t Kd_num = (int32_t)volatile_param_array.param[PID_YAW_D];
	const int32_t Kp_den = 60000;//12;
	const int32_t Kd_den = 60000;
	
	int32_t proportional = 0;
	int32_t derivative = 0;
	
	int32_t power_difference = 0;

	derivative = -imu->gyro_z;
	
	proportional = desired_yaw - imu->yaw;
	
	power_difference = (Kp_num*proportional/Kp_den) +  (Kd_num*derivative/Kd_den);
	power_difference_yaw = power_difference;

	// Compute the actual motor settings.  We never set either motor
	// to a negative value.
	if(power_difference > 0)
	{
		m1_power = (MAX(base_speed - power_difference,MIN_SPEED));
		m4_power = m1_power;
		
		m2_power = (MIN(base_speed + power_difference,MAX_YAW_SPEED));
		m3_power = m2_power;
		
		//+ form
		if(armed){
			writePWMduty((uint8_t)m1_power,&(motors[0]));
			writePWMduty((uint8_t)m4_power,&(motors[3]));
			
			writePWMduty((uint8_t)m2_power,&(motors[1]));
			writePWMduty((uint8_t)m3_power,&(motors[2]));
		}else{
			writePWMduty(0,&(motors[0]));
			writePWMduty(0,&(motors[3]));
			writePWMduty(0,&(motors[1]));
			writePWMduty(0,&(motors[2]));			
		}
		
	}
	else
	{
		m1_power = (MIN(base_speed - power_difference,MAX_YAW_SPEED));
		m4_power = m1_power;
			
		m2_power = (MAX(base_speed + power_difference,MIN_SPEED));
		m3_power = m2_power;
		//+ form
		if(armed){
			writePWMduty((uint8_t)(MIN(base_speed - power_difference,MAX_YAW_SPEED)),&(motors[0]));
			writePWMduty((uint8_t)(MIN(base_speed - power_difference,MAX_YAW_SPEED)),&(motors[3]));
			
			writePWMduty((uint8_t)(MAX(base_speed + power_difference,MIN_SPEED)),&(motors[1]));
			writePWMduty((uint8_t)(MAX(base_speed + power_difference,MIN_SPEED)),&(motors[2]));
		}else{
			writePWMduty(0,&(motors[0]));
			writePWMduty(0,&(motors[3]));
			writePWMduty(0,&(motors[1]));
			writePWMduty(0,&(motors[2]));
		}

	}
}