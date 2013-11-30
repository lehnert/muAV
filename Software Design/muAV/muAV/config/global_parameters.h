/*
 * global_parameters.h
 *
 * Created: 5/08/2013 2:06:57 PM
 *  Author: Chris
 */ 


#ifndef GLOBAL_PARAMETERS_H_
#define GLOBAL_PARAMETERS_H_

#include <avr/eeprom.h>
#include <avr/io.h>
#include "eeprom_param.h"
#include "global.h"
#include "imu3000.h"


extern volatile IMU_t imu;

//Global Parameters
extern volatile uint8_t global_counter;
extern volatile uint16_t raw_data_freq;
extern volatile uint16_t pos_data_freq;

extern volatile int32_t PID_ROLL_P_value;
extern volatile int32_t PID_ROLL_I_value;
extern volatile int32_t PID_ROLL_D_value;

extern volatile int32_t PID_PITCH_P_value;
extern volatile int32_t PID_PITCH_I_value;
extern volatile int32_t PID_PITCH_D_value;

extern volatile int32_t PID_YAW_P_value;
extern volatile int32_t PID_YAW_I_value;
extern volatile int32_t PID_YAW_D_value;


extern volatile int32_t roll_desired;
extern volatile int32_t pitch_desired;
extern volatile int32_t yaw_desired;
extern volatile int32_t thrust_desired;

extern volatile int32_t motor_power1;
extern volatile int32_t motor_power2;

extern volatile uint8_t MAVLINK_HEARTBEAT;

extern volatile uint64_t time_us;

extern volatile state_t muav_state;
extern volatile uint8_t system_mode;
extern volatile uint32_t custom_mode;
extern volatile uint8_t mavlink_state;

extern volatile uint16_t m_parameter_i;
extern volatile uint8_t m_data_stream_on;
extern volatile uint8_t m_data_stream_type;

extern param_array_t volatile_param_array;
extern param_array_t EEMEM nonvolatile_param_array;

extern volatile uint8_t IMPLEMENT_STEP;

extern volatile int32_t m1_power, m2_power, m3_power, m4_power, power_difference_yaw, power_difference_roll, power_difference_pitch;
#define STEP_SPEED_SIZE 5
#define STEP_IDLE_SPEED 20
#define STEP_FINAL_SPEED 100
extern volatile uint8_t step_speed;

#endif /* GLOBAL_PARAMETERS_H_ */