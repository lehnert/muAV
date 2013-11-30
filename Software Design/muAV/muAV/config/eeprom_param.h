/*
 * eeprom_param.h
 *
 * Created: 13/08/2013 10:14:19 AM
 *  Author: Chris
 */ 


#ifndef EEPROM_PARAM_H_
#define EEPROM_PARAM_H_

#define ONBOARD_PARAM_COUNT 11
#define ONBOARD_PARAM_NAME_LENGTH  16

void write_param_to_eeprom(void);
void read_param_from_eeprom(void);

typedef struct
{
	float param[ONBOARD_PARAM_COUNT];
	char param_name[ONBOARD_PARAM_COUNT][ONBOARD_PARAM_NAME_LENGTH];
} param_array_t;


enum PARAMETERS
{
	PARAM_SYSTEM_ID =0,
	PARAM_COMPONENT_ID,
	PID_ROLL_P, /* PID roll proportional gain*/
	PID_ROLL_I, /* PID roll integral gain*/
	PID_ROLL_D, /* PID roll derivative gain*/
	
	PID_PITCH_P, /* PID pitch proportional gain*/
	PID_PITCH_I, /* PID pitch integral gain*/
	PID_PITCH_D, /* PID pitch derivative gain*/
	
	PID_YAW_P, /* PID yaw proportional gain*/
	PID_YAW_I, /* PID yaw integral gain*/
	PID_YAW_D, /* PID yaw derivative gain*/
};

#endif /* EEPROM_PARAM_H_ */