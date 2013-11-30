/*
 * eeprom_param.c
 *
 * Created: 13/08/2013 10:14:07 AM
 *  Author: Chris
 */ 
#include <avr/io.h>
#include "eeprom_param.h"
#include "global_parameters.h"

param_array_t EEMEM nonvolatile_param_array = { .param_name[PARAM_SYSTEM_ID] = "SYS_ID",			.param[PARAM_SYSTEM_ID] = 20,\
												.param_name[PARAM_COMPONENT_ID] = "PARAM_COMP_ID",	.param[PARAM_COMPONENT_ID] = 250,\
												.param_name[PID_ROLL_P] = "PID_ROLL_P",				.param[PID_ROLL_P] = 2,\
												.param_name[PID_ROLL_I] = "PID_ROLL_I",				.param[PID_ROLL_I] = 0,\
												.param_name[PID_ROLL_D] = "PID_ROLL_D",				.param[PID_ROLL_D] = 1,\
												.param_name[PID_PITCH_P] = "PID_PITCH_P",			.param[PID_PITCH_P] = 2,\
												.param_name[PID_PITCH_I] = "PID_PITCH_I",			.param[PID_PITCH_I] = 0,\
												.param_name[PID_PITCH_D] = "PID_PITCH_D",			.param[PID_PITCH_D] = 1,\
												.param_name[PID_YAW_P] = "PID_YAW_P",				.param[PID_YAW_P] = 1,\
												.param_name[PID_YAW_I] = "PID_YAW_I",				.param[PID_YAW_I] = 0,\
												.param_name[PID_YAW_D] = "PID_YAW_D",				.param[PID_YAW_D] = 1};
																																																
param_array_t volatile_param_array;

inline void write_param_to_eeprom(){
	eeprom_update_block((void*)&volatile_param_array , (void*)&nonvolatile_param_array, sizeof(param_array_t));
}

inline void read_param_from_eeprom(){
	eeprom_read_block((void*)&volatile_param_array, (void*)&nonvolatile_param_array, sizeof(param_array_t));
}

