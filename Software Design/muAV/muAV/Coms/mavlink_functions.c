/*
 * CFile1.c
 *
 * Created: 3/08/2013 10:40:15 PM
 *  Author: Chris
 */ 

/* The default UART header for your MCU */

#include <avr/eeprom.h>
#include "eeprom_param.h"
#include "global_parameters.h"
#include "mavlink_functions.h"
#include "string.h"
#include "uart.h"
#include "imu3000.h"
#include <avr/io.h>

volatile mavlink_system_t mavlink_system;

uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

volatile uint16_t m_parameter_i = ONBOARD_PARAM_COUNT;
volatile uint8_t m_data_stream_on = 0;
volatile uint8_t m_data_stream_type = 0;


uint32_t packet_drops = 0;
volatile uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
volatile uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
volatile uint8_t mavlink_state = MAV_STATE_STANDBY; ///< System ready for flight

#define SENSORS_PRESENT (BV(GYRO) | BV(ACCEL) | BV(ATTITUDE_CONTROL) | BV(MOTOR_CONTROL))
#define SENSORS_ENABLED (BV(GYRO) | BV(ACCEL) | BV(ATTITUDE_CONTROL) | BV(MOTOR_CONTROL))



void mavlink_init(void){
	
	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_SYSTEM_CONTROL;     ///< The component sending the message is the IMU, it could be also a Linux process
	mavlink_system.type = MAV_TYPE_QUADROTOR;

	system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
	custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
	mavlink_state = MAV_STATE_STANDBY; ///< System ready for flight
	
	//volatile_param_array_reset_param_defaults();
	
}

inline void mavlink_set_mode(uint8_t mode){
	system_mode = mode;
}

inline void mavlink_set_state(uint8_t state){
	mavlink_state = state;
}

inline void mavlink_send_heartbeat() {
	
	uint16_t len;
	char buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_message_t msg;

	
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.type, autopilot_type, system_mode, custom_mode, mavlink_state);
	len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
	//send buffer but check if buffer is full
	while(!uartSendBuffer(buf, len));
	
}

inline void mavlink_send_status(){
	
	uint16_t len;
	char buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_message_t msg;

	
	mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg, SENSORS_PRESENT,SENSORS_ENABLED,SENSORS_ENABLED,1000,7400,-1,100,0,0,0,0,0,0);
	len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
	//send buffer but check if buffer is full
	while(!uartSendBuffer(buf, len));
	
}

/**
* @brief Receive communication packets and handle them
*
* This function decodes packets on the protocol level and also handles
* their value by calling the appropriate functions.
*/
inline void communication_receive()
{	
	uint16_t len;
	char buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_message_t msg;
	mavlink_status_t status;
	
	uint8_t c;
	// COMMUNICATION THROUGH EXTERNAL UART PORT (XBee serial)
	if(!uartReceiveBufferIsEmpty())
	{
		uartReceiveByte(&c);
		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message
			LED1_TOGGLE;
			switch(msg.msgid)
			{	
				case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:
				{
					//mavlink_change_operator_control_t set;
					//mavlink_msg_change_operator_control_decode(&msg, &set);
				}					
				break;
				
				case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
				{
					mavlink_request_data_stream_t set;

					mavlink_msg_request_data_stream_decode(&msg, &set);			
					switch(set.req_stream_id)
					{									
						case MAV_DATA_STREAM_RAW_SENSORS:
						{
							if(set.start_stop){
								m_data_stream_on = TRUE;
								m_data_stream_type = MAV_DATA_STREAM_RAW_SENSORS;
								raw_data_freq = set.req_message_rate;
								mavlink_msg_data_stream_pack(mavlink_system.sysid, mavlink_system.compid, &msg, MAV_DATA_STREAM_RAW_SENSORS, raw_data_freq, 1);
								len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
								while(!uartSendBuffer(buf, len));
							}else{
								m_data_stream_on = FALSE;
								mavlink_msg_data_stream_pack(mavlink_system.sysid, mavlink_system.compid, &msg, MAV_DATA_STREAM_RAW_SENSORS, raw_data_freq, 0);
								len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
								while(!uartSendBuffer(buf, len));
							}								
							
						}	
						break;
						
						case MAV_DATA_STREAM_POSITION:
						{
							if(set.start_stop){
								m_data_stream_on = TRUE;
								m_data_stream_type = MAV_DATA_STREAM_POSITION;
								raw_data_freq = set.req_message_rate;
								mavlink_msg_data_stream_pack(mavlink_system.sysid, mavlink_system.compid, &msg, MAV_DATA_STREAM_POSITION, raw_data_freq, 1);
								len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
								while(!uartSendBuffer(buf, len));
							}else{
								m_data_stream_on = FALSE;
								mavlink_msg_data_stream_pack(mavlink_system.sysid, mavlink_system.compid, &msg, MAV_DATA_STREAM_POSITION, raw_data_freq, 0);
								len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
								while(!uartSendBuffer(buf, len));
							}
						
						}
						break;							
						
						case MAV_DATA_STREAM_RAW_CONTROLLER:
						{
							if(set.start_stop){
								m_data_stream_on = TRUE;
								m_data_stream_type = MAV_DATA_STREAM_RAW_CONTROLLER;
								raw_data_freq = set.req_message_rate;
								mavlink_msg_data_stream_pack(mavlink_system.sysid, mavlink_system.compid, &msg, MAV_DATA_STREAM_RAW_CONTROLLER, raw_data_freq, 1);
								len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
								while(!uartSendBuffer(buf, len));
							}else{
								m_data_stream_on = FALSE;
								mavlink_msg_data_stream_pack(mavlink_system.sysid, mavlink_system.compid, &msg, MAV_DATA_STREAM_RAW_CONTROLLER, raw_data_freq, 0);
								len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
								while(!uartSendBuffer(buf, len));
							}
							
						}
						break;		
											
						default:
						//Do nothing
						break;
					}
				}																		
				break;
				
				
				
				case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
				{
					// Start sending parameters
					m_parameter_i = 0;
				}				
				break;
				
				case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
				{
					mavlink_param_request_read_t set;
					// Start sending parameters
					mavlink_msg_param_request_read_decode(&msg, &set);

					mavlink_msg_param_value_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
					(char*) volatile_param_array.param_name[set.param_index],
					volatile_param_array.param[set.param_index], MAVLINK_TYPE_FLOAT,
					ONBOARD_PARAM_COUNT, set.param_index);				
				}				
				break;
				
				case MAVLINK_MSG_ID_PARAM_SET:
				{
					mavlink_param_set_t set;
					mavlink_msg_param_set_decode(&msg, &set);
					
									
					char* key = (char*) set.param_id;
						
					for (uint16_t i = 0; i < ONBOARD_PARAM_COUNT; i++)
					{
						uint8_t match = TRUE;
						for (uint16_t j = 0; j < ONBOARD_PARAM_NAME_LENGTH; j++)
						{
							// Compare
							if (((char) (volatile_param_array.param_name[i][j]))
							!= (char) (key[j]))
							{
								match = FALSE;
							}
								
							// End matching if null termination is reached
							if (((char) volatile_param_array.param_name[i][j]) == '\0')
							{
								break;
							}
						}
							
						// Check if matched
						if (match)
						{
								
							// Only write and emit changes if there is actually a difference
							// AND only write if new value is NOT "not-a-number"
							// AND is NOT infinity
							if (volatile_param_array.param[i] != set.param_value	&& !isnan(set.param_value) && !isinf(set.param_value))// && set.param_type == MAVLINK_TYPE_FLOAT)
							{	
													
								volatile_param_array.param[i] = set.param_value; 
								
								// Report back new value
								mavlink_msg_param_value_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
								(char*) volatile_param_array.param_name[i],
								volatile_param_array.param[i],MAVLINK_TYPE_FLOAT,
								ONBOARD_PARAM_COUNT, i);
								
								len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
								while(!uartSendBuffer(buf, len));
									
							}
						}
					}
				}				
				break;
				
				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					// E.g. read GCS heartbeat and go into
					// comm lost mode if timer times out
				}					
				break;
				
				case MAVLINK_MSG_ID_SET_MODE:
				{
					mavlink_set_mode_t set;		
					mavlink_msg_set_mode_decode(&msg, &set);					
					system_mode = set.base_mode;
				}					
				break;
				
				case MAVLINK_MSG_ID_COMMAND_LONG:
				{
					mavlink_command_long_t set;
					mavlink_msg_command_long_decode(&msg, &set);
					
					switch(set.command){
						
						case MAV_CMD_PREFLIGHT_STORAGE:
						{
							if(system_mode == MAV_MODE_PREFLIGHT)
							{
								if(set.param1 == 1){
									write_param_to_eeprom();
								}else{
									read_param_from_eeprom();
								}
							}								
							
						}
						break;
						
						case MAV_CMD_DO_SET_MODE:
						{
							system_mode = (uint8_t)set.param1;
							// Report back new value
							mavlink_msg_command_ack_pack(mavlink_system.sysid, mavlink_system.compid, &msg, MAV_CMD_DO_SET_MODE, MAV_CMD_ACK_OK);
							len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
							while(!uartSendBuffer(buf, len));
							
							
						}
						break;
						
						case MAV_CMD_NAV_TAKEOFF:
						{						
							//implement take off
							//muav_state = TAKE_OFF;
							
							//LED1_ON;
							
							/*LED1_ON;
							
							if(step_speed < STEP_FINAL_SPEED){
								step_speed += STEP_SPEED_SIZE;
							}else{
								step_speed = STEP_IDLE_SPEED;
							}*/
							mavlink_msg_command_ack_pack(mavlink_system.sysid, mavlink_system.compid, &msg, MAV_CMD_NAV_TAKEOFF, MAV_CMD_ACK_OK);
								
												
							len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
							while(!uartSendBuffer(buf, len));
							
							LED1_OFF;	
						}
						break;
						
						case MAV_CMD_NAV_LAND:
						{											
							//implement take off
							//muav_state = LAND;
							if(mavlink_state == MAV_STATE_ACTIVE)
							{
								mavlink_state = MAV_STATE_STANDBY;
								mavlink_msg_command_ack_pack(mavlink_system.sysid, mavlink_system.compid, &msg, MAV_CMD_NAV_LAND, MAV_CMD_ACK_OK);
							}else{
								mavlink_msg_command_ack_pack(mavlink_system.sysid, mavlink_system.compid, &msg, MAV_CMD_NAV_LAND, MAV_CMD_ACK_ERR_FAIL);
							}									
							len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
							while(!uartSendBuffer(buf, len));	
						}
						break;
						
						case MAV_CMD_PREFLIGHT_CALIBRATION:
						{															
							if(system_mode == MAV_MODE_PREFLIGHT)calibrate_imu();
							mavlink_msg_command_ack_pack(mavlink_system.sysid, mavlink_system.compid, &msg, MAV_CMD_PREFLIGHT_CALIBRATION, MAV_CMD_ACK_OK);
							len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
							while(!uartSendBuffer(buf, len));
							
						}
						break;
						
						case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
						{
							if(system_mode == MAV_MODE_PREFLIGHT)calibrate_imu();
							mavlink_msg_command_ack_pack(mavlink_system.sysid, mavlink_system.compid, &msg, MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS, MAV_CMD_ACK_OK);
							len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
							while(!uartSendBuffer(buf, len));
							
						}
						break;
						
						case MAV_CMD_COMPONENT_ARM_DISARM:
						{
							if(set.param1 == 1){
								//set arm flag in system mode
								system_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
							}else if(set.param1 == 0)
							{
								///clear arm flag in system mode
								system_mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
							}
							
							mavlink_msg_command_ack_pack(mavlink_system.sysid, mavlink_system.compid, &msg, MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_ACK_OK);
							len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
							while(!uartSendBuffer(buf, len));
							
						}
						break;	
						
						default:
						break;	
					}
				}
				break;
				
				
				case MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST:
				{
					mavlink_param_union_t mav_roll, mav_pitch, mav_yaw, mav_thrust;
					
					mavlink_set_roll_pitch_yaw_thrust_t set;
					mavlink_msg_set_roll_pitch_yaw_thrust_decode(&msg, &set);
					
					mav_roll.param_float = set.roll;	
					roll_desired = mav_roll.param_uint32; 
					
					mav_pitch.param_float = set.pitch;	
					pitch_desired = mav_pitch.param_uint32;
					
					mav_yaw.param_float = set.yaw;		
					yaw_desired = mav_yaw.param_uint32;
					
					mav_thrust.param_float = set.thrust;	
					thrust_desired = mav_thrust.param_uint32;
				}					
				break;
						
				
				case MAVLINK_MSG_ID_MANUAL_CONTROL:
				{			
					mavlink_manual_control_t set;
					mavlink_msg_manual_control_decode(&msg, &set);
					int32_t num_scale = 0;
					int32_t den_scale = 1;
					int32_t dead_zone = 0;
					
					
					//switch between manual mode PWM inputs scale to desired angle
					if(system_mode == MAV_MODE_STABILIZE_DISARMED){
						//scale for desired angles
						
						num_scale = BITS_PER_DEG*45;
						den_scale = 1000;
					}else{
						//scale for motor power
						num_scale = 1;
						den_scale = 4;
						dead_zone = 200;
						
					}		
									
					//handle deadzone inputs and scaling
					if((int32_t)set.y >= dead_zone){
						roll_desired = (((int32_t)set.y - dead_zone)*num_scale)/den_scale;
					}else if((int32_t)set.y < -dead_zone){
						roll_desired = (((int32_t)set.y + dead_zone)*num_scale)/den_scale;
					}else{
						roll_desired = 0;
					}						
						
					if((int32_t)set.x > dead_zone){
						pitch_desired = (((int32_t)set.x - dead_zone)*num_scale)/den_scale;
					}else if((int32_t)set.x < -dead_zone){
						pitch_desired = (((int32_t)set.x + dead_zone)*num_scale)/den_scale;
					}else{
						pitch_desired = 0;
					}
						
					if((int32_t)set.r > dead_zone){
						yaw_desired = (((int32_t)set.r - dead_zone)*num_scale)/den_scale;
					}else if((int32_t)set.r < -dead_zone){
						yaw_desired = (((int32_t)set.r + dead_zone)*num_scale)/den_scale;
					}else{
						yaw_desired = 0;
					}
								
					//thrust is always scaled as PWM input		
					thrust_desired = (int32_t)set.z/4;
										
										
					
					
					//ARM button
					if(set.buttons & (1<<0)){	
						//set arm flag in system mode
						system_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
					}
					//DISARM button
					if(set.buttons & (1<<1)){
						//clear arm flag in system mode
						 system_mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
					}

					if(set.buttons & (1<<3)){
						
					}
					
					if(set.buttons & (1<<4)){
						
					}
					
				}					
				break;
				
							
								
				default:
				//Do nothing
				break;
			}
			
			
			// Update global packet drops counter
			packet_drops += status.packet_rx_drop_count;
		}
	}
}

 
/**
* @brief Send low-priority messages at a maximum rate of xx Hertz
*
* This function sends messages at a lower rate to not exceed the wireless
* bandwidth. It sends one message each time it is called until the buffer is empty.
* Call this function with xx Hertz to increase/decrease the bandwidth.
*/

void param_queued_send(void)
{
	uint16_t len;
	char buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_message_t msg;

	//send parameters one by one
	if (m_parameter_i < ONBOARD_PARAM_COUNT)
	{
		mavlink_msg_param_value_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
		(char*) volatile_param_array.param_name[m_parameter_i],
		volatile_param_array.param[m_parameter_i],MAVLINK_TYPE_FLOAT,
		ONBOARD_PARAM_COUNT, m_parameter_i);
 
 		len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
 		//send buffer but check if buffer is full
 		while(!uartSendBuffer(buf, len));
		 
		m_parameter_i++;
		 
	}					
}


void data_stream_send(void){
	
	uint16_t len;
	char buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_message_t msg;
	
	if(m_data_stream_on)
	{
		switch(m_data_stream_type){
			
			case MAV_DATA_STREAM_RAW_SENSORS:
				mavlink_msg_raw_imu_pack(20, MAV_COMP_ID_SYSTEM_CONTROL, &msg, time_us,imu.accel_roll, imu.accel_pitch, imu.accel_z, imu.gyro_x, imu.gyro_y, imu.gyro_z,0,0,0);
				break;
			case MAV_DATA_STREAM_POSITION:
				mavlink_msg_attitude_pack(20, MAV_COMP_ID_SYSTEM_CONTROL, &msg, (uint32_t)(time_us/1000),(float)imu.roll, (float)imu.pitch, (float)imu.yaw, (float)imu.gyro_x, (float)imu.gyro_y,(float)imu.gyro_z);
				break;
			case MAV_DATA_STREAM_RAW_CONTROLLER:
				mavlink_msg_nav_controller_output_pack(20, MAV_COMP_ID_SYSTEM_CONTROL, &msg,(float)m1_power,(float)m2_power,(int16_t)m3_power,(int16_t)m4_power,0,(float)power_difference_yaw, (float)power_difference_roll,(float)power_difference_pitch);

				break;
		}				
				//m_parameter_i=0;
				len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
				//send buffer but check if buffer is full
				while(!uartSendBuffer(buf, len));
	}
}			

 
 /**
 * @brief reset all parameters to default
 * @warning DO NOT USE THIS IN FLIGHT!
 */
 
inline void volatile_param_array_reset_param_defaults(void)
{
	
	strcpy(volatile_param_array.param_name[PARAM_SYSTEM_ID], "SYS_ID");
	volatile_param_array.param[PARAM_SYSTEM_ID] = 20;
	
	strcpy(volatile_param_array.param_name[PARAM_COMPONENT_ID], "PARAM_COMP_ID");
	volatile_param_array.param[PARAM_COMPONENT_ID] = MAV_COMP_ID_SYSTEM_CONTROL;
	
	strcpy(volatile_param_array.param_name[PID_ROLL_P], "PID_ROLL_P");
	volatile_param_array.param[PID_ROLL_P] = (float)PID_ROLL_P_value;
	
	strcpy(volatile_param_array.param_name[PID_ROLL_I], "PID_ROLL_I");
	volatile_param_array.param[PID_ROLL_I] = (float)PID_ROLL_I_value;
	
	strcpy(volatile_param_array.param_name[PID_ROLL_D], "PID_ROLL_D");
	volatile_param_array.param[PID_ROLL_D] = (float)PID_ROLL_D_value;

	strcpy(volatile_param_array.param_name[PID_PITCH_P], "PID_PITCH_P");
	volatile_param_array.param[PID_PITCH_P] = (float)PID_PITCH_P_value;
	
	strcpy(volatile_param_array.param_name[PID_PITCH_I], "PID_PITCH_I");
	volatile_param_array.param[PID_PITCH_I] = (float)PID_PITCH_I_value;

	strcpy(volatile_param_array.param_name[PID_PITCH_D], "PID_PITCH_D");
	volatile_param_array.param[PID_PITCH_D] = (float)PID_PITCH_D_value;

	strcpy(volatile_param_array.param_name[PID_YAW_P], "PID_YAW_P");
	volatile_param_array.param[PID_YAW_P] = (float)PID_YAW_P_value;
	
	strcpy(volatile_param_array.param_name[PID_YAW_I], "PID_YAW_I");
	volatile_param_array.param[PID_YAW_I] = (float)PID_YAW_I_value;

	strcpy(volatile_param_array.param_name[PID_YAW_D], "PID_YAW_D");
	volatile_param_array.param[PID_YAW_D] = (float)PID_YAW_D_value;

}	