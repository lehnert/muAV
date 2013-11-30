/*
 * mavlink.h
 *
 * Created: 4/08/2013 8:31:25 PM
 *  Author: Chris
 */ 


#ifndef MAVLINK_FUNCTIONS_H_
#define MAVLINK_FUNCTIONS_H_

#include <mavlink/v1.0/common/mavlink.h>
#include <avr/io.h>


enum {
	GYRO =0, 		 //3 axis gyro present
	ACCEL,			//3 axis accelerometer present
	MAG,				//3 axis magnetometer present
	ABS_PRESSURE,	   //absollute pressure present
	DIFF_PRESSURE,	   //differential pressure present
	GPS,				//gps present
	OPTICAL_FLOW,	   //optical flow sensor present
	CV_POS,				//computer vision position present
	LASER_POS,
	GROUND_TRUTH,
	ANGLE_RATE_CONTROL,
	ATTITUDE_CONTROL,
	YAW_POSITION_CONTROL,
	ALITUDE_CONTROL,
	XY_POS_CONTROL,
	MOTOR_CONTROL,

} control_sensors_present_t;

void mavlink_init(void);
void mavlink_set_mode(uint8_t mode);
void mavlink_set_state(uint8_t state);
void mavlink_send_heartbeat(void);
void communication_receive(void);
void param_queued_send(void);
void data_stream_send(void);
void mavlink_send_status(void);
void volatile_param_array_reset_param_defaults(void);

#endif /* MAVLINK_H_ */