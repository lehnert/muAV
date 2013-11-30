/*
 * control.h
 *
 * Created: 1/07/2013 10:16:18 AM
 *  Author: Chris
 */ 


#ifndef CONTROL_H_
#define CONTROL_H_


#include "imu3000.h"

void initControlLoop(void);
void PDcontrol(int32_t desired_yaw, int32_t desired_pitch, int32_t desired_roll, int32_t base_speed, volatile IMU_t *imu, uint8_t armed);
void rollPDcontrol(int32_t desired_roll, int32_t base_speed, volatile IMU_t *imu, uint8_t armed);
void pitchPDcontrol(int32_t desired_pitch, int32_t base_speed, volatile IMU_t *imu, uint8_t armed);
void yawPDcontrol(int32_t desired_yaw, int32_t base_speed, volatile IMU_t *imu, uint8_t armed);

#endif /* CONTROL_H_ */