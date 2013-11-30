/*
 * muAV.cpp
 *
 * Created: 18/12/2012 12:39:31 PM
 *  Author: n7574258
 */ 

#include "global.h"
#include "global_parameters.h"

#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "imu3000.h"
#include "motorControl.h"
#include "control.h"
#include "uart.h"
#include "i2c.h"
#include "mavlink_functions.h"

void process_request(char myReceivedByte);
void sendIMUData(void);

volatile uint8_t global_counter;
volatile uint16_t raw_data_freq = 1;

volatile int32_t roll_desired = 0;
volatile int32_t pitch_desired = 0;
volatile int32_t yaw_desired = 0;
volatile int32_t thrust_desired = 0;



//MAVLINK Flags
volatile uint8_t MAVLINK_DATA_STREAM = FALSE;
volatile uint8_t MAVLINK_HEARTBEAT = FALSE;
volatile uint8_t MAVLINK_RECEIVE = FALSE;
volatile uint8_t SEND_STATUS = FALSE;

volatile uint64_t time_us = 0;

volatile IMU_t imu;

volatile state_t muav_state;

volatile uint8_t IMPLEMENT_STEP = FALSE;

int32_t manual_inputs[4] = {0,0,0,0};
	
volatile uint8_t step_speed = STEP_IDLE_SPEED;

int main(void)
{
	//muav_state = STOP;
	char myReceivedByte;
	
	uartInit();
	//uartSetBaudRate(57600);
	uartSetBaudRate(115200);
	
	//rprintfInit(uartSendByte);

	//Setup Motor control
	init_motors();
	
	i2cInit();
	i2cSetBitrate(400);
	
	// Initialise IMU
	init_imu();
	
	mavlink_init();
	
	//initalise parameters from eeprom
	read_param_from_eeprom();

	ENABLE_LED1;
	ENABLE_LED2;

	mavlink_set_mode(MAV_MODE_PREFLIGHT);
	mavlink_set_state(MAV_STATE_STANDBY);
	
	//init control loop
	initControlLoop();
	

	while(1){	
			
		communication_receive();
		
		if(MAVLINK_HEARTBEAT){
			MAVLINK_HEARTBEAT = FALSE;
			mavlink_send_heartbeat();
		}
		
		//if(SEND_STATUS){
		//	SEND_STATUS = FALSE;
		//	mavlink_send_status();
		//}		
				
		if(MAVLINK_RECEIVE){	
			param_queued_send(); // Send parameters at 10 Hz, if previously requested	
			MAVLINK_RECEIVE = FALSE;

		}else if(MAVLINK_DATA_STREAM ){
			data_stream_send();		
			MAVLINK_DATA_STREAM = FALSE;
		}

	
		/*if(uartReceiveByte(&myReceivedByte))
		{
			LED1_TOGGLE;
			if(myReceivedByte == 'R'){
				system_mode = MAV_MODE_STABILIZE_ARMED;		
				sendIMUData();
				
				//LED1_TOGGLE;
			}else if(myReceivedByte == 'P'){
				system_mode = MAV_MODE_STABILIZE_DISARMED;
				sendIMUData();
			}
			//process_request(myReceivedByte);	
		}*/		
	} 

	return 0;   
}


ISR(TIMER2_COMPA_vect)
{	
	static uint16_t heartbeat_counter = 0, param_counter = 0, status_counter = 0, data_counter, step_counter = 0;

	//global_counter = TCNT2;
	
		
	switch(system_mode)
	{
		case MAV_MODE_PREFLIGHT:
		{
			
			
		}
		break;
		
		case MAV_MODE_STABILIZE_DISARMED:
		{
			//LED1_OFF;
			read_imu(&imu);
			for(int i = 0; i < nMOTORS; i++) {
				writePWMduty(0, &(motors[i]));
			}
			//PDcontrol(0,0,0,35,&imu,FALSE);
			pitchPDcontrol(0, 35, &imu, FALSE);
		}
		break;
		
		case MAV_MODE_STABILIZE_ARMED:
		{
			//LED1_OFF;
			read_imu(&imu);
			//PDcontrol(0,0,0,35,&imu,TRUE);
			
			pitchPDcontrol(0, 35, &imu, TRUE);
			
			//writePWMduty((uint8_t)25, &(motors[0]));
			
			//rollPDcontrol(0, 30, &imu, TRUE);
			//pitchPDcontrol(pitch_desired, thrust_desired, &imu, &motor_power1, &motor_power2, FALSE);
			//yawPDcontrol(0, 35, &imu, TRUE);
			//yawPDcontrol(yaw_desired, thrust_desired, &imu, TRUE);
		}
		break;
		
		case MAV_MODE_MANUAL_ARMED:
		{
			
			const uint8_t MAX_INPUT = 160;
			const uint8_t MIN_INPUT = 0;
			
			manual_inputs[2] = thrust_desired + roll_desired + yaw_desired;
			manual_inputs[1] = thrust_desired - roll_desired + yaw_desired;
			
			manual_inputs[0] = thrust_desired + pitch_desired - yaw_desired;
			manual_inputs[3] = thrust_desired - pitch_desired - yaw_desired;
			

			for(int i = 0; i < nMOTORS; i++) {
				manual_inputs[i] = MAX(MIN(manual_inputs[i],MAX_INPUT),MIN_INPUT);
				writePWMduty((uint8_t)manual_inputs[i], &(motors[i]));
								
			}
			
			//read_imu(&imu);
			//manual_inputs[1] = MAX(MIN(manual_inputs[1],MAX_INPUT),MIN_INPUT);
			//writePWMduty((uint8_t)manual_inputs[1], &(motors[1]));

			
		
			/*if(step_counter < 250*STEP_PRETIME){
				writePWMduty(step_speed, &(motors[STEP_MOTOR]));
				step_counter++;
				
			}else if((step_counter >= 250*STEP_PRETIME) && (step_counter < 250*(STEP_PRETIME+STEP_TIME))){
				writePWMduty(step_speed, &(motors[STEP_MOTOR]));
				step_counter++;
				LED1_ON;
			}else{
				writePWMduty(0, &(motors[STEP_MOTOR]));
				LED1_OFF;
				if(step_speed < STEP_FINAL_SPEED){
					step_speed += STEP_SPEED_SIZE;
					step_counter = 250*STEP_PRETIME;
			    }else{
					step_speed += STEP_IDLE_SPEED;
					step_counter = 0;
				}								
				
			}*

			
			manual_inputs[0] = MAX(MIN(manual_inputs[0],MAX_INPUT),MIN_INPUT);
			manual_inputs[3] = MAX(MIN(manual_inputs[3],MAX_INPUT),MIN_INPUT);
			
			writePWMduty((uint8_t)manual_inputs[0], &(motors[0]));
			writePWMduty((uint8_t)manual_inputs[3], &(motors[3]));
			writePWMduty(0, &(motors[2]));
			writePWMduty(0, &(motors[1]));*/

					
		}
		break;
		
	
		
		//mode unknown stop motors
		default:
		{
			//for(int i = 0; i < nMOTORS; i++) {
			//	writePWMduty(0,&(motors[i]));
			//}
			
		}
		break;
		
		
	}
	
	
	
	//update counters
	heartbeat_counter++; param_counter++; status_counter++; data_counter++;
	
	//0.5Hz
	if(status_counter > 500){SEND_STATUS = TRUE; status_counter = 0;}
	//10Hz
	if(param_counter > 25){MAVLINK_RECEIVE = TRUE; param_counter = 0;}
	//1Hz		
	if(heartbeat_counter > 250){LED2_TOGGLE;MAVLINK_HEARTBEAT = TRUE; heartbeat_counter = 0;}
	//Varaible Freq
	if(m_data_stream_on){
		if(data_counter > 250/raw_data_freq){MAVLINK_DATA_STREAM = TRUE; data_counter = 0;	}
	}else{
		MAVLINK_DATA_STREAM = FALSE;
	}	
	//update time in microseconds (250Hz = 4000us per cycle)
	time_us+=4000;
	
	//global_counter = TCNT2 - global_counter;
	//uartSendByte(global_counter);
		
}


void sendIMUData(void)
{
	char *ptr = 0;
	uartAddToTxBuffer(0xAA);
	uartAddToTxBuffer(0xAA);
	
	/*ptr = (char *)&(motor_power1);
	uartAddToTxBuffer(ptr[0]);
	uartAddToTxBuffer(ptr[1]);
	uartAddToTxBuffer(ptr[2]);
	uartAddToTxBuffer(ptr[3]);
	
	ptr = (char *)&(motor_power2);
	uartAddToTxBuffer(ptr[0]);
	uartAddToTxBuffer(ptr[1]);
	uartAddToTxBuffer(ptr[2]);
	uartAddToTxBuffer(ptr[3]);*/
	
	/*ptr = (char *)&(imu.raw_accel_x
	uartAddToTxBuffer(ptr[0]);
	uartAddToTxBuffer(ptr[1]);

	ptr = (char *)&(imu.gyro_y);
	uartAddToTxBuffer(ptr[0]);
	uartAddToTxBuffer(ptr[1]);*/


	/*ptr = (char *)&(imu.accel_x);
	uartAddToTxBuffer(ptr[0]);
	uartAddToTxBuffer(ptr[1]);

	ptr = (char *)&(imu.accel_y);
	uartAddToTxBuffer(ptr[0]);
	uartAddToTxBuffer(ptr[1]);*/

	ptr = (char *)&(imu.roll);
	uartAddToTxBuffer(ptr[0]);
	uartAddToTxBuffer(ptr[1]);
	uartAddToTxBuffer(ptr[2]);
	uartAddToTxBuffer(ptr[3]);
	
	ptr = (char *)&(imu.pitch);
	uartAddToTxBuffer(ptr[0]);
	uartAddToTxBuffer(ptr[1]);
	uartAddToTxBuffer(ptr[2]);
	uartAddToTxBuffer(ptr[3]);

	/*ptr = (char *)&(imu.pitch);
	uartAddToTxBuffer(ptr[0]);
	uartAddToTxBuffer(ptr[1]);
	uartAddToTxBuffer(ptr[2]);
	uartAddToTxBuffer(ptr[3]);*/
		
	uartSendTxBuffer();
}

void process_request(char myReceivedByte){

	if(myReceivedByte == SPIN_UP)
	{
		muav_state = IDLE;
	}
	
	if(myReceivedByte == START_CONTROL)
	{
		muav_state = START;
	}
	
	if(myReceivedByte == PAUSE)
	{
		muav_state = STOP;
	}
	
	if(myReceivedByte == CALIBRATE)
	{
		LED1_TOGGLE;
		calibrate_motors();
	}
	
	if(myReceivedByte == RESET_MOTORS)
	{
		reset_motors();
	}



	if(myReceivedByte == ROLL_LEFT){
		writePWMduty(motors[0].duty+ROLL_DELTA,&(motors[0]));
		
		if((int8_t)motors[2].duty-ROLL_DELTA < 0)
		{
			writePWMduty(0,&(motors[2]));
		}else{
			writePWMduty(motors[2].duty-ROLL_DELTA,&(motors[2]));
		}
	}
	
	if(myReceivedByte == ROLL_RIGHT){
		if((int8_t)motors[0].duty-ROLL_DELTA < 0)
		{
			writePWMduty(0,&(motors[0]));
		}else{
			writePWMduty(motors[0].duty-ROLL_DELTA,&(motors[0]));
		}
		writePWMduty(motors[2].duty+ROLL_DELTA,&(motors[2]));
	}
	
	if(myReceivedByte == PITCH_UP){
		writePWMduty(motors[1].duty+PITCH_DELTA,&(motors[1]));
		if(motors[3].duty < 0)
		{
			writePWMduty(0,&(motors[3]));
		}else{
			writePWMduty(motors[3].duty-ROLL_DELTA,&(motors[3]));
		}
	}
	
	if(myReceivedByte == PITCH_DOWN){
		if(motors[1].duty < ROLL_DELTA)
		{
			writePWMduty(0,&(motors[1]));
		}else{
			writePWMduty(motors[1].duty-ROLL_DELTA,&(motors[1]));
		}
		writePWMduty(motors[3].duty+PITCH_DELTA,&(motors[3]));
	}
		

	
	if(myReceivedByte == MOTOR_UP_1)
	{
		writePWMduty(motors[0].duty + UP_DELTA,&(motors[0]));
		
		/*for(int i = 0; i < nMOTORS; i++) {
			writePWMduty(motors[i].duty + 100,&(motors[i]));
		}*/
	}
	
	if(myReceivedByte == MOTOR_UP_2)
	{
		writePWMduty(motors[1].duty + UP_DELTA,&(motors[1]));
	}
	if(myReceivedByte == MOTOR_UP_3)
	{
		writePWMduty(motors[2].duty + UP_DELTA,&(motors[2]));
	}
	if(myReceivedByte == MOTOR_UP_4)
	{
		writePWMduty(motors[3].duty + UP_DELTA,&(motors[3]));
	}
	
	if(myReceivedByte == MOTOR_DOWN_1)
	{
		writePWMduty(motors[0].duty < DOWN_DELTA ? 0 : (motors[0].duty - DOWN_DELTA),&(motors[0]));
	}
	
	if(myReceivedByte == MOTOR_DOWN_2)
	{
		writePWMduty(motors[1].duty < DOWN_DELTA ? 0 : (motors[1].duty - DOWN_DELTA),&(motors[1]));
	}
	
	if(myReceivedByte == MOTOR_DOWN_3)
	{
		writePWMduty(motors[2].duty < DOWN_DELTA ? 0 : (motors[2].duty - DOWN_DELTA),&(motors[2]));
	}
	if(myReceivedByte == MOTOR_DOWN_4)
	{
		writePWMduty(motors[3].duty < DOWN_DELTA ? 0 : (motors[3].duty - DOWN_DELTA),&(motors[3]));
	}
	
	/*if(myReceivedByte == MOTOR_DOWN)
	{
		for(int i = 0; i < nMOTORS; i++) {
			if(motors[i].duty > 0)
			writePWMduty(motors[i].duty - 1,&(motors[i]));
		}
	}*/
	

	
	
		
	
}

