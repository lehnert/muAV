
	#ifndef _MOTOR_CONTROL_H
	#define _MOTOR_CONTROL_H
	
	#include <avr/io.h>
		
	#define IDLE_SPEED 0
	#define MAX_DUTY 255
	#define MAX_DUTY_TICKS 250
	#define MIN_DUTY_TICKS 50
	//#define MIN_DUTY_TICKS 130
		
		
	#define UP_DELTA 1
	#define DOWN_DELTA 1
		
	#define M1_OFFSET 0	//25
	#define M2_OFFSET 0
	#define M3_OFFSET 0
	#define M4_OFFSET 0	
		
	#define M1_IDLE_SPEED IDLE_SPEED + M1_OFFSET
	#define M2_IDLE_SPEED IDLE_SPEED + M2_OFFSET
	#define M3_IDLE_SPEED IDLE_SPEED + M3_OFFSET
	#define M4_IDLE_SPEED IDLE_SPEED + M4_OFFSET
	
	#define ROLL_DELTA 1
	#define PITCH_DELTA 1
	
	#define M1_SERVO_CHANNEL 0
	#define M2_SERVO_CHANNEL 1
	#define M3_SERVO_CHANNEL 2
	#define M4_SERVO_CHANNEL 3
	
	#define nMOTORS 4
	
	typedef struct {
		
		uint8_t channel;
		uint8_t Pin;
		uint8_t Port;
		uint8_t idle_speed;
		uint8_t trim;
		uint8_t duty;

	}BLMOTOR_t;
	
	volatile BLMOTOR_t motors[nMOTORS];
	
	void init_motors(void);
	void configure_motor(volatile BLMOTOR_t *motor, uint8_t channel, uint8_t outputPort, uint8_t outputPin, uint8_t idle_speed, uint8_t pwmOffset);
	void calibrate_motors(void);
	void reset_motors(void);
	void start_motors(void);
	void initPWM(void);
	void writePWMduty(uint8_t duty, volatile BLMOTOR_t *motor);

	//void servoPWM_init(uint16_t freq);
		
	#endif
	
/*#ifdef __cplusplus
}
#endif*/
