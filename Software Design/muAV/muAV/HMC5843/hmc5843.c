/*
    5-18-10
    Copyright Spark Fun Electronics© 2010
    Nathan Seidle
	
	Example I2C to control the HMC5843 3-axis magnetometer
	
	Based on Aaron Weiss' code.
	
	Designed to run on an Arduino using the standard serial bootloader.
	This is not written in Arduino, this is a C example.
	
	Things to know: 
	Unlike other I2C devices, in the HMC5843 you can keep reading registers and 
	the adress pointer will continue to increment.
	
	The only register you have to write to, to get the HMC5843 to start outputting data
	is 0x02, the 'Mode' register. You have to clear bit 1 (MD1) to go into continous coversion mode.
	
	Don't forget to enable or add pullups to SDA/SCL. This firmware uses the internal
	pullups. Should work fine without them.
	
	SCL is Analog pin 5 (aka PC5)
	SDA is Analog pin 4 (aka PC4)
*/
#ifdef __cplusplus
extern "C" {
	#endif
	
	#include "hmc5843.h"
	
	#include <stdio.h>
	#include <avr/io.h>
	#include "i2c.h" //Needed for I2C sensors
	#include "i2cmaster.h"
	#include "avrlibtypes.h"
	
	#define HMC5843_ADDR	0x3C
	#define HMC5883_ADDR	0x1E

	//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
	int16_t read_hmc5843(char reg_adr)
	{
		//char lsb, msb;
		unsigned char localBuffer[2] = {0,0};
		unsigned char local = reg_adr;
		
		//i2cMasterSend(HMC5843_ADDR,1,&local);
		//i2cMasterReceive(HMC5843_ADDR,2,localBuffer);
		
		//i2cMasterSend(HMC5883_ADDR,1,&local);
		//i2cMasterReceive(HMC5883_ADDR,2,localBuffer);
	
		return( (localBuffer[0]<<8) | localBuffer[1]);
	}
	
	int16_t read_hmc5883(char reg_adr, int16_t *mag_x, int16_t *mag_y, int16_t *mag_z){
		
		uint8_t buffer[6];
		unsigned char local = 0x03;
		
		//send read who am I register
		//i2cMasterSend(HMC5883_ADDR, 1, &local);

		//read data registers (temp, gyro x,y,z and accel x,y,z)
		//i2cMasterReceive(HMC5883_ADDR, 6, buffer);
		
		*mag_x	= buffer[0] << 8 | buffer[1];
		*mag_y   = buffer[2] << 8 | buffer[3];
		*mag_z	= buffer[4] << 8 | buffer[5];

	}

	//Setup HMC5843 for constant measurement mode
	void init_hmc5843(void)
	{
		unsigned char localBuffer[2] = {0,0};
			
		localBuffer[0]=0x02;	localBuffer[1]=0x00;
		//i2cMasterSend(HMC5843_ADDR, 2, localBuffer);
		//i2cMasterSend(HMC5883_ADDR, 2, localBuffer);

	}

	//int16_t read_hmc5843(char reg_adr)
	//{		
		//char lsb, msb;

		//i2c_start_wait(HMC5843_ADDR+I2C_WRITE);
	
		//i2c_write(reg_adr);	//Read from a given address
	
		//i2c_start(HMC5843_ADDR+I2C_READ);
	
		//msb = i2c_readAck();
		//lsb = i2c_readNak();
		//i2c_stop();
	
		//return( (msb<<8) | lsb);
	//}

	//Setup HMC5843 for constant measurement mode
	//void init_hmc5843(void)
	//{
		//i2c_start_wait(HMC5843_W);

		//i2c_write(0x02); //Write to Mode register
		//i2c_write(0x00);

		//i2c_stop();
	//}


#ifdef __cplusplus
}
#endif