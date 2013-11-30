/*
 * CFile1.c
 *
 * Created: 14/06/2013 10:06:56 PM
 *  Author: Chris
 */ 
#include "adxl345.h"
#include "i2cmaster.h"

//Setup ADXL345 for constant measurement mode
void init_adxl345(void)
{
	//unsigned char localBuffer[2] = {ADXL345_POWER_CTL, MEASURE};	
	//i2cMasterSend(ADXL345_ADDR, 2, localBuffer);

	i2c_start(ADXL345_W); //write to ADXL345
	i2c_write(ADXL345_POWER_CTL); //Write to Power CTL register
	i2c_write(MEASURE); //Set the measure bit on D3
	i2c_stop();
	
	i2c_start(ADXL345_W); //write to ADXL345
	i2c_write(ADXL345_DATA_FORMAT); //Write to Power CTL register
	i2c_write(FULL_RES|0b11); //Set the full res bit on D4
	i2c_stop();
	
	i2c_start(ADXL345_W); //write to ADXL345
	i2c_write(ADXL345_BW_RATE); //Write to Power CTL register
	i2c_write(0x0A); //Set the full res bit on D4
	i2c_stop();
		
}

void read_adxl345(int16_t *x, int16_t* y, int16_t *z)
{		
	unsigned char buffer[6];

	i2c_start(ADXL345_W);
	i2c_write(ADXL345_DATAX0);
	i2c_start(ADXL345_R);
	

	buffer[0] =  i2c_readAck();
	buffer[1] =  i2c_readAck();
	buffer[2] =  i2c_readAck();
	buffer[3] =  i2c_readAck();
	buffer[4] =  i2c_readAck();
	buffer[5] =  i2c_readNak();
	i2c_stop();

	
	*x = (buffer[1]<<8) | buffer[0];
	*y = (buffer[3]<<8) | buffer[2];
	*z = (buffer[5]<<8) | buffer[4];
	
		//i2cMasterSend(HMC5843_ADDR,1,&local);
		//i2cMasterReceive(HMC5843_ADDR,2,localBuffer);
		
	//i2cMasterSend(ADXL345_ADDR,1,&local);
	//read data registers (temp, gyro x,y,z and accel x,y,z)
	//i2cMasterReceive(ADXL345_ADDR, 2, buffer);


	//i2c_start(ADXL345_W); //write to ADXL345
	
	//i2c_write(reg_adr);	//Read from a given address
		
	//i2c_start(ADXL345_R); // read from this I2C address, R/*W Set
	
	//lsb =  i2c_readAck();
	//msb =  i2c_readNak();
	//i2c_stop();

	
}