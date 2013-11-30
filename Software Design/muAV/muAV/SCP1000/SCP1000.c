//**********************************************************
//
//             SCP100 Pressure Sensor Library
//						SCP1000.c
//                     Ryan Owens
//			Copyright Sparkfun Electronics
//
//**********************************************************
#include "LPC214x.h"
#include "SCP1000.h"
#include "SPI0.h"
#include "PackageTracker.h"
#include "serial.h"
#include "rprintf.h"

//NOTE: This library uses the macros SelectSCP() and UnSelectSCP(). 
// These macros must be defined before the library will work properly.

void SCPinit(void)
{
	char scp_status;

	SelectSCP();
	SPI0_send(((REVID<<2) & 0xFC) | SCP_READ);	
	UnselectSCP();
	
	SelectSCP();
	SPI0_send(((STATUS<<2) & 0xFC) | SCP_READ);	
	scp_status=SPI0_recv();
	UnselectSCP();
	//rprintf("SCP Status: %d\n", scp_status);
	
	SelectSCP();
	SPI0_send(((OPERATION<<2) & 0xFC) | SCP_WRITE);	
	SPI0_send(0x0A);		//Select High Resolution Read Mode
	UnselectSCP();
	
	SelectSCP();
	SPI0_send(((OPERATION<<2) & 0xFC) | SCP_READ);	
	scp_status=SPI0_recv();
	UnselectSCP();
}

void readSCP(unsigned int *scp_pressure, int *scp_temperature)
{
	SelectSCP();
	SPI0_send(((TEMPOUT<<2) & 0xFC) | SCP_READ);
	*scp_temperature=(SPI0_recv()<<8);
	*scp_temperature|=SPI0_recv();			
	UnselectSCP();
	//rprintf("SCP Temp: %d\n", scp_temperature/2);

	SelectSCP();
	SPI0_send(((DATARD8<<2) & 0xFC) | SCP_READ);
	*scp_pressure=((SPI0_recv()&0x0007)<<16);
	UnselectSCP();	

	SelectSCP();
	SPI0_send(((DATARD16<<2) & 0xFC) | SCP_READ);
	*scp_pressure|=(SPI0_recv()<<8);
	*scp_pressure|=SPI0_recv();
	UnselectSCP();		
	//rprintf("SCP Pressure: %d\n\n", scp_pressure/4);
}


