//**********************************************************
//
//             SCP1000 Pressure Sensor Library
//						SCP1000.h
//                     Ryan Owens
//			Copyright Sparkfun Electronics
//
//**********************************************************

//This file is used to define the functions  for the SCP1000 library
//as well as the register addresses for the SCP1000.

//**********************************************************
//
//                  Register Addresses
//
//**********************************************************
#define	REVID		0x00
#define DATAWR		0x01
#define ADDPTR		0x02
#define OPERATION	0x03
#define	OPSTATUS	0x04
#define	RSTR		0x06
#define	STATUS		0x07
#define	DATARD8		0x1F
#define	DATARD16	0x20
#define	TEMPOUT		0x21
#define	CFG			0x00
#define	CFG2		0x09
#define	MODTEST2	0x2D
#define	USERDATA1	0x29
#define	USERDATA2	0x2A
#define	USERDATA3	0x2B
#define	USERDATA4	0x2C

#define SCP_READ	0x0
#define	SCP_WRITE	0x2

//**********************************************************
//
//                  Function Protocols
//
//**********************************************************
void SCPinit(void);
void readSCP(unsigned int *scp_pressure, int *scp_temperature);
