/*
 * umpl_nvram.h
 *
 * Created: 5/2/2011 2:27:56 PM
 *  Author: sgurumani
 */ 


#ifndef UMPL_NVMEM_H_
#define UMPL_NVMEM_H_


#include "mltypes.h"
#include <avr/eeprom.h>
//! \name Data type for NVRAM memory addresses.

typedef uint16_t nvram_addr_t;


//MAP EEMEM EEStruct;
//! \name Atmel platform non-volatile memory spaces.
//void WriteMAPToEeprom(MAP a){
//	eeprom_write_block((const void*)&a, (void*)&EEStruct, sizeof(MAP));
//}

//MAP ReadMAPFromEeprom(void){
//	MAP temp;
//	eeprom_read_block((void*)&temp, (const void*)&EEStruct, sizeof(MAP));
//	return(temp);
//}


#define NVRAM_BASE_ADDR      (0)
#define NVRAM_SIZE           (1024)

void nvram_read (nvram_addr_t src, void * dst, size_t count);
void nvram_write (nvram_addr_t dst, const void * src, size_t count, bool erase);



#endif /* UMPL_NVMEM_H_ */