/*
 * umpl_nvmem.c
 *
 * Created: 5/2/2011 2: :37 PM
 *  Author: sgurumani
 */ 
#include    "umpl_nvmem.h"

#include	<avr/eeprom.h>
#include	<avr/interrupt.h>
#include    <string.h>
#include	<stdio.h>
#include	"mlos.h"
//#include	"packet.h"

static unsigned int calLength;
static bool calDataFlag = false;
#define MAX_CAL_BUFFER_LEN 60

#define DEBUG_READ_CAL 

/*! \brief Write a buffer to non-volatile RAM
 *
 * This routine writes \c count Bytes to the NVRAM destination pointed
 * to by \c dst from the source buffer pointed to by \c src.
 *
 * \param   dst     the write destination in the NVRAM address space
 * \param   src     the source buffer in program data memory space
 * \param   count   the number of Bytes to write
 *
 * \return  Nothing.
 */
void nvram_write (nvram_addr_t dst, const void * src, size_t count, bool erase)
{
    eeprom_write_block(src,(volatile void *)dst,count);
}



/*! \brief Read a buffer from non-volatile RAM
 *
 * This routine reads \c count Bytes from the NVRAM source pointed
 * to by \c src to the destination buffer pointed to by \c dst.
 *
 * \param   src     the read source in the NVRAM address space
 * \param   dst     the destination buffer in program data memory space
 * \param   count   the number of Bytes to read
 *
 * \return  Nothing.
 */
void nvram_read (nvram_addr_t src, void * dst, size_t count)
{
	cli();
	eeprom_read_block(dst,(const void *)src,count);
	sei();
}
