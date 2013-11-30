/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */

/*******************************************************************************
 *
 * $Id:$
 *
 ******************************************************************************/


#include <string.h>

#include "ustore_manager.h"
#include "ustore_delegate_io.h"
#include "ustore_manager_io.h"
#include "umpl_nvmem.h"
#include "mlsl.h"
//#include "packet.h"

#define USTORE_STATE_CLOSED      0
#define USTORE_STATE_STORE_OPEN  1
#define USTORE_STATE_LOAD_OPEN   2

#define STORE_CAL_SIZE 100


/* Don't allow overlapping store and load. 
 * (Just in case of a programming error in
 *  manager or delegate.)
 */
static int state = USTORE_STATE_CLOSED;

/* Buffer used for store and load */
nvram_addr_t cal_data = 0;
nvram_addr_t cal_end;

/* ustore state */
nvram_addr_t pcal;
nvram_addr_t section_end;

inv_error_t inv_ustoreload_set_max_len(int l)
{
    if (state == USTORE_STATE_CLOSED)
        return INV_ERROR;

    if (pcal + l <= cal_end){
        section_end = pcal + l;
        return INV_SUCCESS;
    } else {
        return INV_ERROR_MEMORY_EXAUSTED;
    }

}

void inv_ustoreload_reset_len(void)
{
    section_end = 0;
}

/* ------------- S T O R E - R O U T I N E S ------- */

inv_error_t inv_ustore_open(void)
{
    if (state != USTORE_STATE_CLOSED)
        return INV_ERROR;
    pcal = cal_data;
    section_end = 0;
    cal_end = cal_data + STORE_CAL_SIZE;
    state = USTORE_STATE_STORE_OPEN;

    /* Erase page on open. */
    {
        unsigned char garbage[1];
        const bool erase_page = true;
        nvram_write(pcal, garbage, 1, erase_page);
    }
    return INV_SUCCESS;
}

inv_error_t inv_ustore_close(void)
{
    inv_error_t result = INV_SUCCESS;
    if (state != USTORE_STATE_STORE_OPEN)
        return INV_ERROR;
    section_end = NULL;
    cal_end = NULL;
    state = USTORE_STATE_CLOSED;
    return result;
}

inv_error_t inv_ustore_byte(unsigned char b)
{
    return inv_ustore_mem((const void *)&b, 1);
}

inv_error_t inv_ustore_mem(const void *b, int length)
{
    const bool erase_page = false;
    if (state != USTORE_STATE_STORE_OPEN)
        return INV_ERROR;
    if (section_end != NULL && 
        (pcal + length) > section_end)
        return INV_ERROR_MEMORY_EXAUSTED;

    if (pcal + length >= cal_end)
        return INV_ERROR_MEMORY_EXAUSTED;

    nvram_write(pcal,  b, (size_t)length, erase_page);
    pcal += length;
    return INV_SUCCESS;
}

/* ------------- L O A D - R O U T I N E S ------- */

inv_error_t inv_uload_open(void)
{
    inv_error_t result;
    unsigned int store_length;
    unsigned char dataLen[4];
    if (state != USTORE_STATE_CLOSED)
        return INV_ERROR;

    nvram_read( 0, dataLen, 4 );
    store_length = dataLen[3];
    if (store_length > STORE_CAL_SIZE)
        return INV_ERROR_MEMORY_EXAUSTED;
    

    pcal = cal_data + 4;
    cal_end = cal_data + STORE_CAL_SIZE;
    section_end = 0;
    state = USTORE_STATE_LOAD_OPEN;
    return INV_SUCCESS;
}

inv_error_t inv_uload_close(void)
{
    if (state != USTORE_STATE_LOAD_OPEN)
        return INV_ERROR;

    section_end = NULL;
    cal_end = NULL;
    state = USTORE_STATE_CLOSED;
    return INV_SUCCESS;
}


inv_error_t inv_uload_byte(unsigned char *b)
{
    return inv_uload_mem(b, 1);
}

inv_error_t inv_uload_mem(void *b, int length)
{
    if (state != USTORE_STATE_LOAD_OPEN)
        return INV_ERROR;
    if (section_end != NULL &&
    (pcal + length) > section_end)
        return INV_ERROR_MEMORY_EXAUSTED;
    if (pcal + length >= cal_end)
        return INV_ERROR_MEMORY_EXAUSTED;

    nvram_read(pcal, b, length);
    pcal += length;
    return INV_SUCCESS;
}

inv_error_t inv_clear_nvram(void)
{
    unsigned char clearData[99];
    inv_error_t result = INV_SUCCESS;
    memset(clearData, 0, sizeof(clearData));
    result = inv_ustore_open();
    if (result != INV_SUCCESS){
        
        return result;
    }
    result = inv_ustore_mem(clearData, sizeof(clearData));
    if (result != INV_SUCCESS) {
        
        return result;
    }
    result = inv_ustore_close();
    if (result != INV_SUCCESS) {
        
        return result;
    }
    return INV_SUCCESS;
}


