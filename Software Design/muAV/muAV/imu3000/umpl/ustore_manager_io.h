/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */

#ifndef __INV_USTORE_IO_H__
#define __INV_USTORE_IO_H__

#include "mltypes.h"

/* Only ustore_manager.c is allowed to use these functions. */

inv_error_t inv_ustore_open(void);
inv_error_t inv_ustore_close(void);


inv_error_t inv_uload_open(void);
inv_error_t inv_uload_close(void);
inv_error_t inv_ustoreload_set_max_len(int);
void inv_ustoreload_reset_len(void);
inv_error_t inv_clear_nvram(void);

#endif // __INV_USTORE_IO_H__

