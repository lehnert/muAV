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

#ifndef __INV_USTORE_MANAGER_H__
#define __INV_USTORE_MANAGER_H__


/* Includes */
#include "mltypes.h"

/* Defines */
#define INV_USTORE_ID_TC            (1)
#define INV_USTORE_ID_ADV_FUSION    (INV_USTORE_ID_TC + 1)
#define INV_USTORE_ID_LITE_FUSION   (INV_USTORE_ID_ADV_FUSION + 1)
#define INV_USTORE_ID_COMP_SUP_LITE (INV_USTORE_ID_LITE_FUSION + 1)
#define INV_USTORE_ID_GYRO_CAL      (INV_USTORE_ID_COMP_SUP_LITE + 1)
#define INV_USTORE_ID_INVALID       (-1)

/* Typedefs */


struct uloadstoredelegate {
        inv_error_t (*store)(void);
        inv_error_t (*load) (void);
        unsigned short  len;
        unsigned short  tag;
};

/* Master APIs: These are called to enable ustore module and cause a load or store. */
inv_error_t inv_ustore_enable(void);

inv_error_t inv_uload_calibration(void);

inv_error_t inv_ustore_calibration(void);

inv_error_t inv_ustore_register_handler(const struct uloadstoredelegate *);


#endif // __INV_USTORE_MANAGER_H__

