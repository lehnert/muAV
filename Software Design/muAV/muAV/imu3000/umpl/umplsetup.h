/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
#ifndef __UMPL_SETUP_H__
#define __UMPL_SETUP_H__

#include "mltypes.h"

// Called by umplInit
inv_error_t umplPlatformSetup(void);

// Called by umplStartMPU
inv_error_t umplDmpSetup(void);

#endif //  __UMPL_SETUP_H__

