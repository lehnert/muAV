/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */

#ifndef _MLOS_H
#define _MLOS_H

#include <stdio.h>
#include <string.h>
#include "mltypes.h"

	/* ------------ */
	/* - Defines. - */
	/* ------------ */

	/* - MLOSCreateFile defines. - */

#define MLOS_GENERIC_READ         ((unsigned int)0x80000000)
#define MLOS_GENERIC_WRITE        ((unsigned int)0x40000000)
#define MLOS_FILE_SHARE_READ      ((unsigned int)0x00000001)
#define MLOS_FILE_SHARE_WRITE     ((unsigned int)0x00000002)
#define MLOS_OPEN_EXISTING        ((unsigned int)0x00000003)

	/* ---------- */
	/* - Enums. - */
	/* ---------- */

	/* --------------- */
	/* - Structures. - */
	/* --------------- */

	/* --------------------- */
	/* - Function p-types. - */
	/* --------------------- */

	void inv_sleep(int mSecs);
	unsigned long inv_get_tick_count(void);

	/* Kernel implmentations */
#define GFP_KERNEL (0x70)

	static inline void msleep(long msecs)
	{
		inv_sleep(msecs);
	}
	static inline void udelay(unsigned long usecs)
	{
		inv_sleep((usecs + 999) / 1000);
	}

#endif				/* _MLOS_H */
