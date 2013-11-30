/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: mlos_win32.c 4598 2011-01-25 19:33:13Z prao $
 *
 *******************************************************************************/

/**
 *  @defgroup MLOS
 *  @brief OS Interface for Atmel AVR32
 *
 *  @{
 *      @file mlos.c
 *      @brief OS Interface.
 */

/* ------------- */
/* - Includes. - */
/* ------------- */
#include "timerx8.h"
#include "mlos.h"
#include <stdint.h>

/* -------------- */
/* - Functions. - */
/* -------------- */

/**
 *  @brief  Sleep function.
**/
void inv_sleep(int mSecs)
{
    delay_ms(mSecs);
}


/**
 *  @brief  get system's internal tick count.
 *          Used for time reference.
 *  @return current tick count.
**/
unsigned long inv_get_tick_count(void)
{
	const long cpu_hz = 8000000;
	long count, ms;
	//count = Get_system_register(AVR32_COUNT);
	//ms = cpu_cy_2_ms(count,cpu_hz);
	return ms;
}

  /**********************/
 /** @} */ /* defgroup */
/**********************/


