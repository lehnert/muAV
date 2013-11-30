/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
#include "advFunc.h"
#include "mlMathFunc.h"
#include "mlmath.h"
#include "mldl.h"
#include "dmpKey.h"

inv_error_t inv_set_dmp_quaternion(long *q)
{
    inv_error_t result = INV_SUCCESS;
    unsigned char reg[16];

    inv_int32_to_big8(q[0],reg);
    inv_int32_to_big8(q[1],&reg[4]);
    inv_int32_to_big8(q[2],&reg[8]);
    inv_int32_to_big8(q[3],&reg[12]);
    if (inv_dmpkey_supported(KEY_D_0_192))
        result = inv_set_mpu_memory(KEY_D_0_192,16,reg);
    return result;
}
