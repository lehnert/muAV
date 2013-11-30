

#ifndef __UMPL_STATES_H__
#define __UMPL_STATES_H__




#include "mltypes.h"

/* Five power states:
 *  0.  UMPL_UNINIT        : Unititialized (only on boot)
 *  1.  UMPL_STOP          : All sensors off (suitable for host processor asleep)
 *  2.  UMPL_RUN           : MPU + Accel sensor fusion
 *  3.  UMPL_ACCEL_ONLY    : Accel only with motion interrupt
 *  4.  UMPL_LPACCEL_ONLY  : Accel only in Low power mode with motion interrupt
 *
 * Transitions:
 *  0 is the state on boot
 *  0       -> 1 : Call to umplInit
 *  1, 3, 4 -> 2 : Call to umplStartMPU
 *  1, 2 ,4 -> 3 : Call to umplStartAccelOnly
 *  1, 2, 3 -> 4 : Call to umplStartAccelOnlyLowPower
 *  2, 3, 4 -> 1 : Call to umplStop
 */

#define UMPL_UNINIT                     (0)
#define UMPL_STOP                       (1)
#define UMPL_RUN                        (2)
#define UMPL_ACCEL_ONLY                 (3)
#define UMPL_LPACCEL_ONLY               (4)

int umplGetState(void);
inv_error_t umplInit(unsigned char *port);
inv_error_t umplStartMPU(void);
inv_error_t umplStartAccelOnly(float freq);
inv_error_t umplStartAccelOnlyLowPower(float freq);
inv_error_t umplStop(void);
inv_error_t umplNotifyInterrupt(unsigned char interrupt);
inv_error_t umplOnTimer(void);

#endif

