/*
 * foc_driver.h
 *
 *  Created on: Sep 22th, 2024
 *      Author: Reiji Terunuma
 */

#ifndef FOC_DRIVER_H_
#define FOC_DRIVER_H_

#include "main.h"

enum PhaseState : uint8_t
{
    PHASE_OFF = 0, // both sides of the phase are off
    PHASE_ON = 1,  // both sides of the phase are driven with PWM, dead time is applied in 6-PWM mode
    PHASE_HI = 2,  // only the high side of the phase is driven with PWM (6-PWM mode only)
    PHASE_LO = 3,  // only the low side of the phase is driven with PWM (6-PWM mode only)
};

#endif // FOC_DRIVER_H_