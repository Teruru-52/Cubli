/*
 * main_exec.cpp
 *
 *  Created on: Aug 13th, 2023
 *      Author: Reiji Terunuma
 */

#include "main_exec.h"
#include "instance.h"
#include "driver_controller.h"

void UpdateControl()
{
    uvw_t input_duty = driver_controller->Control();
    PWM_Update(&blcd_pwm, input_duty.u, input_duty.v, input_duty.w);
}