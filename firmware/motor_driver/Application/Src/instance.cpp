/*
 * instance.cpp
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#include "instance.h"

BLDC_PWM bldc_pwm = {&htim4, TIM_CHANNEL_4, TIM_CHANNEL_2, TIM_CHANNEL_1};
DRV8323 drv(&hspi1, SPI1_CS_DRV, DRV_nFAULT, DRV_ENABLE, DRV_CAL, DRV_INLC, DRV_INLB, DRV_INLA);
HallSensor hall(HALL_U, HALL_V, HALL_W);
A1333 encoder(&hspi2, SPI2_CS_ENC);

DriverController driver_controller(&bldc_pwm, &encoder, &drv, &hall);