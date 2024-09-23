/*
 * instance.cpp
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#include "instance.h"

BLDC_PWM bldc_pwm = {&htim4, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_4};
DRV8323 drv(&hspi1, SPI1_CS_DRV, DRV_nFAULT, DRV_ENABLE, DRV_CAL, DRV_INLA, DRV_INLB, DRV_INLC);
HallSensor hall(HALL_U, HALL_V, HALL_W);
A1333 encoder(&hspi2, SPI2_CS_ENC);

VelocityDriver velocity_driver(&bldc_pwm, &encoder, &drv, &hall);
TorqueDriver torque_driver(&bldc_pwm, &encoder, &drv, &hall);

DriverControllerBase *driver_controller{&velocity_driver};
// DriverControllerBase *driver_controller{&torque_driver};