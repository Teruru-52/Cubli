/*
 * instance.cpp
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#include "instance.h"

DRV8323 drv(&hspi1, SPI1_CS_DRV, DRV_nFAULT, DRV_ENABLE, DRV_CAL, LED_BLUE, DRV_INLx, LED_YELLOW);
HallSensor hall(HALL_U, HALL_V, HALL_W);
A1333 encoder(&hspi2, SPI2_CS_ENC);

VelocityDriver velocity_driver(&encoder, &drv, &hall);
TorqueDriver torque_driver(&encoder, &drv, &hall);

DriverControllerBase *driver_controller{&velocity_driver};
// DriverControllerBase *driver_controller{&torque_driver};