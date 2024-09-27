/*
 * instance.h
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef INSTANCE_H_
#define INSTANCE_H_

#include "main.h"
#include "drivers/driver_controller.h"

extern DRV8323 drv;
extern HallSensor hall;
extern A1333 encoder;

extern DriverController driver_controller;

#endif // INSTANCE_H_