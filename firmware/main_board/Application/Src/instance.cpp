/*
 * instance.cpp
 *
 *  Created on: Aug 13th, 2023
 *      Author: Reiji Terunuma
 */

#include "instance.h"

const float sampling_period = 0.01;
const float control_period = 0.01;

hardware::MPU6500 mpu6500_1(&spi_imu1);
hardware::MPU6500 mpu6500_2(&spi_imu2);
hardware::MPU6500 mpu6500_3(&spi_imu3);
hardware::MPU6500 mpu6500_4(&spi_imu4);
hardware::MPU6500 mpu6500_5(&spi_imu5);
hardware::MPU6500 mpu6500_6(&spi_imu6);

hardware::IMUBase *imu1{&mpu6500_1};
hardware::IMUBase *imu2{&mpu6500_2};
hardware::IMUBase *imu3{&mpu6500_3};
hardware::IMUBase *imu4{&mpu6500_4};
hardware::IMUBase *imu5{&mpu6500_5};
hardware::IMUBase *imu6{&mpu6500_6};

StateComplimantaryFilter state_comp(imu1, imu2, imu3, imu4, imu5, imu6);
StateBase *state{&state_comp};

LQR lqr_controller;
ControllerBase *controller{&lqr_controller};