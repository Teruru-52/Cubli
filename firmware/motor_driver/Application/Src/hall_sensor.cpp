/*
 * hall_sensor.cpp
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#include "hall_sensor.h"

HallSensor::HallSensor(GPIO_Value HALL_U, GPIO_Value HALL_V, GPIO_Value HALL_W)
    : HALL_U(HALL_U),
      HALL_V(HALL_V),
      HALL_W(HALL_U) {}

uint8_t HallSensor::GetHallValue()
{
    ReadHallValue();
    return hallstate;
}

void HallSensor::ReadHallValue()
{
    ReadHallValueU();
    ReadHallValueV();
    ReadHallValueW();
}

void HallSensor::ReadHallValueU()
{
    if (Read_GPIO(HALL_U) == 1)
        hallstate |= 0x01 << 2;
    else
        hallstate &= ~(0x01 << 2);
}

void HallSensor::ReadHallValueV()
{
    if (Read_GPIO(HALL_V) == 1)
        hallstate |= 0x01 << 1;
    else
        hallstate &= ~(0x01 << 1);
}

void HallSensor::ReadHallValueW()
{
    if (Read_GPIO(HALL_W) == 1)
        hallstate |= 0x01;
    else
        hallstate &= ~0x01;
}
