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
      HALL_W(HALL_W) {}

uint8_t HallSensor::GetHallValue()
{
    return hallstate;
}

void HallSensor::SetHallValueU(GPIO_PinState PinState)
{
    if (PinState == GPIO_PIN_SET)
    {
        hallstate |= 0x01 << 2;
        Write_GPIO(LED_BLUE, GPIO_PIN_SET);
    }
    else
    {
        hallstate &= ~(0x01 << 2);
        Write_GPIO(LED_BLUE, GPIO_PIN_RESET);
    }
}

void HallSensor::SetHallValueV(GPIO_PinState PinState)
{
    if (PinState == GPIO_PIN_SET)
    {
        hallstate |= 0x01 << 1;
        Write_GPIO(LED_GREEN, GPIO_PIN_SET);
    }
    else
    {
        hallstate &= ~(0x01 << 1);
        Write_GPIO(LED_GREEN, GPIO_PIN_RESET);
    }
}

void HallSensor::SetHallValueW(GPIO_PinState PinState)
{
    if (PinState == GPIO_PIN_SET)
    {
        hallstate |= 0x01;
        Write_GPIO(LED_YELLOW, GPIO_PIN_SET);
    }
    else
    {
        hallstate &= ~0x01;
        Write_GPIO(LED_YELLOW, GPIO_PIN_RESET);
    }
}
