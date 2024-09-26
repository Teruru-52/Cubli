/*
 * hall_sensor.cpp
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#include "sensors/hall_sensor.h"

HallSensor::HallSensor(GPIO_Value HALL_U, GPIO_Value HALL_V, GPIO_Value HALL_W)
    : HALL_U(HALL_U),
      HALL_V(HALL_V),
      HALL_W(HALL_W) {}

void HallSensor::ReadHallValue()
{
    SetHallValueU(Read_GPIO(HALL_U));
    SetHallValueV(Read_GPIO(HALL_V));
    SetHallValueW(Read_GPIO(HALL_W));
}

void HallSensor::HandleCallback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == Hall_U_Pin)
        SetHallValueU(Read_GPIO(HALL_U));
    if (GPIO_Pin == Hall_V_Pin)
        SetHallValueV(Read_GPIO(HALL_V));
    if (GPIO_Pin == Hall_W_Pin)
        SetHallValueW(Read_GPIO(HALL_W));
    // FlashLED();
}

void HallSensor::SetHallValueU(GPIO_PinState PinState)
{
    if (PinState == GPIO_PIN_SET)
        hall_state |= 0x01 << 2;
    else
        hall_state &= ~(0x01 << 2);
}

void HallSensor::SetHallValueV(GPIO_PinState PinState)
{
    if (PinState == GPIO_PIN_SET)
        hall_state |= 0x01 << 1;
    else
        hall_state &= ~(0x01 << 1);
}

void HallSensor::SetHallValueW(GPIO_PinState PinState)
{
    if (PinState == GPIO_PIN_SET)
        hall_state |= 0x01;
    else
        hall_state &= ~0x01;
}

void HallSensor::FlashLED()
{
    if ((hall_state >> 2) & 0x01) // check hall U
        Write_GPIO(LED_BLUE, GPIO_PIN_SET);
    else
        Write_GPIO(LED_BLUE, GPIO_PIN_RESET);

    if ((hall_state >> 1) & 0x01) // check hall V
        Write_GPIO(LED_YELLOW, GPIO_PIN_SET);
    else
        Write_GPIO(LED_YELLOW, GPIO_PIN_RESET);

    if (hall_state & 0x01) // check hall W
        Write_GPIO(LED_RED, GPIO_PIN_SET);
    else
        Write_GPIO(LED_RED, GPIO_PIN_RESET);
}

void HallSensor::PrintLog()
{
    printf("hall = %d\n", hall_state);
}