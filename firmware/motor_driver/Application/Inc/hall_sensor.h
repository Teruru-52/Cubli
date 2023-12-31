/*
 * hall_sensor.h
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef HALL_SENSOR_H_
#define HALL_SENSOR_H_

#include "main.h"

class HallSensor
{
private:
    uint8_t hallstate{0x00};
    GPIO_Value HALL_U;
    GPIO_Value HALL_V;
    GPIO_Value HALL_W;

public:
    HallSensor(GPIO_Value HALL_U, GPIO_Value HALL_V, GPIO_Value HALL_W);

    uint8_t GetHallValue();
    void ReadHallValue();
    void SetHallValueU(GPIO_PinState PinState);
    void SetHallValueV(GPIO_PinState PinState);
    void SetHallValueW(GPIO_PinState PinState);
};

#endif // HALL_SENSOR_H_