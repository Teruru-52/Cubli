/*
 * hall_sensor.h
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef HALL_SENSOR_H_
#define HALL_SENSOR_H_

#include "main.h"
#include "common/foc_utils.h"

class HallSensor
{
private:
    GPIO_Value HALL_U;
    GPIO_Value HALL_V;
    GPIO_Value HALL_W;
    uint8_t hall_state = 0x00;
    uint8_t pre_hall_state = 0x00;
    int8_t sector = -1;

    // 000 001 010 011 100 101 110 111
    const int8_t sectors[8] = {-1, 4, 2, 3, 0, 5, 1, -1}; // u:0
    // const int8_t sectors[8] = {-1, 5, 3, 4, 1, 0, 2, -1};
    // const int8_t sectors[8] = {-1, 0, 4, 5, 2, 1, 3, -1}; // w:0
    // const int8_t sectors[8] = {-1, 1, 5, 0, 3, 2, 4, -1};
    // const int8_t sectors[8] = {-1, 2, 0, 1, 4, 3, 5, -1}; // v:0
    // const int8_t sectors[8] = {-1, 3, 1, 2, 5, 4, 0, -1};

    // const int8_t sectors[8] = {-1, 2, 4, 3, 0, 1, 5, -1}; // u:0
    // const int8_t sectors[8] = {-1, 3, 5, 4, 1, 2, 0, -1};
    // const int8_t sectors[8] = {-1, 4, 0, 5, 2, 3, 1, -1}; // v:0
    // const int8_t sectors[8] = {-1, 5, 1, 0, 3, 4, 2, -1};
    // const int8_t sectors[8] = {-1, 0, 2, 1, 4, 5, 3, -1}; // w:0 (simpleFOC)
    // const int8_t sectors[8] = {-1, 1, 3, 2, 5, 0, 4, -1};

public:
    HallSensor(GPIO_Value HALL_U, GPIO_Value HALL_V, GPIO_Value HALL_W);

    void Initialize();
    uint8_t GetHallValue() { return hall_state; }
    void ReadHallValue();
    void HandleCallback(uint16_t GPIO_Pin);
    void SetHallValueU(GPIO_PinState PinState);
    void SetHallValueV(GPIO_PinState PinState);
    void SetHallValueW(GPIO_PinState PinState);
    void UpdateSector();
    int8_t GetSector() { return sector; }
    void FlashLED();
    void PrintLog();
};

#endif // HALL_SENSOR_H_