/*
 * encoder.h
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "main.h"
#include "spi.h"

class A1333
{
protected:
    SPI_HandleTypeDef *spi;
    GPIO_Value SPI_CS_ENC;
    float angle;
    int16_t angle_raw;
    float velocity;
    uint16_t flag_err;

public:
    A1333(SPI_HandleTypeDef *spi, GPIO_Value SPI_CS_ENC);

    uint16_t read_byte(uint8_t reg1, uint8_t reg2);
    void write_byte(uint8_t reg, uint8_t data);

    void Initialize();
    void Update();
    uint16_t GetErrFlag() { return read_byte(0x24, 0x25); };
    float GetAngle() { return angle; };
    uint16_t GetRawAngle() { return angle_raw; };
    float GetVelocity() { return velocity; };
};

#endif // ENCODER_H_