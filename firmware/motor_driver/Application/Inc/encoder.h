/*
 * encoder.h
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "main.h"

class A1333
{
protected:
    SPI_TypeDef *spi;
    GPIO_Value SPI_CS_ENC;
    float angle;
    float velocity;

public:
    A1333(SPI_TypeDef *spi, GPIO_Value SPI_CS_ENC);

    uint8_t read_byte(uint8_t reg);
    void write_byte(uint8_t reg, uint8_t data);

    void Initialize();
    void Update();
    float GetAngle();
    float GetVelocity();
};

#endif // ENCODER_H_