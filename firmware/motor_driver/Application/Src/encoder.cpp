/*
 * encoder.cpp
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#include "encoder.h"

A1333::A1333(SPI_TypeDef *spi, GPIO_Value SPI_CS_ENC)
    : spi(spi),
      SPI_CS_ENC(SPI_CS_ENC),
      angle(0.0),
      velocity(0.0) {}

uint8_t A1333::read_byte(uint8_t reg)
{
    uint8_t rx_data[2];
    uint8_t tx_data[2];

    tx_data[0] = reg | 0x80;
    tx_data[1] = 0x00; // dummy

    Write_GPIO(SPI_CS_ENC, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 10);
    Write_GPIO(SPI_CS_ENC, GPIO_PIN_SET);

    return rx_data[1];
}

void A1333::write_byte(uint8_t reg, uint8_t data)
{
    uint8_t rx_data[2];
    uint8_t tx_data[2];

    tx_data[0] = reg & 0x7F;
    //   tx_data[0] = reg | 0x00;
    tx_data[1] = data; // write data

    Write_GPIO(SPI_CS_ENC, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 10);
    Write_GPIO(SPI_CS_ENC, GPIO_PIN_SET);
}

void A1333::Update()
{
}

float A1333::GetAngle()
{
    return angle;
}

float A1333::GetVelocity()
{
    return velocity;
}