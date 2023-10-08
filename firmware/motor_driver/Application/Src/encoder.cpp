/*
 * encoder.cpp
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#include "encoder.h"

A1333::A1333(SPI_HandleTypeDef *spi, GPIO_Value SPI_CS_ENC)
    : spi(spi),
      SPI_CS_ENC(SPI_CS_ENC),
      angle(0.0),
      velocity(0.0)
{
}

uint16_t A1333::Read_byte(uint8_t reg1, uint8_t reg2)
{
    uint8_t rx_data[2];
    uint8_t tx_data[2];

    tx_data[0] = reg1;
    tx_data[1] = reg2;

    Write_GPIO(SPI_CS_ENC, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(spi, tx_data, rx_data, 2, 10);
    Write_GPIO(SPI_CS_ENC, GPIO_PIN_SET);

    uint16_t data = (int16_t)((int16_t)(rx_data[0] << 8) | rx_data[1]);
    return data;
}

void A1333::Update()
{
    angle_raw = Read_byte(0x20, 0x21) & 0x0FFF;
    // printf("angle_raw = %d\n", angle_raw);
    angle = (2.0 * M_PI) * static_cast<float>(angle_raw) / 4095.0;
}