/*
 * drv8323.cpp
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#include "drv8323.h"

DRV8323::DRV8323(SPI_HandleTypeDef *spi, GPIO_Value SPI_CS_DRV, GPIO_Value nFault, GPIO_Value DRV_ENABLE, GPIO_Value DRV_CAL, GPIO_Value INLx)
    : spi(spi),
      SPI_CS_DRV(SPI_CS_DRV),
      nFault(nFault),
      DRV_ENABLE(DRV_ENABLE),
      DRV_CAL(DRV_CAL),
      INLx(INLx)
{
}

uint8_t DRV8323::read_byte(uint8_t reg)
{
    uint8_t rx_data[2];
    uint8_t tx_data[2];

    tx_data[0] = reg | 0x80;
    tx_data[1] = 0x00; // dummy

    Write_GPIO(SPI_CS_DRV, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(spi, tx_data, rx_data, 2, 10);
    Write_GPIO(SPI_CS_DRV, GPIO_PIN_SET);

    return rx_data[1];
}

void DRV8323::write_byte(uint8_t reg, uint8_t data)
{
    uint8_t rx_data[2];
    uint8_t tx_data[2];

    tx_data[0] = reg & 0x7F;
    //   tx_data[0] = reg | 0x00;
    tx_data[1] = data; // write data

    Write_GPIO(SPI_CS_DRV, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(spi, tx_data, rx_data, 2, 10);
    Write_GPIO(SPI_CS_DRV, GPIO_PIN_SET);
}

void DRV8323::Initialize()
{
    Write_GPIO(SPI_CS_DRV, GPIO_PIN_SET);
    __HAL_SPI_ENABLE(&hspi1); // clockが動かないように、あらかじめEnableにしておく
    // SET_BIT(spi->CR1, SPI_CR1_SPE); // clockが動かないように、あらかじめEnableにしておく

    // HAL_Delay(50);
    // write_byte(0x02, 0x00); // Driver Control Register
    // HAL_Delay(50);
    // write_byte(0x03, 0x00); // Gate Drive HS Register
    // HAL_Delay(50);
    // write_byte(0x04, 0x00); // Driver Drive LS Register
    // HAL_Delay(50);
    // write_byte(0x05, 0x00); // OCP Control Register
    // HAL_Delay(50);
    // write_byte(0x06, 0x00); // CSA Control Register

    Write_GPIO(DRV_ENABLE, GPIO_PIN_SET);
    Write_GPIO(DRV_CAL, GPIO_PIN_SET);
}

void DRV8323::CheckFaultStatus()
{
    fault_status1 = read_byte(0x00); // Fault Status Register1
    fault_status2 = read_byte(0x01); // Fault Status Register2

    // if (Read_GPIO(nFault))
    // {
    // }
}

void DRV8323::Stop()
{
    Write_GPIO(INLx, GPIO_PIN_RESET);
}