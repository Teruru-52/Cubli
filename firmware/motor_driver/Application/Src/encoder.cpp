/*
 * encoder.cpp
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#include "encoder.h"

uint16_t A1333::ReadByte(uint8_t reg1, uint8_t reg2)
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

void A1333::Initialize()
{
    Write_GPIO(SPI_CS_ENC, GPIO_PIN_SET);
    Update();
    pre_angle_full = angle_full;
}

void A1333::Update()
{
    int16_t angle_int16t = ReadByte(ANGLE_OUT_H, ANGLE_OUT_L) & 0x0FFF;
    angle_raw = M_2PI * static_cast<float>(angle_int16t) / 4095.0;

    // angle difference correction
    float angle_diff = angle_raw - pre_angle_raw;
    if (abs(angle_diff) > angle_diff_max)
        angle_full += (angle_diff > 0) ? -M_2PI : M_2PI; // angle correction [0~2pi) to (-inf~inf)
    if (abs(angle_diff) < angle_diff_min)
        angle_diff = 0;
    angle_full += angle_diff;
    pre_angle_raw = angle_raw;
}

float A1333::GetVelocity()
{
    float velocity_raw = (angle_full - pre_angle_full) * inv_dt; // differentiation by euler method
    // velocity = vel_filt.Update(velocity_raw); // digital filter
    velocity = velocity_raw; // no filter
    pre_angle_full = angle_full;
    return velocity;
}

void A1333::LogPrint()
{
    printf("angle = %.3f, angle_raw = %.3f, velocity = %.3f\n", angle_full, angle_raw, velocity);
}