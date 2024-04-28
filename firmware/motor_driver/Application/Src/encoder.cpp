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

// void A1333::Initialize()
// {
//     Update();
//     angle_base = angle;
// }

void A1333::Update()
{
    int16_t angle_int16t = ReadByte(ANGLE_OUT_H, ANGLE_OUT_L) & 0x0FFF;
    angle_raw = M_2PI * static_cast<float>(angle_int16t) / 4095.0;

    // angle difference correction
    angle_diff = angle_raw - pre_angle_raw;
    if (angle_diff > angle_diff_max)
    {
        angle_diff -= M_2PI;
        angle_offset -= M_2PI;
    }
    else if (angle_diff < -angle_diff_max)
    {
        angle_diff += M_2PI;
        angle_offset += M_2PI;
    }
    if (abs(angle_diff) < angle_diff_min)
        angle_diff = 0;

    angle = angle_raw + angle_offset;         // angle correction [0~2pi) to (-inf~inf)
    float velocity_raw = angle_diff * inv_dt; // differentiation by euler method
    // velocity = vel_filt.Update(velocity_raw); // digital filter
    velocity = velocity_raw; // no filter
    pre_angle_raw = angle_raw;

    // printf("angle_int16t = %d\n", angle_int16t);
    // printf("angle_raw = %.3f\n", angle_raw);
}

void A1333::LogPrint()
{
    printf("angle = %.3f, angle_base = %.3f, angle_raw = %.3f, velocty = %.3f\n", angle, angle_base, angle_raw, velocity);
}