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
#include "tim.h"
#include "common/digital_filter.h"

#define ANGLE_OUT_H 0x20
#define ANGLE_OUT_L 0x21
#define ERR_FLAG_H 0x24
#define ERR_FLAG_L 0x25
#define M_2PI 6.28318530717958647692
#define M_PI_3 1.04719755119659774615

class A1333
{
protected:
    SPI_HandleTypeDef *hspi;
    GPIO_Value SPI_CS_ENC;
    // MovingAverageFilter vel_filt = MovingAverageFilter(100);
    MedianFilter vel_filt = MedianFilter(7);

    // float dt = 1.0f / static_cast<float>(PWM_FREQUENCY); // need to be changed
    const float dt = 0.001f; // need to be changed
    float inv_dt = 1.0f / dt;
    float angle_raw = 0.0f; // raw angle of encoder output [rad]
    float pre_angle_raw = 0.0f;
    float angle_full = 0.0f;
    float pre_angle_full = 0.0f;
    float angle_base = 0.0f;
    const float angle_diff_min = 0.001535f;   // M_2PI / 4095 = 0.001534... [rad]
    const float angle_diff_max = 1.5f * M_PI; // 270 [deg]
    float velocity = 0.0f;
    // uint16_t flag_err;

public:
    A1333(SPI_HandleTypeDef *hspi, GPIO_Value SPI_CS_ENC) : hspi(hspi),
                                                            SPI_CS_ENC(SPI_CS_ENC) {}

    int16_t ReadByte(uint8_t reg1, uint8_t reg2);

    void Initialize();
    void Update();
    uint16_t GetErrFlag() { return ReadByte(ERR_FLAG_H, ERR_FLAG_L); }
    float GetMechanicalAngle() { return angle_raw; }
    float GetAngle() { return angle_full; }
    float GetAngleDiff() { return angle_full - angle_base; }
    float GetVelocity() { return velocity; };
    void PrintLog();
};

#endif // ENCODER_H_