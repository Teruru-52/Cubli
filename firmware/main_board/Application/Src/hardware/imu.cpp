/*
 * imu.cpp
 *
 *  Created on: June 16th, 2023
 *      Author: Reiji Terunuma
 */

#include "hardware/imu.h"

namespace hardware
{
    // explicit IMUBase::IMUBase()
    //     :

    uint8_t IMUBase::read_byte(uint8_t reg)
    {
        uint8_t rx_data[2];
        uint8_t tx_data[2];

        tx_data[0] = reg | 0x80;
        tx_data[1] = 0x00; // dummy

        // Write_GPIO(SPI_CS, GPIO_PIN_RESET);
        // HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 10);
        // Write_GPIO(SPI_CS, GPIO_PIN_SET);

        return rx_data[1];
    }

    void IMUBase::write_byte(uint8_t reg, uint8_t data)
    {
        uint8_t rx_data[2];
        uint8_t tx_data[2];

        tx_data[0] = reg & 0x7F;
        //   tx_data[0] = reg | 0x00;
        tx_data[1] = data; // write data

        // Write_GPIO(SPI_CS, GPIO_PIN_RESET);
        // HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 10);
        // Write_GPIO(SPI_CS, GPIO_PIN_SET);
    }

    void IMUBase::Update()
    {
        UpdateGyro();
        UpdateAcc();
    }

    void IMUBase::UpdateGyro()
    {
        int16_t gyro_raw;

        // H:8bit shift, Link h and l
        gyro_raw = (int16_t)((int16_t)(read_byte(0x47) << 8) | read_byte(0x48));
        gyro.x = (float)(gyro_raw) / gyro_factor * M_PI / 180.0f - gyro_offset.x; // dps to deg/sec

        // H:8bit shift, Link h and l
        gyro_raw = (int16_t)((int16_t)(read_byte(0x47) << 8) | read_byte(0x48));
        gyro.y = (float)(gyro_raw) / gyro_factor * M_PI / 180.0f - gyro_offset.y; // dps to deg/sec

        // H:8bit shift, Link h and l
        gyro_raw = (int16_t)((int16_t)(read_byte(0x47) << 8) | read_byte(0x48));
        gyro.z = (float)(gyro_raw) / gyro_factor * M_PI / 180.0f - gyro_offset.z; // dps to deg/sec
    }

    void IMUBase::UpdateAcc()
    {
        int16_t acc_raw;

        // H:8bit shift, Link h and l
        acc_raw = (int16_t)((int16_t)(read_byte(0x3D) << 8) | read_byte(0x3E));
        acc.x = (float)(acc_raw) / acc_factor - acc_offset.x;

        // H:8bit shift, Link h and l
        acc_raw = (int16_t)((int16_t)(read_byte(0x3D) << 8) | read_byte(0x3E));
        acc.y = (float)(acc_raw) / acc_factor - acc_offset.y;

        // H:8bit shift, Link h and l
        acc_raw = (int16_t)((int16_t)(read_byte(0x3D) << 8) | read_byte(0x3E));
        acc.z = (float)(acc_raw) / acc_factor - acc_offset.z;
    }

    void IMUBase::Calibrate()
    {
        Pose data_sum;
        for (int i = 0; i < cali_count; i++)
        {
            UpdateGyro();
            data_sum.x += gyro.x;
            data_sum.y += gyro.y;
            data_sum.z += gyro.z;
            HAL_Delay(1);
        }
        gyro_offset.x = data_sum.x / static_cast<float>(cali_count);
        gyro_offset.y = data_sum.y / static_cast<float>(cali_count);
        gyro_offset.z = data_sum.z / static_cast<float>(cali_count);

        data_sum.clear();
        for (int i = 0; i < cali_count; i++)
        {
            UpdateAcc();
            data_sum.x += acc.x;
            data_sum.y += acc.y;
            data_sum.z += acc.z;
            HAL_Delay(1);
        }
        acc_offset.x = data_sum.x / static_cast<float>(cali_count);
        acc_offset.y = data_sum.y / static_cast<float>(cali_count);
        acc_offset.z = data_sum.z / static_cast<float>(cali_count);
    }

    // float IMUBase::GetAngularVelocity() const
    // {
    //     return gz;
    // }

    // float IMUBase::GetAcceleration() const
    // {
    //     return ax;
    // }

    void MPU6500::Initialize()
    {
        uint8_t who_am_i;
        // Write_GPIO(SPI_CS, GPIO_PIN_SET);
        // __HAL_SPI_ENABLE(&hspi1); // clockが動かないように、あらかじめEnableにしておく

        HAL_Delay(100);                          // wait start up
        who_am_i = read_byte(0x75);              // read who am i
        printf("who_am_i = 0x%x\r\n", who_am_i); // check who am i value
        HAL_Delay(10);
        while (who_am_i != 0x70)
        {
            who_am_i = read_byte(0x75);
            printf("who_am_i = 0x%x\r\n", who_am_i);
            HAL_Delay(20);
        }

        HAL_Delay(50);
        write_byte(0x6B, 0x00); // set pwr_might (20MHz)
        HAL_Delay(50);
        write_byte(0x1A, 0x00); // set config (FSYNCはNC)
        HAL_Delay(50);
        write_byte(0x1B, 0x18); // set gyro config (2000dps)
        HAL_Delay(50);
        write_byte(0x1C, 0x08); // set acc config (4g)
        HAL_Delay(50);
        // write_byte(0x1D, 0x00); // LPF (Accelerometer, Bandwidth460 Hz)
        // HAL_Delay(50);
    }
}