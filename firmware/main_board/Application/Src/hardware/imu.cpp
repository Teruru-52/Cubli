/*
 * imu.cpp
 *
 *  Created on: June 16th, 2023
 *      Author: Reiji Terunuma
 */

#include "hardware/imu.h"

#define WHO_AM_I 0x75
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define PWR_MGMT_1 0x6B

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

namespace hardware
{
    void IMUBase::Update()
    {
        UpdateGyro();
        UpdateAcc();
    }

    void IMUBase::UpdateGyro()
    {
        int16_t gyro_raw;

        // H:8bit shift, Link h and l
        gyro_raw = (int16_t)((int16_t)(Read_1byte(spi_imu, GYRO_XOUT_H) << 8) | Read_1byte(spi_imu, GYRO_XOUT_L));
        gyro.x = (float)(gyro_raw) / gyro_factor * M_PI / 180.0f - gyro_offset.x; // dps to deg/sec

        // H:8bit shift, Link h and l
        gyro_raw = (int16_t)((int16_t)(Read_1byte(spi_imu, GYRO_YOUT_H) << 8) | Read_1byte(spi_imu, GYRO_YOUT_L));
        gyro.y = (float)(gyro_raw) / gyro_factor * M_PI / 180.0f - gyro_offset.y; // dps to deg/sec

        // H:8bit shift, Link h and l
        gyro_raw = (int16_t)((int16_t)(Read_1byte(spi_imu, GYRO_ZOUT_H) << 8) | Read_1byte(spi_imu, GYRO_ZOUT_L));
        gyro.z = (float)(gyro_raw) / gyro_factor * M_PI / 180.0f - gyro_offset.z; // dps to deg/sec
    }

    void IMUBase::UpdateAcc()
    {
        int16_t acc_raw;

        // H:8bit shift, Link h and l
        acc_raw = -(int16_t)((int16_t)(Read_1byte(spi_imu, ACCEL_XOUT_H) << 8) | Read_1byte(spi_imu, ACCEL_XOUT_L));
        acc.x = (float)(acc_raw) / acc_factor - acc_offset.x;

        // H:8bit shift, Link h and l
        acc_raw = -(int16_t)((int16_t)(Read_1byte(spi_imu, ACCEL_YOUT_H) << 8) | Read_1byte(spi_imu, ACCEL_YOUT_L));
        acc.y = (float)(acc_raw) / acc_factor - acc_offset.y;

        // H:8bit shift, Link h and l
        acc_raw = -(int16_t)((int16_t)(Read_1byte(spi_imu, ACCEL_ZOUT_H) << 8) | Read_1byte(spi_imu, ACCEL_ZOUT_L));
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

        // printf("%.3f, %.3f, %.3f\n", gyro_offset.x, gyro_offset.y, gyro_offset.z);

        // data_sum.clear();
        // for (int i = 0; i < cali_count; i++)
        // {
        //     UpdateAcc();
        //     data_sum.x += acc.x;
        //     data_sum.y += acc.y;
        //     data_sum.z += acc.z;
        //     HAL_Delay(1);
        // }
        // acc_offset.x = data_sum.x / static_cast<float>(cali_count);
        // acc_offset.y = data_sum.y / static_cast<float>(cali_count);
        // acc_offset.z = data_sum.z / static_cast<float>(cali_count);

        // printf("%.3f, %.3f, %.3f\n", acc_offset.x, acc_offset.y, acc_offset.z);
    }

    void MPU6500::Initialize()
    {
        uint8_t who_am_i;
        Write_GPIO(*(spi_imu->SPI_CS), GPIO_PIN_SET);
        __HAL_SPI_ENABLE(spi_imu->hspi); // clockが動かないように、あらかじめEnableにしておく

        HAL_Delay(50);                            // wait start up
        who_am_i = Read_1byte(spi_imu, WHO_AM_I); // read who am i
        printf("who_am_i = 0x%x\r\n", who_am_i);  // check who am i value
        HAL_Delay(10);
        while (who_am_i != 0x70)
        {
            who_am_i = Read_1byte(spi_imu, WHO_AM_I);
            printf("who_am_i = 0x%x\r\n", who_am_i);
        }

        HAL_Delay(10);
        Write_1byte(spi_imu, PWR_MGMT_1, 0x00); // set clock (20MHz)
        // Write_1byte(spi_imu, PWR_MGMT_1, 0x88); // set clock (20MHz) & device reset & disable temp sensor
        HAL_Delay(10);
        Write_1byte(spi_imu, CONFIG, 0x00); // set config (FSYNCはNC)
        HAL_Delay(10);
        Write_1byte(spi_imu, GYRO_CONFIG, 0x18); // set gyro config (2000dps)
        HAL_Delay(10);
        Write_1byte(spi_imu, ACCEL_CONFIG, 0x08); // set acc config (4g)
        HAL_Delay(10);
        // write_byte(0x1D, 0x00); // LPF (Accelerometer, Bandwidth460 Hz)
        // HAL_Delay(50);
    }
}