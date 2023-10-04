/*
 * imu.h
 *
 *  Created on: June 16th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef IMU_H_
#define IMU_H_

#include "main.h"
#include "spi.h"
#include "controller/pose.h"

#define cali_count 100

namespace hardware
{
    class IMUBase
    {
    protected:
        SPI_Value *spi_imu;
        Pose gyro_offset;
        Pose acc_offset;

        const float gyro_factor = 16.6;
        const float acc_factor = 8192.0;

        void UpdateGyro();
        void UpdateAcc();
        float GetAngularVelocity() const;
        float GetAcceleration() const;
        void Reset();

    public:
        Pose gyro;
        Pose acc;

        explicit IMUBase(SPI_Value *spi_value) : spi_imu(spi_value) {}
        void Update();
        void Calibrate();
        virtual void Initialize() = 0;
        virtual ~IMUBase() {}
    };

    class MPU6500 : public IMUBase
    {
    public:
        explicit MPU6500(SPI_Value *spi_value) : IMUBase(spi_value){};
        void Initialize() override;
    };
} // namespace hardware

#endif // IMU_H_