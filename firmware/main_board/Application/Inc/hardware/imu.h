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

#define cali_count 500

namespace hardware
{
    class IMUBase
    {
    protected:
        Pose gyro_offset;
        Pose acc_offset;

        float gyro_factor = 0;
        float acc_factor = 0;

        void UpdateGyro();
        void UpdateAcc();
        void Calibrate();
        float GetAngularVelocity() const;
        float GetAcceleration() const;
        void Reset();

        uint8_t read_byte(uint8_t reg);
        void write_byte(uint8_t reg, uint8_t data);

    public:
        Pose gyro;
        Pose acc;

        explicit IMUBase() {}
        void Update();
        virtual void Initialize() = 0;
        virtual ~IMUBase() {}
    };

    class MPU6500 : public IMUBase
    {
    private:
        // SPI_Value *spi_value;
        float sampling_period; // [s]

        const float gyro_factor = 16.6;
        const float acc_factor = 8192.0;
        float offset_gz = 0.0;

    public:
        // MPU6500(SPI_Value *spi_value, float sampling_period);
        explicit MPU6500() : IMUBase(){};
        void Initialize() override;
    };
} // namespace hardware

#endif // IMU_H_