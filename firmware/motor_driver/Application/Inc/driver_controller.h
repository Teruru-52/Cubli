/*
 * controller.h
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef DRIVER_CONTROLLER_H_
#define DRIVER_CONTROLLER_H_

#include "main.h"
#include "protocol.h"
#include "state_struct.h"
#include "motor_param.h"
#include "encoder.h"
#include "drv8323.h"
#include "hall_sensor.h"

#define max_cali_count 5000
#define adc_resolution 4096

#define Sign(val) (((val) > 0) - ((val) < 0))

class DriverControllerBase
{
protected:
    A1333 *encoder;
    DRV8323 *drv8323;
    HallSensor *hall;

    arm_pid_instance_f32 pid_id = {0.0, 0.0, 0.0, {0.0}, 0.0, 0.0, 0.0};
    arm_pid_instance_f32 pid_iq = {0.0, 0.0, 0.0, {0.0}, 0.0, 0.0, 0.0};

    // coefficient of IIR filter
    float32_t b0{0.1};
    float32_t b1{0.1};
    float32_t b2{0.1};
    float32_t a1{0.1};
    float32_t a2{0.1};
    float32_t iir_statebuff_d[2];
    float32_t iir_statebuff_q[2];
    float32_t iir_coeff_d[5]{b0, b1, b2, -a1, -a2};
    float32_t iir_coeff_q[5]{b0, b1, b2, -a1, -a2};
    arm_biquad_cascade_df2T_instance_f32 iir_id;
    arm_biquad_cascade_df2T_instance_f32 iir_iq;

    uvw_t current_uvw;      // [A]
    uvw_t current_offset;   // [A]
    ab_t current_ab;        // [A]
    dq_t current_dq;        // [A]
    ab_t voltage_ab;        // [V]
    dq_t voltage_dq;        // [V]
    float disturbance{0};   // [V]
    uvw_t input_pulsewidth; // [s]
    uvw_t input_duty;
    dq_t ref_current_dq; // [A]
    uint8_t hallstate{0x00};
    uint8_t pre_hallstate{0x00};

    // electric angle and electric angular velocity in motor
    float theta_e{0.0};
    // float omega_e{0.0};

    // mechanical angle and mechanical angular velocity in motor
    // float theta_m{0.0};
    float omega_m{0.0};

    uint32_t cali_count{0};
    bool adc_calibration{true};
    const float Vref{3.3};     // [V]
    const float Rsense{0.005}; // [Ω]
    const float Gcsa{20.0};

    // coefficient of Park and Clarke transform
    const float zet{sqrt(2.0 / 3.0)};
    const float cos23{cos((2.0 / 3.0) * M_PI)};
    const float cos43{cos((4.0 / 3.0) * M_PI)};
    const float sin23{sin((2.0 / 3.0) * M_PI)};
    const float sin43{sin((4.0 / 3.0) * M_PI)};

    // coefficient of SVPWM
    const float sq3{1.0 / sqrt(3.0)};
    const float sq32{sqrt(3.0 / 2.0)};
    const float sq23{2.0 / sqrt(3.0)};
    float d1, d2, d3, d4, d5, d6, d07;
    float zero_vector_param{1.0};

    void CurrentControlInit();
    uvw_t CalculateCurrent(uint32_t *ADC_Data);
    uvw_t GetCurrent();
    ab_t ClarkeTransform(const uvw_t &current_uvw);
    dq_t ParkTransform(const ab_t &current_ab);
    // uvw_t InvClarkeTransform(const ab_t &voltage_ab);
    ab_t InvParkTransform(const dq_t &voltage_dq);
    uvw_t CalculateSVPWM(const ab_t &voltage_ab);
    float CalibrateElectricAngle(float theta_e, float omega_m);
    void ResetBase();

public:
    explicit DriverControllerBase(A1333 *encoder, DRV8323 *drv8323, HallSensor *hall) : encoder(encoder), drv8323(drv8323), hall(hall){};
    virtual void Initialize() = 0;
    void SetCurrentoffset();
    bool GetCalibrationFlag();
    virtual uvw_t Control() = 0;
    virtual void Reset() = 0;
    virtual ~DriverControllerBase() {}
};

class VelocityDriver : public DriverControllerBase
{
private:
    arm_pid_instance_f32 pid_vel = {0.0, 0.0, 0.0, {0.0}, 0.0, 0.0, 0.0};

    float ref_vel{0.0};

public:
    explicit VelocityDriver(A1333 *encoder, DRV8323 *drv8323, HallSensor *hall) : DriverControllerBase(encoder, drv8323, hall){};
    void Initialize() override;
    uvw_t Control() override;
    void Reset() override;
};

class TorqueDriver : public DriverControllerBase
{
private:
    // arm_pid_instance_f32 pid_torque = {0.0, 0.0, 0.0, {0.0}, 0.0, 0.0, 0.0};

    float ref_torque{0.0};

public:
    explicit TorqueDriver(A1333 *encoder, DRV8323 *drv8323, HallSensor *hall) : DriverControllerBase(encoder, drv8323, hall){};
    void Initialize() override;
    uvw_t Control() override;
    void Reset() override;
};

#endif // DRIVER_CONTROLLER_H_