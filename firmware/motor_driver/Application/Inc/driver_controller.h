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
#include "pid_controller.h"

#define max_cali_count 5000
#define adc_resolution 4095

#define Sign(val) (((val) > 0) - ((val) < 0))

class DriverControllerBase
{
protected:
    A1333 *encoder;
    DRV8323 *drv;
    HallSensor *hall;

    arm_pid_instance_f32 pid_id = {0.0, 0.0, 0.0, {0.0}, 0.0, 0.0, 0.0};
    arm_pid_instance_f32 pid_iq = {0.0, 0.0, 0.0, {0.0}, 0.0, 0.0, 0.0};
    PID _pid_id = PID(0.1, 0.1, 0.0, 0.0, 1.0 / static_cast<float>(PWM_FREQUENCY), 5.0);
    PID _pid_iq = PID(0.1, 0.1, 0.0, 0.0, 1.0 / static_cast<float>(PWM_FREQUENCY), 5.0);

    // coefficient of IIR filter
    float32_t b0 = 0.1f;
    float32_t b1 = 0.1f;
    float32_t b2 = 0.1f;
    float32_t a1 = 0.1f;
    float32_t a2 = 0.1f;
    float32_t iir_statebuff_d[2];
    float32_t iir_statebuff_q[2];
    float32_t iir_coeff_d[5] = {b0, b1, b2, -a1, -a2};
    float32_t iir_coeff_q[5] = {b0, b1, b2, -a1, -a2};
    arm_biquad_cascade_df2T_instance_f32 iir_id;
    arm_biquad_cascade_df2T_instance_f32 iir_iq;
    // lowpass filter
    float tau = 1.0f / (2.0f * M_PI * 10000.0f);
    float T = 1.0f / static_cast<float>(PWM_FREQUENCY);
    float coeff_lp = tau / (tau + T);
    dq_t pre_current_dq;

    uvw_t current_uvw;       // [A]
    uvw_t current_offset;    // [A]
    ab_t current_ab;         // [A]
    dq_t current_dq;         // [A]
    ab_t voltage_ab;         // [V]
    dq_t voltage_dq;         // [V]
    float disturbance = 0.0; // [V]
    uvw_t input_pulsewidth;  // [s]
    uvw_t input_duty;
    float input_trape;   // common input for trapezoidal control
    dq_t ref_current_dq; // [A]
    uint8_t hall_state = 0x00;
    uint8_t pre_hall_state = 0x00;

    // electric angle and angular velocity
    float theta_e = 0.0;
    float theta_e_base = 0.0;
    float theta_e_offset = 0.0;
    // float omega_e = 0.0;

    // mechanical angle and angular velocity
    // float theta_m = 0.0;
    float omega_m = 0.0;

    bool adc_calibrated = false;
    bool initialized = false;
    const float Vref = 3.3f;     // [V]
    const float Rsense = 0.005f; // [Ω]
    // const float Gcsa = 20.0f;
    const float Gcsa = 40.0f;
    const float Gcsa_calib = 40.0f;
    const int phase_num = 3; // U, V, W

    // coefficient of Park and Clarke transform
    const float cos23 = cos((2.0f / 3.0f) * M_PI);
    const float cos43 = cos((4.0f / 3.0f) * M_PI);
    const float sin23 = sin((2.0f / 3.0f) * M_PI);
    const float sin43 = sin((4.0f / 3.0f) * M_PI);
    const float sq23 = sqrt(2.0f / 3.0f);

    // coefficient of SVPWM
    const float sq13 = 1.0f / sqrt(3.0f);
    const float sq32 = sqrt(3.0f / 2.0f);
    const float sq43 = 2.0f / sqrt(3.0f);
    float d1, d2, d3, d4, d5, d6, d7;
    const float zero_vector_param = 1.0f;

    float UpdateElectricAngle(float theta_m_diff);
    uvw_t CalculateCurrent(uint32_t *adc_data);
    uvw_t GetCurrent();
    ab_t ClarkeTransform(const uvw_t &current_uvw);
    dq_t ParkTransform(const ab_t &current_ab);
    // uvw_t InvClarkeTransform(const ab_t &voltage_ab);
    ab_t InvParkTransform(const dq_t &voltage_dq);
    uvw_t CalculateSVPWM(const ab_t &voltage_ab);
    void LimitInput();
    void LimitCurrent();
    void ResetBase();

public:
    explicit DriverControllerBase(A1333 *encoder, DRV8323 *drv, HallSensor *hall) : encoder(encoder), drv(drv), hall(hall){};
    void Initialize();
    void CorrectElectricAngle(uint8_t _hall_state);
    void UpdateSensorAngle();
    void SetCurrentoffset();
    bool GetCalibrationFlag() { return adc_calibrated; };
    bool GetInitilizationFlag() { return initialized; };
    virtual uvw_t Control() = 0;
    virtual void Reset() = 0;
    void LogPrint();
    virtual ~DriverControllerBase() {}
};

class VelocityDriver : public DriverControllerBase
{
private:
    // arm_pid_instance_f32 pid_vel; // need to be deleted
    PID _pid_vel = PID(0.01, 1.0, 0.0, 0.0, 1.0 / static_cast<float>(PWM_FREQUENCY), 5.0);

    const float ref_vel = 50.0;
    const float integral_max = 10.0;

public:
    explicit VelocityDriver(A1333 *encoder, DRV8323 *drv, HallSensor *hall) : DriverControllerBase(encoder, drv, hall){};
    uvw_t Control() override;
    void UpdateFOC();
    void UpdateTrapezoidalControl();
    void Reset() override;
};

class TorqueDriver : public DriverControllerBase
{
private:
    // arm_pid_instance_f32 pid_torque = {0.0, 0.0, 0.0, {0.0}, 0.0, 0.0, 0.0};

    float ref_torque = 0.0;

public:
    explicit TorqueDriver(A1333 *encoder, DRV8323 *drv, HallSensor *hall) : DriverControllerBase(encoder, drv, hall){};
    uvw_t Control() override;
    void Reset() override;
};

#endif // DRIVER_CONTROLLER_H_