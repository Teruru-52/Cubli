/*
 * controller.h
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef DRIVER_CONTROLLER_H_
#define DRIVER_CONTROLLER_H_

#include "main.h"
#include "common/base_classes/foc_motor.h"
#include "common/defaults.h"
#include "common/protocol.h"
#include "common/foc_utils.h"
#include "common/pid.h"
#include "common/motor_param.h"
#include "sensors/encoder.h"
#include "sensors/hall_sensor.h"
#include "drivers/drv8323.h"
#include <algorithm>

#define max_cali_count 5000
#define adc_resolution 4095

#define Sign(val) (((val) > 0) - ((val) < 0))

using namespace protocol;
using namespace std;

class DriverControllerBase
{
protected:
    BLDC_PWM *bldc_pwm;
    A1333 *encoder;
    DRV8323 *driver;
    HallSensor *hall;

    arm_pid_instance_f32 pid_id = {0.0, 0.0, 0.0, {0.0}, 0.0, 0.0, 0.0};
    arm_pid_instance_f32 pid_iq = {0.0, 0.0, 0.0, {0.0}, 0.0, 0.0, 0.0};
    // PID _pid_id = PID(0.1, 0.1, 0.0, 0.0, 1.0 / static_cast<float>(PWM_FREQUENCY), 5.0);
    // PID _pid_iq = PID(0.1, 0.1, 0.0, 0.0, 1.0 / static_cast<float>(PWM_FREQUENCY), 5.0);
    PID _pid_id = PID(DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, 0.0, 0.001f, 5.0f);
    PID _pid_iq = PID(DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, 0.0, 0.001f, 5.0f);

    // arm_pid_instance_f32 pid_vel; // need to be deleted
    PID _pid_vel = PID(DEF_PID_VEL_P, DEF_PID_VEL_I, DEF_PID_VEL_D, 0.0, 1.0 / static_cast<float>(PWM_FREQUENCY), 100.0);
    // PID _pid_vel = PID(0.02, 0.2, 0.0, 0.0, 0.001f, 100.0);

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

    uvw_t current_uvw;        // [A]
    uvw_t current_offset;     // [A]
    ab_t current_ab;          // [A]
    dq_t current_dq;          // [A]
    uvw_t voltage_uvw;        // [V]
    ab_t voltage_ab;          // [V]
    dq_t voltage_dq;          // [V]
    float disturbance = 0.0f; // [V]
    uvw_t input_pulsewidth;   // [s]
    uvw_t input_duty;
    dq_t ref_current_dq; // [A]

    // electric angle and angular velocity
    float theta_e = 0.0f;
    float zero_theta_e = 0.0f; // zero electric angle
    float theta_e_offset = 0.0f;
    // float omega_e = 0.0f;

    // mechanical angle and angular velocity
    // float theta_m = 0.0;
    float omega_m = 0.0;

    float target = 50.0f;

    bool adc_calibrated = false;
    bool initialized = false;
    const float Vref = 3.3f;     // [V]
    const float Rsense = 0.005f; // [Ω]
    // const float Gcsa = 20.0f;
    const float Gcsa = 40.0f;
    const float Gcsa_calib = 40.0f;
    const int phase_num = 3; // U, V, W
    uint32_t adc_data[3] = {0, 0, 0};

    float voltage_power_supply = 12.0f; // power supply voltage
    float voltage_limit = 12.0f;        // limiting voltage set to the motor

    float d1, d2, d3, d4, d5, d6, d7;
    const float zero_vector_param = 1.0f;

    void SearchZeroElectricAngle();
    float GetElectricAngle();
    uvw_t GetPhaseCurrents();
    dq_t GetDQCurrents();
    uint8_t GetSector();
    void SetPhaseVoltage(float Uq, float Ud, float angle_el);
    void AngleOpenLoop(float target_angle);

public:
    // pwm modulation related variables
    FOCModulationType foc_modulation; //!<  parameter determining modulation algorithm
    // configuration structures
    TorqueControlType torque_controller; //!< parameter determining the torque control type
    MotionControlType controller;        //!< parameter determining the control loop to be used

    explicit DriverControllerBase(BLDC_PWM *bldc_pwm, A1333 *encoder, DRV8323 *drv, HallSensor *hall) : bldc_pwm(bldc_pwm), encoder(encoder), driver(drv), hall(hall) {};
    void Initialize();
    void Enable();
    void Disable();
    void UpdateSensorAngle();
    void SetCurrentoffsets();
    void UpdateReference(float new_target) { target = new_target; };
    void Update();
    void UpdateFOC();
    void Move();
    void SetPwm();
    void SetPwm(float Vu, float Vv, float Vw);
    void Stop();
    void Free();
    bool GetCalibrationFlag() { return adc_calibrated; };
    bool GetInitilizationFlag() { return initialized; };
    void PrintLog();
    virtual ~DriverControllerBase() {}
};

class VelocityDriver : public DriverControllerBase
{
public:
    explicit VelocityDriver(BLDC_PWM *bldc_pwm, A1333 *encoder, DRV8323 *drv, HallSensor *hall) : DriverControllerBase(bldc_pwm, encoder, drv, hall) {};
};

class TorqueDriver : public DriverControllerBase
{
public:
    explicit TorqueDriver(BLDC_PWM *bldc_pwm, A1333 *encoder, DRV8323 *drv, HallSensor *hall) : DriverControllerBase(bldc_pwm, encoder, drv, hall) {};
};

#endif // DRIVER_CONTROLLER_H_