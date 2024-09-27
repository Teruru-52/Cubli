/*
 * foc_driver.h
 *
 *  Created on: Sep 22th, 2024
 *      Author: Reiji Terunuma
 */

#ifndef FOC_MOTOR_H_
#define FOC_MOTOR_H_

#include "main.h"
#include "common/foc_utils.h"
#include "common/defaults.h"
#include "common/pid.h"
#include "common/digital_filter.h"

/**
 *  Motiron control type
 */
enum MotionControlType : uint8_t
{
    torque = 0x00,   //!< Torque control
    velocity = 0x01, //!< Velocity motion control
};

/**
 *  Torque control type
 */
enum TorqueControlType : uint8_t
{
    voltage = 0x00,     //!< Torque control using voltage
    foc_current = 0x01, //!< torque control using dq currents
};

/**
 *  FOC modulation type
 */
enum FOCModulationType : uint8_t
{
    SpaceVectorPWM = 0x00, //!< Space vector modulation method
    Trapezoid_120 = 0x01,
};

// enum FOCMotorStatus : uint8_t
// {
//     motor_uninitialized = 0x00, //!< Motor is not yet initialized
//     motor_initializing = 0x01,  //!< Motor intiialization is in progress
//     motor_uncalibrated = 0x02,  //!< Motor is initialized, but not calibrated (open loop possible)
//     motor_calibrating = 0x03,   //!< Motor calibration in progress
//     motor_ready = 0x04,         //!< Motor is initialized and calibrated (closed loop possible)
//     motor_error = 0x08,         //!< Motor is in error state (recoverable, e.g. overcurrent protection active)
//     motor_calib_failed = 0x0E,  //!< Motor calibration failed (possibly recoverable)
//     motor_init_failed = 0x0F,   //!< Motor initialization failed (not recoverable)
// };

/**
 Generic motor class
*/
class FOCMotor
{
public:
    /**
     * Default constructor - setting all variabels to default values
     */
    FOCMotor();

    virtual void Initialize() = 0;
    virtual void Enable() = 0;
    virtual void Disable() = 0;
    virtual void UpdateSensorAngle() = 0;
    void UpdateTarget(float new_target) { target = new_target; }
    virtual void Update() = 0;
    virtual void UpdateFOC() = 0;
    virtual void Move() = 0;
    virtual void SetPhaseVoltage(float Uq, float Ud, float angle_el) = 0;
    virtual float GetElectricAngle() = 0;
    virtual dq_t GetDQCurrents() = 0;
    virtual uint8_t GetSector() = 0;

    virtual void SetPwm() = 0;
    virtual void SetPwm(float Vu, float Vv, float Vw) = 0;
    int8_t Calibrated() { return adc_calibrated; };
    int8_t Initilized() { return initialized; };
    virtual void PrintLog() = 0;

    arm_pid_instance_f32 pid_id{0.0, 0.0, 0.0, {0.0}, 0.0, 0.0, 0.0};
    arm_pid_instance_f32 pid_iq{0.0, 0.0, 0.0, {0.0}, 0.0, 0.0, 0.0};
    // PID _pid_id{0.1, 0.1, 0.0, 0.0, 1.0f / static_cast<float>(PWM_FREQUENCY), 5.0};
    // PID _pid_iq{0.1, 0.1, 0.0, 0.0, 1.0f / static_cast<float>(PWM_FREQUENCY), 5.0};
    PID _pid_id{DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, 0.0, 0.001f, 5.0f};
    PID _pid_iq{DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, 0.0, 0.001f, 5.0f};

    // arm_pid_instance_f32 pid_vel; // need to be deleted
    PID _pid_vel{DEF_PID_VEL_P, DEF_PID_VEL_I, DEF_PID_VEL_D, 0.0, 1.0f / static_cast<float>(PWM_FREQUENCY), 100.0};
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

    //!<  parameter determining the current Low pass filter configuration
    LowPassFilter lpf_current_q{DEF_CURR_FILTER_Tf, 1.0f / static_cast<float>(PWM_FREQUENCY)};
    LowPassFilter lpf_current_d{DEF_CURR_FILTER_Tf, 1.0f / static_cast<float>(PWM_FREQUENCY)};

    uvw_t current_uvw;        // [A]
    ab_t current_ab;          // [A]
    dq_t current_dq;          // [A]
    uvw_t voltage_uvw;        // [V]
    ab_t voltage_ab;          // [V]
    dq_t voltage_dq;          // [V]
    float disturbance = 0.0f; // [V]
    uvw_t input_pulsewidth;   // [s]
    uvw_t input_duty;
    dq_t ref_current_dq; // [A]
    float voltage_bemf;  //!< estimated backemf voltage (if provided KV constant)

    // electric angle and angular velocity
    float electric_angle = 0.0f;
    float zero_electric_angle = NOT_SET; // zero electric angle

    // mechanical angular velocity
    // float shaft_angle = 0.0f;
    float shaft_velocity = 0.0f;

    float target = 0.0f;

    float voltage_power_supply = 12.0f; // power supply voltage

    // motor configuration parameters
    float voltage_sensor_align;  //!< sensor and motor align voltage parameter
    float velocity_index_search; //!< target velocity for index search

    // motor physical parameters
    // float phase_resistance = 64e-3f;  //!< motor phase resistance
    float phase_resistance = NOT_SET; //!< motor phase resistance
    float pole_pairs = 8.0f;          //!< motor pole pairs number
    float KV_rating = NOT_SET;        //!< motor KV rating
    // float phase_inductance = 0.5e-3f; //!< motor phase inductance
    float phase_inductance = NOT_SET; //!< motor phase inductance

    // limiting variables
    float voltage_limit;  //!< Voltage limiting variable - global limit
    float current_limit;  //!< Current limiting variable - global limit
    float velocity_limit; //!< Velocity limiting variable - global limit

    // motor status vairables
    int8_t enabled = 0; //!< enabled or disabled motor flag
    int8_t adc_calibrated = 0;
    int8_t initialized = 0;
    // FOCMotorStatus motor_status = FOCMotorStatus::motor_uninitialized; //!< motor status

    // pwm modulation related variables
    FOCModulationType foc_modulation; //!<  parameter determining modulation algorithm
    int8_t modulation_centered = 1;   //!< flag (1) centered modulation around driver limit /2  or  (0) pulled to 0

    // configuration structures
    TorqueControlType torque_controller; //!< parameter determining the torque control type
    MotionControlType controller;        //!< parameter determining the control loop to be used
};

#endif // FOC_MOTOR_H_