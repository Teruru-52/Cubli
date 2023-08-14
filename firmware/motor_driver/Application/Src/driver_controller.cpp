/*
 * driver_controller.cpp
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#include "driver_controller.h"

#define CURRENT_LIMIT 20.0

using namespace protocol;

void DriverControllerBase::CurrentControlInit()
{
    arm_pid_init_f32(&pid_id, 0);
    arm_pid_init_f32(&pid_iq, 0);

    arm_biquad_cascade_df2T_init_f32(&iir_id, 1, iir_coeff_d, iir_statebuff_d);
    arm_biquad_cascade_df2T_init_f32(&iir_iq, 1, iir_coeff_q, iir_statebuff_q);
}

uvw_t DriverControllerBase::CalculateCurrent(uint32_t *ADC_Data)
{
    float Vsox[3] = {0.0, 0.0, 0.0}; // [V]
    for (int i = 0; i < 3; i++)
    {
        Vsox[i] = static_cast<float>(ADC_Data[i]) * Vref / static_cast<float>(adc_resolution);
    }
    current_uvw.u = (Vref - Vsox[0]) / (Gcsa * Rsense) - current_offset.u;
    current_uvw.v = (Vref - Vsox[1]) / (Gcsa * Rsense) - current_offset.v;
    current_uvw.w = (Vref - Vsox[2]) / (Gcsa * Rsense) - current_offset.w;

    return current_uvw;
}

uvw_t DriverControllerBase::GetCurrent()
{
    uint32_t ADC_Data[3] = {0, 0, 0};
    ADC_Get_Value(ADC_Data); // Get Digital Data
    return CalculateCurrent(ADC_Data);
}

ab_t DriverControllerBase::ClarkeTransform(const uvw_t &current_uvw)
{
    ab_t current_ab_;
    current_ab_.a = (current_uvw.u + current_uvw.v * cos23 + current_uvw.w * cos43) * zet;
    current_ab_.b = (current_uvw.v * sin23 + current_uvw.w * sin43) * zet;

    return current_ab_;
}

dq_t DriverControllerBase::ParkTransform(const ab_t &current_ab)
{
    dq_t current_dq_;
    // current_dq_.d = current_ab.a * cos(theta_m) + current_ab.b * sin(theta_m);
    // current_dq_.q = -current_ab.a * sin(theta_m) + current_ab.b * cos(theta_m);
    arm_park_f32(current_ab.a, current_ab.b, &(current_dq_.d), &(current_dq_.q), arm_sin_f32(theta_e), arm_cos_f32(theta_e));

    return current_dq_;
}

// uvw_t DriverControllerBase::InvClarkeTransform(const ab_t &voltage_ab)
// {
//     uvw_t voltage_uvw_;
//     voltage_uvw_.u = voltage_ab.a * zet;
//     voltage_uvw_.v = (voltage_ab.a * cos23 + voltage_ab.b * sin23) * zet;
//     voltage_uvw_.w = (voltage_ab.a * cos43 + voltage_ab.b * sin43) * zet;

//     return voltage_uvw_;
// }

ab_t DriverControllerBase::InvParkTransform(const dq_t &voltage_dq)
{
    ab_t voltage_ab_;
    // voltage_ab_.a = voltage_dq.d * cos(theta_m) - voltage_dq.q * sin(theta_m);
    // voltage_ab_.b = voltage_dq.d * sin(theta_m) + voltage_dq.q * cos(theta_m);
    arm_inv_park_f32(voltage_dq.d, voltage_dq.q, &(voltage_ab_.a), &(voltage_ab_.b), arm_sin_f32(theta_e), arm_cos_f32(theta_e));

    return voltage_ab_;
}

uvw_t DriverControllerBase::CalculateSVPWM(const ab_t &voltage_ab)
{
    uvw_t input_duty_;

    // Sector 0
    if ((voltage_ab.a >= 0) && (voltage_ab.b >= 0) && (abs(voltage_ab.a) >= abs(sq3 * voltage_ab.b)))
    {
        d1 = sq32 * (voltage_ab.a - sq3 * voltage_ab.b) / max_voltage;
        d2 = sq32 * (sq23 * voltage_ab.b) / max_voltage;
        d07 = (zero_vector_param - (d1 + d2)) * 0.5;
        input_duty_.u = d1 + d2 + d07;
        input_duty_.v = d2 + d07;
        input_duty_.w = d07;
    }
    // Sector 1
    if ((abs(voltage_ab.a) <= sq3 * voltage_ab.b))
    {
        d3 = sq32 * (-voltage_ab.a + sq3 * voltage_ab.b) / max_voltage;
        d2 = sq32 * (voltage_ab.a + sq3 * voltage_ab.b) / max_voltage;
        d07 = (zero_vector_param - (d2 + d3)) * 0.5;
        input_duty_.u = d2 + d07;
        input_duty_.v = d2 + d3 + d07;
        input_duty_.w = d07;
    }
    // Sector 2
    if ((voltage_ab.a <= 0) && (voltage_ab.b >= 0) && (abs(voltage_ab.a) >= abs(sq3 * voltage_ab.b)))
    {
        d3 = sq32 * sq23 * voltage_ab.b / max_voltage;
        d4 = sq32 * (-voltage_ab.a - sq3 * voltage_ab.b) / max_voltage;
        d07 = (zero_vector_param - (d3 + d4)) * 0.5;
        input_duty_.u = d07;
        input_duty_.v = d3 + d4 + d07;
        input_duty_.w = d4 + d07;
    }
    // Sector 3
    if ((voltage_ab.a <= 0) && (voltage_ab.b <= 0) && (abs(voltage_ab.a) >= abs(sq3 * voltage_ab.b)))
    {
        d5 = -sq32 * sq23 * voltage_ab.b / max_voltage;
        d4 = sq32 * (-voltage_ab.a + sq3 * voltage_ab.b) / max_voltage;
        d07 = (zero_vector_param - (d4 + d5)) * 0.5;
        input_duty_.u = d07;
        input_duty_.v = d4 + d07;
        input_duty_.w = d4 + d5 + d07;
    }
    // Sector 4
    if ((abs(voltage_ab.a) <= -sq3 * voltage_ab.b))
    {
        d5 = sq32 * (-voltage_ab.a - sq3 * voltage_ab.b) / max_voltage;
        d6 = sq32 * (voltage_ab.a - sq3 * voltage_ab.b) / max_voltage;
        d07 = (zero_vector_param - (d5 + d6)) * 0.5;
        input_duty_.u = d6 + d07;
        input_duty_.v = d07;
        input_duty_.w = d5 + d6 + d07;
    }
    // Sector 5
    if ((voltage_ab.a >= 0) && (voltage_ab.b <= 0) && (abs(voltage_ab.a) >= abs(sq3 * voltage_ab.b)))
    {
        d1 = sq32 * (voltage_ab.a + sq3 * voltage_ab.b) / max_voltage;
        d6 = -sq32 * sq23 * voltage_ab.b / max_voltage;
        d07 = (zero_vector_param - (d1 + d6)) * 0.5;
        input_duty_.u = d1 + d6 + d07;
        input_duty_.v = d07;
        input_duty_.w = d6 + d07;
    }

    return input_duty_;
}

float DriverControllerBase::CalibrateElectricAngle(float theta_e, float omega_m)
{
    float theta_e_ = theta_e;
    hallstate = hall->GetHallValue();

    if (hallstate != pre_hallstate)
    {
        if (hallstate == 0b00000100)
        {
            theta_e_ = 0.0;
        }
        else if (hallstate == 0b00000110)
        {
            theta_e_ = M_PI / 3.0;
        }
        else if (hallstate == 0b00000010)
        {
            theta_e_ = M_PI * 2.0 / 3.0;
        }
        else if (hallstate == 0b00000011)
        {
            theta_e_ = M_PI;
        }
        else if (hallstate == 0b00000001)
        {
            theta_e_ = M_PI * 4.0 / 3.0;
        }
        else if (hallstate == 0b00000101)
        {
            theta_e_ = M_PI * 5.0 / 3.0;
        }
    }
    else
    {
        theta_e_ += omega_m * Pn * max_pulsewidth;
        theta_e_ /= (2 * M_PI);
    }

    pre_hallstate = hallstate;
    return theta_e_;
}

void DriverControllerBase::ResetBase()
{
    arm_pid_reset_f32(&pid_id);
    arm_pid_reset_f32(&pid_iq);
}

void DriverControllerBase::SetCurrentoffset()
{
    uint32_t ADC_Data[3] = {0, 0, 0};
    float Vsox[3] = {0.0, 0.0, 0.0}; // [V]
    ADC_Get_Value(ADC_Data);         // Get Digital Data

    for (int i = 0; i < 3; i++)
    {
        Vsox[i] = static_cast<float>(ADC_Data[i]) * Vref / static_cast<float>(adc_resolution);
    }
    current_offset.u += (Vref - Vsox[0]) / (Gcsa * Rsense);
    current_offset.v += (Vref - Vsox[1]) / (Gcsa * Rsense);
    current_offset.w += (Vref - Vsox[2]) / (Gcsa * Rsense);

    cali_count++;

    if (cali_count >= max_cali_count)
    {
        adc_calibration = false;
        current_offset.u /= static_cast<float>(cali_count);
        current_offset.v /= static_cast<float>(cali_count);
        current_offset.w /= static_cast<float>(cali_count);
    }
}

bool DriverControllerBase::GetCalibrationFlag()
{
    return adc_calibration;
}

void VelocityDriver::Initialize()
{
    CurrentControlInit();
    theta_e = CalibrateElectricAngle(0.0, 0.0);
    arm_pid_init_f32(&pid_vel, 0);
}

uvw_t VelocityDriver::Control()
{
    encoder->Update();
    omega_m = encoder->GetVelocity();
    theta_e = CalibrateElectricAngle(theta_e, omega_m);
    current_uvw = GetCurrent();

    current_ab = ClarkeTransform(current_uvw);
    dq_t raw_current_dq = ParkTransform(current_ab);

    arm_biquad_cascade_df2T_f32(&iir_id, &raw_current_dq.d, &current_dq.d, 1);
    arm_biquad_cascade_df2T_f32(&iir_iq, &raw_current_dq.q, &current_dq.q, 1);

    // ref_current_dq.d = 0.0;
    ref_current_dq.q = arm_pid_f32(&pid_vel, ref_vel - omega_m);

    voltage_dq.d = arm_pid_f32(&pid_id, ref_current_dq.d - current_dq.d);
    voltage_dq.q = arm_pid_f32(&pid_iq, ref_current_dq.q - current_dq.q);

    voltage_ab = InvParkTransform(voltage_dq);
    input_duty = CalculateSVPWM(voltage_ab);

    if (input_duty.u > max_duty)
        input_duty.u = max_duty;
    if (input_duty.v > max_duty)
        input_duty.v = max_duty;
    if (input_duty.w > max_duty)
        input_duty.w = max_duty;

    if (input_duty.u < 0.0)
        input_duty.u = 0.0;
    if (input_duty.v < 0.0)
        input_duty.v = 0.0;
    if (input_duty.w < 0.0)
        input_duty.w = 0.0;

    if (abs(current_uvw.u) > CURRENT_LIMIT || abs(current_uvw.v) > CURRENT_LIMIT || abs(current_uvw.w) > CURRENT_LIMIT)
    {
        input_duty.u = 0.0;
        input_duty.v = 0.0;
        input_duty.w = 0.0;
    }

    return input_duty;
}

void VelocityDriver::Reset()
{
    ResetBase();
    arm_pid_reset_f32(&(pid_vel));
    ref_vel = 0.0;
}

void TorqueDriver::Initialize()
{
    CurrentControlInit();
    theta_e = CalibrateElectricAngle(0.0, 0.0);
    // arm_pid_init_f32(&pid_torque, 0);
}

uvw_t TorqueDriver::Control()
{
    encoder->Update();
    omega_m = encoder->GetVelocity();
    theta_e = CalibrateElectricAngle(theta_e, omega_m);
    current_uvw = GetCurrent();

    current_ab = ClarkeTransform(current_uvw);
    dq_t raw_current_dq = ParkTransform(current_ab);

    arm_biquad_cascade_df2T_f32(&iir_id, &raw_current_dq.d, &current_dq.d, 1);
    arm_biquad_cascade_df2T_f32(&iir_iq, &raw_current_dq.q, &current_dq.q, 1);

    // ref_current_dq.d = 0.0;
    // ref_current_dq.q = arm_pid_f32(&pid_torque, ref_torque / Ktau - current_dq.q);
    ref_current_dq.q = ref_torque / Ktau;

    voltage_dq.d = arm_pid_f32(&pid_id, ref_current_dq.d - current_dq.d);
    voltage_dq.q = arm_pid_f32(&pid_iq, ref_current_dq.q - current_dq.q);

    voltage_ab = InvParkTransform(voltage_dq);
    input_duty = CalculateSVPWM(voltage_ab);

    if (input_duty.u > max_duty)
        input_duty.u = max_duty;
    if (input_duty.v > max_duty)
        input_duty.v = max_duty;
    if (input_duty.w > max_duty)
        input_duty.w = max_duty;

    if (input_duty.u < 0.0)
        input_duty.u = 0.0;
    if (input_duty.v < 0.0)
        input_duty.v = 0.0;
    if (input_duty.w < 0.0)
        input_duty.w = 0.0;

    if (abs(current_uvw.u) > CURRENT_LIMIT || abs(current_uvw.v) > CURRENT_LIMIT || abs(current_uvw.w) > CURRENT_LIMIT)
    {
        input_duty.u = 0.0;
        input_duty.v = 0.0;
        input_duty.w = 0.0;
    }

    return input_duty;
}

void TorqueDriver::Reset()
{
    ResetBase();
    // arm_pid_reset_f32(&pid_torque);
    ref_torque = 0.0;
}