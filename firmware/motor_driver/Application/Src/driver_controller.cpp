/*
 * driver_controller.cpp
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#include "driver_controller.h"

#define CURRENT_LIMIT 20.0

using namespace protocol;

void DriverControllerBase::Initialize()
{
    drv->Initialize();
    SetCurrentoffset();
    theta_e = UpdateElectricAngle(0.0); // Need to be changed
    hall->ReadHallValue();
    pre_hall_state = hall->GetHallValue();

    arm_pid_init_f32(&pid_id, 0);
    arm_pid_init_f32(&pid_iq, 0);

    arm_biquad_cascade_df2T_init_f32(&iir_id, 1, iir_coeff_d, iir_statebuff_d);
    arm_biquad_cascade_df2T_init_f32(&iir_iq, 1, iir_coeff_q, iir_statebuff_q);
    initialized = true;
}

void DriverControllerBase::UpdateSensorAngle()
{
    // hall->ReadHallValue();
    hall_state = hall->GetHallValue();
    // encoder->Update();
    float theta_m_diff = encoder->GetAngleDiff();
    omega_m = encoder->GetVelocity();
    theta_e = UpdateElectricAngle(theta_m_diff);
}

void DriverControllerBase::CorrectElectricAngle(uint8_t _hall_state)
{
    // update theta_e by hall sensor
    hall_state = _hall_state;
    if (hall_state != pre_hall_state)
    {
        if (hall_state == 0b00000100 && pre_hall_state == 0b00000110)
            // theta_e_base = 0.0;
            theta_e_base = M_PI_3;
        else if (hall_state == 0b00000100 && pre_hall_state == 0b00000101)
            // theta_e_base = 0.0;
            theta_e_base = M_PI_3;
        else if (hall_state == 0b00000110 && pre_hall_state == 0b00000100)
            // theta_e_base = M_PI_3;
            theta_e_base = 2.0f * M_PI_3;
        else if (hall_state == 0b00000110 && pre_hall_state == 0b00000010)
            // theta_e_base = M_PI_3;
            theta_e_base = 2.0f * M_PI_3;
        else if (hall_state == 0b00000010 && pre_hall_state == 0b00000110)
            // theta_e_base = 2.0f * M_PI_3;
            theta_e_base = M_PI;
        else if (hall_state == 0b00000010 && pre_hall_state == 0b00000011)
            // theta_e_base = 2.0f * M_PI_3;
            theta_e_base = M_PI;
        else if (hall_state == 0b00000011 && pre_hall_state == 0b00000010)
            // theta_e_base = M_PI;
            theta_e_base = 4.0f * M_PI_3;
        else if (hall_state == 0b00000011 && pre_hall_state == 0b00000001)
            // theta_e_base = M_PI;
            theta_e_base = 4.0f * M_PI_3;
        else if (hall_state == 0b00000001 && pre_hall_state == 0b00000011)
            // theta_e_base = 4.0f * M_PI_3;
            theta_e_base = 5.0f * M_PI_3;
        else if (hall_state == 0b00000001 && pre_hall_state == 0b00000101)
            // theta_e_base = 4.0f * M_PI_3;
            theta_e_base = 5.0f * M_PI_3;
        else if (hall_state == 0b00000101 && pre_hall_state == 0b00000001)
            // theta_e_base = 5.0f * M_PI_3;
            theta_e_base = 0;
        else if (hall_state == 0b00000101 && pre_hall_state == 0b00000100)
            // theta_e_base = 5.0f * M_PI_3;
            theta_e_base = 0;
    }

    encoder->SetAngleBase();
    pre_hall_state = hall_state;
}

float DriverControllerBase::UpdateElectricAngle(float theta_m_diff)
{
    // update theta_e by encoder
    theta_e = theta_e_base + theta_m_diff * Pn;

    // limit theta_e to [-2pi, 2pi)
    if (theta_e >= M_2PI)
        theta_e -= M_2PI;
    // else if (theta_e <= -M_2PI)
    else if (theta_e <= 0)
        theta_e += M_2PI;

    return theta_e;
}

uvw_t DriverControllerBase::CalculateCurrent(uint32_t *adc_data)
{
    float Vsox[phase_num] = {0.0, 0.0, 0.0}; // [V]

    for (int i = 0; i < phase_num; i++)
        Vsox[i] = static_cast<float>(adc_data[i]) * Vref / static_cast<float>(adc_resolution);
    current_uvw.u = (Vref * 0.5f - Vsox[2]) / (Gcsa * Rsense) - current_offset.u;
    current_uvw.v = (Vref * 0.5f - Vsox[1]) / (Gcsa * Rsense) - current_offset.v;
    current_uvw.w = (Vref * 0.5f - Vsox[0]) / (Gcsa * Rsense) - current_offset.w;

    // current_uvw.u = (Vref * 0.5f - Vsox[2]) / (Gcsa * Rsense);
    // current_uvw.v = (Vref * 0.5f - Vsox[1]) / (Gcsa * Rsense);
    // current_uvw.w = (Vref * 0.5f - Vsox[0]) / (Gcsa * Rsense);

    return current_uvw;
}

uvw_t DriverControllerBase::GetCurrent()
{
    uint32_t adc_data[phase_num] = {0, 0, 0};
    ADC_Get_Value(adc_data); // get digital current Data
    return CalculateCurrent(adc_data);
}

void DriverControllerBase::SetCurrentoffset()
{
    uint32_t adc_data[phase_num] = {0, 0, 0};
    float Vsox[phase_num] = {0.0, 0.0, 0.0}; // [V]

    drv->StartCalibration();
    for (int j = 0; j < max_cali_count; j++)
    {
        ADC_Get_Value(adc_data); // get digital current data
        for (int i = 0; i < phase_num; i++)
            Vsox[i] = static_cast<float>(adc_data[i]) * Vref / static_cast<float>(adc_resolution);

        current_offset.u += (Vref * 0.5f - Vsox[2]) / (Gcsa_calib * Rsense);
        current_offset.v += (Vref * 0.5f - Vsox[1]) / (Gcsa_calib * Rsense);
        current_offset.w += (Vref * 0.5f - Vsox[0]) / (Gcsa_calib * Rsense);
        // printf("current_offset.u = %.3f, current_offset.v = %.3f, current_offset.w = %.3f\n", current_offset.u, current_offset.v, current_offset.w);
        // HAL_Delay(1);
    }

    current_offset.u /= static_cast<float>(max_cali_count);
    current_offset.v /= static_cast<float>(max_cali_count);
    current_offset.w /= static_cast<float>(max_cali_count);
    printf("current_offset.u = %.3f, current_offset.v = %.3f, current_offset.w = %.3f\n", current_offset.u, current_offset.v, current_offset.w);
    drv->FinishCalibration();
    adc_calibrated = true;
}

ab_t DriverControllerBase::ClarkeTransform(const uvw_t &current_uvw)
{
    ab_t current_ab_;
    current_ab_.a = (current_uvw.u + current_uvw.v * cos23 + current_uvw.w * cos43) * sq23;
    current_ab_.b = (current_uvw.v * sin23 + current_uvw.w * sin43) * sq23;

    return current_ab_;
}

dq_t DriverControllerBase::ParkTransform(const ab_t &current_ab)
{
    dq_t current_dq_;
    current_dq_.d = current_ab.a * cos(theta_e) + current_ab.b * sin(theta_e);
    current_dq_.q = -current_ab.a * sin(theta_e) + current_ab.b * cos(theta_e);
    // arm_park_f32(current_ab.a, current_ab.b, &(current_dq_.d), &(current_dq_.q), arm_sin_f32(theta_e), arm_cos_f32(theta_e));

    return current_dq_;
}

// uvw_t DriverControllerBase::InvClarkeTransform(const ab_t &voltage_ab)
// {
//     uvw_t voltage_uvw_;
//     voltage_uvw_.u = voltage_ab.a * sq23;
//     voltage_uvw_.v = (voltage_ab.a * cos23 + voltage_ab.b * sin23) * sq23;
//     voltage_uvw_.w = (voltage_ab.a * cos43 + voltage_ab.b * sin43) * sq23;

//     return voltage_uvw_;
// }

ab_t DriverControllerBase::InvParkTransform(const dq_t &voltage_dq)
{
    ab_t voltage_ab_;
    voltage_ab_.a = voltage_dq.d * cos(theta_e) - voltage_dq.q * sin(theta_e);
    voltage_ab_.b = voltage_dq.d * sin(theta_e) + voltage_dq.q * cos(theta_e);
    // arm_inv_park_f32(voltage_dq.d, voltage_dq.q, &(voltage_ab_.a), &(voltage_ab_.b), arm_sin_f32(theta_e), arm_cos_f32(theta_e));

    return voltage_ab_;
}

uvw_t DriverControllerBase::CalculateSVPWM(const ab_t &voltage_ab)
{
    uvw_t input_duty_;

    // Sector 0
    if ((voltage_ab.a >= 0) && (voltage_ab.b >= 0) && (abs(voltage_ab.a) >= abs(sq13 * voltage_ab.b)))
    {
        d1 = sq32 * (voltage_ab.a - sq13 * voltage_ab.b) / max_voltage;
        d2 = sq32 * (sq43 * voltage_ab.b) / max_voltage;
        d7 = (zero_vector_param - (d1 + d2)) * 0.5f;
        input_duty_.u = d1 + d2 + d7;
        input_duty_.v = d2 + d7;
        input_duty_.w = d7;
    }
    // Sector 1
    else if ((abs(voltage_ab.a) <= sq13 * voltage_ab.b))
    {
        d3 = sq32 * (-voltage_ab.a + sq13 * voltage_ab.b) / max_voltage;
        d2 = sq32 * (voltage_ab.a + sq13 * voltage_ab.b) / max_voltage;
        d7 = (zero_vector_param - (d2 + d3)) * 0.5f;
        input_duty_.u = d2 + d7;
        input_duty_.v = d2 + d3 + d7;
        input_duty_.w = d7;
    }
    // Sector 2
    else if ((voltage_ab.a <= 0) && (voltage_ab.b >= 0) && (abs(voltage_ab.a) >= abs(sq13 * voltage_ab.b)))
    {
        d3 = sq32 * (sq43 * voltage_ab.b) / max_voltage;
        d4 = sq32 * (-voltage_ab.a - sq13 * voltage_ab.b) / max_voltage;
        d7 = (zero_vector_param - (d3 + d4)) * 0.5f;
        input_duty_.u = d7;
        input_duty_.v = d3 + d4 + d7;
        input_duty_.w = d4 + d7;
    }
    // Sector 3
    else if ((voltage_ab.a <= 0) && (voltage_ab.b <= 0) && (abs(voltage_ab.a) >= abs(sq13 * voltage_ab.b)))
    {
        d5 = -sq32 * (sq43 * voltage_ab.b) / max_voltage;
        d4 = sq32 * (-voltage_ab.a + sq13 * voltage_ab.b) / max_voltage;
        d7 = (zero_vector_param - (d4 + d5)) * 0.5f;
        input_duty_.u = d7;
        input_duty_.v = d4 + d7;
        input_duty_.w = d4 + d5 + d7;
    }
    // Sector 4
    else if ((abs(voltage_ab.a) <= -sq13 * voltage_ab.b))
    {
        d5 = sq32 * (-voltage_ab.a - sq13 * voltage_ab.b) / max_voltage;
        d6 = sq32 * (voltage_ab.a - sq13 * voltage_ab.b) / max_voltage;
        d7 = (zero_vector_param - (d5 + d6)) * 0.5f;
        input_duty_.u = d6 + d7;
        input_duty_.v = d7;
        input_duty_.w = d5 + d6 + d7;
    }
    // Sector 5
    else if ((voltage_ab.a >= 0) && (voltage_ab.b <= 0) && (abs(voltage_ab.a) >= abs(sq13 * voltage_ab.b)))
    {
        d1 = sq32 * (voltage_ab.a + sq13 * voltage_ab.b) / max_voltage;
        d6 = -sq32 * (sq43 * voltage_ab.b) / max_voltage;
        d7 = (zero_vector_param - (d1 + d6)) * 0.5f;
        input_duty_.u = d1 + d6 + d7;
        input_duty_.v = d7;
        input_duty_.w = d6 + d7;
    }

    // input_duty_.u = max_duty - input_duty_.u;
    // input_duty_.v = max_duty - input_duty_.v;
    // input_duty_.w = max_duty - input_duty_.w;

    // input_duty_.u = -0.5f + input_duty_.u;
    // input_duty_.v = -0.5f + input_duty_.v;
    // input_duty_.w = -0.5f + input_duty_.w;

    return input_duty_;
}

uvw_t VelocityDriver::Control()
{
    UpdateSensorAngle();
    current_uvw = GetCurrent();

    // select control method
    // UpdateFOC();
    UpdateTrapezoidalControl();

    LimitInput();
    // LimitCurrent();

    return input_duty;
}

void VelocityDriver::UpdateFOC()
{
    current_ab = ClarkeTransform(current_uvw);
    dq_t raw_current_dq = ParkTransform(current_ab);

    // arm_biquad_cascade_df2T_f32(&iir_id, &raw_current_dq.d, &current_dq.d, 1);
    // arm_biquad_cascade_df2T_f32(&iir_iq, &raw_current_dq.q, &current_dq.q, 1);
    // current_dq = raw_current_dq;
    current_dq.d = coeff_lp * pre_current_dq.d + (1.0f - coeff_lp) * raw_current_dq.d;
    current_dq.q = coeff_lp * pre_current_dq.q + (1.0f - coeff_lp) * raw_current_dq.q;
    pre_current_dq = current_dq;

    ref_current_dq.d = 0.0;
    // ref_current_dq.q = arm_pid_f32(&pid_vel, ref_vel - omega_m);
    // ref_current_dq.q = _pid_vel.Update(ref_vel - omega_m);
    ref_current_dq.q = 0.5f;

    // voltage_dq.d = arm_pid_f32(&pid_id, ref_current_dq.d - current_dq.d);
    // voltage_dq.q = arm_pid_f32(&pid_iq, ref_current_dq.q - current_dq.q);
    voltage_dq.d = _pid_id.Update(ref_current_dq.d - current_dq.d);
    voltage_dq.q = _pid_iq.Update(ref_current_dq.q - current_dq.q);

    voltage_ab = InvParkTransform(voltage_dq);
    input_duty = CalculateSVPWM(voltage_ab);

    drv->SetINLA();
    drv->SetINLB();
    drv->SetINLC();
}

void VelocityDriver::UpdateTrapezoidalControl()
{
    // float input_trape = arm_pid_f32(&pid_vel, ref_vel - omega_m);
    input_trape = _pid_vel.Update(ref_vel - omega_m);
    if (ref_vel > 0)
        input_trape = 0.5f - input_trape;
    else
        input_trape = 0.5f + input_trape;

    if (hall_state == 0b00000110)
    {
        if (ref_vel > 0)
        {
            input_duty.u = 0.0;
            drv->ResetINLA();
            input_duty.v = input_trape;
            drv->SetINLB();
            input_duty.w = max_duty;
            drv->SetINLC();
        }
        else
        {
            input_duty.u = 0.0;
            drv->ResetINLA();
            input_duty.v = max_duty;
            drv->SetINLB();
            input_duty.w = input_trape;
            drv->SetINLC();
        }
    }
    else if (hall_state == 0b00000100)
    {
        if (ref_vel > 0)
        {
            input_duty.u = input_trape;
            drv->SetINLA();
            input_duty.v = 0.0;
            drv->ResetINLB();
            input_duty.w = max_duty;
            drv->SetINLC();
        }
        else
        {
            input_duty.u = max_duty;
            drv->SetINLA();
            input_duty.v = 0.0;
            drv->ResetINLB();
            input_duty.w = input_trape;
            drv->SetINLC();
        }
    }
    else if (hall_state == 0b00000101)
    {
        if (ref_vel > 0)
        {
            input_duty.u = input_trape;
            drv->SetINLA();
            input_duty.v = max_duty;
            drv->SetINLB();
            input_duty.w = 0.0;
            drv->ResetINLC();
        }
        else
        {
            input_duty.u = max_duty;
            drv->SetINLA();
            input_duty.v = input_trape;
            drv->SetINLB();
            input_duty.w = 0.0;
            drv->ResetINLC();
        }
    }
    else if (hall_state == 0b00000001)
    {
        if (ref_vel > 0)
        {
            input_duty.u = 0.0;
            drv->ResetINLA();
            input_duty.v = max_duty;
            drv->SetINLB();
            input_duty.w = input_trape;
            drv->SetINLC();
        }
        else
        {
            input_duty.u = 0.0;
            drv->ResetINLA();
            input_duty.v = input_trape;
            drv->SetINLB();
            input_duty.w = max_duty;
            drv->SetINLC();
        }
    }
    else if (hall_state == 0b00000011)
    {
        if (ref_vel > 0)
        {
            input_duty.u = max_duty;
            drv->SetINLA();
            input_duty.v = 0.0;
            drv->ResetINLB();
            input_duty.w = input_trape;
            drv->SetINLC();
        }
        else
        {
            input_duty.u = input_trape;
            drv->SetINLA();
            input_duty.v = 0.0;
            drv->ResetINLB();
            input_duty.w = max_duty;
            drv->SetINLC();
        }
    }
    else if (hall_state == 0b00000010)
    {
        if (ref_vel > 0)
        {
            input_duty.u = max_duty;
            drv->SetINLA();
            input_duty.v = input_trape;
            drv->SetINLB();
            input_duty.w = 0.0;
            drv->ResetINLC();
        }
        else
        {
            input_duty.u = input_trape;
            drv->SetINLA();
            input_duty.v = max_duty;
            drv->SetINLB();
            input_duty.w = 0.0;
            drv->ResetINLC();
        }
    }
    else
    {
        Write_GPIO(LED_RED, GPIO_PIN_RESET);
        drv->ResetINLA();
        drv->ResetINLB();
        drv->ResetINLC();
    }
}

void DriverControllerBase::LimitInput()
{
    // limit input duty
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
}

void DriverControllerBase::LimitCurrent()
{
    // limit current
    if (abs(current_uvw.u) > CURRENT_LIMIT || abs(current_uvw.v) > CURRENT_LIMIT || abs(current_uvw.w) > CURRENT_LIMIT)
    {
        drv->Stop();
        input_duty.u = 0.0;
        input_duty.v = 0.0;
        input_duty.w = 0.0;
    }
}

void DriverControllerBase::ResetBase()
{
    arm_pid_reset_f32(&pid_id);
    arm_pid_reset_f32(&pid_iq);

    _pid_id.Reset();
    _pid_iq.Reset();
}

void DriverControllerBase::LogPrint()
{
    printf("theta_e = %.3f, omega_m = %.3f\n", theta_e, omega_m);

    printf("cur_u = %.3f, cur_v = %.3f, cur_w = %.3f\n", current_uvw.u, current_uvw.v, current_uvw.w);
    printf("cur_a = %.3f, cur_b = %.3f\n", current_ab.a, current_ab.b);
    printf("cur_d = %.3f, cur_q = %.3f\n", current_dq.d, current_dq.q);
    printf("input_u = %.3f, input_v = %.3f, input_w = %.3f\n", input_duty.u, input_duty.v, input_duty.w);

    // printf("input_trapezoidal = %.3f\n", input_trape);
}

void VelocityDriver::Reset()
{
    ResetBase();
    // arm_pid_reset_f32(&(pid_vel));
    _pid_vel.Reset();
}

// void TorqueDriver::Initialize()
// {
// CurrentControlInit();
// arm_pid_init_f32(&pid_torque, 0);
// }

uvw_t TorqueDriver::Control()
{
    UpdateSensorAngle();
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

    LimitInput();
    LimitCurrent();

    return input_duty;
}

void TorqueDriver::Reset()
{
    ResetBase();
    // arm_pid_reset_f32(&pid_torque);
    ref_torque = 0.0;
}