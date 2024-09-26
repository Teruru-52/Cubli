/*
 * driver_controller.cpp
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#include "drivers/driver_controller.h"

// see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
// each is 60 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
int trap_120_map[6][3] = {
    {_HIGH_IMPEDANCE, 1, -1},
    {-1, 1, _HIGH_IMPEDANCE},
    {-1, _HIGH_IMPEDANCE, 1},
    {_HIGH_IMPEDANCE, -1, 1},
    {1, -1, _HIGH_IMPEDANCE},
    {1, _HIGH_IMPEDANCE, -1}};

void DriverControllerBase::Initialize()
{
    drv->Initialize();
    HAL_Delay(100);
    SetCurrentoffset();

    encoder->Initialize();
    // _configure3PWM(bldc_pwm);
    SearchZeroElectricAngle();

    arm_pid_init_f32(&pid_id, 0);
    arm_pid_init_f32(&pid_iq, 0);

    arm_biquad_cascade_df2T_init_f32(&iir_id, 1, iir_coeff_d, iir_statebuff_d);
    arm_biquad_cascade_df2T_init_f32(&iir_iq, 1, iir_coeff_q, iir_statebuff_q);
    initialized = true;
}

void DriverControllerBase::SearchZeroElectricAngle()
{
    uint8_t pre_sector = GetSector();
    // Need to add searching movement
    while (1)
    {
        uint8_t sector = GetSector();
        encoder->Update();
        if ((sector == 1 && pre_sector == 6) || (sector == 6 && pre_sector == 1))
        {
            // AngleOpenLoop(3.0f * M_PI);
            zero_theta_e = Pn * encoder->GetMechanicalAngle();
            break;
        }
        HAL_Delay(1);
        pre_sector = sector;
    }
    printf("zero_theta_e = %.3f\n", zero_theta_e);
}

void DriverControllerBase::UpdateSensorAngle()
{
    // encoder->Update();
    omega_m = encoder->GetVelocity();
    theta_e = GetElectricAngle();
}

float DriverControllerBase::GetElectricAngle()
{
    // [-2pi, 2pi)
    float electric_angle = fmod(Pn * encoder->GetMechanicalAngle() - zero_theta_e, M_2PI);
    // [0, 2pi)
    return (electric_angle >= 0) ? electric_angle : electric_angle + M_2PI;
}

void DriverControllerBase::CalculateCurrent()
{
    float Vsox[phase_num] = {0.0, 0.0, 0.0}; // [V]
    uint32_t adc_data[phase_num] = {0, 0, 0};
    ADC_Get_Value(adc_data); // get digital current Data

    for (int i = 0; i < phase_num; i++)
        Vsox[i] = static_cast<float>(adc_data[i]) * Vref / static_cast<float>(adc_resolution);
    current_uvw.u = (Vref * 0.5f - Vsox[0]) / (Gcsa * Rsense) - current_offset.u;
    current_uvw.v = (Vref * 0.5f - Vsox[1]) / (Gcsa * Rsense) - current_offset.v;
    current_uvw.w = (Vref * 0.5f - Vsox[2]) / (Gcsa * Rsense) - current_offset.w;

    // current_uvw.u = (Vref * 0.5f - Vsox[2]) / (Gcsa * Rsense);
    // current_uvw.v = (Vref * 0.5f - Vsox[1]) / (Gcsa * Rsense);
    // current_uvw.w = (Vref * 0.5f - Vsox[0]) / (Gcsa * Rsense);
    // printf("%.3f, %.3f, %.3f\n", Vsox[0], Vsox[1], Vsox[2]);
}

uint8_t DriverControllerBase::GetSector()
{
    uint8_t hall_state = hall->GetHallValue();
    uint8_t sector = 0;
    if (hall_state == 0b00000110)
        sector = 1;
    else if (hall_state == 0b00000100)
        sector = 2;
    else if (hall_state == 0b00000101)
        sector = 3;
    else if (hall_state == 0b00000001)
        sector = 4;
    else if (hall_state == 0b00000011)
        sector = 5;
    else if (hall_state == 0b00000010)
        sector = 6;
    return sector;
}

void DriverControllerBase::SetCurrentoffset()
{
    uvw_t current_sum;

    drv->StartCalibration();
    for (int j = 0; j < max_cali_count; j++)
    {
        CalculateCurrent();
        current_sum.u += current_uvw.u;
        current_sum.v += current_uvw.v;
        current_sum.w += current_uvw.w;
    }

    current_offset.u = current_sum.u / static_cast<float>(max_cali_count);
    current_offset.v = current_sum.v / static_cast<float>(max_cali_count);
    current_offset.w = current_sum.w / static_cast<float>(max_cali_count);
    printf("current_offset.u = %.3f, current_offset.v = %.3f, current_offset.w = %.3f\n", current_offset.u, current_offset.v, current_offset.w);
    drv->FinishCalibration();
    adc_calibrated = true;
}

void DriverControllerBase::SetPwm()
{
    SetPwm(voltage_uvw.u, voltage_uvw.v, voltage_uvw.w);
}

void DriverControllerBase::SetPwm(float Vu, float Vv, float Vw)
{
    // limit the voltage in driver
    Vu = _constrain(Vu, 0.0f, voltage_limit);
    Vv = _constrain(Vv, 0.0f, voltage_limit);
    Vw = _constrain(Vw, 0.0f, voltage_limit);
    // calculate duty cycle
    // limited in [0, 1]
    input_duty.u = _constrain(Vu / voltage_power_supply, 0.0f, 1.0f);
    input_duty.v = _constrain(Vv / voltage_power_supply, 0.0f, 1.0f);
    input_duty.w = _constrain(Vw / voltage_power_supply, 0.0f, 1.0f);

    _writeDutyCycle3PWM(bldc_pwm, input_duty.u, input_duty.v, input_duty.w);
}

void DriverControllerBase::Stop()
{
    drv->Align();
    SetPwm(0.0, 0.0, 0.0);
}

void DriverControllerBase::Free()
{
    drv->Free();
    SetPwm(0.0, 0.0, 0.0);
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
    current_dq_.d = current_ab.a * arm_cos_f32(theta_e) + current_ab.b * arm_sin_f32(theta_e);
    current_dq_.q = -current_ab.a * arm_sin_f32(theta_e) + current_ab.b * arm_cos_f32(theta_e);
    // arm_park_f32(current_ab.a, current_ab.b, &(current_dq_.d), &(current_dq_.q), arm_sin_f32(theta_e), arm_cos_f32(theta_e));

    return current_dq_;
}

uvw_t DriverControllerBase::InvClarkeTransform(const ab_t &voltage_ab)
{
    uvw_t voltage_uvw_;
    // voltage_uvw_.u = voltage_ab.a * sq23;
    // voltage_uvw_.v = (voltage_ab.a * cos23 + voltage_ab.b * sin23) * sq23;
    // voltage_uvw_.w = (voltage_ab.a * cos43 + voltage_ab.b * sin43) * sq23;
    voltage_uvw_.u = voltage_ab.a;
    voltage_uvw_.v = -0.5f * voltage_ab.a + sq34 * voltage_ab.b;
    voltage_uvw_.w = -0.5f * voltage_ab.a - sq34 * voltage_ab.b;

    return voltage_uvw_;
}

ab_t DriverControllerBase::InvParkTransform(const dq_t &voltage_dq)
{
    ab_t voltage_ab_;
    voltage_ab_.a = voltage_dq.d * arm_cos_f32(theta_e) - voltage_dq.q * arm_sin_f32(theta_e);
    voltage_ab_.b = voltage_dq.d * arm_sin_f32(theta_e) + voltage_dq.q * arm_cos_f32(theta_e);
    // arm_inv_park_f32(voltage_dq.d, voltage_dq.q, &(voltage_ab_.a), &(voltage_ab_.b), arm_sin_f32(theta_e), arm_cos_f32(theta_e));

    return voltage_ab_;
}

void DriverControllerBase::SetPhaseVoltage(float Uq, float Ud, float angle_el)
{
    float center;
    uint8_t sector = GetSector();
    sector--;

    center = Uq;
    // center = voltage_limit / 2.0f;
    if (trap_120_map[sector][0] == _HIGH_IMPEDANCE)
    {
        voltage_uvw.u = center;
        voltage_uvw.v = trap_120_map[sector][1] * Uq + center;
        voltage_uvw.w = trap_120_map[sector][2] * Uq + center;
        drv->SetPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // disable phase if possible
    }
    else if (trap_120_map[sector][1] == _HIGH_IMPEDANCE)
    {
        voltage_uvw.u = trap_120_map[sector][0] * Uq + center;
        voltage_uvw.v = center;
        voltage_uvw.w = trap_120_map[sector][2] * Uq + center;
        drv->SetPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON); // disable phase if possible
    }
    else
    {
        voltage_uvw.u = trap_120_map[sector][0] * Uq + center;
        voltage_uvw.v = trap_120_map[sector][1] * Uq + center;
        voltage_uvw.w = center;
        drv->SetPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF); // disable phase if possible
    }

    // voltage_dq.q = Uq;
    // voltage_dq.d = Ud;
    // voltage_ab = InvParkTransform(voltage_dq);
    // voltage_uvw = InvClarkeTransform(voltage_ab);

    // center = voltage_limit / 2.0f;
    // // discussed here: https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
    // // a bit more info here: https://microchipdeveloper.com/mct5001:which-zsm-is-best
    // // Midpoint Clamp
    // float Umin = min(voltage_uvw.u, min(voltage_uvw.v, voltage_uvw.w));
    // float Umax = max(voltage_uvw.u, max(voltage_uvw.v, voltage_uvw.w));
    // center -= (Umax + Umin) / 2.0f;

    // // Umin = min(voltage_uvw.u, min(voltage_uvw.v, voltage_uvw.w));
    // // voltage_uvw.u -= Umin;
    // // voltage_uvw.v -= Umin;
    // // voltage_uvw.w -= Umin;
    // voltage_uvw.u += center;
    // voltage_uvw.v += center;
    // voltage_uvw.w += center;
    // drv->SetPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON);
}

void DriverControllerBase::AngleOpenLoop(float target_angle)
{
    float Ts = 0.001f;
    float shaft_angle = encoder->GetMechanicalAngle();

    // calculate the necessary angle to move from current position towards target angle
    // with maximal velocity (velocity_limit)
    // TODO sensor precision: this calculation is not numerically precise. The angle can grow to the point
    //                        where small position changes are no longer captured by the precision of floats
    //                        when the total position is large.
    // if (abs(target_angle - shaft_angle) > abs(velocity_limit * Ts))
    // {
    //     shaft_angle += _sign(target_angle - shaft_angle) * abs(velocity_limit) * Ts;
    //     shaft_velocity = velocity_limit;
    // }
    // else
    // {
    //     shaft_angle = target_angle;
    //     shaft_velocity = 0;
    // }

    // use voltage limit or current limit
    float Uq = voltage_limit;
    // if (_isset(phase_resistance))
    // {
    //     Uq = _constrain(current_limit * phase_resistance + fabs(voltage_bemf), -voltage_limit, voltage_limit);
    //     // recalculate the current
    //     current.q = (Uq - fabs(voltage_bemf)) / phase_resistance;
    // }
    SetPhaseVoltage(Uq, 0, 0);
}

void VelocityDriver::Control()
{
    UpdateSensorAngle();
    CalculateCurrent();

    // select control method
    // UpdateFOC();
    // UpdateTrapezoid_120();
    SetPhaseVoltage(1.0, 0.0, 0.0);

    // LimitCurrent();
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
    // CalculateSVPWM(voltage_ab);

    drv->SetPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON);
}

void VelocityDriver::UpdateTrapezoid_120()
{
    // float input_trape = arm_pid_f32(&pid_vel, ref_vel - omega_m);
    voltage_dq.q = _pid_vel.Update(ref_vel - omega_m);
    voltage_dq.d = 0.0f;

    SetPhaseVoltage(voltage_dq.q, voltage_dq.d, theta_e);
}

void DriverControllerBase::LimitCurrent()
{
    // limit current
    if (abs(current_uvw.u) > CURRENT_LIMIT || abs(current_uvw.v) > CURRENT_LIMIT || abs(current_uvw.w) > CURRENT_LIMIT)
    {
        drv->Free();
        voltage_uvw.u = 0.0;
        voltage_uvw.v = 0.0;
        voltage_uvw.w = 0.0;
    }
}

void DriverControllerBase::ResetBase()
{
    arm_pid_reset_f32(&pid_id);
    arm_pid_reset_f32(&pid_iq);

    _pid_id.Reset();
    _pid_iq.Reset();
}

void DriverControllerBase::PrintLog()
{
    // printf("theta_e = %.3f, omega_m = %.3f\n", theta_e, omega_m);

    // printf("cur_u = %.3f, cur_v = %.3f, cur_w = %.3f\n", current_uvw.u, current_uvw.v, current_uvw.w);
    // printf("cur_a = %.3f, cur_b = %.3f\n", current_ab.a, current_ab.b);
    // printf("cur_d = %.3f, cur_q = %.3f\n", current_dq.d, current_dq.q);
    // printf("vol_d = %.3f, vol_q = %.3f\n", voltage_dq.d, voltage_dq.q);
    printf("vol_u = %.3f, vol_v = %.3f, vol_w = %.3f\n", voltage_uvw.u, voltage_uvw.v, voltage_uvw.w);
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

void TorqueDriver::Control()
{
    UpdateSensorAngle();
    CalculateCurrent();

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
    // CalculateSVPWM(voltage_ab);

    LimitCurrent();
}

void TorqueDriver::Reset()
{
    ResetBase();
    // arm_pid_reset_f32(&pid_torque);
    ref_torque = 0.0;
}