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

DriverController::DriverController(BLDC_PWM *bldc_pwm, A1333 *encoder, DRV8323 *drv, HallSensor *hall)
    : FOCMotor(),
      bldc_pwm(bldc_pwm),
      encoder(encoder),
      driver(drv),
      hall(hall)
{
    // torque control type is torque by default
    torque_controller = TorqueControlType::foc_current;
}

void DriverController::Initialize()
{
    driver->Initialize();
    HAL_Delay(100);
    adc_calibrated = driver->SetCurrentoffsets();

    encoder->Initialize();
    // _configure3PWM(bldc_pwm);
    SearchZeroElectricAngle();

    arm_pid_init_f32(&pid_id, 0);
    arm_pid_init_f32(&pid_iq, 0);

    arm_biquad_cascade_df2T_init_f32(&iir_id, 1, iir_coeff_d, iir_statebuff_d);
    arm_biquad_cascade_df2T_init_f32(&iir_iq, 1, iir_coeff_q, iir_statebuff_q);
    initialized = 1;
}

void DriverController::Enable()
{
    // arm_pid_reset_f32(&(pid_vel));
    // arm_pid_reset_f32(&pid_id);
    // arm_pid_reset_f32(&pid_iq);

    _pid_vel.Reset();
    _pid_id.Reset();
    _pid_iq.Reset();
    target = 0.0;
}

void DriverController::Disable()
{
}

void DriverController::SearchZeroElectricAngle()
{
    // search the absolute zero with small velocity
    float limit_vel = velocity_limit;
    float limit_volt = voltage_limit;
    velocity_limit = velocity_index_search;
    voltage_limit = voltage_sensor_align;
    shaft_angle = 0;
    uint8_t pre_sector = GetSector();
    // Need to add searching movement
    while (1)
    {
        uint8_t sector = GetSector();
        // AngleOpenLoop(3.0f * M_PI);
        if ((sector == 1 && pre_sector == 6) || (sector == 6 && pre_sector == 1))
        {
            zero_electric_angle = pole_pairs * encoder->GetMechanicalAngle();
            break;
        }
        pre_sector = sector;
    }
    printf("zero_electric_angle = %.3f\n", zero_electric_angle);
    // disable motor
    SetPhaseVoltage(0, 0, 0);
    SetPwm();
    // reinit the limits
    velocity_limit = limit_vel;
    voltage_limit = limit_volt;

    // HAL_Delay(500);
}

void DriverController::UpdateSensorAngle()
{
    // encoder->Update();
    shaft_velocity = encoder->GetVelocity();
    electric_angle = GetElectricAngle();
}

float DriverController::GetElectricAngle()
{
    // [0, 2pi)
    if (_isset(zero_electric_angle))
        return NormalizeAngle(pole_pairs * encoder->GetMechanicalAngle() - zero_electric_angle);
    else
        return NormalizeAngle(pole_pairs * encoder->GetMechanicalAngle());
}

dq_t DriverController::GetDQCurrents()
{
    current_uvw = driver->GetPhaseCurrents();
    current_ab = ClarkeTransform(current_uvw);
    dq_t current_dq_ = ParkTransform(current_ab, electric_angle);

    return current_dq_;
}

uint8_t DriverController::GetSector()
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

void DriverController::Update()
{
    UpdateFOC();
    Move();
}

void DriverController::UpdateFOC()
{
    UpdateSensorAngle();

    switch (torque_controller)
    {
    case TorqueControlType::voltage:
        // no need to do anything really
        break;
    case TorqueControlType::foc_current:
        current_dq = GetDQCurrents();

        // arm_biquad_cascade_df2T_f32(&iir_id, &raw_current_dq.d, &current_dq.d, 1);
        // arm_biquad_cascade_df2T_f32(&iir_iq, &raw_current_dq.q, &current_dq.q, 1);
        current_dq.d = lpf_current_d.Update(current_dq.d);
        current_dq.q = lpf_current_q.Update(current_dq.q);

        // voltage_dq.d = arm_pid_f32(&pid_id, -current_dq.d);
        // voltage_dq.q = arm_pid_f32(&pid_iq, ref_current_dq.q - current_dq.q);
        voltage_dq.d = _pid_id.Update(-current_dq.d);
        voltage_dq.q = _pid_iq.Update(ref_current_dq.q - current_dq.q);
        break;
    default:
        // no torque control selected
        printf("MOT: no torque control selected!\n");
        Write_GPIO(LED_RED, GPIO_PIN_SET);
        break;
    }

    // set the phase voltage - FOC heart function :)
    SetPhaseVoltage(voltage_dq.q, voltage_dq.d, electric_angle);
}

void DriverController::Move()
{
    switch (controller)
    {
    case MotionControlType::velocity:
        // float input_trape = arm_pid_f32(&pid_vel, ref_vel - shaft_velocity);
        voltage_dq.q = _pid_vel.Update(target - shaft_velocity);
        voltage_dq.d = 0.0f;

        // calculate the torque command
        ref_current_dq.q = _pid_vel.Update(target - shaft_velocity); // if current torque control
        // ref_current_dq.q = arm_pid_f32(&pid_vel, target - shaft_velocity);

        // if torque controlled through voltage control
        if (torque_controller == TorqueControlType::voltage)
        {
            // // use voltage if phase-resistance not provided
            // if (!_isset(phase_resistance))
            //     voltage_dq.q = ref_current_dq.q;
            // else
            //     voltage_dq.q = _constrain(ref_current_dq.q * phase_resistance + voltage_bemf, -voltage_limit, voltage_limit);
            // // set d-component (lag compensation if known inductance)
            // if (!_isset(phase_inductance))
            //     voltage_dq.d = 0;
            // else
            //     voltage_dq.d = _constrain(-current_sp * shaft_velocity * pole_pairs * phase_inductance, -voltage_limit, voltage_limit);
        }
        break;
    case MotionControlType::torque:
        if (torque_controller == TorqueControlType::voltage)
        { // if voltage torque control
          // if (!_isset(phase_resistance))
          //     voltage_dq.q = target;
          // else
          //     voltage_dq.q = target * phase_resistance + voltage_bemf;
          // voltage_dq.q = _constrain(voltage_dq.q, -voltage_limit, voltage_limit);
          // // set d-component (lag compensation if known inductance)
          // if (!_isset(phase_inductance))
          //     voltage_dq.d = 0;
          // else
          //     voltage_dq.d = _constrain(-target * shaft_velocity * pole_pairs * phase_inductance, -voltage_limit, voltage_limit);
        }
        else
            current_dq.q = target; // if current torque control
        break;
    }
}

void DriverController::SetPhaseVoltage(float Uq, float Ud, float angle_el)
{
    float center;
    uint8_t sector = GetSector();
    sector--;

    switch (foc_modulation)
    {
    case FOCModulationType::Trapezoid_120:
        center = Uq;
        // center = voltage_limit / 2.0f;
        if (trap_120_map[sector][0] == _HIGH_IMPEDANCE)
        {
            voltage_uvw.u = center;
            voltage_uvw.v = trap_120_map[sector][1] * Uq + center;
            voltage_uvw.w = trap_120_map[sector][2] * Uq + center;
            driver->SetPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // disable phase if possible
        }
        else if (trap_120_map[sector][1] == _HIGH_IMPEDANCE)
        {
            voltage_uvw.u = trap_120_map[sector][0] * Uq + center;
            voltage_uvw.v = center;
            voltage_uvw.w = trap_120_map[sector][2] * Uq + center;
            driver->SetPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON); // disable phase if possible
        }
        else
        {
            voltage_uvw.u = trap_120_map[sector][0] * Uq + center;
            voltage_uvw.v = trap_120_map[sector][1] * Uq + center;
            voltage_uvw.w = center;
            driver->SetPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF); // disable phase if possible
        }
        break;

    case FOCModulationType::SpaceVectorPWM:
        voltage_dq.q = Uq;
        voltage_dq.d = Ud;
        voltage_ab = InvParkTransform(voltage_dq, electric_angle);
        voltage_uvw = InvClarkeTransform(voltage_ab);

        center = voltage_limit / 2.0f;
        // discussed here: https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
        // a bit more info here: https://microchipdeveloper.com/mct5001:which-zsm-is-best
        // Midpoint Clamp
        float Umin = min(voltage_uvw.u, min(voltage_uvw.v, voltage_uvw.w));
        float Umax = max(voltage_uvw.u, max(voltage_uvw.v, voltage_uvw.w));
        center -= (Umax + Umin) / 2.0f;

        // Umin = min(voltage_uvw.u, min(voltage_uvw.v, voltage_uvw.w));
        // voltage_uvw.u -= Umin;
        // voltage_uvw.v -= Umin;
        // voltage_uvw.w -= Umin;
        voltage_uvw.u += center;
        voltage_uvw.v += center;
        voltage_uvw.w += center;
        driver->SetPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON);
        break;
    }
}

void DriverController::AngleOpenLoop(float target_angle)
{
    // get current timestamp
    unsigned long now_us = micros();
    // calculate the sample time from last call
    float Ts = (now_us - open_loop_timestamp) * 1e-6f;
    // quick fix for strange cases (micros overflow + timestamp not defined)
    if (Ts <= 0 || Ts > 0.5f)
        Ts = 1e-3f;

    // calculate the necessary angle to move from current position towards target angle
    // with maximal velocity (velocity_limit)
    // TODO sensor precision: this calculation is not numerically precise. The angle can grow to the point
    //                        where small position changes are no longer captured by the precision of floats
    //                        when the total position is large.
    if (abs(target_angle - shaft_angle) > abs(velocity_limit * Ts))
    {
        shaft_angle += _sign(target_angle - shaft_angle) * abs(velocity_limit) * Ts;
        shaft_velocity = velocity_limit;
    }
    else
    {
        shaft_angle = target_angle;
        shaft_velocity = 0;
    }

    // use voltage limit or current limit
    float Uq = voltage_limit;
    // if (_isset(phase_resistance))
    // {
    //     Uq = _constrain(current_limit * phase_resistance + fabs(voltage_bemf), -voltage_limit, voltage_limit);
    //     // recalculate the current
    //     current_dq.q = (Uq - fabs(voltage_bemf)) / phase_resistance;
    // }
    SetPhaseVoltage(Uq, 0, NormalizeAngle(pole_pairs * shaft_angle));
    SetPwm();
}

void DriverController::SetPwm()
{
    SetPwm(voltage_uvw.u, voltage_uvw.v, voltage_uvw.w);
}

void DriverController::SetPwm(float Vu, float Vv, float Vw)
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

void DriverController::Stop()
{
    driver->Align();
    SetPwm(0.0, 0.0, 0.0);
}

void DriverController::Free()
{
    driver->Free();
    SetPwm(0.0, 0.0, 0.0);
}

void DriverController::PrintLog()
{
    // printf("electric_angle = %.3f, shaft_velocity = %.3f\n", electric_angle, shaft_velocity);

    // printf("cur_u = %.3f, cur_v = %.3f, cur_w = %.3f\n", current_uvw.u, current_uvw.v, current_uvw.w);
    // printf("cur_a = %.3f, cur_b = %.3f\n", current_ab.a, current_ab.b);
    // printf("cur_d = %.3f, cur_q = %.3f\n", current_dq.d, current_dq.q);
    // printf("vol_d = %.3f, vol_q = %.3f\n", voltage_dq.d, voltage_dq.q);
    printf("vol_u = %.3f, vol_v = %.3f, vol_w = %.3f\n", voltage_uvw.u, voltage_uvw.v, voltage_uvw.w);
    printf("input_u = %.3f, input_v = %.3f, input_w = %.3f\n", input_duty.u, input_duty.v, input_duty.w);

    // printf("input_trapezoidal = %.3f\n", input_trape);
}