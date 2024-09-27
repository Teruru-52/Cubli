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
#include "common/micros.h"
#include "sensors/encoder.h"
#include "sensors/hall_sensor.h"
#include "drivers/drv8323.h"
#include <algorithm>

#define Sign(val) (((val) > 0) - ((val) < 0))

using namespace protocol;
using namespace std;

class DriverController : public FOCMotor
{
private:
    BLDC_PWM *bldc_pwm;
    A1333 *encoder;
    DRV8323 *driver;
    HallSensor *hall;

    // open loop variables
    long open_loop_timestamp;

    void UpdateSensorAngle() override;
    float GetElectricAngle() override;
    dq_t GetDQCurrents() override;
    uint8_t GetSector() override;
    void SetPhaseVoltage(float Uq, float Ud, float angle_el) override;
    void UpdateFOC() override;
    void Move() override;
    void SetPwm(float Vu, float Vv, float Vw) override;

    void SearchZeroElectricAngle();
    void AngleOpenLoop(float target_angle);

public:
    DriverController(BLDC_PWM *bldc_pwm, A1333 *encoder, DRV8323 *drv, HallSensor *hall);
    void Initialize() override;
    void Enable() override;
    void Disable() override;
    void Update() override;
    void SetPwm() override;
    void Stop() override;
    void Free() override;
    void PrintLog() override;
};

#endif // DRIVER_CONTROLLER_H_