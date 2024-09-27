#include "common/base_classes/foc_motor.h"

/**
 * Default constructor - setting all variabels to default values
 */
FOCMotor::FOCMotor()
{
    // maximum angular velocity to be used for positioning
    velocity_limit = DEF_VEL_LIM;
    // maximum voltage to be set to the motor
    voltage_limit = DEF_POWER_SUPPLY;
    // not set on the begining
    current_limit = DEF_CURRENT_LIM;

    // sensor and motor align voltage
    voltage_sensor_align = DEF_VOLTAGE_SENSOR_ALIGN;

    // default modulation is SpaceVectorPWM
    foc_modulation = FOCModulationType::SpaceVectorPWM;

    // default target value
    target = 50.0f;
    voltage_dq.d = 0.0f;
    voltage_dq.q = 0.0f;
    // current target values
    ref_current_dq.q = 0.0f;
    current_dq.q = 0.0f;
    current_dq.d = 0.0f;

    // voltage bemf
    voltage_bemf = 0.0f;

    // Initialize phase voltages used for inverse Park and Clarke transform
    voltage_ab.a = 0.0f;
    voltage_ab.b = 0.0f;
}