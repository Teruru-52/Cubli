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

    // index search velocity
    velocity_index_search = DEF_INDEX_SEARCH_TARGET_VELOCITY;
    // sensor and motor align voltage
    voltage_sensor_align = DEF_VOLTAGE_SENSOR_ALIGN;

    // default modulation is SpaceVectorPWM
    foc_modulation = FOCModulationType::SpaceVectorPWM;

    // default target value
    target = 0;
    voltage_dq.d = 0;
    voltage_dq.q = 0;
    // current target values
    ref_current_dq.q = 0;
    current_dq.q = 0;
    current_dq.d = 0;

    // voltage bemf
    voltage_bemf = 0;

    // Initialize phase voltages used for inverse Park and Clarke transform
    voltage_ab.a = 0;
    voltage_ab.b = 0;
}