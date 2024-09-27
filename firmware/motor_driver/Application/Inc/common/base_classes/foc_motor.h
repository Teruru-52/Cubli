/*
 * foc_driver.h
 *
 *  Created on: Sep 22th, 2024
 *      Author: Reiji Terunuma
 */

#ifndef FOC_MOTOR_H_
#define FOC_MOTOR_H_

#include "main.h"

/**
 *  Motiron control type
 */
enum MotionControlType : uint8_t
{
    torque = 0x00,   //!< Torque control
    velocity = 0x01, //!< Velocity motion control
    angle = 0x02,    //!< Position/angle motion control
    velocity_openloop = 0x03,
    angle_openloop = 0x04
};

/**
 *  Torque control type
 */
enum TorqueControlType : uint8_t
{
    voltage = 0x00, //!< Torque control using voltage
    // dc_current = 0x01,  //!< Torque control using DC current (one current magnitude)
    foc_current = 0x02, //!< torque control using dq currents
};

/**
 *  FOC modulation type
 */
enum FOCModulationType : uint8_t
{
    SpaceVectorPWM = 0x00, //!< Space vector modulation method
    Trapezoid_120 = 0x01,
};

enum FOCMotorStatus : uint8_t
{
    motor_uninitialized = 0x00, //!< Motor is not yet initialized
    motor_initializing = 0x01,  //!< Motor intiialization is in progress
    motor_uncalibrated = 0x02,  //!< Motor is initialized, but not calibrated (open loop possible)
    motor_calibrating = 0x03,   //!< Motor calibration in progress
    motor_ready = 0x04,         //!< Motor is initialized and calibrated (closed loop possible)
    motor_error = 0x08,         //!< Motor is in error state (recoverable, e.g. overcurrent protection active)
    motor_calib_failed = 0x0E,  //!< Motor calibration failed (possibly recoverable)
    motor_init_failed = 0x0F,   //!< Motor initialization failed (not recoverable)
};

#endif // FOC_MOTOR_H_