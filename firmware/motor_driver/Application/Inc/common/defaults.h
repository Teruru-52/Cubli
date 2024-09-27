/*
 * defaults.h
 *
 *  Created on: Sep 27th, 2024
 *      Author: Reiji Terunuma
 */

#ifndef DEFAULTS_H_
#define DEFAULTS_H_

#include "main.h"

// default configuration values
// change this file to optimal values for your application

#define DEF_POWER_SUPPLY 12.0f //!< default power supply voltage

// velocity PID controller params
#define DEF_PID_VEL_P 0.02f //!< default PID controller P value
#define DEF_PID_VEL_I 6.0f  //!<  default PID controller I value
#define DEF_PID_VEL_D 0.0f  //!<  default PID controller D value

// dq current PID controller params
#define DEF_PID_CURR_P 0.1f //!< default PID controller P value
#define DEF_PID_CURR_I 0.1f //!<  default PID controller I value
#define DEF_PID_CURR_D 0.0f //!<  default PID controller D value

// default current limit values
#define DEF_CURRENT_LIM 5.0f //!<  current limit by default
// default angle velocity limit values
#define DEF_VEL_LIM 150.0f //!< angle velocity limit by default

// index search
#define DEF_INDEX_SEARCH_TARGET_VELOCITY 1.0f //!< default index search velocity
// align voltage
#define DEF_VOLTAGE_SENSOR_ALIGN 3.0f //!< default voltage for sensor and motor zero alignemt

// lowpass filter default parameters
#define DEF_VEL_FILTER_Tf 0.005f  //!< default velocity filter time constant
#define DEF_CURR_FILTER_Tf 0.005f //!< default currnet filter time constant

#endif // DEFAULTS_H_