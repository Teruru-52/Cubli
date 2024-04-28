/*
 * motor_param.h
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef MOTOR_PARAM_H_
#define MOTOR_PARAM_H_

#include "main.h"

const float Ra = 64e-3f;
const float Ld = 0.5e-3f;
const float Lq = 0.5e-3f;
const float Pn = 8.0f; // Number of magnetic pole pairs
const float Ke = 0.1f;
const float Ktau = 1.0f;

// Filter for observer
// struct MotorCurrentControlParam
// {
// 	float y2i_coeff[5] = {0.0};
// 	float y2i_buffer[2] = {0.0};
// 	float y2i_out = 0;

// 	float u2i_coeff[5] = {0.0};
// 	float u2i_buffer[2] = {0.0};
// 	float u2i_out = 0;

// 	float y2e_coeff[5] = {0.0};
// 	float y2e_buffer[2] = {0.0};
// 	float y2e_out = 0;

// 	float u2e_coeff[5] = {0.0};
// 	float u2e_buffer[2] = {0.0};
// 	float u2e_out = 0;
// };

#endif // MOTOR_PARAM_H_
