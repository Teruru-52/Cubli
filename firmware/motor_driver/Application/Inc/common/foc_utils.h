/*
 * foc_utils.h
 *
 *  Created on: Aug 15th, 2024
 *      Author: Reiji Terunuma
 */

#ifndef FOC_UTILS_H_
#define FOC_UTILS_H_

#include "main.h"

#define _sign(a) (((a) < 0) ? -1 : ((a) > 0))
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define _isset(a) ((a) != (NOT_SET))

#define NOT_SET -12345.0f
#define _HIGH_IMPEDANCE 0
#define M_2PI 2.0f * M_PI
#define M_PI_3 M_PI / 3.0f
#define M_3PI_2 1.5f * M_PI
#define M_PI_6 M_PI / 6.0f

// variables on u-v-w axis
struct uvw_t
{
    float u;
    float v;
    float w;

public:
    uvw_t()
        : u(0.0), v(0.0), w(0.0) {}
};

// variables on a-b axis
struct ab_t
{
    float a;
    float b;

public:
    ab_t()
        : a(0.0), b(0.0) {}
};

// variables on d-q axis
struct dq_t
{
    float d;
    float q;

public:
    dq_t()
        : d(0.0), q(0.0) {}
};

// coefficient of Park and Clarke transform
const float cos23 = -0.5f;
const float cos43 = -0.5f;
const float sin23 = sin((2.0f / 3.0f) * M_PI);
const float sin43 = sin((4.0f / 3.0f) * M_PI);
const float sq23 = sqrt(2.0f / 3.0f);

// coefficient of SVPWM
const float sq13 = 1.0f / sqrt(3.0f);
const float sq32 = sqrt(3.0f / 2.0f);
const float sq43 = 2.0f / sqrt(3.0f);

float NormalizeAngle(float angle);

ab_t ClarkeTransform(const uvw_t &current_uvw);
dq_t ParkTransform(const ab_t &current_ab, float electrical_angle);
uvw_t InvClarkeTransform(const ab_t &voltage_ab);
ab_t InvParkTransform(const dq_t &voltage_dq, float electrical_angle);

#endif // FOC_UTILS_H_