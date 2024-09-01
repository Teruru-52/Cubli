/*
 * foc_utils.h
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef FOC_UTILS_H_
#define FOC_UTILS_H_

#include "main.h"

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
const float cos23 = cos((2.0f / 3.0f) * M_PI);
const float cos43 = cos((4.0f / 3.0f) * M_PI);
const float sin23 = sin((2.0f / 3.0f) * M_PI);
const float sin43 = sin((4.0f / 3.0f) * M_PI);
const float sq23 = sqrt(2.0f / 3.0f);

// coefficient of SVPWM
const float sq13 = 1.0f / sqrt(3.0f);
const float sq32 = sqrt(3.0f / 2.0f);
const float sq43 = 2.0f / sqrt(3.0f);

#endif // FOC_UTILS_H_