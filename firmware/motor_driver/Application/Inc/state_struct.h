/*
 * state_struct.h
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef STATE_STRUCT_H_
#define STATE_STRUCT_H_

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

#endif // STATE_STRUCT_H_