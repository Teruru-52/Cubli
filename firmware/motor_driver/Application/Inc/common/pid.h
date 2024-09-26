#ifndef PID_H
#define PID_H

#include "main.h"

// 後退差分による離散化

/**
 * @brief class for integrator
 */
class Integrator
{
public:
    Integrator(float control_period, float bound_windup);

    float Update(float error);
    void ResetIntegrator();

private:
    float control_period;
    float bound_windup;
    float error_sum;
    float pre_error;
};

/**
 * @brief class for differentiator
 */
class Differentiator
{
public:
    Differentiator(float tf, float control_period);

    float Update(float error);
    void ResetDifferentiator();

private:
    float tf;
    float control_period;
    float coeff;
    float deriv;
    float pre_error;
    float pre_deriv;
};

/**
 * @brief class for PID controller
 */
class PID
{
public:
    PID(float kp, float ki, float kd, float tf, float control_period, float bound_windup);

    float Update(float error);
    void Reset();
    void OutputLog();

private:
    float kp;
    float ki;
    float kd;
    float error;
    float sum;
    float deriv;
    float input;
    Integrator integrator;
    Differentiator differentiator;
};

#endif //  PID_H