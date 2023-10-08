/*
 * controller.h
 *
 *  Created on: June 16th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "main.h"
#include "controller/state.h"

class ControllerBase
{
protected:
    float coeff_u[3]; // input
    Pose u_torque;

public:
    arm_matrix_instance_f32 vec_u;

    explicit ControllerBase() {}
    virtual void Initialize() = 0;
    virtual void CalcInput(arm_matrix_instance_f32 vec_x) = 0;
    void ResetInput() { u_torque.clear(); };
    Pose GetInput() { return u_torque; };
    virtual ~ControllerBase() {}
};

class LQR : public ControllerBase
{
private:
    float32_t coeff_K[64];
    arm_matrix_instance_f32 mat_K; // feedback gain

public:
    explicit LQR() : ControllerBase(){};
    void Initialize() override;
    void CalcInput(arm_matrix_instance_f32 vec_x) override;
};

#endif // CONTROLLER_H_