/*
 * controller.cpp
 *
 *  Created on: June 16th, 2023
 *      Author: Reiji Terunuma
 */

#include "controller/controller.h"

void LQR::Initialize()
{
    arm_mat_init_f32(&mat_K, 8, 8, coeff_K);
    arm_mat_init_f32(&vec_u, 3, 1, coeff_u);
}

void LQR::CalcInput(arm_matrix_instance_f32 vec_x)
{
    arm_mat_mult_f32(&mat_K, &vec_x, &vec_u);
    u_torque.x = coeff_u[0];
    u_torque.y = coeff_u[1];
    u_torque.z = coeff_u[2];
}
