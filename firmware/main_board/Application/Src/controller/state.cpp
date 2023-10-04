/*
 * state.cpp
 *
 *  Created on: June 16th, 2023
 *      Author: Reiji Terunuma
 */

#include "controller/state.h"
void StateBase::UpdateIMUs()
{
    imu1->Update();
    imu2->Update();
    imu3->Update();
    imu4->Update();
    imu5->Update();
    imu6->Update();

    ConvertAxes();
    MeanAngularVelocity();
}

void StateBase::ConvertAxes()
{
    Pose temp_data;
    // imu1
    temp_data.x = imu1->gyro.x;
    temp_data.y = -imu1->gyro.z;
    temp_data.z = -imu1->gyro.y;
    imu1->gyro.x = temp_data.x;
    imu1->gyro.y = temp_data.y;
    imu1->gyro.z = temp_data.z;
    // imu1->gyro = temp_data;

    temp_data.x = imu1->acc.x;
    temp_data.y = -imu1->acc.z;
    temp_data.z = -imu1->acc.y;
    imu1->acc = temp_data;
}

void StateBase::MeanAngularVelocity()
{
    angular_vel.x = (imu1->gyro.x + imu2->gyro.x + imu3->gyro.x + imu4->gyro.x + imu5->gyro.x + imu6->gyro.x) / 6.0f;
    angular_vel.y = (imu1->gyro.y + imu2->gyro.y + imu3->gyro.y + imu4->gyro.y + imu5->gyro.y + imu6->gyro.y) / 6.0f;
    angular_vel.z = (imu1->gyro.z + imu2->gyro.z + imu3->gyro.z + imu4->gyro.z + imu5->gyro.z + imu6->gyro.z) / 6.0f;
}

void StateBase::CalcGravityVector()
{
    float coeff_M[18] =
        {imu1->acc.x, imu2->acc.x, imu3->acc.x, imu4->acc.x, imu5->acc.x, imu6->acc.x,
         imu1->acc.y, imu2->acc.y, imu3->acc.y, imu4->acc.y, imu5->acc.y, imu6->acc.y,
         imu1->acc.z, imu2->acc.z, imu3->acc.z, imu4->acc.z, imu5->acc.z, imu6->acc.z};
    arm_mat_init_f32(&mat_M, 3, 6, coeff_M);
    arm_mat_mult_f32(&mat_M, &mat_X, &vec_g);
}

void StateBase::CalcEulerAngle()
{
    euler.x = atan2(coeff_g[1], coeff_g[2]);
    euler.y = atan2(-coeff_g[0], std::sqrt(std::pow(coeff_g[1], 2) + std::pow(coeff_g[2], 2)));
}

void StateComplimantaryFilter::Initialize()
{
    arm_mat_init_f32(&vec_g, 3, 1, coeff_g);
    arm_mat_init_f32(&vec_x, 8, 1, coeff_x);
    arm_mat_init_f32(&mat_X, 6, 1, coeff_X);
}

void StateComplimantaryFilter::Update()
{
    UpdateIMUs();
    CalcGravityVector();
    CalcEulerAngle();

    Pose dot_euler;

    dot_euler.x = angular_vel.x + (angular_vel.y * sin(euler.x) + angular_vel.z * cos(euler.x)) * tan(euler.y);
    dot_euler.y = angular_vel.y * cos(euler.x) - angular_vel.z * sin(euler.x);
    dot_euler.z = (angular_vel.y * sin(euler.x) + angular_vel.z * cos(euler.x)) / cos(euler.y);

    filt_euler.x = kappa * euler.x + (1.0 - kappa) * (filt_euler.x + dot_euler.x * dt);
    filt_euler.y = kappa * euler.y + (1.0 - kappa) * (filt_euler.y + dot_euler.y * dt);
    filt_euler.z = kappa * euler.z + (1.0 - kappa) * (filt_euler.z + dot_euler.z * dt);

    euler.x = filt_euler.x;
    euler.y = filt_euler.y;
    euler.z = filt_euler.z;

    UpdateStateVector();
}

void StateComplimantaryFilter::UpdateStateVector()
{
    coeff_x[0] = euler.x;
    coeff_x[1] = euler.y;
    // coeff_x[2] = euler.z;
    coeff_x[2] = angular_vel.x;
    coeff_x[3] = angular_vel.y;
    coeff_x[4] = angular_vel.z;
    // coeff_x[5] = wheel_vel[0];
    // coeff_x[6] = wheel_vel[1];
    // coeff_x[7] = wheel_vel[2];
    arm_mat_init_f32(&vec_x, 8, 1, coeff_x);
}