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
}

void StateBase::UpdateWheelVels(float wheel_vel_x, float wheel_vel_y, float wheel_vel_z)
{
    wheel_vel.x = wheel_vel_x;
    wheel_vel.y = wheel_vel_y;
    wheel_vel.z = wheel_vel_z;
}

void StateBase::ConvertAxes()
{
    Pose temp_data;
    // imu1
    temp_data.x = -imu1->gyro.z;
    temp_data.y = -imu1->gyro.x;
    temp_data.z = imu1->gyro.y;
    imu1->gyro = temp_data;
    temp_data.x = -imu1->acc.z;
    temp_data.y = -imu1->acc.x;
    temp_data.z = imu1->acc.y;
    imu1->acc = temp_data;
    // imu2
    temp_data.x = -imu2->gyro.x;
    temp_data.y = imu2->gyro.y;
    temp_data.z = -imu2->gyro.z;
    imu2->gyro = temp_data;
    temp_data.x = -imu2->acc.x;
    temp_data.y = imu2->acc.y;
    temp_data.z = -imu2->acc.z;
    imu2->acc = temp_data;
    // imu3
    temp_data.x = -imu3->gyro.y;
    temp_data.y = imu3->gyro.z;
    temp_data.z = -imu3->gyro.x;
    imu3->gyro = temp_data;
    temp_data.x = -imu3->acc.y;
    temp_data.y = imu3->acc.z;
    temp_data.z = -imu3->acc.x;
    imu3->acc = temp_data;
    // imu4
    temp_data.x = imu4->gyro.y;
    temp_data.y = -imu4->gyro.z;
    temp_data.z = -imu4->gyro.x;
    imu4->gyro = temp_data;
    temp_data.x = imu4->acc.y;
    temp_data.y = -imu4->acc.z;
    temp_data.z = -imu4->acc.x;
    imu4->acc = temp_data;
    // imu5
    temp_data.x = -imu5->gyro.x;
    temp_data.y = -imu5->gyro.y;
    temp_data.z = imu5->gyro.z;
    imu5->gyro = temp_data;
    temp_data.x = -imu5->acc.x;
    temp_data.y = -imu5->acc.y;
    temp_data.z = imu5->acc.z;
    imu5->acc = temp_data;
    // imu6
    temp_data.x = imu6->gyro.z;
    temp_data.y = -imu6->gyro.x;
    temp_data.z = -imu6->gyro.y;
    imu6->gyro = temp_data;
    temp_data.x = imu6->acc.z;
    temp_data.y = -imu6->acc.x;
    temp_data.z = -imu6->acc.y;
    imu6->acc = temp_data;
}

void StateBase::MeanAngularVelocity()
{
    angular_vel.x = (imu1->gyro.x + imu2->gyro.x + imu3->gyro.x + imu4->gyro.x + imu5->gyro.x + imu6->gyro.x) / 6.0f;
    angular_vel.y = (imu1->gyro.y + imu2->gyro.y + imu3->gyro.y + imu4->gyro.y + imu5->gyro.y + imu6->gyro.y) / 6.0f;
    angular_vel.z = (imu1->gyro.z + imu2->gyro.z + imu3->gyro.z + imu4->gyro.z + imu5->gyro.z + imu6->gyro.z) / 6.0f;
}

void StateBase::CalcGravityVector()
{
    ConvertAxes();
    MeanAngularVelocity();
    float coeff_M[18] =
        {imu1->acc.x, imu2->acc.x, imu3->acc.x, imu4->acc.x, imu5->acc.x, imu6->acc.x,
         imu1->acc.y, imu2->acc.y, imu3->acc.y, imu4->acc.y, imu5->acc.y, imu6->acc.y,
         imu1->acc.z, imu2->acc.z, imu3->acc.z, imu4->acc.z, imu5->acc.z, imu6->acc.z};
    arm_mat_init_f32(&mat_M, 3, 6, coeff_M);
    arm_mat_mult_f32(&mat_M, &mat_X, &vec_g);
}

void StateBase::CalcEulerAngle()
{
    euler.x = -atan2(coeff_g[1], -coeff_g[2]);
    euler.y = atan2(coeff_g[0], std::sqrt(std::pow(coeff_g[1], 2) + std::pow(coeff_g[2], 2)));
}

void StateBase::UpdateStateVector()
{
    coeff_x[0] = euler.x;
    coeff_x[1] = euler.y;
    coeff_x[2] = angular_vel.x;
    coeff_x[3] = angular_vel.y;
    coeff_x[4] = angular_vel.z;
    coeff_x[5] = wheel_vel.x;
    coeff_x[6] = wheel_vel.y;
    coeff_x[7] = wheel_vel.z;
    arm_mat_init_f32(&vec_x, 8, 1, coeff_x);
}

void StateBase::LogPrint()
{
    // printf("gx = %.3f, gy = %.3f, gz = %.3f\n", imu1->gyro.x, imu1->gyro.y, imu1->gyro.z);
    // printf("gx = %.3f, gy = %.3f, gz = %.3f\n", imu2->gyro.x, imu2->gyro.y, imu2->gyro.z);
    // printf("gx = %.3f, gy = %.3f, gz = %.3f\n", imu3->gyro.x, imu3->gyro.y, imu3->gyro.z);
    // printf("gx = %.3f, gy = %.3f, gz = %.3f\n", imu4->gyro.x, imu4->gyro.y, imu4->gyro.z);
    // printf("gx = %.3f, gy = %.3f, gz = %.3f\n", imu5->gyro.x, imu5->gyro.y, imu5->gyro.z);
    // printf("gx = %.3f, gy = %.3f, gz = %.3f\n", imu6->gyro.x, imu6->gyro.y, imu6->gyro.z);

    // printf("gx1 = %.3f, gx2 = %.3f, gx3 = %.3f, gx4 = %.3f, gx5 = %.3f, gx6 = %.3f\n", imu1->gyro.x, imu2->gyro.x, imu3->gyro.x, imu4->gyro.x, imu5->gyro.x, imu6->gyro.x);
    // printf("gy1 = %.3f, gy2 = %.3f, gy3 = %.3f, gy4 = %.3f, gy5 = %.3f, gy6 = %.3f\n", imu1->gyro.y, imu2->gyro.y, imu3->gyro.y, imu4->gyro.y, imu5->gyro.y, imu6->gyro.y);
    // printf("gz1 = %.3f, gz2 = %.3f, gz3 = %.3f, gz4 = %.3f, gz5 = %.3f, gz6 = %.3f\n", imu1->gyro.z, imu2->gyro.z, imu3->gyro.z, imu4->gyro.z, imu5->gyro.z, imu6->gyro.z);

    // printf("az1 = %.3f, az2 = %.3f, az3 = %.3f, az4 = %.3f, az5 = %.3f, az6 = %.3f\n", imu1->acc.y, imu2->acc.z, imu3->acc.x, imu4->acc.x, imu5->acc.z, imu6->acc.y);
    // printf("ax1 = %.3f, ax2 = %.3f, ax3 = %.3f, ax4 = %.3f, ax5 = %.3f, ax6 = %.3f\n", imu1->acc.x, imu2->acc.x, imu3->acc.x, imu4->acc.x, imu5->acc.x, imu6->acc.x);
    // printf("ay1 = %.3f, ay2 = %.3f, ay3 = %.3f, ay4 = %.3f, ay5 = %.3f, ay6 = %.3f\n", imu1->acc.y, imu2->acc.y, imu3->acc.y, imu4->acc.y, imu5->acc.y, imu6->acc.y);
    // printf("az1 = %.3f, az2 = %.3f, az3 = %.3f, az4 = %.3f, az5 = %.3f, az6 = %.3f\n", imu1->acc.z, imu2->acc.z, imu3->acc.z, imu4->acc.z, imu5->acc.z, imu6->acc.z);

    // printf("gx = %.3f, gy = %.3f, gz = %.3f\n", angular_vel.x, angular_vel.y, angular_vel.z);
    // printf("ax = %.3f, ay = %.3f, az = %.3f\n", coeff_g[0], coeff_g[1], coeff_g[2]);
    // printf("roll = %.3f, pitch = %.3f\n", euler.x, euler.y);
}

void StateComplimantaryFilter::Initialize()
{
    arm_mat_init_f32(&vec_g, 3, 1, coeff_g);
    arm_mat_init_f32(&vec_x, 8, 1, coeff_x);
    arm_mat_init_f32(&mat_X, 6, 1, coeff_X);

    Write_GPIO(LED_YELLOW, GPIO_PIN_SET);

    Write_GPIO(LED_GREEN, GPIO_PIN_SET);
    imu1->Initialize();
    imu1->Calibrate();

    Write_GPIO(LED_GREEN, GPIO_PIN_RESET);
    Write_GPIO(LED_BLUE, GPIO_PIN_SET);
    imu2->Initialize();
    imu2->Calibrate();

    Write_GPIO(LED_GREEN, GPIO_PIN_SET);
    imu3->Initialize();
    imu3->Calibrate();

    Write_GPIO(LED_GREEN, GPIO_PIN_RESET);
    Write_GPIO(LED_BLUE, GPIO_PIN_RESET);
    Write_GPIO(LED_WHITE, GPIO_PIN_SET);
    imu4->Initialize();
    imu4->Calibrate();

    Write_GPIO(LED_GREEN, GPIO_PIN_SET);
    imu5->Initialize();
    imu5->Calibrate();

    Write_GPIO(LED_GREEN, GPIO_PIN_RESET);
    Write_GPIO(LED_BLUE, GPIO_PIN_SET);
    imu6->Initialize();
    imu6->Calibrate();

    Write_GPIO(LED_GREEN, GPIO_PIN_RESET);
    Write_GPIO(LED_BLUE, GPIO_PIN_RESET);
    Write_GPIO(LED_WHITE, GPIO_PIN_RESET);
    Write_GPIO(LED_YELLOW, GPIO_PIN_RESET);
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
