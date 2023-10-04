/*
 * state.h
 *
 *  Created on: June 16th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef STATE_H_
#define STATE_H_

#include "main.h"
// #include <cmath>
#include "hardware/imu.h"
#include "controller/pose.h"

using hardware::IMUBase;

class StateBase
{
protected:
    IMUBase *imu1;
    IMUBase *imu2;
    IMUBase *imu3;
    IMUBase *imu4;
    IMUBase *imu5;
    IMUBase *imu6;
    float coeff_g[3];
    // default
    // float coeff_X[6] = {0.8639, 0.3746, -0.2096, 0.0165, 0.2052, -0.2506};
    // original
    float coeff_X[6] = {0.6708, 0.3034, 0.0613, 0.2721, 0.0299, -0.3375};
    float coeff_x[8];
    arm_matrix_instance_f32 vec_g;
    arm_matrix_instance_f32 mat_M;
    arm_matrix_instance_f32 mat_X;

    Pose euler; // ZYX euler
    Pose angular_vel;

    void ConvertAxes();
    void MeanAngularVelocity();
    void CalcGravityVector();
    void CalcEulerAngle();
    void UpdateStateVector();

public:
    float dt;
    arm_matrix_instance_f32 vec_x;

    explicit StateBase(IMUBase *imu1, IMUBase *imu2, IMUBase *imu3, IMUBase *imu4, IMUBase *imu5, IMUBase *imu6)
        : imu1(imu1), imu2(imu2), imu3(imu3), imu4(imu4), imu5(imu5), imu6(imu6) {}
    virtual void Initialize() = 0;
    virtual void Update() = 0;
    void UpdateIMUs();
    void LogPrint();
    virtual ~StateBase() {}
};

class StateComplimantaryFilter : public StateBase
{
private:
    float kappa;
    Pose filt_euler;

public:
    explicit StateComplimantaryFilter(IMUBase *imu1, IMUBase *imu2, IMUBase *imu3, IMUBase *imu4, IMUBase *imu5, IMUBase *imu6) : StateBase(imu1, imu2, imu3, imu4, imu5, imu6){};
    void Initialize() override;
    void Update() override;
};

#endif // STATE_H_