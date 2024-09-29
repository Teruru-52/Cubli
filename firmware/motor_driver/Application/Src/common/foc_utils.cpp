#include "common/foc_utils.h"

// normalizing radian angle to [0,2PI]
float NormalizeAngle(float angle)
{
  float a = fmod(angle, M_2PI);
  return a >= 0 ? a : (a + M_2PI);
}

// int8_t NormalizeSector(int8_t sector)
// {
//   sector < 0 ? (sector + 6) : sector;
//   return sector > 5 ? (sector - 6) : sector;
// }

ab_t ClarkeTransform(const uvw_t &current_uvw)
{
  ab_t current_ab_;
  // current_ab_.a = current_uvw.u + current_uvw.v * cos23 + current_uvw.w * cos43;
  // current_ab_.b = current_uvw.v * sin23 + current_uvw.w * sin43;
  current_ab_.a = (current_uvw.u + current_uvw.v * cos23 + current_uvw.w * cos43) * sq23;
  current_ab_.b = (current_uvw.v * sin23 + current_uvw.w * sin43) * sq23;

  return current_ab_;
}

dq_t ParkTransform(const ab_t &current_ab, float electrical_angle)
{
  dq_t current_dq;
  current_dq.d = current_ab.a * arm_cos_f32(electrical_angle) + current_ab.b * arm_sin_f32(electrical_angle);
  current_dq.q = -current_ab.a * arm_sin_f32(electrical_angle) + current_ab.b * arm_cos_f32(electrical_angle);
  // arm_park_f32(current_ab.a, current_ab.b, &(current_dq.d), &(current_dq.q), arm_sin_f32(electrical_angle), arm_cos_f32(electrical_angle));

  return current_dq;
}

uvw_t InvClarkeTransform(const ab_t &voltage_ab)
{
  uvw_t voltage_uvw;
  // voltage_uvw.u = voltage_ab.a;
  // voltage_uvw.v = voltage_ab.a * cos23 + voltage_ab.b * sin23;
  // voltage_uvw.w = voltage_ab.a * cos43 + voltage_ab.b * sin43;
  voltage_uvw.u = voltage_ab.a * sq23;
  voltage_uvw.v = (voltage_ab.a * cos23 + voltage_ab.b * sin23) * sq23;
  voltage_uvw.w = (voltage_ab.a * cos43 + voltage_ab.b * sin43) * sq23;

  return voltage_uvw;
}

ab_t InvParkTransform(const dq_t &voltage_dq, float electrical_angle)
{
  ab_t voltage_ab;
  voltage_ab.a = voltage_dq.d * arm_cos_f32(electrical_angle) - voltage_dq.q * arm_sin_f32(electrical_angle);
  voltage_ab.b = voltage_dq.d * arm_sin_f32(electrical_angle) + voltage_dq.q * arm_cos_f32(electrical_angle);
  // arm_inv_park_f32(voltage_dq.d, voltage_dq.q, &(voltage_ab.a), &(voltage_ab.b), arm_sin_f32(electrical_angle), arm_cos_f32(electrical_angle));

  return voltage_ab;
}
