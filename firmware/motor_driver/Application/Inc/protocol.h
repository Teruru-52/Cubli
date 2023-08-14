/*
 * protocol.h
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

namespace protocol
{
    constexpr float max_duty = 1.0;
    constexpr float max_pulsewidth = 1.0 / 34000.0; // [s], PWM frequency 34[kHz]
    constexpr float max_voltage = 12.6;             // [V]

    float PulseWidth2Duty(float pulsewidth);
    // float Voltage2Duty(float voltage);
    // float Duty2Voltage(float duty);

    // enum class ControlMode : uint8_t
    // {
    //     velocity,
    //     torque
    // };

} // namespace protocol

#endif // PROTOCOL_H_
