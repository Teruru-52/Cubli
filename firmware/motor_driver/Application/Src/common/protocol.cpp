/*
 * protocol.cpp
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#include "common/protocol.h"

namespace protocol
{
    float PulseWidth2Duty(float pulsewidth)
    {
        return pulsewidth / max_pulsewidth;
    }

    // float Voltage2Duty(float voltage)
    // {
    //     return voltage / max_voltage;
    // }

    // float Duty2Voltage(float duty)
    // {
    //     return duty * max_voltage;
    // }
} // namespace protocol