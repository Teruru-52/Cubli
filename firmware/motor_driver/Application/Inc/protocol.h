/*
 * protocol.h
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <cstdint>
// #include "etl/deque.h"

namespace protocol
{
    // template <const size_t MAX_SIZE_>
    // using Buffer = etl::deque<uint8_t, MAX_SIZE_>;

    // using IBuffer = etl::ideque<uint8_t>;

    constexpr float max_duty = 1.0;
    constexpr float max_pulsewidth = 1.0 / 34000.0; // [s], PWM frequency 34[kHz]
    constexpr float max_voltage = 12.6;             // [V]

    float PulseWidth2Duty(float pulsewidth);
    // float Voltage2Duty(float voltage);
    // float Duty2Voltage(float duty);

    enum class MessageHeader : uint8_t
    {
        init,
        tx,
        rx,
        stop,
        free
    };

    struct Stop
    {
        MessageHeader header = MessageHeader::stop;
        template <class Archive>
        void serialize(Archive &ar)
        {
            ar(header);
        }
    };

    struct Free
    {
        MessageHeader header = MessageHeader::free;
        template <class Archive>
        void serialize(Archive &ar)
        {
            ar(header);
        }
    };

    namespace torque
    {
        struct Init
        {
            MessageHeader header = MessageHeader::init;
            uint16_t period;
            bool get_raw_current;

            template <class Archive>
            void serialize(Archive &ar)
            {
                ar(header, period, get_raw_current);
            }
        };

        // mb -> blmd
        struct TX
        {
            MessageHeader header = MessageHeader::tx;
            float torque;

            template <class Archive>
            void serialize(Archive &ar)
            {
                ar(header, torque);
            }
        };

        struct RX
        {
            MessageHeader header = MessageHeader::rx;
            int16_t pulse;
            int32_t pulse_sum;
            float torque;

            template <class Archive>
            void serialize(Archive &ar)
            {
                ar(header, pulse, pulse_sum, torque);
            }
        };

    } // namespace torque

} // namespace protocol

#endif // PROTOCOL_H_
