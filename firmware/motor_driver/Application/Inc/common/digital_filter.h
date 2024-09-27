/*
 * digital_filter.h
 *
 *  Created on: Mar 4th, 2024
 *      Author: Reiji Terunuma
 */

#ifndef DIGITAL_FILTER_H_
#define DIGITAL_FILTER_H_

#include "main.h"
#include <deque>
#include <algorithm>

class DigitalFilter
{
public:
    DigitalFilter() = default;
    virtual ~DigitalFilter() = default;
    virtual float Update(float cur_val) = 0;
};

class MovingAverageFilter : public DigitalFilter
{
public:
    explicit MovingAverageFilter(uint16_t win_size = 10) : DigitalFilter(), win_size(win_size) {}
    ~MovingAverageFilter() = default;
    float Update(float cur_val) override
    {
        if (buffer.size() > win_size)
            buffer.pop_front();
        buffer.push_back(cur_val);
        float sum = 0;
        for (auto &past_val : buffer)
            sum += past_val;
        return sum / buffer.size();
    }

private:
    uint16_t win_size; // window size
    std::deque<float> buffer;
};

class MedianFilter : public DigitalFilter
{
public:
    explicit MedianFilter(uint16_t win_size = 3) : DigitalFilter(), win_size(win_size) {}
    ~MedianFilter() = default;
    float Update(float cur_val) override
    {
        if (buffer.size() > win_size)
            buffer.pop_front();
        buffer.push_back(cur_val);
        std::deque<float> tmp_buffer = buffer;
        std::sort(tmp_buffer.begin(), tmp_buffer.end());
        return tmp_buffer[tmp_buffer.size() / 2];
    }

private:
    uint16_t win_size; // window size
    std::deque<float> buffer;
};

class LowPassFilter : public DigitalFilter
{
public:
    explicit LowPassFilter(float Tf, float dt = 0.001f)
        : DigitalFilter(),
          Tf(Tf),
          dt(dt),
          prev_val(0.0f)
    {
        alpha = Tf / (Tf + dt);
    }
    ~LowPassFilter() = default;

    float Update(float cur_val) override
    {
        float y = alpha * prev_val + (1.0f - alpha) * cur_val;
        prev_val = y;
        return y;
    }

protected:
    float Tf; //!< Low pass filter time constant
    float dt;
    float prev_val; //!< filtered value in previous execution step
    float alpha;
};

#endif // DIGITAL_FILTER_H_