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

class MovingAverageFilter
{
public:
    explicit MovingAverageFilter(uint16_t win_size = 100) : win_size(win_size) {}
    float Update(float cur_val)
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

class MedianFilter
{
public:
    explicit MedianFilter(uint16_t win_size = 3) : win_size(win_size) {}
    float Update(float cur_val)
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

#endif // DIGITAL_FILTER_H_