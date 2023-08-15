/*
 * pose.h
 *
 *  Created on: Aug 13th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef POSE_H_
#define POSE_H_

#include "main.h"
#include <iostream>

struct Pose
{
    float x; /**< @brief x */
    float y; /**< @brief y */
    float z; /**< @brief z */

public:
    Pose(const float x = 0, const float y = 0, const float z = 0)
        : x(x), y(y), z(z) {}
    void clear() { x = y = z = 0; }
    Pose &operator+=(const Pose &o)
    {
        return x += o.x, y += o.y, z += o.z, *this;
    }
    Pose &operator-=(const Pose &o)
    {
        return x -= o.x, y -= o.y, z -= o.z, *this;
    }
    Pose operator+(const Pose &o) const { return {x + o.x, y + o.y, z + o.z}; }
    Pose operator-(const Pose &o) const { return {x - o.x, y - o.y, z - o.z}; }
    friend std::ostream &operator<<(std::ostream &os, const Pose &o)
    {
        return os << "(" << o.x << ", " << o.y << ", " << o.z << ")";
    }
};

#endif // POSE_H_