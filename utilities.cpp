#include "capstone.hpp"

/// converts float to int, given range and number of bits
int float2int(const float x, const float x_min, const float x_max, const int bits) // don't touch me
{
    const float span = x_max - x_min;
    const float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/// converts unsigned int to float, given range and number of bits
float int2float(const int x_int, const float x_min, const float x_max, const int bits) // don't touch me
{
    const float span = x_max - x_min;
    const float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/// scales the lenght of vector (x, y) to be <= limit
void limitNorm(float &x, float &y, const float limit)
{
    const float norm = sqrt(x * x + y * y);
    if (norm > limit) {
        x *= limit / norm;
        y *= limit / norm;
    }
}

/// returns the 2nd largest value among x, y and z
float middle(const float x, const float y, const float z)
{
    if (y <= x && x <= z || z <= x && x <= y) {
        return x;
    }
    if (x <= y && y <= z || z <= y && y <= x) {
        return y;
    }
    return z;
}

/// returns the current time
double getTime()
{
    return timer.read();
}

/// returns whether the two astrings are the same
bool areSameStr(const char *const lhs, const char *const rhs)
{
    if (lhs == NULL || rhs == NULL) {
        return false;
    }
    return strcmp(lhs, rhs) == 0;
}

/// returns wheter x is in the interval [left, right]
bool inRange(const float left, const float x, const float right)
{
    return (left <= x) && (x <= right);
}

Gear::Gear(const int gear)
    : gear(gear), gear_cnt(0)
{
}

Gear::Gear(const Gear &other)
    : gear(other.gear), gear_cnt(other.gear_cnt)
{
}

Gear::~Gear()
{
}

bool Gear::go()
{
    if ((gear_cnt + 1) % gear == 0) {
        gear_cnt = 0;
        return true;
    }
    if (gear_cnt >= 0) {
        gear_cnt++;
    }
    else {
        gear_cnt = 0;
    }
    return false;
}

void Gear::reset()
{
    gear_cnt = 0;
}
