#pragma once

template <typename T>
T abs(const T &val)
{
    return val > 0 ? val : -val;
}

template <typename T>
constexpr T clamp(T x, T min, T max)
{
    return x < min ? min : x > max ? max : x;
}

template <typename T>
constexpr T clamp(T x, T absLimit)
{
    return clamp(x, -absLimit, absLimit);
}

template <typename T>
constexpr T isReached(T x, T target, T torlorence)
{
    return ((x) < (target + torlorence) && (x) > (target - torlorence)) ? true : false;
}

template <typename T>
constexpr T mapValue(T value, double inMin, double inMax, double outMin, double outMax)
{
    return outMin + (value - inMin) * (outMax - outMin) / (inMax - inMin);
}