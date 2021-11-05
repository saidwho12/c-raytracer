
#ifndef MATH_H
#define MATH_H

template <typename T>
static T Min(const T& x, const T& y) { return x < y ? x : y; }

template <typename T>
static T Max(const T& x, const T& y) { return x > y ? x : y; }

static float Clamp(float const& x, float const& a, float const& b) {
    return Min(Max(x, a), b);
}

#endif //MATH_H
