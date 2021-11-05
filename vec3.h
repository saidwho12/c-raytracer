#ifndef VEC3_H
#define VEC3_H

#include "math.h"

struct Vector3f {
    float x, y, z;

    constexpr Vector3f(const float &v) : x(v), y(v), z(v) {}
    constexpr Vector3f(const float &a, const float &b, const float &c) : x(a), y(b), z(c) {}
    constexpr Vector3f() : x(0.0f), y(0.0f), z(0.0f) {}
    ~Vector3f() = default;

    Vector3f operator +(const float &other) const {
        return Vector3f(x + other, y + other, z + other);
    }

    Vector3f operator -(const float &other) const {
        return Vector3f(x - other, y - other, z - other);
    }

    Vector3f operator *(const float &other) const {
        return Vector3f(x * other, y * other, z * other);
    }

    Vector3f operator /(const float &other) const {
        return Vector3f(x / other, y / other, z / other);
    }

    Vector3f operator +(const Vector3f &other) const {
        return Vector3f(x + other.x, y + other.y, z + other.z);
    }

    Vector3f operator -(const Vector3f &other) const {
        return Vector3f(x - other.x, y - other.y, z - other.z);
    }

    Vector3f operator *(const Vector3f &other) const {
        return Vector3f(x * other.x, y * other.y, z * other.z);
    }

    Vector3f operator /(const Vector3f &other) const {
        return Vector3f(x / other.x, y / other.y, z / other.z);
    }

    Vector3f operator +=(const Vector3f &other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vector3f operator -=(const Vector3f &other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vector3f operator *=(const Vector3f &other) {
        x *= other.x;
        y *= other.y;
        z *= other.z;
        return *this;
    }

    Vector3f operator /=(const Vector3f &other) {
        x /= other.x;
        y /= other.y;
        z /= other.z;
        return *this;
    }
};

static float Length(const Vector3f &v) {
    return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

static Vector3f Normalize(const Vector3f &v) {
    float len = Length(v);
    return v / len;
}

static Vector3f Saturate(const Vector3f &v) {
    return Vector3f(Clamp(v.x, 0.0f, 1.0f), Clamp(v.y, 0.0f, 1.0f), Clamp(v.z, 0.0f, 1.0f));
}

static Vector3f Cross(const Vector3f& a, const Vector3f& b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

static float Dot(const Vector3f& a, const Vector3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

using Point3f = Vector3f;

#endif //VEC3_H
