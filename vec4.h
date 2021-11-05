#ifndef VEC4_H
#define VEC4_H

struct Vector4f {
    float x, y, z, w;

    constexpr Vector4f(const float &v) : x(v), y(v), z(v) {}
    constexpr Vector4f(const float &a, const float &b, const float &c) : x(a), y(b), z(c) {}
    constexpr Vector4f() : x(0.0f), y(0.0f), z(0.0f) {}
    ~Vector4f() = default;

    Vector4f operator +(const float &other) const {
        return Vector4f(x + other, y + other, z + other);
    }

    Vector4f operator -(const float &other) const {
        return Vector4f(x - other, y - other, z - other);
    }

    Vector4f operator *(const float &other) const {
        return Vector4f(x * other, y * other, z * other);
    }

    Vector4f operator /(const float &other) const {
        return Vector4f(x / other, y / other, z / other);
    }

    Vector4f operator +(const Vector4f &other) const {
        return Vector4f(x + other.x, y + other.y, z + other.z);
    }

    Vector4f operator -(const Vector4f &other) const {
        return Vector4f(x - other.x, y - other.y, z - other.z);
    }

    Vector3f operator *(const Vector4f &other) const {
        return Vector4f(x * other.x, y * other.y, z * other.z);
    }

    Vector4f operator /(const Vector4f &other) const {
        return Vector4f(x / other.x, y / other.y, z / other.z);
    }
};

static float length(const Vector4f &v) {
    return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z + v.w*v.w);
}

static Vector4f normalize(const Vector4f &v) {
    float len = length(v);
    return v / len;
}


#endif //VEC4_H
