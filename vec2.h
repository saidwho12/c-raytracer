#ifndef VEC2_H
#define VEC2_H

struct Vector2f {
    float x, y;

    Vector2f() : x(0.0f), y(0.0f) {}
    Vector2f(float const& a, float const& b) : x(a), y(b) {}
    Vector2f(float const& v) : x(v), y(v) {}
    ~Vector2f() {};


    Vector2f operator +(const float &other) const {
        return Vector2f(x + other, y + other);
    }

    Vector2f operator -(const float &other) const {
        return Vector2f(x - other, y - other);
    }

    Vector2f operator *(const float &other) const {
        return Vector2f(x * other, y * other);
    }

    Vector2f operator /(const float &other) const {
        return Vector2f(x / other, y / other);
    }

    Vector2f operator +(const Vector2f &other) const {
        return Vector2f(x + other.x, y + other.y);
    }

    Vector2f operator -(const Vector2f &other) const {
        return Vector2f(x - other.x, y - other.y);
    }

    Vector2f operator *(const Vector2f &other) const {
        return Vector2f(x * other.x, y * other.y);
    }

    Vector2f operator /(const Vector2f &other) const {
        return Vector2f(x / other.x, y / other.y);
    }
};

#endif //VEC2_H
