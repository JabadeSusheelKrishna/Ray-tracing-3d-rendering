#pragma once

// Implementations taken from PBRT v3 (https://github.com/mmp/pbrt-v3/blob/master/src/core/geometry.h)

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <chrono>
#include <cmath>

template <typename T>
inline bool isNaN(const T x) {
    return std::isnan(x);
}
template <>
inline bool isNaN(const int x) {
    return false;
}

template <typename T>
class Vector2 {
public:
    // Vector2 Public Methods
    Vector2() { x = y = 0; }
    Vector2(T xx, T yy) : x(xx), y(yy) {  }
    bool HasNaNs() const { return isNaN(x) || isNaN(y); }

    Vector2<T> operator+(const Vector2<T>& v) const {
        return Vector2(x + v.x, y + v.y);
    }

    Vector2<T>& operator+=(const Vector2<T>& v) {
        x += v.x;
        y += v.y;
        return *this;
    }
    Vector2<T> operator-(const Vector2<T>& v) const {
        return Vector2(x - v.x, y - v.y);
    }

    Vector2<T>& operator-=(const Vector2<T>& v) {
        x -= v.x;
        y -= v.y;
        return *this;
    }
    bool operator==(const Vector2<T>& v) const { return x == v.x && y == v.y; }
    bool operator!=(const Vector2<T>& v) const { return x != v.x || y != v.y; }

    template <typename U>
    Vector2<T> operator*(U f) const {
        return Vector2<T>(f * x, f * y);
    }
    Vector2<T> operator*(const Vector2<T>& v) const {
        return Vector2<T>(v.x * x, v.y * y);
    }

    template <typename U>
    Vector2<T>& operator*=(U f) {
        x *= f;
        y *= f;
        return *this;
    }
    Vector2<T>& operator*=(const Vector2<T>& v) {
        x *= v.x;
        y *= v.y;
        return *this;
    }

    template <typename U>
    Vector2<T> operator/(U f) const {
        float inv = (float)1 / f;
        return Vector2<T>(x * inv, y * inv);
    }
    Vector2<T> operator/(const Vector2<T>& v) const {
        return Vector2<T>(x / v.x, y / v.y);
    }

    template <typename U>
    Vector2<T>& operator/=(U f) {
        float inv = (float)1 / f;
        x *= inv;
        y *= inv;
        return *this;
    }
    Vector2<T>& operator/=(const Vector2<T>& v) {
        x /= v.x;
        y /= v.y;
        return *this;
    }

    Vector2<T> operator-() const { return Vector2<T>(-x, -y); }
    T operator[](int i) const {
        if (i == 0) return x;
        return y;
    }

    T& operator[](int i) {
        if (i == 0) return x;
        return y;
    }
    float LengthSquared() const { return x * x + y * y; }
    float Length() const { return std::sqrt(LengthSquared()); }

    // Vector2 Public Data
    T x, y;
};

template <typename T>
class Vector3 {
public:
    // Vector3 Public Methods
    T operator[](int i) const {
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }
    T& operator[](int i) {
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }
    Vector3() { x = y = z = 0; }
    Vector3(T x, T y, T z) : x(x), y(y), z(z) { }
    bool HasNaNs() const { return isNaN(x) || isNaN(y) || isNaN(z); }

    Vector3<T> operator+(const Vector3<T>& v) const {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }
    Vector3<T>& operator+=(const Vector3<T>& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }
    Vector3<T> operator-(const Vector3<T>& v) const {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }
    Vector3<T>& operator-=(const Vector3<T>& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }
    bool operator==(const Vector3<T>& v) const {
        return x == v.x && y == v.y && z == v.z;
    }
    bool operator!=(const Vector3<T>& v) const {
        return x != v.x || y != v.y || z != v.z;
    }

    template <typename U>
    Vector3<T> operator*(U s) const {
        return Vector3<T>(s * x, s * y, s * z);
    }
    Vector3<T> operator*(const Vector3<T>& v) const {
        return Vector3<T>(v.x * x, v.y * y, v.z * z);
    }

    template <typename U>
    Vector3<T>& operator*=(U s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }
    Vector3<T>& operator*=(const Vector3<T>& v) {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        return *this;
    }

    template <typename U>
    Vector3<T> operator/(U f) const {
        float inv = (float)1 / f;
        return Vector3<T>(x * inv, y * inv, z * inv);
    }
    Vector3<T> operator/(const Vector3<T>& v) const {
        return Vector3<T>(x / v.x, y / v.y, z / v.z);
    }

    template <typename U>
    Vector3<T>& operator/=(U f) {
        float inv = (float)1 / f;
        x *= inv;
        y *= inv;
        z *= inv;
        return *this;
    }
    Vector3<T>& operator/=(const Vector3<T>& v) {
        x /= v.x;
        y /= v.y;
        z /= v.z;
        return *this;
    }

    Vector3<T> operator-() const { return Vector3<T>(-x, -y, -z); }
    float LengthSquared() const { return x * x + y * y + z * z; }
    float Length() const { return std::sqrt(LengthSquared()); }

    // Vector3 Public Data
    T x, y, z;
};

typedef Vector2<float> Vector2f;
typedef Vector2<int> Vector2i;
typedef Vector3<float> Vector3f;
typedef Vector3<int> Vector3i;

template <typename T, typename U>
inline Vector3<T> operator*(U s, const Vector3<T>& v) {
    return v * s;
}

template <typename T>
Vector3<T> Abs(const Vector3<T>& v) {
    return Vector3<T>(std::abs(v.x), std::abs(v.y), std::abs(v.z));
}

template <typename T>
inline T Dot(const Vector3<T>& v1, const Vector3<T>& v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

template <typename T>
inline T AbsDot(const Vector3<T>& v1, const Vector3<T>& v2) {
    return std::abs(Dot(v1, v2));
}

template <typename T>
inline Vector3<T> Cross(const Vector3<T>& v1, const Vector3<T>& v2) {
    double v1x = v1.x, v1y = v1.y, v1z = v1.z;
    double v2x = v2.x, v2y = v2.y, v2z = v2.z;
    return Vector3<T>((v1y * v2z) - (v1z * v2y), (v1z * v2x) - (v1x * v2z),
        (v1x * v2y) - (v1y * v2x));
}

template <typename T>
inline Vector3<T> Normalize(const Vector3<T>& v) {
    return v / v.Length();
}

template <typename T, typename U>
inline Vector2<T> operator*(U f, const Vector2<T>& v) {
    return v * f;
}

template <typename T>
inline float Dot(const Vector2<T>& v1, const Vector2<T>& v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

template <typename T>
inline float AbsDot(const Vector2<T>& v1, const Vector2<T>& v2) {
    return std::abs(Dot(v1, v2));
}

template <typename T>
inline Vector2<T> Normalize(const Vector2<T>& v) {
    return v / v.Length();
}
template <typename T>
Vector2<T> Abs(const Vector2<T>& v) {
    return Vector2<T>(std::abs(v.x), std::abs(v.y));
}
