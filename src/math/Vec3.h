#pragma once

#include <cmath>
#include <iostream>

namespace Velox {

    struct Vec3 {
        Real x, y, z;

        Vec3() : x(0), y(0), z(0) {}
        Vec3(Real _x, Real _y, Real _z) : x(_x), y(_y), z(_z) {}

        Vec3 operator+(const Vec3& other) const { return Vec3(x + other.x, y + other.y, z + other.z); }
        Vec3 operator-(const Vec3& other) const { return Vec3(x - other.x, y - other.y, z - other.z); }
        Vec3 operator*(Real scalar) const { return Vec3(x * scalar, y * scalar, z * scalar); }
        Vec3 operator/(Real scalar) const { return Vec3(x / scalar, y / scalar, z / scalar); }

        Vec3& operator+=(const Vec3& other) { x += other.x; y += other.y; z += other.z; return *this; }
        Vec3& operator-=(const Vec3& other) { x -= other.x; y -= other.y; z -= other.z; return *this; }
        Vec3& operator*=(Real scalar) { x *= scalar; y *= scalar; z *= scalar; return *this; }

        Real Dot(const Vec3& other) const { return x * other.x + y * other.y + z * other.z; }
        Vec3 Cross(const Vec3& other) const {
            return Vec3(
                y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x
            );
        }

        Real MagnitudeSquared() const { return x * x + y * y + z * z; }
        Real Magnitude() const { return std::sqrt(MagnitudeSquared()); }

        Vec3 Normalized() const {
            Real mag = Magnitude();
            if (mag > 0) return *this / mag;
            return Vec3(0, 0, 0);
        }
        
        static Vec3 Zero() { return Vec3(0, 0, 0); }
    };
    
    inline Vec3 operator*(Real scalar, const Vec3& v) { return v * scalar; }

}
