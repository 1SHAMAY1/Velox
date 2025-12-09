#pragma once

#include <velox/Types.h>
#include <cmath>
#include <iostream>

namespace Velox {

    struct Vec2 {
        Real x, y;

        Vec2() : x(0), y(0) {}
        Vec2(Real _x, Real _y) : x(_x), y(_y) {}

        Vec2 operator+(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
        Vec2 operator-(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
        Vec2 operator*(Real scalar) const { return Vec2(x * scalar, y * scalar); }
        Vec2 operator/(Real scalar) const { return Vec2(x / scalar, y / scalar); }
        
        Vec2& operator+=(const Vec2& other) { x += other.x; y += other.y; return *this; }
        Vec2& operator-=(const Vec2& other) { x -= other.x; y -= other.y; return *this; }
        Vec2& operator*=(Real scalar) { x *= scalar; y *= scalar; return *this; }

        Real Dot(const Vec2& other) const { return x * other.x + y * other.y; }
        Real MagnitudeSqr() const { return x * x + y * y; }
        Real Magnitude() const { return std::sqrt(MagnitudeSqr()); }
        
        Vec2 Normalized() const {
            Real mag = Magnitude();
            if (mag > 0) return *this / mag;
            return Vec2(0, 0);
        }

        Vec2 Rotate(Real angle) const {
            Real c = std::cos(angle);
            Real s = std::sin(angle);
            return Vec2(x * c - y * s, x * s + y * c);
        }

        static Vec2 Zero() { return Vec2(0, 0); }
    };

    inline Vec2 operator*(Real scalar, const Vec2& v) { return v * scalar; }

}
