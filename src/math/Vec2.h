#pragma once

/**
 * @file Vec2.h
 * @brief 2D vector type used throughout the math, physics, and ECS layers.
 */

#include <velox/Types.h>
#include <cmath>
#include <iostream>

namespace Velox {

    /// A 2-component vector with basic arithmetic and geometric operations.
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

        /// Dot product with another vector.
        Real Dot(const Vec2& other) const { return x * other.x + y * other.y; }

        /// Squared length. Prefer over Magnitude() when only comparing distances.
        Real MagnitudeSqr() const { return x * x + y * y; }

        /// Euclidean length of the vector.
        Real Magnitude() const { return std::sqrt(MagnitudeSqr()); }

        /// Returns a unit-length copy of this vector, or (0,0) if the vector is zero-length.
        Vec2 Normalized() const {
            Real mag = Magnitude();
            if (mag > 0) return *this / mag;
            return Vec2(0, 0);
        }

        /// Returns this vector rotated counter-clockwise by `angle` radians.
        Vec2 Rotate(Real angle) const {
            Real c = std::cos(angle);
            Real s = std::sin(angle);
            return Vec2(x * c - y * s, x * s + y * c);
        }

        /// Convenience constant for the zero vector.
        static Vec2 Zero() { return Vec2(0, 0); }
    };

    /// Allows `scalar * vector` in addition to `vector * scalar`.
    inline Vec2 operator*(Real scalar, const Vec2& v) { return v * scalar; }

}