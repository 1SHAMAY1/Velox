#pragma once

/**
 * @file Vec2.h
 * @brief 2D vector type used throughout the math, physics, and ECS layers.
 */

#include <velox/Types.h>
#include <cmath>
#include <iostream>

#ifdef VELOX_SIMD
#include <xmmintrin.h>
#include <emmintrin.h>
#endif

namespace Velox {

    /// A 2-component vector with basic arithmetic and geometric operations.
    struct Vec2 {
        Real x, y;

        Vec2() : x(0), y(0) {}
        Vec2(Real _x, Real _y) : x(_x), y(_y) {}

#ifdef VELOX_SIMD
        Vec2 operator+(const Vec2& other) const {
            __m128 a = _mm_set_ps(0.0f, 0.0f, y, x);
            __m128 b = _mm_set_ps(0.0f, 0.0f, other.y, other.x);
            __m128 r = _mm_add_ps(a, b);
            alignas(16) float res[4];
            _mm_store_ps(res, r);
            return Vec2(res[0], res[1]);
        }
        Vec2 operator-(const Vec2& other) const {
            __m128 a = _mm_set_ps(0.0f, 0.0f, y, x);
            __m128 b = _mm_set_ps(0.0f, 0.0f, other.y, other.x);
            __m128 r = _mm_sub_ps(a, b);
            alignas(16) float res[4];
            _mm_store_ps(res, r);
            return Vec2(res[0], res[1]);
        }
        Vec2 operator*(Real scalar) const {
            __m128 a = _mm_set_ps(0.0f, 0.0f, y, x);
            __m128 b = _mm_set1_ps(scalar);
            __m128 r = _mm_mul_ps(a, b);
            alignas(16) float res[4];
            _mm_store_ps(res, r);
            return Vec2(res[0], res[1]);
        }
        Vec2 operator/(Real scalar) const {
            __m128 a = _mm_set_ps(0.0f, 0.0f, y, x);
            __m128 b = _mm_set1_ps(scalar);
            __m128 r = _mm_div_ps(a, b);
            alignas(16) float res[4];
            _mm_store_ps(res, r);
            return Vec2(res[0], res[1]);
        }

        Vec2& operator+=(const Vec2& other) {
            __m128 a = _mm_set_ps(0.0f, 0.0f, y, x);
            __m128 b = _mm_set_ps(0.0f, 0.0f, other.y, other.x);
            __m128 r = _mm_add_ps(a, b);
            alignas(16) float res[4];
            _mm_store_ps(res, r);
            x = res[0]; y = res[1];
            return *this;
        }
        Vec2& operator-=(const Vec2& other) {
            __m128 a = _mm_set_ps(0.0f, 0.0f, y, x);
            __m128 b = _mm_set_ps(0.0f, 0.0f, other.y, other.x);
            __m128 r = _mm_sub_ps(a, b);
            alignas(16) float res[4];
            _mm_store_ps(res, r);
            x = res[0]; y = res[1];
            return *this;
        }
        Vec2& operator*=(Real scalar) {
            __m128 a = _mm_set_ps(0.0f, 0.0f, y, x);
            __m128 b = _mm_set1_ps(scalar);
            __m128 r = _mm_mul_ps(a, b);
            alignas(16) float res[4];
            _mm_store_ps(res, r);
            x = res[0]; y = res[1];
            return *this;
        }

        Real Dot(const Vec2& other) const {
            __m128 a = _mm_set_ps(0.0f, 0.0f, y, x);
            __m128 b = _mm_set_ps(0.0f, 0.0f, other.y, other.x);
            __m128 m = _mm_mul_ps(a, b);
            __m128 shuf = _mm_shuffle_ps(m, m, _MM_SHUFFLE(1, 1, 1, 1));
            __m128 r = _mm_add_ss(m, shuf);
            return _mm_cvtss_f32(r);
        }
        Real MagnitudeSqr() const {
            return Dot(*this);
        }
        Real Magnitude() const {
            return std::sqrt(MagnitudeSqr());
        }
#else
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
#endif

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