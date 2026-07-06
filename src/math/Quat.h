#pragma once

/**
 * @file Quat.h
 * @brief Quaternion type for 3D orientation and rotation (reserved for future 3D support).
 */

#include "Vec3.h"
#include <cmath>

namespace Velox {

    /// Unit quaternion representing a 3D rotation, in (w, x, y, z) order.
    struct Quat {
        Real w, x, y, z;

        Quat() : w(1), x(0), y(0), z(0) {}
        Quat(Real _w, Real _x, Real _y, Real _z) : w(_w), x(_x), y(_y), z(_z) {}

        /// Hamilton product — composes this rotation with `other` (this applied after other).
        Quat operator*(const Quat& other) const {
            return Quat(
                w * other.w - x * other.x - y * other.y - z * other.z,
                w * other.x + x * other.w + y * other.z - z * other.y,
                w * other.y - x * other.z + y * other.w + z * other.x,
                w * other.z + x * other.y - y * other.x + z * other.w
            );
        }

        /// Rotates vector `v` by this quaternion using the optimized q*v*q^-1 expansion.
        Vec3 Rotate(const Vec3& v) const {
            Vec3 u(x, y, z);
            Real s = w;

            return u * 2.0f * u.Dot(v)
                 + v * (s*s - u.Dot(u))
                 + u.Cross(v) * 2.0f * s;
        }

        /// Normalizes this quaternion to unit length in place. No-op if magnitude is zero.
        void Normalize() {
            Real mag = std::sqrt(w*w + x*x + y*y + z*z);
            if (mag > 0) {
                w /= mag; x /= mag; y /= mag; z /= mag;
            }
        }

        /// Returns the identity rotation (no rotation).
        static Quat Identity() { return Quat(1, 0, 0, 0); }
    };

}