#pragma once

#include "Vec3.h"
#include <cmath>

namespace Velox {

    struct Quat {
        Real w, x, y, z;

        Quat() : w(1), x(0), y(0), z(0) {}
        Quat(Real _w, Real _x, Real _y, Real _z) : w(_w), x(_x), y(_y), z(_z) {}

        // Basic multiplication
        Quat operator*(const Quat& other) const {
            return Quat(
                w * other.w - x * other.x - y * other.y - z * other.z,
                w * other.x + x * other.w + y * other.z - z * other.y,
                w * other.y - x * other.z + y * other.w + z * other.x,
                w * other.z + x * other.y - y * other.x + z * other.w
            );
        }

        // Rotate vector
        Vec3 Rotate(const Vec3& v) const {
            // q * v * q_inverse
            // Optimized implementation
            Vec3 u(x, y, z);
            Real s = w;
            
            return u * 2.0f * u.Dot(v)
                 + v * (s*s - u.Dot(u))
                 + u.Cross(v) * 2.0f * s;
        }

        void Normalize() {
            Real mag = std::sqrt(w*w + x*x + y*y + z*z);
            if (mag > 0) {
                w /= mag; x /= mag; y /= mag; z /= mag;
            }
        }
        
        static Quat Identity() { return Quat(1, 0, 0, 0); }
    };

}
