#include "CCD.h"
#include <cmath>
#include <algorithm>

namespace Velox {

    TOIResult SweptCircleCircle(Vec2 pA, Vec2 vA, Real rA,
                                Vec2 pB, Vec2 vB, Real rB,
                                Real subDt) {
        Vec2 relP = pA - pB;
        Vec2 relV = vA - vB;
        Real rSum = rA + rB;

        Real a = relV.Dot(relV);
        if (a < 0.0001f) {
            Real distSqr = relP.MagnitudeSqr();
            if (distSqr < rSum * rSum) {
                return { true, 0.0f };
            }
            return { false, 0.0f };
        }

        Real b = 2.0f * relP.Dot(relV);
        Real c = relP.MagnitudeSqr() - rSum * rSum;

        if (c < 0.0f) {
            return { true, 0.0f };
        }

        Real disc = b * b - 4.0f * a * c;
        if (disc < 0.0f) {
            return { false, 0.0f };
        }

        Real t = (-b - std::sqrt(disc)) / (2.0f * a);
        if (t >= 0.0f && t <= subDt) {
            return { true, t };
        }

        return { false, 0.0f };
    }

    TOIResult SweptAABB(Vec2 minA, Vec2 maxA, Vec2 vA,
                        Vec2 minB, Vec2 maxB, Vec2 vB,
                        Real subDt) {
        Vec2 relV = vA - vB;
        Real tMin = 0.0f;
        Real tMax = subDt;

        // X axis
        if (relV.x != 0.0f) {
            Real t1 = (minB.x - maxA.x) / relV.x;
            Real t2 = (maxB.x - minA.x) / relV.x;
            tMin = std::max(tMin, std::min(t1, t2));
            tMax = std::min(tMax, std::max(t1, t2));
        } else {
            if (maxA.x < minB.x || maxB.x < minA.x) {
                return { false, 0.0f };
            }
        }

        // Y axis
        if (relV.y != 0.0f) {
            Real t1 = (minB.y - maxA.y) / relV.y;
            Real t2 = (maxB.y - minA.y) / relV.y;
            tMin = std::max(tMin, std::min(t1, t2));
            tMax = std::min(tMax, std::max(t1, t2));
        } else {
            if (maxA.y < minB.y || maxB.y < minA.y) {
                return { false, 0.0f };
            }
        }

        if (tMin <= tMax) {
            return { true, tMin };
        }

        return { false, 0.0f };
    }
}
