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

    TOIResult SweptCircleBox(Vec2 circleP, Vec2 circleV, Real radius,
                             Vec2 boxP, Vec2 boxV, Vec2 boxHalfExtents, Real boxRotation,
                             Real subDt, Vec2& outNormal) {
        // Transform relative trajectory to Box Local Space
        Vec2 relV = circleV - boxV;
        Vec2 relP = circleP - boxP;

        Vec2 localP = relP.Rotate(-boxRotation);
        Vec2 localV = relV.Rotate(-boxRotation);

        // Expanded box by radius (Minkowski sum with circle = rounded box)
        Vec2 expHalf = boxHalfExtents + Vec2(radius, radius);

        // Ray vs Expanded AABB in local space
        Real tMin = 0.0f;
        Real tMax = subDt;
        Vec2 normal = {0.0f, 0.0f};

        // X slab
        if (std::abs(localV.x) > 1e-6f) {
            Real t1 = (-expHalf.x - localP.x) / localV.x;
            Real t2 = (expHalf.x - localP.x) / localV.x;
            Vec2 n1 = {-1.0f, 0.0f};
            Vec2 n2 = {1.0f, 0.0f};
            if (t1 > t2) { std::swap(t1, t2); std::swap(n1, n2); }
            if (t1 > tMin) { tMin = t1; normal = n1; }
            tMax = std::min(tMax, t2);
            if (tMin > tMax) return { false, 0.0f };
        } else {
            if (std::abs(localP.x) > expHalf.x) return { false, 0.0f };
        }

        // Y slab
        if (std::abs(localV.y) > 1e-6f) {
            Real t1 = (-expHalf.y - localP.y) / localV.y;
            Real t2 = (expHalf.y - localP.y) / localV.y;
            Vec2 n1 = {0.0f, -1.0f};
            Vec2 n2 = {0.0f, 1.0f};
            if (t1 > t2) { std::swap(t1, t2); std::swap(n1, n2); }
            if (t1 > tMin) { tMin = t1; normal = n1; }
            tMax = std::min(tMax, t2);
            if (tMin > tMax) return { false, 0.0f };
        } else {
            if (std::abs(localP.y) > expHalf.y) return { false, 0.0f };
        }

        if (tMin >= 0.0f && tMin <= subDt) {
            outNormal = normal.Rotate(boxRotation);
            return { true, tMin };
        }

        return { false, 0.0f };
    }
}

