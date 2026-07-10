#pragma once

#include "../math/Vec2.h"
#include <velox/Types.h>

namespace Velox {
    struct TOIResult {
        bool  hit;
        Real  toi;      ///< Time of impact in [0, subDt]
    };

    /// Swept circle-circle TOI solve. Returns hit=false if no collision in [0, subDt].
    TOIResult SweptCircleCircle(Vec2 pA, Vec2 vA, Real rA,
                                Vec2 pB, Vec2 vB, Real rB,
                                Real subDt);

    /// Swept AABB overlap interval — used as a fast CCD broadphase filter.
    TOIResult SweptAABB(Vec2 minA, Vec2 maxA, Vec2 vA,
                        Vec2 minB, Vec2 maxB, Vec2 vB,
                        Real subDt);
}
