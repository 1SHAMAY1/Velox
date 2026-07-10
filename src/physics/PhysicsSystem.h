#pragma once

/**
 * @file PhysicsSystem.h
 * @brief Core physics pipeline for the Velox engine.
 *
 * Implements a sub-stepped XPBD (Extended Position Based Dynamics) solver with:
 *  - Spatial hash grid broadphase
 *  - Narrowphase dispatch for Circle, Box, Polygon, and Chain colliders
 *  - Impulse-based velocity resolution with friction and restitution
 *  - Distance joint constraints with motor support
 *  - Raycast queries
 */

#include "../core/VelcoxECS.h"
#include "../math/Vec2.h"
#include "Components.h"

namespace Velox {

    /**
     * @brief Stores the result of a narrowphase collision detection test.
     *
     * Produced by the narrowphase solvers and consumed by ResolveVelocities
     * to apply impulse-based corrections after positional resolution.
     */
    struct ContactInfo {
        EntityID idA;
        EntityID idB;
        Vec2 normal;       ///< Collision normal, pointing from B toward A.
        Real penetration;  ///< Signed penetration depth along the normal.
        std::vector<Vec2> contactPoints; ///< World-space contact manifold points.
    };

    /**
     * @brief Stateless physics simulation system.
     *
     * Processes all entities with physics components each frame via Step().
     * The pipeline runs multiple sub-steps per frame for numerical stability.
     *
     * Pipeline per sub-step:
     *   1. Integrate      — semi-implicit Euler velocity and position prediction.
     *   2. SolveConstraints — broadphase + narrowphase collision and joint resolution.
     *   3. DeriveVelocities — recompute velocities from position deltas (XPBD).
     *   4. ResolveVelocities — impulse-based bounce, friction, and restitution.
     */
    class PhysicsSystem {
    public:
        explicit PhysicsSystem(std::shared_ptr<EntityManager> entityManager)
            : m_entityManager(entityManager) {}

        /// Advance the simulation by dt seconds (sub-stepped internally).
        void Step(Real dt);

        /// Set the global gravity vector. Any direction and magnitude are valid.
        /// Default is (0, 0) — zero gravity.
        void SetGravity(Vec2 gravity) { m_gravity = gravity; }

        /**
         * @brief Cast a ray into the scene and return the first hit.
         * @param start         Ray origin in world space.
         * @param direction     Normalized ray direction.
         * @param maxDistance   Maximum travel distance.
         * @param hitPoint      [out] World-space intersection point.
         * @param hitNormal     [out] Surface normal at the hit point.
         * @param fraction      [out] Parametric hit distance in [0, maxDistance].
         * @param hitEntity     [out] EntityID of the intersected body.
         * @return true if any entity was hit.
         */
        bool Raycast(const Vec2& start, const Vec2& direction, Real maxDistance,
                     Vec2& hitPoint, Vec2& hitNormal, Real& fraction, EntityID& hitEntity);

    private:
        void ApplyRules(Real dt);       ///< Apply force fields, oscillators, and rotation motors.
        void Integrate(Real dt);         ///< Predict positions via semi-implicit Euler integration.
        void SolveConstraints(Real dt);  ///< Broadphase + narrowphase collision and joint solving.
        void SolveRevoluteJoints(Real dt); ///< Solve revolute hinge constraints.
        void SolvePrismaticJoints(Real dt); ///< Solve prismatic slider constraints.
        void SolveGearJoints(Real dt);   ///< Solve gear coupling constraints.
        void SolvePulleyJoints(Real dt); ///< Solve pulley rope constraints.
        void SolveSoftBodies(Real dt);   ///< Solve soft body constraints.
        void SolveSoftBodyArea(EntityID id, SoftBodyComponent& softBody, Real dt);
        void SolveSoftBodyShapeMatch(EntityID id, SoftBodyComponent& softBody, Real dt);
        void DeriveVelocities(Real dt);  ///< Derive corrected velocities from position deltas.
        void ResolveVelocities(Real dt); ///< Apply impulse-based restitution and friction.

        std::shared_ptr<EntityManager> m_entityManager;
        Vec2 m_gravity = Vec2(0.0f, 0.0f); ///< Global directional gravity vector (world units/s²).
        std::vector<ContactInfo> m_contacts; ///< Contact manifold accumulated per sub-step.
        std::vector<std::pair<EntityID, EntityID>> m_candidatePairs; ///< Cache for unique broadphase pairs.

        /**
         * @brief Flat 2D grid for broadphase collision culling (cache-friendly, zero-allocation).
         */
        struct FlatGrid {
            static constexpr int CELL_SIZE   = 60;
            static constexpr int GRID_W      = 1280 / CELL_SIZE + 2; // 23 cells wide
            static constexpr int GRID_H      = 720  / CELL_SIZE + 2; // 14 cells tall
            static constexpr int MAX_PER_CELL = 64;

            std::array<std::array<EntityID, MAX_PER_CELL>, GRID_W * GRID_H> cells;
            std::array<int, GRID_W * GRID_H> counts;

            void Clear() {
                counts.fill(0);
            }

            void Insert(EntityID id, const Vec2& min, const Vec2& max) {
                int startX = std::max(0, (int)std::floor(min.x / CELL_SIZE));
                int endX   = std::min(GRID_W - 1, (int)std::floor(max.x / CELL_SIZE));
                int startY = std::max(0, (int)std::floor(min.y / CELL_SIZE));
                int endY   = std::min(GRID_H - 1, (int)std::floor(max.y / CELL_SIZE));

                for (int x = startX; x <= endX; ++x) {
                    for (int y = startY; y <= endY; ++y) {
                        int idx = y * GRID_W + x;
                        if (counts[idx] < MAX_PER_CELL) {
                            cells[idx][counts[idx]++] = id;
                        }
                    }
                }
            }
        } m_grid;
    };
}