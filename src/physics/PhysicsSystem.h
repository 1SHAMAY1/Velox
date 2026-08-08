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
#include "../core/Containers.h"
#include "../math/Vec2.h"
#include "Components.h"
#include "DynamicTree.h"
#include "FlatSpatialHash.h"
#include "IslandManager.h"
#include "PhysicsBehavior.h"

namespace Velox {

    /**
     * @brief Stores the result of a narrowphase collision detection test.
     *
     * Produced by the narrowphase solvers and consumed by ResolveVelocities
     * to apply impulse-based corrections after positional resolution.
     * Uses zero-allocation fixed stack storage for contact manifolds.
     */
    struct ContactInfo {
        EntityID idA = 0;
        EntityID idB = 0;
        Vec2 normal;       ///< Collision normal, pointing from B toward A.
        Real penetration = 0.0f;  ///< Signed penetration depth along the normal.
        std::array<Vec2, 4> contactPoints{}; ///< World-space contact manifold points (fixed stack).
        uint8_t contactCount = 0;

        ContactInfo() = default;
        ContactInfo(EntityID a, EntityID b, Vec2 n, Real pen, const Vec2& pt)
            : idA(a), idB(b), normal(n), penetration(pen), contactCount(1) {
            contactPoints[0] = pt;
        }
        ContactInfo(EntityID a, EntityID b, Vec2 n, Real pen, const std::array<Vec2, 4>& pts, uint8_t count)
            : idA(a), idB(b), normal(n), penetration(pen), contactPoints(pts), contactCount(count) {}
        ContactInfo(EntityID a, EntityID b, Vec2 n, Real pen, std::initializer_list<Vec2> pts)
            : idA(a), idB(b), normal(n), penetration(pen), contactCount(0) {
            for (const auto& pt : pts) {
                if (contactCount < 4) {
                    contactPoints[contactCount++] = pt;
                }
            }
        }
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
    class VELOX_API PhysicsSystem {
    public:
        explicit PhysicsSystem(std::shared_ptr<EntityManager> entityManager);

        /// Advance the simulation by dt seconds (sub-stepped internally).
        void Step(Real dt);

        /// Set the global gravity vector. Any direction and magnitude are valid.
        /// Default is (0, 0) — zero gravity.
        void SetGravity(Vec2 gravity) { m_gravity = gravity; }

        /// Collision Callback Signatures
        using CollisionCallback = void(*)(EntityID entityA, EntityID entityB, Real normalX, Real normalY, void* userData);
        using SensorCallback = void(*)(EntityID sensorEntity, EntityID otherEntity, bool isEntering, void* userData);

        void SetCollisionBeginCallback(CollisionCallback cb, void* userData = nullptr) { m_collisionBeginCb = cb; m_collisionBeginUserData = userData; }
        void SetCollisionEndCallback(CollisionCallback cb, void* userData = nullptr) { m_collisionEndCb = cb; m_collisionEndUserData = userData; }
        void SetSensorCallback(SensorCallback cb, void* userData = nullptr) { m_sensorCb = cb; m_sensorUserData = userData; }

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

        /// Wakes a specific rigid body from sleep.
        void WakeBody(EntityID id);

        /// Wakes a rigid body and all adjacent bodies currently in contact with it.
        void WakeTouching(EntityID id);

    private:
        void ApplyRules(Real dt);       ///< Apply force fields, oscillators, and rotation motors.
        void UpdateBroadphase(Real dt); ///< Build Broadphase & query candidate pairs once per frame.
        void Integrate(Real dt);         ///< Predict positions via semi-implicit Euler integration.
        void SolveConstraints(Real dt);  ///< Broadphase + narrowphase collision and joint solving.
        void SolveRevoluteJoints(Real dt); ///< Solve revolute hinge constraints.
        void SolvePrismaticJoints(Real dt); ///< Solve prismatic slider constraints.
        void SolveGearJoints(Real dt);   ///< Solve gear coupling constraints.
        void SolvePulleyJoints(Real dt); ///< Solve pulley rope constraints.
        void SolveSoftBodies(Real dt);   ///< Solve soft body constraints.
        void SolveSoftBodyArea(EntityID id, SoftBodyComponent& softBody, Real dt);
        void SolveSoftBodyShapeMatch(EntityID id, SoftBodyComponent& softBody, Real dt);
        void ProjectSoftBodyAgainstStaticObstacles(EntityID id, SoftBodyComponent& softBody);
        void SolveSoftBodyEdgeCollisions(Real dt);
        void DeriveVelocities(Real dt);  ///< Derive corrected velocities from position deltas.
        void ResolveVelocities(Real dt); ///< Apply impulse-based restitution and friction.

        std::shared_ptr<EntityManager> m_entityManager;
        Vec2 m_gravity = Vec2(0.0f, 0.0f); ///< Global directional gravity vector (world units/s²).
        PodVector<ContactInfo> m_contacts; ///< Contact manifold accumulated per sub-step.
        
        struct CandidatePair {
            EntityID first;
            EntityID second;
        };
        PodVector<CandidatePair> m_candidatePairs; ///< Flat cache for unique broadphase pairs.

        FlatSpatialHash m_spatialHash; ///< Cache-aligned Zero-Allocation Flat Spatial Hash Grid
        DynamicTree m_dynamicTree;     ///< Dynamic AABB Tree BVH for raycasts
        Bitset m_activeBitset;         ///< Fast 1-bit tag mask for dynamic active entities
        IslandManager m_islandManager; ///< Fast disjoint set island sleeping graph

        // Event callbacks & persistent contact state
        CollisionCallback m_collisionBeginCb = nullptr;
        void* m_collisionBeginUserData = nullptr;
        CollisionCallback m_collisionEndCb = nullptr;
        void* m_collisionEndUserData = nullptr;
        SensorCallback m_sensorCb = nullptr;
        void* m_sensorUserData = nullptr;

        struct ContactPairKey {
            EntityID idA;
            EntityID idB;
            bool operator==(const ContactPairKey& o) const { return idA == o.idA && idB == o.idB; }
        };
        struct ContactPairKeyHash {
            size_t operator()(const ContactPairKey& k) const {
                return (size_t)k.idA ^ ((size_t)k.idB << 16);
            }
        };
        std::unordered_map<ContactPairKey, Vec2, ContactPairKeyHash> m_persistentContacts;
        std::unordered_map<ContactPairKey, bool, ContactPairKeyHash> m_persistentSensors;
    };
}