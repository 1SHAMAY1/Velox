#pragma once

/**
 * @file World.h
 * @brief Top-level container tying the ECS and physics pipeline together.
 *
 * World is the entry point most host applications interact with: it owns the
 * EntityManager and PhysicsSystem, registers all built-in component types,
 * and exposes a single Step() call to advance the simulation.
 */

#include "VelcoxECS.h"
#include "../physics/PhysicsSystem.h"
#include "../math/Vec2.h"
#include <memory>

namespace Velox {

    /// Owns an entity manager + physics system pair representing one simulated scene.
    class World {
    public:
        /// Constructs an empty world with zero gravity and all built-in components registered.
        World() : m_gravity(0.0f, 0.0f) {
            m_entityManager = std::make_shared<EntityManager>();

            // Register Components
            m_entityManager->RegisterComponent<TransformComponent>();
            m_entityManager->RegisterComponent<RigidBodyComponent>();
            m_entityManager->RegisterComponent<MovementComponent>();
            m_entityManager->RegisterComponent<ColliderComponent>();
            m_entityManager->RegisterComponent<ForceFieldComponent>();
            m_entityManager->RegisterComponent<RotationComponent>();
            m_entityManager->RegisterComponent<OscillationComponent>();
            m_entityManager->RegisterComponent<ProjectileComponent>();
            m_entityManager->RegisterComponent<PhysicalMaterialComponent>();
            m_entityManager->RegisterComponent<JointComponent>();
            m_entityManager->RegisterComponent<RevoluteJointComponent>();
            m_entityManager->RegisterComponent<PrismaticJointComponent>();
            m_entityManager->RegisterComponent<GearJointComponent>();
            m_entityManager->RegisterComponent<PulleyJointComponent>();
            m_entityManager->RegisterComponent<SoftBodyComponent>();
            
            m_physicsSystem = std::make_unique<PhysicsSystem>(m_entityManager);
        }

        /// Advances the simulation by `dt` seconds, applying the current gravity setting.
        void Step(Real dt) {
            m_physicsSystem->SetGravity(m_gravity);
            m_physicsSystem->Step(dt);
        }

        // Set the global gravity direction and magnitude as a free vector.
        // e.g. SetGravity(0, 980) for downward gravity, SetGravity(980, 0) for sideways.
        void SetGravity(Real x, Real y) { m_gravity = Vec2(x, y); }
        Vec2 GetGravity() const { return m_gravity; }

        /// Access to the world's entity/component storage.
        EntityManager& GetEntityManager() { return *m_entityManager; }

        /// Access to the world's physics pipeline (e.g. for raycasts).
        PhysicsSystem& GetPhysicsSystem() { return *m_physicsSystem; }

    private:
        std::shared_ptr<EntityManager> m_entityManager;
        std::unique_ptr<PhysicsSystem> m_physicsSystem;
        Vec2 m_gravity;
    };

}