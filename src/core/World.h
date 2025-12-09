#pragma once

#include "VelcoxECS.h"
#include "../physics/PhysicsSystem.h"
#include <memory>

namespace Velox {

    class World {
    public:
        World() {
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
            
            m_physicsSystem = std::make_unique<PhysicsSystem>(m_entityManager);
        }

        void Step(Real dt) {
            m_physicsSystem->Step(dt);
        }

        EntityManager& GetEntityManager() { return *m_entityManager; }
        PhysicsSystem& GetPhysicsSystem() { return *m_physicsSystem; }

    private:
        std::shared_ptr<EntityManager> m_entityManager;
        std::unique_ptr<PhysicsSystem> m_physicsSystem;
    };

}
