#pragma once

#include "../core/ECS.h"
#include "Components.h"

namespace Velox {

    class PhysicsSystem {
    public:
        PhysicsSystem(std::shared_ptr<EntityManager> entityManager) 
            : m_entityManager(entityManager) {}

        void Step(Real dt);

    private:
        void ApplyRules();
        void Integrate(Real dt);
        void SolveConstraints(Real dt);
        void Broadphase();
        void Narrowphase();

        std::shared_ptr<EntityManager> m_entityManager;
        Vec3 m_gravity = {0, -9.81f, 0};
    };

}
