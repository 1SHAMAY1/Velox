#pragma once

#include "../core/VelcoxECS.h"
#include <memory>
#include <vector>

namespace Velox {

// ============================================================================
// IPhysicsBehavior: Base Interface for Decentralized Physical Components
// ============================================================================
class IPhysicsBehavior {
public:
    virtual ~IPhysicsBehavior() = default;

    /**
     * @brief Called once per simulation step before velocity integration and sub-stepping.
     */
    virtual void OnPreStep(EntityManager& em, Real dt) { (void)em; (void)dt; }

    /**
     * @brief Called before XPBD integration to apply forces (gravity, buoyancy, fields, motors).
     */
    virtual void OnApplyForces(EntityManager& em, Real dt) { (void)em; (void)dt; }

    /**
     * @brief Called inside each XPBD sub-step to project custom positional constraints.
     */
    virtual void OnSolveConstraints(EntityManager& em, Real dt) { (void)em; (void)dt; }

    /**
     * @brief Called once per simulation step after all sub-steps, derivation, and resolution complete.
     */
    virtual void OnPostStep(EntityManager& em, Real dt) { (void)em; (void)dt; }
};

// ============================================================================
// PhysicsBehaviorRegistry: Centralized Container for Registered Behaviors
// ============================================================================
class VELOX_API PhysicsBehaviorRegistry {
public:
    static PhysicsBehaviorRegistry& Get();

    PhysicsBehaviorRegistry(const PhysicsBehaviorRegistry&) = delete;
    PhysicsBehaviorRegistry& operator=(const PhysicsBehaviorRegistry&) = delete;

    void Register(std::unique_ptr<IPhysicsBehavior> behavior);
    const std::vector<std::unique_ptr<IPhysicsBehavior>>& GetBehaviors() const;

private:
    PhysicsBehaviorRegistry() = default;
    std::vector<std::unique_ptr<IPhysicsBehavior>> m_behaviors;
};

} // namespace Velox

// ============================================================================
// Auto-Registration Macro: Register Any Component/Behavior at Startup
// ============================================================================
#define VELOX_REGISTER_PHYSICS_BEHAVIOR(BehaviorClass) \
    namespace { \
        struct AutoRegister_##BehaviorClass { \
            AutoRegister_##BehaviorClass() { \
                ::Velox::PhysicsBehaviorRegistry::Get().Register( \
                    std::make_unique<BehaviorClass>() \
                ); \
            } \
        } g_autoRegister_##BehaviorClass; \
    }
