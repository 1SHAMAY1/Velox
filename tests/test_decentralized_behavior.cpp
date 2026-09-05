#include <velox/VeloxAPI.h>
#include <physics/PhysicsBehavior.h>
#include <physics/Components.h>
#include <core/World.h>
#include <math/Vec2.h>
#include <iostream>
#include <cassert>
#include <cmath>

// ============================================================================
// 1. User-Defined Custom Component (Decentralized)
// ============================================================================
struct CustomBuoyancyComponent {
    float FluidDensity = 1000.0f;
    float WaterSurfaceY = 500.0f;
    bool ForceApplied = false;
};

// ============================================================================
// 2. User-Defined Custom Physics Behavior implementing IPhysicsBehavior
// ============================================================================
class CustomBuoyancyBehavior : public Velox::IPhysicsBehavior {
public:
    void OnApplyForces(Velox::EntityManager& em, Velox::Real dt) override {
        (void)dt;
        const auto& entities = em.GetEntitiesWithComponent<CustomBuoyancyComponent>();
        for (auto id : entities) {
            if (!em.HasComponent<Velox::TransformComponent>(id) ||
                !em.HasComponent<Velox::MovementComponent>(id)) continue;

            auto& trans = em.GetComponent<Velox::TransformComponent>(id);
            auto& move  = em.GetComponent<Velox::MovementComponent>(id);
            auto& buoy  = em.GetComponent<CustomBuoyancyComponent>(id);

            // If submerged below water surface, apply upward buoyant force
            if (trans.Position.y > buoy.WaterSurfaceY) {
                float depth = trans.Position.y - buoy.WaterSurfaceY;
                float upwardForce = depth * buoy.FluidDensity * 0.1f;
                move.Force.y -= upwardForce;
                buoy.ForceApplied = true;
            }
        }
    }
};

// ============================================================================
// 3. Auto-Register Behavior via Macro (0 Core Engine Modifications Required)
// ============================================================================
VELOX_REGISTER_PHYSICS_BEHAVIOR(CustomBuoyancyBehavior);

void TestDecentralizedPhysicsBehavior() {
    std::cout << "[Test Decentralized: Physics Behavior Registry] Testing custom component auto-registration...\n" << std::flush;
    
    // Create Velox World
    std::cout << "Creating world...\n" << std::flush;
    VeloxWorld* world = Velox_CreateWorld();
    Velox_SetGravity(world, 0.0f, 980.0f); // Gravity pointing down

    std::cout << "Getting EntityManager...\n" << std::flush;
    auto& em = reinterpret_cast<Velox::World*>(world)->GetEntityManager();
    std::cout << "Registering CustomBuoyancyComponent...\n" << std::flush;
    em.RegisterComponent<CustomBuoyancyComponent>();

    // Create a buoyant object placed underwater (y = 550, water surface at y = 500)
    std::cout << "Creating entity...\n" << std::flush;
    Velox::EntityID boat = em.CreateEntity();
    em.AddComponent(boat, Velox::TransformComponent{Velox::Vec2(500.0f, 550.0f), 0.0f});
    em.AddComponent(boat, Velox::MovementComponent{});
    em.AddComponent(boat, Velox::RigidBodyComponent{1.0f, 1.0f, 0.1f, 100.0f, false});
    
    // Attach our custom decentralized component
    CustomBuoyancyComponent buoyComp;
    buoyComp.FluidDensity = 2000.0f;
    buoyComp.WaterSurfaceY = 500.0f;
    em.AddComponent(boat, buoyComp);

    // Step physics world
    std::cout << "Stepping world...\n" << std::flush;
    Velox_Step(world, 1.0f / 60.0f);

    // Verify custom behavior executed automatically
    std::cout << "Verifying results...\n" << std::flush;
    auto& buoyResult = em.GetComponent<CustomBuoyancyComponent>(boat);
    if (!buoyResult.ForceApplied) {
        std::cerr << "FAIL: buoyResult.ForceApplied is FALSE!\n" << std::flush;
        exit(1);
    }

    auto& moveResult = em.GetComponent<Velox::MovementComponent>(boat);
    auto& transResult = em.GetComponent<Velox::TransformComponent>(boat);

    std::cout << "  -> Boat Position: (" << transResult.Position.x << ", " << transResult.Position.y << ")\n";
    std::cout << "  -> Boat Velocity: (" << moveResult.Velocity.x << ", " << moveResult.Velocity.y << ")\n";

    Velox_DestroyWorld(world);
    std::cout << "  -> PASSED: Decentralized component registration and execution verified!\n" << std::flush;
}

int main() {
    try {
        TestDecentralizedPhysicsBehavior();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n" << std::flush;
        return 1;
    } catch (...) {
        std::cerr << "Unknown exception!\n" << std::flush;
        return 1;
    }
    return 0;
}

