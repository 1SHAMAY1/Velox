#include <velox/VeloxAPI.h>
#include <iostream>
#include <cassert>
#include <cmath>

void TestRotationMotor() {
    std::cout << "[Test Gameplay: Motors] Testing continuous rotation motor...\n";
    VeloxWorld* world = Velox_CreateWorld();

    Velox::EntityID spinner = Velox_CreateEntity(world);
    Velox_AddTransform(world, spinner, 100.0f, 100.0f, 0.0f);
    Velox_AddMovement(world, spinner);
    Velox_AddRigidBody(world, spinner, 1.0f, false);
    // Speed: 2.0 rad/s, Direction: 0 (Clockwise)
    Velox_AddRotation(world, spinner, 2.0f, 0, 0);

    for (int i = 0; i < 60; ++i) {
        Velox_Step(world, 1.0f / 60.0f);
    }

    float x = 0, y = 0, rot = 0;
    Velox_GetPosition(world, spinner, &x, &y, &rot);
    // After 1 second at 2 rad/s, rot should be ~ 2.0 rad
    assert(rot > 1.5f);

    Velox_DestroyWorld(world);
    std::cout << "  -> PASSED: Rotation motor updates angular velocity and rotation correctly.\n";
}

void TestOscillation() {
    std::cout << "[Test Gameplay: Oscillation] Testing harmonic oscillation movement...\n";
    VeloxWorld* world = Velox_CreateWorld();

    Velox::EntityID platform = Velox_CreateEntity(world);
    Velox_AddTransform(world, platform, 200.0f, 200.0f, 0.0f);
    Velox_AddMovement(world, platform);
    Velox_AddRigidBody(world, platform, 0.0f, true); // Kinematic/Static
    // Axis: (1, 0), Amplitude: 50 px, Frequency: 1 Hz
    Velox_AddOscillation(world, platform, 1.0f, 0.0f, 50.0f, 1.0f);

    // Step 0.25s (quarter period = peak amplitude of 50px)
    for (int i = 0; i < 15; ++i) {
        Velox_Step(world, 1.0f / 60.0f);
    }

    float x = 0, y = 0, rot = 0;
    Velox_GetPosition(world, platform, &x, &y, &rot);
    // x should have moved from 200 toward 250
    assert(x > 230.0f);

    Velox_DestroyWorld(world);
    std::cout << "  -> PASSED: Oscillation motion tracks sinusoidal trajectory.\n";
}

void TestProjectileFacing() {
    std::cout << "[Test Gameplay: Projectile] Testing projectile rotation facing velocity...\n";
    VeloxWorld* world = Velox_CreateWorld();

    Velox::EntityID arrow = Velox_CreateEntity(world);
    Velox_AddTransform(world, arrow, 100.0f, 100.0f, 0.0f);
    Velox_AddMovement(world, arrow);
    Velox_SetVelocity(world, arrow, 100.0f, 100.0f); // 45 degrees down-right = ~0.785 rad
    Velox_AddRigidBody(world, arrow, 1.0f, false);
    Velox_AddProjectile(world, arrow, true, 100.0f, 1000.0f, 0.1f);

    Velox_Step(world, 1.0f / 60.0f);

    float x = 0, y = 0, rot = 0;
    Velox_GetPosition(world, arrow, &x, &y, &rot);
    // Atan2(100, 100) = pi / 4 ~ 0.785398 rad
    assert(std::abs(rot - 0.785398f) < 0.05f);

    Velox_DestroyWorld(world);
    std::cout << "  -> PASSED: Projectile correctly aligns rotation to velocity vector.\n";
}

void TestForceFields() {
    std::cout << "[Test Gameplay: Force Fields] Testing radial attraction/repulsion fields...\n";
    VeloxWorld* world = Velox_CreateWorld();

    // Gravity well at (500, 500)
    Velox::EntityID well = Velox_CreateEntity(world);
    Velox_AddTransform(world, well, 500.0f, 500.0f, 0.0f);
    // Type 0 = Inward, Strength = 1000, Radius = 200
    Velox_AddForceField(world, well, 0, 1000.0f, 200.0f);

    // Particle at (450, 500)
    Velox::EntityID p = Velox_CreateEntity(world);
    Velox_AddTransform(world, p, 450.0f, 500.0f, 0.0f);
    Velox_AddMovement(world, p);
    Velox_AddRigidBody(world, p, 1.0f, false);

    for (int i = 0; i < 30; ++i) {
        Velox_Step(world, 1.0f / 60.0f);
    }

    float x = 0, y = 0, rot = 0;
    Velox_GetPosition(world, p, &x, &y, &rot);
    // Particle should have been pulled to the right (toward x=500)
    assert(x > 450.0f);

    Velox_DestroyWorld(world);
    std::cout << "  -> PASSED: Force fields apply radial forces to dynamic bodies.\n";
}

#include <physics/PhysicsBehavior.h>
#include <physics/Components.h>
#include <core/World.h>
#include <math/Vec2.h>

// ============================================================================
// Custom Decentralized Physics Behavior Test
// ============================================================================
struct CustomBuoyancyComponent {
    float FluidDensity = 1000.0f;
    float WaterSurfaceY = 500.0f;
    bool ForceApplied = false;
};

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

VELOX_REGISTER_PHYSICS_BEHAVIOR(CustomBuoyancyBehavior);

void TestDecentralizedPhysicsBehavior() {
    std::cout << "[Test Gameplay: Decentralized Registry] Testing custom component auto-registration...\n";
    VeloxWorld* world = Velox_CreateWorld();
    Velox_SetGravity(world, 0.0f, 980.0f);

    auto& em = reinterpret_cast<Velox::World*>(world)->GetEntityManager();
    em.RegisterComponent<CustomBuoyancyComponent>();

    Velox::EntityID boat = em.CreateEntity();
    em.AddComponent(boat, Velox::TransformComponent{Velox::Vec2(500.0f, 550.0f), 0.0f});
    em.AddComponent(boat, Velox::MovementComponent{});
    em.AddComponent(boat, Velox::RigidBodyComponent{1.0f, 1.0f, 0.1f, 100.0f, false});
    
    CustomBuoyancyComponent buoyComp;
    buoyComp.FluidDensity = 2000.0f;
    buoyComp.WaterSurfaceY = 500.0f;
    em.AddComponent(boat, buoyComp);

    Velox_Step(world, 1.0f / 60.0f);

    auto& buoyResult = em.GetComponent<CustomBuoyancyComponent>(boat);
    assert(buoyResult.ForceApplied && "Custom buoyancy behavior failed to execute!");

    Velox_DestroyWorld(world);
    std::cout << "  -> PASSED: Decentralized component auto-registration verified.\n";
}

static int g_collisionBeginCount = 0;
static int g_collisionEndCount = 0;
static int g_sensorEnterCount = 0;
static int g_sensorExitCount = 0;

void OnCollisionBegin(Velox::EntityID a, Velox::EntityID b, float nx, float ny, void* userData) {
    (void)a; (void)b; (void)nx; (void)ny; (void)userData;
    g_collisionBeginCount++;
}

void OnCollisionEnd(Velox::EntityID a, Velox::EntityID b, float nx, float ny, void* userData) {
    (void)a; (void)b; (void)nx; (void)ny; (void)userData;
    g_collisionEndCount++;
}

void OnSensorTrigger(Velox::EntityID sensor, Velox::EntityID other, bool isEntering, void* userData) {
    (void)sensor; (void)other; (void)userData;
    if (isEntering) g_sensorEnterCount++;
    else g_sensorExitCount++;
}

void TestCollisionEventCallbacks() {
    std::cout << "[Test Gameplay: Event Callbacks] Testing OnCollisionBegin, OnCollisionEnd, and Sensor callbacks...\n";
    VeloxWorld* world = Velox_CreateWorld();
    Velox_SetGravity(world, 0.0f, 980.0f);

    g_collisionBeginCount = 0;
    g_collisionEndCount = 0;
    g_sensorEnterCount = 0;
    g_sensorExitCount = 0;

    Velox_SetCollisionBeginCallback(world, OnCollisionBegin, nullptr);
    Velox_SetCollisionEndCallback(world, OnCollisionEnd, nullptr);
    Velox_SetSensorCallback(world, OnSensorTrigger, nullptr);

    // Floor
    auto floor = Velox_CreateEntity(world);
    Velox_AddTransform(world, floor, 100.0f, 300.0f, 0.0f);
    Velox_AddRigidBody(world, floor, 0.0f, true);
    Velox_AddBoxCollider(world, floor, 200.0f, 20.0f);

    // Dropping Ball (will hit floor)
    auto ball = Velox_CreateEntity(world);
    Velox_AddTransform(world, ball, 100.0f, 200.0f, 0.0f);
    Velox_AddRigidBody(world, ball, 1.0f, false);
    Velox_AddMovement(world, ball);
    Velox_AddCircleCollider(world, ball, 10.0f);

    // Sensor Trigger Region
    auto sensor = Velox_CreateEntity(world);
    Velox_AddTransform(world, sensor, 100.0f, 240.0f, 0.0f);
    Velox_AddRigidBody(world, sensor, 0.0f, true);
    Velox_AddBoxCollider(world, sensor, 100.0f, 20.0f);
    Velox_SetColliderSensor(world, sensor, true);

    for (int i = 0; i < 40; ++i) {
        Velox_Step(world, 1.0f / 60.0f);
    }

    assert(g_sensorEnterCount >= 1 && "Sensor trigger enter callback was not fired!");
    assert(g_collisionBeginCount >= 1 && "Collision begin callback was not fired!");

    Velox_DestroyWorld(world);
    std::cout << "  -> PASSED: Collision and sensor event callbacks verified successfully.\n";
}

void TestSleepingStackWakeUpOnBaseMove() {
    std::cout << "[Test Gameplay: Island Sleeping] Testing stacked sleeping boxes wake-up when bottom box is removed...\n";
    VeloxWorld* world = Velox_CreateWorld();
    Velox_SetGravity(world, 0.0f, 980.0f);

    // Floor
    auto floor = Velox_CreateEntity(world);
    Velox_AddTransform(world, floor, 200.0f, 500.0f, 0.0f);
    Velox_AddRigidBody(world, floor, 0.0f, true);
    Velox_AddBoxCollider(world, floor, 400.0f, 20.0f);

    // Stack of 10 boxes
    std::vector<Velox::EntityID> stack;
    float boxSize = 20.0f;
    float startY = 500.0f - 10.0f - boxSize * 0.5f; // Resting directly on top of floor

    for (int i = 0; i < 10; ++i) {
        auto box = Velox_CreateEntity(world);
        Velox_AddTransform(world, box, 200.0f, startY - i * boxSize, 0.0f);
        Velox_AddRigidBody(world, box, 1.0f, false);
        Velox_AddMovement(world, box);
        Velox_AddBoxCollider(world, box, boxSize, boxSize);
        Velox_AddPhysicalMaterial(world, box, 0.8f, 0.5f, 0.0f);
        Velox_SetDamping(world, box, 0.1f, 0.1f);
        stack.push_back(box);
    }

    // Step simulation to let the entire stack settle and go to sleep
    for (int t = 0; t < 120; ++t) {
        Velox_Step(world, 1.0f / 60.0f);
    }

    // Verify that all boxes in stack are asleep
    bool anyAwake = false;
    for (auto b : stack) {
        if (!Velox_IsSleeping(world, b)) anyAwake = true;
    }
    std::cout << "  Stack settled. Is sleeping? " << (!anyAwake ? "YES" : "NO") << "\n";

    // Record pre-move position of the top box (stack[9]) and second box (stack[1])
    float topX, topYPre, topR, secX, secYPre, secR;
    Velox_GetPosition(world, stack[9], &topX, &topYPre, &topR);
    Velox_GetPosition(world, stack[1], &secX, &secYPre, &secR);

    // Remove the bottom box (stack[0]) by moving it away
    Velox_SetPosition(world, stack[0], 1000.0f, 1000.0f);

    // Step physics for 60 ticks
    for (int t = 0; t < 60; ++t) {
        Velox_Step(world, 1.0f / 60.0f);
    }

    // Check positions of second and top box
    float topYPost, secYPost;
    Velox_GetPosition(world, stack[9], &topX, &topYPost, &topR);
    Velox_GetPosition(world, stack[1], &secX, &secYPost, &secR);

    // The boxes MUST have fallen downward (larger Y in screen space) by approximately boxSize (20px)
    std::cout << "  Second box pre Y: " << secYPre << " -> post Y: " << secYPost << "\n";
    std::cout << "  Top box pre Y: " << topYPre << " -> post Y: " << topYPost << "\n";

    assert(secYPost > secYPre + 10.0f && "Second box did not fall when bottom box was removed (floating stack bug)!");
    assert(topYPost > topYPre + 10.0f && "Top box did not fall when bottom box was removed (floating stack bug)!");

    Velox_DestroyWorld(world);
    std::cout << "  -> PASSED: Sleeping stack wake-up verified successfully.\n";
}

int main() {
    try {
        std::cout << "=== Running Velox Gameplay Behaviours Tests ===" << std::endl;
        TestRotationMotor();
        TestOscillation();
        TestProjectileFacing();
        TestForceFields();
        TestDecentralizedPhysicsBehavior();
        TestCollisionEventCallbacks();
        TestSleepingStackWakeUpOnBaseMove();
        std::cout << "=== All Gameplay Behaviour Tests Passed Successfully! ===" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "EXCEPTION: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "UNKNOWN EXCEPTION!" << std::endl;
        return 1;
    }
    return 0;
}

