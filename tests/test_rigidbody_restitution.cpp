#include <velox/VeloxAPI.h>
#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>
#include <random>

void TestRestitutionDrop() {
    std::cout << "[Test RigidBody: Restitution] Testing Coefficient of Restitution (COR)...\n";
    VeloxWorld* world = Velox_CreateWorld();
    Velox_SetGravity(world, 0.0f, 980.0f);

    // Floor at y = 800
    Velox::EntityID floor = Velox_CreateEntity(world);
    Velox_AddTransform(world, floor, 500.0f, 800.0f, 0.0f);
    Velox_AddRigidBody(world, floor, 0.0f, true);
    Velox_AddBoxCollider(world, floor, 1000.0f, 40.0f);
    Velox_AddPhysicalMaterial(world, floor, 0.0f, 0.0f, 1.0f); // 1.0 Restitution floor

    // Ball 1: Restitution = 0.0 (Inelastic - should stick to floor)
    Velox::EntityID ballInelastic = Velox_CreateEntity(world);
    Velox_AddTransform(world, ballInelastic, 300.0f, 400.0f, 0.0f);
    Velox_AddMovement(world, ballInelastic);
    Velox_AddRigidBody(world, ballInelastic, 1.0f, false);
    Velox_AddCircleCollider(world, ballInelastic, 15.0f);
    Velox_AddPhysicalMaterial(world, ballInelastic, 0.0f, 0.0f, 0.0f);

    // Ball 2: Restitution = 0.8 (Elastic - should bounce significantly)
    Velox::EntityID ballElastic = Velox_CreateEntity(world);
    Velox_AddTransform(world, ballElastic, 700.0f, 400.0f, 0.0f);
    Velox_AddMovement(world, ballElastic);
    Velox_AddRigidBody(world, ballElastic, 1.0f, false);
    Velox_AddCircleCollider(world, ballElastic, 15.0f);
    Velox_AddPhysicalMaterial(world, ballElastic, 0.0f, 0.0f, 0.8f);

    float minY_after_bounce = 1000.0f;
    bool touchedFloor = false;

    // Step 120 frames (~2.0 seconds) and track rebound
    for (int frame = 0; frame < 120; ++frame) {
        Velox_Step(world, 1.0f / 60.0f);
        
        float x2 = 0, y2 = 0, r2 = 0;
        Velox_GetPosition(world, ballElastic, &x2, &y2, &r2);
        if (y2 >= 750.0f) {
            touchedFloor = true;
        }
        if (touchedFloor && y2 < minY_after_bounce) {
            minY_after_bounce = y2;
        }
    }

    float x1 = 0, y1 = 0, r1 = 0;
    Velox_GetPosition(world, ballInelastic, &x1, &y1, &r1);

    float x2 = 0, y2 = 0, r2 = 0;
    Velox_GetPosition(world, ballElastic, &x2, &y2, &r2);

    assert(std::isfinite(y1) && std::isfinite(y2));
    // Inelastic ball should be settled on floor (~765px with radius 15 and half-floor 20 at y=800)
    assert(y1 > 750.0f && y1 <= 786.0f);

    // Elastic ball must have hit floor and rebounded significantly (minY < 620px)
    assert(touchedFloor == true);
    assert(minY_after_bounce < 620.0f);

    Velox_DestroyWorld(world);
    std::cout << "  -> PASSED: Restitution response validated for elastic vs inelastic bodies (Rebound apex: " << minY_after_bounce << " px).\n";
}

void TestKineticEnergyConservation() {
    std::cout << "[Test RigidBody: Energy Conservation] Testing 40 Colliding Bodies in Closed Box (0 Gravity)...\n";
    VeloxWorld* world = Velox_CreateWorld();
    Velox_SetGravity(world, 0.0f, 0.0f); // Pure kinetic gas simulation

    // Create enclosing walls
    struct WallDef { float x, y, w, h; };
    WallDef walls[] = {
        { 500.0f, 100.0f, 800.0f, 20.0f }, // Top
        { 500.0f, 700.0f, 800.0f, 20.0f }, // Bottom
        { 100.0f, 400.0f, 20.0f, 600.0f }, // Left
        { 900.0f, 400.0f, 20.0f, 600.0f }  // Right
    };
    for (const auto& w : walls) {
        auto wall = Velox_CreateEntity(world);
        Velox_AddTransform(world, wall, w.x, w.y, 0.0f);
        Velox_AddRigidBody(world, wall, 0.0f, true);
        Velox_AddBoxCollider(world, wall, w.w, w.h);
        Velox_AddPhysicalMaterial(world, wall, 0.0f, 0.0f, 0.9f);
    }

    // Spawn 40 balls with restitution e = 0.8
    const int count = 40;
    std::vector<Velox::EntityID> balls;
    balls.reserve(count);

    std::mt19937 rng(1337);
    std::uniform_real_distribution<float> distVel(-150.0f, 150.0f);

    for (int i = 0; i < count; ++i) {
        float gx = 220.0f + (i % 8) * 70.0f;
        float gy = 220.0f + (i / 8) * 70.0f;
        auto b = Velox_CreateEntity(world);
        Velox_AddTransform(world, b, gx, gy, 0.0f);
        Velox_AddMovement(world, b);
        Velox_SetVelocity(world, b, distVel(rng), distVel(rng));
        Velox_AddRigidBody(world, b, 1.0f, false);
        Velox_AddCircleCollider(world, b, 10.0f);
        Velox_AddPhysicalMaterial(world, b, 0.0f, 0.0f, 0.8f);
        balls.push_back(b);
    }

    auto ComputeTotalKineticEnergy = [&]() -> double {
        double totalKE = 0.0;
        for (auto b : balls) {
            float vx = 0, vy = 0, av = 0;
            Velox_GetVelocity(world, b, &vx, &vy, &av);
            totalKE += 0.5 * (vx * vx + vy * vy);
        }
        return totalKE;
    };

    double initialKE = ComputeTotalKineticEnergy();
    assert(initialKE > 0.0);

    // Simulate 300 frames (~5 seconds of dense multi-body bouncing)
    for (int frame = 0; frame < 300; ++frame) {
        Velox_Step(world, 1.0f / 60.0f);
        double currentKE = ComputeTotalKineticEnergy();
        // Strict invariant: Energy must NEVER blow up or increase artificially
        assert(currentKE <= initialKE * 1.02); // Tolerance for small floating point numerical variance
    }

    double finalKE = ComputeTotalKineticEnergy();
    // In an e = 0.8 restitution box, final energy must naturally dissipate (final < initial)
    assert(finalKE <= initialKE);
    assert(std::isfinite(finalKE));

    Velox_DestroyWorld(world);
    std::cout << "  -> PASSED: Zero kinetic energy accumulation across 300 multi-collision frames! (Initial KE: " 
              << initialKE << " -> Final KE: " << finalKE << ")\n";
}

int main() {
    TestRestitutionDrop();
    TestKineticEnergyConservation();
    return 0;
}

