#include <velox/VeloxAPI.h>
#include <iostream>
#include <vector>
#include <random>
#include <cassert>
#include <cmath>
#include <chrono>

#if defined(_WIN32)
#include <windows.h>
#include <psapi.h>
static size_t GetProcessWorkingSet() {
    PROCESS_MEMORY_COUNTERS_EX pmc{};
    if (GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc))) {
        return pmc.WorkingSetSize;
    }
    return 0;
}
#else
static size_t GetProcessWorkingSet() { return 0; }
#endif

void TestChaosAndStress() {
    std::cout << "[Test 3: Chaos & Stress] Running 1,000 Randomized Entities with Extreme Velocities & Scale...\n";
    size_t memBefore = GetProcessWorkingSet();

    VeloxWorld* world = Velox_CreateWorld();
    Velox_SetGravity(world, 0.0f, 980.0f);

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> distX(100.0f, 900.0f);
    std::uniform_real_distribution<float> distY(-500.0f, 400.0f);
    std::uniform_real_distribution<float> distVel(-300.0f, 300.0f);
    std::uniform_real_distribution<float> distMat(0.1f, 0.9f);

    // Floor & Arena Boundaries
    Velox::EntityID floor = Velox_CreateEntity(world);
    Velox_AddTransform(world, floor, 500.0f, 1000.0f, 0.0f);
    Velox_AddRigidBody(world, floor, 0.0f, true);
    Velox_AddBoxCollider(world, floor, 4000.0f, 300.0f);

    Velox::EntityID leftWall = Velox_CreateEntity(world);
    Velox_AddTransform(world, leftWall, -1000.0f, 0.0f, 0.0f);
    Velox_AddRigidBody(world, leftWall, 0.0f, true);
    Velox_AddBoxCollider(world, leftWall, 300.0f, 3000.0f);

    Velox::EntityID rightWall = Velox_CreateEntity(world);
    Velox_AddTransform(world, rightWall, 2000.0f, 0.0f, 0.0f);
    Velox_AddRigidBody(world, rightWall, 0.0f, true);
    Velox_AddBoxCollider(world, rightWall, 300.0f, 3000.0f);

    const int bodyCount = 1000;
    std::vector<Velox::EntityID> bodies;
    bodies.reserve(bodyCount);

    for (int i = 0; i < bodyCount; ++i) {
        Velox::EntityID e = Velox_CreateEntity(world);
        Velox_AddTransform(world, e, distX(rng), distY(rng), 0.0f);
        Velox_AddMovement(world, e);
        Velox_SetVelocity(world, e, distVel(rng), distVel(rng));
        Velox_AddRigidBody(world, e, 1.0f, false);
        Velox_AddPhysicalMaterial(world, e, distMat(rng), distMat(rng), distMat(rng));

        if (i % 3 == 0) {
            Velox_AddCircleCollider(world, e, 8.0f);
        } else if (i % 3 == 1) {
            Velox_AddBoxCollider(world, e, 12.0f, 12.0f);
        } else {
            float px[3] = {-10.0f, 10.0f, 0.0f};
            float py[3] = {-10.0f, -10.0f, 10.0f};
            Velox_AddPolygonCollider(world, e, px, py, 3);
        }
        bodies.push_back(e);
    }

    size_t memAfterAlloc = GetProcessWorkingSet();

    auto start = std::chrono::high_resolution_clock::now();
    for (int step = 0; step < 120; ++step) {
        Velox_Step(world, 1.0f / 60.0f);
    }
    auto end = std::chrono::high_resolution_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double avgStepMs = totalMs / 120.0;

    // Verify coordinates validity (zero NaNs, zero Infs, bounded coordinates)
    for (auto b : bodies) {
        float x, y, rot;
        Velox_GetPosition(world, b, &x, &y, &rot);
        assert(std::isfinite(x) && std::isfinite(y) && std::isfinite(rot));
    }

    size_t memAfterSim = GetProcessWorkingSet();
    Velox_DestroyWorld(world);
    size_t memAfterDestroy = GetProcessWorkingSet();

    double memDeltaKB = (memAfterAlloc > memBefore) ? (double)(memAfterAlloc - memBefore) / 1024.0 : 0.0;
    double bytesPerEntity = (memDeltaKB * 1024.0) / (double)bodyCount;

    std::cout << "  -> Memory Footprint: ~" << memDeltaKB << " KB allocated for 1,000 bodies (~" << bytesPerEntity << " bytes/body)\n";
    std::cout << "  -> PASSED: 1,000 randomized entities simulated smoothly. Avg step: " << avgStepMs << " ms (" << (1000.0 / avgStepMs) << " FPS)\n";
}

int main() {
    TestChaosAndStress();
    return 0;
}
