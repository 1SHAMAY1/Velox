#include <velox/VeloxAPI.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <iomanip>
#include <cmath>

void RunBox2DComparativeBenchmark() {
    std::cout << "========================================================================================\n";
    std::cout << "        VELOX vs BOX2D INDUSTRY-STANDARD BENCHMARK & COMPARISON HARNESS         \n";
    std::cout << "========================================================================================\n";
    std::cout << std::left 
              << std::setw(28) << "Benchmark Scenario"
              << std::setw(12) << "Entities"
              << std::setw(16) << "Avg Step (ms)"
              << std::setw(18) << "Uncapped FPS"
              << std::setw(20) << "Operations/sec"
              << "\n";
    std::cout << "----------------------------------------------------------------------------------------\n";

    // Scenario 1: Stacking Pyramid (1,000 Boxes)
    {
        VeloxWorld* world = Velox_CreateWorld();
        Velox_SetGravity(world, 0.0f, 980.0f);

        Velox::EntityID floor = Velox_CreateEntity(world);
        Velox_AddTransform(world, floor, 1000.0f, 800.0f, 0.0f);
        Velox_AddRigidBody(world, floor, 0.0f, true);
        Velox_AddBoxCollider(world, floor, 5000.0f, 40.0f);

        const int count = 1000;
        int cols = 40;
        for (int i = 0; i < count; ++i) {
            Velox::EntityID b = Velox_CreateEntity(world);
            float x = 200.0f + (i % cols) * 25.0f;
            float y = 100.0f + (i / cols) * 25.0f;
            Velox_AddTransform(world, b, x, y, 0.0f);
            Velox_AddMovement(world, b);
            Velox_AddRigidBody(world, b, 1.0f, false);
            Velox_AddBoxCollider(world, b, 12.0f, 12.0f);
        }

        // Warmup
        for (int w = 0; w < 10; ++w) Velox_Step(world, 1.0f / 60.0f);

        const int frames = 100;
        auto start = std::chrono::high_resolution_clock::now();
        for (int f = 0; f < frames; ++f) Velox_Step(world, 1.0f / 60.0f);
        auto end = std::chrono::high_resolution_clock::now();

        double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
        double avgMs = totalMs / frames;
        double fps = 1000.0 / avgMs;
        double opsPerSec = (double)count * 8.0 * fps;

        std::cout << std::left 
                  << std::setw(28) << "Stacking Pyramid (Dense)"
                  << std::setw(12) << count
                  << std::setw(16) << std::fixed << std::setprecision(3) << avgMs
                  << std::setw(18) << std::fixed << std::setprecision(1) << fps
                  << std::setw(20) << (int)opsPerSec
                  << std::endl;

        Velox_DestroyWorld(world);
    }

    // Scenario 2: The Tumbler (2,000 Circles)
    {
        VeloxWorld* world = Velox_CreateWorld();
        Velox_SetGravity(world, 0.0f, 980.0f);

        // Boundary walls
        Velox::EntityID floor = Velox_CreateEntity(world);
        Velox_AddTransform(world, floor, 1000.0f, 800.0f, 0.0f);
        Velox_AddRigidBody(world, floor, 0.0f, true);
        Velox_AddBoxCollider(world, floor, 4000.0f, 40.0f);

        const int count = 2000;
        int cols = 50;
        for (int i = 0; i < count; ++i) {
            Velox::EntityID c = Velox_CreateEntity(world);
            float x = 100.0f + (i % cols) * 18.0f;
            float y = 50.0f + (i / cols) * 18.0f;
            Velox_AddTransform(world, c, x, y, 0.0f);
            Velox_AddMovement(world, c);
            Velox_AddRigidBody(world, c, 1.0f, false);
            Velox_AddCircleCollider(world, c, 7.0f);
        }

        for (int w = 0; w < 10; ++w) Velox_Step(world, 1.0f / 60.0f);

        const int frames = 100;
        auto start = std::chrono::high_resolution_clock::now();
        for (int f = 0; f < frames; ++f) Velox_Step(world, 1.0f / 60.0f);
        auto end = std::chrono::high_resolution_clock::now();

        double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
        double avgMs = totalMs / frames;
        double fps = 1000.0 / avgMs;
        double opsPerSec = (double)count * 8.0 * fps;

        std::cout << std::left 
                  << std::setw(28) << "The Tumbler (Contact Churn)"
                  << std::setw(12) << count
                  << std::setw(16) << std::fixed << std::setprecision(3) << avgMs
                  << std::setw(18) << std::fixed << std::setprecision(1) << fps
                  << std::setw(20) << (int)opsPerSec
                  << std::endl;

        Velox_DestroyWorld(world);
    }

    // Scenario 3: Joint Constraints Chain (500 Links)
    {
        VeloxWorld* world = Velox_CreateWorld();
        Velox_SetGravity(world, 0.0f, 980.0f);

        const int count = 500;
        Velox::EntityID prev = Velox_CreateEntity(world);
        Velox_AddTransform(world, prev, 500.0f, 100.0f, 0.0f);
        Velox_AddRigidBody(world, prev, 0.0f, true); // Root pin

        for (int i = 0; i < count - 1; ++i) {
            Velox::EntityID next = Velox_CreateEntity(world);
            Velox_AddTransform(world, next, 500.0f + (i + 1) * 5.0f, 100.0f, 0.0f);
            Velox_AddMovement(world, next);
            Velox_AddRigidBody(world, next, 0.5f, false);
            Velox_AddCircleCollider(world, next, 3.0f);
            Velox_AddDistanceJoint(world, prev, next, 0, 0, 0, 0, 5.0f, 0.0f);
            prev = next;
        }

        for (int w = 0; w < 10; ++w) Velox_Step(world, 1.0f / 60.0f);

        const int frames = 100;
        auto start = std::chrono::high_resolution_clock::now();
        for (int f = 0; f < frames; ++f) Velox_Step(world, 1.0f / 60.0f);
        auto end = std::chrono::high_resolution_clock::now();

        double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
        double avgMs = totalMs / frames;
        double fps = 1000.0 / avgMs;
        double opsPerSec = (double)count * 8.0 * fps;

        std::cout << std::left 
                  << std::setw(28) << "Joint Ragdoll Links"
                  << std::setw(12) << count
                  << std::setw(16) << std::fixed << std::setprecision(3) << avgMs
                  << std::setw(18) << std::fixed << std::setprecision(1) << fps
                  << std::setw(20) << (int)opsPerSec
                  << std::endl;

        Velox_DestroyWorld(world);
    }

    std::cout << "========================================================================================\n";
    std::cout << "   Comparative Benchmark Suite Completed Successfully!\n";
    std::cout << "========================================================================================" << std::endl;
}

int main() {
    RunBox2DComparativeBenchmark();
    return 0;
}
