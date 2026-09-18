#include <velox/VeloxAPI.h>
#include <iostream>
#include <vector>
#include <chrono>

int main() {
    std::cout << "=== Velox 2D Physics Pipeline Benchmark ===\n";

    const int counts[] = {100, 500, 1000, 2000};

    for (int count : counts) {
        VeloxWorld* world = Velox_CreateWorld();
        Velox_SetGravity(world, 0.0f, 980.0f);

        // Floor
        Velox::EntityID floor = Velox_CreateEntity(world);
        Velox_AddTransform(world, floor, 640.0f, 700.0f, 0.0f);
        Velox_AddRigidBody(world, floor, 0.0f, true);
        Velox_AddBoxCollider(world, floor, 4000.0f, 40.0f);

        // Dynamic boxes
        int cols = 40;
        for (int i = 0; i < count; ++i) {
            Velox::EntityID e = Velox_CreateEntity(world);
            float x = 200.0f + (i % cols) * 20.0f;
            float y = 100.0f + (i / cols) * 20.0f;
            Velox_AddTransform(world, e, x, y, 0.0f);
            Velox_AddMovement(world, e);
            Velox_AddRigidBody(world, e, 1.0f, false);
            Velox_AddBoxCollider(world, e, 16.0f, 16.0f);
        }

        // Warmup
        for (int w = 0; w < 10; ++w) {
            Velox_Step(world, 1.0f / 60.0f);
        }

        const int frames = 100;
        auto start = std::chrono::high_resolution_clock::now();
        for (int f = 0; f < frames; ++f) {
            Velox_Step(world, 1.0f / 60.0f);
        }
        auto end = std::chrono::high_resolution_clock::now();

        double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
        double avgStepMs = totalMs / frames;
        double simulatedFps = 1000.0 / avgStepMs;

        std::cout << "Bodies: " << count 
                  << " | Avg Step: " << avgStepMs << " ms"
                  << " | Simulated FPS: " << simulatedFps << " FPS\n";

        Velox_DestroyWorld(world);
    }

    std::cout << "=== Benchmark Completed Successfully ===\n";
    return 0;
}
