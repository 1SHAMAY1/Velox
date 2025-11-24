#include <iostream>
#include <vector>
#include <chrono>
#include <velox/PhysicsEngineAPI.h>

int main() {
    std::cout << "Initializing Velox World..." << std::endl;
    VeloxWorld* world = Velox_CreateWorld();

    // Create Ground
    auto ground = Velox_CreateEntity(world);
    Velox_AddTransform(world, ground, 0, -5.0f, 0);
    Velox_AddRigidBody(world, ground, 0.0f, true); // Static

    // Create Falling Object
    auto ball = Velox_CreateEntity(world);
    Velox_AddTransform(world, ball, 0, 10.0f, 0);
    Velox_AddRigidBody(world, ball, 1.0f, false); // Dynamic

    std::cout << "Simulating..." << std::endl;

    const float dt = 0.016f;
    for (int i = 0; i < 100; ++i) {
        Velox_Step(world, dt);
        
        float x, y, z;
        Velox_GetPosition(world, ball, &x, &y, &z);
        
        if (i % 10 == 0) {
            std::cout << "Frame " << i << ": Ball Y = " << y << std::endl;
        }
    }

    Velox_DestroyWorld(world);
    std::cout << "Simulation Complete." << std::endl;
    return 0;
}
