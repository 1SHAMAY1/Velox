#include <velox/VeloxAPI.h>
#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>

void TestContinuousCollisionDetection() {
    std::cout << "[Test RigidBody: CCD] Testing Extreme Multi-Angle Hyper-Velocity CCD (up to 50,000 px/s)...\n";

    // Test angles: 0 deg (horizontal), 30 deg, 45 deg, 60 deg, 85 deg
    float anglesDeg[] = { 0.0f, 30.0f, 45.0f, 60.0f, 85.0f };
    float speeds[] = { 8000.0f, 15000.0f, 30000.0f, 50000.0f };

    for (float speed : speeds) {
        for (float deg : anglesDeg) {
            VeloxWorld* world = Velox_CreateWorld();
            Velox_SetGravity(world, 0.0f, 0.0f);

            // Thin 4px static vertical wall at x = 500
            Velox::EntityID wall = Velox_CreateEntity(world);
            Velox_AddTransform(world, wall, 500.0f, 300.0f, 0.0f);
            Velox_AddRigidBody(world, wall, 0.0f, true);
            Velox_AddBoxCollider(world, wall, 4.0f, 800.0f);
            Velox_AddPhysicalMaterial(world, wall, 0.0f, 0.0f, 0.5f);

            // Bullet with non-aligned start x = 113.7px
            float rad = deg * 3.14159265f / 180.0f;
            float vx = speed * cosf(rad);
            float vy = speed * sinf(rad);

            Velox::EntityID bullet = Velox_CreateEntity(world);
            Velox_AddTransform(world, bullet, 113.7f, 300.0f - 100.0f * sinf(rad), 0.0f);
            Velox_AddMovement(world, bullet);
            Velox_SetVelocity(world, bullet, vx, vy);
            Velox_AddRigidBody(world, bullet, 1.0f, false);
            Velox_AddCircleCollider(world, bullet, 6.0f);
            Velox_AddPhysicalMaterial(world, bullet, 0.0f, 0.0f, 0.5f);

            // Step 10 frames
            for (int frame = 0; frame < 10; ++frame) {
                Velox_Step(world, 1.0f / 60.0f);
            }

            float bx = 0, by = 0, rot = 0;
            Velox_GetPosition(world, bullet, &bx, &by, &rot);

            // Bullet must NEVER tunnel past wall (x = 500). Max valid x with radius 6 is 506.0
            assert(bx <= 506.5f);
            assert(std::isfinite(bx) && std::isfinite(by));

            Velox_DestroyWorld(world);
        }
    }

    std::cout << "  -> PASSED: 100% Zero tunneling across all angles (0-85 deg) and extreme speeds (up to 50,000 px/s)!\n";
}

int main() {
    TestContinuousCollisionDetection();
    return 0;
}

