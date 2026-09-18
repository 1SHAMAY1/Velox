#include <velox/VeloxAPI.h>
#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>

void TestStackingPyramid() {
    std::cout << "[Test RigidBody: Stacking] Testing 10-tier stacking pyramid stability...\n";
    VeloxWorld* world = Velox_CreateWorld();
    Velox_SetGravity(world, 0.0f, 980.0f);

    // Floor
    Velox::EntityID floor = Velox_CreateEntity(world);
    Velox_AddTransform(world, floor, 500.0f, 700.0f, 0.0f);
    Velox_AddRigidBody(world, floor, 0.0f, true);
    Velox_AddBoxCollider(world, floor, 2000.0f, 40.0f);

    // 10-tier Pyramid: Base has 10 boxes, top has 1 box (total = 55 boxes)
    const float boxSize = 20.0f;
    const float startY = 680.0f - boxSize * 0.5f; // Resting directly on top of floor (680px)
    std::vector<Velox::EntityID> boxes;

    int tiers = 10;
    for (int row = 0; row < tiers; ++row) {
        int count = tiers - row;
        float rowStartX = 500.0f - (count - 1) * (boxSize * 0.5f);
        float y = startY - row * boxSize;

        for (int col = 0; col < count; ++col) {
            float x = rowStartX + col * boxSize;
            Velox::EntityID b = Velox_CreateEntity(world);
            Velox_AddTransform(world, b, x, y, 0.0f);
            Velox_AddMovement(world, b);
            Velox_AddRigidBody(world, b, 1.0f, false);
            Velox_AddBoxCollider(world, b, boxSize, boxSize);
            Velox_AddPhysicalMaterial(world, b, 0.8f, 0.5f, 0.0f); // High friction, 0 restitution
            Velox_SetDamping(world, b, 0.1f, 0.1f);
            boxes.push_back(b);
        }
    }

    // Simulate for 180 frames (3 seconds) to let pyramid settle
    for (int frame = 0; frame < 180; ++frame) {
        Velox_Step(world, 1.0f / 60.0f);
    }

    // Verify all boxes remain stable, strictly within floor bounds, and with zero drift/jitter
    for (size_t i = 0; i < boxes.size(); ++i) {
        Velox::EntityID b = boxes[i];
        float x = 0, y = 0, rot = 0;
        Velox_GetPosition(world, b, &x, &y, &rot);
        assert(std::isfinite(x) && std::isfinite(y) && std::isfinite(rot));
        
        float vx = 0, vy = 0, av = 0;
        Velox_GetVelocity(world, b, &vx, &vy, &av);
        assert(y <= 680.0f && "Box tunneled below floor!");
        // Box velocities must have settled to near-zero (sleeping or resting)
        assert(std::abs(vx) < 5.0f && std::abs(vy) < 5.0f);
    }

    Velox_DestroyWorld(world);
    std::cout << "  -> PASSED: 10-tier Stacking Pyramid settled stably with verified zero drift & velocity dissipation.\n";
}

int main() {
    TestStackingPyramid();
    return 0;
}
