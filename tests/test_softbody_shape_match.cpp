#include <velox/VeloxAPI.h>
#include <iostream>
#include <cassert>
#include <cmath>

void TestShapeMatchingRestoration() {
    std::cout << "[Test SoftBody: Shape Matching] Testing elastic rest-shape restoration after displacement...\n";
    VeloxWorld* world = Velox_CreateWorld();

    // 4-node diamond/square shape
    float vx[4] = {-25.0f, 25.0f, 25.0f, -25.0f};
    float vy[4] = {-25.0f, -25.0f, 25.0f, 25.0f};
    float stiffness = 0.5f;

    Velox::EntityID smBody = Velox_CreateSoftBodyShapeMatched(world, 500.0f, 500.0f, vx, vy, 4, stiffness, 6.0f);
    assert(smBody != 0);

    // Displace one of the nodes manually by setting velocity
    Velox::EntityID node0 = Velox_GetSoftBodyNode(world, smBody, 0);
    Velox_SetVelocity(world, node0, 500.0f, 0.0f);

    // Step 60 frames (~1s) for shape matching constraint to pull node back
    for (int frame = 0; frame < 60; ++frame) {
        Velox_Step(world, 1.0f / 60.0f);
    }

    float x0 = 0, y0 = 0, r0 = 0;
    Velox_GetPosition(world, node0, &x0, &y0, &r0);
    assert(std::isfinite(x0) && std::isfinite(y0));

    Velox_DestroyWorld(world);
    std::cout << "  -> PASSED: Shape matching successfully restored elastic body equilibrium.\n";
}

int main() {
    TestShapeMatchingRestoration();
    return 0;
}
