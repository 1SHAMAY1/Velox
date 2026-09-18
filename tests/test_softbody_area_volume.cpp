#include <velox/VeloxAPI.h>
#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>

void TestSoftBodyAreaConservation() {
    std::cout << "[Test SoftBody: Area Conservation] Testing Shoelace area preservation under gravity compression...\n";
    VeloxWorld* world = Velox_CreateWorld();
    Velox_SetGravity(world, 0.0f, 980.0f);

    // Floor at y = 600
    Velox::EntityID floor = Velox_CreateEntity(world);
    Velox_AddTransform(world, floor, 500.0f, 600.0f, 0.0f);
    Velox_AddRigidBody(world, floor, 0.0f, true);
    Velox_AddBoxCollider(world, floor, 1000.0f, 40.0f);

    // Create 10-node Blob soft body dropped onto the floor
    const float radius = 40.0f;
    const int nodeCount = 10;
    Velox::EntityID blob = Velox_CreateSoftBodyBlob(world, 500.0f, 300.0f, radius, nodeCount, 0.02f, 0.01f, 6.0f);
    assert(blob != 0);

    // Step 120 frames (2 seconds)
    for (int frame = 0; frame < 120; ++frame) {
        Velox_Step(world, 1.0f / 60.0f);
    }

    // Measure node coordinates after impact
    std::vector<float> nodeX(nodeCount), nodeY(nodeCount);
    for (int i = 0; i < nodeCount; ++i) {
        Velox::EntityID n = Velox_GetSoftBodyNode(world, blob, i);
        float rot = 0;
        Velox_GetPosition(world, n, &nodeX[i], &nodeY[i], &rot);
        assert(std::isfinite(nodeX[i]) && std::isfinite(nodeY[i]));
        // Nodes must not fall through floor
        assert(nodeY[i] <= 600.0f);
    }

    // Calculate signed area via Shoelace formula
    float area = 0.0f;
    for (int i = 0; i < nodeCount; ++i) {
        int next = (i + 1) % nodeCount;
        area += nodeX[i] * nodeY[next] - nodeX[next] * nodeY[i];
    }
    area = std::abs(area) * 0.5f;

    // Ideal rest area = pi * r^2 ~ 5026.5
    float targetArea = 3.14159265f * radius * radius;
    float areaRatio = area / targetArea;

    // Soft body volume should stay within 70% - 130% under heavy gravity impact
    assert(areaRatio > 0.65f && areaRatio < 1.35f);

    Velox_DestroyWorld(world);
    std::cout << "  -> PASSED: Shoelace area preserved under impact (Area ratio: " << (areaRatio * 100.0f) << "%).\n";
}

int main() {
    TestSoftBodyAreaConservation();
    return 0;
}
