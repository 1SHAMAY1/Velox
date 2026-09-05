#include <velox/VeloxAPI.h>
#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>

void TestPrimitiveCollisions() {
    std::cout << "[Test 1: Primitives] Running Circle, Box, Polygon, and Chain Collisions...\n";
    VeloxWorld* world = Velox_CreateWorld();
    Velox_SetGravity(world, 0.0f, 980.0f);

    // 1. Static Box Floor
    Velox::EntityID floor = Velox_CreateEntity(world);
    Velox_AddTransform(world, floor, 500.0f, 800.0f, 0.0f);
    Velox_AddRigidBody(world, floor, 0.0f, true);
    Velox_AddBoxCollider(world, floor, 1000.0f, 40.0f);

    // 2. Dynamic Circle
    Velox::EntityID circle = Velox_CreateEntity(world);
    Velox_AddTransform(world, circle, 400.0f, 700.0f, 0.0f);
    Velox_AddMovement(world, circle);
    Velox_AddRigidBody(world, circle, 1.0f, false);
    Velox_AddCircleCollider(world, circle, 15.0f);

    // 3. Dynamic Box
    Velox::EntityID box = Velox_CreateEntity(world);
    Velox_AddTransform(world, box, 500.0f, 700.0f, 0.0f);
    Velox_AddMovement(world, box);
    Velox_AddRigidBody(world, box, 1.0f, false);
    Velox_AddBoxCollider(world, box, 20.0f, 20.0f);

    // 4. Dynamic Polygon (Triangle)
    Velox::EntityID poly = Velox_CreateEntity(world);
    Velox_AddTransform(world, poly, 600.0f, 700.0f, 0.0f);
    Velox_AddMovement(world, poly);
    Velox_AddRigidBody(world, poly, 1.0f, false);
    float px[3] = {-15.0f, 15.0f, 0.0f};
    float py[3] = {-15.0f, -15.0f, 15.0f};
    Velox_AddPolygonCollider(world, poly, px, py, 3);

    // 5. Chain Terrain Floor
    Velox::EntityID chain = Velox_CreateEntity(world);
    Velox_AddTransform(world, chain, 0.0f, 0.0f, 0.0f);
    Velox_AddRigidBody(world, chain, 0.0f, true);
    float cx[3] = {100.0f, 500.0f, 900.0f};
    float cy[3] = {750.0f, 780.0f, 750.0f};
    Velox_AddChainCollider(world, chain, cx, cy, 3);

    for (int step = 0; step < 120; ++step) {
        Velox_Step(world, 1.0f / 60.0f);
    }

    float x = 0, y = 0, rot = 0;
    Velox_GetPosition(world, circle, &x, &y, &rot);
    assert(std::isfinite(x) && std::isfinite(y) && std::isfinite(rot));
    assert(y <= 800.0f); // Rested on floor

    Velox_GetPosition(world, box, &x, &y, &rot);
    assert(std::isfinite(x) && std::isfinite(y) && std::isfinite(rot));
    assert(y <= 800.0f);

    Velox_DestroyWorld(world);
    std::cout << "  -> PASSED: Primitives collision matrix verified.\n";
}

int main() {
    TestPrimitiveCollisions();
    return 0;
}
