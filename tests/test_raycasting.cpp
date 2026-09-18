#include <velox/VeloxAPI.h>
#include <iostream>
#include <cassert>
#include <cmath>

void TestRaycastBoxHit() {
    std::cout << "[Test Raycast: Box] Testing ray intersection with Box collider...\n";
    VeloxWorld* world = Velox_CreateWorld();

    Velox::EntityID box = Velox_CreateEntity(world);
    Velox_AddTransform(world, box, 500.0f, 500.0f, 0.0f);
    Velox_AddRigidBody(world, box, 0.0f, true);
    Velox_AddBoxCollider(world, box, 100.0f, 100.0f); // Extents: x in [450, 550], y in [450, 550]

    float hitX = 0, hitY = 0, normX = 0, normY = 0, frac = 0;
    Velox::EntityID hitEntity = 0;

    // Cast ray from (300, 500) heading right (1, 0)
    bool hit = Velox_Raycast(world, 300.0f, 500.0f, 1.0f, 0.0f, 500.0f,
                             &hitX, &hitY, &normX, &normY, &frac, &hitEntity);

    assert(hit == true);
    assert(hitEntity == box);
    assert(std::abs(hitX - 450.0f) < 1.0f); // Left face of box
    assert(std::abs(hitY - 500.0f) < 1.0f);
    assert(std::abs(normX - (-1.0f)) < 0.01f); // Facing left towards ray origin

    Velox_DestroyWorld(world);
    std::cout << "  -> PASSED: Raycast against Box collider returned exact intersection and normal.\n";
}

void TestRaycastCircleHit() {
    std::cout << "[Test Raycast: Circle] Testing ray intersection with Circle collider...\n";
    VeloxWorld* world = Velox_CreateWorld();

    Velox::EntityID circle = Velox_CreateEntity(world);
    Velox_AddTransform(world, circle, 600.0f, 600.0f, 0.0f);
    Velox_AddRigidBody(world, circle, 0.0f, true);
    Velox_AddCircleCollider(world, circle, 20.0f); // Radius 20

    float hitX = 0, hitY = 0, normX = 0, normY = 0, frac = 0;
    Velox::EntityID hitEntity = 0;

    // Cast ray downwards from (600, 400) heading down (0, 1)
    bool hit = Velox_Raycast(world, 600.0f, 400.0f, 0.0f, 1.0f, 500.0f,
                             &hitX, &hitY, &normX, &normY, &frac, &hitEntity);

    assert(hit == true);
    assert(hitEntity == circle);
    assert(std::abs(hitX - 600.0f) < 1.0f);
    assert(std::abs(hitY - 580.0f) < 1.0f); // Top of circle (600 - 20)
    assert(std::abs(normY - (-1.0f)) < 0.01f); // Facing upward

    // Raycast that misses
    bool miss = Velox_Raycast(world, 200.0f, 200.0f, 0.0f, 1.0f, 100.0f,
                              &hitX, &hitY, &normX, &normY, &frac, &hitEntity);
    assert(miss == false);

    Velox_DestroyWorld(world);
    std::cout << "  -> PASSED: Raycast against Circle collider verified on hit and miss.\n";
}

int main() {
    std::cout << "=== Running Velox Raycasting Unit Tests ===\n";
    TestRaycastBoxHit();
    TestRaycastCircleHit();
    std::cout << "=== All Raycasting Unit Tests Passed Successfully! ===\n";
    return 0;
}
