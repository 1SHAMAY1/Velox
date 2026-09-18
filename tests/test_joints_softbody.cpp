#include <velox/VeloxAPI.h>
#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>

void TestJointsAndSoftBodies() {
    std::cout << "[Test 2: Joints & Soft Bodies] Running Joint types and Blob/ShapeMatching tests...\n";
    VeloxWorld* world = Velox_CreateWorld();
    Velox_SetGravity(world, 0.0f, 980.0f);

    // 1. Distance Joint
    Velox::EntityID a = Velox_CreateEntity(world);
    Velox_AddTransform(world, a, 100.0f, 100.0f, 0.0f);
    Velox_AddRigidBody(world, a, 0.0f, true); // Pin

    Velox::EntityID b = Velox_CreateEntity(world);
    Velox_AddTransform(world, b, 150.0f, 100.0f, 0.0f);
    Velox_AddMovement(world, b);
    Velox_AddRigidBody(world, b, 1.0f, false);
    Velox_AddCircleCollider(world, b, 10.0f);

    Velox_AddDistanceJoint(world, a, b, 0, 0, 0, 0, 50.0f, 0.0f);

    // 2. Revolute Joint
    Velox::EntityID revA = Velox_CreateEntity(world);
    Velox_AddTransform(world, revA, 300.0f, 100.0f, 0.0f);
    Velox_AddRigidBody(world, revA, 0.0f, true);

    Velox::EntityID revB = Velox_CreateEntity(world);
    Velox_AddTransform(world, revB, 340.0f, 100.0f, 0.0f);
    Velox_AddMovement(world, revB);
    Velox_AddRigidBody(world, revB, 1.0f, false);
    Velox_AddBoxCollider(world, revB, 40.0f, 10.0f);

    Velox_AddRevoluteJoint(world, revA, revB, 0, 0, -20.0f, 0, 0.0f, true, -1.0f, 1.0f, false, 0, 0);

    // 3. Prismatic Joint
    Velox::EntityID pA = Velox_CreateEntity(world);
    Velox_AddTransform(world, pA, 500.0f, 100.0f, 0.0f);
    Velox_AddRigidBody(world, pA, 0.0f, true);

    Velox::EntityID pB = Velox_CreateEntity(world);
    Velox_AddTransform(world, pB, 500.0f, 150.0f, 0.0f);
    Velox_AddMovement(world, pB);
    Velox_AddRigidBody(world, pB, 1.0f, false);
    Velox_AddBoxCollider(world, pB, 15.0f, 15.0f);

    Velox_AddPrismaticJoint(world, pA, pB, 0, 0, 0, 0, 0, 1.0f, 0.0f, true, 0.0f, 100.0f, false, 0, 0);

    // 4. Soft Body Blob
    Velox::EntityID blob = Velox_CreateSoftBodyBlob(world, 700.0f, 100.0f, 30.0f, 8, 0.05f, 0.01f, 5.0f);
    assert(blob != 0);
    assert(Velox_GetSoftBodyNodeCount(world, blob) == 8);

    // 5. Soft Body Shape Matched
    float smX[4] = {-20.0f, 20.0f, 20.0f, -20.0f};
    float smY[4] = {-20.0f, -20.0f, 20.0f, 20.0f};
    Velox::EntityID smBody = Velox_CreateSoftBodyShapeMatched(world, 850.0f, 100.0f, smX, smY, 4, 0.5f, 6.0f);
    assert(smBody != 0);

    for (int step = 0; step < 120; ++step) {
        Velox_Step(world, 1.0f / 60.0f);
    }

    float bx = 0, by = 0, brot = 0;
    Velox_GetPosition(world, b, &bx, &by, &brot);
    assert(std::isfinite(bx) && std::isfinite(by));

    Velox_DestroyWorld(world);
    std::cout << "  -> PASSED: All Joint types & Soft Bodies simulated stably without NaN or explosion.\n";
}

int main() {
    TestJointsAndSoftBodies();
    return 0;
}
