#include <velox/VeloxAPI.h>
#include "core/World.h"
#include "physics/Components.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <cassert>
#include <iomanip>
#include <random>

#if defined(_WIN32)
#include <windows.h>
#include <psapi.h>
static size_t GetProcessWorkingSet() {
    PROCESS_MEMORY_COUNTERS_EX pmc{};
    if (GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc))) {
        return pmc.WorkingSetSize;
    }
    return 0;
}
#else
static size_t GetProcessWorkingSet() { return 0; }
#endif

static bool ValidateWorldIntegrity(VeloxWorld* worldHandle, std::string& outError) {
    auto* world = reinterpret_cast<Velox::World*>(worldHandle);
    auto& em = world->GetEntityManager();

    for (Velox::EntityID e = 0; e < 2000; ++e) {
        if (em.HasComponent<Velox::TransformComponent>(e)) {
            const auto& tf = em.GetComponent<Velox::TransformComponent>(e);
            if (std::isnan(tf.Position.x) || std::isnan(tf.Position.y) || std::isinf(tf.Position.x) || std::isinf(tf.Position.y)) {
                outError = "Transform position NaN/Inf at entity " + std::to_string(e);
                return false;
            }
            if (std::isnan(tf.Rotation) || std::isinf(tf.Rotation)) {
                outError = "Transform rotation NaN/Inf at entity " + std::to_string(e);
                return false;
            }
        }
        if (em.HasComponent<Velox::MovementComponent>(e)) {
            const auto& mv = em.GetComponent<Velox::MovementComponent>(e);
            if (std::isnan(mv.Velocity.x) || std::isnan(mv.Velocity.y) || std::isinf(mv.Velocity.x) || std::isinf(mv.Velocity.y)) {
                outError = "Movement linear velocity NaN/Inf at entity " + std::to_string(e);
                return false;
            }
            if (std::isnan(mv.AngularVelocity) || std::isinf(mv.AngularVelocity)) {
                outError = "Movement angular velocity NaN/Inf at entity " + std::to_string(e);
                return false;
            }
        }
    }
    return true;
}

static void AddWalls(VeloxWorld* world, float screenWidth, float screenHeight, float wallThickness = 40.0f) {
    struct WallDef { float x, y, w, h; };
    WallDef walls[] = {
        {screenWidth/2.0f, wallThickness/2.0f, screenWidth, wallThickness},
        {screenWidth/2.0f, screenHeight - wallThickness/2.0f, screenWidth, wallThickness},
        {wallThickness/2.0f, screenHeight/2.0f, wallThickness, screenHeight - 2*wallThickness},
        {screenWidth - wallThickness/2.0f, screenHeight/2.0f, wallThickness, screenHeight - 2*wallThickness}
    };

    for (const auto& w : walls) {
        auto id = Velox_CreateEntity(world);
        Velox_AddTransform(world, id, w.x, w.y, 0.0f);
        Velox_AddRigidBody(world, id, 0.0f, true);
        Velox_AddMovement(world, id);
        Velox_AddBoxCollider(world, id, w.w, w.h);
    }
}

// 1. Bouncing Balls
static void Setup1_BouncingBalls(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H, 20.0f);
    Velox_SetGravity(world, 0.0f, 0.0f);

    for (int i = 0; i < 2; ++i) {
        auto id = Velox_CreateEntity(world);
        float startX = (i == 0) ? W * 0.35f : W * 0.65f;
        float startY = H * 0.3f;
        Velox_AddTransform(world, id, startX, startY, 0.0f);
        Velox_AddRigidBody(world, id, 1.0f, false);
        Velox_AddMovement(world, id);
        Velox_AddCircleCollider(world, id, 20.0f);
        Velox_AddPhysicalMaterial(world, id, 0.0f, 0.0f, 1.0f);
        float vx = (i == 0) ? 500.0f : -520.0f;
        float vy = (i == 0) ? 480.0f : -460.0f;
        Velox_SetVelocity(world, id, vx, vy);
        Velox_SetDamping(world, id, 0.0f, 0.0f);
        Velox_AddRotation(world, id, 5.0f, 0, 0);
    }

    struct Obstacle { float x, y, r; };
    Obstacle obstacles[] = {
        {W * 0.25f, H * 0.25f, 40.0f},
        {W * 0.75f, H * 0.25f, 40.0f},
        {W * 0.25f, H * 0.75f, 40.0f},
        {W * 0.75f, H * 0.75f, 40.0f},
        {W * 0.5f, H * 0.5f, 60.0f}
    };
    for (const auto& obs : obstacles) {
        auto id = Velox_CreateEntity(world);
        Velox_AddTransform(world, id, obs.x, obs.y, 0.0f);
        Velox_AddRigidBody(world, id, 0.0f, true);
        Velox_AddMovement(world, id);
        Velox_AddCircleCollider(world, id, obs.r);
        Velox_AddPhysicalMaterial(world, id, 0.0f, 0.0f, 1.0f);
    }
}

// 2. Force Field Demo
static void Setup2_ForceFieldDemo(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H);
    Velox_SetGravity(world, 0.0f, 0.0f);

    auto f1 = Velox_CreateEntity(world);
    Velox_AddTransform(world, f1, W * 0.3f, H * 0.5f, 0.0f);
    Velox_AddForceField(world, f1, 0, 600.0f, 200.0f);

    auto f2 = Velox_CreateEntity(world);
    Velox_AddTransform(world, f2, W * 0.7f, H * 0.5f, 0.0f);
    Velox_AddForceField(world, f2, 1, -600.0f, 200.0f);

    for (int i = 0; i < 60; ++i) {
        auto id = Velox_CreateEntity(world);
        float x = W * 0.4f + (rand() % 200);
        float y = H * 0.2f + (rand() % 400);
        Velox_AddTransform(world, id, x, y, 0.0f);
        Velox_AddRigidBody(world, id, 1.0f, false);
        Velox_AddMovement(world, id);
        Velox_AddCircleCollider(world, id, 10.0f);
        Velox_SetDamping(world, id, 0.1f, 0.1f);
    }
}

// 3. Oscillation Demo
static void Setup3_OscillationDemo(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H);
    Velox_SetGravity(world, 0.0f, 980.0f);

    for (int i = 0; i < 5; ++i) {
        float anchorX = W * 0.2f + i * (W * 0.15f);
        float anchorY = H * 0.15f;

        auto anchor = Velox_CreateEntity(world);
        Velox_AddTransform(world, anchor, anchorX, anchorY, 0.0f);
        Velox_AddRigidBody(world, anchor, 0.0f, true);
        Velox_AddMovement(world, anchor);
        Velox_AddCircleCollider(world, anchor, 8.0f);

        auto bob = Velox_CreateEntity(world);
        float initOffset = (i % 2 == 0) ? 150.0f : -150.0f;
        Velox_AddTransform(world, bob, anchorX + initOffset, anchorY + 200.0f, 0.0f);
        Velox_AddRigidBody(world, bob, 2.0f, false);
        Velox_AddMovement(world, bob);
        Velox_AddCircleCollider(world, bob, 18.0f);
        Velox_AddPhysicalMaterial(world, bob, 0.1f, 0.05f, 0.95f);
        Velox_SetDamping(world, bob, 0.001f, 0.001f);

        Velox_AddDistanceJoint(world, anchor, bob, 0.0f, 0.0f, 0.0f, 0.0f, 220.0f, 0.0f);
    }
}

// 4. Projectile Demo
static void Setup4_ProjectileDemo(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H);
    Velox_SetGravity(world, 0.0f, 980.0f);

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 8; ++j) {
            auto b = Velox_CreateEntity(world);
            Velox_AddTransform(world, b, W * 0.7f + i * 26.0f, H - 60.0f - j * 26.0f, 0.0f);
            Velox_AddRigidBody(world, b, 1.0f, false);
            Velox_AddMovement(world, b);
            Velox_AddBoxCollider(world, b, 24.0f, 24.0f);
            Velox_AddPhysicalMaterial(world, b, 0.5f, 0.4f, 0.1f);
        }
    }

    auto proj = Velox_CreateEntity(world);
    Velox_AddTransform(world, proj, 80.0f, H * 0.6f, 0.0f);
    Velox_AddRigidBody(world, proj, 15.0f, false);
    Velox_AddMovement(world, proj);
    Velox_AddCircleCollider(world, proj, 22.0f);
    Velox_SetVelocity(world, proj, 900.0f, -400.0f);
}

// 5. Gravity Demo
static void Setup5_GravityDemo(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H);
    Velox_SetGravity(world, 0.0f, 980.0f);

    for (int i = 0; i < 50; ++i) {
        auto e = Velox_CreateEntity(world);
        Velox_AddTransform(world, e, W * 0.3f + (rand() % 500), H * 0.3f + (rand() % 300), 0.0f);
        Velox_AddRigidBody(world, e, 1.0f, false);
        Velox_AddMovement(world, e);
        if (i % 2 == 0) Velox_AddCircleCollider(world, e, 14.0f);
        else Velox_AddBoxCollider(world, e, 24.0f, 24.0f);
        Velox_AddPhysicalMaterial(world, e, 0.3f, 0.2f, 0.5f);
    }
}

// 6. Joint Demo
static void Setup6_JointDemo(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H);
    Velox_SetGravity(world, 0.0f, 980.0f);

    auto anchor = Velox_CreateEntity(world);
    Velox_AddTransform(world, anchor, W * 0.25f, 80.0f, 0.0f);
    Velox_AddRigidBody(world, anchor, 0.0f, true);
    Velox_AddMovement(world, anchor);
    Velox_AddCircleCollider(world, anchor, 8.0f);

    Velox::EntityID prev = anchor;
    float segLen = 60.0f;
    for (int i = 0; i < 5; ++i) {
        auto link = Velox_CreateEntity(world);
        Velox_AddTransform(world, link, W * 0.25f + 20.0f * i, 80.0f + segLen * (i + 1), 0.0f);
        Velox_AddRigidBody(world, link, 1.0f, false);
        Velox_AddMovement(world, link);
        Velox_AddCircleCollider(world, link, 12.0f);
        Velox_SetDamping(world, link, 0.1f, 0.1f);
        Velox_AddPhysicalMaterial(world, link, 0.3f, 0.2f, 0.2f);
        Velox_AddDistanceJoint(world, prev, link, 0.0f, 0.0f, 0.0f, 0.0f, segLen, 0.0f);
        prev = link;
    }
}

// 7. Sandbox Demo
static void Setup7_SandboxDemo(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H);
    Velox_SetGravity(world, 0.0f, 900.0f);

    float pivotX = W * 0.5f;
    float pivotY = H * 0.35f;

    auto motorAnchorId = Velox_CreateEntity(world);
    Velox_AddTransform(world, motorAnchorId, pivotX, pivotY, 0.0f);
    Velox_AddRigidBody(world, motorAnchorId, 0.0f, true);
    Velox_AddMovement(world, motorAnchorId);
    Velox_AddCircleCollider(world, motorAnchorId, 10.0f);

    auto bladeId = Velox_CreateEntity(world);
    Velox_AddTransform(world, bladeId, pivotX + 100.0f, pivotY, 0.0f);
    Velox_AddRigidBody(world, bladeId, 1.5f, false);
    Velox_AddMovement(world, bladeId);
    Velox_SetDamping(world, bladeId, 0.02f, 0.02f);

    float pxVertsX[] = { -60.0f, 60.0f, 0.0f };
    float pxVertsY[] = { -20.0f, -20.0f, 30.0f };
    Velox_AddPolygonCollider(world, bladeId, pxVertsX, pxVertsY, 3);

    auto jointId = Velox_CreateEntity(world);
    Velox_AddDistanceJoint(world, motorAnchorId, bladeId, 0.0f, 0.0f, -100.0f, 0.0f, 100.0f, 0.0f);
    Velox_SetJointMotor(world, jointId, true, 2.5f, 20.0f);

    for (int i = 0; i < 4; ++i) {
        auto polyId = Velox_CreateEntity(world);
        Velox_AddTransform(world, polyId, W * 0.25f + i * 160.0f, 100.0f, 0.5f * i);
        Velox_AddRigidBody(world, polyId, 1.0f, false);
        Velox_AddMovement(world, polyId);
        Velox_SetDamping(world, polyId, 0.005f, 0.005f);
        Velox_AddPhysicalMaterial(world, polyId, 0.3f, 0.2f, 0.85f);

        float pX[5], pY[5];
        float r = 25.0f;
        for (int v = 0; v < 5; ++v) {
            float angle = (float)v * (2.0f * 3.14159265f / 5.0f);
            pX[v] = std::cos(angle) * r;
            pY[v] = std::sin(angle) * r;
        }
        Velox_AddPolygonCollider(world, polyId, pX, pY, 5);
    }
}

// 8. Chain Demo
static void Setup8_ChainDemo(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H);
    Velox_SetGravity(world, 0.0f, 900.0f);

    auto chainId = Velox_CreateEntity(world);
    Velox_AddTransform(world, chainId, 0.0f, 0.0f, 0.0f);
    Velox_AddRigidBody(world, chainId, 0.0f, true);
    Velox_AddMovement(world, chainId);

    std::vector<float> cx, cy;
    int numPts = 30;
    for (int i = 0; i < numPts; ++i) {
        float x = (W / (float)(numPts - 1)) * i;
        float y = H * 0.7f + std::sin(i * 0.5f) * 60.0f;
        cx.push_back(x);
        cy.push_back(y);
    }
    Velox_AddChainCollider(world, chainId, cx.data(), cy.data(), numPts);

    for (int i = 0; i < 6; ++i) {
        auto ballId = Velox_CreateEntity(world);
        Velox_AddTransform(world, ballId, 120.0f + i * 90.0f, 100.0f, 0.0f);
        Velox_AddRigidBody(world, ballId, 1.0f, false);
        Velox_AddMovement(world, ballId);
        Velox_AddCircleCollider(world, ballId, 14.0f);
        Velox_AddPhysicalMaterial(world, ballId, 0.1f, 0.05f, 0.7f);
    }
}

// 9. Raycast Demo
static void Setup9_RaycastDemo(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H);
    Velox_SetGravity(world, 0.0f, 0.0f);

    for (int i = 0; i < 25; ++i) {
        auto e = Velox_CreateEntity(world);
        Velox_AddTransform(world, e, 200.0f + (i % 5) * 200.0f, 150.0f + (i / 5) * 120.0f, 0.2f * i);
        Velox_AddRigidBody(world, e, 0.0f, true);
        Velox_AddMovement(world, e);
        if (i % 2 == 0) Velox_AddBoxCollider(world, e, 50.0f, 50.0f);
        else Velox_AddCircleCollider(world, e, 30.0f);
    }
}

// 10. Revolute & Prismatic Demo
static void Setup10_RevolutePrismatic(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H);
    Velox_SetGravity(world, 0.0f, 980.0f);

    float rx = W * 0.22f;
    float ry = H * 0.25f;

    auto pin = Velox_CreateEntity(world);
    Velox_AddTransform(world, pin, rx, ry, 0.0f);
    Velox_AddRigidBody(world, pin, 0.0f, true);
    Velox_AddMovement(world, pin);
    Velox_AddCircleCollider(world, pin, 10.0f);

    auto arm = Velox_CreateEntity(world);
    Velox_AddTransform(world, arm, rx + 60.0f, ry, 0.0f);
    Velox_AddRigidBody(world, arm, 2.0f, false);
    Velox_AddMovement(world, arm);
    Velox_AddBoxCollider(world, arm, 120.0f, 16.0f);
    Velox_AddRevoluteJoint(world, pin, arm, 0.0f, 0.0f, -60.0f, 0.0f, 0.0f, false, 0.0f, 0.0f, true, 3.0f, 500000.0f);

    float sx = W * 0.58f;
    float sy = H * 0.25f;
    auto anchor = Velox_CreateEntity(world);
    Velox_AddTransform(world, anchor, sx, sy, 0.0f);
    Velox_AddRigidBody(world, anchor, 0.0f, true);
    Velox_AddMovement(world, anchor);
    Velox_AddCircleCollider(world, anchor, 8.0f);

    auto slider = Velox_CreateEntity(world);
    Velox_AddTransform(world, slider, sx, sy, 0.0f);
    Velox_AddRigidBody(world, slider, 1.5f, false);
    Velox_AddMovement(world, slider);
    Velox_AddBoxCollider(world, slider, 45.0f, 30.0f);
    Velox_AddPrismaticJoint(world, anchor, slider, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, -120.0f, 120.0f, true, 80.0f, 50.0f);
}

// 11. CCD Showcase
static void Setup11_CCDShowcase(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H);
    Velox_SetGravity(world, 0.0f, 0.0f);

    auto wall = Velox_CreateEntity(world);
    Velox_AddTransform(world, wall, W * 0.5f, H * 0.5f, 0.0f);
    Velox_AddRigidBody(world, wall, 0.0f, true);
    Velox_AddMovement(world, wall);
    Velox_AddBoxCollider(world, wall, 15.0f, H - 200.0f);

    auto bullet = Velox_CreateEntity(world);
    Velox_AddTransform(world, bullet, 100.0f, H * 0.5f, 0.0f);
    Velox_AddRigidBody(world, bullet, 1.0f, false);
    Velox_AddMovement(world, bullet);
    Velox_AddCircleCollider(world, bullet, 10.0f);
    Velox_AddPhysicalMaterial(world, bullet, 0.2f, 0.1f, 0.9f);
    Velox_SetVelocity(world, bullet, 35000.0f, 0.0f);
}

// 12. Sleeping Showcase
static void Setup12_SleepingShowcase(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H);
    Velox_SetGravity(world, 0.0f, 980.0f);

    float boxSize = 35.0f;
    float startX = W * 0.5f;
    float startY = H - 20.0f - 20.0f - boxSize/2.0f;

    for (int row = 0; row < 6; ++row) {
        auto box = Velox_CreateEntity(world);
        float bx = startX;
        float by = startY - row * boxSize;
        Velox_AddTransform(world, box, bx, by, 0.0f);
        Velox_AddRigidBody(world, box, 1.0f, false);
        Velox_AddMovement(world, box);
        Velox_AddBoxCollider(world, box, boxSize, boxSize);
        Velox_AddPhysicalMaterial(world, box, 0.9f, 0.8f, 0.0f);
        Velox_SetDamping(world, box, 0.2f, 0.2f);
    }

    auto triggerBall = Velox_CreateEntity(world);
    Velox_AddTransform(world, triggerBall, W * 0.2f, H * 0.5f, 0.0f);
    Velox_AddRigidBody(world, triggerBall, 10.0f, false);
    Velox_AddMovement(world, triggerBall);
    Velox_AddCircleCollider(world, triggerBall, 30.0f);
    Velox_SetVelocity(world, triggerBall, 450.0f, -50.0f);
}

// 13. Soft Body Sandbox
static void Setup13_SoftBodySandbox(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H);
    Velox_SetGravity(world, 0.0f, 900.0f);

    auto blob = Velox_CreateSoftBodyBlob(world, W * 0.3f, H * 0.25f, 65.0f, 16, 0.04f, 0.05f, 10.0f);

    float starVertsX[] = { 0, 20, 65, 30, 45, 0, -45, -30, -65, -20 };
    float starVertsY[] = { -65, -20, -20, 10, 55, 30, 55, 10, -20, -20 };
    int starCount = 10;
    auto star = Velox_CreateSoftBodyShapeMatched(world, W * 0.7f, H * 0.25f, starVertsX, starVertsY, starCount, 0.02f, 9.0f);
}

// 14. Soft Body Funnel
static void Setup14_SoftBodyFunnel(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H);
    Velox_SetGravity(world, 0.0f, 950.0f);

    float wallH = 25.0f;
    float flatW = 380.0f;
    float slopeW = 290.0f;

    auto leftFlat = Velox_CreateEntity(world);
    Velox_AddTransform(world, leftFlat, W * 0.5f - 350.0f, H * 0.22f, 0.0f);
    Velox_AddRigidBody(world, leftFlat, 0.0f, true);
    Velox_AddMovement(world, leftFlat);
    Velox_AddBoxCollider(world, leftFlat, flatW, wallH);

    auto rightFlat = Velox_CreateEntity(world);
    Velox_AddTransform(world, rightFlat, W * 0.5f + 350.0f, H * 0.22f, 0.0f);
    Velox_AddRigidBody(world, rightFlat, 0.0f, true);
    Velox_AddMovement(world, rightFlat);
    Velox_AddBoxCollider(world, rightFlat, flatW, wallH);

    auto leftSlope = Velox_CreateEntity(world);
    Velox_AddTransform(world, leftSlope, W * 0.5f - 145.0f, H * 0.40f, 0.65f);
    Velox_AddRigidBody(world, leftSlope, 0.0f, true);
    Velox_AddMovement(world, leftSlope);
    Velox_AddBoxCollider(world, leftSlope, slopeW, wallH);

    auto rightSlope = Velox_CreateEntity(world);
    Velox_AddTransform(world, rightSlope, W * 0.5f + 145.0f, H * 0.40f, -0.65f);
    Velox_AddRigidBody(world, rightSlope, 0.0f, true);
    Velox_AddMovement(world, rightSlope);
    Velox_AddBoxCollider(world, rightSlope, slopeW, wallH);

    Velox_CreateSoftBodyBlob(world, W * 0.5f, 80.0f, 55.0f, 14, 0.04f, 0.05f, 9.0f);
}

// 15. Soft Body Stacking
static void Setup15_SoftBodyStacking(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H);
    Velox_SetGravity(world, 0.0f, 980.0f);

    auto mattress = Velox_CreateSoftBodyBlob(world, W * 0.5f, H * 0.75f, 90.0f, 20, 0.02f, 0.03f, 12.0f);

    auto box = Velox_CreateEntity(world);
    Velox_AddTransform(world, box, W * 0.48f, H * 0.35f, 0.0f);
    Velox_AddRigidBody(world, box, 12.0f, false);
    Velox_AddMovement(world, box);
    Velox_AddBoxCollider(world, box, 60.0f, 60.0f);
    Velox_AddPhysicalMaterial(world, box, 0.8f, 0.6f, 0.1f);

    auto ball = Velox_CreateEntity(world);
    Velox_AddTransform(world, ball, W * 0.53f, H * 0.15f, 0.0f);
    Velox_AddRigidBody(world, ball, 8.0f, false);
    Velox_AddMovement(world, ball);
    Velox_AddCircleCollider(world, ball, 35.0f);
    Velox_AddPhysicalMaterial(world, ball, 0.5f, 0.3f, 0.1f);
}

// 16. Buoyancy
static void Setup16_Buoyancy(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H);
    Velox_SetGravity(world, 0.0f, 850.0f);

    for (int i = 0; i < 4; ++i) {
        auto boat = Velox_CreateEntity(world);
        float bx = W * (0.2f + i * 0.18f);
        float by = H * 0.45f;
        Velox_AddTransform(world, boat, bx, by, 0.0f);
        Velox_AddRigidBody(world, boat, 2.0f, false);
        Velox_AddMovement(world, boat);
        Velox_AddBoxCollider(world, boat, 60.0f, 20.0f);
        Velox_AddPhysicalMaterial(world, boat, 0.6f, 0.4f, 0.2f);

        auto ball = Velox_CreateEntity(world);
        Velox_AddTransform(world, ball, bx + 20.0f, by - 80.0f, 0.0f);
        Velox_AddRigidBody(world, ball, 1.0f, false);
        Velox_AddMovement(world, ball);
        Velox_AddCircleCollider(world, ball, 14.0f);
        Velox_AddPhysicalMaterial(world, ball, 0.8f, 0.2f, 0.6f);
    }
}

// 17. Chaos & Destruction (300 entities)
static void Setup17_ChaosDestruction(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H);
    Velox_SetGravity(world, 0.0f, 980.0f);

    int cols = 20;
    int rows = 15;
    float blockW = 14.0f;
    float blockH = 14.0f;
    float startY = H - 50.0f;
    float startX = W * 0.45f;

    for (int r = 0; r < rows; ++r) {
        float rowY = startY - (r * (blockH + 1.5f));
        for (int c = 0; c < cols; ++c) {
            auto b = Velox_CreateEntity(world);
            float bx = startX + (c * (blockW + 1.5f));
            Velox_AddTransform(world, b, bx, rowY, 0.0f);
            Velox_AddRigidBody(world, b, 1.0f, false);
            Velox_AddMovement(world, b);
            Velox_AddBoxCollider(world, b, blockW, blockH);
            Velox_AddPhysicalMaterial(world, b, 0.5f, 0.3f, 0.1f);
        }
    }

    float anchorX = W * 0.22f;
    float anchorY = 80.0f;
    auto anchor = Velox_CreateEntity(world);
    Velox_AddTransform(world, anchor, anchorX, anchorY, 0.0f);
    Velox_AddRigidBody(world, anchor, 0.0f, true);

    Velox::EntityID prevLink = anchor;
    int numLinks = 4;
    float segLen = 45.0f;

    for (int i = 0; i < numLinks; ++i) {
        auto link = Velox_CreateEntity(world);
        float lx = anchorX + (i + 1) * 35.0f;
        float ly = anchorY + (i + 1) * 20.0f;
        Velox_AddTransform(world, link, lx, ly, 0.0f);
        Velox_AddRigidBody(world, link, 0.5f, false);
        Velox_AddMovement(world, link);
        Velox_AddCircleCollider(world, link, 6.0f);
        Velox_SetDamping(world, link, 0.2f, 0.2f);
        Velox_AddDistanceJoint(world, prevLink, link, 0.0f, 0.0f, 0.0f, 0.0f, segLen, 0.0f);
        prevLink = link;
    }

    auto wreckingBall = Velox_CreateEntity(world);
    Velox_AddTransform(world, wreckingBall, anchorX + (numLinks + 1) * 35.0f, anchorY + (numLinks + 1) * 20.0f, 0.0f);
    Velox_AddRigidBody(world, wreckingBall, 40.0f, false);
    Velox_AddMovement(world, wreckingBall);
    Velox_SetVelocity(world, wreckingBall, 300.0f, 0.0f);
    Velox_AddCircleCollider(world, wreckingBall, 30.0f);
    Velox_AddPhysicalMaterial(world, wreckingBall, 0.8f, 0.2f, 0.4f);
    Velox_AddDistanceJoint(world, prevLink, wreckingBall, 0.0f, 0.0f, 0.0f, 0.0f, segLen, 0.0f);
}

// 18. Articulated Ragdoll Network
static void Setup18_RagdollNetwork(VeloxWorld* world, float W, float H) {
    AddWalls(world, W, H);
    Velox_SetGravity(world, 0.0f, 1800.0f);

    for (int r = 0; r < 3; ++r) {
        float rx = W * (0.28f + r * 0.22f);
        float ry = H * 0.15f + r * 30.0f;

        auto head = Velox_CreateEntity(world);
        Velox_AddTransform(world, head, rx, ry, 0.0f);
        Velox_AddRigidBody(world, head, 1.0f, false);
        Velox_AddMovement(world, head);
        Velox_AddCircleCollider(world, head, 10.0f);

        auto torso = Velox_CreateEntity(world);
        Velox_AddTransform(world, torso, rx, ry + 22.0f, 0.0f);
        Velox_AddRigidBody(world, torso, 3.0f, false);
        Velox_AddMovement(world, torso);
        Velox_AddBoxCollider(world, torso, 14.0f, 24.0f);

        Velox_AddDistanceJoint(world, head, torso, 0.0f, 0.0f, 0.0f, -12.0f, 12.0f, 0.0f);
    }
}

struct SceneDescriptor {
    const char* name;
    void (*setup)(VeloxWorld*, float, float);
    int targetSteps;
};

static const SceneDescriptor g_scenes[] = {
    {"01. Bouncing Balls", Setup1_BouncingBalls, 600},
    {"02. Force Field Demo", Setup2_ForceFieldDemo, 600},
    {"03. Oscillation Demo", Setup3_OscillationDemo, 600},
    {"04. Projectile Demo", Setup4_ProjectileDemo, 600},
    {"05. Gravity Direction Demo", Setup5_GravityDemo, 600},
    {"06. Distance Joint Demo", Setup6_JointDemo, 600},
    {"07. Convex Polygons & Sandbox", Setup7_SandboxDemo, 600},
    {"08. Chain Shapes Showcase", Setup8_ChainDemo, 600},
    {"09. Raycast Queries Showcase", Setup9_RaycastDemo, 600},
    {"10. Revolute & Prismatic Showcase", Setup10_RevolutePrismatic, 600},
    {"11. CCD vs Tunneling Showcase", Setup11_CCDShowcase, 600},
    {"12. Sleeping & Activation Showcase", Setup12_SleepingShowcase, 600},
    {"13. Soft Body Sandbox (Blob vs Matched)", Setup13_SoftBodySandbox, 600},
    {"14. Soft Body Funnel & Squeeze", Setup14_SoftBodyFunnel, 600},
    {"15. Soft Body Stacking & Loads", Setup15_SoftBodyStacking, 600},
    {"16. Buoyancy & Floating Water Tank", Setup16_Buoyancy, 600},
    {"17. Chaos & Destruction 300-Body", Setup17_ChaosDestruction, 600},
    {"18. Articulated Ragdoll Network", Setup18_RagdollNetwork, 600}
};

int main() {
    std::cout << "======================================================================\n";
    std::cout << "       VELOX PHYSICS ENGINE - FLAGSHIP HEADLESS ALL-SCENE SUITE        \n";
    std::cout << "======================================================================\n";
    std::cout << "Target: 18 Scenes x 600 Frames (10.0s Sim Time @ 60Hz) = 180.0s Total\n\n";

    const float screenW = 1280.0f;
    const float screenH = 720.0f;
    const float dt = 1.0f / 60.0f;

    int passedScenes = 0;
    int failedScenes = 0;
    double totalWallTimeMs = 0.0;
    size_t initialMem = GetProcessWorkingSet();

    for (size_t sceneIdx = 0; sceneIdx < sizeof(g_scenes)/sizeof(g_scenes[0]); ++sceneIdx) {
        const auto& sc = g_scenes[sceneIdx];
        std::cout << "[" << std::setw(2) << std::setfill('0') << (sceneIdx + 1) << "/18] " 
                  << std::left << std::setw(42) << std::setfill(' ') << sc.name << " ... ";
        std::cout.flush();

        VeloxWorld* world = Velox_CreateWorld();
        sc.setup(world, screenW, screenH);

        bool sceneOk = true;
        std::string failReason;
        auto startTime = std::chrono::high_resolution_clock::now();

        for (int frame = 0; frame < sc.targetSteps; ++frame) {
            Velox_Step(world, dt);

            if (!ValidateWorldIntegrity(world, failReason)) {
                sceneOk = false;
                failReason += " (at frame " + std::to_string(frame) + "/" + std::to_string(sc.targetSteps) + ")";
                break;
            }
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        double elapsedMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        totalWallTimeMs += elapsedMs;

        Velox_DestroyWorld(world);

        if (sceneOk) {
            std::cout << "[PASS] (" << std::fixed << std::setprecision(1) << elapsedMs << " ms, "
                      << (sc.targetSteps * dt) << "s sim)\n";
            passedScenes++;
        } else {
            std::cout << "[FAIL] -> " << failReason << "\n";
            failedScenes++;
        }
    }

    size_t finalMem = GetProcessWorkingSet();
    double memDeltaMB = (finalMem >= initialMem) ? (double)(finalMem - initialMem) / (1024.0 * 1024.0) : 0.0;

    std::cout << "======================================================================\n";
    std::cout << "RESULTS: " << passedScenes << " / 18 Passed, " << failedScenes << " Failed.\n";
    std::cout << "Total Wallclock Compute: " << (totalWallTimeMs / 1000.0) << " s | Simulated Time: 180.0 s\n";
    std::cout << "Memory Working Set Delta: " << std::fixed << std::setprecision(2) << memDeltaMB << " MB\n";
    std::cout << "======================================================================\n";

    return (failedScenes == 0) ? 0 : 1;
}
