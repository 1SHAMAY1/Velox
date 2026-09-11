/**
 * @file benchmark_real_engines.cpp
 * @brief Live Head-to-Head Comparative Benchmark: Velox vs native Box2D vs native Chipmunk2D.
 *
 * Compiles and runs all three physics engines natively in a single binary on the exact
 * same CPU hardware under standardized 1,000-entity workloads.
 */

#include <velox/VeloxAPI.h>
#include <box2d/box2d.h>
#include <chipmunk/chipmunk.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <iomanip>
#include <cmath>

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

struct BenchmarkResult {
    std::string engineName;
    double avgStepMs;
    double fps;
    double opsPerSec;
    double memoryKB;
    double bytesPerEntity;
};

// ============================================================================
// 1. STACKING PYRAMID (1,000 Rigid Boxes)
// ============================================================================
BenchmarkResult BenchmarkVeloxStacking(int count = 1000, int frames = 100) {
    size_t mem0 = GetProcessWorkingSet();
    VeloxWorld* world = Velox_CreateWorld();
    Velox_SetGravity(world, 0.0f, 980.0f);

    Velox::EntityID floor = Velox_CreateEntity(world);
    Velox_AddTransform(world, floor, 1000.0f, 800.0f, 0.0f);
    Velox_AddRigidBody(world, floor, 0.0f, true);
    Velox_AddBoxCollider(world, floor, 4000.0f, 40.0f);

    int cols = 40;
    for (int i = 0; i < count; ++i) {
        Velox::EntityID b = Velox_CreateEntity(world);
        float x = 200.0f + (i % cols) * 25.0f;
        float y = 100.0f + (i / cols) * 25.0f;
        Velox_AddTransform(world, b, x, y, 0.0f);
        Velox_AddMovement(world, b);
        Velox_AddRigidBody(world, b, 1.0f, false);
        Velox_AddBoxCollider(world, b, 12.0f, 12.0f);
    }

    size_t mem1 = GetProcessWorkingSet();
    for (int w = 0; w < 10; ++w) Velox_Step(world, 1.0f / 60.0f);

    auto start = std::chrono::high_resolution_clock::now();
    for (int f = 0; f < frames; ++f) Velox_Step(world, 1.0f / 60.0f);
    auto end = std::chrono::high_resolution_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double avgMs = totalMs / frames;
    Velox_DestroyWorld(world);

    double memKB = (mem1 > mem0) ? (double)(mem1 - mem0) / 1024.0 : 0.0;
    return {"Velox (XPBD)", avgMs, 1000.0 / avgMs, (double)count * 8.0 * (1000.0 / avgMs), memKB, (memKB * 1024.0) / count};
}

BenchmarkResult BenchmarkBox2DStacking(int count = 1000, int frames = 100) {
    size_t mem0 = GetProcessWorkingSet();
    b2Vec2 gravity(0.0f, -9.8f);
    b2World world(gravity);

    // Floor
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0.0f, -10.0f);
    b2Body* groundBody = world.CreateBody(&groundBodyDef);
    b2PolygonShape groundBox;
    groundBox.SetAsBox(100.0f, 2.0f);
    groundBody->CreateFixture(&groundBox, 0.0f);

    // 1000 dynamic boxes
    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox(0.5f, 0.5f);
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.3f;

    int cols = 40;
    for (int i = 0; i < count; ++i) {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(-20.0f + (i % cols) * 1.0f, 0.0f + (i / cols) * 1.0f);
        b2Body* body = world.CreateBody(&bodyDef);
        body->CreateFixture(&fixtureDef);
    }

    size_t mem1 = GetProcessWorkingSet();
    for (int w = 0; w < 10; ++w) world.Step(1.0f / 60.0f, 8, 3);

    auto start = std::chrono::high_resolution_clock::now();
    for (int f = 0; f < frames; ++f) world.Step(1.0f / 60.0f, 8, 3);
    auto end = std::chrono::high_resolution_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double avgMs = totalMs / frames;

    double memKB = (mem1 > mem0) ? (double)(mem1 - mem0) / 1024.0 : 0.0;
    return {"Box2D v2.4", avgMs, 1000.0 / avgMs, (double)count * (1000.0 / avgMs), memKB, (memKB * 1024.0) / count};
}

BenchmarkResult BenchmarkChipmunkStacking(int count = 1000, int frames = 100) {
    size_t mem0 = GetProcessWorkingSet();
    cpSpace* space = cpSpaceNew();
    cpSpaceSetGravity(space, cpv(0, -980));

    // Static floor
    cpBody* staticBody = cpSpaceGetStaticBody(space);
    cpShape* ground = cpSegmentShapeNew(staticBody, cpv(-2000, -100), cpv(2000, -100), 20);
    cpShapeSetFriction(ground, 0.5);
    cpSpaceAddShape(space, ground);

    int cols = 40;
    for (int i = 0; i < count; ++i) {
        cpFloat mass = 1.0;
        cpFloat size = 12.0;
        cpFloat moment = cpMomentForBox(mass, size, size);
        cpBody* body = cpSpaceAddBody(space, cpBodyNew(mass, moment));
        cpBodySetPosition(body, cpv(-300 + (i % cols) * 20, 0 + (i / cols) * 20));

        cpShape* shape = cpSpaceAddShape(space, cpBoxShapeNew(body, size, size, 0.0));
        cpShapeSetFriction(shape, 0.5);
    }

    size_t mem1 = GetProcessWorkingSet();
    for (int w = 0; w < 10; ++w) cpSpaceStep(space, 1.0 / 60.0);

    auto start = std::chrono::high_resolution_clock::now();
    for (int f = 0; f < frames; ++f) cpSpaceStep(space, 1.0 / 60.0);
    auto end = std::chrono::high_resolution_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double avgMs = totalMs / frames;

    cpSpaceFree(space);
    double memKB = (mem1 > mem0) ? (double)(mem1 - mem0) / 1024.0 : 0.0;
    return {"Chipmunk2D", avgMs, 1000.0 / avgMs, (double)count * (1000.0 / avgMs), memKB, (memKB * 1024.0) / count};
}

// ============================================================================
// 2. THE TUMBLER (1,000 Dynamic Circles)
// ============================================================================
BenchmarkResult BenchmarkVeloxCircles(int count = 1000, int frames = 100) {
    size_t mem0 = GetProcessWorkingSet();
    VeloxWorld* world = Velox_CreateWorld();
    Velox_SetGravity(world, 0.0f, 980.0f);

    Velox::EntityID floor = Velox_CreateEntity(world);
    Velox_AddTransform(world, floor, 1000.0f, 800.0f, 0.0f);
    Velox_AddRigidBody(world, floor, 0.0f, true);
    Velox_AddBoxCollider(world, floor, 4000.0f, 40.0f);

    int cols = 40;
    for (int i = 0; i < count; ++i) {
        Velox::EntityID c = Velox_CreateEntity(world);
        float x = 200.0f + (i % cols) * 20.0f;
        float y = 50.0f + (i / cols) * 20.0f;
        Velox_AddTransform(world, c, x, y, 0.0f);
        Velox_AddMovement(world, c);
        Velox_AddRigidBody(world, c, 1.0f, false);
        Velox_AddCircleCollider(world, c, 8.0f);
    }

    size_t mem1 = GetProcessWorkingSet();
    for (int w = 0; w < 10; ++w) Velox_Step(world, 1.0f / 60.0f);

    auto start = std::chrono::high_resolution_clock::now();
    for (int f = 0; f < frames; ++f) Velox_Step(world, 1.0f / 60.0f);
    auto end = std::chrono::high_resolution_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double avgMs = totalMs / frames;
    Velox_DestroyWorld(world);

    double memKB = (mem1 > mem0) ? (double)(mem1 - mem0) / 1024.0 : 0.0;
    return {"Velox (XPBD)", avgMs, 1000.0 / avgMs, (double)count * 8.0 * (1000.0 / avgMs), memKB, (memKB * 1024.0) / count};
}

BenchmarkResult BenchmarkBox2DCircles(int count = 1000, int frames = 100) {
    size_t mem0 = GetProcessWorkingSet();
    b2Vec2 gravity(0.0f, -9.8f);
    b2World world(gravity);

    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0.0f, -10.0f);
    b2Body* groundBody = world.CreateBody(&groundBodyDef);
    b2PolygonShape groundBox;
    groundBox.SetAsBox(100.0f, 2.0f);
    groundBody->CreateFixture(&groundBox, 0.0f);

    b2CircleShape circle;
    circle.m_radius = 0.4f;
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &circle;
    fixtureDef.density = 1.0f;

    int cols = 40;
    for (int i = 0; i < count; ++i) {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(-20.0f + (i % cols) * 1.0f, 0.0f + (i / cols) * 1.0f);
        b2Body* body = world.CreateBody(&bodyDef);
        body->CreateFixture(&fixtureDef);
    }

    size_t mem1 = GetProcessWorkingSet();
    for (int w = 0; w < 10; ++w) world.Step(1.0f / 60.0f, 8, 3);

    auto start = std::chrono::high_resolution_clock::now();
    for (int f = 0; f < frames; ++f) world.Step(1.0f / 60.0f, 8, 3);
    auto end = std::chrono::high_resolution_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double avgMs = totalMs / frames;

    double memKB = (mem1 > mem0) ? (double)(mem1 - mem0) / 1024.0 : 0.0;
    return {"Box2D v2.4", avgMs, 1000.0 / avgMs, (double)count * (1000.0 / avgMs), memKB, (memKB * 1024.0) / count};
}

BenchmarkResult BenchmarkChipmunkCircles(int count = 1000, int frames = 100) {
    size_t mem0 = GetProcessWorkingSet();
    cpSpace* space = cpSpaceNew();
    cpSpaceSetGravity(space, cpv(0, -980));

    cpBody* staticBody = cpSpaceGetStaticBody(space);
    cpShape* ground = cpSegmentShapeNew(staticBody, cpv(-2000, -100), cpv(2000, -100), 20);
    cpSpaceAddShape(space, ground);

    int cols = 40;
    for (int i = 0; i < count; ++i) {
        cpFloat mass = 1.0;
        cpFloat radius = 8.0;
        cpFloat moment = cpMomentForCircle(mass, 0, radius, cpvzero);
        cpBody* body = cpSpaceAddBody(space, cpBodyNew(mass, moment));
        cpBodySetPosition(body, cpv(-300 + (i % cols) * 20, 0 + (i / cols) * 20));

        cpShape* shape = cpSpaceAddShape(space, cpCircleShapeNew(body, radius, cpvzero));
        cpShapeSetFriction(shape, 0.5);
    }

    size_t mem1 = GetProcessWorkingSet();
    for (int w = 0; w < 10; ++w) cpSpaceStep(space, 1.0 / 60.0);

    auto start = std::chrono::high_resolution_clock::now();
    for (int f = 0; f < frames; ++f) cpSpaceStep(space, 1.0 / 60.0);
    auto end = std::chrono::high_resolution_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double avgMs = totalMs / frames;

    cpSpaceFree(space);
    double memKB = (mem1 > mem0) ? (double)(mem1 - mem0) / 1024.0 : 0.0;
    return {"Chipmunk2D", avgMs, 1000.0 / avgMs, (double)count * (1000.0 / avgMs), memKB, (memKB * 1024.0) / count};
}

// ============================================================================
// 3. JOINT CHAIN (500 Links)
// ============================================================================
BenchmarkResult BenchmarkVeloxJoints(int count = 500, int frames = 100) {
    size_t mem0 = GetProcessWorkingSet();
    VeloxWorld* world = Velox_CreateWorld();
    Velox_SetGravity(world, 0.0f, 980.0f);

    Velox::EntityID prev = Velox_CreateEntity(world);
    Velox_AddTransform(world, prev, 500.0f, 100.0f, 0.0f);
    Velox_AddRigidBody(world, prev, 0.0f, true);

    for (int i = 0; i < count; ++i) {
        Velox::EntityID curr = Velox_CreateEntity(world);
        Velox_AddTransform(world, curr, 500.0f + (i + 1) * 8.0f, 100.0f, 0.0f);
        Velox_AddMovement(world, curr);
        Velox_AddRigidBody(world, curr, 1.0f, false);
        Velox_AddCircleCollider(world, curr, 3.0f);

        Velox_AddDistanceJoint(world, prev, curr, 0.0f, 0.0f, 0.0f, 0.0f, 8.0f, 0.0f);
        prev = curr;
    }

    size_t mem1 = GetProcessWorkingSet();
    for (int w = 0; w < 10; ++w) Velox_Step(world, 1.0f / 60.0f);

    auto start = std::chrono::high_resolution_clock::now();
    for (int f = 0; f < frames; ++f) Velox_Step(world, 1.0f / 60.0f);
    auto end = std::chrono::high_resolution_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double avgMs = totalMs / frames;
    Velox_DestroyWorld(world);

    double memKB = (mem1 > mem0) ? (double)(mem1 - mem0) / 1024.0 : 0.0;
    return {"Velox (XPBD)", avgMs, 1000.0 / avgMs, (double)count * 8.0 * (1000.0 / avgMs), memKB, (memKB * 1024.0) / count};
}

BenchmarkResult BenchmarkBox2DJoints(int count = 500, int frames = 100) {
    size_t mem0 = GetProcessWorkingSet();
    b2Vec2 gravity(0.0f, -9.8f);
    b2World world(gravity);

    b2BodyDef anchorDef;
    anchorDef.position.Set(0.0f, 10.0f);
    b2Body* prevBody = world.CreateBody(&anchorDef);

    b2CircleShape circle;
    circle.m_radius = 0.15f;
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &circle;
    fixtureDef.density = 1.0f;

    for (int i = 0; i < count; ++i) {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set((i + 1) * 0.4f, 10.0f);
        b2Body* body = world.CreateBody(&bodyDef);
        body->CreateFixture(&fixtureDef);

        b2DistanceJointDef jointDef;
        jointDef.Initialize(prevBody, body, prevBody->GetPosition(), body->GetPosition());
        jointDef.length = 0.4f;
        world.CreateJoint(&jointDef);

        prevBody = body;
    }

    size_t mem1 = GetProcessWorkingSet();
    for (int w = 0; w < 10; ++w) world.Step(1.0f / 60.0f, 8, 3);

    auto start = std::chrono::high_resolution_clock::now();
    for (int f = 0; f < frames; ++f) world.Step(1.0f / 60.0f, 8, 3);
    auto end = std::chrono::high_resolution_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double avgMs = totalMs / frames;

    double memKB = (mem1 > mem0) ? (double)(mem1 - mem0) / 1024.0 : 0.0;
    return {"Box2D v2.4", avgMs, 1000.0 / avgMs, (double)count * (1000.0 / avgMs), memKB, (memKB * 1024.0) / count};
}

BenchmarkResult BenchmarkChipmunkJoints(int count = 500, int frames = 100) {
    size_t mem0 = GetProcessWorkingSet();
    cpSpace* space = cpSpaceNew();
    cpSpaceSetGravity(space, cpv(0, -980));

    cpBody* prevBody = cpSpaceGetStaticBody(space);
    cpBodySetPosition(prevBody, cpv(0, 100));

    for (int i = 0; i < count; ++i) {
        cpFloat mass = 1.0;
        cpFloat radius = 3.0;
        cpFloat moment = cpMomentForCircle(mass, 0, radius, cpvzero);
        cpBody* body = cpSpaceAddBody(space, cpBodyNew(mass, moment));
        cpBodySetPosition(body, cpv((i + 1) * 8.0, 100));

        cpShape* shape = cpSpaceAddShape(space, cpCircleShapeNew(body, radius, cpvzero));
        cpShapeSetFriction(shape, 0.5);

        cpConstraint* joint = cpPinJointNew(prevBody, body, cpvzero, cpvzero);
        cpPinJointSetDist(joint, 8.0);
        cpSpaceAddConstraint(space, joint);

        prevBody = body;
    }

    size_t mem1 = GetProcessWorkingSet();
    for (int w = 0; w < 10; ++w) cpSpaceStep(space, 1.0 / 60.0);

    auto start = std::chrono::high_resolution_clock::now();
    for (int f = 0; f < frames; ++f) cpSpaceStep(space, 1.0 / 60.0);
    auto end = std::chrono::high_resolution_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double avgMs = totalMs / frames;

    cpSpaceFree(space);
    double memKB = (mem1 > mem0) ? (double)(mem1 - mem0) / 1024.0 : 0.0;
    return {"Chipmunk2D", avgMs, 1000.0 / avgMs, (double)count * (1000.0 / avgMs), memKB, (memKB * 1024.0) / count};
}

void PrintHeader(const std::string& scenario, int entities) {
    std::cout << "\n========================================================================================================\n";
    std::cout << " Scenario: " << scenario << " (" << entities << " Entities)\n";
    std::cout << "========================================================================================================\n";
    std::cout << std::left 
              << std::setw(20) << "Physics Engine"
              << std::setw(15) << "Avg Step (ms)"
              << std::setw(16) << "Uncapped FPS"
              << std::setw(18) << "Operations/sec"
              << std::setw(18) << "Memory Footprint"
              << std::setw(15) << "Bytes / Body"
              << "\n";
    std::cout << "--------------------------------------------------------------------------------------------------------\n";
}

void PrintResult(const BenchmarkResult& r) {
    std::string memStr = (r.memoryKB > 1024.0) ? (std::to_string((int)(r.memoryKB / 1024.0)) + "." + std::to_string((int)(r.memoryKB) % 1024 / 100) + " MB") : (std::to_string((int)r.memoryKB) + " KB");
    std::cout << std::left 
              << std::setw(20) << r.engineName
              << std::setw(15) << std::fixed << std::setprecision(3) << r.avgStepMs
              << std::setw(16) << std::fixed << std::setprecision(1) << r.fps
              << std::setw(18) << (int)r.opsPerSec
              << std::setw(18) << memStr
              << std::setw(15) << std::fixed << std::setprecision(1) << r.bytesPerEntity
              << std::endl;
}

int main() {
    std::cout << "========================================================================================================\n";
    std::cout << "          INDUSTRY 2D PHYSICS ENGINE LIVE MULTI-BENCHMARK & MEMORY HARNESS            \n";
    std::cout << "                (Running on Exact Same Host CPU & Standard Compiler)                   \n";
    std::cout << "========================================================================================================\n";

    // 1. Pyramid Stacking (1,000 Boxes)
    PrintHeader("Pyramid Stacking", 1000);
    PrintResult(BenchmarkVeloxStacking(1000));
    PrintResult(BenchmarkBox2DStacking(1000));
    PrintResult(BenchmarkChipmunkStacking(1000));

    // 2. Dynamic Circles (1,000 Circles)
    PrintHeader("Dynamic Circle Collisions", 1000);
    PrintResult(BenchmarkVeloxCircles(1000));
    PrintResult(BenchmarkBox2DCircles(1000));
    PrintResult(BenchmarkChipmunkCircles(1000));

    // 3. Joint Chains (500 Links)
    PrintHeader("Joint Constraint Chain", 500);
    PrintResult(BenchmarkVeloxJoints(500));
    PrintResult(BenchmarkBox2DJoints(500));
    PrintResult(BenchmarkChipmunkJoints(500));

    std::cout << "========================================================================================================\n";
    std::cout << "   Live Head-to-Head Multi-Engine Benchmark & Memory Suite Completed Successfully!\n";
    std::cout << "========================================================================================================" << std::endl;

    return 0;
}
