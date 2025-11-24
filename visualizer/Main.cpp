#define NOGDI
#define NOUSER
#define NOMINMAX
#include <windows.h>
#include <psapi.h>

// Undefine Windows macros that conflict with Raylib
#undef DrawText
#undef CloseWindow
#undef ShowCursor
#undef Rectangle
#undef PlaySound
#undef LoadImage
#undef DrawTextEx

#include "raylib.h"
#include "raymath.h"
#include <velox/PhysicsEngineAPI.h>
#include <vector>
#include <iostream>
#include <string>
#include <deque>

// Simple struct to track visual entities
struct VisualEntity {
    Velox::EntityID id;
    Color color;
    float radius; // For spheres
    Vector3 size; // For boxes
    int type; // 0 = Sphere, 1 = Box
};

struct VisualSpring {
    Velox::EntityID a;
    Velox::EntityID b;
};

int main() {
    // Initialization
    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "Velox Physics Engine - Visualizer");

    // Load Window Icon
    Image icon = LoadImage("assets/velox_icon_window.png");
    SetWindowIcon(icon);
    UnloadImage(icon);

    // Camera
    Camera3D camera = { 0 };
    camera.position = Vector3{ 10.0f, 10.0f, 10.0f };
    camera.target = Vector3{ 0.0f, 0.0f, 0.0f };
    camera.up = Vector3{ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SetTargetFPS(60);

    // Physics World
    VeloxWorld* world = Velox_CreateWorld();
    std::vector<VisualEntity> entities;
    std::vector<VisualSpring> springs;

    // Meshes for Instancing
    Mesh sphereMesh = GenMeshSphere(1.0f, 8, 8); // Low poly for performance
    Mesh cubeMesh = GenMeshCube(1.0f, 1.0f, 1.0f);
    Material defaultMat = LoadMaterialDefault();
    
    // Instance Buffers
    const int MAX_INSTANCES = 20000;
    std::vector<Matrix> sphereTransforms;
    std::vector<Matrix> cubeTransforms;
    std::vector<Color> sphereColors; // Raylib Instanced doesn't support per-instance color easily without shader.
    // For now, we will just use transforms and a single color per batch, or multiple batches.
    // Actually, DrawMeshInstanced doesn't support per-instance colors by default.
    // To keep it simple and fast, we'll group by color or just accept uniform color for stress test.
    // Let's stick to simple drawing for small counts, and Instanced for Stress Test (uniform color).
    
    // Actually, for 10k objects, we really need instancing. 
    // Let's use a custom shader or just accept they are all the same color in "Stress Mode".
    // Or we can just use DrawSphere (which is slow) and prove it with a toggle.
    // Let's add the toggle first, it's safer than writing a shader right now.
    
    // Create Ground
    {
        auto id = Velox_CreateEntity(world);
        Velox_AddTransform(world, id, 0, -1.0f, 0);
        Velox_AddRigidBody(world, id, 0.0f, true);
        
        VisualEntity ve;
        ve.id = id;
        ve.color = GRAY;
        ve.type = 1; // Box
        ve.size = Vector3{ 20.0f, 2.0f, 20.0f };
        entities.push_back(ve);
    }

    // UI State
    bool showUI = true;
    bool enableRendering = true;
    bool enableWires = false; // Wires are slow
    int targetFPS = 60;
    bool spawnAsProjectile = false;
    bool spawnAsSoft = false;
    int softBodyCount = 0;
    int rigidBodyCount = 0;
    float packetLossChance = 0.0f; // 0.0 to 1.0
    float networkLag = 0.0f; // Seconds
    std::deque<float> frameTimes;
    std::deque<float> memoryUsage;
    const int maxFrameHistory = 100;

    // Main game loop
    while (!WindowShouldClose()) {
        // Input Handling
        if (IsKeyPressed(KEY_GRAVE)) showUI = !showUI;
        
        // Spawn Logic
        Vector3 spawnPos = { 0, 15, 0 };
        Vector3 spawnVel = { 0, 0, 0 };

        if (spawnAsProjectile) {
             spawnPos = camera.position;
             Ray ray = GetMouseRay(GetMousePosition(), camera);
             Vector3 dir = Vector3Normalize(Vector3Subtract(ray.direction, Vector3Zero()));
             spawnVel = Vector3Scale(dir, 20.0f);
        }

        auto SpawnCube = [&](bool soft) {
            if (soft) {
                softBodyCount++;
                float size = 2.0f;
                std::vector<Velox::EntityID> ids;
                for (int x=0; x<2; x++) {
                    for (int y=0; y<2; y++) {
                        for (int z=0; z<2; z++) {
                            auto id = Velox_CreateEntity(world);
                            Velox_AddTransform(world, id, spawnPos.x + x*size, spawnPos.y + y*size, spawnPos.z + z*size);
                            Velox_AddRigidBody(world, id, 1.0f, false);
                            Velox_AddCollider(world, id, 0, 0.3f);
                            Velox_SetVelocity(world, id, spawnVel.x, spawnVel.y, spawnVel.z);
                            
                            VisualEntity ve; ve.id = id; ve.color = BLUE; ve.type = 0; ve.radius = 0.3f;
                            entities.push_back(ve);
                            ids.push_back(id);
                        }
                    }
                }
                for (size_t i=0; i<ids.size(); ++i) {
                    for (size_t j=i+1; j<ids.size(); ++j) {
                        Velox_AddSpring(world, ids[i], ids[j], 50.0f, 0.5f);
                        springs.push_back({ids[i], ids[j]});
                    }
                }
            } else {
                rigidBodyCount++;
                // Hard Cube
                auto id = Velox_CreateEntity(world);
                Velox_AddTransform(world, id, spawnPos.x, spawnPos.y, spawnPos.z);
                Velox_AddRigidBody(world, id, 5.0f, false);
                Velox_SetVelocity(world, id, spawnVel.x, spawnVel.y, spawnVel.z);
                Velox_AddCollider(world, id, 0, 1.0f); // Sphere Collider approximation
                
                VisualEntity ve; ve.id = id; ve.color = BLUE; ve.type = 1; ve.size = {2,2,2};
                entities.push_back(ve);
            }
        };

        auto SpawnTetra = [&](bool soft) {
            if (soft) {
                softBodyCount++;
                float size = 3.0f;
                std::vector<Velox::EntityID> ids;
                Vector3 points[] = {{0, 1, 0}, {-1, -1, 1}, {1, -1, 1}, {0, -1, -1}};
                for (int i=0; i<4; ++i) {
                    auto id = Velox_CreateEntity(world);
                    Velox_AddTransform(world, id, spawnPos.x + points[i].x*size, spawnPos.y + points[i].y*size, spawnPos.z + points[i].z*size);
                    Velox_AddRigidBody(world, id, 1.0f, false);
                    Velox_AddCollider(world, id, 0, 0.3f);
                    Velox_SetVelocity(world, id, spawnVel.x, spawnVel.y, spawnVel.z);
                    VisualEntity ve; ve.id = id; ve.color = RED; ve.type = 0; ve.radius = 0.3f;
                    entities.push_back(ve);
                    ids.push_back(id);
                }
                for (size_t i=0; i<ids.size(); ++i) {
                    for (size_t j=i+1; j<ids.size(); ++j) {
                        Velox_AddSpring(world, ids[i], ids[j], 80.0f, 0.5f);
                        springs.push_back({ids[i], ids[j]});
                    }
                }
            } else {
                rigidBodyCount++;
                // Hard Tetrahedron
                auto id = Velox_CreateEntity(world);
                Velox_AddTransform(world, id, spawnPos.x, spawnPos.y, spawnPos.z);
                Velox_AddRigidBody(world, id, 2.0f, false);
                Velox_SetVelocity(world, id, spawnVel.x, spawnVel.y, spawnVel.z);
                Velox_AddCollider(world, id, 0, 1.0f); // Sphere Collider approximation
                
                VisualEntity ve; ve.id = id; ve.color = RED; ve.type = 2; // 2 = Tetra
                ve.radius = 1.0f; 
                entities.push_back(ve);
            }
        };

        auto SpawnSphere = [&](Vector3 pos, Vector3 vel) {
            rigidBodyCount++;
            auto id = Velox_CreateEntity(world);
            Velox_AddTransform(world, id, pos.x, pos.y, pos.z);
            Velox_AddRigidBody(world, id, 1.0f, false);
            Velox_AddCollider(world, id, 0, 0.5f);
            Velox_SetVelocity(world, id, vel.x, vel.y, vel.z);
            
            VisualEntity ve; ve.id = id; ve.color = GREEN; ve.type = 0; ve.radius = 0.5f;
            entities.push_back(ve);
        };

        if (IsKeyPressed(KEY_ONE)) SpawnCube(spawnAsSoft);
        if (IsKeyPressed(KEY_TWO)) SpawnTetra(spawnAsSoft);
        
        // Stress Test Spawner
        if (IsKeyPressed(KEY_S)) {
            for (int i = 0; i < 100; ++i) {
                // Random Position
                spawnPos = Vector3{ (float)GetRandomValue(-20, 20), (float)GetRandomValue(20, 100), (float)GetRandomValue(-20, 20) };
                
                // Random Velocity (Some are projectiles)
                bool isProjectile = GetRandomValue(0, 10) > 8; // 20% chance
                if (isProjectile) {
                    spawnVel = Vector3{ (float)GetRandomValue(-50, 50), (float)GetRandomValue(-50, 0), (float)GetRandomValue(-50, 50) };
                } else {
                    spawnVel = Vector3{ 0, 0, 0 };
                }

                int type = GetRandomValue(0, 4);
                // 0: Soft Cube, 1: Soft Tetra, 2: Rigid Cube, 3: Rigid Tetra, 4: Projectile (Sphere)
                
                if (isProjectile) {
                    // Projectiles can be anything now!
                    int projType = GetRandomValue(0, 4); 
                    // 0: Sphere (Rigid), 1: Soft Cube, 2: Rigid Cube, 3: Soft Tetra, 4: Rigid Tetra
                    
                    switch (projType) {
                        case 0: SpawnSphere(spawnPos, spawnVel); break;
                        case 1: spawnAsSoft = true; SpawnCube(true); break;
                        case 2: spawnAsSoft = false; SpawnCube(false); break;
                        case 3: spawnAsSoft = true; SpawnTetra(true); break;
                        case 4: spawnAsSoft = false; SpawnTetra(false); break;
                    }
                } else {
                    switch (type) {
                        case 0: spawnAsSoft = true; SpawnCube(true); break;
                        case 1: spawnAsSoft = true; SpawnTetra(true); break;
                        case 2: spawnAsSoft = false; SpawnCube(false); break;
                        case 3: spawnAsSoft = false; SpawnTetra(false); break;
                        default: spawnAsSoft = false; SpawnCube(false); break;
                    }
                }
            }
        }
        
        // Update Physics (Simulate Network Lag/Loss)
        float dt = 1.0f / targetFPS;
        
        // Simple Packet Loss Simulation
        if (GetRandomValue(0, 100) / 100.0f > packetLossChance) {
             Velox_Step(world, dt);
        }

        // Update Camera
        UpdateCamera(&camera, CAMERA_ORBITAL);

        // Profiling
        frameTimes.push_back(GetFrameTime());
        if (frameTimes.size() > maxFrameHistory) frameTimes.pop_front();

        // Draw
        BeginDrawing();
            ClearBackground(Color{20, 20, 20, 255}); // Dark Mode

            if (enableRendering) {
                BeginMode3D(camera);
                    DrawGrid(20, 1.0f);

                    // Draw Entities
                    for (const auto& ve : entities) {
                        float x, y, z;
                        Velox_GetPosition(world, ve.id, &x, &y, &z);
                        Vector3 pos = { x, y, z };

                        if (ve.type == 0) {
                            DrawSphere(pos, ve.radius, ve.color);
                            if (enableWires) DrawSphereWires(pos, ve.radius, 8, 8, Fade(BLACK, 0.3f));
                        } else if (ve.type == 1) {
                            DrawCube(pos, ve.size.x, ve.size.y, ve.size.z, ve.color);
                            if (enableWires) DrawCubeWires(pos, ve.size.x, ve.size.y, ve.size.z, Fade(BLACK, 0.3f));
                        } else if (ve.type == 2) {
                            // Draw Tetrahedron (Pyramid)
                            DrawCylinder(pos, 0.0f, ve.radius, ve.radius * 1.5f, 4, ve.color);
                            if (enableWires) DrawCylinderWires(pos, 0.0f, ve.radius, ve.radius * 1.5f, 4, Fade(BLACK, 0.3f));
                        }
                    }
                    
                    // Draw Springs (Expensive for 10k)
                    if (enableWires) {
                        for (const auto& s : springs) {
                            float x1, y1, z1, x2, y2, z2;
                            Velox_GetPosition(world, s.a, &x1, &y1, &z1);
                            Velox_GetPosition(world, s.b, &x2, &y2, &z2);
                            DrawLine3D(Vector3{x1, y1, z1}, Vector3{x2, y2, z2}, Fade(BLACK, 0.5f));
                        }
                    }

                EndMode3D();
            } else {
                DrawText("RENDERING DISABLED (Physics Only)", screenWidth/2 - 150, screenHeight/2, 20, RED);
            }

            // UI Overlay
            if (showUI) {
                // Background
                DrawRectangle(10, 10, 320, 650, Fade(BLACK, 0.8f));
                DrawRectangleLines(10, 10, 320, 650, DARKBLUE);
                
                int y = 20;
                DrawText("Velox Physics - Debug UI", 20, y, 20, SKYBLUE); y += 30;
                DrawText(TextFormat("FPS: %i", GetFPS()), 20, y, 20, GREEN); y += 20;
                DrawText(TextFormat("Entities: %i", entities.size()), 20, y, 20, WHITE); y += 20;
                DrawText(TextFormat("Soft Bodies: %i", softBodyCount), 20, y, 20, PURPLE); y += 20;
                DrawText(TextFormat("Rigid Bodies: %i", rigidBodyCount), 20, y, 20, ORANGE); y += 20;
                
                // FPS Control
                DrawText(TextFormat("Target FPS: %i", targetFPS), 20, y, 20, WHITE); y += 25;
                DrawRectangle(20, y, 200, 20, GRAY);
                DrawRectangle(20, y, (int)((targetFPS - 10) / 110.0f * 200), 20, BLUE); // Slider visual
                if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                    Vector2 mouse = GetMousePosition();
                    if (mouse.x >= 20 && mouse.x <= 220 && mouse.y >= y && mouse.y <= y + 20) {
                        float ratio = (mouse.x - 20) / 200.0f;
                        targetFPS = 10 + (int)(ratio * 110); // 10 to 120 FPS
                        SetTargetFPS(targetFPS);
                    }
                }
                y += 30;

                // Controls
                DrawText("Controls:", 20, y, 10, LIGHTGRAY); y += 15;
                DrawText("1: Spawn Cube", 20, y, 10, WHITE); y += 15;
                DrawText("2: Spawn Tetrahedron", 20, y, 10, WHITE); y += 15;
                DrawText("S: Stress Test (Spawn Batch)", 20, y, 10, ORANGE); y += 15;
                DrawText("R: Toggle Rendering", 20, y, 10, PINK); y += 15;
                DrawText("F: Toggle Wireframes/Springs", 20, y, 10, PINK); y += 20;
                
                if (IsKeyPressed(KEY_R)) enableRendering = !enableRendering;
                if (IsKeyPressed(KEY_F)) enableWires = !enableWires;

                // Toggles
                if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                    Vector2 m = GetMousePosition();
                    if (CheckCollisionPointRec(m, Rectangle{20, (float)y, 150, 20})) spawnAsProjectile = !spawnAsProjectile;
                    if (CheckCollisionPointRec(m, Rectangle{20, (float)y+25, 150, 20})) spawnAsSoft = !spawnAsSoft;
                }
                
                DrawRectangleLines(20, y, 15, 15, WHITE);
                if (spawnAsProjectile) DrawRectangle(22, y+2, 11, 11, GREEN);
                DrawText("Projectile Mode", 45, y, 10, WHITE); y += 25;

                DrawRectangleLines(20, y, 15, 15, WHITE);
                if (spawnAsSoft) DrawRectangle(22, y+2, 11, 11, GREEN);
                DrawText("Soft Body Mode", 45, y, 10, WHITE); y += 25;
                
                // Network Sim
                DrawText("Network Simulation:", 20, y, 10, LIGHTGRAY); y += 15;
                DrawText(TextFormat("Packet Loss: %.2f%%", packetLossChance * 100), 20, y, 10, RED); y += 15;
                
                if (IsKeyDown(KEY_UP)) packetLossChance += 0.005f;
                if (IsKeyDown(KEY_DOWN)) packetLossChance -= 0.005f;
                if (packetLossChance < 0) packetLossChance = 0;
                if (packetLossChance > 1) packetLossChance = 1;
                DrawText("(UP/DOWN arrows to adjust)", 20, y, 10, DARKGRAY); y += 25;

                // Performance Graph
                DrawText("Frame Time (ms):", 20, y, 10, LIGHTGRAY); y += 15;
                for (size_t i = 1; i < frameTimes.size(); ++i) {
                    DrawLine(20 + (i-1)*2, y + 50 - frameTimes[i-1]*1000, 20 + i*2, y + 50 - frameTimes[i]*1000, GREEN);
                }
                y += 60;

                // Memory Graph
                PROCESS_MEMORY_COUNTERS_EX pmc;
                GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc));
                float memMB = pmc.PrivateUsage / (1024.0f * 1024.0f);
                memoryUsage.push_back(memMB);
                if (memoryUsage.size() > maxFrameHistory) memoryUsage.pop_front();

                DrawText(TextFormat("Memory: %.2f MB", memMB), 20, y, 10, LIGHTGRAY); y += 15;
                float maxMem = 100.0f; // Baseline for graph scaling
                for (size_t i = 1; i < memoryUsage.size(); ++i) {
                    float h1 = (memoryUsage[i-1] / maxMem) * 50; 
                    float h2 = (memoryUsage[i] / maxMem) * 50;
                    DrawLine(20 + (i-1)*2, y + 50 - h1, 20 + i*2, y + 50 - h2, YELLOW);
                }
            }

        EndDrawing();
    }

    // De-Initialization
    Velox_DestroyWorld(world);
    CloseWindow();

    return 0;
}
