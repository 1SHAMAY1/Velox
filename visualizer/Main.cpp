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
#include <velox/VeloxAPI.h>
#include <vector>
#include <iostream>
#include <string>
#include <deque>

struct VisualEntity {
    Velox::EntityID id;
    Color color;
    float radius; // For circles
    float width, height; // For boxes
    int type; // 0 = Circle, 1 = Box, 2 = ForceField (visual only)
    float spawnTime; // Time when entity was spawned
    bool isProjectile = false; // Flag for projectile behavior cleanup
    const char* label; // Label for force fields
};

// --- Scene Management ---
enum class SceneType {
    BouncingBalls,
    ForceFieldDemo,
    OscillationDemo,
    ProjectileDemo
};

SceneType currentScene = SceneType::BouncingBalls;
bool sceneChanged = true;

// UI State
bool isDropdownOpen = false;
int selectedItem = 0;
const char* sceneNames[] = { "Bouncing Balls", "Force Field Demo", "Oscillation Demo", "Projectile Demo" };

void SetupBouncingBalls(VeloxWorld* world, std::vector<VisualEntity>& entities, int screenWidth, int screenHeight) {
    float wallThickness = 20.0f;
    
    // 1. Walls (Visible Frame)
    struct WallDef { float x, y, w, h; };
    WallDef walls[] = {
        {screenWidth/2.0f, wallThickness/2.0f, (float)screenWidth, wallThickness}, // Top
        {screenWidth/2.0f, screenHeight - wallThickness/2.0f, (float)screenWidth, wallThickness}, // Bottom
        {wallThickness/2.0f, screenHeight/2.0f, wallThickness, (float)screenHeight - 2*wallThickness}, // Left
        {screenWidth - wallThickness/2.0f, screenHeight/2.0f, wallThickness, (float)screenHeight - 2*wallThickness} // Right
    };

    for (const auto& w : walls) {
        auto id = Velox_CreateEntity(world);
        Velox_AddTransform(world, id, w.x, w.y, 0.0f);
        Velox_AddRigidBody(world, id, 0.0f, true); // Static
        Velox_AddMovement(world, id);
        Velox_AddBoxCollider(world, id, w.w, w.h);
        
        VisualEntity ve;
        ve.id = id; ve.color = WHITE; ve.type = 1; ve.width = w.w; ve.height = w.h;
        entities.push_back(ve);
    }

    // 2. Dynamic Balls
    for (int i = 0; i < 2; ++i) {
        auto id = Velox_CreateEntity(world);
        float startX = screenWidth/2.0f + (i * 100.0f - 50.0f); 
        float startY = screenHeight/2.0f;
        Velox_AddTransform(world, id, startX, startY, 0.0f);
        Velox_AddRigidBody(world, id, 1.0f, false);
        Velox_AddMovement(world, id);
        Velox_AddCircleCollider(world, id, 20.0f);
        float vx = (i == 0) ? 300.0f : -300.0f;
        Velox_SetVelocity(world, id, vx, 225.0f);
        Velox_SetDamping(world, id, 0.0f, 0.0f);
        Velox_AddRotation(world, id, 5.0f, 0, 0);

        VisualEntity ve;
        ve.id = id; ve.color = (i == 0) ? MAROON : DARKBLUE; ve.type = 0; ve.radius = 20.0f;
        entities.push_back(ve);
    }

    // 3. Static Obstacles
    struct Obstacle { float x, y, r; };
    Obstacle obstacles[] = {
        {screenWidth * 0.25f, screenHeight * 0.25f, 40.0f},
        {screenWidth * 0.75f, screenHeight * 0.25f, 40.0f},
        {screenWidth * 0.25f, screenHeight * 0.75f, 40.0f},
        {screenWidth * 0.75f, screenHeight * 0.75f, 40.0f},
        {screenWidth * 0.5f, screenHeight * 0.5f, 60.0f}
    };
    for (const auto& obs : obstacles) {
        auto id = Velox_CreateEntity(world);
        Velox_AddTransform(world, id, obs.x, obs.y, 0.0f);
        Velox_AddRigidBody(world, id, 0.0f, true);
        Velox_AddMovement(world, id);
        Velox_AddCircleCollider(world, id, obs.r);
        VisualEntity ve; ve.id = id; ve.color = GRAY; ve.type = 0; ve.radius = obs.r;
        entities.push_back(ve);
    }
}

// Spawner State (not needed for Force Field Demo, but keep for compatibility)
float spawnTimer = 0.0f;
float spawnInterval = 0.1f;
int maxBalls = 150;
float gameTime = 0.0f; // Track elapsed time

void SetupForceFieldDemo(VeloxWorld* world, std::vector<VisualEntity>& entities, int screenWidth, int screenHeight) {
    float centerX = screenWidth / 2.0f;
    float centerY = screenHeight / 2.0f;
    float offsetX = 250.0f;
    float offsetY = 200.0f;
    
    // Create 4 Force Fields in a 2x2 grid
    struct ForceFieldDef { float x, y; int type; Color color; const char* name; };
    ForceFieldDef fields[] = {
        {centerX - offsetX, centerY - offsetY, 0, Fade(BLUE, 0.3f), "Inward"},      // Top-Left
        {centerX + offsetX, centerY - offsetY, 1, Fade(RED, 0.3f), "Outward"},      // Top-Right
        {centerX - offsetX, centerY + offsetY, 2, Fade(GREEN, 0.3f), "Clockwise"},  // Bottom-Left
        {centerX + offsetX, centerY + offsetY, 3, Fade(YELLOW, 0.3f), "AntiClockwise"} // Bottom-Right
    };

    for (const auto& f : fields) {
        auto id = Velox_CreateEntity(world);
        Velox_AddTransform(world, id, f.x, f.y, 0.0f);
        Velox_AddForceField(world, id, f.type, 1500.0f, 150.0f); // Strength 1500, Radius 150
        
        VisualEntity ve;
        ve.id = id;
        ve.color = f.color;
        ve.type = 2; // ForceField visual
        ve.radius = 150.0f;
        ve.spawnTime = 0.0f; // Force fields don't expire
        ve.label = f.name; // Store the label
        entities.push_back(ve);
    }
    
    // Reset game time
    gameTime = 0.0f;
}

void SetupOscillationDemo(VeloxWorld* world, std::vector<VisualEntity>& entities, int screenWidth, int screenHeight) {
    // World is already reset by main loop

    // 1. Horizontal Oscillation (Red)
    auto id1 = Velox_CreateEntity(world);
    Velox_AddTransform(world, id1, screenWidth * 0.25f, screenHeight * 0.5f, 0.0f);
    Velox_AddRigidBody(world, id1, 1.0f, true); // Static body, moved by oscillation
    Velox_AddCircleCollider(world, id1, 20.0f);
    Velox_AddOscillation(world, id1, 1.0f, 0.0f, 100.0f, 2.0f); // X-axis, Amp 100, Freq 2
    
    VisualEntity ve1; ve1.id = id1; ve1.color = RED; ve1.type = 0; ve1.radius = 20.0f;
    entities.push_back(ve1);

    // 2. Vertical Oscillation (Green)
    auto id2 = Velox_CreateEntity(world);
    Velox_AddTransform(world, id2, screenWidth * 0.5f, screenHeight * 0.5f, 0.0f);
    Velox_AddRigidBody(world, id2, 1.0f, true);
    Velox_AddCircleCollider(world, id2, 20.0f);
    Velox_AddOscillation(world, id2, 0.0f, 1.0f, 100.0f, 3.0f); // Y-axis, Amp 100, Freq 3
    
    VisualEntity ve2; ve2.id = id2; ve2.color = GREEN; ve2.type = 0; ve2.radius = 20.0f;
    entities.push_back(ve2);

    // 3. Diagonal Oscillation (Blue)
    auto id3 = Velox_CreateEntity(world);
    Velox_AddTransform(world, id3, screenWidth * 0.75f, screenHeight * 0.5f, 0.0f);
    Velox_AddRigidBody(world, id3, 1.0f, true);
    Velox_AddCircleCollider(world, id3, 20.0f);
    Velox_AddOscillation(world, id3, 1.0f, 1.0f, 100.0f, 1.5f); // Diagonal, Amp 100, Freq 1.5
    
    VisualEntity ve3; ve3.id = id3; ve3.color = BLUE; ve3.type = 0; ve3.radius = 20.0f;
    entities.push_back(ve3);
}

void SetupProjectileDemo(VeloxWorld* world, std::vector<VisualEntity>& entities, int screenWidth, int screenHeight) {
    // Ground
    auto groundId = Velox_CreateEntity(world);
    Velox_AddTransform(world, groundId, screenWidth / 2.0f, screenHeight - 20.0f, 0.0f);
    Velox_AddRigidBody(world, groundId, 0.0f, true);
    Velox_AddBoxCollider(world, groundId, screenWidth, 40.0f);
    // Ground Material: Extreme Friction (2.0), Low Restitution (0.0)
    Velox_AddPhysicalMaterial(world, groundId, 2.0f, 2.0f, 0.0f);
    
    VisualEntity groundVe; groundVe.id = groundId; groundVe.color = DARKGRAY; groundVe.type = 1; groundVe.width = (float)screenWidth; groundVe.height = 40.0f;
    entities.push_back(groundVe);

    // Gravity Force Field (Massive Inward Field below ground)
    auto gravId = Velox_CreateEntity(world);
    // Move origin to be directly below the target boxes to minimize lateral pull
    Velox_AddTransform(world, gravId, screenWidth * 0.8f, screenHeight + 50000.0f, 0.0f); 
    // Strength needs to be high to pull from this distance. 
    // F = Strength * Mass * (1 - dist/Radius). 
    // If Radius is huge, falloff is small.
    Velox_AddForceField(world, gravId, 0, 2000.0f, 100000.0f); // Type 0 = Inward
    
    // Visual for Gravity (Optional, maybe don't show it since it's "global")
    // entities.push_back(ve); // Skip visual for invisible gravity

    // Targets (Stack of boxes)
    float startX = screenWidth * 0.8f;
    float startY = screenHeight - 60.0f;
    float boxSize = 40.0f;
    
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 3; ++j) {
            auto boxId = Velox_CreateEntity(world);
            Velox_AddTransform(world, boxId, startX + j * (boxSize + 5.0f), startY - i * (boxSize + 5.0f), 0.0f);
            Velox_AddRigidBody(world, boxId, 1.0f, false);
            Velox_AddMovement(world, boxId);
            Velox_AddBoxCollider(world, boxId, boxSize, boxSize);
            // Box Material: High Friction (1.0), Low Restitution (0.0)
            Velox_AddPhysicalMaterial(world, boxId, 1.0f, 1.0f, 0.0f);
            Velox_SetDamping(world, boxId, 0.5f, 0.5f); // High damping for stability

            VisualEntity boxVe; boxVe.id = boxId; boxVe.color = ORANGE; boxVe.type = 1; boxVe.width = boxSize; boxVe.height = boxSize;
            entities.push_back(boxVe);
        }
    }
}



int main() {
    // Initialization
    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "Velox Physics Engine");

    // Load Window Icon
    Image icon = LoadImage("assets/velox_icon_window.png");
    SetWindowIcon(icon);
    UnloadImage(icon);

    SetTargetFPS(60);

    // Physics World
    VeloxWorld* world = nullptr;
    std::vector<VisualEntity> entities;

    bool paused = false;
    float frameRotation = 0.0f; 

    // Main game loop
    while (!WindowShouldClose()) {
        // --- Scene Switching Logic ---
        if (sceneChanged) {
            if (world) Velox_DestroyWorld(world);
            world = Velox_CreateWorld();
            entities.clear();
            frameRotation = 0.0f;

            if (currentScene == SceneType::BouncingBalls) {
                SetupBouncingBalls(world, entities, screenWidth, screenHeight);
            } else if (currentScene == SceneType::ForceFieldDemo) {
                SetupForceFieldDemo(world, entities, screenWidth, screenHeight);
            } else if (currentScene == SceneType::OscillationDemo) {
                SetupOscillationDemo(world, entities, screenWidth, screenHeight);
            } else {
                SetupProjectileDemo(world, entities, screenWidth, screenHeight);
            }
            sceneChanged = false;
        }

        // Input
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        
        // Mouse Spawning (Force Field Demo)
        if (currentScene == SceneType::ForceFieldDemo && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            Vector2 mouse = GetMousePosition();
            
            auto id = Velox_CreateEntity(world);
            Velox_AddTransform(world, id, mouse.x, mouse.y, 0.0f);
            Velox_AddRigidBody(world, id, 1.0f, false);
            Velox_AddMovement(world, id);
            Velox_AddCircleCollider(world, id, 15.0f);
            Velox_SetDamping(world, id, 0.05f, 0.1f);
            Velox_AddRotation(world, id, 0.0f, 0, 0);

            VisualEntity ve;
            ve.id = id;
            ve.color = ColorFromHSV((float)GetRandomValue(0, 360), 0.9f, 1.0f);
            ve.type = 0;
            ve.radius = 15.0f;
            ve.spawnTime = gameTime; // Track spawn time
            entities.push_back(ve);
        }

        // Mouse Shooting (Projectile Demo)
        if (currentScene == SceneType::ProjectileDemo && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            Vector2 mouse = GetMousePosition();
            float spawnX = 100.0f;
            float spawnY = screenHeight - 100.0f;
            
            auto id = Velox_CreateEntity(world);
            Velox_AddTransform(world, id, spawnX, spawnY, 0.0f);
            Velox_AddRigidBody(world, id, 1.0f, false);
            Velox_AddMovement(world, id);
            Velox_AddBoxCollider(world, id, 40.0f, 10.0f); // Arrow shape
            // FaceVelocity=true, Speed=0 (set later), MaxSpeed=1000, BounceFactor=0.1 (stick)
            Velox_AddProjectile(world, id, true, 0.0f, 1000.0f, 0.1f);
            
            // Calculate velocity towards mouse
            float dx = mouse.x - spawnX;
            float dy = mouse.y - spawnY;
            float len = sqrt(dx*dx + dy*dy);
            float speed = 800.0f;
            Velox_SetVelocity(world, id, (dx/len)*speed, (dy/len)*speed);

            VisualEntity ve;
            ve.id = id;
            ve.color = YELLOW;
            ve.type = 1; // Box
            ve.width = 40.0f;
            ve.height = 10.0f;
            ve.spawnTime = gameTime;
            ve.isProjectile = true;
            entities.push_back(ve);
        }
        
        // Frame Rotation (Only for Bouncing Balls)
        if (currentScene == SceneType::BouncingBalls) {
            float rotSpeed = 2.0f * (1.0f/60.0f);
            if (IsKeyDown(KEY_LEFT)) frameRotation -= rotSpeed;
            if (IsKeyDown(KEY_RIGHT)) frameRotation += rotSpeed;

            // Update Wall Transforms for Bouncing Balls
             Vector2 center = {screenWidth/2.0f, screenHeight/2.0f};
             float wallThickness = 20.0f;
             struct WallRel { float x, y, w, h; };
             WallRel relWalls[] = {
                {0, -(screenHeight - wallThickness)/2.0f, (float)screenWidth, wallThickness}, // Top
                {0, (screenHeight - wallThickness)/2.0f, (float)screenWidth, wallThickness}, // Bottom
                {-(screenWidth - wallThickness)/2.0f, 0, wallThickness, (float)screenHeight - 2*wallThickness}, // Left
                {(screenWidth - wallThickness)/2.0f, 0, wallThickness, (float)screenHeight - 2*wallThickness} // Right
            };

            for (int i = 0; i < 4 && i < entities.size(); ++i) {
                VisualEntity& ve = entities[i];
                float rx = relWalls[i].x;
                float ry = relWalls[i].y;
                float c = cos(frameRotation);
                float s = sin(frameRotation);
                float newX = center.x + (rx * c - ry * s);
                float newY = center.y + (rx * s + ry * c);
                Velox_AddTransform(world, ve.id, newX, newY, frameRotation);
            }
        }

        // Physics Step
        float dt = 1.0f / 60.0f;
        if (!paused) {
            // Update game time
            gameTime += dt;

            // --- Entity Cleanup (Force Field Demo) ---
            if (currentScene == SceneType::ForceFieldDemo) {
                // Remove entities older than 10 seconds (but keep force fields - type 2)
                for (auto it = entities.begin(); it != entities.end(); ) {
                    if (it->type == 0 && (gameTime - it->spawnTime) >= 10.0f) {
                        Velox_DestroyEntity(world, it->id);
                        it = entities.erase(it);
                    } else {
                        ++it;
                    }
                }
            } else if (currentScene == SceneType::ProjectileDemo) {
                // Check projectile lifetimes
                for (auto it = entities.begin(); it != entities.end(); ) {
                    if (it->isProjectile && (gameTime - it->spawnTime > 10.0f)) {
                        Velox_DestroyEntity(world, it->id);
                        it = entities.erase(it);
                    } else {
                        ++it;
                    }
                }
            }

            Velox_Step(world, dt);
        }

        // Draw
        BeginDrawing();
            ClearBackground(Color{20, 20, 20, 255}); // Dark Mode

            for (const auto& ve : entities) {
                float x, y, rot;
                Velox_GetPosition(world, ve.id, &x, &y, &rot);
                
                if (ve.type == 0) { // Circle (Ball)
                    DrawCircleV({x, y}, ve.radius, ve.color);
                    DrawLineEx({x, y}, {x + cos(rot)*ve.radius, y + sin(rot)*ve.radius}, 2.0f, BLACK);
                } else if (ve.type == 1) { // Box
                    DrawRectanglePro({x, y, ve.width, ve.height}, {ve.width/2, ve.height/2}, rot * RAD2DEG, ve.color);
                } else if (ve.type == 2) { // Force Field (visual)
                    DrawCircleV({x, y}, ve.radius, ve.color);
                    DrawCircleLines((int)x, (int)y, (int)ve.radius, Fade(WHITE, 0.6f));
                }
            }

            // --- UI Overlay ---
            // 1. Widget Window
            int widgetX = 10;
            int widgetY = 10;
            int widgetW = 320;
            int widgetH = 220; // Taller for force field info
            
            DrawRectangle(widgetX, widgetY, widgetW, widgetH, Fade(BLACK, 0.8f));
            DrawRectangleLines(widgetX, widgetY, widgetW, widgetH, GREEN);
            
            DrawText("Velox Physics", widgetX + 10, widgetY + 10, 20, GREEN); 
            DrawText(TextFormat("Entities: %d | FPS: %d", (int)entities.size(), GetFPS()), widgetX + 10, widgetY + 40, 10, WHITE);
            
            // Dynamic Instructions
            if (currentScene == SceneType::BouncingBalls) {
                DrawText("Sim: Bouncing Balls", widgetX + 10, widgetY + 60, 10, YELLOW);
                DrawText("Controls:", widgetX + 10, widgetY + 80, 10, GRAY);
                DrawText("- Left/Right: Rotate Frame", widgetX + 10, widgetY + 95, 10, GRAY);
                DrawText("- Space: Pause", widgetX + 10, widgetY + 110, 10, GRAY);
            } else if (currentScene == SceneType::ForceFieldDemo) {
                DrawText("Sim: Force Field Demo", widgetX + 10, widgetY + 60, 10, YELLOW);
                DrawText("Controls:", widgetX + 10, widgetY + 80, 10, GRAY);
                DrawText("- Click: Spawn Ball", widgetX + 10, widgetY + 95, 10, GRAY);
                DrawText("- Space: Pause", widgetX + 10, widgetY + 110, 10, GRAY);
                DrawText("(Balls auto-destroy after 10s)", widgetX + 10, widgetY + 125, 10, Fade(WHITE, 0.6f));
                
                // Force Field Key
                DrawText("Force Fields:", widgetX + 10, widgetY + 145, 10, GRAY);
                DrawText("Blue: Inward | Red: Outward", widgetX + 10, widgetY + 160, 9, Fade(WHITE, 0.8f));
                DrawText("Green: Clockwise | Yellow: AntiCW", widgetX + 10, widgetY + 175, 9, Fade(WHITE, 0.8f));
            } else if (currentScene == SceneType::OscillationDemo) {
                DrawText("Sim: Oscillation Demo", widgetX + 10, widgetY + 60, 10, YELLOW);
                DrawText("Controls:", widgetX + 10, widgetY + 80, 10, GRAY);
                DrawText("- Space: Pause", widgetX + 10, widgetY + 95, 10, GRAY);
                
                DrawText("Oscillation Types:", widgetX + 10, widgetY + 120, 10, GRAY);
                DrawText("Red: Horizontal (X-Axis)", widgetX + 10, widgetY + 135, 9, Fade(RED, 0.8f));
                DrawText("Green: Vertical (Y-Axis)", widgetX + 10, widgetY + 150, 9, Fade(GREEN, 0.8f));
                DrawText("Blue: Diagonal", widgetX + 10, widgetY + 165, 9, Fade(BLUE, 0.8f));
            } else if (currentScene == SceneType::ProjectileDemo) {
                DrawText("Sim: Projectile Demo", widgetX + 10, widgetY + 60, 10, YELLOW);
                DrawText("Controls:", widgetX + 10, widgetY + 80, 10, GRAY);
                DrawText("- Click: Shoot Arrow", widgetX + 10, widgetY + 95, 10, GRAY);
                DrawText("- Space: Pause", widgetX + 10, widgetY + 110, 10, GRAY);
                DrawText("Aim with mouse!", widgetX + 10, widgetY + 130, 10, ORANGE);
            }

            // Combo Box
            int comboX = widgetX;
            int comboY = widgetY + widgetH + 10;
            int comboW = 200;
            int comboH = 30;
            
            DrawRectangle(comboX, comboY, comboW, comboH, Fade(WHITE, 0.2f));
            DrawRectangleLines(comboX, comboY, comboW, comboH, WHITE);
            DrawText(sceneNames[selectedItem], comboX + 10, comboY + 8, 10, WHITE);
            DrawText("v", comboX + comboW - 20, comboY + 8, 10, WHITE);
            
            Vector2 mouse = GetMousePosition();
            bool hoverCombo = (mouse.x >= comboX && mouse.x <= comboX + comboW && mouse.y >= comboY && mouse.y <= comboY + comboH);
            
            if (hoverCombo && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
                isDropdownOpen = !isDropdownOpen;
            }

            if (isDropdownOpen) {
                for (int i = 0; i < 4; ++i) { // Updated to 4 options
                    int itemY = comboY + comboH + (i * comboH);
                    bool hoverItem = (mouse.x >= comboX && mouse.x <= comboX + comboW && mouse.y >= itemY && mouse.y <= itemY + comboH);
                    
                    DrawRectangle(comboX, itemY, comboW, comboH, hoverItem ? Fade(GREEN, 0.5f) : Fade(BLACK, 0.9f));
                    DrawRectangleLines(comboX, itemY, comboW, comboH, WHITE);
                    DrawText(sceneNames[i], comboX + 10, itemY + 8, 10, WHITE);
                    
                    if (hoverItem && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
                        selectedItem = i;
                        currentScene = (SceneType)i;
                        sceneChanged = true;
                        isDropdownOpen = false;
                    }
                }
            }

        EndDrawing();
    }

    // De-Initialization
    if (world) Velox_DestroyWorld(world);
    CloseWindow();

    return 0;
}
