
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
#undef near
#undef far

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
    ProjectileDemo,
    GravityDemo,
    JointDemo,
    SandboxDemo,
    ChainDemo,
    RaycastDemo,
    RevolutePrismaticDemo,
    CCDShowcase,
    SleepingShowcase,
    SoftBodySandbox,
    SoftBodyFunnel,
    SoftBodyStacking
};

SceneType currentScene = SceneType::BouncingBalls;
bool sceneChanged = true;

// UI State
bool isDropdownOpen = false;
int selectedItem = 0;
const char* sceneNames[] = {
    "Bouncing Balls",
    "Force Field Demo",
    "Oscillation Demo",
    "Projectile Demo",
    "Gravity Direction Demo",
    "Distance Joint Demo",
    "Convex Polygons & Motors Sandbox",
    "Chain Shapes Showcase",
    "Raycast Queries Showcase",
    "Revolute & Prismatic & Gear & Pulley Showcase",
    "CCD vs Tunneling Showcase",
    "Sleeping & Activation Showcase",
    "Soft Body Sandbox Demos",
    "Soft Body Funnel & Squeeze Showcase",
    "Soft Body Stacking & Loads Showcase"
};

void AddScreenBoundaries(VeloxWorld* world, std::vector<VisualEntity>& entities, int screenWidth, int screenHeight) {
    float wallThickness = 40.0f;
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
        ve.id = id; ve.color = GRAY; ve.type = 1; ve.width = w.w; ve.height = w.h;
        entities.push_back(ve);
    }
}

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

    // Set directional gravity (straight down for this scene)
    Velox_SetGravity(world, 0.0f, 400.0f);

    // 2. Dynamic Balls
    for (int i = 0; i < 2; ++i) {
        auto id = Velox_CreateEntity(world);
        float startX = (i == 0) ? screenWidth * 0.35f : screenWidth * 0.65f;
        float startY = screenHeight * 0.3f;

        Velox_AddTransform(world, id, startX, startY, 0.0f);
        Velox_AddRigidBody(world, id, 1.0f, false);
        Velox_AddMovement(world, id);
        Velox_AddCircleCollider(world, id, 20.0f);
        // High restitution so bouncing is visible
        Velox_AddPhysicalMaterial(world, id, 0.1f, 0.05f, 0.85f);
        float vx = (i == 0) ? 300.0f : -300.0f;
        Velox_SetVelocity(world, id, vx, 1000.0f);
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
    AddScreenBoundaries(world, entities, screenWidth, screenHeight);
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
        Velox_AddForceField(world, id, f.type, 5000.0f, 150.0f); // Strength 1500, Radius 150
        
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
    AddScreenBoundaries(world, entities, screenWidth, screenHeight);
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
    AddScreenBoundaries(world, entities, screenWidth, screenHeight);
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

// ============================================================
// SCENE: Gravity Direction Demo
// Showcases the new Velox_SetGravity directional gravity API.
// Balls fall in the direction the user controls (WASD).
// The gravity arrow rotates in real-time.
// ============================================================
void SetupGravityDemo(VeloxWorld* world, std::vector<VisualEntity>& entities, int screenWidth, int screenHeight) {
    // Walls (closed arena)
    float wallThickness = 20.0f;
    struct WallDef { float x, y, w, h; };
    WallDef walls[] = {
        {screenWidth/2.0f, wallThickness/2.0f,             (float)screenWidth, wallThickness},
        {screenWidth/2.0f, screenHeight - wallThickness/2.0f, (float)screenWidth, wallThickness},
        {wallThickness/2.0f, screenHeight/2.0f,            wallThickness, (float)screenHeight - 2*wallThickness},
        {screenWidth - wallThickness/2.0f, screenHeight/2.0f, wallThickness, (float)screenHeight - 2*wallThickness}
    };
    for (const auto& w : walls) {
        auto id = Velox_CreateEntity(world);
        Velox_AddTransform(world, id, w.x, w.y, 0.0f);
        Velox_AddRigidBody(world, id, 0.0f, true);
        Velox_AddMovement(world, id);
        Velox_AddBoxCollider(world, id, w.w, w.h);
        VisualEntity ve; ve.id = id; ve.color = DARKGRAY; ve.type = 1; ve.width = w.w; ve.height = w.h;
        entities.push_back(ve);
    }

    // Scatter a mix of circles and boxes
    Color palette[] = {
        {255, 80,  80,  255}, {80,  180, 255, 255}, {80,  255, 140, 255},
        {255, 200, 60,  255}, {200, 80,  255, 255}, {255, 130, 30,  255},
        {60,  220, 220, 255}, {255, 80,  180, 255}, {160, 255, 80,  255},
    };
    int numPalette = sizeof(palette) / sizeof(palette[0]);

    for (int i = 0; i < 18; ++i) {
        auto id = Velox_CreateEntity(world);
        float x = 100.0f + (float)(i % 6) * 180.0f;
        float y = 100.0f + (float)(i / 6) * 160.0f;
        Velox_AddTransform(world, id, x, y, 0.0f);
        Velox_AddRigidBody(world, id, 1.0f, false);
        Velox_AddMovement(world, id);
        Velox_SetDamping(world, id, 0.02f, 0.05f);
        Velox_AddPhysicalMaterial(world, id, 0.3f, 0.2f, 0.6f);

        VisualEntity ve; ve.id = id; ve.color = palette[i % numPalette];
        if (i % 3 == 0) {
            Velox_AddBoxCollider(world, id, 36.0f, 36.0f);
            ve.type = 1; ve.width = 36.0f; ve.height = 36.0f;
        } else {
            Velox_AddCircleCollider(world, id, 18.0f);
            ve.type = 0; ve.radius = 18.0f;
        }
        entities.push_back(ve);
    }

    // Start gravity pointing down
    Velox_SetGravity(world, 0.0f, 400.0f);
}

// ============================================================
// SCENE: Distance Joint Demo
// Showcases JointComponent XPBD constraints:
//  - A hanging pendulum chain
//  - A swinging double-pendulum
//  - A Newton's cradle style row
// ============================================================

// We keep joint metadata so we can draw them as lines
struct JointVisual {
    Velox::EntityID idA;
    Velox::EntityID idB;
    Color color;
};
std::vector<JointVisual> g_jointVisuals;

struct SoftBodyVisual {
    Velox::EntityID managerId;
    std::vector<Velox::EntityID> nodes;
    Color color;
};
std::vector<SoftBodyVisual> g_softBodyVisuals;

void SetupJointDemo(VeloxWorld* world, std::vector<VisualEntity>& entities, int screenWidth, int screenHeight) {
    AddScreenBoundaries(world, entities, screenWidth, screenHeight);
    g_jointVisuals.clear();

    Velox_SetGravity(world, 0.0f, 500.0f);

    // --- 1. Hanging Pendulum Chain (center-left) ---
    {
        // Static anchor point at top
        float anchorX = screenWidth * 0.25f;
        float anchorY = 80.0f;
        auto anchorId = Velox_CreateEntity(world);
        Velox_AddTransform(world, anchorId, anchorX, anchorY, 0.0f);
        Velox_AddRigidBody(world, anchorId, 0.0f, true);
        Velox_AddMovement(world, anchorId);
        Velox_AddCircleCollider(world, anchorId, 8.0f);
        VisualEntity anchorVe; anchorVe.id = anchorId; anchorVe.color = WHITE;
        anchorVe.type = 0; anchorVe.radius = 8.0f;
        entities.push_back(anchorVe);

        Velox::EntityID prevId = anchorId;
        float segLen = 60.0f;
        Color chainColors[] = {
            {255, 100, 100, 255}, {255, 180, 60, 255}, {80, 200, 255, 255},
            {160, 80, 255, 255}, {80, 240, 140, 255}
        };
        for (int i = 0; i < 5; ++i) {
            auto linkId = Velox_CreateEntity(world);
            float lx = anchorX + 20.0f * (float)i; // slight offset for pendulum swing
            float ly = anchorY + segLen * (float)(i + 1);
            Velox_AddTransform(world, linkId, lx, ly, 0.0f);
            Velox_AddRigidBody(world, linkId, 1.0f, false);
            Velox_AddMovement(world, linkId);
            Velox_AddCircleCollider(world, linkId, 12.0f);
            Velox_SetDamping(world, linkId, 0.3f, 0.3f);
            Velox_AddPhysicalMaterial(world, linkId, 0.3f, 0.2f, 0.2f);

            VisualEntity ve; ve.id = linkId; ve.color = chainColors[i];
            ve.type = 0; ve.radius = 12.0f;
            entities.push_back(ve);

            // Correct API call: anchors are (0,0) (centers of bodies), targetDistance, compliance
            Velox_AddDistanceJoint(world, prevId, linkId, 0.0f, 0.0f, 0.0f, 0.0f, segLen, 0.0f);
            g_jointVisuals.push_back({prevId, linkId, Fade(WHITE, 0.5f)});
            prevId = linkId;
        }
    }

    // --- 2. Double Pendulum (center) ---
    {
        float anchorX = screenWidth * 0.5f;
        float anchorY = 80.0f;
        auto anchorId = Velox_CreateEntity(world);
        Velox_AddTransform(world, anchorId, anchorX, anchorY, 0.0f);
        Velox_AddRigidBody(world, anchorId, 0.0f, true);
        Velox_AddMovement(world, anchorId);
        Velox_AddCircleCollider(world, anchorId, 8.0f);
        VisualEntity anchorVe; anchorVe.id = anchorId; anchorVe.color = WHITE;
        anchorVe.type = 0; anchorVe.radius = 8.0f;
        entities.push_back(anchorVe);

        // Bob 1 - offset for chaotic initial condition
        auto bob1Id = Velox_CreateEntity(world);
        Velox_AddTransform(world, bob1Id, anchorX + 90.0f, anchorY + 130.0f, 0.0f);
        Velox_AddRigidBody(world, bob1Id, 2.0f, false);
        Velox_AddMovement(world, bob1Id);
        Velox_AddCircleCollider(world, bob1Id, 18.0f);
        Velox_SetDamping(world, bob1Id, 0.01f, 0.01f);
        Velox_AddPhysicalMaterial(world, bob1Id, 0.1f, 0.05f, 0.1f);
        VisualEntity ve1; ve1.id = bob1Id; ve1.color = {255, 120, 50, 255};
        ve1.type = 0; ve1.radius = 18.0f;
        entities.push_back(ve1);

        // Bob 2
        auto bob2Id = Velox_CreateEntity(world);
        Velox_AddTransform(world, bob2Id, anchorX + 120.0f, anchorY + 270.0f, 0.0f);
        Velox_AddRigidBody(world, bob2Id, 1.5f, false);
        Velox_AddMovement(world, bob2Id);
        Velox_AddCircleCollider(world, bob2Id, 14.0f);
        Velox_SetDamping(world, bob2Id, 0.01f, 0.01f);
        Velox_AddPhysicalMaterial(world, bob2Id, 0.1f, 0.05f, 0.1f);
        VisualEntity ve2; ve2.id = bob2Id; ve2.color = {80, 180, 255, 255};
        ve2.type = 0; ve2.radius = 14.0f;
        entities.push_back(ve2);

        Velox_AddDistanceJoint(world, anchorId, bob1Id, 0.0f, 0.0f, 0.0f, 0.0f, 150.0f, 0.0f);
        Velox_AddDistanceJoint(world, bob1Id,   bob2Id, 0.0f, 0.0f, 0.0f, 0.0f, 150.0f, 0.0f);
        g_jointVisuals.push_back({anchorId, bob1Id, {255, 120, 50, 200}});
        g_jointVisuals.push_back({bob1Id, bob2Id, {80, 180, 255, 200}});
    }

    // --- 3. Newton's Cradle (right side) ---
    {
        float anchorX = screenWidth * 0.78f;
        float anchorY = 80.0f;
        float spacing = 30.0f;
        float armLen  = 200.0f;
        int   numBalls = 5;

        // Top bar (static anchor strip)
        auto barId = Velox_CreateEntity(world);
        float barW = spacing * (numBalls - 1) + 40.0f;
        Velox_AddTransform(world, barId, anchorX, anchorY - 10.0f, 0.0f);
        Velox_AddRigidBody(world, barId, 0.0f, true);
        Velox_AddMovement(world, barId);
        Velox_AddBoxCollider(world, barId, barW, 12.0f);
        VisualEntity barVe; barVe.id = barId; barVe.color = GRAY; barVe.type = 1;
        barVe.width = barW; barVe.height = 12.0f;
        entities.push_back(barVe);

        // Anchor pivot entities (static)
        std::vector<Velox::EntityID> pivots;
        for (int i = 0; i < numBalls; ++i) {
            float px = anchorX - spacing * (numBalls - 1) * 0.5f + spacing * i;
            auto pivId = Velox_CreateEntity(world);
            Velox_AddTransform(world, pivId, px, anchorY, 0.0f);
            Velox_AddRigidBody(world, pivId, 0.0f, true);
            Velox_AddMovement(world, pivId);
            Velox_AddCircleCollider(world, pivId, 3.0f);
            VisualEntity pVe; pVe.id = pivId; pVe.color = WHITE; pVe.type = 0; pVe.radius = 3.0f;
            entities.push_back(pVe);
            pivots.push_back(pivId);
        }

        // Ball entities — leftmost pulled aside
        Color ballColor = {255, 220, 50, 255};
        for (int i = 0; i < numBalls; ++i) {
            float px = anchorX - spacing * (numBalls - 1) * 0.5f + spacing * i;
            float bx = (i == 0) ? px - 120.0f : px;  // pull left ball aside
            float by = anchorY + armLen;
            auto ballId = Velox_CreateEntity(world);
            Velox_AddTransform(world, ballId, bx, by, 0.0f);
            Velox_AddRigidBody(world, ballId, 1.0f, false);
            Velox_AddMovement(world, ballId);
            Velox_AddCircleCollider(world, ballId, 13.0f);
            Velox_SetDamping(world, ballId, 0.005f, 0.005f);
            Velox_AddPhysicalMaterial(world, ballId, 0.05f, 0.01f, 0.95f); // Very elastic
            VisualEntity bVe; bVe.id = ballId; bVe.color = ballColor;
            bVe.type = 0; bVe.radius = 13.0f;
            entities.push_back(bVe);

            Velox_AddDistanceJoint(world, pivots[i], ballId, 0.0f, 0.0f, 0.0f, 0.0f, armLen, 0.0f);
            g_jointVisuals.push_back({pivots[i], ballId, Fade(LIGHTGRAY, 0.7f)});
        }
    }
}

// ============================================================
// SCENE: Advanced Features Sandbox (Polygons, Chains, Motors, Raycasts)
// ============================================================
struct SandboxShapeInfo {
    Velox::EntityID id;
    int type; // 0 = Circle, 1 = Box, 2 = Polygon, 3 = Chain
    Color color;
    std::vector<Vector2> pts; // local verts for drawing
};
std::vector<SandboxShapeInfo> g_sandboxShapes;

// Shared Raycast hit values to draw
bool g_raycastHit = false;
Vector2 g_raycastStart = { 0, 0 };
Vector2 g_raycastEnd = { 0, 0 };
Vector2 g_raycastHitPt = { 0, 0 };
Vector2 g_raycastNormal = { 0, 0 };
float g_raycastFrac = 0.0f;

void SetupSandboxDemo(VeloxWorld* world, std::vector<VisualEntity>& entities, int screenWidth, int screenHeight) {
    AddScreenBoundaries(world, entities, screenWidth, screenHeight);
    g_sandboxShapes.clear();
    g_jointVisuals.clear();

    Velox_SetGravity(world, 0.0f, 300.0f);

    // Motorized Joint Carousel (swings a polygon blade around a static point)
    float pivotX = screenWidth * 0.5f;
    float pivotY = screenHeight * 0.35f;

    auto motorAnchorId = Velox_CreateEntity(world);
    Velox_AddTransform(world, motorAnchorId, pivotX, pivotY, 0.0f);
    Velox_AddRigidBody(world, motorAnchorId, 0.0f, true);
    Velox_AddMovement(world, motorAnchorId);
    Velox_AddCircleCollider(world, motorAnchorId, 10.0f);
    g_sandboxShapes.push_back({motorAnchorId, 0, WHITE, {}});

    auto bladeId = Velox_CreateEntity(world);
    Velox_AddTransform(world, bladeId, pivotX + 100.0f, pivotY, 0.0f);
    Velox_AddRigidBody(world, bladeId, 1.5f, false);
    Velox_AddMovement(world, bladeId);
    Velox_SetDamping(world, bladeId, 0.02f, 0.02f);

    // Add a custom triangular polygon collider (3 vertices)
    float pxVertsX[] = { -60.0f, 60.0f, 0.0f };
    float pxVertsY[] = { -20.0f, -20.0f, 30.0f };
    Velox_AddPolygonCollider(world, bladeId, pxVertsX, pxVertsY, 3);
    
    std::vector<Vector2> bladeLocal;
    bladeLocal.push_back({-60.0f, -20.0f});
    bladeLocal.push_back({60.0f, -20.0f});
    bladeLocal.push_back({0.0f, 30.0f});
    g_sandboxShapes.push_back({bladeId, 2, VIOLET, bladeLocal});

    // Create joint
    auto jointId = Velox_CreateEntity(world);
    Velox_AddDistanceJoint(world, motorAnchorId, bladeId, 0.0f, 0.0f, -100.0f, 0.0f, 100.0f, 0.0f);
    g_jointVisuals.push_back({motorAnchorId, bladeId, LIGHTGRAY});

    // Enable motor to spin the blade
    Velox_SetJointMotor(world, jointId, true, 2.5f, 20.0f); // 2.5 rad/s target speed

    // Spawn static ground box so they don't fall forever in this demo
    auto groundBox = Velox_CreateEntity(world);
    Velox_AddTransform(world, groundBox, screenWidth * 0.5f, screenHeight - 40.0f, 0.0f);
    Velox_AddRigidBody(world, groundBox, 0.0f, true);
    Velox_AddMovement(world, groundBox);
    Velox_AddBoxCollider(world, groundBox, screenWidth - 100.0f, 25.0f);
    
    std::vector<Vector2> groundLocal = {
        {-(screenWidth - 100.0f)*0.5f, -12.5f},
        {(screenWidth - 100.0f)*0.5f, -12.5f},
        {(screenWidth - 100.0f)*0.5f, 12.5f},
        {-(screenWidth - 100.0f)*0.5f, 12.5f}
    };
    g_sandboxShapes.push_back({groundBox, 2, GRAY, groundLocal});

    // Falling convex polygons (Pentagons/Triangles)
    Color palette[] = { RED, ORANGE, GOLD, GREEN, BLUE, MAGENTA };
    for (int i = 0; i < 4; ++i) {
        auto polyId = Velox_CreateEntity(world);
        Velox_AddTransform(world, polyId, screenWidth * 0.25f + i * 160.0f, 100.0f, 0.5f * i);
        Velox_AddRigidBody(world, polyId, 1.0f, false);
        Velox_AddMovement(world, polyId);
        Velox_SetDamping(world, polyId, 0.05f, 0.05f);
        Velox_AddPhysicalMaterial(world, polyId, 0.4f, 0.3f, 0.3f);

        // Pentagon (5 vertices)
        float pX[5], pY[5];
        std::vector<Vector2> localVerts;
        float r = 25.0f;
        for (int v = 0; v < 5; ++v) {
            float angle = (float)v * (2.0f * PI / 5.0f);
            pX[v] = std::cos(angle) * r;
            pY[v] = std::sin(angle) * r;
            localVerts.push_back({pX[v], pY[v]});
        }
        Velox_AddPolygonCollider(world, polyId, pX, pY, 5);
        g_sandboxShapes.push_back({polyId, 2, palette[i % 6], localVerts});
    }
}

void SetupChainDemo(VeloxWorld* world, std::vector<VisualEntity>& entities, int screenWidth, int screenHeight) {
    AddScreenBoundaries(world, entities, screenWidth, screenHeight);
    g_sandboxShapes.clear();
    g_jointVisuals.clear();

    Velox_SetGravity(world, 0.0f, 350.0f);

    // Create static sinusoidal chain floor
    auto chainId = Velox_CreateEntity(world);
    Velox_AddTransform(world, chainId, 0.0f, 0.0f, 0.0f);
    Velox_AddRigidBody(world, chainId, 0.0f, true);
    Velox_AddMovement(world, chainId);

    std::vector<float> cx, cy;
    std::vector<Vector2> localPts;
    int numPoints = 18;
    for (int i = 0; i < numPoints; ++i) {
        float px = (float)i * ((float)screenWidth / (float)(numPoints - 1));
        float py = screenHeight - 100.0f - std::sin((float)i * 0.9f) * 60.0f;
        cx.push_back(px);
        cy.push_back(py);
        localPts.push_back({px, py});
    }
    Velox_AddChainCollider(world, chainId, cx.data(), cy.data(), numPoints);
    g_sandboxShapes.push_back({chainId, 3, GRAY, localPts});

    // Spawn falling balls to bounce on the chain
    for (int i = 0; i < 6; ++i) {
        auto ballId = Velox_CreateEntity(world);
        Velox_AddTransform(world, ballId, 150.0f + i * 180.0f, 80.0f, 0.0f);
        Velox_AddRigidBody(world, ballId, 1.0f, false);
        Velox_AddMovement(world, ballId);
        Velox_AddCircleCollider(world, ballId, 16.0f);
        Velox_SetDamping(world, ballId, 0.01f, 0.01f);
        Velox_AddPhysicalMaterial(world, ballId, 0.4f, 0.2f, 0.6f); // Good bounce

        VisualEntity ve; ve.id = ballId; ve.color = SKYBLUE;
        ve.type = 0; ve.radius = 16.0f;
        entities.push_back(ve);
    }

    // Spawn a couple of boxes too
    for (int i = 0; i < 3; ++i) {
        auto boxId = Velox_CreateEntity(world);
        Velox_AddTransform(world, boxId, 250.0f + i * 320.0f, 150.0f, 0.2f * i);
        Velox_AddRigidBody(world, boxId, 1.5f, false);
        Velox_AddMovement(world, boxId);
        Velox_AddBoxCollider(world, boxId, 35.0f, 35.0f);
        Velox_SetDamping(world, boxId, 0.02f, 0.02f);
        Velox_AddPhysicalMaterial(world, boxId, 0.5f, 0.3f, 0.3f);

        VisualEntity ve; ve.id = boxId; ve.color = ORANGE;
        ve.type = 1; ve.width = 35.0f; ve.height = 35.0f;
        entities.push_back(ve);
    }
}

void SetupRaycastDemo(VeloxWorld* world, std::vector<VisualEntity>& entities, int screenWidth, int screenHeight) {
    AddScreenBoundaries(world, entities, screenWidth, screenHeight);
    g_sandboxShapes.clear();
    g_jointVisuals.clear();

    Velox_SetGravity(world, 0.0f, 0.0f); // Zero gravity for static obstacle showcase

    // 1. Static Circles
    for (int i = 0; i < 3; ++i) {
        auto targetId = Velox_CreateEntity(world);
        Velox_AddTransform(world, targetId, screenWidth * 0.3f + i * 200.0f, screenHeight * 0.4f + (i % 2) * 80.0f, 0.0f);
        Velox_AddRigidBody(world, targetId, 0.0f, true);
        Velox_AddMovement(world, targetId);
        Velox_AddCircleCollider(world, targetId, 40.0f);

        VisualEntity ve; ve.id = targetId; ve.color = MAROON;
        ve.type = 0; ve.radius = 40.0f;
        entities.push_back(ve);
    }

    // 2. Static Boxes
    for (int i = 0; i < 2; ++i) {
        auto targetId = Velox_CreateEntity(world);
        Velox_AddTransform(world, targetId, screenWidth * 0.2f + i * 500.0f, screenHeight * 0.65f, 0.4f);
        Velox_AddRigidBody(world, targetId, 0.0f, true);
        Velox_AddMovement(world, targetId);
        Velox_AddBoxCollider(world, targetId, 90.0f, 50.0f);

        VisualEntity ve; ve.id = targetId; ve.color = DARKGREEN;
        ve.type = 1; ve.width = 90.0f; ve.height = 50.0f;
        entities.push_back(ve);
    }
}

void SetupRevolutePrismaticDemo(VeloxWorld* world, std::vector<VisualEntity>& entities, int screenWidth, int screenHeight) {
    AddScreenBoundaries(world, entities, screenWidth, screenHeight);
    g_sandboxShapes.clear();
    g_jointVisuals.clear();

    Velox_SetGravity(world, 0.0f, 300.0f);

    // --- 1. Revolute Hinge (Pin Joint) with motor ---
    {
        float pinX = screenWidth * 0.25f;
        float pinY = screenHeight * 0.3f;
        
        auto pin = Velox_CreateEntity(world);
        Velox_AddTransform(world, pin, pinX, pinY, 0.0f);
        Velox_AddRigidBody(world, pin, 0.0f, true);
        Velox_AddMovement(world, pin);
        Velox_AddCircleCollider(world, pin, 12.0f);
        VisualEntity pinVe; pinVe.id = pin; pinVe.color = WHITE; pinVe.type = 0; pinVe.radius = 12.0f;
        entities.push_back(pinVe);

        auto blade = Velox_CreateEntity(world);
        Velox_AddTransform(world, blade, pinX, pinY + 80.0f, 0.0f);
        Velox_AddRigidBody(world, blade, 1.5f, false);
        Velox_AddMovement(world, blade);
        Velox_AddBoxCollider(world, blade, 20.0f, 150.0f);
        VisualEntity bladeVe; bladeVe.id = blade; bladeVe.color = ORANGE; bladeVe.type = 1; bladeVe.width = 20.0f; bladeVe.height = 150.0f;
        entities.push_back(bladeVe);

        // Hinge joint at pin position
        Velox_AddRevoluteJoint(world, pin, blade, 0.0f, 0.0f, 0.0f, -80.0f, 0.0f, false, 0.0f, 0.0f, true, 1.5f, 25.0f);
        g_jointVisuals.push_back({pin, blade, WHITE});
    }

    // --- 2. Prismatic Slider with limits ---
    {
        float anchorX = screenWidth * 0.5f;
        float anchorY = screenHeight * 0.3f;

        auto anchor = Velox_CreateEntity(world);
        Velox_AddTransform(world, anchor, anchorX, anchorY, 0.0f);
        Velox_AddRigidBody(world, anchor, 0.0f, true);
        Velox_AddMovement(world, anchor);
        Velox_AddBoxCollider(world, anchor, 40.0f, 40.0f);
        VisualEntity anchorVe; anchorVe.id = anchor; anchorVe.color = WHITE; anchorVe.type = 1; anchorVe.width = 40.0f; anchorVe.height = 40.0f;
        entities.push_back(anchorVe);

        auto slider = Velox_CreateEntity(world);
        Velox_AddTransform(world, slider, anchorX + 60.0f, anchorY, 0.0f);
        Velox_AddRigidBody(world, slider, 1.0f, false);
        Velox_AddMovement(world, slider);
        Velox_AddBoxCollider(world, slider, 50.0f, 30.0f);
        VisualEntity sliderVe; sliderVe.id = slider; sliderVe.color = SKYBLUE; sliderVe.type = 1; sliderVe.width = 50.0f; sliderVe.height = 30.0f;
        entities.push_back(sliderVe);

        // Prismatic slider along horizontal axis (1.0f, 0.0f), with limits [-150, 150]
        Velox_AddPrismaticJoint(world, anchor, slider, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, -150.0f, 150.0f, true, 80.0f, 10.0f);
        g_jointVisuals.push_back({anchor, slider, GRAY});
    }

    // --- 3. Gear Joint coupling two revolute hinges ---
    {
        float g1X = screenWidth * 0.75f;
        float g1Y = screenHeight * 0.3f;
        float g2X = screenWidth * 0.75f + 100.0f;
        float g2Y = g1Y;

        auto pin1 = Velox_CreateEntity(world);
        Velox_AddTransform(world, pin1, g1X, g1Y, 0.0f);
        Velox_AddRigidBody(world, pin1, 0.0f, true);
        Velox_AddCircleCollider(world, pin1, 10.0f);
        VisualEntity pin1Ve; pin1Ve.id = pin1; pin1Ve.color = WHITE; pin1Ve.type = 0; pin1Ve.radius = 10.0f;
        entities.push_back(pin1Ve);

        auto wheel1 = Velox_CreateEntity(world);
        Velox_AddTransform(world, wheel1, g1X, g1Y, 0.0f);
        Velox_AddRigidBody(world, wheel1, 1.0f, false);
        Velox_AddMovement(world, wheel1);
        Velox_AddCircleCollider(world, wheel1, 45.0f);
        VisualEntity w1Ve; w1Ve.id = wheel1; w1Ve.color = GREEN; w1Ve.type = 0; w1Ve.radius = 45.0f;
        entities.push_back(w1Ve);

        Velox_AddRevoluteJoint(world, pin1, wheel1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, 0.0f, 0.0f, true, 2.0f, 15.0f);

        auto pin2 = Velox_CreateEntity(world);
        Velox_AddTransform(world, pin2, g2X, g2Y, 0.0f);
        Velox_AddRigidBody(world, pin2, 0.0f, true);
        Velox_AddCircleCollider(world, pin2, 10.0f);
        VisualEntity pin2Ve; pin2Ve.id = pin2; pin2Ve.color = WHITE; pin2Ve.type = 0; pin2Ve.radius = 10.0f;
        entities.push_back(pin2Ve);

        auto wheel2 = Velox_CreateEntity(world);
        Velox_AddTransform(world, wheel2, g2X, g2Y, 0.0f);
        Velox_AddRigidBody(world, wheel2, 1.0f, false);
        Velox_AddMovement(world, wheel2);
        Velox_AddCircleCollider(world, wheel2, 45.0f);
        VisualEntity w2Ve; w2Ve.id = wheel2; w2Ve.color = LIME; w2Ve.type = 0; w2Ve.radius = 45.0f;
        entities.push_back(w2Ve);

        Velox_AddRevoluteJoint(world, pin2, wheel2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, 0.0f, 0.0f, false, 0.0f, 0.0f);

        // Gear Coupling: B spins in opposite direction (ratio = -1.0)
        Velox_AddGearJoint(world, wheel1, wheel2, -1.0f, 0.0f);
    }

    // --- 4. Pulley Joint ---
    {
        float leftX = screenWidth * 0.4f;
        float rightX = screenWidth * 0.6f;
        float groundY = screenHeight * 0.6f;
        float weightY = screenHeight * 0.75f;

        auto weight1 = Velox_CreateEntity(world);
        Velox_AddTransform(world, weight1, leftX, weightY, 0.0f);
        Velox_AddRigidBody(world, weight1, 2.0f, false);
        Velox_AddMovement(world, weight1);
        Velox_AddBoxCollider(world, weight1, 40.0f, 40.0f);
        VisualEntity w1Ve; w1Ve.id = weight1; w1Ve.color = PURPLE; w1Ve.type = 1; w1Ve.width = 40.0f; w1Ve.height = 40.0f;
        entities.push_back(w1Ve);

        auto weight2 = Velox_CreateEntity(world);
        Velox_AddTransform(world, weight2, rightX, weightY, 0.0f);
        Velox_AddRigidBody(world, weight2, 4.0f, false);
        Velox_AddMovement(world, weight2);
        Velox_AddBoxCollider(world, weight2, 40.0f, 40.0f);
        VisualEntity w2Ve; w2Ve.id = weight2; w2Ve.color = MAGENTA; w2Ve.type = 1; w2Ve.width = 40.0f; w2Ve.height = 40.0f;
        entities.push_back(w2Ve);

        Velox_AddPulleyJoint(world, weight1, weight2, leftX, groundY, rightX, groundY, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 300.0f, 0.0f);
    }
}

void SetupCCDShowcase(VeloxWorld* world, std::vector<VisualEntity>& entities, int screenWidth, int screenHeight) {
    AddScreenBoundaries(world, entities, screenWidth, screenHeight);
    g_sandboxShapes.clear();
    g_jointVisuals.clear();

    Velox_SetGravity(world, 0.0f, 0.0f);

    // Thin wall in the middle
    auto wall = Velox_CreateEntity(world);
    Velox_AddTransform(world, wall, screenWidth * 0.5f, screenHeight * 0.5f, 0.0f);
    Velox_AddRigidBody(world, wall, 0.0f, true);
    Velox_AddMovement(world, wall);
    Velox_AddBoxCollider(world, wall, 15.0f, screenHeight - 200.0f);
    VisualEntity wallVe; wallVe.id = wall; wallVe.color = DARKGRAY; wallVe.type = 1; wallVe.width = 15.0f; wallVe.height = screenHeight - 200.0f;
    entities.push_back(wallVe);

    // High speed bullet (CCD Showcase)
    auto bullet = Velox_CreateEntity(world);
    Velox_AddTransform(world, bullet, 100.0f, screenHeight * 0.5f, 0.0f);
    Velox_AddRigidBody(world, bullet, 1.0f, false);
    Velox_AddMovement(world, bullet);
    Velox_AddCircleCollider(world, bullet, 10.0f);
    Velox_AddPhysicalMaterial(world, bullet, 0.2f, 0.1f, 0.9f);
    Velox_SetVelocity(world, bullet, 35000.0f, 0.0f); // high speed bullet
    
    VisualEntity bulletVe; bulletVe.id = bullet; bulletVe.color = RED; bulletVe.type = 0; bulletVe.radius = 10.0f;
    entities.push_back(bulletVe);
}

void SetupSleepingShowcase(VeloxWorld* world, std::vector<VisualEntity>& entities, int screenWidth, int screenHeight) {
    AddScreenBoundaries(world, entities, screenWidth, screenHeight);
    g_sandboxShapes.clear();
    g_jointVisuals.clear();

    Velox_SetGravity(world, 0.0f, 400.0f);

    // Build a stack of boxes
    float boxSize = 35.0f;
    float startX = screenWidth * 0.5f;
    float startY = screenHeight - 20.0f - 20.0f - boxSize/2.0f; // resting directly on bottom boundary floor (startY = 662.5f)

    for (int row = 0; row < 6; ++row) {
        auto box = Velox_CreateEntity(world);
        float bx = startX;
        float by = startY - row * boxSize;
        Velox_AddTransform(world, box, bx, by, 0.0f);
        Velox_AddRigidBody(world, box, 1.0f, false);
        Velox_AddMovement(world, box);
        Velox_AddBoxCollider(world, box, boxSize, boxSize);
        Velox_AddPhysicalMaterial(world, box, 0.9f, 0.8f, 0.0f); // High friction, zero bounce
        Velox_SetDamping(world, box, 0.4f, 0.4f);

        VisualEntity ve; ve.id = box; ve.color = GOLD; ve.type = 1; ve.width = boxSize; ve.height = boxSize;
        entities.push_back(ve);
    }

    // Heavy trigger ball to hit the stack
    auto triggerBall = Velox_CreateEntity(world);
    Velox_AddTransform(world, triggerBall, screenWidth * 0.2f, screenHeight * 0.5f, 0.0f);
    Velox_AddRigidBody(world, triggerBall, 10.0f, false);
    Velox_AddMovement(world, triggerBall);
    Velox_AddCircleCollider(world, triggerBall, 30.0f);
    Velox_SetVelocity(world, triggerBall, 350.0f, -50.0f);
    
    VisualEntity tbVe; tbVe.id = triggerBall; tbVe.color = ORANGE; tbVe.type = 0; tbVe.radius = 30.0f;
    entities.push_back(tbVe);
}

void DrawSoftBodyVisual(VeloxWorld* world, const SoftBodyVisual& sbv) {
    int n = sbv.nodes.size();
    if (n < 3) return;

    std::vector<Vector2> pts(n);
    Vector2 center = {0.0f, 0.0f};

    for (int i = 0; i < n; ++i) {
        float x, y, rot;
        Velox_GetPosition(world, sbv.nodes[i], &x, &y, &rot);
        pts[i] = {x, y};
        center.x += x;
        center.y += y;
    }
    center.x /= n;
    center.y /= n;

    // Draw filled polygon using triangle fan from center
    for (int i = 0; i < n; ++i) {
        int next = (i + 1) % n;
        DrawTriangle(center, pts[i], pts[next], Fade(sbv.color, 0.5f));
    }

    // Draw outline
    for (int i = 0; i < n; ++i) {
        int next = (i + 1) % n;
        DrawLineEx(pts[i], pts[next], 3.0f, sbv.color);
        DrawCircleV(pts[i], 3.5f, WHITE);
    }
}

void SetupSoftBodySandbox(VeloxWorld* world, std::vector<VisualEntity>& entities, int screenWidth, int screenHeight) {
    AddScreenBoundaries(world, entities, screenWidth, screenHeight);
    g_softBodyVisuals.clear();
    g_jointVisuals.clear();

    Velox_SetGravity(world, 0.0f, 400.0f);

    // 1. Static obstacles to bounce off
    auto circleObstacle = Velox_CreateEntity(world);
    Velox_AddTransform(world, circleObstacle, screenWidth * 0.3f, screenHeight * 0.65f, 0.0f);
    Velox_AddRigidBody(world, circleObstacle, 0.0f, true);
    Velox_AddMovement(world, circleObstacle);
    Velox_AddCircleCollider(world, circleObstacle, 50.0f);
    entities.push_back({circleObstacle, GRAY, 50.0f, 0.0f, 0.0f, 0, 0.0f, false, nullptr});

    auto boxObstacle = Velox_CreateEntity(world);
    Velox_AddTransform(world, boxObstacle, screenWidth * 0.7f, screenHeight * 0.65f, 0.35f); // Rotated
    Velox_AddRigidBody(world, boxObstacle, 0.0f, true);
    Velox_AddMovement(world, boxObstacle);
    Velox_AddBoxCollider(world, boxObstacle, 160.0f, 40.0f);
    entities.push_back({boxObstacle, GRAY, 0.0f, 160.0f, 40.0f, 1, 0.0f, false, nullptr});

    // 2. Blob soft body (Area preserved)
    auto blob = Velox_CreateSoftBodyBlob(world, screenWidth * 0.3f, screenHeight * 0.25f, 65.0f, 16, 0.04f, 0.05f, 10.0f);
    SoftBodyVisual sbv1;
    sbv1.managerId = blob;
    sbv1.color = SKYBLUE;
    int count = Velox_GetSoftBodyNodeCount(world, blob);
    for (int i = 0; i < count; ++i) {
        sbv1.nodes.push_back(Velox_GetSoftBodyNode(world, blob, i));
    }
    g_softBodyVisuals.push_back(sbv1);

    // 3. Shape matched soft body (Star shape)
    float starVertsX[] = { 0, 20, 65, 30, 45, 0, -45, -30, -65, -20 };
    float starVertsY[] = { -65, -20, -20, 10, 55, 30, 55, 10, -20, -20 };
    int starCount = 10;
    auto star = Velox_CreateSoftBodyShapeMatched(world, screenWidth * 0.7f, screenHeight * 0.25f, starVertsX, starVertsY, starCount, 0.02f, 9.0f);
    SoftBodyVisual sbv2;
    sbv2.managerId = star;
    sbv2.color = LIME;
    int count2 = Velox_GetSoftBodyNodeCount(world, star);
    for (int i = 0; i < count2; ++i) {
        sbv2.nodes.push_back(Velox_GetSoftBodyNode(world, star, i));
    }
    g_softBodyVisuals.push_back(sbv2);
}

void SetupSoftBodyFunnel(VeloxWorld* world, std::vector<VisualEntity>& entities, int screenWidth, int screenHeight) {
    AddScreenBoundaries(world, entities, screenWidth, screenHeight);
    g_softBodyVisuals.clear();
    g_jointVisuals.clear();

    Velox_SetGravity(world, 0.0f, 450.0f);

    // Funnel Dimensions
    float wallH = 25.0f;
    float flatW = 380.0f;
    float slopeW = 290.0f;

    // 1. Horizontal Flat Top Left Wall (___) - Shifted out slightly for spacing
    auto leftFlat = Velox_CreateEntity(world);
    Velox_AddTransform(world, leftFlat, screenWidth * 0.5f - 350.0f, screenHeight * 0.22f, 0.0f);
    Velox_AddRigidBody(world, leftFlat, 0.0f, true);
    Velox_AddMovement(world, leftFlat);
    Velox_AddBoxCollider(world, leftFlat, flatW, wallH);
    Velox_AddPhysicalMaterial(world, leftFlat, 0.1f, 0.05f, 0.4f);
    entities.push_back({leftFlat, GRAY, 0.0f, flatW, wallH, 1, 0.0f, false, nullptr});

    // 2. Horizontal Flat Top Right Wall (___) - Shifted out slightly for spacing
    auto rightFlat = Velox_CreateEntity(world);
    Velox_AddTransform(world, rightFlat, screenWidth * 0.5f + 350.0f, screenHeight * 0.22f, 0.0f);
    Velox_AddRigidBody(world, rightFlat, 0.0f, true);
    Velox_AddMovement(world, rightFlat);
    Velox_AddBoxCollider(world, rightFlat, flatW, wallH);
    Velox_AddPhysicalMaterial(world, rightFlat, 0.1f, 0.05f, 0.4f);
    entities.push_back({rightFlat, GRAY, 0.0f, flatW, wallH, 1, 0.0f, false, nullptr});

    // 3. Left Diagonal Slope (\) - Shifted very close to center to restrict exit hole
    auto leftSlope = Velox_CreateEntity(world);
    Velox_AddTransform(world, leftSlope, screenWidth * 0.5f - 145.0f, screenHeight * 0.40f, 0.65f);
    Velox_AddRigidBody(world, leftSlope, 0.0f, true);
    Velox_AddMovement(world, leftSlope);
    Velox_AddBoxCollider(world, leftSlope, slopeW, wallH);
    Velox_AddPhysicalMaterial(world, leftSlope, 0.1f, 0.05f, 0.4f);
    entities.push_back({leftSlope, GRAY, 0.0f, slopeW, wallH, 1, 0.65f, false, nullptr});

    // 4. Right Diagonal Slope (/) - Shifted very close to center to restrict exit hole
    auto rightSlope = Velox_CreateEntity(world);
    Velox_AddTransform(world, rightSlope, screenWidth * 0.5f + 145.0f, screenHeight * 0.40f, -0.65f);
    Velox_AddRigidBody(world, rightSlope, 0.0f, true);
    Velox_AddMovement(world, rightSlope);
    Velox_AddBoxCollider(world, rightSlope, slopeW, wallH);
    Velox_AddPhysicalMaterial(world, rightSlope, 0.1f, 0.05f, 0.4f);
    entities.push_back({rightSlope, GRAY, 0.0f, slopeW, wallH, 1, -0.65f, false, nullptr});

    // Pre-spawning of soft bodies is removed from SetupSoftBodyFunnel
    // They are now spawned dynamically in the update loop 1 second apart.
}

void SetupSoftBodyStacking(VeloxWorld* world, std::vector<VisualEntity>& entities, int screenWidth, int screenHeight) {
    AddScreenBoundaries(world, entities, screenWidth, screenHeight);
    g_softBodyVisuals.clear();
    g_jointVisuals.clear();

    Velox_SetGravity(world, 0.0f, 400.0f);

    // Large Blob at bottom serving as soft mattress
    auto mattress = Velox_CreateSoftBodyBlob(world, screenWidth * 0.5f, screenHeight * 0.75f, 90.0f, 20, 0.02f, 0.03f, 12.0f);
    SoftBodyVisual sbv;
    sbv.managerId = mattress;
    sbv.color = MAGENTA;
    int count = Velox_GetSoftBodyNodeCount(world, mattress);
    for (int i = 0; i < count; ++i) {
        sbv.nodes.push_back(Velox_GetSoftBodyNode(world, mattress, i));
    }
    g_softBodyVisuals.push_back(sbv);

    // Rigid heavy dynamic box to fall on the soft body
    auto box = Velox_CreateEntity(world);
    Velox_AddTransform(world, box, screenWidth * 0.48f, screenHeight * 0.35f, 0.0f);
    Velox_AddRigidBody(world, box, 12.0f, false);
    Velox_AddMovement(world, box);
    Velox_AddBoxCollider(world, box, 60.0f, 60.0f);
    Velox_AddPhysicalMaterial(world, box, 0.8f, 0.6f, 0.1f);
    entities.push_back({box, YELLOW, 0.0f, 60.0f, 60.0f, 1, 0.0f, false, nullptr});

    // Rigid heavy ball to fall on the soft body
    auto ball = Velox_CreateEntity(world);
    Velox_AddTransform(world, ball, screenWidth * 0.53f, screenHeight * 0.15f, 0.0f);
    Velox_AddRigidBody(world, ball, 8.0f, false);
    Velox_AddMovement(world, ball);
    Velox_AddCircleCollider(world, ball, 35.0f);
    Velox_AddPhysicalMaterial(world, ball, 0.5f, 0.3f, 0.1f);
    entities.push_back({ball, ORANGE, 35.0f, 0.0f, 0.0f, 0, 0.0f, false, nullptr});
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
            g_softBodyVisuals.clear();
            frameRotation = 0.0f;

            if (currentScene == SceneType::BouncingBalls) {
                std::cout << "[SCENE] Loading scene: Bouncing Balls" << std::endl;
                SetupBouncingBalls(world, entities, screenWidth, screenHeight);
            } else if (currentScene == SceneType::ForceFieldDemo) {
                std::cout << "[SCENE] Loading scene: Force Field Demo" << std::endl;
                SetupForceFieldDemo(world, entities, screenWidth, screenHeight);
            } else if (currentScene == SceneType::OscillationDemo) {
                std::cout << "[SCENE] Loading scene: Oscillation Demo" << std::endl;
                SetupOscillationDemo(world, entities, screenWidth, screenHeight);
            } else if (currentScene == SceneType::ProjectileDemo) {
                std::cout << "[SCENE] Loading scene: Projectile Demo" << std::endl;
                SetupProjectileDemo(world, entities, screenWidth, screenHeight);
            } else if (currentScene == SceneType::GravityDemo) {
                std::cout << "[SCENE] Loading scene: Gravity Demo" << std::endl;
                SetupGravityDemo(world, entities, screenWidth, screenHeight);
            } else if (currentScene == SceneType::JointDemo) {
                std::cout << "[SCENE] Loading scene: Joint Demo" << std::endl;
                SetupJointDemo(world, entities, screenWidth, screenHeight);
            } else if (currentScene == SceneType::SandboxDemo) {
                std::cout << "[SCENE] Loading scene: Sandbox Demo" << std::endl;
                SetupSandboxDemo(world, entities, screenWidth, screenHeight);
            } else if (currentScene == SceneType::ChainDemo) {
                std::cout << "[SCENE] Loading scene: Chain Demo" << std::endl;
                SetupChainDemo(world, entities, screenWidth, screenHeight);
            } else if (currentScene == SceneType::RaycastDemo) {
                std::cout << "[SCENE] Loading scene: Raycast Demo" << std::endl;
                SetupRaycastDemo(world, entities, screenWidth, screenHeight);
            } else if (currentScene == SceneType::RevolutePrismaticDemo) {
                std::cout << "[SCENE] Loading scene: Revolute Prismatic Demo" << std::endl;
                SetupRevolutePrismaticDemo(world, entities, screenWidth, screenHeight);
            } else if (currentScene == SceneType::CCDShowcase) {
                std::cout << "[SCENE] Loading scene: CCD Showcase" << std::endl;
                SetupCCDShowcase(world, entities, screenWidth, screenHeight);
            } else if (currentScene == SceneType::SleepingShowcase) {
                std::cout << "[SCENE] Loading scene: Sleeping Showcase" << std::endl;
                SetupSleepingShowcase(world, entities, screenWidth, screenHeight);
            } else if (currentScene == SceneType::SoftBodySandbox) {
                std::cout << "[SCENE] Loading scene: Soft Body Sandbox" << std::endl;
                SetupSoftBodySandbox(world, entities, screenWidth, screenHeight);
            } else if (currentScene == SceneType::SoftBodyFunnel) {
                std::cout << "[SCENE] Loading scene: Soft Body Funnel & Squeeze" << std::endl;
                SetupSoftBodyFunnel(world, entities, screenWidth, screenHeight);
            } else if (currentScene == SceneType::SoftBodyStacking) {
                std::cout << "[SCENE] Loading scene: Cushion Stacking Showcase" << std::endl;
                SetupSoftBodyStacking(world, entities, screenWidth, screenHeight);
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
            Velox_AddProjectile(world, id, true, 0.0f, 1000.0f, 0.01f);
            
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
            ve.isProjectile = false;
            entities.push_back(ve);
        }
        
        // Directional Gravity control (Gravity Demo) — WASD rotates gravity direction
        if (currentScene == SceneType::GravityDemo) {
            static float gravAngle = 1.5708f; // pi/2 = straight down
            float rotSpeed = 2.0f * (1.0f / 60.0f);
            if (IsKeyDown(KEY_A)) gravAngle -= rotSpeed * 2.0f;
            if (IsKeyDown(KEY_D)) gravAngle += rotSpeed * 2.0f;
            if (IsKeyPressed(KEY_W)) gravAngle = -1.5708f; // straight up
            if (IsKeyPressed(KEY_S)) gravAngle =  1.5708f; // straight down
            float gMag = 980.0f;
            Velox_SetGravity(world, cosf(gravAngle) * gMag, sinf(gravAngle) * gMag);
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
            
            // Perform mouse Raycast in Raycast Demo only
            if (currentScene == SceneType::RaycastDemo) {
                Vector2 mouse = GetMousePosition();
                g_raycastStart = { (float)screenWidth * 0.1f, 80.0f };
                g_raycastEnd = mouse;
                
                float dx = g_raycastEnd.x - g_raycastStart.x;
                float dy = g_raycastEnd.y - g_raycastStart.y;
                float dist = std::sqrt(dx*dx + dy*dy);
                
                float hx, hy, nx, ny, frac;
                Velox::EntityID hitId;
                g_raycastHit = Velox_Raycast(world, g_raycastStart.x, g_raycastStart.y, dx, dy, dist, &hx, &hy, &nx, &ny, &frac, &hitId);
                if (g_raycastHit) {
                    g_raycastHitPt = {hx, hy};
                    g_raycastNormal = {nx, ny};
                    g_raycastFrac = frac;
                }
            }

            // --- Dynamic Soft Body Spawner (Funnel Demo) ---
            if (currentScene == SceneType::SoftBodyFunnel) {
                static float funnelSpawnTimer = 0.0f;
                static int nextFunnelSpawnIndex = 0;
                
                // Reset spawns when scene is reloaded/changed
                static SceneType lastScene = SceneType::BouncingBalls;
                if (lastScene != currentScene) {
                    funnelSpawnTimer = 0.0f;
                    nextFunnelSpawnIndex = 0;
                    lastScene = currentScene;
                }

                funnelSpawnTimer += dt;
                
                // Spawn one body every 1.0 second, up to 4 soft bodies
                if (funnelSpawnTimer >= 1.0f && nextFunnelSpawnIndex < 4) {
                    funnelSpawnTimer = 0.0f;
                    float spawnX = screenWidth * 0.5f; // Vertical center line
                    float spawnY = screenHeight * 0.08f; // Spawning height above funnel

                    if (nextFunnelSpawnIndex == 0) {
                        // Spawn 1st Blob (Orange)
                        auto blob1 = Velox_CreateSoftBodyBlob(world, spawnX, spawnY, 45.0f, 12, 0.04f, 0.05f, 9.0f);
                        SoftBodyVisual sbv1;
                        sbv1.managerId = blob1;
                        sbv1.color = ORANGE;
                        for (int i = 0; i < 12; ++i) sbv1.nodes.push_back(Velox_GetSoftBodyNode(world, blob1, i));
                        g_softBodyVisuals.push_back(sbv1);
                    } else if (nextFunnelSpawnIndex == 1) {
                        // Spawn 1st Star (Pink)
                        float starVertsX[] = { 0, 15, 50, 22, 35, 0, -35, -22, -50, -15 };
                        float starVertsY[] = { -50, -15, -15, 8, 42, 22, 42, 8, -15, -15 };
                        auto star1 = Velox_CreateSoftBodyShapeMatched(world, spawnX, spawnY, starVertsX, starVertsY, 10, 0.02f, 8.0f);
                        SoftBodyVisual sbv2;
                        sbv2.managerId = star1;
                        sbv2.color = PINK;
                        for (int i = 0; i < 10; ++i) sbv2.nodes.push_back(Velox_GetSoftBodyNode(world, star1, i));
                        g_softBodyVisuals.push_back(sbv2);
                    } else if (nextFunnelSpawnIndex == 2) {
                        // Spawn 2nd Blob (Green)
                        auto blob2 = Velox_CreateSoftBodyBlob(world, spawnX, spawnY, 40.0f, 12, 0.04f, 0.05f, 9.0f);
                        SoftBodyVisual sbv3;
                        sbv3.managerId = blob2;
                        sbv3.color = GREEN;
                        for (int i = 0; i < 12; ++i) sbv3.nodes.push_back(Velox_GetSoftBodyNode(world, blob2, i));
                        g_softBodyVisuals.push_back(sbv3);
                    } else if (nextFunnelSpawnIndex == 3) {
                        // Spawn 2nd Star/Shape (Blue)
                        float hexVertsX[] = { 0, 35, 35, 0, -35, -35 };
                        float hexVertsY[] = { -40, -20, 20, 40, 20, -20 };
                        auto star2 = Velox_CreateSoftBodyShapeMatched(world, spawnX, spawnY, hexVertsX, hexVertsY, 6, 0.03f, 8.0f);
                        SoftBodyVisual sbv4;
                        sbv4.managerId = star2;
                        sbv4.color = SKYBLUE;
                        for (int i = 0; i < 6; ++i) sbv4.nodes.push_back(Velox_GetSoftBodyNode(world, star2, i));
                        g_softBodyVisuals.push_back(sbv4);
                    }
                    nextFunnelSpawnIndex++;
                }
            }

            Velox_Step(world, dt);
        }

        // Draw
        BeginDrawing();
            ClearBackground(Color{15, 15, 20, 255}); // Dark Mode

            // --- Draw Distance Joints ---
            if (currentScene == SceneType::JointDemo) {
                for (const auto& jv : g_jointVisuals) {
                    float ax, ay, ar, bx, by, br;
                    Velox_GetPosition(world, jv.idA, &ax, &ay, &ar);
                    Velox_GetPosition(world, jv.idB, &bx, &by, &br);
                    DrawLineEx({ax, ay}, {bx, by}, 2.5f, jv.color);
                }
            }

            // --- Custom Joint Renderings for Revolute & Prismatic Showcase ---
            if (currentScene == SceneType::RevolutePrismaticDemo) {
                for (const auto& jv : g_jointVisuals) {
                    float ax, ay, ar, bx, by, br;
                    Velox_GetPosition(world, jv.idA, &ax, &ay, &ar);
                    Velox_GetPosition(world, jv.idB, &bx, &by, &br);
                    DrawLineEx({ax, ay}, {bx, by}, 2.0f, jv.color);
                }

                // 1. Draw Prismatic Slider Rail
                float railY = screenHeight * 0.3f;
                float railMinX = screenWidth * 0.5f - 150.0f;
                float railMaxX = screenWidth * 0.5f + 150.0f;
                DrawLineEx({railMinX, railY}, {railMaxX, railY}, 2.0f, Fade(SKYBLUE, 0.4f));
                DrawCircleV({railMinX, railY}, 4.0f, SKYBLUE);
                DrawCircleV({railMaxX, railY}, 4.0f, SKYBLUE);
                DrawText("Slider Axis Guide", (int)railMinX, (int)railY - 15, 10, SKYBLUE);

                // 2. Draw Pulley Cables
                float leftX = screenWidth * 0.4f;
                float rightX = screenWidth * 0.6f;
                float groundY = screenHeight * 0.6f;

                float w1x = 0, w1y = 0, w2x = 0, w2y = 0;
                for (const auto& ve : entities) {
                    if (ve.color.r == PURPLE.r && ve.color.g == PURPLE.g && ve.color.b == PURPLE.b) {
                        float rot;
                        Velox_GetPosition(world, ve.id, &w1x, &w1y, &rot);
                    }
                    if (ve.color.r == MAGENTA.r && ve.color.g == MAGENTA.g && ve.color.b == MAGENTA.b) {
                        float rot;
                        Velox_GetPosition(world, ve.id, &w2x, &w2y, &rot);
                    }
                }

                if (w1x != 0 && w2x != 0) {
                    DrawLineEx({w1x, w1y}, {leftX, groundY}, 2.0f, LIGHTGRAY);
                    DrawLineEx({leftX, groundY}, {rightX, groundY}, 2.0f, LIGHTGRAY);
                    DrawLineEx({rightX, groundY}, {w2x, w2y}, 2.0f, LIGHTGRAY);

                    // Draw pulley wheel circles
                    DrawCircleV({leftX, groundY}, 12.0f, DARKGRAY);
                    DrawCircleLines((int)leftX, (int)groundY, 12, WHITE);
                    DrawCircleV({rightX, groundY}, 12.0f, DARKGRAY);
                    DrawCircleLines((int)rightX, (int)groundY, 12, WHITE);
                }

                // 3. Draw Labels for clarity
                DrawText("Revolute Hinge", (int)(screenWidth * 0.25f) - 40, (int)(screenHeight * 0.3f) - 60, 11, ORANGE);
                DrawText("Prismatic Slider", (int)(screenWidth * 0.5f) - 40, (int)(screenHeight * 0.3f) - 60, 11, SKYBLUE);
                DrawText("Coupled Gears", (int)(screenWidth * 0.75f) - 30, (int)(screenHeight * 0.3f) - 60, 11, GREEN);
                DrawText("Pulley System", (int)(screenWidth * 0.5f) - 40, (int)(screenHeight * 0.6f) - 30, 11, PURPLE);
            }

            // --- Draw Gravity Arrow (Gravity Demo) ---
            if (currentScene == SceneType::GravityDemo) {
                static float gravAngle = 1.5708f;
                // Re-read from controls (kept in sync via same static)
                float gx = 0.0f, gy = 0.0f;
                if (IsKeyDown(KEY_A)) gravAngle -= 2.0f * (1.0f/60.0f) * 2.0f;
                if (IsKeyDown(KEY_D)) gravAngle += 2.0f * (1.0f/60.0f) * 2.0f;
                // Draw arrow
                float cx = 100.0f, cy = 660.0f;
                float len = 60.0f;
                float ex = cx + cosf(gravAngle) * len;
                float ey = cy + sinf(gravAngle) * len;
                DrawLineEx({cx, cy}, {ex, ey}, 3.0f, {255, 220, 60, 220});
                DrawCircleV({ex, ey}, 7.0f, {255, 220, 60, 255});
                DrawText("Gravity", (int)cx - 30, (int)cy + 12, 10, {255, 220, 60, 200});
            }

            // --- Draw Sandbox & Chain Shapes (Polygons & Chains) ---
            if (currentScene == SceneType::SandboxDemo || currentScene == SceneType::ChainDemo) {
                for (const auto& sb : g_sandboxShapes) {
                    float x, y, rot;
                    Velox_GetPosition(world, sb.id, &x, &y, &rot);
                    
                    if (sb.type == 0) { // Circle
                        DrawCircleV({x, y}, 10.0f, sb.color);
                    } else if (sb.type == 2) { // Polygon
                        int count = sb.pts.size();
                        for (int i = 0; i < count; ++i) {
                            Vector2 localP1 = sb.pts[i];
                            Vector2 localP2 = sb.pts[(i + 1) % count];
                            float cosRot = std::cos(rot);
                            float sinRot = std::sin(rot);
                            Vector2 p1 = { x + (localP1.x * cosRot - localP1.y * sinRot), y + (localP1.x * sinRot + localP1.y * cosRot) };
                            Vector2 p2 = { x + (localP2.x * cosRot - localP2.y * sinRot), y + (localP2.x * sinRot + localP2.y * cosRot) };
                            DrawLineEx(p1, p2, 2.5f, sb.color);
                        }
                    } else if (sb.type == 3) { // Chain
                        int count = sb.pts.size();
                        for (int i = 0; i < count - 1; ++i) {
                            DrawLineEx(sb.pts[i], sb.pts[i+1], 4.0f, sb.color);
                        }
                    }
                }
            }

            // Draw Raycast overlays
            if (currentScene == SceneType::RaycastDemo) {
                DrawLineEx(g_raycastStart, g_raycastEnd, 1.5f, Fade(SKYBLUE, 0.4f));
                if (g_raycastHit) {
                    DrawLineEx(g_raycastStart, g_raycastHitPt, 2.0f, RED);
                    DrawCircleV(g_raycastHitPt, 6.0f, RED);
                    Vector2 normalEnd = { g_raycastHitPt.x + g_raycastNormal.x * 25.0f, g_raycastHitPt.y + g_raycastNormal.y * 25.0f };
                    DrawLineEx(g_raycastHitPt, normalEnd, 2.0f, GREEN);
                }
            }

            for (const auto& ve : entities) {
                float x, y, rot;
                Velox_GetPosition(world, ve.id, &x, &y, &rot);
                
                Color drawColor = ve.color;
                if (Velox_IsSleeping(world, ve.id)) {
                    drawColor = DARKGRAY;
                }
                
                if (ve.type == 0) { // Circle (Ball)
                    DrawCircleV({x, y}, ve.radius, drawColor);
                    // Subtle shine highlight
                    DrawCircleV({x - ve.radius*0.25f, y - ve.radius*0.25f}, ve.radius*0.35f, Fade(WHITE, 0.25f));
                    DrawLineEx({x, y}, {x + cosf(rot)*ve.radius, y + sinf(rot)*ve.radius}, 2.0f, Fade(BLACK, 0.5f));
                } else if (ve.type == 1) { // Box
                    DrawRectanglePro({x, y, ve.width, ve.height}, {ve.width/2, ve.height/2}, rot * RAD2DEG, drawColor);
                } else if (ve.type == 2) { // Force Field (visual)
                    DrawCircleV({x, y}, ve.radius, ve.color);
                    DrawCircleLines((int)x, (int)y, (int)ve.radius, Fade(WHITE, 0.6f));
                }
            }
            for (const auto& sbv : g_softBodyVisuals) {
                DrawSoftBodyVisual(world, sbv);
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
            } else if (currentScene == SceneType::GravityDemo) {
                DrawText("Sim: Gravity Direction Demo", widgetX + 10, widgetY + 60, 10, YELLOW);
                DrawText("Controls:", widgetX + 10, widgetY + 80, 10, GRAY);
                DrawText("- A/D: Rotate Gravity Vector", widgetX + 10, widgetY + 95, 10, GRAY);
                DrawText("- W: Set Gravity Up", widgetX + 10, widgetY + 110, 10, GRAY);
                DrawText("- S: Set Gravity Down", widgetX + 10, widgetY + 125, 10, GRAY);
                DrawText("- Space: Pause", widgetX + 10, widgetY + 140, 10, GRAY);
            } else if (currentScene == SceneType::JointDemo) {
                DrawText("Sim: Distance Joint Demo", widgetX + 10, widgetY + 60, 10, YELLOW);
                DrawText("Features:", widgetX + 10, widgetY + 80, 10, GRAY);
                DrawText("- Left: Pendulum Chain", widgetX + 10, widgetY + 95, 10, GRAY);
                DrawText("- Center: Chaotic Double Pendulum", widgetX + 10, widgetY + 110, 10, GRAY);
                DrawText("- Right: Newton's Cradle (Elastic)", widgetX + 10, widgetY + 125, 10, GRAY);
                DrawText("- Space: Pause", widgetX + 10, widgetY + 140, 10, GRAY);
            } else if (currentScene == SceneType::SandboxDemo) {
                DrawText("Sim: Convex Polygons & Motors", widgetX + 10, widgetY + 60, 10, YELLOW);
                DrawText("Showcased Features:", widgetX + 10, widgetY + 80, 9, {255, 220, 60, 255});
                DrawText("- Triangle: Custom Polygon collider", widgetX + 10, widgetY + 95, 9, GRAY);
                DrawText("- Carousel: Motorized distance joint", widgetX + 10, widgetY + 110, 9, GRAY);
                DrawText("- Falling shapes: Pentagons (SAT)", widgetX + 10, widgetY + 125, 9, GRAY);
            } else if (currentScene == SceneType::ChainDemo) {
                DrawText("Sim: Chain Shapes Showcase", widgetX + 10, widgetY + 60, 10, YELLOW);
                DrawText("Showcased Features:", widgetX + 10, widgetY + 80, 9, {255, 220, 60, 255});
                DrawText("- Wavy floor: Sinusoidal Chain collider", widgetX + 10, widgetY + 95, 9, GRAY);
                DrawText("- Interactive: Falling circles & boxes", widgetX + 10, widgetY + 110, 9, GRAY);
                DrawText("- Math: Segmented SAT resolution", widgetX + 10, widgetY + 125, 9, GRAY);
            } else if (currentScene == SceneType::RaycastDemo) {
                DrawText("Sim: Raycast Queries Showcase", widgetX + 10, widgetY + 60, 10, YELLOW);
                DrawText("Showcased Features:", widgetX + 10, widgetY + 80, 9, {255, 220, 60, 255});
                DrawText("- Laser line: projects from top-left", widgetX + 10, widgetY + 95, 9, GRAY);
                DrawText("- Interaction: Casts to mouse point", widgetX + 10, widgetY + 110, 9, GRAY);
                DrawText("- Highlight: Red line to hit point", widgetX + 10, widgetY + 125, 9, GRAY);
                DrawText("- Vector: Green line shows hit normal", widgetX + 10, widgetY + 140, 9, GRAY);
            } else if (currentScene == SceneType::RevolutePrismaticDemo) {
                DrawText("Sim: Revolute & Prismatic Joints", widgetX + 10, widgetY + 60, 10, YELLOW);
                DrawText("Showcased Features:", widgetX + 10, widgetY + 80, 9, {255, 220, 60, 255});
                DrawText("- Orange blade: Revolute Hinge Motor", widgetX + 10, widgetY + 95, 9, GRAY);
                DrawText("- Blue box: Prismatic Slider Motor", widgetX + 10, widgetY + 110, 9, GRAY);
                DrawText("- Green/Lime: Coupled Gear rotation", widgetX + 10, widgetY + 125, 9, GRAY);
                DrawText("- Purple boxes: Interactive Pulley link", widgetX + 10, widgetY + 140, 9, GRAY);
            } else if (currentScene == SceneType::CCDShowcase) {
                DrawText("Sim: CCD vs Tunneling", widgetX + 10, widgetY + 60, 10, YELLOW);
                DrawText("Showcased Features:", widgetX + 10, widgetY + 80, 9, {255, 220, 60, 255});
                DrawText("- Red Ball: Hyper-speed bullet (3500px/s)", widgetX + 10, widgetY + 95, 9, GRAY);
                DrawText("- Thin wall: Bounces bullet perfectly", widgetX + 10, widgetY + 110, 9, GRAY);
                DrawText("- Precision: Zero tunneling through math CCD", widgetX + 10, widgetY + 125, 9, GRAY);
            } else if (currentScene == SceneType::SleepingShowcase) {
                DrawText("Sim: Sleeping & Activation", widgetX + 10, widgetY + 60, 10, YELLOW);
                DrawText("Showcased Features:", widgetX + 10, widgetY + 80, 9, {255, 220, 60, 255});
                DrawText("- Dark Gray: Sleeping bodies (deactivated)", widgetX + 10, widgetY + 95, 9, GRAY);
                DrawText("- Gold/Orange: Active awake bodies", widgetX + 10, widgetY + 110, 9, GRAY);
                DrawText("- Launch: Trigger ball wakes up stack", widgetX + 10, widgetY + 125, 9, GRAY);
            } else if (currentScene == SceneType::SoftBodySandbox) {
                DrawText("Sim: Soft Body Sandbox", widgetX + 10, widgetY + 60, 10, YELLOW);
                DrawText("Showcased Features:", widgetX + 10, widgetY + 80, 9, {255, 220, 60, 255});
                DrawText("- Blue Blob: Area preserved squishy body", widgetX + 10, widgetY + 95, 9, GRAY);
                DrawText("- Green Star: Shape-matched elastic shape", widgetX + 10, widgetY + 110, 9, GRAY);
                DrawText("- Interact: Obstacles deform soft bodies", widgetX + 10, widgetY + 125, 9, GRAY);
            } else if (currentScene == SceneType::SoftBodyFunnel) {
                DrawText("Sim: Squeeze & Funnel Demo", widgetX + 10, widgetY + 60, 10, YELLOW);
                DrawText("Showcased Features:", widgetX + 10, widgetY + 80, 9, {255, 220, 60, 255});
                DrawText("- Funnel slopes: static angled walls", widgetX + 10, widgetY + 95, 9, GRAY);
                DrawText("- Deform: Soft shapes squeeze through gap", widgetX + 10, widgetY + 110, 9, GRAY);
                DrawText("- Elasticity: Recover original rest shape", widgetX + 10, widgetY + 125, 9, GRAY);
            } else if (currentScene == SceneType::SoftBodyStacking) {
                DrawText("Sim: Cushion Stacking Showcase", widgetX + 10, widgetY + 60, 10, YELLOW);
                DrawText("Showcased Features:", widgetX + 10, widgetY + 80, 9, {255, 220, 60, 255});
                DrawText("- Magenta Cushion: squishy soft mattress", widgetX + 10, widgetY + 95, 9, GRAY);
                DrawText("- Heavy load: dynamic box & ball stack", widgetX + 10, widgetY + 110, 9, GRAY);
                DrawText("- Compliance: Cushion sags under weight", widgetX + 10, widgetY + 125, 9, GRAY);
            }
            
            // Combo Box
            int comboX = widgetX;
            int comboY = widgetY + widgetH + 10;
            int comboW = 320;
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
                int sceneCount = sizeof(sceneNames) / sizeof(sceneNames[0]);
                for (int i = 0; i < sceneCount; ++i) {
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
