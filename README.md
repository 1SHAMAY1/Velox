# ⚡ Velox Physics Engine

**Velox** is a lightweight, high-performance 2D physics engine built from scratch in C++. It leverages modern **Data-Oriented Design (ECS)** and an **Extended Position Based Dynamics (XPBD)** solver to deliver stable, scalable, and cache-friendly physics simulations.

---

## 🚀 Key Features

*   ** ECS Architecture**: Custom interactions between *Entity-Component-System* ensure maximum cache locality and performance for thousands of objects.
*   ** XPBD Constraint Solver**: Superior stability for rigid body collisions and joints compared to traditional impulse-based solvers.
*   ** Broadphase Optimization**: Spatial Hashing Grid (60x60 cells) reduces collision checks from O(N²) to O(N) average case.
*   ** Modular Behaviors**: Plug-and-play components for Force Fields, Oscillators, and constant Rotations.
*   ** C-API Export**: Clean, C-style API (`VeloxAPI.h`) for easy integration into other C/C++ applications or game engines.
*   ** Visualizer**: Includes a Raylib-powered visualizer for real-time testing and debugging.

---

## 🏛️ Architecture Overview

Velox separates **Data** from **Logic**:
1.  **EntityManager** (`VelcoxECS`): Stores all Component data in contiguous arrays.
2.  **PhysicsSystem**: The logic core. It doesn't own data; it iterates over Component arrays.

### The Physics Step
The `Velox_Step(dt)` function executes the simulation loop:
1.  **Apply External Forces**: Gravity, Force Fields, and Damping.
2.  **Sub-Stepping Loop** (8 iterations):
    *   **Integrate**: Predict new positions based on velocity.
    *   **Broadphase**: Spatial Grid inserts all colliders.
    *   **Narrowphase & Solve**: Detect overlaps and resolve constraints (XPBD) to correct positions.
    *   **Velocity Update**: Update velocities based on position corrections.

---

## 📦 Components Reference

### 🧱 Core Physics
These components define the physical existence of an object.

| Component | Description | Key Properties |
| :--- | :--- | :--- |
| **Transform** | The object's location in the world. | `Position (x,y)`, `Rotation (radians)` |
| **RigidBody** | Defines mass and dynamics. | `Mass` (0 = Static), `IsStatic` (bool) |
| **Movement** | Internal simulation state (Velocity/Forces). | `Velocity`, `AngularVelocity`, `Damping` |
| **Collider** | The physical shape for collisions. | `Type` (Box/Circle), `Radius`, `Dimensions` |
| **PhysicalMaterial** | Surface interaction properties. | `StaticFriction` (Start moving), `DynamicFriction` (Keep moving), `Restitution` (Bounciness 0.0-1.0) |

### ⚙️ Behaviors
Specialized components to add unique behaviors without writing custom simulation code.

| Component | Description | Usage |
| :--- | :--- | :--- |
| **ForceField** | Creates a radial zone that pushes/pulls. | **Gravity Wells** (Inward), **Explosions** (Outward), **Vortexes** (Rotation). |
| **Oscillation** | Moves an object back and forth (Sine wave). | Platforms, elevators, or decorative floating items. |
| **Rotation** | Constant motor rotation. | Wheels, fans, or spinning hazards. |
| **Projectile** | Rotation always faces Velocity vector. | Arrows, missiles, or rockets. |

---

## � Quick Start Guide

Here is a minimal example of how to initialize Velox and simulate a falling ball.

```cpp
#include <velox/VeloxAPI.h>

int main() {
    // 1. Create the World
    VeloxWorld* world = Velox_CreateWorld();

    // 2. Create an Entity (The Ball)
    int ballID = Velox_CreateEntity(world);

    // 3. Add Components
    Velox_AddTransform(world, ballID, 0.0f, 100.0f, 0.0f);   // Start at Y=100
    Velox_AddRigidBody(world, ballID, 1.0f, false);          // Mass=1kg, Dynamic
    Velox_AddMovement(world, ballID);                        // Enable movement
    Velox_AddCircleCollider(world, ballID, 10.0f);           // Radius=10
    Velox_AddPhysicalMaterial(world, ballID, 0.5f, 0.3f, 0.8f); // High Bounciness (0.8)

    // 4. Simulation Loop
    float dt = 1.0f / 60.0f;
    for (int i = 0; i < 600; ++i) {
        Velox_Step(world, dt);
        
        float x, y, rot;
        Velox_GetPosition(world, ballID, &x, &y, &rot);
        // Print/Draw position...
    }

    // 5. Cleanup
    Velox_DestroyWorld(world);
    return 0;
}
```

---

## 🎮 Included Demos

The project comes with a **Visualizer** that showcases the engine's capabilities.

### 1. Bouncing Balls (Stress Test)
*   **Scenario**: Hundreds of balls spawn and collide within a box.
*   **Showcases**: Broadphase efficiency, collision resolution stability, and restitution.
*   **Interactions**: Use arrow keys to rotate the entire container!

### 2. Force Fields
*   **Scenario**: Particles interacting with invisible zones.
*   **Types**:
    *   🟦 **Inward**: Simulates gravity wells / black holes.
    *   🟥 **Outward**: Simulates repulsor shields.
    *   🟩 **Vortex**: Swirls objects clockwise or anti-clockwise.

### 3. Oscillators
*   **Scenario**: Platforms moving in perfect sine-waves (Horizontal, Vertical, Diagonal).
*   **Mechanic**: Uses `OscillationComponent` to directly manipulate position without affecting physics velocity, allowing bodies to "ride" them.

### 4. Projectiles
*   **Scenario**: Launching arrows with the mouse.
*   **Mechanic**: The `ProjectileComponent` ensures the arrows tip always points forward, even as gravity arcs their path.

---

## 🛠️ Building the Project

### Prerequisites
*   **CMake**: 3.10 or higher.
*   **Compiler**: MSVC (Windows), GCC/Clang (Linux/macOS) supporting C++17.

### Build Instructions
```bash
# 1. Clone the repository
git clone https://github.com/your-repo/velox.git
cd velox

# 2. Create build directory
mkdir build
cd build

# 3. Configure and Build
cmake ..
cmake --build . --config Release
```

### Running the Visualizer
After building, the visualizer executable will be in `bin/Release`.
```bash
# Windows
.\bin\Release\VeloxVisualizer.exe
```
