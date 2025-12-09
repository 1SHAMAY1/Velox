# ⚡ Velox Physics Engine

![Velox Logo](assets/velox_icon_window.png)

**Velox** is a lightweight, high-performance, and modular 2D physics engine written in C++. Designed for stability and speed, it utilizes a **Data-Oriented Design (ECS)** architecture and an **Extended Position Based Dynamics (XPBD)** solver to handle thousands of objects with ease.

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Platform](https://img.shields.io/badge/platform-Windows-lightgrey.svg)
![Standard](https://img.shields.io/badge/C%2B%2B-17-blue.svg)

---

## 🚀 Key Features

*   **⚡ High Performance**: Custom ECS architecture optimized for CPU cache locality (Arrays over Pointers).
*   **📐 Stable Solver**: XPBD (Extended Position Based Dynamics) solver ensures stability for stacking, high-speed collisions, and stiff constraints.
*   **🧊 2D Optimized**: Specialized for 2D XY-plane simulations with reduced overhead.
*   **🌍 Modular Logic**: Behavior-based components (Force Fields, Oscillators, Projectiles) decouple logic from the core solver.
*   **🏗️ Broadphase Optimization**: Spatial Hashing Grid (60x60 cells) reduces collision complexity from $O(N^2)$ to $O(N)$ for typical scenes.
*   **🔌 C-API**: Simple, portable C-style API (`VeloxAPI.h`) for zero-friction integration into other languages and engines.
*   **🎮 Visualizer**: Included Raylib-powered visualizer for real-time demos and stress testing.

---

## 🏛️ Architecture

Velox separates **Data** (Components) from **Logic** (Systems).

1.  **EntityManager (`VelcoxECS`)**: detailed contiguous storage for all component data.
2.  **PhysicsSystem**: Stateless logic core that iterates over component arrays to apply rules and solve constraints.

### The Physics Pipeline
1.  **Integration**: Predict tentative positions ($x' = x + v \Delta t$).
2.  **Broadphase**: Spatial Grid inserts all colliders to find potential pairs.
3.  **Narrowphase**: Detailed collision checks (Circle-Circle, Circle-Box, etc.).
4.  **Solver (XPBD)**: Iteratively correct positions to satisfy constraints (Non-penetration, Distance).
5.  **Velocity Update**: Update velocities based on position corrections ($v = (x_{new} - x_{old}) / \Delta t$).

---

## 📦 Components Reference

Components are Pure Old Data (POD) structs. They define *what* an entity is.

### Core Physics
| Component | Properties | Description |
| :--- | :--- | :--- |
| **Transform** | `Position (Vec2)`, `Rotation (rad)`, `Scale (Vec2)` | Defines world space location and orientation. |
| **RigidBody** | `Mass`, `InverseMass`, `Inertia`, `IsStatic` | Defines dynamic properties. 0 Mass = Static. |
| **Movement** | `Velocity`, `AngularVelocity`, `Damping` | Stores implicit solver state for motion. |
| **Collider** | `Type` (Box/Circle), `Radius`, `BoxHalfExtents` | Defines the physical shape used for collision detection. |
| **PhysicalMaterial** | `StaticFriction`, `DynamicFriction`, `Restitution` | Surface properties. High restitution = Bouncy. |

### Modular Behaviors
| Component | Properties | Description |
| :--- | :--- | :--- |
| **ForceField** | `Type`, `Strength`, `Radius` | Applies radial forces. Types: `Inward`, `Outward`, `Clockwise` (Vortex). |
| **Oscillation** | `Axis`, `Amplitude`, `Frequency`, `CenterPos` | Moves entity in a Sine wave pattern. Good for elevators/platforms. |
| **Rotation** | `Speed`, `Direction` | Applies constant angular velocity. Good for motors/fans. |
| **Projectile** | `FaceVelocity`, `Speed`, `MaxSpeed`, `Bounce` | Align rotation to velocity vector. Good for arrows/missiles. |

---

## 📚 API Integration

Velox exposes a clean C-API to manage the world and entities.

### 1. Initialization
```cpp
#include <velox/VeloxAPI.h>

VeloxWorld* world = Velox_CreateWorld();
```

### 2. Creating an Object (Entity)
```cpp
// Create a new ID
int id = Velox_CreateEntity(world);

// Add Components to define behavior
Velox_AddTransform(world, id, 100.0f, 200.0f, 0.0f);      // x, y, rotation
Velox_AddRigidBody(world, id, 1.0f, false);               // Mass=1.0, Static=false
Velox_AddCircleCollider(world, id, 15.0f);                // Radius=15
Velox_AddMovement(world, id);                             // Enable physics movement
Velox_AddPhysicalMaterial(world, id, 0.5f, 0.3f, 0.8f);   // Friction, Restitution
```

### 3. Simulation Loop
```cpp
const float dt = 1.0f / 60.0f;
while (appRunning) {
    Velox_Step(world, dt);
    
    // Get Data for Rendering
    float x, y, rot;
    Velox_GetPosition(world, id, &x, &y, &rot);
    renderCircle(x, y, rot);
}
```

### 4. Cleanup
```cpp
Velox_DestroyWorld(world);
```

---

## 🎮 Visualizer & Demos

The included **Visualizer** (built with Raylib) demonstrates the engine's capabilities.

### Controls
*   **Scene Selection**: Dropdown menu to switch demos.
*   **Space**: Pause/Resume.
*   **Mouse**: Context-sensitive interaction (Spawn balls, Shoot arrows).
*   **Arrow Keys**: Rotate the container (Bouncing Balls scene).

### Scenes
1.  **Bouncing Balls**: Stress test spatial hash grid with hundreds of colliding bodies.
2.  **Force Fields**: Interaction with invisible physics zones (Gravity Wells, Repulsors, Vortexes).
3.  **Oscillation**: Platforms moving in perfect sine-waves, affecting physical bodies on top.
4.  **Projectiles**: Simulation of aerodynamic object alignment (Arrows).

---

## 🛠️ Building

### Prerequisites
*   **CMake**: 3.10+
*   **Compiler**: C++17 compliant (MSVC, GCC, Clang)

### Steps
```bash
# 1. Clone
git clone https://github.com/1SHAMAY1/Velox.git
cd Velox

# 2. Configure
mkdir build && cd build
cmake ..

# 3. Build (Release recommended for performance)
cmake --build . --config Release
```

### Running (Windows)
```bash
.\bin\Release\VeloxVisualizer.exe
```

---

## 📄 License
This project is licensed under the **MIT License** - see the `LICENSE` file for details.
