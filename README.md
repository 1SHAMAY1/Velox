# Velox Physics Engine

**Velox** is a high-performance, modular, and customizable physics engine written in C++. It is designed to be a robust foundation for next-generation games and simulations, prioritizing stability, speed, and developer control.

## 🏗️ Architecture Deep Dive

Velox is built upon a **Data-Oriented Design** philosophy, strictly separating data (Components) from logic (Systems). This ensures cache efficiency and scalability.

### 1. Entity Component System (ECS)
The core of Velox is a custom, lightweight ECS.
- **Entities**: Simple 32-bit Integer IDs. They are just handles.
- **Components**: Pure data structs (PODs). They contain no logic.
    - `RigidBodyComponent`: Velocity, Mass, Forces.
    - `ColliderComponent`: Shape data (Sphere, Box, etc.).
    - `SpringComponent`: Constraints for soft bodies.
- **Systems**: Logic processors that iterate over entities with specific components.
    - `PhysicsSystem`: The main solver loop.

**Storage**: Components are stored in **Sparse Sets** (or packed arrays with sparse mapping). This allows for O(1) access by Entity ID while keeping component data contiguous in memory for efficient iteration (Cache Locality).

### 2. Physics Simulation (XPBD)
Velox uses **Extended Position Based Dynamics (XPBD)** as its solver. Unlike traditional impulse-based solvers, XPBD solves constraints directly on positions.
- **Stability**: Extremely stable, even with large time steps or stiff constraints.
- **Soft Bodies**: Naturally handles cloth, ropes, and soft bodies using Distance Constraints (Springs).
- **Sub-stepping**: The solver can run multiple iterations per frame to converge on a solution.

**Pipeline**:
1.  **Integration**: Predict tentative positions based on velocity and gravity ($x' = x + v \Delta t$).
2.  **Broadphase**: Identify potential collision pairs (currently O(N^2), planned Dynamic AABB Tree).
3.  **Narrowphase**: Detailed collision checks (Sphere-Sphere, Ground Plane).
4.  **Solver**: Iteratively correct positions to satisfy constraints (Collisions, Springs).
5.  **Velocity Update**: Update velocities based on position changes.

### 3. Rule System 🧠
A unique feature of Velox is the **Rule System**. It allows developers to inject custom logic into the physics pipeline without modifying the engine core.
- **RuleComponent**: Attaches to entities.
- **Rules**: Can override gravity, modify friction, or apply custom forces based on conditions.
- **Use Case**: "Anti-gravity zones", "Sticky surfaces", or "Magnetic fields" can be implemented as Rules.

### 4. Visualizer 🎮
The engine includes a standalone **Visualizer** built with **Raylib**.
- **Real-time Rendering**: Visualizes the physics world at 60+ FPS.
- **Debug Tools**:
    - **Performance Graph**: Tracks frame times and memory usage.
    - **Network Sim**: Simulates packet loss to test deterministic networking.
    - **Network Sim**: Simulates packet loss to test deterministic networking.
    - **Interactive**: Spawn projectiles, soft bodies, and manipulate the scene.

### 🎮 Visualizer Controls
| Key | Action |
| :--- | :--- |
| **1** | Spawn Soft/Rigid Cube |
| **2** | Spawn Soft/Rigid Tetrahedron |
| **S** | **Stress Test**: Spawn chaotic batch of 100 entities |
| **R** | Toggle Rendering (Physics Performance Mode) |
| **F** | Toggle Wireframes & Springs |
| **Click** | Spawn Projectile (in Projectile Mode) |
| **`** | Toggle Debug UI |

## 🚀 Getting Started

### Prerequisites
- CMake 3.10+
- C++17 Compiler (MSVC, GCC, Clang)

### Building
```bash
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

### Integration
Velox currently provides a **C-API** for integration with C++ engines or languages that support FFI (Foreign Function Interface).

**Note**: Native plugins for **Unity** and **Unreal Engine 5** are currently in development (see Roadmap).

Include the public headers in your project:
```cpp
#include <velox/PhysicsEngineAPI.h>

// Initialize
VeloxWorld* world = Velox_CreateWorld();

// Create a Sphere
Velox::EntityID id = Velox_CreateEntity(world);
Velox_AddTransform(world, id, 0, 10, 0);
Velox_AddRigidBody(world, id, 1.0f, false);
// ...
```

## 🔮 Future Roadmap
- **AI-Driven Simulation**: AI based Simulation to improve performance and memory management.
- **GJK/EPA**: Support for arbitrary convex hull collisions.
- **Spatial Partitioning**: BVH or Octree for optimized Broadphase.
- **SIMD Optimization**: Vectorize the solver loop for massive performance gains.
- **Unity/Unreal Plugins**: Native C# and Blueprint wrappers.

## License
MIT License
