# ⚡ Velox Physics Engine

![Velox Logo](assets/velox_icon_window.png)

**Velox** is a lightweight, high-performance 2D physics engine written in C++17. Powered by a **Data-Oriented ECS** architecture and an **XPBD (Extended Position Based Dynamics)** solver, Velox simulates thousands of rigid and deformable bodies with unconditional stability and minimal memory footprint.

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
![Platform](https://img.shields.io/badge/Platform-Windows-lightgrey.svg)
![Standard](https://img.shields.io/badge/C%2B%2B-17-blue.svg)

---

## ✨ Features

- **⚡ Sub-Stepped XPBD Solver:** Highly stable stacking, mass ratios, and joints with zero solver drift.
- **🚀 Cache-Friendly ECS:** Flat contiguous component pools with zero runtime allocations in inner loops.
- **🛡️ Continuous Collision Detection (CCD):** Swept-volume TOI calculations prevent high-speed tunneling.
- **💤 Rigid Body Sleeping:** Automatically deactivates dormant bodies to save CPU cycles.
- **📐 Rich Collider Support:** Circle, Box (OBB), Convex Polygon, and One-Sided Chain (Terrain).
- **🔗 Advanced Constraints:** Distance, Revolute (Hinge), Prismatic (Slider), Gear, and Pulley joints.
- **🎈 Soft Bodies:** XPBD Area-preserving Blobs and Elastic Shape-Matched meshes.
- **🌊 Gameplay Dynamics:** Buoyancy, Force Fields (vortex, gravity wells), Motors, Sine Oscillators, and Projectiles.
- **🔌 Flat C API:** Direct integration with C#, Rust, Python, Lua, or custom engines (`VeloxAPI.h`).
- **🎮 Interactive Visualizer:** Raylib-powered interactive demo suite with live performance telemetry.

---

## ⚡ Performance Benchmarks

Head-to-head performance benchmarks against **Box2D (v2.4.2)** and **Chipmunk2D (v7.0.3)** under identical simulation scenarios:

### 1. 1,000-Body Pyramid Stacking
| Physics Engine | Solver Type | Avg Step Time | Uncapped FPS | Peak RAM | Bytes / Body |
| :--- | :--- | :---: | :---: | :---: | :---: |
| ⚡ **Velox** | **XPBD (8 Sub-steps)** | **1.55 ms** | **644 FPS** | **712 KB** | **~729 B** |
| **Chipmunk2D** | Impulse-based | 1.55 ms | 643 FPS | 1.3 MB | ~1,392 B |
| **Box2D v2.4** | PGS (Gauss-Seidel) | 2.86 ms | 349 FPS | 700 KB | ~716 B |

> 🚀 **Velox achieves 1.84× higher throughput than Box2D** while consuming **half the RAM of Chipmunk2D**.

### 2. 1,000-Body Dynamic Collisions
| Physics Engine | Broadphase Strategy | Avg Step Time | Uncapped FPS | Peak RAM |
| :--- | :--- | :---: | :---: | :---: |
| ⚡ **Velox** | **Fat-AABB Spatial Hash + Caching** | **0.74 ms** | **1,345 FPS** | **588 KB** |
| **Box2D v2.4** | Dynamic Tree (BVH) | 1.08 ms | 921 FPS | 420 KB |
| **Chipmunk2D** | Spatial Hash Grid | 0.29 ms | 3,391 FPS | 800 KB |

> ⚡ **Velox achieves sub-millisecond step times (>1,300 FPS)** with zero runtime allocations.

### 3. 500-Body Joint Constraint Chain
| Physics Engine | Constraint Formulation | Avg Step Time | Uncapped FPS | Peak RAM |
| :--- | :--- | :---: | :---: | :---: |
| ⚡ **Velox** | **Sub-stepped XPBD Distance** | **0.37 ms** | **2,644 FPS** | **452 KB** |
| **Box2D v2.4** | Distance Joint PGS | 0.19 ms | 5,116 FPS | 420 KB |
| **Chipmunk2D** | Pivot / Pin Joint | 0.30 ms | 3,293 FPS | 300 KB |

```bash
# Run comparative benchmark suite
.\build\bin\benchmark_real_engines.exe
```

---

## 🛠️ Setup & Build

### Prerequisites
- CMake 3.20+
- C++17 compatible compiler (MSVC 2019+, GCC 10+, Clang 11+)
- Git

### Build Instructions

```bash
# Clone the repository
git clone https://github.com/1SHAMAY1/Velox.git
cd Velox

# Configure and build (Release recommended)
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

### Launch Interactive Visualizer

```bash
.\build\bin\VeloxVisualizer.exe
# or run helper script:
.\run_visualizer.bat
```

---

## 💻 Quick Start (C API)

```c
#include <velox/VeloxAPI.h>

// Initialize world
VeloxWorld* world = Velox_CreateWorld();
Velox_SetGravity(world, 0.0f, 400.0f);

// Create static ground
VeloxEntityID floor = Velox_CreateEntity(world);
Velox_AddTransform(world, floor, 640.0f, 680.0f, 0.0f);
Velox_AddRigidBody(world, floor, 0.0f, true); // mass=0 -> static
Velox_AddBoxCollider(world, floor, 1280.0f, 20.0f);

// Create dynamic circle
VeloxEntityID ball = Velox_CreateEntity(world);
Velox_AddTransform(world, ball, 640.0f, 100.0f, 0.0f);
Velox_AddRigidBody(world, ball, 1.0f, false);
Velox_AddMovement(world, ball);
Velox_AddCircleCollider(world, ball, 20.0f);
Velox_AddPhysicalMaterial(world, ball, 0.4f, 0.2f, 0.7f); // friction, friction, restitution

// Simulation loop
const float dt = 1.0f / 60.0f;
while (running) {
    Velox_Step(world, dt);
    
    float x, y, rot;
    Velox_GetPosition(world, ball, &x, &y, &rot);
    // Render ball at (x, y)
}

Velox_DestroyWorld(world);
```

---

## 📖 In-Depth Documentation

For detailed guides and references, check the [`docs/`](docs/) directory:

- 🏛️ **[Architecture & Pipeline](docs/architecture.md)** — Deep dive into ECS data layout, sub-stepped XPBD pipeline, broadphase/narrowphase collision dispatch, and solver stages.
- 📦 **[Component Reference](docs/components.md)** — Detailed specification of all physics, constraint, soft-body, and gameplay components.
- 📚 **[API Reference](docs/api_reference.md)** — Comprehensive C/C++ API reference covering world lifecycle, entity creation, joints, soft bodies, and raycasting.
- 🎮 **[Visualizer & Demos](docs/visualizer.md)** — Scene catalog (16 interactive demos), controls, keybindings, and live HUD metrics.
- 🧪 **[Testing & Validation](docs/testing.md)** — Automated test suite guide, test matrix, and chaos/stress validation.

---

## 📄 License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.
