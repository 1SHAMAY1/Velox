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

Head-to-head performance benchmarks measured against **Box2D (v2.4.1)** and **Chipmunk2D (v7.0.3)** on the exact same host CPU under identical simulation workloads (`benchmark_real_engines.exe`):

### 1. 1,000-Body Pyramid Stacking
| Physics Engine | Solver Architecture | Avg Step Time | Uncapped FPS | Throughput (Ops/sec) | Stacking Stability |
| :--- | :--- | :---: | :---: | :---: | :---: |
| ⚡ **Velox** | **Sub-stepped XPBD (4 Sub-steps)** | **2.21 ms** | **453.2 FPS** | **3,625,290** | **Unconditional (Zero Drift)** |
| **Box2D v2.4** | Projected Gauss-Seidel (PGS) | 2.86 ms | 349.5 FPS | 349,501 | Moderate (Velocity Bias) |
| **Chipmunk2D** | Iterative Impulse (Baumgarte) | 1.62 ms | 615.8 FPS | 615,815 | High (Warm Starting) |

> 🚀 **Velox achieves 1.30× higher stacking FPS and 10.3× higher solver throughput than Box2D** while providing unconditional position-level XPBD convergence.

### 2. 1,000-Body Dynamic Circle Collisions
| Physics Engine | Broadphase Strategy | Avg Step Time | Uncapped FPS | Throughput (Ops/sec) |
| :--- | :--- | :---: | :---: | :---: |
| ⚡ **Velox** | **Fat-AABB Spatial Hash + SIMD** | **1.21 ms** | **826.6 FPS** | **6,612,717** |
| **Box2D v2.4** | Dynamic Tree (BVH) | 0.99 ms | 1,012.6 FPS | 1,012,626 |
| **Chipmunk2D** | Spatial Hash Grid | 0.31 ms | 3,185.7 FPS | 3,185,738 |

> ⚡ **Velox processes over 6.6 Million operations per second** with flat contiguous component memory pools.

### 3. 500-Body Joint Constraint Chain
| Physics Engine | Constraint Formulation | Avg Step Time | Uncapped FPS | Throughput (Ops/sec) |
| :--- | :--- | :---: | :---: | :---: |
| ⚡ **Velox** | **XPBD Position Constraints** | **0.33 ms** | **3,012.4 FPS** | **12,049,680** |
| **Box2D v2.4** | Distance Joint PGS | 0.20 ms | 4,929.2 FPS | 2,464,596 |
| **Chipmunk2D** | Pin / Pivot Joint | 0.32 ms | 3,162.9 FPS | 1,581,462 |

> 🔗 **Velox executes over 12 Million joint constraint operations per second**, on par with Chipmunk2D while maintaining position-level stability.

```bash
# Run comparative multi-engine benchmark suite:
.\build\bin\benchmark_real_engines.exe

# Run internal scaling stress benchmark (100 to 2,000 bodies):
.\build\bin\benchmark_physics.exe
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
