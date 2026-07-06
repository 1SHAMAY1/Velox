# ⚡ Velox Physics Engine

![Velox Logo](assets/velox_icon_window.png)

**Velox** is a lightweight, high-performance 2D physics engine written in C++17. It uses a **Data-Oriented ECS** architecture and an **XPBD (Extended Position Based Dynamics)** solver to simulate thousands of bodies with stability and speed.

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Platform](https://img.shields.io/badge/platform-Windows-lightgrey.svg)
![Standard](https://img.shields.io/badge/C%2B%2B-17-blue.svg)

---

## 🚀 Key Features

| Feature | Details |
| :--- | :--- |
| **ECS Architecture** | Cache-friendly component arrays for high entity counts with minimal overhead |
| **XPBD Solver** | Sub-stepped (8×/frame) for stable stacking, joints, and high-speed collisions |
| **Broadphase** | Spatial hash grid reduces collision pair complexity from O(N²) to O(N) |
| **Collider Types** | Circle, Box (OBB), Convex Polygon, Chain (one-sided terrain) |
| **Constraints** | Distance joints with compliance (stiffness), damping, and angular motors |
| **Force Fields** | Gravity wells, repulsors, and vortex fields affecting bodies in range |
| **Raycasting** | Scene raycast returning hit point, normal, fraction, and entity ID |
| **C API** | Flat C-style API (`VeloxAPI.h`) for easy integration with any language or engine |
| **Visualizer** | Raylib-powered interactive demo suite for testing and showcase |

---

## 🏛️ Architecture

Velox separates **data** (components) from **logic** (systems) in a classic ECS pattern.

- **EntityManager** (`VelcoxECS.h`) — manages contiguous component arrays indexed by entity ID.
- **PhysicsSystem** — stateless system that iterates component arrays to apply physics rules.

### Pipeline (per sub-step)

```
Integrate → SolveConstraints → DeriveVelocities → ResolveVelocities
```

1. **Integrate** — semi-implicit Euler velocity and position prediction.
2. **SolveConstraints** — spatial hash broadphase → narrowphase collision dispatch → positional correction.
3. **DeriveVelocities** — recompute velocities from XPBD position deltas.
4. **ResolveVelocities** — impulse-based restitution and Coulomb friction.

### Narrowphase Dispatch

| A \ B | Circle | Box | Polygon | Chain |
| :---: | :---: | :---: | :---: | :---: |
| **Circle** | ✅ | ✅ | ✅ | ✅ |
| **Box** | ✅ | ✅ (OBB SAT) | ✅ | ✅ |
| **Polygon** | ✅ | ✅ | ✅ (SAT) | ✅ |

Chain collision uses a **half-space / support-point** test (Box2D style). SAT is not used against segments because a zero-thickness edge has no reliable volume projection.

---

## 📦 Components

Components are pure POD structs — no logic, no virtual functions.

### Core Physics
| Component | Key Fields | Purpose |
| :--- | :--- | :--- |
| `TransformComponent` | `Position`, `Rotation`, `Scale` | World-space pose |
| `RigidBodyComponent` | `Mass`, `InverseMass`, `IsStatic` | Dynamic properties (InverseMass=0 → static) |
| `MovementComponent` | `Velocity`, `AngularVelocity`, `Force`, `Damping` | Integration state |
| `ColliderComponent` | `Type`, `Radius` / `BoxHalfExtents` / `Vertices` | Collision shape |
| `PhysicalMaterialComponent` | `StaticFriction`, `DynamicFriction`, `Restitution` | Surface response |

### Behaviour
| Component | Purpose |
| :--- | :--- |
| `ForceFieldComponent` | Radial forces: `Inward`, `Outward`, `Clockwise`, `AntiClockwise` |
| `OscillationComponent` | Sine-wave platform motion along a configurable axis |
| `RotationComponent` | Constant angular velocity motor |
| `ProjectileComponent` | Aligns rotation to velocity (arrows, missiles) |
| `JointComponent` | XPBD distance constraint with optional angular motor |

---

## 📚 API Quick-Start

```cpp
#include <velox/VeloxAPI.h>

// Create world
VeloxWorld* world = Velox_CreateWorld();
Velox_SetGravity(world, 0.0f, 400.0f);

// Static floor
auto floor = Velox_CreateEntity(world);
Velox_AddTransform(world, floor, 640.0f, 680.0f, 0.0f);
Velox_AddRigidBody(world, floor, 0.0f, true);   // mass=0 → static
Velox_AddMovement(world, floor);
Velox_AddBoxCollider(world, floor, 1280.0f, 20.0f);

// Dynamic ball
auto ball = Velox_CreateEntity(world);
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
    DrawCircle(x, y, 20.0f); // your renderer
}

Velox_DestroyWorld(world);
```

---

## 🎮 Visualizer Demos

Launch `VeloxVisualizer.exe` and select a scene from the dropdown.

| Scene | What it shows |
| :--- | :--- |
| **Bouncing Balls** | Spatial hash stress test — hundreds of colliding circles |
| **Force Field Demo** | Gravity wells, repulsors, and vortex fields |
| **Oscillation Demo** | Sine-wave platforms carrying dynamic bodies |
| **Projectile Demo** | Aerodynamic arrow alignment (rotation tracks velocity) |
| **Gravity Direction Demo** | Runtime gravity vector control (WASD to rotate) |
| **Distance Joint Demo** | XPBD joints with stiffness, damping, and motors |
| **Convex Polygons & Motors Sandbox** | OBB + polygon SAT, rotating motor platforms |
| **Chain Shapes Showcase** | One-sided chain floor; circles and boxes rest on sinusoidal terrain |
| **Raycast Queries Showcase** | Live raycast with hit normals and entity detection |

### Controls
- **Dropdown** — switch scene
- **Space** — pause / resume
- **Mouse** — spawn or shoot objects (scene-dependent)
- **WASD** — rotate gravity direction (Gravity Direction Demo)

---

## 🛠️ Building

### Prerequisites
- CMake 3.10+
- C++17 compiler (MSVC 2019+, GCC 9+, Clang 10+)

### Steps

```bash
# Clone
git clone https://github.com/1SHAMAY1/Velox.git
cd Velox

# Configure and build (Release recommended)
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

### Run (Windows)
```bash
.\build\bin\Release\VeloxVisualizer.exe
# or use the included helper:
.\run_visualizer.bat
```

---

## 📄 License

MIT — see [LICENSE](LICENSE) for details.
