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
| **SSE2 SIMD Optimization** | High-performance vector operations using SSE2 CPU instructions (compile-time opt-in via `-DVELOX_SIMD=ON`) |
| **CCD (Continuous Collision)** | Swept-volume TOI calculations prevent high-speed tunneling with sub-stepping |
| **Rigid Body Sleeping** | Deactivates dormant bodies to save CPU cycles; automatically wakes them on collision/contact |
| **ECS Architecture** | Cache-friendly component arrays for high entity counts with minimal overhead |
| **XPBD Solver** | Sub-stepped (8×/frame) for stable stacking, joints, and high-speed collisions |
| **Broadphase** | Flat contiguous array grid — zero heap allocations, $O(N)$ average complexity, cache-friendly cell traversal |
| **Collider Types** | Circle, Box (OBB), Convex Polygon, Chain (one-sided terrain) |
| **XPBD Joints** | Distance, Revolute (Hinge), Prismatic (Slider), Gear, and Pulley constraints |
| **Soft Bodies** | XPBD-based deformable bodies: **Blob** (area-preservation) and **ShapeMatched** (elastic rest-shape) |
| **Sensors & Groups** | Sensor colliders detect overlaps without response; collision group IDs exclude friendly-fire pairs |
| **Force Fields** | Gravity wells, repulsors, and vortex fields affecting bodies in range |
| **Raycasting** | Scene raycast returning hit point, normal, fraction, and entity ID |
| **C API** | Flat C-style API (`VeloxAPI.h`) for easy integration with any language or engine |
| **Visualizer** | Raylib-powered interactive demo suite with sleeping, CCD, joint, and soft-body showcases |

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
| `RigidBodyComponent` | `Mass`, `InverseMass`, `IsStatic`, `IsSleeping`, `SleepTimer`, `AllowSleep` | Dynamic properties (InverseMass=0 → static), and deactivation sleep state |
| `MovementComponent` | `Velocity`, `AngularVelocity`, `Force`, `Damping`, `PrevPosition`, `PrevVelocity` | Integration state + XPBD solver snapshots |
| `ColliderComponent` | `Type`, `Radius` / `BoxHalfExtents` / `Vertices`, `CenterOffset`, `IsSensor`, `GroupId` | Collision shape; sensor flag; group-ID filtering |
| `PhysicalMaterialComponent` | `StaticFriction`, `DynamicFriction`, `Restitution` | Surface response |

### Behaviour & Constraints
| Component | Purpose |
| :--- | :--- |
| `ForceFieldComponent` | Radial forces: `Inward`, `Outward`, `Clockwise`, `AntiClockwise` |
| `OscillationComponent` | Sine-wave platform motion along a configurable axis |
| `RotationComponent` | Constant angular velocity motor |
| `ProjectileComponent` | Aligns rotation to velocity (arrows, missiles); configurable `BounceFactor` |
| `JointComponent` | XPBD distance constraint with optional angular motor |
| `RevoluteJointComponent` | XPBD angular hinge constraint with limits and motor |
| `PrismaticJointComponent` | XPBD sliding prismatic constraint with axis limits and motor |
| `GearJointComponent` | Coupling constraint linking rotation speed of two bodies |
| `PulleyJointComponent` | Rope pulley constraint linking suspended lengths of two bodies |
| `SoftBodyComponent` | XPBD deformable body: `Blob` (area preservation) or `ShapeMatched` (elastic rest-shape) |

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

### Soft Body Quick-Start

```cpp
// Blob — closed loop with area preservation
Velox::EntityID blob = Velox_CreateSoftBodyBlob(
    world,
    640.0f, 200.0f,  // center
    60.0f,           // radius
    12,              // node count
    0.001f,          // area compliance (lower = stiffer volume)
    0.0f,            // distance joint compliance between nodes
    8.0f             // per-node circle radius
);

// ShapeMatched — elastic body that snaps back to a rest shape
float vx[] = {-40,-40, 40, 40};
float vy[] = {-40, 40, 40,-40};
Velox::EntityID box = Velox_CreateSoftBodyShapeMatched(
    world,
    400.0f, 200.0f,  // center
    vx, vy, 4,       // local vertex array
    0.15f,           // stiffness [0, 1]
    10.0f            // per-node radius
);

// Iterate nodes for rendering
int n = Velox_GetSoftBodyNodeCount(world, blob);
for (int i = 0; i < n; i++) {
    Velox::EntityID node = Velox_GetSoftBodyNode(world, blob, i);
    float x, y, rot;
    Velox_GetPosition(world, node, &x, &y, &rot);
    DrawCircle(x, y, 8.0f);
}
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
| **Revolute & Prismatic & Gear & Pulley Showcase** | Visualizes hinges with motors/limits, slider guides, gear drives, and linked pulley cords |
| **CCD vs Tunneling Showcase** | Hyper-speed bullets fired at a thin wall, demonstrating zero tunneling |
| **Sleeping & Activation Showcase** | Pile of boxes deactivating (turning gray) when settled and waking up on impact |
| **Soft Body Showcase** | Blob bodies (area-preservation) and ShapeMatched elastic bodies interacting with rigid geometry |

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
