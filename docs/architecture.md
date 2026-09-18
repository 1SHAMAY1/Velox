# 🏛️ Velox Architecture & Design

Velox is designed around two primary principles: **Data-Oriented ECS** for maximum cache efficiency, and **Extended Position Based Dynamics (XPBD)** for unconditional numerical stability.

---

## 🧩 ECS Foundation

Velox cleanly decouples state (components) from simulation logic (systems):

- **`EntityManager` (`src/core/VelcoxECS.h`)**
  - Manages contiguous, cache-aligned component pools indexed directly by entity ID.
  - Generational entity recycling prevents stale index lookups.
  - Zero dynamic heap allocations during runtime updates.
- **`PhysicsSystem` (`src/physics/PhysicsSystem.h`)**
  - Stateless system processing flat component arrays sequentially.
  - Maximizes SIMD vectorization and hardware prefetching.

---

## ⚡ XPBD Simulation Pipeline

Each frame is subdivided into configurable sub-steps (default: **8 sub-steps per frame**):

```
       ┌─────────────┐
       │  Integrate  │   Semi-implicit Euler position & velocity prediction
       └──────┬──────┘
              │
       ┌──────▼──────┐
       │    Solve    │   Broadphase → Narrowphase → Constraint Projection
       │ Constraints │   (Joints, Collisions, Contact Manifolds, Soft Bodies)
       └──────┬──────┘
              │
       ┌──────▼──────┐
       │   Derive    │   Update velocities from positional displacements:
       │ Velocities  │   v = (x - x_prev) / dt
       └──────┬──────┘
              │
       ┌──────▼──────┐
       │   Resolve   │   Apply XPBD restitution & Coulomb friction impulses
       │ Velocities  │
       └─────────────┘
```

### Sub-step Breakdown

1. **Integrate:**
   - Applies external forces, gravity, and velocities to compute predicted candidate positions $\tilde{x}$.
   - Stores previous states ($x_{\text{prev}}, v_{\text{prev}}$) for XPBD velocity derivation.
2. **Solve Constraints:**
   - **Broadphase:** Spatial hash grid / fat-AABB spatial partition quickly prunes non-colliding entity pairs.
   - **Narrowphase:** Dispatches SAT (Separating Axis Theorem) and shape-specific contact solvers.
   - **XPBD Projection:** Positions are iteratively corrected using compliant constraint formulations.
3. **Derive Velocities:**
   - Evaluates true velocities based on total positional delta over sub-step $\Delta t$.
4. **Resolve Velocities:**
   - Applies friction (static & dynamic) and restitution (bouncing) at active contact manifolds.

---

## 🔍 Collision Detection Pipeline

### Broadphase
- **Flat Spatial Hash:** Contiguous cell array providing $O(N)$ average time complexity with zero heap allocation during traversal.
- **Dynamic Tree (BVH):** Optional bounding volume hierarchy for sparse and non-uniform scenes.

### Narrowphase Dispatch Matrix

| Shape A \ Shape B | Circle | Box (OBB) | Convex Polygon | Chain (Terrain) |
| :---: | :---: | :---: | :---: | :---: |
| **Circle** | Direct Distance | Closest Point | SAT / Closest Edge | Support Half-Space |
| **Box (OBB)** | Closest Point | SAT (Separating Axis) | SAT | Support Half-Space |
| **Convex Polygon** | SAT / Closest Edge | SAT | SAT | Support Half-Space |
| **Chain (Terrain)** | Support Half-Space | Support Half-Space | Support Half-Space | — |

> **Note on Chain Terrain:** Polyline chains use half-space support tests rather than volumetric SAT to avoid tunneling on zero-thickness edges.

---

## 🛡️ Continuous Collision Detection (CCD)

- Swept-volume TOI (Time of Impact) calculation for fast-moving dynamic colliders.
- Prevents tunneling through thin walls and dynamic barriers under high velocities.

---

## 💤 Rigid Body Sleeping & Islands

- Automatically puts settled bodies to sleep after inactivity thresholds.
- Sleeping bodies are bypassed in solver loops, dramatically reducing CPU cycles.
- Automatically awakened on contact impulses, joint pulls, or external forces.
