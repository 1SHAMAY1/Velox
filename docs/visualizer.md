# 🎮 Velox Visualizer & Demo Suite

The visualizer (`VeloxVisualizer.exe`) is powered by **Raylib** and features 16 interactive testbeds and benchmarks.

---

## 🕹️ Controls

| Input | Action |
| :--- | :--- |
| **Dropdown / Tab** | Switch active simulation scene |
| **Space** | Pause / Resume simulation |
| **R** | Reset current scene |
| **Mouse Left-Click** | Spawn bodies / Grab & Drag / Fire wrecking ball (scene-dependent) |
| **Mouse Right-Click** | Apply impulse / explosion shockwave |
| **WASD** | Rotate gravity direction (in Gravity Direction Demo) |
| **F1** | Toggle debug colliders / AABBs / contact manifolds overlay |

---

## 🎪 Available Scenes

| # | Scene | Highlights |
| :---: | :--- | :--- |
| 1 | **Bouncing Balls** | Spatial hash stress test simulating hundreds of colliding circles |
| 2 | **Force Field Demo** | Gravity wells, repulsors, and vortex fields |
| 3 | **Oscillation Demo** | Sine-wave moving platforms carrying dynamic bodies |
| 4 | **Projectile Demo** | Aerodynamic arrow alignment (rotation tracks velocity) |
| 5 | **Gravity Direction Demo** | Dynamic gravity vector manipulation in real time |
| 6 | **Distance Joint Demo** | XPBD compliant distance joints, chains, and rope bridges |
| 7 | **Convex Polygons & Motors** | OBB + Polygon SAT collision response with motorized platforms |
| 8 | **Chain Shapes Showcase** | Smooth polyline terrain collisions with boxes and spheres |
| 9 | **Raycast Queries Showcase** | Interactive raycast scanner with normal visualization |
| 10 | **Revolute & Prismatic & Gear & Pulley** | Comprehensive mechanical joint suite (hinges, sliders, gears, pulleys) |
| 11 | **CCD vs Tunneling Showcase** | Hyper-speed projectile firing at thin static barriers without tunneling |
| 12 | **Sleeping & Activation Showcase** | Body deactivation on rest and automatic wake-up on impact |
| 13 | **Soft Body Showcase** | Deformable blobs and elastic shape-matched meshes |
| 14 | **Buoyancy Water Tank** | Floating bodies with fluid drag and displaced mass buoyancy |
| 15 | **1,000-Body Chaos & Destruction** | Massive 1,000-body stack smashed by high-mass wrecking ball |
| 16 | **Articulated Ragdoll Network** | Multi-jointed humanoids with revolute joint angle limits |

---

## 📊 Live HUD Telemetry

The top-left telemetry panel shows real-time performance indicators:
- **Sub-step Latency:** Execution time per physics sub-step (in microseconds $\mu\text{s}$).
- **Uncapped Sim FPS:** Pure physics solver throughput.
- **RAM Footprint:** Resident memory footprint in Kilobytes (KB).
