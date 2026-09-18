# 🧪 Testing & Validation Suite

Velox comes with an automated test suite verifying mathematical precision, ECS integrity, constraint convergence, and numerical stability.

---

## 🏃 Running Tests

All test executables are output to `build/bin/`:

```bash
# Run individual test suites
.\build\bin\test_math.exe
.\build\bin\test_ecs.exe
.\build\bin\test_primitives.exe
.\build\bin\test_joints_softbody.exe
.\build\bin\test_gameplay_components.exe
.\build\bin\test_decentralized_behavior.exe
.\build\bin\test_raycasting.exe
.\build\bin\test_chaos_stress.exe
```

---

## 📋 Test Matrix

| Suite | Target | Focus |
| :--- | :--- | :--- |
| `test_math` | Vec2 & Matrix math | Vector operations, dot/cross products, normalization, rotations |
| `test_ecs` | Entity Manager | Entity allocation, deletion, ID recycling, component registration |
| `test_primitives` | Narrowphase Collision | SAT for circles, OBB boxes, convex polygons, and continuous chains |
| `test_joints_softbody` | XPBD Constraints | Distance, Revolute, Prismatic constraints, Blob volume, Shape Matching |
| `test_gameplay_components` | Auxiliary Components | Motors, Sine Oscillators, Projectile tracking, Radial force fields |
| `test_decentralized_behavior` | Extensibility | Macro-driven custom component registration and per-frame update hooks |
| `test_raycasting` | Ray Queries | Exact hit point, normal vector, fractional distance, miss detection |
| `test_chaos_stress` | Stress & Stability | 1,000 chaotic colliding entities: ensures 0 NaNs, numerical stability, $<800\text{ bytes/body}$ RAM |
