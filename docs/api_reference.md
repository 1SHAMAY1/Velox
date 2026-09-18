# 📚 API Reference

Velox exposes a clean C-style interface through `include/velox/VeloxAPI.h` for seamless bindings to C#, Rust, Python, Lua, or game engines.

---

## 🌐 World Management

```c
// Create and destroy simulation world
VeloxWorld* Velox_CreateWorld();
void Velox_DestroyWorld(VeloxWorld* world);

// World configuration
void Velox_SetGravity(VeloxWorld* world, float gx, float gy);
void Velox_GetGravity(VeloxWorld* world, float* gx, float* gy);
void Velox_Step(VeloxWorld* world, float dt);
void Velox_SetSubsteps(VeloxWorld* world, int substeps);
```

---

## 🧱 Entity & Component Management

```c
// Entity lifecycle
VeloxEntityID Velox_CreateEntity(VeloxWorld* world);
void Velox_DestroyEntity(VeloxWorld* world, VeloxEntityID entity);
bool Velox_IsEntityValid(VeloxWorld* world, VeloxEntityID entity);

// Transforms
void Velox_AddTransform(VeloxWorld* world, VeloxEntityID entity, float x, float y, float rot);
void Velox_GetPosition(VeloxWorld* world, VeloxEntityID entity, float* x, float* y, float* rot);
void Velox_SetPosition(VeloxWorld* world, VeloxEntityID entity, float x, float y, float rot);

// Rigid Body & Motion
void Velox_AddRigidBody(VeloxWorld* world, VeloxEntityID entity, float mass, bool isStatic);
void Velox_AddMovement(VeloxWorld* world, VeloxEntityID entity);
void Velox_SetVelocity(VeloxWorld* world, VeloxEntityID entity, float vx, float vy);
void Velox_GetVelocity(VeloxWorld* world, VeloxEntityID entity, float* vx, float* vy);
void Velox_ApplyForce(VeloxWorld* world, VeloxEntityID entity, float fx, float fy);
void Velox_ApplyImpulse(VeloxWorld* world, VeloxEntityID entity, float ix, float iy);

// Colliders & Materials
void Velox_AddCircleCollider(VeloxWorld* world, VeloxEntityID entity, float radius);
void Velox_AddBoxCollider(VeloxWorld* world, VeloxEntityID entity, float width, float height);
void Velox_AddPolygonCollider(VeloxWorld* world, VeloxEntityID entity, const float* vx, const float* vy, int count);
void Velox_AddChainCollider(VeloxWorld* world, VeloxEntityID entity, const float* vx, const float* vy, int count, bool isLoop);
void Velox_AddPhysicalMaterial(VeloxWorld* world, VeloxEntityID entity, float staticFriction, float dynamicFriction, float restitution);
```

---

## 🔗 Constraints & Soft Bodies

```c
// Distance Joint
VeloxEntityID Velox_CreateDistanceJoint(
    VeloxWorld* world,
    VeloxEntityID bodyA, VeloxEntityID bodyB,
    float anchorAX, float anchorAY,
    float anchorBX, float anchorBY,
    float length, float compliance
);

// Revolute (Hinge) Joint
VeloxEntityID Velox_CreateRevoluteJoint(
    VeloxWorld* world,
    VeloxEntityID bodyA, VeloxEntityID bodyB,
    float anchorAX, float anchorAY,
    float anchorBX, float anchorBY
);

// Prismatic (Slider) Joint
VeloxEntityID Velox_CreatePrismaticJoint(
    VeloxWorld* world,
    VeloxEntityID bodyA, VeloxEntityID bodyB,
    float axisX, float axisY
);

// Soft Body - Area Preserving Blob
VeloxEntityID Velox_CreateSoftBodyBlob(
    VeloxWorld* world,
    float cx, float cy, float radius,
    int nodeCount, float compliance,
    float distanceCompliance, float nodeRadius
);

// Soft Body - Elastic Shape Matching
VeloxEntityID Velox_CreateSoftBodyShapeMatched(
    VeloxWorld* world,
    float cx, float cy,
    const float* vx, const float* vy, int vertexCount,
    float stiffness, float nodeRadius
);
```

---

## 🎯 Raycasting

```c
VeloxRaycastHit hit;
bool hasHit = Velox_Raycast(
    world,
    startX, startY,
    dirX, dirY,
    maxDistance,
    &hit
);

if (hasHit) {
    // hit.Entity, hit.PointX, hit.PointY, hit.NormalX, hit.NormalY, hit.Fraction
}
```
