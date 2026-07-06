/**
 * @file VeloxAPI.cpp
 * @brief Implementation of the flat C ABI declared in VeloxAPI.h.
 *
 * Each function forwards to the corresponding World / EntityManager call.
 * See VeloxAPI.h for per-function documentation.
 */
#include <velox/VeloxAPI.h>
#include "../core/World.h"
#include "../physics/Components.h"

using namespace Velox;

extern "C" {

    VeloxWorld* Velox_CreateWorld() {
        return reinterpret_cast<VeloxWorld*>(new World());
    }

    void Velox_DestroyWorld(VeloxWorld* world) {
        delete reinterpret_cast<World*>(world);
    }

    void Velox_Step(VeloxWorld* world, float dt) {
        reinterpret_cast<World*>(world)->Step(dt);
    }

    EntityID Velox_CreateEntity(VeloxWorld* world) {
        return reinterpret_cast<World*>(world)->GetEntityManager().CreateEntity();
    }

    void Velox_DestroyEntity(VeloxWorld* world, EntityID entity) {
        reinterpret_cast<World*>(world)->GetEntityManager().DestroyEntity(entity);
    }

    void Velox_AddTransform(VeloxWorld* world, EntityID entity, float x, float y, float rotation) {
        TransformComponent tc;
        tc.Position = {x, y};
        tc.Rotation = rotation;
        tc.Scale = {1, 1};
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(entity, tc);
    }

    void Velox_GetPosition(VeloxWorld* world, EntityID entity, float* x, float* y, float* rotation) {
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        if (em.HasComponent<TransformComponent>(entity)) {
            auto& tc = em.GetComponent<TransformComponent>(entity);
            *x = tc.Position.x;
            *y = tc.Position.y;
            *rotation = tc.Rotation;
        }
    }

    void Velox_AddRigidBody(VeloxWorld* world, EntityID entity, float mass, bool isStatic) {
        RigidBodyComponent rb;
        rb.Mass = mass;
        rb.InverseMass = (isStatic || mass == 0.0f) ? 0.0f : 1.0f / mass;
        rb.IsStatic = isStatic;
        // Default Inertia for a box/circle (approx)
        rb.Inertia = mass * 1.0f; 
        rb.InverseInertia = (isStatic || mass == 0.0f) ? 0.0f : 1.0f / rb.Inertia;
        
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(entity, rb);
    }

    void Velox_AddMovement(VeloxWorld* world, EntityID entity) {
        MovementComponent mc;
        mc.Velocity = {0, 0};
        mc.AngularVelocity = 0.0f;
        mc.Torque = 0.0f;
        mc.LinearDamping = 0.0f;
        mc.AngularDamping = 0.0f;
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(entity, mc);
    }

    void Velox_AddRotation(VeloxWorld* world, EntityID entity, float speed, int direction, int mode) {
        if (!world) return;
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(entity, Velox::RotationComponent{
            speed, 
            (Velox::RotationDirection)direction
        });
    }

    void Velox_AddOscillation(VeloxWorld* world, EntityID entity, float axisX, float axisY, float amplitude, float frequency) {
        if (!world) return;
        
        // Get current position as center
        float x, y, r;
        Velox_GetPosition(world, entity, &x, &y, &r);
        
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(entity, Velox::OscillationComponent{
            {axisX, axisY},
            amplitude,
            frequency,
            0.0f, // Time accumulator
            {x, y} // Center Position
        });
    }

    void Velox_AddProjectile(VeloxWorld* world, EntityID entity, bool faceVelocity, float speed, float maxSpeed, float bounceFactor) {
        if (!world) return;
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(entity, Velox::ProjectileComponent{
            faceVelocity,
            speed,
            maxSpeed,
            bounceFactor
        });
    }

    void Velox_AddPhysicalMaterial(VeloxWorld* world, EntityID entity, float staticFriction, float dynamicFriction, float restitution) {
        if (!world) return;
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(entity, Velox::PhysicalMaterialComponent{
            staticFriction,
            dynamicFriction,
            restitution
        });
    }

    void Velox_AddForceField(VeloxWorld* world, int entityID, int type, float strength, float radius) {
        if (!world) return;
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(entityID, Velox::ForceFieldComponent{
            (Velox::ForceFieldType)type,
            strength,
            radius
        });
    }

    void Velox_SetVelocity(VeloxWorld* world, EntityID entity, float x, float y) {
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        if (em.HasComponent<MovementComponent>(entity)) {
            auto& mc = em.GetComponent<MovementComponent>(entity);
            mc.Velocity = {x, y};
        }
    }

    void Velox_SetAngularVelocity(VeloxWorld* world, EntityID entity, float angularVelocity) {
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        if (em.HasComponent<MovementComponent>(entity)) {
            auto& mc = em.GetComponent<MovementComponent>(entity);
            mc.AngularVelocity = angularVelocity;
        }
    }

    void Velox_SetDamping(VeloxWorld* world, EntityID entity, float linear, float angular) {
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        if (em.HasComponent<MovementComponent>(entity)) {
            auto& mc = em.GetComponent<MovementComponent>(entity);
            mc.LinearDamping = linear;
            mc.AngularDamping = angular;
        }
    }

    void Velox_AddCircleCollider(VeloxWorld* world, Velox::EntityID entityID, float radius) {
        Velox::ColliderComponent col;
        col.Type = Velox::ColliderType::Circle;
        col.Data.Radius = radius;
        col.CenterOffset = { 0, 0 };
        col.IsSensor = false;
        
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        em.AddComponent(entityID, col);

        // Compute proper inertia for a circle: 0.5 * m * r^2
        if (em.HasComponent<RigidBodyComponent>(entityID)) {
            auto& rb = em.GetComponent<RigidBodyComponent>(entityID);
            if (!rb.IsStatic && rb.Mass > 0.0f) {
                rb.Inertia = 0.5f * rb.Mass * radius * radius;
                rb.InverseInertia = 1.0f / rb.Inertia;
            }
        }
    }

    void Velox_AddBoxCollider(VeloxWorld* world, Velox::EntityID entityID, float width, float height) {
        Velox::ColliderComponent col;
        col.Type = Velox::ColliderType::Box;
        col.Data.BoxHalfExtents = { width * 0.5f, height * 0.5f };
        col.CenterOffset = { 0, 0 };
        col.IsSensor = false;
        
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        em.AddComponent(entityID, col);

        // Compute proper inertia for a box: (1/12) * m * (w^2 + h^2)
        if (em.HasComponent<RigidBodyComponent>(entityID)) {
            auto& rb = em.GetComponent<RigidBodyComponent>(entityID);
            if (!rb.IsStatic && rb.Mass > 0.0f) {
                rb.Inertia = (1.0f / 12.0f) * rb.Mass * (width * width + height * height);
                rb.InverseInertia = 1.0f / rb.Inertia;
            }
        }
    }

    void Velox_SetGravity(VeloxWorld* world, float gx, float gy) {
        if (!world) return;
        reinterpret_cast<World*>(world)->SetGravity(gx, gy);
    }

    void Velox_AddDistanceJoint(VeloxWorld* world, Velox::EntityID entityA, Velox::EntityID entityB, float anchorAX, float anchorAY, float anchorBX, float anchorBY, float targetDistance, float compliance) {
        if (!world) return;
        auto id = reinterpret_cast<World*>(world)->GetEntityManager().CreateEntity();
        
        Velox::JointComponent jc;
        jc.EntityA = entityA;
        jc.EntityB = entityB;
        jc.LocalAnchorA = {anchorAX, anchorAY};
        jc.LocalAnchorB = {anchorBX, anchorBY};
        jc.TargetDistance = targetDistance;
        jc.Compliance = compliance;
        jc.Damping = 0.0f;
        jc.IsActive = true;
        
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(id, jc);
    }

    void Velox_SetColliderSensor(VeloxWorld* world, Velox::EntityID entity, bool isSensor) {
        if (!world) return;
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        if (em.HasComponent<ColliderComponent>(entity)) {
            em.GetComponent<ColliderComponent>(entity).IsSensor = isSensor;
        }
    }

    void Velox_AddPolygonCollider(VeloxWorld* world, Velox::EntityID entityID, float* verticesX, float* verticesY, int vertexCount) {
        if (!world) return;
        Velox::ColliderComponent col;
        col.Type = Velox::ColliderType::Polygon;
        col.CenterOffset = { 0, 0 };
        col.IsSensor = false;
        
        for (int i = 0; i < vertexCount; ++i) {
            col.Vertices.push_back({verticesX[i], verticesY[i]});
        }
        
        // Approximate Radius & Box Half Extents for spatial hashing broadphase
        float maxR = 0.0f;
        for (const auto& v : col.Vertices) {
            float dist = v.Magnitude();
            if (dist > maxR) maxR = dist;
        }
        col.Data.Radius = maxR;
        col.Data.BoxHalfExtents = { maxR, maxR };
        
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        em.AddComponent(entityID, col);

        // Compute proper inertia for a polygon: Approximate using solid circle or sum
        if (em.HasComponent<RigidBodyComponent>(entityID)) {
            auto& rb = em.GetComponent<RigidBodyComponent>(entityID);
            if (!rb.IsStatic && rb.Mass > 0.0f) {
                rb.Inertia = 0.5f * rb.Mass * maxR * maxR;
                rb.InverseInertia = 1.0f / rb.Inertia;
            }
        }
    }

    void Velox_AddChainCollider(VeloxWorld* world, Velox::EntityID entityID, float* pointsX, float* pointsY, int pointCount) {
        if (!world) return;
        Velox::ColliderComponent col;
        col.Type = Velox::ColliderType::Chain;
        col.CenterOffset = { 0, 0 };
        col.IsSensor = false;
        
        for (int i = 0; i < pointCount; ++i) {
            col.Vertices.push_back({pointsX[i], pointsY[i]});
        }
        
        // Dynamic bounding box for spatial hashing
        float minX = 1e9f, maxX = -1e9f, minY = 1e9f, maxY = -1e9f;
        for (const auto& v : col.Vertices) {
            if (v.x < minX) minX = v.x;
            if (v.x > maxX) maxX = v.x;
            if (v.y < minY) minY = v.y;
            if (v.y > maxY) maxY = v.y;
        }
        col.Data.BoxHalfExtents = { (maxX - minX)*0.5f, (maxY - minY)*0.5f };
        col.CenterOffset = { (minX + maxX)*0.5f, (minY + maxY)*0.5f };
        col.Data.Radius = col.Data.BoxHalfExtents.Magnitude();

        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        em.AddComponent(entityID, col);
    }

    void Velox_SetJointMotor(VeloxWorld* world, Velox::EntityID jointEntity, bool enableMotor, float targetSpeed, float maxTorque) {
        if (!world) return;
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        if (em.HasComponent<JointComponent>(jointEntity)) {
            auto& jc = em.GetComponent<JointComponent>(jointEntity);
            jc.EnableMotor = enableMotor;
            jc.MotorSpeed = targetSpeed;
            jc.MaxMotorTorque = maxTorque;
        }
    }

    bool Velox_Raycast(VeloxWorld* world, float startX, float startY, float dirX, float dirY, float maxDistance, float* hitX, float* hitY, float* normalX, float* normalY, float* fraction, Velox::EntityID* hitEntity) {
        if (!world) return false;
        auto& phys = reinterpret_cast<World*>(world)->GetPhysicsSystem();
        Velox::Vec2 start = {startX, startY};
        Velox::Vec2 dir = {dirX, dirY};
        
        // Normalize raycast direction
        float len = dir.Magnitude();
        if (len > 0.0001f) {
            dir = dir / len;
        }
        
        Velox::Vec2 hitPt, hitNorm;
        Velox::Real frac = 0.0f;
        Velox::EntityID id = 0;
        
        bool hit = phys.Raycast(start, dir, maxDistance, hitPt, hitNorm, frac, id);
        if (hit) {
            if (hitX) *hitX = hitPt.x;
            if (hitY) *hitY = hitPt.y;
            if (normalX) *normalX = hitNorm.x;
            if (normalY) *normalY = hitNorm.y;
            if (fraction) *fraction = frac;
            if (hitEntity) *hitEntity = id;
            return true;
        }
        return false;
    }

}