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
#include <iostream>

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

    bool Velox_IsSleeping(VeloxWorld* world, EntityID entity) {
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        if (em.HasComponent<RigidBodyComponent>(entity)) {
            return em.GetComponent<RigidBodyComponent>(entity).IsSleeping;
        }
        return false;
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
        (void)mode;
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
        if (em.HasComponent<RigidBodyComponent>(entity)) {
            auto& rb = em.GetComponent<RigidBodyComponent>(entity);
            rb.IsSleeping = false;
            rb.SleepTimer = 0.0f;
        }
    }

    void Velox_SetAngularVelocity(VeloxWorld* world, EntityID entity, float angularVelocity) {
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        if (em.HasComponent<MovementComponent>(entity)) {
            auto& mc = em.GetComponent<MovementComponent>(entity);
            mc.AngularVelocity = angularVelocity;
        }
        if (em.HasComponent<RigidBodyComponent>(entity)) {
            auto& rb = em.GetComponent<RigidBodyComponent>(entity);
            rb.IsSleeping = false;
            rb.SleepTimer = 0.0f;
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

    void Velox_AddRevoluteJoint(VeloxWorld* world, Velox::EntityID entityA, Velox::EntityID entityB, float anchorAX, float anchorAY, float anchorBX, float anchorBY, float compliance, bool limitsEnabled, float lowerAngle, float upperAngle, bool enableMotor, float motorSpeed, float maxMotorTorque) {
        if (!world) return;
        auto id = reinterpret_cast<World*>(world)->GetEntityManager().CreateEntity();
        
        Velox::RevoluteJointComponent rjc;
        rjc.EntityA = entityA;
        rjc.EntityB = entityB;
        rjc.LocalAnchorA = {anchorAX, anchorAY};
        rjc.LocalAnchorB = {anchorBX, anchorBY};
        rjc.Compliance = compliance;
        rjc.IsActive = true;
        rjc.LimitsEnabled = limitsEnabled;
        rjc.LowerAngle = lowerAngle;
        rjc.UpperAngle = upperAngle;
        rjc.EnableMotor = enableMotor;
        rjc.MotorSpeed = motorSpeed;
        rjc.MaxMotorTorque = maxMotorTorque;
        
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(id, rjc);
    }

    void Velox_AddPrismaticJoint(VeloxWorld* world, Velox::EntityID entityA, Velox::EntityID entityB, float anchorAX, float anchorAY, float anchorBX, float anchorBY, float axisAX, float axisAY, float compliance, bool limitsEnabled, float minTranslation, float maxTranslation, bool enableMotor, float motorSpeed, float maxMotorForce) {
        if (!world) return;
        auto id = reinterpret_cast<World*>(world)->GetEntityManager().CreateEntity();
        
        Velox::PrismaticJointComponent pjc;
        pjc.EntityA = entityA;
        pjc.EntityB = entityB;
        pjc.LocalAnchorA = {anchorAX, anchorAY};
        pjc.LocalAnchorB = {anchorBX, anchorBY};
        pjc.LocalAxisA = {axisAX, axisAY};
        pjc.Compliance = compliance;
        pjc.IsActive = true;
        pjc.LimitsEnabled = limitsEnabled;
        pjc.MinTranslation = minTranslation;
        pjc.MaxTranslation = maxTranslation;
        pjc.EnableMotor = enableMotor;
        pjc.MotorSpeed = motorSpeed;
        pjc.MaxMotorForce = maxMotorForce;
        
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(id, pjc);
    }

    void Velox_AddGearJoint(VeloxWorld* world, Velox::EntityID entityA, Velox::EntityID entityB, float gearRatio, float compliance) {
        if (!world) return;
        auto id = reinterpret_cast<World*>(world)->GetEntityManager().CreateEntity();
        
        Velox::GearJointComponent gjc;
        gjc.EntityA = entityA;
        gjc.EntityB = entityB;
        gjc.GearRatio = gearRatio;
        gjc.Compliance = compliance;
        gjc.IsActive = true;
        
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(id, gjc);
    }

    void Velox_AddPulleyJoint(VeloxWorld* world, Velox::EntityID entityA, Velox::EntityID entityB, float groundAX, float groundAY, float groundBX, float groundBY, float anchorAX, float anchorAY, float anchorBX, float anchorBY, float ratio, float totalLength, float compliance) {
        if (!world) return;
        auto id = reinterpret_cast<World*>(world)->GetEntityManager().CreateEntity();
        
        Velox::PulleyJointComponent pjc;
        pjc.EntityA = entityA;
        pjc.EntityB = entityB;
        pjc.GroundAnchorA = {groundAX, groundAY};
        pjc.GroundAnchorB = {groundBX, groundBY};
        pjc.LocalAnchorA = {anchorAX, anchorAY};
        pjc.LocalAnchorB = {anchorBX, anchorBY};
        pjc.Ratio = ratio;
        pjc.TotalLength = totalLength;
        pjc.Compliance = compliance;
        pjc.IsActive = true;
        
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(id, pjc);
    }

    void Velox_SetColliderGroupId(VeloxWorld* world, Velox::EntityID entity, int groupId) {
        if (!world) return;
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        if (em.HasComponent<ColliderComponent>(entity)) {
            em.GetComponent<ColliderComponent>(entity).GroupId = groupId;
        }
    }

    Velox::EntityID Velox_CreateSoftBodyBlob(VeloxWorld* world, float cx, float cy, float radius, int nodeCount, float compliance, float jointCompliance, float nodeRadius) {
        if (!world || nodeCount < 3) return 0;
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();

        static int nextGroupId = 1000;
        int groupId = nextGroupId++;

        std::vector<Velox::EntityID> nodes;
        nodes.reserve(nodeCount);

        float nodeMass = 1.0f / nodeCount;

        for (int i = 0; i < nodeCount; ++i) {
            float angle = (i * 2.0f * 3.14159265f) / nodeCount;
            float nx = cx + radius * cosf(angle);
            float ny = cy + radius * sinf(angle);

            auto node = em.CreateEntity();
            em.AddComponent(node, Velox::TransformComponent{{nx, ny}, 0.0f});
            em.AddComponent(node, Velox::RigidBodyComponent{nodeMass, 1.0f / nodeMass, 0.01f, 100.0f, false});
            em.GetComponent<RigidBodyComponent>(node).AllowSleep = false;
            em.AddComponent(node, Velox::MovementComponent{});
            
            Velox::ColliderComponent col;
            col.Type = Velox::ColliderType::Circle;
            col.CenterOffset = {0.0f, 0.0f};
            col.IsSensor = false;
            col.GroupId = groupId;
            col.Data.Radius = nodeRadius;
            col.Data.BoxHalfExtents = {nodeRadius, nodeRadius};
            em.AddComponent(node, col);

            em.AddComponent(node, Velox::PhysicalMaterialComponent{0.4f, 0.2f, 0.2f});
            nodes.push_back(node);
        }

        for (int i = 0; i < nodeCount; ++i) {
            int next = (i + 1) % nodeCount;
            auto jointEntity = em.CreateEntity();
            
            float angle1 = (i * 2.0f * 3.14159265f) / nodeCount;
            float angle2 = (next * 2.0f * 3.14159265f) / nodeCount;
            float dx = radius * (cosf(angle1) - cosf(angle2));
            float dy = radius * (sinf(angle1) - sinf(angle2));
            float restDist = sqrtf(dx*dx + dy*dy);

            Velox::JointComponent jc;
            jc.EntityA = nodes[i];
            jc.EntityB = nodes[next];
            jc.LocalAnchorA = {0.0f, 0.0f};
            jc.LocalAnchorB = {0.0f, 0.0f};
            jc.TargetDistance = restDist;
            jc.Compliance = jointCompliance;
            jc.Damping = 0.5f;
            jc.IsActive = true;
            em.AddComponent(jointEntity, jc);
        }

        for (int i = 0; i < nodeCount; ++i) {
            // Connect to 3 opposite nodes to form a robust triangulation network
            int oppBase = (i + nodeCount / 2) % nodeCount;
            int offsets[] = {-1, 0, 1};
            
            for (int offset : offsets) {
                int opp = (oppBase + offset + nodeCount) % nodeCount;
                if (i >= opp) continue; // Avoid duplicate double-joints

                float angle1 = (i * 2.0f * 3.14159265f) / nodeCount;
                float angle2 = (opp * 2.0f * 3.14159265f) / nodeCount;
                float dx = radius * (cosf(angle1) - cosf(angle2));
                float dy = radius * (sinf(angle1) - sinf(angle2));
                float restDist = sqrtf(dx*dx + dy*dy);

                auto jointEntity = em.CreateEntity();
                Velox::JointComponent jc;
                jc.EntityA = nodes[i];
                jc.EntityB = nodes[opp];
                jc.LocalAnchorA = {0.0f, 0.0f};
                jc.LocalAnchorB = {0.0f, 0.0f};
                jc.TargetDistance = restDist;
                jc.Compliance = jointCompliance * 4.0f; // Sightly softer compliance for diagonals
                jc.Damping = 0.5f;
                jc.IsActive = true;
                em.AddComponent(jointEntity, jc);
            }
        }

        auto softBodyEntity = em.CreateEntity();
        Velox::SoftBodyComponent sbc;
        sbc.Type = Velox::SoftBodyType::Blob;
        sbc.Nodes = nodes;
        
        Real signedArea = 0.0f;
        int n = nodes.size();
        for (int i = 0; i < n; ++i) {
            int next = (i + 1) % n;
            auto& transCurr = em.GetComponent<TransformComponent>(nodes[i]);
            auto& transNext = em.GetComponent<TransformComponent>(nodes[next]);
            signedArea += (transCurr.Position.x * transNext.Position.y - transNext.Position.x * transCurr.Position.y);
        }
        sbc.TargetArea = 0.5f * signedArea;
        sbc.AreaCompliance = compliance;
        
        em.AddComponent(softBodyEntity, sbc);
        std::cout << "[SOFT BODY] Created Blob Soft Body Entity " << softBodyEntity 
                  << " with " << nodeCount << " nodes at center (" << cx << ", " << cy 
                  << "), radius: " << radius 
                  << ", Target Area: " << sbc.TargetArea 
                  << ", Compliance: " << compliance 
                  << ", Group ID: " << groupId << std::endl;
        return softBodyEntity;
    }

    Velox::EntityID Velox_CreateSoftBodyShapeMatched(VeloxWorld* world, float cx, float cy, float* verticesX, float* verticesY, int vertexCount, float stiffness, float nodeRadius) {
        if (!world || vertexCount < 3) return 0;
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();

        static int nextGroupId = 2000;
        int groupId = nextGroupId++;

        std::vector<Velox::EntityID> nodes;
        nodes.reserve(vertexCount);

        std::vector<Velox::Vec2> restPositions;
        restPositions.reserve(vertexCount);

        float restCx = 0.0f, restCy = 0.0f;
        for (int i = 0; i < vertexCount; ++i) {
            restCx += verticesX[i];
            restCy += verticesY[i];
        }
        restCx /= vertexCount;
        restCy /= vertexCount;

        float nodeMass = 1.0f / vertexCount;

        for (int i = 0; i < vertexCount; ++i) {
            float rx = verticesX[i] - restCx;
            float ry = verticesY[i] - restCy;
            restPositions.push_back({rx, ry});

            float nx = cx + rx;
            float ny = cy + ry;

            auto node = em.CreateEntity();
            em.AddComponent(node, Velox::TransformComponent{{nx, ny}, 0.0f});
            em.AddComponent(node, Velox::RigidBodyComponent{nodeMass, 1.0f / nodeMass, 0.01f, 100.0f, false});
            em.GetComponent<RigidBodyComponent>(node).AllowSleep = false;
            em.AddComponent(node, Velox::MovementComponent{});

            Velox::ColliderComponent col;
            col.Type = Velox::ColliderType::Circle;
            col.CenterOffset = {0.0f, 0.0f};
            col.IsSensor = false;
            col.GroupId = groupId;
            col.Data.Radius = nodeRadius;
            col.Data.BoxHalfExtents = {nodeRadius, nodeRadius};
            em.AddComponent(node, col);

            em.AddComponent(node, Velox::PhysicalMaterialComponent{0.4f, 0.2f, 0.2f});
            nodes.push_back(node);
        }

        for (int i = 0; i < vertexCount; ++i) {
            int next = (i + 1) % vertexCount;
            float dx = restPositions[i].x - restPositions[next].x;
            float dy = restPositions[i].y - restPositions[next].y;
            float restDist = sqrtf(dx*dx + dy*dy);

            auto jointEntity = em.CreateEntity();
            Velox::JointComponent jc;
            jc.EntityA = nodes[i];
            jc.EntityB = nodes[next];
            jc.LocalAnchorA = {0.0f, 0.0f};
            jc.LocalAnchorB = {0.0f, 0.0f};
            jc.TargetDistance = restDist;
            jc.Compliance = 0.01f;
            jc.Damping = 0.5f;
            jc.IsActive = true;
            em.AddComponent(jointEntity, jc);
        }

        auto softBodyEntity = em.CreateEntity();
        Velox::SoftBodyComponent sbc;
        sbc.Type = Velox::SoftBodyType::ShapeMatched;
        sbc.Nodes = nodes;
        sbc.RestPositions = restPositions;
        sbc.Stiffness = stiffness;

        em.AddComponent(softBodyEntity, sbc);
        std::cout << "[SOFT BODY] Created Shape-Matched Soft Body Entity " << softBodyEntity 
                  << " with " << vertexCount << " nodes at center (" << cx << ", " << cy 
                  << "), Stiffness: " << stiffness 
                  << ", Group ID: " << groupId << std::endl;
        return softBodyEntity;
    }

    int Velox_GetSoftBodyNodeCount(VeloxWorld* world, Velox::EntityID softBodyEntity) {
        if (!world) return 0;
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        if (em.HasComponent<SoftBodyComponent>(softBodyEntity)) {
            return em.GetComponent<SoftBodyComponent>(softBodyEntity).Nodes.size();
        }
        return 0;
    }

    Velox::EntityID Velox_GetSoftBodyNode(VeloxWorld* world, Velox::EntityID softBodyEntity, int nodeIndex) {
        if (!world) return 0;
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        if (em.HasComponent<SoftBodyComponent>(softBodyEntity)) {
            auto& sb = em.GetComponent<SoftBodyComponent>(softBodyEntity);
            if (nodeIndex >= 0 && nodeIndex < (int)sb.Nodes.size()) {
                return sb.Nodes[nodeIndex];
            }
        }
        return 0;
    }
}