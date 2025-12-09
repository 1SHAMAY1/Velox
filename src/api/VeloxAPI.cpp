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
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(entityID, col);
    }

    void Velox_AddBoxCollider(VeloxWorld* world, Velox::EntityID entityID, float width, float height) {
        Velox::ColliderComponent col;
        col.Type = Velox::ColliderType::Box;
        col.Data.BoxHalfExtents = { width * 0.5f, height * 0.5f };
        col.CenterOffset = { 0, 0 };
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(entityID, col);
    }

}
