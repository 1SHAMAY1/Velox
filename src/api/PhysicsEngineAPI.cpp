#include <velox/PhysicsEngineAPI.h>
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

    void Velox_AddTransform(VeloxWorld* world, EntityID entity, float x, float y, float z) {
        TransformComponent tc;
        tc.Position = {x, y, z};
        tc.Rotation = Quat::Identity();
        tc.Scale = {1, 1, 1};
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(entity, tc);
    }

    void Velox_GetPosition(VeloxWorld* world, EntityID entity, float* x, float* y, float* z) {
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        if (em.HasComponent<TransformComponent>(entity)) {
            auto& tc = em.GetComponent<TransformComponent>(entity);
            *x = tc.Position.x;
            *y = tc.Position.y;
            *z = tc.Position.z;
        }
    }

    void Velox_AddRigidBody(VeloxWorld* world, EntityID entity, float mass, bool isStatic) {
        RigidBodyComponent rb;
        rb.Mass = mass;
        rb.InverseMass = (isStatic || mass == 0.0f) ? 0.0f : 1.0f / mass;
        rb.IsStatic = isStatic;
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(entity, rb);
    }

    void Velox_AddCollider(VeloxWorld* world, Velox::EntityID entityID, int type, float radius) {
        Velox::ColliderComponent col;
        col.Type = (Velox::ColliderType)type;
        col.Data.Radius = radius;
        col.CenterOffset = { 0, 0, 0 };
        reinterpret_cast<World*>(world)->GetEntityManager().AddComponent(entityID, col);
    }

    void Velox_SetVelocity(VeloxWorld* world, EntityID entity, float x, float y, float z) {
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        if (em.HasComponent<RigidBodyComponent>(entity)) {
            auto& rb = em.GetComponent<RigidBodyComponent>(entity);
            rb.Velocity = {x, y, z};
        }
    }

    void Velox_AddSpring(VeloxWorld* world, EntityID entity, EntityID otherEntity, float stiffness, float damping) {
        auto& em = reinterpret_cast<World*>(world)->GetEntityManager();
        
        // Calculate rest length based on current distance
        float x1, y1, z1, x2, y2, z2;
        Velox_GetPosition(world, entity, &x1, &y1, &z1);
        Velox_GetPosition(world, otherEntity, &x2, &y2, &z2);
        
        float dx = x1 - x2;
        float dy = y1 - y2;
        float dz = z1 - z2;
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        SpringComponent sc;
        sc.EntityA = entity;
        sc.EntityB = otherEntity;
        sc.RestLength = dist;
        sc.Stiffness = stiffness;
        sc.Damping = damping;
        
        em.AddComponent(entity, sc);
    }

}
