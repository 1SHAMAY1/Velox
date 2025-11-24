#pragma once

#include "Types.h"

extern "C" {

    // Opaque pointer types
    typedef struct VeloxWorld VeloxWorld;
    
    // World Management
    VELOX_API VeloxWorld* Velox_CreateWorld();
    VELOX_API void Velox_DestroyWorld(VeloxWorld* world);
    VELOX_API void Velox_Step(VeloxWorld* world, float dt);

    // Entity Management
    VELOX_API Velox::EntityID Velox_CreateEntity(VeloxWorld* world);
    VELOX_API void Velox_DestroyEntity(VeloxWorld* world, Velox::EntityID entity);

    // Components
    VELOX_API void Velox_AddTransform(VeloxWorld* world, Velox::EntityID entity, float x, float y, float z);
    VELOX_API void Velox_GetPosition(VeloxWorld* world, Velox::EntityID entity, float* x, float* y, float* z);
    
    VELOX_API void Velox_AddRigidBody(VeloxWorld* world, Velox::EntityID entity, float mass, bool isStatic);
    VELOX_API void Velox_SetVelocity(VeloxWorld* world, Velox::EntityID entity, float x, float y, float z);
    
    // Soft Body / Springs
    VELOX_API void Velox_AddSpring(VeloxWorld* world, Velox::EntityID entityA, Velox::EntityID entityB, float stiffness, float damping);
    VELOX_API void Velox_AddCollider(VeloxWorld* world, Velox::EntityID entityID, int type, float radius); // type: 0=Sphere, 1=Box

}
