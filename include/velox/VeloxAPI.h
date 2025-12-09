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
    VELOX_API void Velox_AddTransform(VeloxWorld* world, Velox::EntityID entity, float x, float y, float rotation);
    VELOX_API void Velox_AddRigidBody(VeloxWorld* world, Velox::EntityID entity, float mass, bool isStatic);
    VELOX_API void Velox_AddMovement(VeloxWorld* world, Velox::EntityID entity);
    VELOX_API void Velox_AddRotation(VeloxWorld* world, Velox::EntityID entity, float speed, int direction, int mode);
    VELOX_API void Velox_AddOscillation(VeloxWorld* world, Velox::EntityID entity, float axisX, float axisY, float amplitude, float frequency);
    VELOX_API void Velox_AddProjectile(VeloxWorld* world, Velox::EntityID entity, bool faceVelocity, float speed, float maxSpeed, float bounceFactor);
    VELOX_API void Velox_AddPhysicalMaterial(VeloxWorld* world, Velox::EntityID entity, float staticFriction, float dynamicFriction, float restitution);

    VELOX_API void Velox_AddForceField(VeloxWorld* world, int entityID, int type, float strength, float radius);

    // Colliders
    VELOX_API void Velox_AddCircleCollider(VeloxWorld* world, Velox::EntityID entityID, float radius);
    VELOX_API void Velox_AddBoxCollider(VeloxWorld* world, Velox::EntityID entityID, float width, float height);

    // Setters
    VELOX_API void Velox_SetVelocity(VeloxWorld* world, Velox::EntityID entity, float x, float y);
    VELOX_API void Velox_SetDamping(VeloxWorld* world, Velox::EntityID entity, float linear, float angular);

    // Getters
    VELOX_API void Velox_GetPosition(VeloxWorld* world, Velox::EntityID entity, float* x, float* y, float* rotation);

}
