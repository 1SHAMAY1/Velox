#pragma once

/**
 * @file VeloxAPI.h
 * @brief Flat C ABI for embedding the Velox physics engine from any language.
 *
 * All functions take an opaque VeloxWorld* obtained from Velox_CreateWorld()
 * and are safe to call across a DLL/shared-library boundary. Entities are
 * plain integer handles (Velox::EntityID) with no lifetime tracking beyond
 * Velox_DestroyEntity().
 */

#include "Types.h"

extern "C" {

    /// Opaque handle to a simulated scene. Create with Velox_CreateWorld(), free with Velox_DestroyWorld().
    typedef struct VeloxWorld VeloxWorld;

    // --- World Management ---

    /// Creates a new, empty simulation world with zero gravity.
    VELOX_API VeloxWorld* Velox_CreateWorld();

    /// Destroys a world and frees all of its entities/components.
    VELOX_API void Velox_DestroyWorld(VeloxWorld* world);

    /// Advances the simulation by `dt` seconds (internally sub-stepped for stability).
    VELOX_API void Velox_Step(VeloxWorld* world, float dt);

    // --- Entity Management ---

    /// Allocates a new entity handle in `world`.
    VELOX_API Velox::EntityID Velox_CreateEntity(VeloxWorld* world);

    /// Destroys `entity` and removes all of its components.
    VELOX_API void Velox_DestroyEntity(VeloxWorld* world, Velox::EntityID entity);


    // --- Components ---

    /// Adds a TransformComponent (position + rotation, unit scale) to `entity`.
    VELOX_API void Velox_AddTransform(VeloxWorld* world, Velox::EntityID entity, float x, float y, float rotation);

    /// Adds a RigidBodyComponent. Pass isStatic=true (or mass=0) for immovable bodies.
    VELOX_API void Velox_AddRigidBody(VeloxWorld* world, Velox::EntityID entity, float mass, bool isStatic);

    /// Adds a MovementComponent (velocity, angular velocity, damping) initialized to rest.
    VELOX_API void Velox_AddMovement(VeloxWorld* world, Velox::EntityID entity);

    /// Adds a RotationComponent that spins `entity` continuously at `speed` rad/s.
    /// `mode` is reserved for future rotation modes and is currently unused.
    VELOX_API void Velox_AddRotation(VeloxWorld* world, Velox::EntityID entity, float speed, int direction, int mode);

    /// Adds an OscillationComponent that moves `entity` sinusoidally along (axisX, axisY)
    /// around its current position, with the given amplitude and frequency.
    VELOX_API void Velox_AddOscillation(VeloxWorld* world, Velox::EntityID entity, float axisX, float axisY, float amplitude, float frequency);

    /// Adds a ProjectileComponent so `entity`'s rotation tracks its velocity direction.
    VELOX_API void Velox_AddProjectile(VeloxWorld* world, Velox::EntityID entity, bool faceVelocity, float speed, float maxSpeed, float bounceFactor);

    /// Adds a PhysicalMaterialComponent controlling friction and restitution during collisions.
    VELOX_API void Velox_AddPhysicalMaterial(VeloxWorld* world, Velox::EntityID entity, float staticFriction, float dynamicFriction, float restitution);

    /// Adds a ForceFieldComponent that pulls, pushes, or spins dynamic bodies within `radius`.
    /// `type` maps to Velox::ForceFieldType (Inward, Outward, Clockwise, AntiClockwise).
    VELOX_API void Velox_AddForceField(VeloxWorld* world, int entityID, int type, float strength, float radius);

    // --- Colliders ---

    /// Adds a circular collider of `radius` and updates the body's inertia accordingly.
    VELOX_API void Velox_AddCircleCollider(VeloxWorld* world, Velox::EntityID entityID, float radius);

    /// Adds an axis-aligned (pre-rotation) box collider of `width` x `height` and updates inertia.
    VELOX_API void Velox_AddBoxCollider(VeloxWorld* world, Velox::EntityID entityID, float width, float height);

    /// Adds a convex polygon collider from parallel X/Y vertex arrays (local space, CCW winding).
    VELOX_API void Velox_AddPolygonCollider(VeloxWorld* world, Velox::EntityID entityID, float* verticesX, float* verticesY, int vertexCount);

    /// Adds an open, one-sided chain collider from parallel X/Y point arrays — ideal for terrain/floors.
    VELOX_API void Velox_AddChainCollider(VeloxWorld* world, Velox::EntityID entityID, float* pointsX, float* pointsY, int pointCount);

    // --- Setters ---

    /// Sets `entity`'s linear velocity directly (requires a MovementComponent).
    VELOX_API void Velox_SetVelocity(VeloxWorld* world, Velox::EntityID entity, float x, float y);

    /// Sets `entity`'s linear and angular velocity damping (requires a MovementComponent).
    VELOX_API void Velox_SetDamping(VeloxWorld* world, Velox::EntityID entity, float linear, float angular);

    // --- Getters ---

    /// Reads `entity`'s current position and rotation into the output pointers.
    /// Leaves the outputs untouched if `entity` has no TransformComponent.
    VELOX_API void Velox_GetPosition(VeloxWorld* world, Velox::EntityID entity, float* x, float* y, float* rotation);

    // --- Gravity Configuration ---

    /// Sets the world's global gravity as a free vector, e.g. (0, 980) for downward gravity.
    VELOX_API void Velox_SetGravity(VeloxWorld* world, float gx, float gy);

    // --- Joint System ---

    /// Creates an XPBD distance joint holding `entityA` and `entityB` at `targetDistance` apart,
    /// anchored at the given local offsets. `compliance` of 0 is rigid; higher values act springy.
    VELOX_API void Velox_AddDistanceJoint(VeloxWorld* world, Velox::EntityID entityA, Velox::EntityID entityB, float anchorAX, float anchorAY, float anchorBX, float anchorBY, float targetDistance, float compliance);

    // --- Sensor Configuration ---

    /// Marks `entity`'s collider as a sensor (detects overlap but produces no collision response).
    VELOX_API void Velox_SetColliderSensor(VeloxWorld* world, Velox::EntityID entity, bool isSensor);

    // --- Motor System ---

    /// Enables/disables the angular motor on a distance joint entity and sets its target speed and torque limit.
    VELOX_API void Velox_SetJointMotor(VeloxWorld* world, Velox::EntityID jointEntity, bool enableMotor, float targetSpeed, float maxTorque);

    // --- Raycast System ---

    /// Casts a ray from (startX, startY) along the (dirX, dirY) direction, up to maxDistance.
    /// On hit, fills the output pointers (any of which may be null) and returns true.
    /// `fraction` is normalized to [0, 1] over maxDistance.
    VELOX_API bool Velox_Raycast(VeloxWorld* world, float startX, float startY, float dirX, float dirY, float maxDistance, float* hitX, float* hitY, float* normalX, float* normalY, float* fraction, Velox::EntityID* hitEntity);

}