#pragma once

/**
 * @file Components.h
 * @brief Plain-data component structs for the Velox ECS.
 *
 * Components are pure POD structs — no logic, no virtual functions.
 * They define *what* an entity is; systems define *what it does*.
 */

#include "../math/Vec3.h"
#include "../math/Vec2.h"
#include "../math/Quat.h"
#include <vector>

namespace Velox {

    // -------------------------------------------------------------------------
    // Core Physics Components
    // -------------------------------------------------------------------------

    /// World-space location and orientation of an entity.
    struct TransformComponent {
        Vec2 Position;
        Real Rotation; ///< Orientation in radians.
        Vec2 Scale = {1.0f, 1.0f};
    };

    /**
     * @brief Dynamic motion state for physics integration.
     *
     * Stores velocities, accumulated forces, and XPBD solver state.
     * PrevPosition / PrevVelocity are snapshotted before each sub-step
     * so restitution can be computed from the true pre-collision approach speed.
     */
    struct MovementComponent {
        Vec2 Velocity;
        Real AngularVelocity = 0.0f;
        Vec2 Force;
        Real Torque = 0.0f;

        Real LinearDamping  = 0.0f; ///< Velocity decay per second (0 = no damping).
        Real AngularDamping = 0.0f; ///< Angular velocity decay per second.

        // XPBD solver state — written each sub-step.
        Vec2 PrevPosition;
        Real PrevRotation = 0.0f;

        // Pre-integration velocity snapshot used for restitution gating.
        Vec2 PrevVelocity;
        Real PrevAngularVelocity = 0.0f;
    };

    /**
     * @brief Mass and inertia properties of a rigid body.
     *
     * InverseMass == 0 marks the body as static (infinite mass).
     * InverseInertia == 0 locks rotation regardless of applied torque.
     */
    struct RigidBodyComponent {
        Real Mass          = 1.0f;
        Real InverseMass   = 1.0f;
        Real Inertia       = 1.0f;
        Real InverseInertia = 1.0f;
        bool IsStatic      = false;
    };

    /// Surface material properties used during impulse resolution.
    struct PhysicalMaterialComponent {
        Real StaticFriction  = 0.5f;
        Real DynamicFriction = 0.3f;
        Real Restitution     = 0.5f; ///< 0 = inelastic, 1 = perfectly elastic.
    };

    // -------------------------------------------------------------------------
    // Collider Component
    // -------------------------------------------------------------------------

    /// Supported collider geometry types.
    enum class ColliderType {
        Circle,
        Box,
        Polygon,
        Chain   ///< Open chain of line segments — one-sided, for terrain/floors.
    };

    /**
     * @brief Collision shape description attached to an entity.
     *
     * Only the fields relevant to the active ColliderType are valid:
     *   - Circle  → Data.Radius
     *   - Box     → Data.BoxHalfExtents
     *   - Polygon → Vertices (local-space convex hull, CCW winding)
     *   - Chain   → Vertices (local-space polyline points, open chain)
     */
    struct ColliderComponent {
        ColliderType Type;
        Vec2 CenterOffset;   ///< Shape offset from the entity's transform position.
        bool IsSensor = false; ///< Sensors detect overlaps but do not generate response forces.

        struct {
            Vec2 BoxHalfExtents; ///< Half-width and half-height for Box colliders.
            Real Radius;         ///< Radius for Circle colliders.
        } Data;

        /// Vertex buffer — local-space for Polygon; world-space polyline for Chain.
        std::vector<Vec2> Vertices;
    };

    // -------------------------------------------------------------------------
    // Behaviour Components
    // -------------------------------------------------------------------------

    /// Radial force field type applied to dynamic bodies within its radius.
    enum class ForceFieldType {
        Inward,       ///< Gravity well — pulls bodies toward the centre.
        Outward,      ///< Repulsor — pushes bodies away from the centre.
        Clockwise,    ///< Vortex — tangential force producing clockwise rotation.
        AntiClockwise ///< Vortex — tangential force producing counter-clockwise rotation.
    };

    /// Applies a radial physics force to all dynamic bodies within its radius.
    struct ForceFieldComponent {
        ForceFieldType Type;
        Real Strength; ///< Force magnitude at the edge of the field (falls off linearly).
        Real Radius;   ///< Influence radius in world units.
    };

    /// Constant rotation applied every frame — useful for motors and spinning platforms.
    enum class RotationDirection { Clockwise, AntiClockwise };

    struct RotationComponent {
        Real Speed;               ///< Angular speed in radians per second.
        RotationDirection Direction;
    };

    /**
     * @brief Sinusoidal oscillation along an axis — useful for moving platforms.
     *
     * Computes position as: CenterPosition + Axis * Amplitude * sin(2π * Frequency * t)
     */
    struct OscillationComponent {
        Vec2 Axis;          ///< Normalised oscillation direction.
        Real Amplitude;     ///< Peak displacement from CenterPosition (world units).
        Real Frequency;     ///< Oscillations per second (Hz).
        Real TimeAccumulator = 0.0f;
        Vec2 CenterPosition; ///< Equilibrium point of the oscillation.
    };

    /**
     * @brief Projectile behaviour — aligns body rotation to its velocity direction.
     * Useful for arrows, missiles, or any object that should "face" its trajectory.
     */
    struct ProjectileComponent {
        bool FaceVelocity  = true;     ///< If true, rotation tracks the velocity vector.
        Real Speed         = 0.0f;     ///< Initial launch speed (optional).
        Real MaxSpeed      = 1000.0f;  ///< Speed cap applied each frame.
        Real BounceFactor  = 0.1f;     ///< Energy retention on surface contact (0=stick, 1=elastic).
    };

    // -------------------------------------------------------------------------
    // Constraint Components
    // -------------------------------------------------------------------------

    /**
     * @brief XPBD distance joint between two entities.
     *
     * Maintains a fixed distance between two anchor points.
     * Compliance controls stiffness: 0 = rigid, >0 = spring-like.
     * Optional motor can apply a constant angular drive on EntityA.
     */
    struct JointComponent {
        EntityID EntityA;
        EntityID EntityB;
        Vec2 LocalAnchorA;      ///< Anchor offset relative to EntityA's position.
        Vec2 LocalAnchorB;      ///< Anchor offset relative to EntityB's position.
        Real TargetDistance;
        Real Compliance = 0.0f; ///< Inverse stiffness (0 = completely rigid).
        Real Damping    = 0.0f;
        bool IsActive   = true;

        // Motor
        bool  HasMotor        = false;
        bool  EnableMotor     = false; ///< Runtime toggle — disables motor force without removing the joint.
        Real  MotorSpeed      = 0.0f;  ///< Target angular speed (rad/s). Positive = CCW.
        Real  MaxMotorTorque  = 0.0f;  ///< Maximum torque the motor can apply per step.
        Real  MotorAngle      = 0.0f;  ///< Accumulated driven angle (rad) — internal state.
    };

} // namespace Velox