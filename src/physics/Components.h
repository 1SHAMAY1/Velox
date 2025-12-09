#pragma once

#include "../math/Vec3.h"
#include "../math/Vec2.h"
#include "../math/Quat.h"
#include <vector>

namespace Velox {

    struct TransformComponent {
        Vec2 Position;
        Real Rotation; // Radians
        Vec2 Scale = {1.0f, 1.0f};
    };

    struct MovementComponent {
        Vec2 Velocity;
        Real AngularVelocity;
        Vec2 Force;
        Real Torque;
        
        Real LinearDamping = 0.0f;
        Real AngularDamping = 0.0f;
    };

    struct RigidBodyComponent {
        Real Mass = 1.0f;
        Real InverseMass = 1.0f; // 0.0f for static
        Real Inertia = 1.0f;
        Real InverseInertia = 1.0f; // 0.0f for fixed rotation
        
        bool IsStatic = false;
    };

    struct PhysicalMaterialComponent {
        Real StaticFriction = 0.5f;
        Real DynamicFriction = 0.3f;
        Real Restitution = 0.5f; 
    };

    enum class ColliderType {
        Circle,
        Box
    };

    struct ColliderComponent {
        ColliderType Type;
        Vec2 CenterOffset; // Offset from transform
        
        // Union or struct for dimensions
        struct {
            Vec2 BoxHalfExtents;
            Real Radius;
        } Data;
    };

    enum class ForceFieldType {
        Inward,
        Outward,
        Clockwise,
        AntiClockwise
    };

    struct ForceFieldComponent {
        ForceFieldType Type;
        Real Strength;
        Real Radius;
    };

    enum class RotationDirection {
        Clockwise,
        AntiClockwise
    };

    struct RotationComponent {
        Real Speed; // Radians per second
        RotationDirection Direction;
    };

    struct OscillationComponent {
        Vec2 Axis; // Direction of oscillation
        Real Amplitude;
        Real Frequency;
        Real TimeAccumulator = 0.0f;
        Vec2 CenterPosition; // Center of oscillation
    };

    struct ProjectileComponent {
        bool FaceVelocity = true; // If true, rotation matches velocity direction
        Real Speed = 0.0f;        // Initial/Current speed magnitude (optional usage)
        Real MaxSpeed = 1000.0f;  // Cap on velocity
        Real BounceFactor = 0.1f; // Energy retained after collision (0.0 = stick, 1.0 = bounce)
    };



}
