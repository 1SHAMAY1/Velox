#pragma once

#include "../math/Vec3.h"
#include "../math/Quat.h"
#include <vector>

namespace Velox {

    struct TransformComponent {
        Vec3 Position;
        Quat Rotation;
        Vec3 Scale = {1.0f, 1.0f, 1.0f};
    };

    struct RigidBodyComponent {
        Vec3 Velocity;
        Vec3 AngularVelocity;
        
        Real Mass = 1.0f;
        Real InverseMass = 1.0f; // 0.0f for static
        
        Vec3 Force;
        Vec3 Torque;
        
        bool IsStatic = false;
        Real LinearDamping = 0.0f;
        Real AngularDamping = 0.0f;
    };

    enum class ColliderType {
        Sphere,
        Box,
        Capsule,
        Plane
    };

    struct ColliderComponent {
        ColliderType Type;
        Vec3 CenterOffset; // Offset from transform
        
        // Union or struct for dimensions
        struct {
            Vec3 BoxHalfExtents;
            Real Radius;
            Real Height; // For capsule
        } Data;
    };

    struct PhysicsMaterialComponent {
        Real StaticFriction = 0.5f;
        Real DynamicFriction = 0.3f;
        Real Restitution = 0.2f; // Bounciness
    };

    // For the custom rule system
    enum class RuleType {
        GravityOverride,
        FrictionOverride,
        CustomForce
    };

    struct Rule {
        RuleType Type;
        Vec3 VectorParam; // e.g., Gravity vector
        Real ScalarParam; // e.g., Friction multiplier
    };

    struct RuleComponent {
        // We can store a small fixed number of rules inline for performance,
        // or use a vector. For now, let's use a vector.
        std::vector<Rule> ActiveRules;
    };

    struct SpringComponent {
        EntityID EntityA;
        EntityID EntityB;
        Real RestLength;
        Real Stiffness; // Compliance = 1 / Stiffness
        Real Damping;
    };

}
