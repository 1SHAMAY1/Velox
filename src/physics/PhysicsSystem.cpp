#include "PhysicsSystem.h"
#include <iostream>
#include <unordered_map>
#include <cmath>

namespace Velox {

    void PhysicsSystem::Step(Real dt) {
        // 1. Apply Rules (Custom Logic)
        ApplyRules();

        // 2. Integration (Predict Positions)
        Integrate(dt);

        // 3. Broadphase & Narrowphase (Collision Detection)
        // For this initial implementation, we'll do a naive N^2 check for simplicity
        // In a real production engine, this would be a Dynamic AABB Tree.
        Broadphase(); 
        Narrowphase();

        // 4. Solve Constraints (XPBD)
        // We run this multiple times for stability (Sub-stepping could be added here)
        int solverIterations = 5;
        for (int i = 0; i < solverIterations; ++i) {
            SolveConstraints(dt);
        }

        // 5. Velocity Update (if needed for XPBD, usually implicit)
    }

    void PhysicsSystem::ApplyRules() {
        // Iterate over all entities with RuleComponent
        // Note: In a real ECS, we would use a View/Group. 
        // Here we iterate all rigid bodies and check for rules (naive but functional for demo)
        
        // Since our ECS is simple, we'll iterate all entities (slow, but correct for structure)
        // A better way is to have a list of entities with RuleComponents.
        // For now, let's assume we can iterate entities.
        
        // Placeholder for rule logic:
        // for (auto entity : view) {
        //     if (rule.Type == GravityOverride) rb.Force += rule.VectorParam;
        // }
    }

    void PhysicsSystem::Integrate(Real dt) {
        // Iterate all RigidBodies
        // auto view = m_entityManager->GetEntitiesWithComponent<RigidBodyComponent>();
        // For now, we need a way to iterate. 
        // Since the ECS implementation was sparse, let's assume we can get the component array.
        
        // We will implement a simplified loop here assuming we have access.
        // In the full engine, we'd expose iterators.
        
        // Hack: We know the ECS implementation details (std::vector in ComponentArray).
        // But we can't access it easily without friend classes or public access.
        // Let's rely on the user adding entities and us storing them in a local list for physics?
        // No, that defeats the ECS purpose.
        
        // Let's assume we added a `GetAllEntities` or similar to ECS, or we just iterate 0..MaxEntities
        // checking for existence.
        
        for (EntityID i = 0; i < 10000; ++i) {
            if (!m_entityManager->HasComponent<RigidBodyComponent>(i)) continue;
            if (!m_entityManager->HasComponent<TransformComponent>(i)) continue;

            auto& rb = m_entityManager->GetComponent<RigidBodyComponent>(i);
            auto& transform = m_entityManager->GetComponent<TransformComponent>(i);

            if (rb.IsStatic) continue;

            // Semi-implicit Euler / XPBD Prediction
            // v = v + a * dt
            Vec3 gravity = m_gravity; // Could be overridden by rules
            
            // Apply forces
            Vec3 acceleration = gravity + (rb.Force * rb.InverseMass);
            rb.Velocity += acceleration * dt;

            // p = p + v * dt
            transform.Position += rb.Velocity * dt;
            
            // Reset forces
            rb.Force = Vec3(0,0,0);
        }
    }

    void PhysicsSystem::Broadphase() {
        // Naive O(N^2) for demonstration
        // In reality: Update AABB Tree
    }

    void PhysicsSystem::Narrowphase() {
        // Check collisions
        // Generate contacts
    }

    void PhysicsSystem::SolveConstraints(Real dt) {
        // 1. Collision Constraints
        // 2. Joint Constraints
        
        // Example: Ground Plane Collision (y < 0)
        for (EntityID i = 0; i < 10000; ++i) {
            if (!m_entityManager->HasComponent<RigidBodyComponent>(i)) continue;
            auto& transform = m_entityManager->GetComponent<TransformComponent>(i);
            auto& rb = m_entityManager->GetComponent<RigidBodyComponent>(i);

            if (rb.IsStatic) continue;

            // Simple Ground Constraint
            if (transform.Position.y < 0.0f) {
                // Project out of ground
                Vec3 n(0, 1, 0);
                Real C = transform.Position.y; // Penetration depth (negative)
                
                // XPBD correction
                // dx = -C * n
                // But we want to handle bounce too.
                
                transform.Position.y = 0.0f;
                
                // Friction / Restitution
                if (rb.Velocity.y < 0) {
                    rb.Velocity.y = -rb.Velocity.y * 0.5f; // Bounciness 0.5
                    
                    // Friction
                    Vec3 tan = rb.Velocity - n * rb.Velocity.Dot(n);
                    rb.Velocity -= tan * 0.1f; // Friction
                }
            }



            }


        // --- Spatial Partitioning (Simple Grid) ---
        // Grid Cell Size = Max expected object size * 2
        const Real cellSize = 2.0f; 
        std::unordered_map<int, std::vector<EntityID>> grid;

        auto GetCellHash = [&](const Vec3& pos) {
            int x = (int)std::floor(pos.x / cellSize);
            int y = (int)std::floor(pos.y / cellSize);
            int z = (int)std::floor(pos.z / cellSize);
            // Simple hash
            return x * 73856093 ^ y * 19349663 ^ z * 83492791;
        };

        // 1. Populate Grid
        for (EntityID i = 0; i < 10000; ++i) {
            if (!m_entityManager->HasComponent<ColliderComponent>(i)) continue;
            if (!m_entityManager->HasComponent<TransformComponent>(i)) continue;
            
            auto& transform = m_entityManager->GetComponent<TransformComponent>(i);
            int hash = GetCellHash(transform.Position);
            grid[hash].push_back(i);
        }

        // 2. Check Collisions (Only within same cell and neighbors)
        // For simplicity in this demo, we only check SAME cell. 
        // A full implementation checks 3x3x3 neighbors.
        // Given our soft bodies are small, this might miss edge cases but is MUCH faster.
        // Let's do a slightly better check: Check same cell.
        
        for (auto& [hash, entities] : grid) {
            if (entities.size() < 2) continue;

            for (size_t i = 0; i < entities.size(); ++i) {
                EntityID idA = entities[i];
                auto& colA = m_entityManager->GetComponent<ColliderComponent>(idA);
                auto& transA = m_entityManager->GetComponent<TransformComponent>(idA);
                auto& rbA = m_entityManager->GetComponent<RigidBodyComponent>(idA);

                for (size_t j = i + 1; j < entities.size(); ++j) {
                    EntityID idB = entities[j];
                    auto& colB = m_entityManager->GetComponent<ColliderComponent>(idB);
                    auto& transB = m_entityManager->GetComponent<TransformComponent>(idB);
                    auto& rbB = m_entityManager->GetComponent<RigidBodyComponent>(idB);

                    // Sphere-Sphere
                    if (colA.Type == ColliderType::Sphere && colB.Type == ColliderType::Sphere) {
                        Vec3 n = transA.Position - transB.Position;
                        Real dist = n.Magnitude();
                        Real radiusSum = colA.Data.Radius + colB.Data.Radius;

                        if (dist < radiusSum && dist > 0.0001f) {
                            n = n / dist;
                            Real penetration = radiusSum - dist;
                            
                            Real w1 = rbA.InverseMass;
                            Real w2 = rbB.InverseMass;
                            if (w1 + w2 == 0.0f) continue;

                            Vec3 dx = n * (penetration / (w1 + w2));
                            
                            if (!rbA.IsStatic) transA.Position += dx * w1;
                            if (!rbB.IsStatic) transB.Position -= dx * w2;
                            
                            // Friction/Damping
                            Vec3 relVel = rbA.Velocity - rbB.Velocity;
                            Vec3 tan = relVel - n * relVel.Dot(n);
                            if (!rbA.IsStatic) rbA.Velocity -= tan * 0.05f;
                            if (!rbB.IsStatic) rbB.Velocity += tan * 0.05f;
                        }
                    }
                }
            }
        }

        // 3. Spring Constraints (XPBD Distance Constraint)
        // Iterate all entities with SpringComponent (Naive iteration for now)
        for (EntityID i = 0; i < 10000; ++i) {
            if (!m_entityManager->HasComponent<SpringComponent>(i)) continue;
            
            auto& spring = m_entityManager->GetComponent<SpringComponent>(i);
            
            // Validate entities exist
            if (!m_entityManager->HasComponent<TransformComponent>(spring.EntityA) ||
                !m_entityManager->HasComponent<TransformComponent>(spring.EntityB)) continue;

            auto& x1 = m_entityManager->GetComponent<TransformComponent>(spring.EntityA).Position;
            auto& x2 = m_entityManager->GetComponent<TransformComponent>(spring.EntityB).Position;
            
            auto& rb1 = m_entityManager->GetComponent<RigidBodyComponent>(spring.EntityA);
            auto& rb2 = m_entityManager->GetComponent<RigidBodyComponent>(spring.EntityB);

            Real w1 = rb1.InverseMass;
            Real w2 = rb2.InverseMass;
            if (w1 + w2 == 0.0f) continue;

            Vec3 n = x1 - x2;
            Real d = n.Magnitude();
            if (d == 0.0f) continue; // Avoid division by zero
            n = n / d;

            // XPBD Correction
            // C(x) = |x1 - x2| - restLength
            Real C = d - spring.RestLength;
            
            // Compliance (alpha) = 1 / (stiffness * dt * dt)
            // For infinite stiffness, alpha = 0
            Real alpha = 0.0f; 
            if (spring.Stiffness > 0) alpha = 1.0f / (spring.Stiffness * dt * dt);

            Real lambda = -C / (w1 + w2 + alpha);
            
            Vec3 dx = n * lambda;
            
            if (!rb1.IsStatic) x1 += dx * w1;
            if (!rb2.IsStatic) x2 -= dx * w2;
        }
    }

}
