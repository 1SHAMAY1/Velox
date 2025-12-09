#include "PhysicsSystem.h"
#include <iostream>
#include <unordered_map>
#include <cmath>
#include <algorithm>

namespace Velox {

    void PhysicsSystem::Step(Real dt) {
        ApplyRules();

        // Sub-stepping for stability and tunneling prevention
        const int subSteps = 8;
        Real subDt = dt / (Real)subSteps;

        for (int s = 0; s < subSteps; ++s) {
            Integrate(subDt);
            SolveConstraints(subDt);
        }
    }

    void PhysicsSystem::ApplyRules() {
        // Apply Force Fields
        // 1. Gather all Force Fields
        std::vector<ForceFieldComponent> fields;
        std::vector<Vec2> fieldPositions; // For PointGravity

        for (EntityID i = 0; i < 10000; ++i) {
            if (m_entityManager->HasComponent<ForceFieldComponent>(i)) {
                fields.push_back(m_entityManager->GetComponent<ForceFieldComponent>(i));
                if (m_entityManager->HasComponent<TransformComponent>(i)) {
                    fieldPositions.push_back(m_entityManager->GetComponent<TransformComponent>(i).Position);
                } else {
                    fieldPositions.push_back({0,0});
                }
            }
        }

        // 2. Apply RotationComponent (Motor)
        for (EntityID i = 0; i < 10000; ++i) {
            if (m_entityManager->HasComponent<RotationComponent>(i) && 
                m_entityManager->HasComponent<MovementComponent>(i)) {
                
                auto& rot = m_entityManager->GetComponent<RotationComponent>(i);
                auto& move = m_entityManager->GetComponent<MovementComponent>(i);
                
                Real dir = (rot.Direction == RotationDirection::Clockwise) ? 1.0f : -1.0f;
                move.AngularVelocity = rot.Speed * dir;
            }
        }

        // 3. Apply OscillationComponent
        for (EntityID i = 0; i < 10000; ++i) {
            if (m_entityManager->HasComponent<OscillationComponent>(i) && 
                m_entityManager->HasComponent<TransformComponent>(i)) {
                
                auto& osc = m_entityManager->GetComponent<OscillationComponent>(i);
                auto& trans = m_entityManager->GetComponent<TransformComponent>(i);
                
                // Update time (assuming fixed step for now, ideally pass dt to ApplyRules)
                osc.TimeAccumulator += 1.0f/60.0f; 
                
                Real offset = std::sin(osc.TimeAccumulator * osc.Frequency) * osc.Amplitude;
                trans.Position = osc.CenterPosition + osc.Axis * offset;
                
                // If it has a rigid body, we should probably update velocity too so collisions work right
                if (m_entityManager->HasComponent<MovementComponent>(i)) {
                    auto& move = m_entityManager->GetComponent<MovementComponent>(i);
                    // v = d/dt (sin(wt)*A) = w * cos(wt) * A
                    Real velMag = osc.Frequency * std::cos(osc.TimeAccumulator * osc.Frequency) * osc.Amplitude;
                    move.Velocity = osc.Axis * velMag;
                }
            }
        }

        // 2. Apply RotationComponent (Motor)
        for (EntityID i = 0; i < 10000; ++i) {
            if (m_entityManager->HasComponent<RotationComponent>(i) && 
                m_entityManager->HasComponent<MovementComponent>(i)) {
                
                auto& rot = m_entityManager->GetComponent<RotationComponent>(i);
                auto& move = m_entityManager->GetComponent<MovementComponent>(i);
                
                Real dir = (rot.Direction == RotationDirection::Clockwise) ? 1.0f : -1.0f;
                move.AngularVelocity = rot.Speed * dir;
            }
        }

        // 4. Apply ProjectileComponent (Face Velocity)
        for (EntityID i = 0; i < 10000; ++i) {
            if (m_entityManager->HasComponent<ProjectileComponent>(i) && 
                m_entityManager->HasComponent<MovementComponent>(i) &&
                m_entityManager->HasComponent<TransformComponent>(i)) {
                
                auto& proj = m_entityManager->GetComponent<ProjectileComponent>(i);
                auto& move = m_entityManager->GetComponent<MovementComponent>(i);
                auto& trans = m_entityManager->GetComponent<TransformComponent>(i);
                
                if (proj.FaceVelocity) {
                    // Only update if moving fast enough to have a direction
                    if (move.Velocity.MagnitudeSqr() > 0.001f) {
                        trans.Rotation = std::atan2(move.Velocity.y, move.Velocity.x);
                    }
                }
            }
        }

        // 3. Apply to all Dynamic Bodies
        for (EntityID i = 0; i < 10000; ++i) {
            if (!m_entityManager->HasComponent<RigidBodyComponent>(i) || 
                !m_entityManager->HasComponent<MovementComponent>(i) ||
                !m_entityManager->HasComponent<TransformComponent>(i)) continue;

            auto& rb = m_entityManager->GetComponent<RigidBodyComponent>(i);
            if (rb.IsStatic) continue;

            auto& move = m_entityManager->GetComponent<MovementComponent>(i);
            auto& trans = m_entityManager->GetComponent<TransformComponent>(i);

            for (size_t f = 0; f < fields.size(); ++f) {
                const auto& field = fields[f];
                
                Vec2 dir = fieldPositions[f] - trans.Position;
                Real distSqr = dir.MagnitudeSqr();
                Real r = field.Radius;
                
                // Check Radius
                if (distSqr < r * r && distSqr > 0.0001f) {
                    Real dist = std::sqrt(distSqr);
                    Vec2 normal = dir / dist; // Points TO field center
                    
                    // F = Strength * Mass (Simple acceleration field)
                    // Or F = Strength * Mass / dist (Inverse linear)
                    // Or F = Strength * Mass / distSqr (Inverse square)
                    // User said "force radius", usually implies a zone of influence.
                    // Let's use Inverse Linear for smoother fields, or just Constant inside radius?
                    // Let's use Constant * Falloff?
                    // For now, let's use a simple Linear Falloff: (1 - dist/r) * Strength
                    // This feels nice for game fields.
                    
                    Real falloff = 1.0f - (dist / r);
                    Real forceMag = field.Strength * rb.Mass * falloff;
                    
                    switch (field.Type) {
                        case ForceFieldType::Inward:
                            move.Force += normal * forceMag;
                            break;
                        case ForceFieldType::Outward:
                            move.Force -= normal * forceMag;
                            break;
                        case ForceFieldType::Clockwise: {
                            // Tangent: (-y, x) relative to normal
                            Vec2 tangent = {-normal.y, normal.x}; 
                            move.Force += tangent * forceMag;
                            break;
                        }
                        case ForceFieldType::AntiClockwise: {
                            Vec2 tangent = {normal.y, -normal.x};
                            move.Force += tangent * forceMag;
                            break;
                        }
                    }
                }
            }
        }
    }



    void PhysicsSystem::Integrate(Real dt) {
        // Count entities first to determine damping mode
        int dynamicCount = 0;
        for (EntityID i = 0; i < 10000; ++i) {
            if (m_entityManager->HasComponent<RigidBodyComponent>(i) && 
                !m_entityManager->GetComponent<RigidBodyComponent>(i).IsStatic) {
                dynamicCount++;
            }
        }

        // "Full Container" Logic: Heavy damping if overloaded
        bool isOverloaded = dynamicCount > 400;
        
        for (EntityID i = 0; i < 10000; ++i) {
            if (!m_entityManager->HasComponent<RigidBodyComponent>(i)) continue;
            if (!m_entityManager->HasComponent<TransformComponent>(i)) continue;
            if (!m_entityManager->HasComponent<MovementComponent>(i)) continue;

            auto& rb = m_entityManager->GetComponent<RigidBodyComponent>(i);
            auto& transform = m_entityManager->GetComponent<TransformComponent>(i);
            auto& move = m_entityManager->GetComponent<MovementComponent>(i);

            if (rb.IsStatic) continue;

            // Apply Damping
            // Base damping from component
            Real linearDamping = move.LinearDamping;
            Real angularDamping = move.AngularDamping;

            // "Full Container" Overload Logic
            if (isOverloaded) {
                linearDamping = std::max(linearDamping, 10.0f); 
            }

            // Apply Damping Factor
            move.Velocity = move.Velocity * (1.0f / (1.0f + linearDamping * dt));
            move.AngularVelocity = move.AngularVelocity * (1.0f / (1.0f + angularDamping * dt));

            // Apply Overload "Brake" directly if needed (legacy behavior requested)
            if (isOverloaded) {
                move.Velocity = move.Velocity * 0.90f; 
                move.AngularVelocity = move.AngularVelocity * 0.90f;
            }

            // Semi-implicit Euler / XPBD Prediction
            
            // Linear Motion
            Vec2 acceleration = move.Force * rb.InverseMass;
            move.Velocity += acceleration * dt;
            transform.Position += move.Velocity * dt;
            
            // Angular Motion
            Real angularAccel = move.Torque * rb.InverseInertia;
            move.AngularVelocity += angularAccel * dt;
            transform.Rotation += move.AngularVelocity * dt;

            // --- Hard Clamp to World Bounds (Tunneling Fix) ---
            // Visual Walls are 20px thick.
            // Screen is 1280x720.
            // Inner Bounds: [20, 1260] x [20, 700]
            // We clamp center to slightly inside to account for radius (approx 15-20).
            // Let's use 40 as safe margin.
            
            float minX = 40.0f; float maxX = 1240.0f; 
            float minY = 40.0f; float maxY = 680.0f;

            if (transform.Position.x < minX) { transform.Position.x = minX; move.Velocity.x = std::abs(move.Velocity.x); }
            if (transform.Position.x > maxX) { transform.Position.x = maxX; move.Velocity.x = -std::abs(move.Velocity.x); }
            if (transform.Position.y < minY) { transform.Position.y = minY; move.Velocity.y = std::abs(move.Velocity.y); }
            if (transform.Position.y > maxY) { transform.Position.y = maxY; move.Velocity.y = -std::abs(move.Velocity.y); }

            // Reset forces
            move.Force = Vec2(0,0);
            move.Torque = 0.0f;
        }
    }



    void PhysicsSystem::SolveConstraints(Real dt) {
        // --- O(N^2) Broadphase for Correctness ---
        std::vector<EntityID> entities;
        entities.reserve(500);

        for (EntityID i = 0; i < 10000; ++i) {
            if (m_entityManager->HasComponent<ColliderComponent>(i) && 
                m_entityManager->HasComponent<TransformComponent>(i) &&
                m_entityManager->HasComponent<MovementComponent>(i)) {
                entities.push_back(i);
            }
        }

        auto ResolveCollision = [&](EntityID idA, EntityID idB) {
            auto& colA = m_entityManager->GetComponent<ColliderComponent>(idA);
            auto& transA = m_entityManager->GetComponent<TransformComponent>(idA);
            auto& rbA = m_entityManager->GetComponent<RigidBodyComponent>(idA);
            auto& moveA = m_entityManager->GetComponent<MovementComponent>(idA);
            
            auto& colB = m_entityManager->GetComponent<ColliderComponent>(idB);
            auto& transB = m_entityManager->GetComponent<TransformComponent>(idB);
            auto& rbB = m_entityManager->GetComponent<RigidBodyComponent>(idB);
            auto& moveB = m_entityManager->GetComponent<MovementComponent>(idB);

            auto ResolveCircleCircle = [&]() {
                Vec2 n = transA.Position - transB.Position;
                Real dist = n.Magnitude();
                Real radiusSum = colA.Data.Radius + colB.Data.Radius;

                if (dist < radiusSum && dist > 0.0001f) {
                    n = n / dist;
                    Real penetration = radiusSum - dist;
                    
                    Real w1 = rbA.InverseMass;
                    Real w2 = rbB.InverseMass;
                    if (w1 + w2 == 0.0f) return;

                    Vec2 dx = n * (penetration / (w1 + w2));
                    
                    if (!rbA.IsStatic) transA.Position += dx * w1;
                    if (!rbB.IsStatic) transB.Position -= dx * w2;
                    
                    // Bounce
                    Vec2 relVel = moveA.Velocity - moveB.Velocity;
                    Real vn = relVel.Dot(n);
                    
                    // Get Material Properties
                    Real e = 0.5f; // Default
                    if (m_entityManager->HasComponent<PhysicalMaterialComponent>(idA) && 
                        m_entityManager->HasComponent<PhysicalMaterialComponent>(idB)) {
                        auto& matA = m_entityManager->GetComponent<PhysicalMaterialComponent>(idA);
                        auto& matB = m_entityManager->GetComponent<PhysicalMaterialComponent>(idB);
                        e = std::min(matA.Restitution, matB.Restitution);
                    } else if (m_entityManager->HasComponent<PhysicalMaterialComponent>(idA)) {
                        e = m_entityManager->GetComponent<PhysicalMaterialComponent>(idA).Restitution;
                    } else if (m_entityManager->HasComponent<PhysicalMaterialComponent>(idB)) {
                        e = m_entityManager->GetComponent<PhysicalMaterialComponent>(idB).Restitution;
                    }

                    // Deep Penetration Fix
                    if (penetration > 1.0f) e = 0.0f;
                    
                    if (vn < 0) {
                        Real j = -(1 + e) * vn / (w1 + w2);
                        Vec2 impulse = n * j;
                        
                        if (!rbA.IsStatic) moveA.Velocity += impulse * w1;
                        if (!rbB.IsStatic) moveB.Velocity -= impulse * w2;
                    }
                }
            };

            auto ResolveCircleBox = [&](EntityID circleID, EntityID boxID) {
                // Get components for specific IDs to handle swapping
                auto& cCol = m_entityManager->GetComponent<ColliderComponent>(circleID);
                auto& cTrans = m_entityManager->GetComponent<TransformComponent>(circleID);
                auto& cRb = m_entityManager->GetComponent<RigidBodyComponent>(circleID);
                auto& cMove = m_entityManager->GetComponent<MovementComponent>(circleID);

                auto& bCol = m_entityManager->GetComponent<ColliderComponent>(boxID);
                auto& bTrans = m_entityManager->GetComponent<TransformComponent>(boxID);
                auto& bRb = m_entityManager->GetComponent<RigidBodyComponent>(boxID);
                auto& bMove = m_entityManager->GetComponent<MovementComponent>(boxID);

                Vec2 circlePos = cTrans.Position + cCol.CenterOffset;
                Vec2 boxPos = bTrans.Position + bCol.CenterOffset;
                Vec2 boxHalf = bCol.Data.BoxHalfExtents;

                // 1. Transform Circle to Box Local Space
                Vec2 relPos = circlePos - boxPos;
                Vec2 localPos = relPos.Rotate(-bTrans.Rotation);

                // 2. Clamp in Local Space (AABB check)
                Vec2 clampedLocal = localPos;
                clampedLocal.x = std::max(-boxHalf.x, std::min(clampedLocal.x, boxHalf.x));
                clampedLocal.y = std::max(-boxHalf.y, std::min(clampedLocal.y, boxHalf.y));
                
                // 3. Transform closest point back to World Space
                Vec2 closestLocal = clampedLocal;
                Vec2 closestWorld = boxPos + closestLocal.Rotate(bTrans.Rotation);

                Vec2 n = circlePos - closestWorld;
                Real distSqr = n.MagnitudeSqr();
                Real radius = cCol.Data.Radius;

                if (distSqr < radius * radius && distSqr > 0.00001f) {
                    Real dist = std::sqrt(distSqr);
                    n = n / dist; 
                    Real penetration = radius - dist;

                    Real w1 = cRb.InverseMass;
                    Real w2 = bRb.InverseMass;
                    if (w1 + w2 == 0.0f) return;

                    Vec2 correction = n * (penetration / (w1 + w2));
                    
                    if (!cRb.IsStatic) cTrans.Position += correction * w1;
                    if (!bRb.IsStatic) bTrans.Position -= correction * w2;

                    if (!cRb.IsStatic) {
                        Vec2 v = cMove.Velocity;
                        Real vn = v.Dot(n);
                        
                        Real e = 0.5f; // Default
                        if (m_entityManager->HasComponent<PhysicalMaterialComponent>(circleID) && 
                            m_entityManager->HasComponent<PhysicalMaterialComponent>(boxID)) {
                            auto& matA = m_entityManager->GetComponent<PhysicalMaterialComponent>(circleID);
                            auto& matB = m_entityManager->GetComponent<PhysicalMaterialComponent>(boxID);
                            e = std::min(matA.Restitution, matB.Restitution);
                        }

                        if (penetration > 1.0f) e = 0.0f;

                        if (vn < 0) {
                            Real j = -(1 + e) * vn / (w1 + w2);
                            Vec2 impulse = n * j;
                            
                            // Friction
                            Real mu = 0.3f; // Default Dynamic Friction
                            if (m_entityManager->HasComponent<PhysicalMaterialComponent>(circleID) && 
                                m_entityManager->HasComponent<PhysicalMaterialComponent>(boxID)) {
                                auto& matA = m_entityManager->GetComponent<PhysicalMaterialComponent>(circleID);
                                auto& matB = m_entityManager->GetComponent<PhysicalMaterialComponent>(boxID);
                                mu = std::sqrt(matA.DynamicFriction * matB.DynamicFriction);
                            }
                            Vec2 rVec = n * -radius; 
                            Vec2 velAtContact = v;
                            velAtContact.x += -cMove.AngularVelocity * rVec.y;
                            velAtContact.y += cMove.AngularVelocity * rVec.x;

                            Vec2 tangent = velAtContact - n * velAtContact.Dot(n);
                            if (tangent.MagnitudeSqr() > 0.0001f) {
                                tangent = tangent.Normalized();
                                Real vt = velAtContact.Dot(tangent);
                                
                                // Coulomb friction impulse
                                Real frictionImpulseMag = -vt * mu * cRb.Mass * 0.5f; 
                                Vec2 frictionImpulse = tangent * frictionImpulseMag;
                                
                                cMove.Velocity += frictionImpulse * cRb.InverseMass;
                                Real torqueImpulse = rVec.x * frictionImpulse.y - rVec.y * frictionImpulse.x;
                                cMove.AngularVelocity += torqueImpulse * cRb.InverseInertia;
                            }
                            
                            cMove.Velocity += impulse * w1; 
                        }
                    }
                }
            };

            auto ResolveBoxBox = [&]() {
                Vec2 posA = transA.Position + colA.CenterOffset;
                Vec2 posB = transB.Position + colB.CenterOffset;
                Vec2 halfA = colA.Data.BoxHalfExtents;
                Vec2 halfB = colB.Data.BoxHalfExtents;
                
                Real rotA = transA.Rotation;
                Real rotB = transB.Rotation;

                Vec2 axes[4];
                axes[0] = Vec2(cos(rotA), sin(rotA));
                axes[1] = Vec2(-sin(rotA), cos(rotA));
                axes[2] = Vec2(cos(rotB), sin(rotB));
                axes[3] = Vec2(-sin(rotB), cos(rotB));

                Real minOverlap = 100000.0f;
                Vec2 mtvAxis;

                bool collision = true;
                for (int i = 0; i < 4; ++i) {
                    Vec2 axis = axes[i];
                    Real rA = halfA.x * std::abs(axis.Dot(axes[0])) + halfA.y * std::abs(axis.Dot(axes[1]));
                    Real rB = halfB.x * std::abs(axis.Dot(axes[2])) + halfB.y * std::abs(axis.Dot(axes[3]));
                    Real dist = std::abs((posB - posA).Dot(axis));
                    Real overlap = (rA + rB) - dist;

                    if (overlap <= 0) {
                        collision = false;
                        break;
                    }

                    if (overlap < minOverlap) {
                        minOverlap = overlap;
                        mtvAxis = axis;
                        if ((posB - posA).Dot(axis) < 0) mtvAxis = mtvAxis * -1.0f;
                    }
                }

                if (collision) {
                    Vec2 n = mtvAxis;
                    Real penetration = minOverlap;

                    Real w1 = rbA.InverseMass;
                    Real w2 = rbB.InverseMass;
                    if (w1 + w2 == 0.0f) return;

                    Vec2 correction = n * (penetration / (w1 + w2));
                    
                    if (!rbA.IsStatic) transA.Position -= correction * w1;
                    if (!rbB.IsStatic) transB.Position += correction * w2;

                    Vec2 relVel = moveA.Velocity - moveB.Velocity;
                    Real vn = relVel.Dot(n);
                    
                    if (vn < 0) {
                        Real e = 0.2f; // Default low restitution for boxes
                        if (m_entityManager->HasComponent<PhysicalMaterialComponent>(idA) && 
                            m_entityManager->HasComponent<PhysicalMaterialComponent>(idB)) {
                            auto& matA = m_entityManager->GetComponent<PhysicalMaterialComponent>(idA);
                            auto& matB = m_entityManager->GetComponent<PhysicalMaterialComponent>(idB);
                            e = std::min(matA.Restitution, matB.Restitution);
                        } 
                        Real j = -(1 + e) * vn / (w1 + w2);
                        Vec2 impulse = n * j;
                        
                        if (!rbA.IsStatic) moveA.Velocity += impulse * w1;
                        if (!rbB.IsStatic) moveB.Velocity -= impulse * w2;

                        // --- Friction Logic ---
                        Vec2 tangent = relVel - n * vn;
                        Real tangentLen = tangent.Magnitude();

                        if (tangentLen > 0.0001f) {
                            tangent = tangent / tangentLen;
                            Real vt = relVel.Dot(tangent);

                            // Get Friction Coefficients
                            Real staticFriction = 0.5f;
                            Real dynamicFriction = 0.3f;

                            if (m_entityManager->HasComponent<PhysicalMaterialComponent>(idA) && 
                                m_entityManager->HasComponent<PhysicalMaterialComponent>(idB)) {
                                auto& matA = m_entityManager->GetComponent<PhysicalMaterialComponent>(idA);
                                auto& matB = m_entityManager->GetComponent<PhysicalMaterialComponent>(idB);
                                
                                // Combine frictions (usually sqrt(a*b) or avg)
                                staticFriction = std::sqrt(matA.StaticFriction * matB.StaticFriction);
                                dynamicFriction = std::sqrt(matA.DynamicFriction * matB.DynamicFriction);
                            }

                            // Coulomb Friction
                            // j = normal impulse magnitude
                            Real jAbs = std::abs(j);
                            
                            Vec2 frictionImpulse;
                            if (std::abs(vt) < jAbs * staticFriction) {
                                // Static Friction (stop completely)
                                frictionImpulse = tangent * -vt / (w1 + w2);
                            } else {
                                // Dynamic Friction (slide with resistance)
                                frictionImpulse = tangent * -jAbs * dynamicFriction;
                            }

                            if (!rbA.IsStatic) moveA.Velocity += frictionImpulse * w1;
                            if (!rbB.IsStatic) moveB.Velocity -= frictionImpulse * w2;
                        }

                        // --- Projectile Impact Logic ---
                        // Use BounceFactor from component
                        bool isProjA = m_entityManager->HasComponent<ProjectileComponent>(idA);
                        bool isProjB = m_entityManager->HasComponent<ProjectileComponent>(idB);

                        if (isProjA) {
                            auto& projA = m_entityManager->GetComponent<ProjectileComponent>(idA);
                            moveA.Velocity = moveA.Velocity * projA.BounceFactor; 
                            moveA.AngularVelocity *= projA.BounceFactor;
                        }
                        if (isProjB) {
                            auto& projB = m_entityManager->GetComponent<ProjectileComponent>(idB);
                            moveB.Velocity = moveB.Velocity * projB.BounceFactor;
                            moveB.AngularVelocity *= projB.BounceFactor;
                        }
                    }
                }
            };

            // Nested Switch Dispatch
            switch (colA.Type) {
                case ColliderType::Circle:
                    switch (colB.Type) {
                        case ColliderType::Circle:
                            ResolveCircleCircle();
                            break;
                        case ColliderType::Box:
                            ResolveCircleBox(idA, idB);
                            break;
                    }
                    break;
                case ColliderType::Box:
                    switch (colB.Type) {
                        case ColliderType::Circle:
                            ResolveCircleBox(idB, idA); // Swap: Circle is first arg
                            break;
                        case ColliderType::Box:
                            ResolveBoxBox();
                            break;
                    }
                    break;
            }
        };

        // --- Spatial Grid Broadphase ---
        m_grid.Clear();

        // 1. Insert all entities into the grid
        for (auto id : entities) {
            if (!m_entityManager->HasComponent<ColliderComponent>(id) || 
                !m_entityManager->HasComponent<TransformComponent>(id)) continue;

            auto& col = m_entityManager->GetComponent<ColliderComponent>(id);
            auto& trans = m_entityManager->GetComponent<TransformComponent>(id);
            
            Vec2 min, max;
            Vec2 pos = trans.Position + col.CenterOffset;

            if (col.Type == ColliderType::Circle) {
                Real r = col.Data.Radius;
                min = pos - Vec2(r, r);
                max = pos + Vec2(r, r);
            } else {
                // AABB approximation for rotated box
                Real r = std::max(col.Data.BoxHalfExtents.x, col.Data.BoxHalfExtents.y) * 1.5f; // Conservative radius
                min = pos - Vec2(r, r);
                max = pos + Vec2(r, r);
            }
            m_grid.Insert(id, min, max);
        }

        // 2. Query and Resolve
        for (auto idA : entities) {
            if (!m_entityManager->HasComponent<ColliderComponent>(idA) || 
                !m_entityManager->HasComponent<TransformComponent>(idA)) continue;

            auto& col = m_entityManager->GetComponent<ColliderComponent>(idA);
            auto& trans = m_entityManager->GetComponent<TransformComponent>(idA);
            
            // Re-calculate bounds (could cache this)
            Vec2 min, max;
            Vec2 pos = trans.Position + col.CenterOffset;
            if (col.Type == ColliderType::Circle) {
                Real r = col.Data.Radius;
                min = pos - Vec2(r, r);
                max = pos + Vec2(r, r);
            } else {
                Real r = std::max(col.Data.BoxHalfExtents.x, col.Data.BoxHalfExtents.y) * 1.5f;
                min = pos - Vec2(r, r);
                max = pos + Vec2(r, r);
            }

            int startX = (int)min.x / SpatialGrid::CELL_SIZE;
            int endX = (int)max.x / SpatialGrid::CELL_SIZE;
            int startY = (int)min.y / SpatialGrid::CELL_SIZE;
            int endY = (int)max.y / SpatialGrid::CELL_SIZE;

            // Collect unique neighbors
            // Using a small vector and linear search might be faster than set for small counts, 
            // but let's use a sorted vector for robustness.
            std::vector<EntityID> neighbors; 
            neighbors.reserve(16);

            for (int x = startX; x <= endX; ++x) {
                for (int y = startY; y <= endY; ++y) {
                    int hash = m_grid.GetHash(x, y);
                    if (m_grid.cells.count(hash)) {
                        const auto& cell = m_grid.cells.at(hash);
                        neighbors.insert(neighbors.end(), cell.begin(), cell.end());
                    }
                }
            }

            std::sort(neighbors.begin(), neighbors.end());
            neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());

            for (auto idB : neighbors) {
                // Only resolve if A < B to avoid duplicates and self-collision
                if (idA < idB) {
                    ResolveCollision(idA, idB);
                }
            }
        }
    }
}
