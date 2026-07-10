/**
 * @file PhysicsSystem.cpp
 * @brief XPBD physics pipeline implementation for the Velox engine.
 *
 * Execution order each sub-step:
 *   ApplyRules → Integrate → SolveConstraints → DeriveVelocities → ResolveVelocities
 *
 * Collision dispatch matrix (narrowphase):
 *   Circle  vs {Circle, Box, Polygon, Chain}
 *   Box     vs {Circle, Box, Polygon, Chain}
 *   Polygon vs {Circle, Box, Polygon, Chain}
 *
 * Chain collision uses a half-space / support-point test rather
 * than SAT, because a zero-thickness segment has no reliable volume for SAT overlap.
 */
#include "PhysicsSystem.h"
#include "CCD.h"
#include <iostream>
#include <unordered_map>
#include <cmath>
#include <algorithm>

namespace Velox {

    // --- Sleep Constants ---
    constexpr Real SLEEP_LINEAR_THRESHOLD  = 4.0f;    // px/s
    constexpr Real SLEEP_ANGULAR_THRESHOLD = 0.08f;   // rad/s
    constexpr Real SLEEP_TIME_THRESHOLD    = 0.5f;    // seconds before sleeping

    /// Advances the full simulation by `dt`, snapshotting pre-integration velocities
    /// (needed for restitution) before running the sub-stepped XPBD pipeline.
    void PhysicsSystem::Step(Real dt) {
        ApplyRules(dt);

        // Snapshot velocity at the beginning of the frame (once)
        // so ResolveVelocities gets the actual approach velocity before integration and sub-stepping
        auto entities = m_entityManager->GetEntitiesWithComponent<RigidBodyComponent>();
        for (auto id : entities) {
            if (m_entityManager->HasComponent<MovementComponent>(id)) {
                auto& move = m_entityManager->GetComponent<MovementComponent>(id);
                move.PrevVelocity = move.Velocity;
                move.PrevAngularVelocity = move.AngularVelocity;
            }
        }

        // Run 8 sub-steps to improve solver stability and reduce tunnelling at high speeds.
        const int subSteps = 8;
        Real subDt = dt / (Real)subSteps;

        for (int s = 0; s < subSteps; ++s) {
            Integrate(subDt);
            SolveConstraints(subDt);
            SolveRevoluteJoints(subDt);
            SolvePrismaticJoints(subDt);
            SolveGearJoints(subDt);
            SolvePulleyJoints(subDt);
            SolveSoftBodies(subDt);
            DeriveVelocities(subDt);
            ResolveVelocities(subDt);
        }

        // Console logging is disabled by default to prevent rendering loop freezes from blocking I/O.
    }

    /// Drives non-collision gameplay behaviours each frame: force fields, rotation
    /// motors, oscillators, and projectile facing. Runs once per Step(), before sub-stepping.
    void PhysicsSystem::ApplyRules(Real dt) {
        // Apply Force Fields
        // 1. Gather all Force Fields
        std::vector<ForceFieldComponent> fields;
        std::vector<Vec2> fieldPositions; // For PointGravity

        auto fieldEntities = m_entityManager->GetEntitiesWithComponent<ForceFieldComponent>();
        for (auto id : fieldEntities) {
            fields.push_back(m_entityManager->GetComponent<ForceFieldComponent>(id));
            if (m_entityManager->HasComponent<TransformComponent>(id)) {
                fieldPositions.push_back(m_entityManager->GetComponent<TransformComponent>(id).Position);
            } else {
                fieldPositions.push_back({0,0});
            }
        }

        // 2. Apply RotationComponent (Motor)
        auto rotEntities = m_entityManager->GetEntitiesWithComponent<RotationComponent>();
        for (auto id : rotEntities) {
            if (m_entityManager->HasComponent<MovementComponent>(id)) {
                auto& rot = m_entityManager->GetComponent<RotationComponent>(id);
                auto& move = m_entityManager->GetComponent<MovementComponent>(id);
                
                Real dir = (rot.Direction == RotationDirection::Clockwise) ? 1.0f : -1.0f;
                move.AngularVelocity = rot.Speed * dir;
            }
        }

        // 3. Apply OscillationComponent
        auto oscEntities = m_entityManager->GetEntitiesWithComponent<OscillationComponent>();
        for (auto id : oscEntities) {
            if (m_entityManager->HasComponent<TransformComponent>(id)) {
                auto& osc = m_entityManager->GetComponent<OscillationComponent>(id);
                auto& trans = m_entityManager->GetComponent<TransformComponent>(id);
                
                osc.TimeAccumulator += dt; 
                
                Real offset = std::sin(osc.TimeAccumulator * osc.Frequency) * osc.Amplitude;
                trans.Position = osc.CenterPosition + osc.Axis * offset;
                
                // Update velocity so physics interactions work correctly
                if (m_entityManager->HasComponent<MovementComponent>(id)) {
                    auto& move = m_entityManager->GetComponent<MovementComponent>(id);
                    Real velMag = osc.Frequency * std::cos(osc.TimeAccumulator * osc.Frequency) * osc.Amplitude;
                    move.Velocity = osc.Axis * velMag;
                }
            }
        }

        // 4. Apply ProjectileComponent (Face Velocity)
        auto projEntities = m_entityManager->GetEntitiesWithComponent<ProjectileComponent>();
        for (auto id : projEntities) {
            if (m_entityManager->HasComponent<MovementComponent>(id) &&
                m_entityManager->HasComponent<TransformComponent>(id)) {
                
                auto& proj = m_entityManager->GetComponent<ProjectileComponent>(id);
                auto& move = m_entityManager->GetComponent<MovementComponent>(id);
                auto& trans = m_entityManager->GetComponent<TransformComponent>(id);
                
                if (proj.FaceVelocity) {
                    if (move.Velocity.MagnitudeSqr() > 0.001f) {
                        trans.Rotation = std::atan2(move.Velocity.y, move.Velocity.x);
                    }
                }
            }
        }

        // 5. Apply Force Fields to all Dynamic Bodies
        auto rbEntities = m_entityManager->GetEntitiesWithComponent<RigidBodyComponent>();
        for (auto id : rbEntities) {
            if (!m_entityManager->HasComponent<MovementComponent>(id) ||
                !m_entityManager->HasComponent<TransformComponent>(id)) continue;

            auto& rb = m_entityManager->GetComponent<RigidBodyComponent>(id);
            if (rb.IsStatic) continue;

            auto& move = m_entityManager->GetComponent<MovementComponent>(id);
            auto& trans = m_entityManager->GetComponent<TransformComponent>(id);

            for (size_t f = 0; f < fields.size(); ++f) {
                const auto& field = fields[f];
                
                Vec2 dir = fieldPositions[f] - trans.Position;
                Real distSqr = dir.MagnitudeSqr();
                Real r = field.Radius;
                
                if (distSqr < r * r && distSqr > 0.0001f) {
                    Real dist = std::sqrt(distSqr);
                    Vec2 normal = dir / dist; // Points TO field center
                    
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

    /// XPBD prediction step: applies damping and gravity, then integrates velocity
    /// and position forward by `dt` for every non-static dynamic body.
    void PhysicsSystem::Integrate(Real dt) {
        auto entities = m_entityManager->GetEntitiesWithComponent<RigidBodyComponent>();
        for (auto id : entities) {
            if (!m_entityManager->HasComponent<TransformComponent>(id) ||
                !m_entityManager->HasComponent<MovementComponent>(id)) continue;

            auto& rb = m_entityManager->GetComponent<RigidBodyComponent>(id);
            auto& transform = m_entityManager->GetComponent<TransformComponent>(id);
            auto& move = m_entityManager->GetComponent<MovementComponent>(id);

            if (rb.IsStatic || rb.IsSleeping) continue;

            // Store current state for XPBD velocity derivation (this is sub-step local)
            move.PrevPosition = transform.Position;
            move.PrevRotation = transform.Rotation;

            // Apply Gravity (Directional Gravity Vector)
            move.Force += m_gravity * rb.Mass;

            // XPBD Prediction (Euler step)
            Vec2 acceleration = move.Force * rb.InverseMass;
            move.Velocity += acceleration * dt;
            transform.Position += move.Velocity * dt;
            
            Real angularAccel = move.Torque * rb.InverseInertia;
            move.AngularVelocity += angularAccel * dt;
            transform.Rotation += move.AngularVelocity * dt;

            // Reset force accumulations
            move.Force = Vec2(0,0);
            move.Torque = 0.0f;
        }
    }

    // Forward declaration of polygon vertex helper for the lambdas
    static void GetPolygonWorldVertices(const TransformComponent& trans, const ColliderComponent& col, std::vector<Vec2>& outVertices);

    /// Broadphase (spatial hash) + narrowphase collision detection and positional
    /// correction, followed by distance-joint (and joint motor) solving. Populates
    /// m_contacts, which ResolveVelocities() later consumes for impulse response.
    void PhysicsSystem::SolveConstraints(Real dt) {
        m_contacts.clear();

        // 1. Gather all collider entities
        auto colliderEntities = m_entityManager->GetEntitiesWithComponent<ColliderComponent>();
        std::vector<EntityID> entities;
        entities.reserve(colliderEntities.size());

        for (auto id : colliderEntities) {
            if (m_entityManager->HasComponent<TransformComponent>(id) &&
                m_entityManager->HasComponent<RigidBodyComponent>(id)) {
                entities.push_back(id);
            }
        }

        // Narrowphase entry point: resolves one candidate pair by dispatching to the
        // shape-specific solver below based on each entity's ColliderType.
        auto ResolveCollision = [&](EntityID idA, EntityID idB) {
            auto& colA = m_entityManager->GetComponent<ColliderComponent>(idA);
            auto& colB = m_entityManager->GetComponent<ColliderComponent>(idB);

            // Skip collisions if both entities belong to the same collision group (e.g., same soft body)
            if (colA.GroupId != -1 && colA.GroupId == colB.GroupId) return;

            auto& transA = m_entityManager->GetComponent<TransformComponent>(idA);
            auto& rbA = m_entityManager->GetComponent<RigidBodyComponent>(idA);
            
            auto& transB = m_entityManager->GetComponent<TransformComponent>(idB);
            auto& rbB = m_entityManager->GetComponent<RigidBodyComponent>(idB);

            // Wake up bodies if one is active and the other is sleeping
            if (rbA.IsSleeping || rbB.IsSleeping) {
                rbA.IsSleeping = false;
                rbA.SleepTimer = 0.0f;
                rbB.IsSleeping = false;
                rbB.SleepTimer = 0.0f;
            }

            // Circle vs Circle: distance-based overlap test with proportional positional correction.
            auto ResolveCircleCircle = [&]() {
                Vec2 n = transA.Position - transB.Position;
                Real dist = n.Magnitude();
                Real radiusSum = colA.Data.Radius + colB.Data.Radius;

                if (dist < radiusSum && dist > 0.0001f) {
                    n = n / dist;
                    Real penetration = radiusSum - dist;
                    
                    if (colA.IsSensor || colB.IsSensor) {
                        m_contacts.push_back({idA, idB, n, penetration, {transB.Position + n * colB.Data.Radius}});
                        return;
                    }
                    
                    Real w1 = rbA.InverseMass;
                    Real w2 = rbB.InverseMass;
                    if (w1 + w2 == 0.0f) return;

                    Vec2 dx = n * (penetration / (w1 + w2));
                    
                    if (!rbA.IsStatic) transA.Position += dx * w1;
                    if (!rbB.IsStatic) transB.Position -= dx * w2;

                    m_contacts.push_back({idA, idB, n, penetration, {transB.Position + n * colB.Data.Radius}});
                }
            };

            // Circle vs Box: clamps the circle center into the box's local space to find
            // the closest surface point, then treats it as a circle-vs-point test.
            auto ResolveCircleBox = [&](EntityID circleID, EntityID boxID) {
                auto& cCol = m_entityManager->GetComponent<ColliderComponent>(circleID);
                auto& cTrans = m_entityManager->GetComponent<TransformComponent>(circleID);
                auto& cRb = m_entityManager->GetComponent<RigidBodyComponent>(circleID);

                auto& bCol = m_entityManager->GetComponent<ColliderComponent>(boxID);
                auto& bTrans = m_entityManager->GetComponent<TransformComponent>(boxID);
                auto& bRb = m_entityManager->GetComponent<RigidBodyComponent>(boxID);

                Vec2 circlePos = cTrans.Position + cCol.CenterOffset;
                Vec2 boxPos = bTrans.Position + bCol.CenterOffset;
                Vec2 boxHalf = bCol.Data.BoxHalfExtents;

                // Transform Circle to Box Local Space
                Vec2 relPos = circlePos - boxPos;
                Vec2 localPos = relPos.Rotate(-bTrans.Rotation);

                // Clamp in Local Space (AABB check)
                Vec2 clampedLocal = localPos;
                clampedLocal.x = std::max(-boxHalf.x, std::min(clampedLocal.x, boxHalf.x));
                clampedLocal.y = std::max(-boxHalf.y, std::min(clampedLocal.y, boxHalf.y));
                
                // Transform closest point back to World Space
                Vec2 closestWorld = boxPos + clampedLocal.Rotate(bTrans.Rotation);

                Vec2 n = circlePos - closestWorld;
                Real distSqr = n.MagnitudeSqr();
                Real radius = cCol.Data.Radius;

                if (distSqr < radius * radius && distSqr > 0.00001f) {
                    Real dist = std::sqrt(distSqr);
                    n = n / dist; 
                    Real penetration = radius - dist;

                    if (cCol.IsSensor || bCol.IsSensor) {
                        m_contacts.push_back({circleID, boxID, n, penetration, {closestWorld}});
                        return;
                    }

                    Real w1 = cRb.InverseMass;
                    Real w2 = bRb.InverseMass;
                    if (w1 + w2 == 0.0f) return;

                    Vec2 correction = n * (penetration / (w1 + w2));
                    
                    if (!cRb.IsStatic) cTrans.Position += correction * w1;
                    if (!bRb.IsStatic) bTrans.Position -= correction * w2;

                    m_contacts.push_back({circleID, boxID, n, penetration, {closestWorld}});
                }
            };

            // Box vs Box: SAT over each box's two face normals, with a corner-containment
            // pass to build a contact manifold once the minimum-translation axis is known.
            auto ResolveBoxBox = [&]() {
                Vec2 posA = transA.Position + colA.CenterOffset;
                Vec2 posB = transB.Position + colB.CenterOffset;
                Vec2 halfA = colA.Data.BoxHalfExtents;
                Vec2 halfB = colB.Data.BoxHalfExtents;
                
                Real rotA = transA.Rotation;
                Real rotB = transB.Rotation;

                Vec2 axes[4];
                axes[0] = Vec2(std::cos(rotA), std::sin(rotA));
                axes[1] = Vec2(-std::sin(rotA), std::cos(rotA));
                axes[2] = Vec2(std::cos(rotB), std::sin(rotB));
                axes[3] = Vec2(-std::sin(rotB), std::cos(rotB));

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

                    // Contact point generation
                    std::vector<Vec2> contactPoints;
                    
                    auto GetBoxCorners = [](const Vec2& center, const Vec2& half, Real rotation) {
                        std::vector<Vec2> corners(4);
                        Vec2 uX(std::cos(rotation), std::sin(rotation));
                        Vec2 uY(-std::sin(rotation), std::cos(rotation));
                        corners[0] = center + uX * half.x + uY * half.y;
                        corners[1] = center - uX * half.x + uY * half.y;
                        corners[2] = center - uX * half.x - uY * half.y;
                        corners[3] = center + uX * half.x - uY * half.y;
                        return corners;
                    };

                    auto IsPointInBox = [](const Vec2& pt, const Vec2& center, const Vec2& half, Real rotation) {
                        Vec2 rel = pt - center;
                        Vec2 local = rel.Rotate(-rotation);
                        return std::abs(local.x) <= half.x + 0.1f && std::abs(local.y) <= half.y + 0.1f;
                    };

                    auto cornersA = GetBoxCorners(posA, halfA, rotA);
                    auto cornersB = GetBoxCorners(posB, halfB, rotB);

                    for (const auto& pt : cornersA) {
                        if (IsPointInBox(pt, posB, halfB, rotB)) {
                            contactPoints.push_back(pt);
                        }
                    }
                    for (const auto& pt : cornersB) {
                        if (IsPointInBox(pt, posA, halfA, rotA)) {
                            contactPoints.push_back(pt);
                        }
                    }

                    if (contactPoints.empty()) {
                        contactPoints.push_back((posA + posB) * 0.5f);
                    }

                    if (colA.IsSensor || colB.IsSensor) {
                        m_contacts.push_back({idA, idB, n, penetration, contactPoints});
                        return;
                    }

                    Real w1 = rbA.InverseMass;
                    Real w2 = rbB.InverseMass;
                    if (w1 + w2 == 0.0f) return;

                    Vec2 correction = n * (penetration / (w1 + w2));
                    
                    if (!rbA.IsStatic) transA.Position -= correction * w1;
                    if (!rbB.IsStatic) transB.Position += correction * w2;

                    m_contacts.push_back({idA, idB, n, penetration, contactPoints});
                }
            };

            // Half-space / support-point solver: polygon vs single chain segment.
            // A segment has no volume, so SAT is unreliable - instead we test the signed
            // distance of every polygon vertex from the segment's half-plane.
            auto SATPolygonVsSegment = [&](EntityID polyID, Vec2 segA, Vec2 segB) {
                auto& pCol = m_entityManager->GetComponent<ColliderComponent>(polyID);
                auto& pTrans = m_entityManager->GetComponent<TransformComponent>(polyID);
                auto& pRb = m_entityManager->GetComponent<RigidBodyComponent>(polyID);
                auto& pMove = m_entityManager->GetComponent<MovementComponent>(polyID);

                if (pRb.IsStatic) return;

                std::vector<Vec2> polyVerts;
                GetPolygonWorldVertices(pTrans, pCol, polyVerts);
                if (polyVerts.empty()) return;

                // --- Step 1: Compute the outward segment normal ---
                // Screen-space: Y increases DOWNWARD.
                // For a chain floor going left->right, "above" = negative Y.
                // Right-hand perpendicular of direction = (dir.y, -dir.x) = points upward.
                Vec2 segEdge = segB - segA;
                Real segLen = segEdge.Magnitude();
                if (segLen < 0.0001f) return;
                Vec2 segDir = segEdge / segLen;
                Vec2 n = Vec2(segDir.y, -segDir.x); // outward normal, pointing "up" in screen space

                // --- Step 2: Find the support vertex (deepest penetrating vertex) ---
                // Signed distance of vertex v from the segment line:
                //   sep = n · (v - segA)
                // Negative sep means the vertex is on the "inside" (below the surface).
                Real deepestSep = 0.0f;        // only care about penetrations (negative values)
                Vec2 deepestVert = polyVerts[0];
                bool anyPenetrating = false;

                for (const auto& v : polyVerts) {
                    Real sep = n.Dot(v - segA);
                    if (sep < deepestSep) {
                        deepestSep = sep;
                        deepestVert = v;
                        anyPenetrating = true;
                    }
                }

                if (!anyPenetrating) return; // all vertices above the surface

                // --- Step 3: Check the contact point is within the segment's extent ---
                // Project the deepest vertex onto the segment to get the contact point.
                Vec2 relVert = deepestVert - segA;
                Real t = relVert.Dot(segDir);
                if (t < -5.0f || t > segLen + 5.0f) return; // outside this segment, skip

                // --- Step 4: Resolve ---
                Real penetration = -deepestSep; // positive depth

                // Positional correction: push body out along normal
                pTrans.Position += n * penetration;

                // Velocity correction: bounce/damp velocity into surface
                Real velAlongN = pMove.Velocity.Dot(n);
                if (velAlongN < 0.0f) {
                    Real restitution = 0.3f;
                    pMove.Velocity += n * (-(1.0f + restitution) * velAlongN);
                }

                Vec2 contactPt = segA + segDir * std::max(0.0f, std::min(t, segLen));
                m_contacts.push_back({polyID, polyID, n, penetration, {contactPt}});
            };

            // SAT solver helper for Convex Polygons
            auto ResolvePolygonPolygon = [&](EntityID idA, EntityID idB, bool isAChain = false, bool isBChain = false) {
                auto& colA_ref = m_entityManager->GetComponent<ColliderComponent>(idA);
                auto& colB_ref = m_entityManager->GetComponent<ColliderComponent>(idB);
                auto& transA_ref = m_entityManager->GetComponent<TransformComponent>(idA);
                auto& transB_ref = m_entityManager->GetComponent<TransformComponent>(idB);
                auto& rbA_ref = m_entityManager->GetComponent<RigidBodyComponent>(idA);
                auto& rbB_ref = m_entityManager->GetComponent<RigidBodyComponent>(idB);

                std::vector<Vec2> vertsA, vertsB;
                GetPolygonWorldVertices(transA_ref, colA_ref, vertsA);
                GetPolygonWorldVertices(transB_ref, colB_ref, vertsB);

                if (vertsA.empty() || vertsB.empty()) return;

                Real minOverlap = 100000.0f;
                Vec2 mtvAxis;

                // Build axis normals for SAT
                std::vector<Vec2> axes;
                if (!isAChain) {
                    for (size_t i = 0; i < vertsA.size(); ++i) {
                        Vec2 edge = vertsA[(i + 1) % vertsA.size()] - vertsA[i];
                        Vec2 norm = Vec2(-edge.y, edge.x).Normalized();
                        axes.push_back(norm);
                    }
                } else {
                    for (size_t i = 0; i < vertsA.size() - 1; ++i) {
                        Vec2 edge = vertsA[i + 1] - vertsA[i];
                        Vec2 norm = Vec2(-edge.y, edge.x).Normalized();
                        axes.push_back(norm);
                    }
                }
                if (!isBChain) {
                    for (size_t i = 0; i < vertsB.size(); ++i) {
                        Vec2 edge = vertsB[(i + 1) % vertsB.size()] - vertsB[i];
                        Vec2 norm = Vec2(-edge.y, edge.x).Normalized();
                        axes.push_back(norm);
                    }
                } else {
                    for (size_t i = 0; i < vertsB.size() - 1; ++i) {
                        Vec2 edge = vertsB[i + 1] - vertsB[i];
                        Vec2 norm = Vec2(-edge.y, edge.x).Normalized();
                        axes.push_back(norm);
                    }
                }

                for (const auto& axis : axes) {
                    // Project vertsA
                    Real minA = axis.Dot(vertsA[0]);
                    Real maxA = minA;
                    for (const auto& v : vertsA) {
                        Real p = axis.Dot(v);
                        if (p < minA) minA = p;
                        if (p > maxA) maxA = p;
                    }

                    // Project vertsB
                    Real minB = axis.Dot(vertsB[0]);
                    Real maxB = minB;
                    for (const auto& v : vertsB) {
                        Real p = axis.Dot(v);
                        if (p < minB) minB = p;
                        if (p > maxB) maxB = p;
                    }

                    Real overlap = std::min(maxA, maxB) - std::max(minA, minB);
                    if (overlap <= 0.0f) return; // Separating axis found

                    if (overlap < minOverlap) {
                        minOverlap = overlap;
                        mtvAxis = axis;
                    }
                }

                Vec2 centerA = transA_ref.Position;
                Vec2 centerB = transB_ref.Position;
                if (mtvAxis.Dot(centerA - centerB) < 0.0f) {
                    mtvAxis = mtvAxis * -1.0f;
                }

                Vec2 n = mtvAxis;
                Real penetration = minOverlap;

                if (colA_ref.IsSensor || colB_ref.IsSensor) {
                    m_contacts.push_back({idA, idB, n, penetration, {(centerA + centerB)*0.5f}});
                    return;
                }

                Real w1 = rbA_ref.InverseMass;
                Real w2 = rbB_ref.InverseMass;
                if (w1 + w2 == 0.0f) return;

                Vec2 correction = n * (penetration / (w1 + w2));
                if (!rbA_ref.IsStatic) transA_ref.Position += correction * w1;
                if (!rbB_ref.IsStatic) transB_ref.Position -= correction * w2;

                m_contacts.push_back({idA, idB, n, penetration, {(centerA + centerB)*0.5f}});
            };

            // Circle vs Polygon/Chain. Chains are tested as a sequence of open segments
            // (closest-point-on-segment); closed polygons use SAT with an extra axis
            // toward the circle's closest vertex to catch corner cases.
            auto ResolveCirclePolygon = [&](EntityID circleID, EntityID polyID, bool isChain = false) {
                auto& cCol = m_entityManager->GetComponent<ColliderComponent>(circleID);
                auto& cTrans = m_entityManager->GetComponent<TransformComponent>(circleID);
                auto& cRb = m_entityManager->GetComponent<RigidBodyComponent>(circleID);

                auto& pCol = m_entityManager->GetComponent<ColliderComponent>(polyID);
                auto& pTrans = m_entityManager->GetComponent<TransformComponent>(polyID);
                auto& pRb = m_entityManager->GetComponent<RigidBodyComponent>(polyID);

                std::vector<Vec2> verts;
                GetPolygonWorldVertices(pTrans, pCol, verts);
                if (verts.empty()) return;

                Vec2 circleCenter = cTrans.Position + cCol.CenterOffset;
                Real radius = cCol.Data.Radius;

                auto ResolveCircleSegment = [&](Vec2 segA, Vec2 segB) {
                    Vec2 edge = segB - segA;
                    Real edgeLength = edge.Magnitude();
                    if (edgeLength < 0.0001f) return;
                    Vec2 edgeDir = edge / edgeLength;

                    // Project circle center onto segment line
                    Vec2 rel = circleCenter - segA;
                    Real t = rel.Dot(edgeDir);
                    t = std::max(0.0f, std::min(t, edgeLength));

                    Vec2 closestPoint = segA + edgeDir * t;
                    Vec2 toCircle = circleCenter - closestPoint;
                    Real distSqr = toCircle.MagnitudeSqr();

                    if (distSqr < radius * radius && distSqr > 0.000001f) {
                        Real dist = std::sqrt(distSqr);
                        Vec2 n = toCircle / dist;
                        Real penetration = radius - dist;

                        if (cCol.IsSensor || pCol.IsSensor) {
                            m_contacts.push_back({circleID, polyID, n, penetration, {closestPoint}});
                            return;
                        }

                        Real w1 = cRb.InverseMass;
                        Real w2 = pRb.InverseMass;
                        if (w1 + w2 == 0.0f) return;

                        Vec2 correction = n * (penetration / (w1 + w2));
                        if (!cRb.IsStatic) cTrans.Position += correction * w1;
                        if (!pRb.IsStatic) pTrans.Position -= correction * w2;

                        m_contacts.push_back({circleID, polyID, n, penetration, {closestPoint}});
                    }
                };

                if (isChain) {
                    for (size_t i = 0; i + 1 < verts.size(); ++i) {
                        ResolveCircleSegment(verts[i], verts[i+1]);
                    }
                } else {
                    Real minOverlap = 100000.0f;
                    Vec2 mtvAxis;

                    // SAT Axes: Polygon edge normals + normal from circle center to closest vertex
                    std::vector<Vec2> axes;
                    for (size_t i = 0; i < verts.size(); ++i) {
                        Vec2 edge = verts[(i + 1) % verts.size()] - verts[i];
                        Vec2 norm = Vec2(-edge.y, edge.x).Normalized();
                        axes.push_back(norm);
                    }

                    // Normal to closest vertex
                    float minV = 1e9f;
                    Vec2 closestVert;
                    for (const auto& v : verts) {
                        float dist = (v - circleCenter).MagnitudeSqr();
                        if (dist < minV) {
                            minV = dist;
                            closestVert = v;
                        }
                    }
                    axes.push_back((circleCenter - closestVert).Normalized());

                    for (const auto& axis : axes) {
                        // Project circle
                        Real cCenter = axis.Dot(circleCenter);
                        Real minC = cCenter - radius;
                        Real maxC = cCenter + radius;

                        // Project polygon
                        Real minP = axis.Dot(verts[0]);
                        Real maxP = minP;
                        for (const auto& v : verts) {
                            Real p = axis.Dot(v);
                            if (p < minP) minP = p;
                            if (p > maxP) maxP = p;
                        }

                        Real overlap = std::min(maxC, maxP) - std::max(minC, minP);
                        if (overlap <= 0.0f) return;

                        if (overlap < minOverlap) {
                            minOverlap = overlap;
                            mtvAxis = axis;
                        }
                    }

                    if (mtvAxis.Dot(circleCenter - pTrans.Position) < 0.0f) {
                        mtvAxis = mtvAxis * -1.0f;
                    }

                    Vec2 n = mtvAxis;
                    Real penetration = minOverlap;

                    if (cCol.IsSensor || pCol.IsSensor) {
                        m_contacts.push_back({circleID, polyID, n, penetration, {circleCenter - n * radius}});
                        return;
                    }

                    Real w1 = cRb.InverseMass;
                    Real w2 = pRb.InverseMass;
                    if (w1 + w2 == 0.0f) return;

                    Vec2 correction = n * (penetration / (w1 + w2));
                    if (!cRb.IsStatic) cTrans.Position += correction * w1;
                    if (!pRb.IsStatic) pTrans.Position -= correction * w2;

                    m_contacts.push_back({circleID, polyID, n, penetration, {circleCenter - n * radius}});
                }
            };

            switch (colA.Type) {
                case ColliderType::Circle:
                    switch (colB.Type) {
                        case ColliderType::Circle:
                            ResolveCircleCircle();
                            break;
                        case ColliderType::Box:
                            ResolveCircleBox(idA, idB);
                            break;
                        case ColliderType::Polygon:
                            ResolveCirclePolygon(idA, idB, false);
                            break;
                        case ColliderType::Chain:
                            ResolveCirclePolygon(idA, idB, true);
                            break;
                    }
                    break;
                case ColliderType::Box:
                    switch (colB.Type) {
                        case ColliderType::Circle:
                            ResolveCircleBox(idB, idA);
                            break;
                        case ColliderType::Box:
                            ResolveBoxBox();
                            break;
                        case ColliderType::Polygon:
                            ResolvePolygonPolygon(idA, idB, false, false);
                            break;
                        case ColliderType::Chain:
                            // Box vs Chain: test each chain segment separately
                            {
                                auto& chainCol = m_entityManager->GetComponent<ColliderComponent>(idB);
                                for (size_t si = 0; si + 1 < chainCol.Vertices.size(); ++si) {
                                    SATPolygonVsSegment(idA, chainCol.Vertices[si], chainCol.Vertices[si+1]);
                                }
                            }
                            break;
                    }
                    break;
                case ColliderType::Polygon:
                    switch (colB.Type) {
                        case ColliderType::Circle:
                            ResolveCirclePolygon(idB, idA, false);
                            break;
                        case ColliderType::Box:
                            ResolvePolygonPolygon(idB, idA, false, false);
                            break;
                        case ColliderType::Polygon:
                            ResolvePolygonPolygon(idA, idB, false, false);
                            break;
                        case ColliderType::Chain:
                            // Polygon vs Chain: test each chain segment separately
                            {
                                auto& chainCol = m_entityManager->GetComponent<ColliderComponent>(idB);
                                for (size_t si = 0; si + 1 < chainCol.Vertices.size(); ++si) {
                                    // Vertices are already in world space (chain transform at origin)
                                    SATPolygonVsSegment(idA, chainCol.Vertices[si], chainCol.Vertices[si+1]);
                                }
                            }
                            break;
                    }
                    break;
                case ColliderType::Chain:
                    switch (colB.Type) {
                        case ColliderType::Circle:
                            ResolveCirclePolygon(idB, idA, true);
                            break;
                        case ColliderType::Box:
                            // Chain vs Box: test each chain segment separately
                            {
                                auto& chainCol = m_entityManager->GetComponent<ColliderComponent>(idA);
                                for (size_t si = 0; si + 1 < chainCol.Vertices.size(); ++si) {
                                    SATPolygonVsSegment(idB, chainCol.Vertices[si], chainCol.Vertices[si+1]);
                                }
                            }
                            break;
                        case ColliderType::Polygon:
                            // Chain vs Polygon: test each chain segment separately
                            {
                                auto& chainCol = m_entityManager->GetComponent<ColliderComponent>(idA);
                                for (size_t si = 0; si + 1 < chainCol.Vertices.size(); ++si) {
                                    SATPolygonVsSegment(idB, chainCol.Vertices[si], chainCol.Vertices[si+1]);
                                }
                            }
                            break;
                        case ColliderType::Chain:
                            ResolvePolygonPolygon(idA, idB, true, true);
                            break;
                    }
                    break;
            }
        };

        // --- Spatial Grid Broadphase ---
        struct CachedAABB {
            EntityID id;
            Vec2 min;
            Vec2 max;
        };
        std::vector<CachedAABB> cachedAABBs;
        cachedAABBs.reserve(entities.size());

        for (auto id : entities) {
            auto& col = m_entityManager->GetComponent<ColliderComponent>(id);
            auto& trans = m_entityManager->GetComponent<TransformComponent>(id);
            
            Vec2 min, max;
            Vec2 pos = trans.Position + col.CenterOffset;

            if (col.Type == ColliderType::Circle) {
                Real r = col.Data.Radius;
                min = pos - Vec2(r, r);
                max = pos + Vec2(r, r);
            } else if (col.Type == ColliderType::Box) {
                Real r = std::max(col.Data.BoxHalfExtents.x, col.Data.BoxHalfExtents.y) * 1.5f;
                min = pos - Vec2(r, r);
                max = pos + Vec2(r, r);
            } else if (col.Type == ColliderType::Chain) {
                // Use actual vertex extents for chains so broadphase covers the whole floor
                std::vector<Vec2> wv;
                GetPolygonWorldVertices(trans, col, wv);
                if (!wv.empty()) {
                    min = max = wv[0];
                    for (const auto& v : wv) {
                        min.x = std::min(min.x, v.x);
                        min.y = std::min(min.y, v.y);
                        max.x = std::max(max.x, v.x);
                        max.y = std::max(max.y, v.y);
                    }
                } else {
                    min = pos - Vec2(500.0f, 200.0f);
                    max = pos + Vec2(500.0f, 200.0f);
                }
            } else { // Polygon dynamic sizing
                std::vector<Vec2> wv;
                GetPolygonWorldVertices(trans, col, wv);
                if (!wv.empty()) {
                    min = max = wv[0];
                    for (const auto& v : wv) {
                        min.x = std::min(min.x, v.x);
                        min.y = std::min(min.y, v.y);
                        max.x = std::max(max.x, v.x);
                        max.y = std::max(max.y, v.y);
                    }
                    // Pad slightly
                    Vec2 pad(5.0f, 5.0f);
                    min = min - pad;
                    max = max + pad;
                } else {
                    min = pos - Vec2(50.0f, 50.0f);
                    max = pos + Vec2(50.0f, 50.0f);
                }
            }
            cachedAABBs.push_back({id, min, max});
        }

        m_grid.Clear();

        for (const auto& cache : cachedAABBs) {
            m_grid.Insert(cache.id, cache.min, cache.max);
        }

        // Query neighbors and resolve overlaps using FlatGrid and a flat candidate pairs list
        m_candidatePairs.clear();

        for (const auto& cache : cachedAABBs) {
            EntityID idA = cache.id;
            bool hasRb = m_entityManager->HasComponent<RigidBodyComponent>(idA);
            if (hasRb && m_entityManager->GetComponent<RigidBodyComponent>(idA).IsSleeping) continue;

            Vec2 min = cache.min;
            Vec2 max = cache.max;

            int startX = std::max(0, (int)std::floor(min.x / FlatGrid::CELL_SIZE));
            int endX   = std::min(FlatGrid::GRID_W - 1, (int)std::floor(max.x / FlatGrid::CELL_SIZE));
            int startY = std::max(0, (int)std::floor(min.y / FlatGrid::CELL_SIZE));
            int endY   = std::min(FlatGrid::GRID_H - 1, (int)std::floor(max.y / FlatGrid::CELL_SIZE));

            for (int x = startX; x <= endX; ++x) {
                for (int y = startY; y <= endY; ++y) {
                    int idx = y * FlatGrid::GRID_W + x;
                    int count = m_grid.counts[idx];
                    for (int i = 0; i < count; ++i) {
                        EntityID idB = m_grid.cells[idx][i];
                        if (idA < idB) {
                            m_candidatePairs.push_back({idA, idB});
                        }
                    }
                }
            }
        }

        // Deduplicate pairs
        std::sort(m_candidatePairs.begin(), m_candidatePairs.end());
        m_candidatePairs.erase(std::unique(m_candidatePairs.begin(), m_candidatePairs.end()), m_candidatePairs.end());

        auto CheckCCD = [&](EntityID idA, EntityID idB, Real stepDt) -> TOIResult {
            auto& colA = m_entityManager->GetComponent<ColliderComponent>(idA);
            auto& transA = m_entityManager->GetComponent<TransformComponent>(idA);
            bool hasMoveA = m_entityManager->HasComponent<MovementComponent>(idA);
            Vec2 velA = hasMoveA ? m_entityManager->GetComponent<MovementComponent>(idA).Velocity : Vec2(0.0f, 0.0f);
            Vec2 prevPosA = hasMoveA ? m_entityManager->GetComponent<MovementComponent>(idA).PrevPosition : transA.Position;

            auto& colB = m_entityManager->GetComponent<ColliderComponent>(idB);
            auto& transB = m_entityManager->GetComponent<TransformComponent>(idB);
            bool hasMoveB = m_entityManager->HasComponent<MovementComponent>(idB);
            Vec2 velB = hasMoveB ? m_entityManager->GetComponent<MovementComponent>(idB).Velocity : Vec2(0.0f, 0.0f);
            Vec2 prevPosB = hasMoveB ? m_entityManager->GetComponent<MovementComponent>(idB).PrevPosition : transB.Position;

            // Only run CCD if relative velocity is significant (e.g. greater than 200 px/s)
            Vec2 relVel = velA - velB;
            if (relVel.MagnitudeSqr() < 40000.0f) {
                return { false, 0.0f };
            }

            if (colA.Type == ColliderType::Circle && colB.Type == ColliderType::Circle) {
                return SweptCircleCircle(prevPosA + colA.CenterOffset, velA, colA.Data.Radius,
                                         prevPosB + colB.CenterOffset, velB, colB.Data.Radius,
                                         stepDt);
            } else {
                auto GetAABBBounds = [&](EntityID id, Vec2& min, Vec2& max) {
                    auto& col = m_entityManager->GetComponent<ColliderComponent>(id);
                    auto& trans = m_entityManager->GetComponent<TransformComponent>(id);
                    
                    bool hasMove = m_entityManager->HasComponent<MovementComponent>(id);
                    Vec2 referencePos = hasMove ? m_entityManager->GetComponent<MovementComponent>(id).PrevPosition : trans.Position;
                    
                    // Construct start-of-step AABB bounds using referencePos
                    if (col.Type == ColliderType::Circle) {
                        min = referencePos + col.CenterOffset - Vec2(col.Data.Radius, col.Data.Radius);
                        max = referencePos + col.CenterOffset + Vec2(col.Data.Radius, col.Data.Radius);
                    } else {
                        // For polygon/box colliders
                        std::vector<Vec2> verts;
                        // Temp transform representing the start-of-step state
                        TransformComponent startTrans = trans;
                        startTrans.Position = referencePos;
                        GetPolygonWorldVertices(startTrans, col, verts);
                        if (verts.empty()) {
                            min = referencePos;
                            max = referencePos;
                            return;
                        }
                        min = verts[0];
                        max = verts[0];
                        for (const auto& v : verts) {
                            min.x = std::min(min.x, v.x);
                            min.y = std::min(min.y, v.y);
                            max.x = std::max(max.x, v.x);
                            max.y = std::max(max.y, v.y);
                        }
                    }
                };
                
                Vec2 minA, maxA, minB, maxB;
                GetAABBBounds(idA, minA, maxA);
                GetAABBBounds(idB, minB, maxB);
                
                return SweptAABB(minA, maxA, velA, minB, maxB, velB, stepDt);
            }
        };

        auto AdvanceToCCD = [&](EntityID idA, EntityID idB, Real t) {
            auto& rbA = m_entityManager->GetComponent<RigidBodyComponent>(idA);
            auto& transA = m_entityManager->GetComponent<TransformComponent>(idA);
            auto& moveA = m_entityManager->GetComponent<MovementComponent>(idA);

            auto& rbB = m_entityManager->GetComponent<RigidBodyComponent>(idB);
            auto& transB = m_entityManager->GetComponent<TransformComponent>(idB);
            auto& moveB = m_entityManager->GetComponent<MovementComponent>(idB);

            // Land exactly at the predicted time-of-impact, then nudge a hair further so
            // the narrowphase (which only registers a contact on strict overlap, i.e.
            // dist < radiusSum) actually sees genuine contact THIS substep. The old 0.95
            // backoff deliberately stopped the body short of contact, so dist stayed
            // >= radiusSum forever and no restitution ever fired -- fast-approaching
            // pairs just crept toward each other in shrinking steps every substep,
            // asymptotically stalling instead of bouncing.
            const Real epsilon = 0.02f; // world units of guaranteed penetration

            if (!rbA.IsStatic) {
                Vec2 velA = moveA.Velocity;
                Real speedA = velA.Magnitude();
                Vec2 nudgeA = speedA > 0.0001f ? (velA / speedA) * epsilon : Vec2(0.0f, 0.0f);
                transA.Position = moveA.PrevPosition + velA * t + nudgeA;
            }
            if (!rbB.IsStatic) {
                Vec2 velB = moveB.Velocity;
                Real speedB = velB.Magnitude();
                Vec2 nudgeB = speedB > 0.0001f ? (velB / speedB) * epsilon : Vec2(0.0f, 0.0f);
                transB.Position = moveB.PrevPosition + velB * t + nudgeB;
            }
        };

        // Resolve overlaps
        for (const auto& pair : m_candidatePairs) {
            TOIResult toi = CheckCCD(pair.first, pair.second, dt);
            if (toi.hit && toi.toi > dt / 4.0f) {
                AdvanceToCCD(pair.first, pair.second, toi.toi);
            }
            ResolveCollision(pair.first, pair.second);
        }

        // --- Solve Distance/Joint Constraints ---
        auto jointEntities = m_entityManager->GetEntitiesWithComponent<JointComponent>();
        for (auto id : jointEntities) {
            auto& joint = m_entityManager->GetComponent<JointComponent>(id);
            if (!joint.IsActive) continue;

            EntityID idA = joint.EntityA;
            EntityID idB = joint.EntityB;

            if (!m_entityManager->HasComponent<TransformComponent>(idA) ||
                !m_entityManager->HasComponent<TransformComponent>(idB)) continue;

            auto& transA = m_entityManager->GetComponent<TransformComponent>(idA);
            auto& transB = m_entityManager->GetComponent<TransformComponent>(idB);
            
            auto& rbA = m_entityManager->GetComponent<RigidBodyComponent>(idA);
            auto& rbB = m_entityManager->GetComponent<RigidBodyComponent>(idB);

            // Anchors in world space
            Vec2 rA = joint.LocalAnchorA.Rotate(transA.Rotation);
            Vec2 rB = joint.LocalAnchorB.Rotate(transB.Rotation);
            
            Vec2 pA = transA.Position + rA;
            Vec2 pB = transB.Position + rB;

            Vec2 dir = pA - pB;
            Real dist = dir.Magnitude();
            if (dist < 0.0001f) continue;

            Vec2 n = dir / dist;
            Real C = dist - joint.TargetDistance;

            Real w1 = rbA.InverseMass;
            Real w2 = rbB.InverseMass;
            
            Real rAxn = rA.x * n.y - rA.y * n.x;
            Real rBxn = rB.x * n.y - rB.y * n.x;
            Real w1_rot = rbA.InverseInertia * rAxn * rAxn;
            Real w2_rot = rbB.InverseInertia * rBxn * rBxn;

            Real denominator = w1 + w2 + w1_rot + w2_rot + (joint.Compliance / (dt * dt));
            if (denominator < 0.0001f) continue;

            Real lambda = -C / denominator;

            if (!rbA.IsStatic) {
                transA.Position += n * (lambda * w1);
                transA.Rotation += lambda * rbA.InverseInertia * rAxn;
            }
            if (!rbB.IsStatic) {
                transB.Position -= n * (lambda * w2);
                transB.Rotation -= lambda * rbB.InverseInertia * rBxn;
            }

            // --- Joint Motor Solver (Angular Constraint) ---
            if (joint.EnableMotor) {
                // Rel angular velocity target
                Real wLinkA = rbA.InverseInertia;
                Real wLinkB = rbB.InverseInertia;
                Real sumInvI = wLinkA + wLinkB;
                if (sumInvI > 0.0001f) {
                    // Angular position difference over step
                    Real currentAngVel = (transB.Rotation - transA.Rotation) / dt;
                    Real targetAngVel = joint.MotorSpeed;
                    Real error = targetAngVel - currentAngVel;

                    Real impulse = error / sumInvI;
                    // Clamp to max motor torque
                    Real maxImp = joint.MaxMotorTorque * dt;
                    impulse = std::max(-maxImp, std::min(impulse, maxImp));

                    if (!rbA.IsStatic) transA.Rotation -= impulse * wLinkA * dt;
                    if (!rbB.IsStatic) transB.Rotation += impulse * wLinkB * dt;
                }
            }
        }
    }

    void PhysicsSystem::SolveRevoluteJoints(Real dt) {
        auto entities = m_entityManager->GetEntitiesWithComponent<RevoluteJointComponent>();
        for (auto id : entities) {
            auto& joint = m_entityManager->GetComponent<RevoluteJointComponent>(id);
            if (!joint.IsActive) continue;

            EntityID idA = joint.EntityA;
            EntityID idB = joint.EntityB;

            if (!m_entityManager->HasComponent<TransformComponent>(idA) ||
                !m_entityManager->HasComponent<TransformComponent>(idB)) continue;

            auto& transA = m_entityManager->GetComponent<TransformComponent>(idA);
            auto& transB = m_entityManager->GetComponent<TransformComponent>(idB);

            auto& rbA = m_entityManager->GetComponent<RigidBodyComponent>(idA);
            auto& rbB = m_entityManager->GetComponent<RigidBodyComponent>(idB);

            // Anchors in world space
            Vec2 rA = joint.LocalAnchorA.Rotate(transA.Rotation);
            Vec2 rB = joint.LocalAnchorB.Rotate(transB.Rotation);

            Vec2 pA = transA.Position + rA;
            Vec2 pB = transB.Position + rB;

            // Positional error (2D vector constraint)
            Vec2 C = pA - pB;
            Real dist = C.Magnitude();
            if (dist > 0.0f) {
                Vec2 n = C.Normalized();

                Real w1 = rbA.InverseMass;
                Real w2 = rbB.InverseMass;

                Real rAxn = rA.x * n.y - rA.y * n.x;
                Real rBxn = rB.x * n.y - rB.y * n.x;
                Real w1_rot = rbA.InverseInertia * rAxn * rAxn;
                Real w2_rot = rbB.InverseInertia * rBxn * rBxn;

                Real denominator = w1 + w2 + w1_rot + w2_rot + (joint.Compliance / (dt * dt));
                if (denominator > 0.0001f) {
                    Real lambda = -dist / denominator;
                    if (!rbA.IsStatic) {
                        transA.Position += n * (lambda * w1);
                        transA.Rotation += lambda * rbA.InverseInertia * rAxn;
                    }
                    if (!rbB.IsStatic) {
                        transB.Position -= n * (lambda * w2);
                        transB.Rotation -= lambda * rbB.InverseInertia * rBxn;
                    }
                }
            }

            // Angular limits
            if (joint.LimitsEnabled) {
                Real diffAngle = transB.Rotation - transA.Rotation;
                while (diffAngle < -3.14159265f) diffAngle += 2.0f * 3.14159265f;
                while (diffAngle >  3.14159265f) diffAngle -= 2.0f * 3.14159265f;

                Real targetAngle = std::max(joint.LowerAngle, std::min(diffAngle, joint.UpperAngle));
                Real C_angle = diffAngle - targetAngle;

                if (std::abs(C_angle) > 0.001f) {
                    Real w1_rot = rbA.InverseInertia;
                    Real w2_rot = rbB.InverseInertia;
                    Real denominator = w1_rot + w2_rot;
                    if (denominator > 0.0001f) {
                        Real lambda = -C_angle / denominator;
                        if (!rbA.IsStatic) transA.Rotation -= lambda * w1_rot;
                        if (!rbB.IsStatic) transB.Rotation += lambda * w2_rot;
                    }
                }
            }

            // Angular Motor
            if (joint.EnableMotor) {
                Real wLinkA = rbA.InverseInertia;
                Real wLinkB = rbB.InverseInertia;
                Real sumInvI = wLinkA + wLinkB;
                if (sumInvI > 0.0001f) {
                    Real w1_ang = 0.0f;
                    Real w2_ang = 0.0f;
                    if (m_entityManager->HasComponent<MovementComponent>(idA)) {
                        w1_ang = m_entityManager->GetComponent<MovementComponent>(idA).AngularVelocity;
                    }
                    if (m_entityManager->HasComponent<MovementComponent>(idB)) {
                        w2_ang = m_entityManager->GetComponent<MovementComponent>(idB).AngularVelocity;
                    }
                    Real currentAngVel = w2_ang - w1_ang;
                    Real targetAngVel = joint.MotorSpeed;
                    Real error = targetAngVel - currentAngVel;

                    Real impulse = error / sumInvI;
                    Real maxImp = joint.MaxMotorTorque * dt;
                    impulse = std::max(-maxImp, std::min(impulse, maxImp));

                    if (!rbA.IsStatic) transA.Rotation -= impulse * wLinkA;
                    if (!rbB.IsStatic) transB.Rotation += impulse * wLinkB;
                }
            }
        }
    }

    void PhysicsSystem::SolvePrismaticJoints(Real dt) {
        auto entities = m_entityManager->GetEntitiesWithComponent<PrismaticJointComponent>();
        for (auto id : entities) {
            auto& joint = m_entityManager->GetComponent<PrismaticJointComponent>(id);
            if (!joint.IsActive) continue;

            EntityID idA = joint.EntityA;
            EntityID idB = joint.EntityB;

            if (!m_entityManager->HasComponent<TransformComponent>(idA) ||
                !m_entityManager->HasComponent<TransformComponent>(idB)) continue;

            auto& transA = m_entityManager->GetComponent<TransformComponent>(idA);
            auto& transB = m_entityManager->GetComponent<TransformComponent>(idB);

            auto& rbA = m_entityManager->GetComponent<RigidBodyComponent>(idA);
            auto& rbB = m_entityManager->GetComponent<RigidBodyComponent>(idB);

            // Anchor points in world space
            Vec2 rA = joint.LocalAnchorA.Rotate(transA.Rotation);
            Vec2 rB = joint.LocalAnchorB.Rotate(transB.Rotation);

            Vec2 pA = transA.Position + rA;
            Vec2 pB = transB.Position + rB;

            Vec2 axisA = joint.LocalAxisA.Rotate(transA.Rotation).Normalized();
            Vec2 perpA = Vec2(-axisA.y, axisA.x);

            Vec2 d = pB - pA;

            // 1. Lateral point-on-line constraint
            Real C_perp = d.Dot(perpA);
            Real w1 = rbA.InverseMass;
            Real w2 = rbB.InverseMass;

            Real rAxP = rA.x * perpA.y - rA.y * perpA.x;
            Real rBxP = rB.x * perpA.y - rB.y * perpA.x;

            Real w1_rot = rbA.InverseInertia * rAxP * rAxP;
            Real w2_rot = rbB.InverseInertia * rBxP * rBxP;

            Real denom_perp = w1 + w2 + w1_rot + w2_rot + (joint.Compliance / (dt * dt));
            if (denom_perp > 0.0001f) {
                Real lambda_perp = -C_perp / denom_perp;
                if (!rbA.IsStatic) {
                    transA.Position -= perpA * (lambda_perp * w1);
                    transA.Rotation -= lambda_perp * rbA.InverseInertia * rAxP;
                }
                if (!rbB.IsStatic) {
                    transB.Position += perpA * (lambda_perp * w2);
                    transB.Rotation += lambda_perp * rbB.InverseInertia * rBxP;
                }
            }

            // Refresh anchor positions after lateral correction
            rA = joint.LocalAnchorA.Rotate(transA.Rotation);
            rB = joint.LocalAnchorB.Rotate(transB.Rotation);
            pA = transA.Position + rA;
            pB = transB.Position + rB;
            d = pB - pA;

            // 2. Angle lock constraint
            Real C_angle = transB.Rotation - transA.Rotation;
            Real w1_rot_ang = rbA.InverseInertia;
            Real w2_rot_ang = rbB.InverseInertia;
            Real denom_ang = w1_rot_ang + w2_rot_ang;
            if (denom_ang > 0.0001f) {
                Real lambda_ang = -C_angle / denom_ang;
                if (!rbA.IsStatic) transA.Rotation -= lambda_ang * w1_rot_ang;
                if (!rbB.IsStatic) transB.Rotation += lambda_ang * w2_rot_ang;
            }

            // 3. Translation limits along sliding axis
            if (joint.LimitsEnabled) {
                Real translation = d.Dot(axisA);
                Real C_limit = 0.0f;
                if (translation < joint.MinTranslation) {
                    C_limit = translation - joint.MinTranslation;
                } else if (translation > joint.MaxTranslation) {
                    C_limit = translation - joint.MaxTranslation;
                }

                if (std::abs(C_limit) > 0.001f) {
                    Real rAxL = rA.x * axisA.y - rA.y * axisA.x;
                    Real rBxL = rB.x * axisA.y - rB.y * axisA.x;
                    Real w1_rot_L = rbA.InverseInertia * rAxL * rAxL;
                    Real w2_rot_L = rbB.InverseInertia * rBxL * rBxL;
                    Real denom_limit = w1 + w2 + w1_rot_L + w2_rot_L;
                    if (denom_limit > 0.0001f) {
                        Real lambda_limit = -C_limit / denom_limit;
                        if (!rbA.IsStatic) {
                            transA.Position -= axisA * (lambda_limit * w1);
                            transA.Rotation -= lambda_limit * rbA.InverseInertia * rAxL;
                        }
                        if (!rbB.IsStatic) {
                            transB.Position += axisA * (lambda_limit * w2);
                            transB.Rotation += lambda_limit * rbB.InverseInertia * rBxL;
                        }
                    }
                }
            }

            // 4. Linear motor along sliding axis
            if (joint.EnableMotor) {
                Real sumM = w1 + w2;
                if (sumM > 0.0001f) {
                    Vec2 v1 = {0, 0};
                    Vec2 v2 = {0, 0};
                    if (m_entityManager->HasComponent<MovementComponent>(idA)) {
                        v1 = m_entityManager->GetComponent<MovementComponent>(idA).Velocity;
                    }
                    if (m_entityManager->HasComponent<MovementComponent>(idB)) {
                        v2 = m_entityManager->GetComponent<MovementComponent>(idB).Velocity;
                    }
                    Real currentSpeed = (v2 - v1).Dot(axisA);
                    Real targetSpeed = joint.MotorSpeed;
                    Real error = targetSpeed - currentSpeed;

                    Real impulse = error / sumM;
                    Real maxImp = joint.MaxMotorForce * dt;
                    impulse = std::max(-maxImp, std::min(impulse, maxImp));

                    if (!rbA.IsStatic) transA.Position -= axisA * (impulse * w1 * dt);
                    if (!rbB.IsStatic) transB.Position += axisA * (impulse * w2 * dt);
                }
            }
        }
    }

    void PhysicsSystem::SolveGearJoints(Real dt) {
        auto entities = m_entityManager->GetEntitiesWithComponent<GearJointComponent>();
        for (auto id : entities) {
            auto& joint = m_entityManager->GetComponent<GearJointComponent>(id);
            if (!joint.IsActive) continue;

            EntityID idA = joint.EntityA;
            EntityID idB = joint.EntityB;

            if (!m_entityManager->HasComponent<TransformComponent>(idA) ||
                !m_entityManager->HasComponent<TransformComponent>(idB)) continue;

            auto& transA = m_entityManager->GetComponent<TransformComponent>(idA);
            auto& transB = m_entityManager->GetComponent<TransformComponent>(idB);

            auto& rbA = m_entityManager->GetComponent<RigidBodyComponent>(idA);
            auto& rbB = m_entityManager->GetComponent<RigidBodyComponent>(idB);

            // C_gear = thetaA + ratio * thetaB
            Real thetaA = transA.Rotation;
            Real thetaB = transB.Rotation;
            Real ratio = joint.GearRatio;

            Real C = thetaA + ratio * thetaB;

            Real w1_rot = rbA.InverseInertia;
            Real w2_rot = rbB.InverseInertia;

            Real denominator = w1_rot + ratio * ratio * w2_rot + (joint.Compliance / (dt * dt));
            if (denominator > 0.0001f) {
                Real lambda = -C / denominator;
                if (!rbA.IsStatic) transA.Rotation += lambda * w1_rot;
                if (!rbB.IsStatic) transB.Rotation += lambda * ratio * w2_rot;
            }
        }
    }

    void PhysicsSystem::SolvePulleyJoints(Real dt) {
        auto entities = m_entityManager->GetEntitiesWithComponent<PulleyJointComponent>();
        for (auto id : entities) {
            auto& joint = m_entityManager->GetComponent<PulleyJointComponent>(id);
            if (!joint.IsActive) continue;

            EntityID idA = joint.EntityA;
            EntityID idB = joint.EntityB;

            if (!m_entityManager->HasComponent<TransformComponent>(idA) ||
                !m_entityManager->HasComponent<TransformComponent>(idB)) continue;

            auto& transA = m_entityManager->GetComponent<TransformComponent>(idA);
            auto& transB = m_entityManager->GetComponent<TransformComponent>(idB);

            auto& rbA = m_entityManager->GetComponent<RigidBodyComponent>(idA);
            auto& rbB = m_entityManager->GetComponent<RigidBodyComponent>(idB);

            Vec2 pA = transA.Position + joint.LocalAnchorA.Rotate(transA.Rotation);
            Vec2 pB = transB.Position + joint.LocalAnchorB.Rotate(transB.Rotation);

            Vec2 dirA = pA - joint.GroundAnchorA;
            Vec2 dirB = pB - joint.GroundAnchorB;

            Real lenA = dirA.Magnitude();
            Real lenB = dirB.Magnitude();

            if (lenA < 0.001f || lenB < 0.001f) continue;

            Vec2 nA = dirA / lenA;
            Vec2 nB = dirB / lenB;

            // C = lenA + ratio * lenB - totalLength
            Real C = lenA + joint.Ratio * lenB - joint.TotalLength;

            Real w1 = rbA.InverseMass;
            Real w2 = rbB.InverseMass;

            Vec2 rA = pA - transA.Position;
            Vec2 rB = pB - transB.Position;

            Real rAxn = rA.x * nA.y - rA.y * nA.x;
            Real rBxn = rB.x * nB.y - rB.y * nB.x;

            Real w1_rot = rbA.InverseInertia * rAxn * rAxn;
            Real w2_rot = rbB.InverseInertia * rBxn * rBxn;

            Real denominator = w1 + w1_rot + joint.Ratio * joint.Ratio * (w2 + w2_rot) + (joint.Compliance / (dt * dt));
            if (denominator > 0.0001f) {
                Real lambda = -C / denominator;
                if (!rbA.IsStatic) {
                    transA.Position += nA * (lambda * w1);
                    transA.Rotation += lambda * rbA.InverseInertia * rAxn;
                }
                if (!rbB.IsStatic) {
                    transB.Position += nB * (lambda * joint.Ratio * w2);
                    transB.Rotation += lambda * joint.Ratio * rbB.InverseInertia * rBxn;
                }
            }
        }
    }

    /// XPBD velocity derivation: recomputes velocity/angular velocity from the
    /// position delta applied during Integrate + SolveConstraints, so subsequent
    /// impulse resolution acts on the corrected motion rather than the raw prediction.
    void PhysicsSystem::DeriveVelocities(Real dt) {
        auto entities = m_entityManager->GetEntitiesWithComponent<RigidBodyComponent>();
        for (auto id : entities) {
            if (!m_entityManager->HasComponent<TransformComponent>(id) || 
                !m_entityManager->HasComponent<MovementComponent>(id)) continue;
                
            auto& rb = m_entityManager->GetComponent<RigidBodyComponent>(id);
            if (rb.IsStatic || rb.IsSleeping) continue;
            
            auto& trans = m_entityManager->GetComponent<TransformComponent>(id);
            auto& move = m_entityManager->GetComponent<MovementComponent>(id);
            
            Vec2 derivedVel = (trans.Position - move.PrevPosition) / dt;
            Real derivedAngVel = (trans.Rotation - move.PrevRotation) / dt;

            // Clamp derived velocities to prevent numerical explosion from massive spawn/teleport corrections
            const Real maxVel = 2000.0f;
            const Real maxAngVel = 50.0f;
            Real velMag = derivedVel.Magnitude();
            if (velMag > maxVel) {
                derivedVel = derivedVel * (maxVel / velMag);
            }
            derivedAngVel = std::max(-maxAngVel, std::min(derivedAngVel, maxAngVel));

            // Apply damping directly to derived velocities to ensure energy dissipates
            Real linearDamping = move.LinearDamping;
            Real angularDamping = move.AngularDamping;
            derivedVel = derivedVel * (1.0f / (1.0f + linearDamping * dt));
            derivedAngVel = derivedAngVel * (1.0f / (1.0f + angularDamping * dt));

            move.Velocity = derivedVel;
            move.AngularVelocity = derivedAngVel;

            // Check if body qualifies for sleep
            bool slow = (move.Velocity.MagnitudeSqr() < SLEEP_LINEAR_THRESHOLD * SLEEP_LINEAR_THRESHOLD)
                      && (std::abs(move.AngularVelocity) < SLEEP_ANGULAR_THRESHOLD);
            if (rb.AllowSleep && slow) {
                rb.SleepTimer += dt;
                if (rb.SleepTimer >= SLEEP_TIME_THRESHOLD) {
                    rb.IsSleeping = true;
                    move.Velocity = {0.0f, 0.0f};
                    move.AngularVelocity = 0.0f;
                }
            } else {
                rb.SleepTimer = 0.0f;
            }
        }
    }

    /// Impulse-based velocity resolution: applies restitution along each contact
    /// normal (gated on the pre-step approach velocity) and Coulomb friction along
    /// the tangent, per contact point in m_contacts.
    void PhysicsSystem::ResolveVelocities(Real dt) {
        (void)dt;
        for (const auto& contact : m_contacts) {
            EntityID idA = contact.idA;
            EntityID idB = contact.idB;

            auto& rbA = m_entityManager->GetComponent<RigidBodyComponent>(idA);
            auto& moveA = m_entityManager->GetComponent<MovementComponent>(idA);
            auto& transA = m_entityManager->GetComponent<TransformComponent>(idA);

            auto& rbB = m_entityManager->GetComponent<RigidBodyComponent>(idB);
            auto& moveB = m_entityManager->GetComponent<MovementComponent>(idB);
            auto& transB = m_entityManager->GetComponent<TransformComponent>(idB);

            Real w1 = rbA.InverseMass;
            Real w2 = rbB.InverseMass;

            Real e = 0.5f; // Default restitution
            Real staticFriction = 0.5f;
            Real dynamicFriction = 0.3f;

            if (m_entityManager->HasComponent<PhysicalMaterialComponent>(idA) &&
                m_entityManager->HasComponent<PhysicalMaterialComponent>(idB)) {
                auto& matA = m_entityManager->GetComponent<PhysicalMaterialComponent>(idA);
                auto& matB = m_entityManager->GetComponent<PhysicalMaterialComponent>(idB);
                e = std::min(matA.Restitution, matB.Restitution);
                staticFriction = std::sqrt(matA.StaticFriction * matB.StaticFriction);
                dynamicFriction = std::sqrt(matA.DynamicFriction * matB.DynamicFriction);
            }

            Real numPts = (Real)contact.contactPoints.size();
            if (numPts == 0) continue;

            for (const auto& pt : contact.contactPoints) {
                Vec2 rA = pt - transA.Position;
                Vec2 rB = pt - transB.Position;

                // Use PRE-integration velocities to determine approach direction.
                // After DeriveVelocities, the velocity reflects position AFTER correction
                // (bodies pushed apart), so vn would be > 0 and restitution would be
                // skipped. PrevVelocity captures the actual approach intent.
                Vec2 prevVA = moveA.PrevVelocity + Vec2(-moveA.PrevAngularVelocity * rA.y, moveA.PrevAngularVelocity * rA.x);
                Vec2 prevVB = moveB.PrevVelocity + Vec2(-moveB.PrevAngularVelocity * rB.y, moveB.PrevAngularVelocity * rB.x);
                Real prevVn = (prevVA - prevVB).Dot(contact.normal);

                // Current velocity for computing the actual impulse
                Vec2 vA = moveA.Velocity + Vec2(-moveA.AngularVelocity * rA.y, moveA.AngularVelocity * rA.x);
                Vec2 vB = moveB.Velocity + Vec2(-moveB.AngularVelocity * rB.y, moveB.AngularVelocity * rB.x);
                Vec2 relVel = vA - vB;

                Real vn = relVel.Dot(contact.normal);
                Real rAxn = rA.x * contact.normal.y - rA.y * contact.normal.x;
                Real rBxn = rB.x * contact.normal.y - rB.y * contact.normal.x;
                Real denomNormal = w1 + w2 + rbA.InverseInertia * rAxn * rAxn + rbB.InverseInertia * rBxn * rBxn;

                Real jn = 0.0f;
                if (prevVn < -0.01f && vn < 0.0f) {
                    if (denomNormal > 0.0001f) {
                        Real targetVn = -e * prevVn;
                        jn = (targetVn - vn) / (denomNormal * numPts);
                        if (jn < 0.0f) jn = 0.0f;
                        
                        Vec2 normalImpulse = contact.normal * jn;

                        if (!rbA.IsStatic) {
                            moveA.Velocity += normalImpulse * w1;
                            moveA.AngularVelocity += rbA.InverseInertia * (rA.x * normalImpulse.y - rA.y * normalImpulse.x);
                        }
                        if (!rbB.IsStatic) {
                            moveB.Velocity -= normalImpulse * w2;
                            moveB.AngularVelocity -= rbB.InverseInertia * (rB.x * normalImpulse.y - rB.y * normalImpulse.x);
                        }
                    }
                }

                vA = moveA.Velocity + Vec2(-moveA.AngularVelocity * rA.y, moveA.AngularVelocity * rA.x);
                vB = moveB.Velocity + Vec2(-moveB.AngularVelocity * rB.y, moveB.AngularVelocity * rB.x);
                relVel = vA - vB;

                Vec2 tangent = relVel - contact.normal * relVel.Dot(contact.normal);
                Real tangentLen = tangent.Magnitude();
                if (tangentLen > 0.0001f) {
                    tangent = tangent / tangentLen;
                    Real vt = relVel.Dot(tangent);

                    Real rAxt = rA.x * tangent.y - rA.y * tangent.x;
                    Real rBxt = rB.x * tangent.y - rB.y * tangent.x;
                    Real denomTangent = w1 + w2 + rbA.InverseInertia * rAxt * rAxt + rbB.InverseInertia * rBxt * rBxt;

                    if (denomTangent > 0.0001f) {
                        Real jt = -vt / (denomTangent * numPts);

                        Real frictionCoefficient = staticFriction;
                        // Approximate normal force for resting contact: use gravity load if jn is 0
                        Real gravityMag = m_gravity.Magnitude();
                        Real normalForce = (jn > 0.0001f) ? jn : (gravityMag * dt / rbA.Mass);
                        Real maxFriction = frictionCoefficient * normalForce;
                        if (std::abs(jt) > maxFriction) {
                            jt = (jt > 0.0f ? 1.0f : -1.0f) * dynamicFriction * normalForce;
                        }

                        Vec2 frictionImpulse = tangent * jt;
                        if (!rbA.IsStatic) {
                            moveA.Velocity += frictionImpulse * w1;
                            moveA.AngularVelocity += rbA.InverseInertia * (rA.x * frictionImpulse.y - rA.y * frictionImpulse.x);
                        }
                        if (!rbB.IsStatic) {
                            moveB.Velocity -= frictionImpulse * w2;
                            moveB.AngularVelocity -= rbB.InverseInertia * (rB.x * frictionImpulse.y - rB.y * frictionImpulse.x);
                        }
                    }
                }
            }
        }
    }

    /// Transforms a Box, Polygon, or Chain collider's local-space vertices into world space
    /// using the owning entity's current position and rotation. No-op for other collider types.
    static void GetPolygonWorldVertices(const TransformComponent& trans, const ColliderComponent& col, std::vector<Vec2>& outVertices) {
        outVertices.clear();
        Real rot = trans.Rotation;
        Real cosRot = std::cos(rot);
        Real sinRot = std::sin(rot);
        
        if (col.Type == ColliderType::Polygon || col.Type == ColliderType::Chain) {
            for (const auto& localPt : col.Vertices) {
                // Apply rotation and translation
                Real rx = localPt.x * cosRot - localPt.y * sinRot;
                Real ry = localPt.x * sinRot + localPt.y * cosRot;
                outVertices.push_back({trans.Position.x + rx, trans.Position.y + ry});
            }
        } else if (col.Type == ColliderType::Box) {
            Vec2 half = col.Data.BoxHalfExtents;
            Vec2 localCorners[4] = {
                {half.x, half.y}, {-half.x, half.y}, {-half.x, -half.y}, {half.x, -half.y}
            };
            for (const auto& localPt : localCorners) {
                Real rx = localPt.x * cosRot - localPt.y * sinRot;
                Real ry = localPt.x * sinRot + localPt.y * cosRot;
                outVertices.push_back({trans.Position.x + rx, trans.Position.y + ry});
            }
        }
    }

    /// Ray vs line-segment intersection. Returns true and fills `t` (ray parameter) and
    /// `hitNormal` (facing against the ray) if the ray crosses the segment.
    static bool RaySegmentIntersect(const Vec2& rayStart, const Vec2& rayDir, const Vec2& segStart, const Vec2& segEnd, Real& t, Vec2& hitNormal) {
        Vec2 v1 = rayStart - segStart;
        Vec2 v2 = segEnd - segStart;
        Vec2 v3 = {-rayDir.y, rayDir.x};

        Real dot = v2.Dot(v3);
        if (std::abs(dot) < 0.000001f) return false;

        Real t1 = (v2.x * v1.y - v2.y * v1.x) / dot;
        Real t2 = v1.Dot(v3) / dot;

        if (t1 >= 0.0f && t2 >= 0.0f && t2 <= 1.0f) {
            t = t1;
            Vec2 edge = segEnd - segStart;
            hitNormal = Vec2(-edge.y, edge.x).Normalized();
            if (hitNormal.Dot(rayDir) > 0.0f) {
                hitNormal = hitNormal * -1.0f;
            }
            return true;
        }
        return false;
    }

    /// Ray vs circle intersection via the quadratic formula. Returns true and fills
    /// `t` (ray parameter) and `hitNormal` if the ray enters the circle at t >= 0.
    static bool RayCircleIntersect(const Vec2& rayStart, const Vec2& rayDir, const Vec2& center, Real radius, Real& t, Vec2& hitNormal) {
        Vec2 f = rayStart - center;
        Real a = rayDir.Dot(rayDir);
        Real b = 2.0f * f.Dot(rayDir);
        Real c = f.Dot(f) - radius * radius;

        Real discriminant = b * b - 4 * a * c;
        if (discriminant >= 0) {
            discriminant = std::sqrt(discriminant);
            Real t1 = (-b - discriminant) / (2.0f * a);
            Real t2 = (-b + discriminant) / (2.0f * a);

            if (t1 >= 0) {
                t = t1;
                Vec2 hitPt = rayStart + rayDir * t;
                hitNormal = (hitPt - center).Normalized();
                return true;
            }
            if (t2 >= 0) {
                t = t2;
                Vec2 hitPt = rayStart + rayDir * t;
                hitNormal = (hitPt - center).Normalized();
                return true;
            }
        }
        return false;
    }

    /// Casts a ray against every collider (Circle, Box, Polygon, Chain) and returns
    /// the closest hit within maxDistance, if any. See PhysicsSystem.h for parameter details.
    bool PhysicsSystem::Raycast(const Vec2& start, const Vec2& direction, Real maxDistance, Vec2& hitPoint, Vec2& hitNormal, Real& fraction, EntityID& hitEntity) {
        Real minT = maxDistance;
        bool hitFound = false;
        EntityID bestEntity = 0;
        Vec2 bestNormal = {0.0f, 0.0f};

        auto colliderEntities = m_entityManager->GetEntitiesWithComponent<ColliderComponent>();
        for (auto id : colliderEntities) {
            if (!m_entityManager->HasComponent<TransformComponent>(id)) continue;
            const auto& col = m_entityManager->GetComponent<ColliderComponent>(id);
            const auto& trans = m_entityManager->GetComponent<TransformComponent>(id);

            if (col.Type == ColliderType::Circle) {
                Real t = 0.0f;
                Vec2 norm;
                if (RayCircleIntersect(start, direction, trans.Position + col.CenterOffset, col.Data.Radius, t, norm)) {
                    if (t < minT) {
                        minT = t;
                        bestEntity = id;
                        bestNormal = norm;
                        hitFound = true;
                    }
                }
            } else if (col.Type == ColliderType::Box || col.Type == ColliderType::Polygon) {
                std::vector<Vec2> worldVerts;
                GetPolygonWorldVertices(trans, col, worldVerts);
                if (worldVerts.size() < 3) continue;

                for (size_t i = 0; i < worldVerts.size(); ++i) {
                    Vec2 p1 = worldVerts[i];
                    Vec2 p2 = worldVerts[(i + 1) % worldVerts.size()];
                    Real t = 0.0f;
                    Vec2 norm;
                    if (RaySegmentIntersect(start, direction, p1, p2, t, norm)) {
                        if (t < minT) {
                            minT = t;
                            bestEntity = id;
                            bestNormal = norm;
                            hitFound = true;
                        }
                    }
                }
            } else if (col.Type == ColliderType::Chain) {
                std::vector<Vec2> worldVerts;
                GetPolygonWorldVertices(trans, col, worldVerts);
                if (worldVerts.size() < 2) continue;

                for (size_t i = 0; i < worldVerts.size() - 1; ++i) {
                    Vec2 p1 = worldVerts[i];
                    Vec2 p2 = worldVerts[i + 1];
                    Real t = 0.0f;
                    Vec2 norm;
                    if (RaySegmentIntersect(start, direction, p1, p2, t, norm)) {
                        if (t < minT) {
                            minT = t;
                            bestEntity = id;
                            bestNormal = norm;
                            hitFound = true;
                        }
                    }
                }
            }
        }

        if (hitFound) {
            fraction = minT / maxDistance;
            hitPoint = start + direction * minT;
            hitNormal = bestNormal;
            hitEntity = bestEntity;
            return true;
        }
        return false;
    }

    void PhysicsSystem::SolveSoftBodies(Real dt) {
        auto entities = m_entityManager->GetEntitiesWithComponent<SoftBodyComponent>();
        for (auto id : entities) {
            auto& softBody = m_entityManager->GetComponent<SoftBodyComponent>(id);
            if (softBody.Nodes.empty()) continue;

            // Wake up nodes if any is awake
            bool anyAwake = false;
            for (auto node : softBody.Nodes) {
                if (m_entityManager->HasComponent<RigidBodyComponent>(node)) {
                    if (!m_entityManager->GetComponent<RigidBodyComponent>(node).IsSleeping) {
                        anyAwake = true;
                        break;
                    }
                }
            }
            if (anyAwake) {
                for (auto node : softBody.Nodes) {
                    if (m_entityManager->HasComponent<RigidBodyComponent>(node)) {
                        auto& rb = m_entityManager->GetComponent<RigidBodyComponent>(node);
                        rb.IsSleeping = false;
                        rb.SleepTimer = 0.0f;
                    }
                }
            }

            if (softBody.Type == SoftBodyType::Blob) {
                SolveSoftBodyArea(id, softBody, dt);
            } else if (softBody.Type == SoftBodyType::ShapeMatched) {
                SolveSoftBodyShapeMatch(id, softBody, dt);
            }
        }
    }

    void PhysicsSystem::SolveSoftBodyArea(EntityID id, SoftBodyComponent& softBody, Real dt) {
        (void)id;
        size_t n = softBody.Nodes.size();
        if (n < 3) return;

        // 1. Fetch current positions and inverse masses
        std::vector<Vec2> positions(n);
        std::vector<Real> invMasses(n);
        for (size_t i = 0; i < n; ++i) {
            EntityID node = softBody.Nodes[i];
            if (!m_entityManager->HasComponent<TransformComponent>(node)) return;
            positions[i] = m_entityManager->GetComponent<TransformComponent>(node).Position;
            invMasses[i] = m_entityManager->HasComponent<RigidBodyComponent>(node) ?
                           m_entityManager->GetComponent<RigidBodyComponent>(node).InverseMass : 0.0f;
        }

        // 2. Compute current signed area (Shoelace formula).
        //    TargetArea is also signed (negative for CW winding in screen-Y-down space).
        Real currentArea = 0.0f;
        for (size_t i = 0; i < n; ++i) {
            size_t next = (i + 1) % n;
            currentArea += positions[i].x * positions[next].y - positions[next].x * positions[i].y;
        }
        currentArea *= 0.5f;

        // 3. Compute constraint error: force current area back to TargetArea,
        //    handling winding sign correctly to restore shape structural integrity.
        Real C = currentArea - softBody.TargetArea;
        Real maxC = std::abs(softBody.TargetArea) * 0.20f;
        if (C >  maxC) C =  maxC;
        if (C < -maxC) C = -maxC;

        // Dead-zone: skip if error is negligible
        if (std::abs(C) < 0.5f) return;

        // 5. Compute gradients: ∂A/∂p_k = 0.5*(y_{k+1}-y_{k-1}, x_{k-1}-x_{k+1})
        //    These are computed once from snapshotted positions (pure Jacobi — no stale-hub bias).
        std::vector<Vec2> grads(n);
        Real denominator = 0.0f;
        for (size_t i = 0; i < n; ++i) {
            size_t prev = (i + n - 1) % n;
            size_t next = (i + 1) % n;
            grads[i] = Vec2(
                0.5f * (positions[next].y - positions[prev].y),
                0.5f * (positions[prev].x - positions[next].x)
            );
            denominator += invMasses[i] * grads[i].MagnitudeSqr();
        }

        denominator += softBody.AreaCompliance / (dt * dt);
        if (denominator < 0.0001f) return;

        Real deltaLambda = -C / denominator;

        // 6. Apply position corrections.
        //    Clamp correction by a velocity budget scaled by the actual sub-step dt (e.g. 300 px/s).
        const Real maxCorrectionVel = 300.0f;
        const Real maxCorrection = maxCorrectionVel * dt;
        for (size_t i = 0; i < n; ++i) {
            if (invMasses[i] > 0.0f) {
                Vec2 corr = grads[i] * (deltaLambda * invMasses[i]);
                Real mag = corr.Magnitude();
                if (mag > maxCorrection) corr = corr * (maxCorrection / mag);
                auto& trans = m_entityManager->GetComponent<TransformComponent>(softBody.Nodes[i]);
                trans.Position += corr;
            }
        }
    }

    void PhysicsSystem::SolveSoftBodyShapeMatch(EntityID id, SoftBodyComponent& softBody, Real dt) {
        (void)id;
        (void)dt;
        size_t n = softBody.Nodes.size();
        if (n == 0) return;

        // 1. Fetch current positions and masses
        std::vector<Vec2> x(n);
        std::vector<Real> m(n);
        Real totalMass = 0.0f;
        Vec2 x_cm = {0.0f, 0.0f};

        for (size_t i = 0; i < n; ++i) {
            EntityID node = softBody.Nodes[i];
            if (!m_entityManager->HasComponent<TransformComponent>(node)) return;
            x[i] = m_entityManager->GetComponent<TransformComponent>(node).Position;
            Real mass = 1.0f;
            if (m_entityManager->HasComponent<RigidBodyComponent>(node)) {
                Real invM = m_entityManager->GetComponent<RigidBodyComponent>(node).InverseMass;
                mass = invM > 0.0f ? 1.0f / invM : 1000.0f; // static behaves as heavy weight
            }
            m[i] = mass;
            totalMass += mass;
            x_cm += x[i] * mass;
        }
        if (totalMass < 0.0001f) return;
        x_cm = x_cm / totalMass;

        // 2. Build covariance matrix A = \sum m_i * (x_i - x_cm) * (x_0_i)^T
        // Note: RestPositions are already local (relative to rest center of mass)
        Real Axx = 0, Axy = 0, Ayx = 0, Ayy = 0;
        for (size_t i = 0; i < n; ++i) {
            Vec2 q = x[i] - x_cm;
            Vec2 p = softBody.RestPositions[i];
            Axx += m[i] * q.x * p.x;
            Axy += m[i] * q.x * p.y;
            Ayx += m[i] * q.y * p.x;
            Ayy += m[i] * q.y * p.y;
        }

        // 3. 2D Polar Decomposition: theta = atan2(Ayx - Axy, Axx + Ayy)
        Real theta = std::atan2(Ayx - Axy, Axx + Ayy);
        Real cosT = std::cos(theta);
        Real sinT = std::sin(theta);

        // 4. Pull nodes toward shape-matched target positions
        for (size_t i = 0; i < n; ++i) {
            Vec2 p = softBody.RestPositions[i];
            // Rotate rest position
            Vec2 rotatedRest = {
                p.x * cosT - p.y * sinT,
                p.x * sinT + p.y * cosT
            };
            Vec2 targetPos = rotatedRest + x_cm;

            // Apply displacement delta based on stiffness
            auto& trans = m_entityManager->GetComponent<TransformComponent>(softBody.Nodes[i]);
            trans.Position += (targetPos - trans.Position) * softBody.Stiffness;
        }
    }
}