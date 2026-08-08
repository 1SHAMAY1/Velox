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

    PhysicsSystem::PhysicsSystem(std::shared_ptr<EntityManager> entityManager)
        : m_entityManager(entityManager) {
        m_contacts.Reserve(1024);
        m_candidatePairs.Reserve(2048);
    }

    /// Advances the full simulation by `dt`, snapshotting pre-integration velocities
    /// (needed for restitution) before running the sub-stepped XPBD pipeline.
    void PhysicsSystem::Step(Real dt) {
        // 1. Execute all registered Pre-Step & Force Application behaviors
        const auto& behaviors = PhysicsBehaviorRegistry::Get().GetBehaviors();
        for (const auto& b : behaviors) {
            b->OnPreStep(*m_entityManager, dt);
        }

        ApplyRules(dt);

        for (const auto& b : behaviors) {
            b->OnApplyForces(*m_entityManager, dt);
        }

        const auto& entities = m_entityManager->GetEntitiesWithComponent<RigidBodyComponent>();
        const int nEntities = static_cast<int>(entities.size());

        // Run BVH Broadphase ONCE per frame with fat AABBs (O(N log N) -> single pass per frame)
        UpdateBroadphase(dt);

        // Run 4 sub-steps for optimal XPBD convergence at ultra-high FPS (>1,000 FPS)
        const int subSteps = 4;
        Real subDt = dt / (Real)subSteps;

        // Initialize island manager with capacity for all entities
        m_islandManager.Initialize(entities.size() * 2 + 1024);

        auto* softBodyArray = m_entityManager->GetComponentArrayFast<SoftBodyComponent>();
        auto* revArray = m_entityManager->GetComponentArrayFast<RevoluteJointComponent>();
        auto* prismArray = m_entityManager->GetComponentArrayFast<PrismaticJointComponent>();
        auto* gearArray = m_entityManager->GetComponentArrayFast<GearJointComponent>();
        auto* pulleyArray = m_entityManager->GetComponentArrayFast<PulleyJointComponent>();

        bool hasSoftBodies = softBodyArray && !softBodyArray->GetDenseEntities().empty();
        bool hasRevJoints = revArray && !revArray->GetDenseEntities().empty();
        bool hasPrismJoints = prismArray && !prismArray->GetDenseEntities().empty();
        bool hasGearJoints = gearArray && !gearArray->GetDenseEntities().empty();
        bool hasPulleyJoints = pulleyArray && !pulleyArray->GetDenseEntities().empty();

        for (int s = 0; s < subSteps; ++s) {
            // Snapshot velocity per sub-step so restitution acts on genuine approach velocity
            #pragma omp parallel for schedule(static, 64)
            for (int i = 0; i < nEntities; ++i) {
                EntityID id = entities[i];
                if (m_entityManager->HasComponent<MovementComponent>(id)) {
                    auto& move = m_entityManager->GetComponent<MovementComponent>(id);
                    move.PrevVelocity = move.Velocity;
                    move.PrevAngularVelocity = move.AngularVelocity;
                }
            }

            Integrate(subDt);

            for (const auto& b : behaviors) {
                b->OnSolveConstraints(*m_entityManager, subDt);
            }

            if (hasSoftBodies) SolveSoftBodies(subDt);
            SolveConstraints(subDt);
            if (hasRevJoints) SolveRevoluteJoints(subDt);
            if (hasPrismJoints) SolvePrismaticJoints(subDt);
            if (hasGearJoints) SolveGearJoints(subDt);
            if (hasPulleyJoints) SolvePulleyJoints(subDt);
            DeriveVelocities(subDt);
            ResolveVelocities(subDt);
        }

        // Island-level sleep propagation
        for (const auto& contact : m_contacts) {
            m_islandManager.Union(contact.idA, contact.idB);
        }

        for (auto id : entities) {
            if (!m_entityManager->HasComponent<MovementComponent>(id)) continue;
            auto& move = m_entityManager->GetComponent<MovementComponent>(id);
            auto& rb = m_entityManager->GetComponent<RigidBodyComponent>(id);
            if (rb.IsStatic) continue;

            float ke = 0.5f * rb.Mass * move.Velocity.MagnitudeSqr();
            m_islandManager.AddKineticEnergy(id, ke);
        }

        for (auto id : entities) {
            auto& rb = m_entityManager->GetComponent<RigidBodyComponent>(id);
            if (rb.IsStatic || !rb.AllowSleep) continue;
            if (m_entityManager->HasComponent<SoftBodyComponent>(id)) continue;

            // Also check if entity has collider with GroupId (softbody node)
            if (m_entityManager->HasComponent<ColliderComponent>(id)) {
                if (m_entityManager->GetComponent<ColliderComponent>(id).GroupId != -1) continue;
            }

            if (m_islandManager.IsIslandSettled(id, 8.0f)) {
                rb.SleepTimer += dt;
                if (rb.SleepTimer >= SLEEP_TIME_THRESHOLD) {
                    rb.IsSleeping = true;
                    if (m_entityManager->HasComponent<MovementComponent>(id)) {
                        auto& move = m_entityManager->GetComponent<MovementComponent>(id);
                        move.Velocity = {0.0f, 0.0f};
                        move.AngularVelocity = 0.0f;
                    }
                }
            }
        }

        for (const auto& b : behaviors) {
            b->OnPostStep(*m_entityManager, dt);
        }

        // --- Event-Based Architecture & Broken Contact Wake-Up ---
        std::unordered_map<ContactPairKey, Vec2, ContactPairKeyHash> currentContacts;
        std::unordered_map<ContactPairKey, bool, ContactPairKeyHash> currentSensors;

        for (const auto& contact : m_contacts) {
            EntityID a = std::min(contact.idA, contact.idB);
            EntityID b = std::max(contact.idA, contact.idB);
            ContactPairKey key{a, b};

            bool isSensor = false;
            if (m_entityManager->HasComponent<ColliderComponent>(a) && m_entityManager->GetComponent<ColliderComponent>(a).IsSensor) isSensor = true;
            if (m_entityManager->HasComponent<ColliderComponent>(b) && m_entityManager->GetComponent<ColliderComponent>(b).IsSensor) isSensor = true;

            if (isSensor) {
                currentSensors[key] = true;
            } else {
                currentContacts[key] = contact.normal;
            }
        }

        // 1. Check Broken Contacts & Collision Ends (Wakes sleeping bodies when supporting objects move/disappear)
        for (const auto& kv : m_persistentContacts) {
            if (currentContacts.find(kv.first) == currentContacts.end()) {
                WakeBody(kv.first.idA);
                WakeBody(kv.first.idB);
                if (m_collisionEndCb) {
                    m_collisionEndCb(kv.first.idA, kv.first.idB, kv.second.x, kv.second.y, m_collisionEndUserData);
                }
            }
        }

        // 2. Check Collision Begins
        if (m_collisionBeginCb) {
            for (const auto& kv : currentContacts) {
                if (m_persistentContacts.find(kv.first) == m_persistentContacts.end()) {
                    m_collisionBeginCb(kv.first.idA, kv.first.idB, kv.second.x, kv.second.y, m_collisionBeginUserData);
                }
            }
        }

        // 3. Check Sensor Triggers (Enter / Exit)
        if (m_sensorCb) {
            for (const auto& kv : currentSensors) {
                if (m_persistentSensors.find(kv.first) == m_persistentSensors.end()) {
                    m_sensorCb(kv.first.idA, kv.first.idB, true, m_sensorUserData); // Entered
                }
            }
            for (const auto& kv : m_persistentSensors) {
                if (currentSensors.find(kv.first) == currentSensors.end()) {
                    m_sensorCb(kv.first.idA, kv.first.idB, false, m_sensorUserData); // Exited
                }
            }
        }

        m_persistentContacts = std::move(currentContacts);
        m_persistentSensors = std::move(currentSensors);
    }

    /// Drives non-collision gameplay behaviours each frame: force fields, rotation
    /// motors, oscillators, and projectile facing. Runs once per Step(), before sub-stepping.
    void PhysicsSystem::ApplyRules(Real dt) {
        const auto& fieldEntities = m_entityManager->GetEntitiesWithComponent<ForceFieldComponent>();
        const auto& rotEntities = m_entityManager->GetEntitiesWithComponent<RotationComponent>();
        const auto& oscEntities = m_entityManager->GetEntitiesWithComponent<OscillationComponent>();
        const auto& projEntities = m_entityManager->GetEntitiesWithComponent<ProjectileComponent>();

        if (fieldEntities.empty() && rotEntities.empty() && oscEntities.empty() && projEntities.empty()) {
            return;
        }

        // Apply Force Fields
        // 1. Gather all Force Fields
        std::vector<ForceFieldComponent> fields;
        std::vector<Vec2> fieldPositions; // For PointGravity

        if (!fieldEntities.empty()) {
            fields.reserve(fieldEntities.size());
            fieldPositions.reserve(fieldEntities.size());
            for (auto id : fieldEntities) {
                fields.push_back(m_entityManager->GetComponent<ForceFieldComponent>(id));
                if (m_entityManager->HasComponent<TransformComponent>(id)) {
                    fieldPositions.push_back(m_entityManager->GetComponent<TransformComponent>(id).Position);
                } else {
                    fieldPositions.push_back({0,0});
                }
            }
        }

        // 2. Apply RotationComponent (Motor)
        for (auto id : rotEntities) {
            if (m_entityManager->HasComponent<MovementComponent>(id)) {
                auto& rot = m_entityManager->GetComponent<RotationComponent>(id);
                auto& move = m_entityManager->GetComponent<MovementComponent>(id);
                
                Real dir = (rot.Direction == RotationDirection::Clockwise) ? 1.0f : -1.0f;
                move.AngularVelocity = rot.Speed * dir;
            }
        }

        // 3. Apply OscillationComponent
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
        if (!fields.empty()) {
            const auto& rbEntities = m_entityManager->GetEntitiesWithComponent<RigidBodyComponent>();
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
    }

    void PhysicsSystem::UpdateBroadphase(Real dt) {
        (void)dt;
        m_spatialHash.Clear();
        m_candidatePairs.Clear();

        auto* colArray = m_entityManager->GetComponentArrayFast<ColliderComponent>();
        auto* transArray = m_entityManager->GetComponentArrayFast<TransformComponent>();
        auto* rbArray = m_entityManager->GetComponentArrayFast<RigidBodyComponent>();
        if (!colArray || !transArray) return;

        const auto& colliderEntities = colArray->GetDenseEntities();
        if (colliderEntities.empty()) return;

        struct EntityAABB {
            EntityID id;
            AABB aabb;
        };
        std::vector<EntityAABB> entityAABBs;
        entityAABBs.reserve(colliderEntities.size());

        // 1. Populate FlatSpatialHash with fast AABB calculation
        for (EntityID id : colliderEntities) {
            if (!transArray->HasData(id)) continue;

            const auto& col = colArray->GetData(id);
            const auto& trans = transArray->GetData(id);

            AABB aabb;
            switch (col.Type) {
                case ColliderType::Circle: {
                    Real r = col.Data.Radius;
                    aabb.min = trans.Position - Vec2(r, r);
                    aabb.max = trans.Position + Vec2(r, r);
                    break;
                }
                case ColliderType::Box: {
                    Vec2 half = col.Data.BoxHalfExtents;
                    Real cosA = std::abs(std::cos(trans.Rotation));
                    Real sinA = std::abs(std::sin(trans.Rotation));
                    Real ex = half.x * cosA + half.y * sinA;
                    Real ey = half.x * sinA + half.y * cosA;
                    aabb.min = trans.Position - Vec2(ex, ey);
                    aabb.max = trans.Position + Vec2(ex, ey);
                    break;
                }
                case ColliderType::Polygon: {
                    if (col.Vertices.empty()) {
                        aabb.min = trans.Position;
                        aabb.max = trans.Position;
                    } else {
                        Vec2 first = trans.Position + col.Vertices[0].Rotate(trans.Rotation);
                        aabb.min = first;
                        aabb.max = first;
                        for (size_t i = 1; i < col.Vertices.size(); ++i) {
                            Vec2 worldV = trans.Position + col.Vertices[i].Rotate(trans.Rotation);
                            aabb.min.x = std::min(aabb.min.x, worldV.x);
                            aabb.min.y = std::min(aabb.min.y, worldV.y);
                            aabb.max.x = std::max(aabb.max.x, worldV.x);
                            aabb.max.y = std::max(aabb.max.y, worldV.y);
                        }
                    }
                    break;
                }
                case ColliderType::Chain: {
                    if (col.Vertices.empty()) {
                        aabb.min = trans.Position;
                        aabb.max = trans.Position;
                    } else {
                        aabb.min = col.Vertices[0];
                        aabb.max = col.Vertices[0];
                        for (size_t i = 1; i < col.Vertices.size(); ++i) {
                            aabb.min.x = std::min(aabb.min.x, col.Vertices[i].x);
                            aabb.min.y = std::min(aabb.min.y, col.Vertices[i].y);
                            aabb.max.x = std::max(aabb.max.x, col.Vertices[i].x);
                            aabb.max.y = std::max(aabb.max.y, col.Vertices[i].y);
                        }
                    }
                    break;
                }
            }

            entityAABBs.push_back({id, aabb});
            m_spatialHash.Insert(id, aabb.min, aabb.max);
        }

        // 2. Query candidate pairs using FlatSpatialHash with fast deduplication stamp
        static std::vector<uint32_t> queryStamps(16384, 0);
        static uint32_t queryToken = 1;

        for (const auto& ea : entityAABBs) {
            EntityID idA = ea.id;
            bool isAStatic = rbArray && rbArray->HasData(idA) && rbArray->GetData(idA).IsStatic;
            uint32_t token = ++queryToken;
            if (token == 0) {
                std::fill(queryStamps.begin(), queryStamps.end(), 0);
                token = ++queryToken;
            }

            m_spatialHash.Query(ea.aabb.min, ea.aabb.max, [&](uint32_t userData) {
                EntityID idB = userData;
                if (idA < idB) {
                    if (idB >= queryStamps.size()) queryStamps.resize(idB + 4096, 0);
                    if (queryStamps[idB] != token) {
                        queryStamps[idB] = token;
                        bool isBStatic = rbArray && rbArray->HasData(idB) && rbArray->GetData(idB).IsStatic;
                        if (!isAStatic || !isBStatic) {
                            m_candidatePairs.PushBack({idA, idB});
                        }
                    }
                }
                return true;
            });
        }
    }

    /// XPBD prediction step: applies damping and gravity, then integrates velocity
    /// and position forward by `dt` for every non-static dynamic body.
    void PhysicsSystem::Integrate(Real dt) {
        const auto& entities = m_entityManager->GetEntitiesWithComponent<RigidBodyComponent>();
        const int n = static_cast<int>(entities.size());

        #pragma omp parallel for schedule(static, 64)
        for (int i = 0; i < n; ++i) {
            EntityID id = entities[i];
            if (!m_entityManager->HasComponent<TransformComponent>(id) ||
                !m_entityManager->HasComponent<MovementComponent>(id)) continue;

            auto& rb = m_entityManager->GetComponent<RigidBodyComponent>(id);
            if (rb.IsStatic || rb.IsSleeping) continue;

            auto& transform = m_entityManager->GetComponent<TransformComponent>(id);
            auto& move = m_entityManager->GetComponent<MovementComponent>(id);

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
        m_contacts.Clear();

        auto* colArray = m_entityManager->GetComponentArrayFast<ColliderComponent>();
        auto* transArray = m_entityManager->GetComponentArrayFast<TransformComponent>();
        auto* rbArray = m_entityManager->GetComponentArrayFast<RigidBodyComponent>();
        if (!colArray || !transArray || !rbArray) return;

        // Narrowphase entry point: resolves one candidate pair by dispatching to the
        // shape-specific solver below based on each entity's ColliderType.
        auto ResolveCollision = [&](EntityID idA, EntityID idB) {
            auto& colA = colArray->GetData(idA);
            auto& colB = colArray->GetData(idB);

            // Skip collisions if both entities belong to the same collision group (e.g., same soft body)
            if (colA.GroupId != -1 && colA.GroupId == colB.GroupId) return;

            auto& transA = transArray->GetData(idA);
            auto& rbA = rbArray->GetData(idA);
            
            auto& transB = transArray->GetData(idB);
            auto& rbB = rbArray->GetData(idB);

            // Circle vs Circle: distance-based overlap test with proportional positional correction.
            auto ResolveCircleCircle = [&]() {
                Vec2 n = transA.Position - transB.Position;
                Real radiusSum = colA.Data.Radius + colB.Data.Radius;
                Real distSqr = n.MagnitudeSqr();

                if (distSqr >= radiusSum * radiusSum) {
                    // Continuous Collision Detection (Swept TOI) only for fast moving circles
                    auto* moveArray = m_entityManager->GetComponentArrayFast<MovementComponent>();
                    Vec2 vA = (moveArray && moveArray->HasData(idA)) ? moveArray->GetData(idA).Velocity : Vec2(0, 0);
                    Vec2 vB = (moveArray && moveArray->HasData(idB)) ? moveArray->GetData(idB).Velocity : Vec2(0, 0);
                    Real relSpeedSqr = (vA - vB).MagnitudeSqr();

                    if (relSpeedSqr * (dt * dt) > radiusSum * radiusSum * 0.25f) {
                        TOIResult toi = SweptCircleCircle(transA.Position - vA * dt, vA, colA.Data.Radius,
                                                          transB.Position - vB * dt, vB, colB.Data.Radius, dt);
                        if (toi.hit && toi.toi >= 0.0f && toi.toi <= dt) {
                            Vec2 hitPosA = (transA.Position - vA * dt) + vA * toi.toi;
                            Vec2 hitPosB = (transB.Position - vB * dt) + vB * toi.toi;
                            Vec2 sweptN = hitPosA - hitPosB;
                            Real sDist = sweptN.Magnitude();
                            if (sDist > 1e-5f) sweptN = sweptN * (1.0f / sDist);
                            else sweptN = Vec2(0, -1);

                            if (!rbA.IsStatic) transA.Position = hitPosA + sweptN * 0.5f;
                            if (!rbB.IsStatic) transB.Position = hitPosB - sweptN * 0.5f;
                            m_contacts.PushBack({idA, idB, sweptN, 0.5f, {hitPosB + sweptN * colB.Data.Radius}});
                            return;
                        }
                    }
                    return;
                }

                if (distSqr > 1e-8f) {
                    Real dist = std::sqrt(distSqr);
                    n = n * (1.0f / dist);
                    Real rawPen = radiusSum - dist;
                    // Contact slop (0.1px) and relaxation (0.8) to prevent jitter and energy gain
                    Real penetration = std::max(0.0f, rawPen - 0.1f) * 0.8f;

                    // Wake up bodies only on genuine contact
                    if (rbA.IsSleeping) { rbA.IsSleeping = false; rbA.SleepTimer = 0.0f; }
                    if (rbB.IsSleeping) { rbB.IsSleeping = false; rbB.SleepTimer = 0.0f; }
                    
                    if (colA.IsSensor || colB.IsSensor) {
                        m_contacts.PushBack({idA, idB, n, rawPen, {transB.Position + n * colB.Data.Radius}});
                        return;
                    }
                    
                    Real w1 = rbA.InverseMass;
                    Real w2 = rbB.InverseMass;
                    if (w1 + w2 == 0.0f) return;

                    Vec2 dx = n * (penetration / (w1 + w2));
                    
                    if (!rbA.IsStatic) transA.Position += dx * w1;
                    if (!rbB.IsStatic) transB.Position -= dx * w2;

                    m_contacts.PushBack({idA, idB, n, rawPen, {transB.Position + n * colB.Data.Radius}});
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

                // Swept CCD for high-speed bullets / circles vs static or dynamic boxes
                auto* moveArray = m_entityManager->GetComponentArrayFast<MovementComponent>();
                Vec2 cV = (moveArray && moveArray->HasData(circleID)) ? moveArray->GetData(circleID).Velocity : Vec2(0, 0);
                Vec2 bV = (moveArray && moveArray->HasData(boxID)) ? moveArray->GetData(boxID).Velocity : Vec2(0, 0);
                Real relSpeedSqr = (cV - bV).MagnitudeSqr();
                Real minHalf = std::min(boxHalf.x, boxHalf.y);

                if (relSpeedSqr * (dt * dt) > minHalf * minHalf * 0.25f) {
                    Vec2 sweptNorm;
                    TOIResult toi = SweptCircleBox(circlePos - cV * dt, cV, cCol.Data.Radius,
                                                   boxPos - bV * dt, bV, boxHalf, bTrans.Rotation,
                                                   dt, sweptNorm);
                    if (toi.hit && toi.toi >= 0.0f && toi.toi <= dt) {
                        Vec2 hitPos = (circlePos - cV * dt) + cV * toi.toi + sweptNorm * 0.5f;
                        if (!cRb.IsStatic) cTrans.Position = hitPos - cCol.CenterOffset;
                        if (moveArray && moveArray->HasData(circleID)) {
                            auto& mv = moveArray->GetData(circleID);
                            Real vn = mv.Velocity.Dot(sweptNorm);
                            if (vn < 0.0f) mv.Velocity -= sweptNorm * vn;
                        }
                        m_contacts.PushBack({circleID, boxID, sweptNorm, 0.5f, {hitPos - sweptNorm * cCol.Data.Radius}});
                        return;
                    }
                }

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

                if (distSqr < radius * radius && distSqr > 1e-8f) {
                    Real dist = std::sqrt(distSqr);
                    n = n * (1.0f / dist); 
                    Real rawPen = radius - dist;
                    Real penetration = std::max(0.0f, rawPen - 0.1f) * 0.8f;

                    // Wake up bodies on contact
                    if (cRb.IsSleeping) { cRb.IsSleeping = false; cRb.SleepTimer = 0.0f; }
                    if (bRb.IsSleeping) { bRb.IsSleeping = false; bRb.SleepTimer = 0.0f; }

                    if (cCol.IsSensor || bCol.IsSensor) {
                        m_contacts.PushBack({circleID, boxID, n, rawPen, {closestWorld}});
                        return;
                    }

                    Real w1 = cRb.InverseMass;
                    Real w2 = bRb.InverseMass;
                    if (w1 + w2 == 0.0f) return;

                    Vec2 correction = n * (penetration / (w1 + w2));
                    
                    if (!cRb.IsStatic) cTrans.Position += correction * w1;
                    if (!bRb.IsStatic) bTrans.Position -= correction * w2;

                    m_contacts.PushBack({circleID, boxID, n, rawPen, {closestWorld}});
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

                // Direct AABB Fast-Path for unrotated boxes (eliminates 4 trig calls and 4-axis SAT loops)
                if (std::abs(rotA) < 1e-4f && std::abs(rotB) < 1e-4f) {
                    Vec2 delta = posB - posA;
                    Real ox = (halfA.x + halfB.x) - std::abs(delta.x);
                    if (ox <= 0.0f) return;
                    Real oy = (halfA.y + halfB.y) - std::abs(delta.y);
                    if (oy <= 0.0f) return;

                    Vec2 n;
                    Real penetration;

                    if (ox < oy) {
                        penetration = ox;
                        n = delta.x < 0.0f ? Vec2(-1.0f, 0.0f) : Vec2(1.0f, 0.0f);
                    } else {
                        penetration = oy;
                        n = delta.y < 0.0f ? Vec2(0.0f, -1.0f) : Vec2(0.0f, 1.0f);
                    }

                    std::array<Vec2, 4> contactPoints{};
                    uint8_t count = 0;

                    if (ox < oy) {
                        Real minAY = posA.y - halfA.y, maxAY = posA.y + halfA.y;
                        Real minBY = posB.y - halfB.y, maxBY = posB.y + halfB.y;
                        Real minY = std::max(minAY, minBY);
                        Real maxY = std::min(maxAY, maxBY);
                        Real contactX = delta.x > 0.0f ? (posA.x + halfA.x) : (posA.x - halfA.x);
                        contactPoints[0] = Vec2(contactX, minY);
                        contactPoints[1] = Vec2(contactX, maxY);
                        count = (minY == maxY) ? 1 : 2;
                    } else {
                        Real minAX = posA.x - halfA.x, maxAX = posA.x + halfA.x;
                        Real minBX = posB.x - halfB.x, maxBX = posB.x + halfB.x;
                        Real minX = std::max(minAX, minBX);
                        Real maxX = std::min(maxAX, maxBX);
                        Real contactY = delta.y > 0.0f ? (posA.y + halfA.y) : (posA.y - halfA.y);
                        contactPoints[0] = Vec2(minX, contactY);
                        contactPoints[1] = Vec2(maxX, contactY);
                        count = (minX == maxX) ? 1 : 2;
                    }

                    // Wake up bodies on genuine contact
                    if (rbA.IsSleeping) { rbA.IsSleeping = false; rbA.SleepTimer = 0.0f; }
                    if (rbB.IsSleeping) { rbB.IsSleeping = false; rbB.SleepTimer = 0.0f; }

                    if (colA.IsSensor || colB.IsSensor) {
                        m_contacts.PushBack(ContactInfo(idA, idB, n, penetration, contactPoints, count));
                        return;
                    }

                    Real w1 = rbA.InverseMass;
                    Real w2 = rbB.InverseMass;
                    if (w1 + w2 == 0.0f) return;

                    Real rawPen = penetration;
                    Real effPen = std::max(0.0f, rawPen - 0.05f) * 0.9f;

                    Vec2 correction = n * (effPen / (w1 + w2));
                    if (!rbA.IsStatic) transA.Position -= correction * w1;
                    if (!rbB.IsStatic) transB.Position += correction * w2;

                    // Stabilization: when unrotated boxes are stacked on a flat horizontal surface,
                    // damp micro-rotational drift to prevent artificial toppling
                    if (std::abs(n.y) > 0.9f) {
                        if (!rbA.IsStatic && std::abs(transA.Rotation) < 0.02f) transA.Rotation *= 0.9f;
                        if (!rbB.IsStatic && std::abs(transB.Rotation) < 0.02f) transB.Rotation *= 0.9f;
                    }

                    m_contacts.PushBack(ContactInfo(idA, idB, n, rawPen, contactPoints, count));
                    return;
                }

                // Rotated Box SAT
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
                    Real rawPen = minOverlap;
                    Real penetration = std::max(0.0f, rawPen - 0.1f) * 0.8f;

                    // Contact point generation (zero heap allocation stack buffer)
                    std::array<Vec2, 4> contactPoints{};
                    uint8_t count = 0;
                    
                    auto GetBoxCorners = [](const Vec2& center, const Vec2& half, Real rotation) -> std::array<Vec2, 4> {
                        std::array<Vec2, 4> corners{};
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
                        if (IsPointInBox(pt, posB, halfB, rotB) && count < 4) {
                            contactPoints[count++] = pt;
                        }
                    }
                    for (const auto& pt : cornersB) {
                        if (IsPointInBox(pt, posA, halfA, rotA) && count < 4) {
                            contactPoints[count++] = pt;
                        }
                    }

                    if (count == 0) {
                        contactPoints[0] = (posA + posB) * 0.5f;
                        count = 1;
                    }

                    // Wake up bodies on contact
                    if (rbA.IsSleeping) { rbA.IsSleeping = false; rbA.SleepTimer = 0.0f; }
                    if (rbB.IsSleeping) { rbB.IsSleeping = false; rbB.SleepTimer = 0.0f; }

                    if (colA.IsSensor || colB.IsSensor) {
                        m_contacts.PushBack(ContactInfo(idA, idB, n, rawPen, contactPoints, count));
                        return;
                    }

                    Real w1 = rbA.InverseMass;
                    Real w2 = rbB.InverseMass;
                    if (w1 + w2 == 0.0f) return;

                    Vec2 correction = n * (penetration / (w1 + w2));
                    
                    if (!rbA.IsStatic) transA.Position -= correction * w1;
                    if (!rbB.IsStatic) transB.Position += correction * w2;

                    m_contacts.PushBack(ContactInfo(idA, idB, n, rawPen, contactPoints, count));
                }
            };

            // Half-space / support-point solver: polygon vs single chain segment.
            // A segment has no volume, so SAT is unreliable - instead we test the signed
            // distance of every polygon vertex from the segment's half-plane.
            auto SATPolygonVsSegment = [&](EntityID polyID, EntityID chainID, Vec2 segA, Vec2 segB) {
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

                if (pRb.IsSleeping) { pRb.IsSleeping = false; pRb.SleepTimer = 0.0f; }

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
                m_contacts.PushBack({polyID, chainID, n, penetration, {contactPt}});
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

                // Wake up bodies on contact
                if (rbA_ref.IsSleeping) { rbA_ref.IsSleeping = false; rbA_ref.SleepTimer = 0.0f; }
                if (rbB_ref.IsSleeping) { rbB_ref.IsSleeping = false; rbB_ref.SleepTimer = 0.0f; }

                if (colA_ref.IsSensor || colB_ref.IsSensor) {
                    m_contacts.PushBack({idA, idB, n, penetration, {(centerA + centerB)*0.5f}});
                    return;
                }

                Real w1 = rbA_ref.InverseMass;
                Real w2 = rbB_ref.InverseMass;
                if (w1 + w2 == 0.0f) return;

                Vec2 correction = n * (penetration / (w1 + w2));
                if (!rbA_ref.IsStatic) transA_ref.Position += correction * w1;
                if (!rbB_ref.IsStatic) transB_ref.Position -= correction * w2;

                m_contacts.PushBack({idA, idB, n, penetration, {(centerA + centerB)*0.5f}});
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

                        // Wake up bodies on contact
                        if (cRb.IsSleeping) { cRb.IsSleeping = false; cRb.SleepTimer = 0.0f; }
                        if (pRb.IsSleeping) { pRb.IsSleeping = false; pRb.SleepTimer = 0.0f; }

                        if (cCol.IsSensor || pCol.IsSensor) {
                            m_contacts.PushBack({circleID, polyID, n, penetration, {closestPoint}});
                            return;
                        }

                        Real w1 = cRb.InverseMass;
                        Real w2 = pRb.InverseMass;
                        if (w1 + w2 == 0.0f) return;

                        Vec2 correction = n * (penetration / (w1 + w2));
                        if (!cRb.IsStatic) cTrans.Position += correction * w1;
                        if (!pRb.IsStatic) pTrans.Position -= correction * w2;

                        m_contacts.PushBack({circleID, polyID, n, penetration, {closestPoint}});
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

                    // Wake up bodies on contact
                    if (cRb.IsSleeping) { cRb.IsSleeping = false; cRb.SleepTimer = 0.0f; }
                    if (pRb.IsSleeping) { pRb.IsSleeping = false; pRb.SleepTimer = 0.0f; }

                    if (cCol.IsSensor || pCol.IsSensor) {
                        m_contacts.PushBack({circleID, polyID, n, penetration, {circleCenter - n * radius}});
                        return;
                    }

                    Real w1 = cRb.InverseMass;
                    Real w2 = pRb.InverseMass;
                    if (w1 + w2 == 0.0f) return;

                    Vec2 correction = n * (penetration / (w1 + w2));
                    if (!cRb.IsStatic) cTrans.Position += correction * w1;
                    if (!pRb.IsStatic) pTrans.Position -= correction * w2;

                    m_contacts.PushBack({circleID, polyID, n, penetration, {circleCenter - n * radius}});
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
                                    SATPolygonVsSegment(idA, idB, chainCol.Vertices[si], chainCol.Vertices[si+1]);
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
                                    SATPolygonVsSegment(idA, idB, chainCol.Vertices[si], chainCol.Vertices[si+1]);
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
                                    SATPolygonVsSegment(idB, idA, chainCol.Vertices[si], chainCol.Vertices[si+1]);
                                }
                            }
                            break;
                        case ColliderType::Polygon:
                            // Chain vs Polygon: test each chain segment separately
                            {
                                auto& chainCol = m_entityManager->GetComponent<ColliderComponent>(idA);
                                for (size_t si = 0; si + 1 < chainCol.Vertices.size(); ++si) {
                                    SATPolygonVsSegment(idB, idA, chainCol.Vertices[si], chainCol.Vertices[si+1]);
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

        // Resolve overlaps using cached candidate pairs (Zero-heap iteration)
        const size_t numPairs = m_candidatePairs.Size();
        for (size_t p = 0; p < numPairs; ++p) {
            const auto& pair = m_candidatePairs[p];
            ResolveCollision(pair.first, pair.second);
        }

        // --- Solve Distance/Joint Constraints ---
        const auto& jointEntities = m_entityManager->GetEntitiesWithComponent<JointComponent>();
        if (jointEntities.empty()) return;

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
        const auto& entities = m_entityManager->GetEntitiesWithComponent<RevoluteJointComponent>();
        if (entities.empty()) return;
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
        const auto& entities = m_entityManager->GetEntitiesWithComponent<PrismaticJointComponent>();
        if (entities.empty()) return;
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
            Vec2 d = pB - pA;

            // Axis in world space (attached to Body A)
            Vec2 axisA = joint.LocalAxisA.Rotate(transA.Rotation);
            Vec2 perpA = {-axisA.y, axisA.x}; // Perpendicular constraint axis

            Real w1 = rbA.InverseMass;
            Real w2 = rbB.InverseMass;

            // 1. Position constraint along perpendicular axis (C_perp = perpA . d = 0)
            Real C_perp = perpA.Dot(d);
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

            // 3. Linear motor along sliding axis (Applied before hard limit enforcement)
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

            // 4. Translation limits along sliding axis (Hard clamp enforcement)
            if (joint.LimitsEnabled) {
                // Refresh anchors after motor
                rA = joint.LocalAnchorA.Rotate(transA.Rotation);
                rB = joint.LocalAnchorB.Rotate(transB.Rotation);
                pA = transA.Position + rA;
                pB = transB.Position + rB;
                d = pB - pA;
                Real translation = d.Dot(axisA);

                if (translation < joint.MinTranslation) {
                    Real delta = joint.MinTranslation - translation;
                    if (!rbB.IsStatic) transB.Position += axisA * delta;
                    else if (!rbA.IsStatic) transA.Position -= axisA * delta;
                } else if (translation > joint.MaxTranslation) {
                    Real delta = joint.MaxTranslation - translation;
                    if (!rbB.IsStatic) transB.Position += axisA * delta;
                    else if (!rbA.IsStatic) transA.Position -= axisA * delta;
                }
            }
        }
    }

    void PhysicsSystem::SolveGearJoints(Real dt) {
        const auto& entities = m_entityManager->GetEntitiesWithComponent<GearJointComponent>();
        if (entities.empty()) return;
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
        const auto& entities = m_entityManager->GetEntitiesWithComponent<PulleyJointComponent>();
        if (entities.empty()) return;
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
        const auto& entities = m_entityManager->GetEntitiesWithComponent<RigidBodyComponent>();
        for (auto id : entities) {
            if (!m_entityManager->HasComponent<TransformComponent>(id) || 
                !m_entityManager->HasComponent<MovementComponent>(id)) continue;
                
            auto& rb = m_entityManager->GetComponent<RigidBodyComponent>(id);
            if (rb.IsStatic || rb.IsSleeping) continue;
            
            auto& trans = m_entityManager->GetComponent<TransformComponent>(id);
            auto& move = m_entityManager->GetComponent<MovementComponent>(id);
            // In XPBD: blend derived velocity from positional correction with integrated velocity
            // to preserve momentum and prevent positional overlap corrections from creating artificial kinetic energy
            Vec2 posDeltaVel = (trans.Position - move.PrevPosition) / dt;
            Real posDeltaAngVel = (trans.Rotation - move.PrevRotation) / dt;

            // Strict velocity bound: derived velocity magnitude cannot exceed incoming velocity + gravity acceleration
            Real prevSpeed = move.PrevVelocity.Magnitude();
            Real maxAllowedSpeed = std::max(prevSpeed * 1.05f, 20.0f);
            Real posDeltaSpeed = posDeltaVel.Magnitude();
            if (posDeltaSpeed > maxAllowedSpeed && prevSpeed > 0.0f) {
                posDeltaVel = posDeltaVel * (maxAllowedSpeed / posDeltaSpeed);
            }

            Vec2 derivedVel = posDeltaVel;
            Real derivedAngVel = posDeltaAngVel;

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

            Real numPts = (Real)contact.contactCount;
            if (numPts == 0) continue;

            for (uint8_t pi = 0; pi < contact.contactCount; ++pi) {
                const auto& pt = contact.contactPoints[pi];
                Vec2 rA = pt - transA.Position;
                Vec2 rB = pt - transB.Position;

                // Current velocity for computing the actual impulse
                Vec2 vA = moveA.Velocity + Vec2(-moveA.AngularVelocity * rA.y, moveA.AngularVelocity * rA.x);
                Vec2 vB = moveB.Velocity + Vec2(-moveB.AngularVelocity * rB.y, moveB.AngularVelocity * rB.x);
                Vec2 relVel = vA - vB;

                Real rAxn = rA.x * contact.normal.y - rA.y * contact.normal.x;
                Real rBxn = rB.x * contact.normal.y - rB.y * contact.normal.x;
                Real denomNormal = w1 + w2 + rbA.InverseInertia * rAxn * rAxn + rbB.InverseInertia * rBxn * rBxn;

                // Pre-step incoming approach velocity (before positional XPBD correction)
                Vec2 vA_in = moveA.PrevVelocity + Vec2(-moveA.PrevAngularVelocity * rA.y, moveA.PrevAngularVelocity * rA.x);
                Vec2 vB_in = moveB.PrevVelocity + Vec2(-moveB.PrevAngularVelocity * rB.y, moveB.PrevAngularVelocity * rB.x);
                Real vn_in = (vA_in - vB_in).Dot(contact.normal);

                // Current derived separating velocity
                Vec2 vA_cur = moveA.Velocity + Vec2(-moveA.AngularVelocity * rA.y, moveA.AngularVelocity * rA.x);
                Vec2 vB_cur = moveB.Velocity + Vec2(-moveB.AngularVelocity * rB.y, moveB.AngularVelocity * rB.x);
                Real vn_cur = (vA_cur - vB_cur).Dot(contact.normal);

                // Resting contact threshold (25.0 px/s): if incoming speed is below gravity sub-step, do not bounce
                Real effectiveRestitution = (vn_in > -25.0f) ? 0.0f : e;
                Real jn = 0.0f;

                if (vn_in < -25.0f && effectiveRestitution > 0.0f) {
                    Real targetVn = -effectiveRestitution * vn_in;
                    Real deltaVn = targetVn - vn_cur;
                    if (deltaVn > 0.0f && denomNormal > 0.0001f) {
                        jn = deltaVn / (denomNormal * numPts);
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
        const auto& entities = m_entityManager->GetEntitiesWithComponent<SoftBodyComponent>();
        if (entities.empty()) return;
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

            // Project soft body nodes against static obstacles after volume/shape restoration
            ProjectSoftBodyAgainstStaticObstacles(id, softBody);
        }

        // Edge-segment continuous envelope collision to prevent rigid bodies penetrating cushion interior
        SolveSoftBodyEdgeCollisions(dt);
    }

    void PhysicsSystem::ProjectSoftBodyAgainstStaticObstacles(EntityID id, SoftBodyComponent& softBody) {
        (void)id;
        auto* rbArray = m_entityManager->GetComponentArrayFast<RigidBodyComponent>();
        auto* colArray = m_entityManager->GetComponentArrayFast<ColliderComponent>();
        auto* transArray = m_entityManager->GetComponentArrayFast<TransformComponent>();
        if (!rbArray || !colArray || !transArray) return;

        const auto& colliderEntities = colArray->GetDenseEntities();

        for (EntityID nodeId : softBody.Nodes) {
            if (!transArray->HasData(nodeId) || !colArray->HasData(nodeId)) continue;
            auto& nodeTrans = transArray->GetData(nodeId);
            auto& nodeCol = colArray->GetData(nodeId);
            Real nodeR = nodeCol.Data.Radius;

            for (EntityID obsId : colliderEntities) {
                if (!rbArray->HasData(obsId) || !rbArray->GetData(obsId).IsStatic) continue;
                if (!transArray->HasData(obsId)) continue;

                const auto& obsCol = colArray->GetData(obsId);
                const auto& obsTrans = transArray->GetData(obsId);

                if (obsCol.Type == ColliderType::Box) {
                    Vec2 boxHalf = obsCol.Data.BoxHalfExtents;
                    Vec2 relPos = (nodeTrans.Position + nodeCol.CenterOffset) - (obsTrans.Position + obsCol.CenterOffset);
                    Vec2 localPos = relPos.Rotate(-obsTrans.Rotation);

                    Vec2 clampedLocal = localPos;
                    clampedLocal.x = std::max(-boxHalf.x, std::min(clampedLocal.x, boxHalf.x));
                    clampedLocal.y = std::max(-boxHalf.y, std::min(clampedLocal.y, boxHalf.y));

                    Vec2 closestWorld = (obsTrans.Position + obsCol.CenterOffset) + clampedLocal.Rotate(obsTrans.Rotation);
                    Vec2 n = (nodeTrans.Position + nodeCol.CenterOffset) - closestWorld;
                    Real distSqr = n.MagnitudeSqr();

                    if (distSqr < nodeR * nodeR && distSqr > 1e-8f) {
                        Real dist = std::sqrt(distSqr);
                        n = n * (1.0f / dist);
                        Real pen = nodeR - dist;
                        nodeTrans.Position += n * pen;
                    } else if (distSqr <= 1e-8f) {
                        // Deeply embedded inside static box - project out along closest axis
                        Real dx = boxHalf.x - std::abs(localPos.x);
                        Real dy = boxHalf.y - std::abs(localPos.y);
                        Vec2 outLocal = localPos;
                        if (dx < dy) {
                            outLocal.x = (localPos.x >= 0.0f) ? (boxHalf.x + nodeR + 0.1f) : (-boxHalf.x - nodeR - 0.1f);
                        } else {
                            outLocal.y = (localPos.y >= 0.0f) ? (boxHalf.y + nodeR + 0.1f) : (-boxHalf.y - nodeR - 0.1f);
                        }
                        nodeTrans.Position = (obsTrans.Position + obsCol.CenterOffset) + outLocal.Rotate(obsTrans.Rotation);
                    }
                }
            }
        }
    }

    void PhysicsSystem::SolveSoftBodyEdgeCollisions(Real dt) {
        (void)dt;
        const auto& sbEntities = m_entityManager->GetEntitiesWithComponent<SoftBodyComponent>();
        if (sbEntities.empty()) return;

        auto* rbArray = m_entityManager->GetComponentArrayFast<RigidBodyComponent>();
        auto* colArray = m_entityManager->GetComponentArrayFast<ColliderComponent>();
        auto* transArray = m_entityManager->GetComponentArrayFast<TransformComponent>();
        if (!rbArray || !colArray || !transArray) return;

        const auto& colliderEntities = colArray->GetDenseEntities();

        for (EntityID sbId : sbEntities) {
            auto& sb = m_entityManager->GetComponent<SoftBodyComponent>(sbId);
            size_t n = sb.Nodes.size();
            if (n < 3) continue;

            for (size_t i = 0; i < n; ++i) {
                size_t next = (i + 1) % n;
                EntityID id1 = sb.Nodes[i];
                EntityID id2 = sb.Nodes[next];
                if (!transArray->HasData(id1) || !transArray->HasData(id2)) continue;

                auto& t1 = transArray->GetData(id1);
                auto& t2 = transArray->GetData(id2);
                Vec2 p1 = t1.Position;
                Vec2 p2 = t2.Position;
                Vec2 edge = p2 - p1;
                Real edgeLenSqr = edge.MagnitudeSqr();
                if (edgeLenSqr < 1e-6f) continue;
                Real edgeLen = std::sqrt(edgeLenSqr);
                Vec2 edgeDir = edge * (1.0f / edgeLen);
                Vec2 edgeNormal = Vec2(-edgeDir.y, edgeDir.x); // CCW perpendicular normal

                for (EntityID rigidId : colliderEntities) {
                    if (!rbArray->HasData(rigidId)) continue;
                    auto& rb = rbArray->GetData(rigidId);
                    if (rb.IsStatic) continue;

                    auto& col = colArray->GetData(rigidId);
                    if (col.GroupId != -1 && col.GroupId == colArray->GetData(id1).GroupId) continue;

                    auto& trans = transArray->GetData(rigidId);
                    Vec2 center = trans.Position + col.CenterOffset;
                    Real r = (col.Type == ColliderType::Circle) ? col.Data.Radius : std::max(col.Data.BoxHalfExtents.x, col.Data.BoxHalfExtents.y);

                    // Project center onto edge line
                    Vec2 toCenter = center - p1;
                    Real t = std::max(0.0f, std::min(edgeLen, toCenter.Dot(edgeDir)));
                    Vec2 closestPoint = p1 + edgeDir * t;
                    Vec2 diff = center - closestPoint;
                    Real distSqr = diff.MagnitudeSqr();

                    if (distSqr < r * r && distSqr > 1e-8f) {
                        Real dist = std::sqrt(distSqr);
                        Vec2 edgeNorm = diff * (1.0f / dist);
                        Real pen = r - dist;

                        Real wRigid = rb.InverseMass;
                        Real wNode = 0.5f; // Each edge endpoint absorbs half correction
                        Real totalW = wRigid + wNode;
                        if (totalW > 0.0f) {
                            trans.Position += edgeNorm * (pen * (wRigid / totalW));
                            t1.Position -= edgeNorm * (pen * (0.5f * wNode / totalW));
                            t2.Position -= edgeNorm * (pen * (0.5f * wNode / totalW));
                            m_contacts.PushBack({rigidId, id1, edgeNorm, pen, {closestPoint}});
                        }
                    }
                }
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

    void PhysicsSystem::WakeBody(EntityID id) {
        if (m_entityManager && m_entityManager->HasComponent<RigidBodyComponent>(id)) {
            auto& rb = m_entityManager->GetComponent<RigidBodyComponent>(id);
            if (!rb.IsStatic) {
                rb.IsSleeping = false;
                rb.SleepTimer = 0.0f;
            }
        }
    }

    void PhysicsSystem::WakeTouching(EntityID id) {
        WakeBody(id);
        for (const auto& kv : m_persistentContacts) {
            if (kv.first.idA == id) {
                WakeBody(kv.first.idB);
            } else if (kv.first.idB == id) {
                WakeBody(kv.first.idA);
            }
        }
    }
}

