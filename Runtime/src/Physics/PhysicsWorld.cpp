#include "Physics/PhysicsWorld.h"

#include <algorithm>
#include <cassert>
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <stdexcept>
#include <utility>

#include "Core/JobSystem.h"

namespace Runtime {
namespace Physics {

// Initialize the shared worker pool and collision detector from default settings.
PhysicsWorld::PhysicsWorld()
    : m_integrator(std::make_unique<SemiImplicitEulerIntegrator>()) {
    Core::JobSystem::Get().Initialize(m_settings.workerCount);
    m_collisionDetector.SetParallelNarrowphaseEnabled(m_settings.parallelNarrowphaseEnabled);
    m_collisionDetector.SetNarrowphaseMinPairsPerJob(m_settings.narrowphaseMinPairsPerJob);
    m_collisionDetector.SetSpeculativeContactDistance(m_settings.speculativeContactDistance);
}

// Reconfigure the shared pool; workerCount includes the calling thread.
void PhysicsWorld::SetPhysicsWorkerCount(uint32_t workerCount) {
    m_settings.workerCount = workerCount;
    Core::JobSystem::Get().Initialize(workerCount);
}

// Read the resolved shared-pool size, which may also be changed by other worlds.
uint32_t PhysicsWorld::PhysicsWorkerCount() const {
    return Core::JobSystem::Get().WorkerCount();
}

// Keep world settings and the detector's narrow-phase dispatch mode synchronized.
void PhysicsWorld::SetParallelNarrowphaseEnabled(bool enabled) {
    m_settings.parallelNarrowphaseEnabled = enabled;
    m_collisionDetector.SetParallelNarrowphaseEnabled(enabled);
}

// Select job-based or inline island solving for subsequent sub-steps.
void PhysicsWorld::SetParallelIslandSolverEnabled(bool enabled) {
    m_settings.parallelIslandSolverEnabled = enabled;
}

// Wake sleepers on a gravity change when sleep evaluation is enabled.
void PhysicsWorld::SetGravity(const glm::vec3& gravity) {
    if (gravity != m_gravity && m_settings.sleepEnabled) {
        for (auto& entry : m_bodies) {
            if (entry.second.IsSleeping()) {
                entry.second.WakeUp();
            }
        }
    }
    m_gravity = gravity;
}

// Update only the CCD activation distance; preserve the speculative band.
void PhysicsWorld::SetCcdMotionThreshold(float distance) {
    if (!std::isfinite(distance) || distance < 0.0f) {
        throw std::invalid_argument("CCD motion threshold must be finite and nonnegative");
    }
    m_settings.ccdMotionThreshold = distance;
}

// Clamp the speculative band to [0, fatMargin] and update narrow-phase settings.
void PhysicsWorld::SetSpeculativeContactDistance(float distance) {
    constexpr float kBroadPhaseFatMargin = 0.08f;
    m_settings.speculativeContactDistance =
        std::max(0.0f, std::min(distance, kBroadPhaseFatMargin));
    m_collisionDetector.SetSpeculativeContactDistance(m_settings.speculativeContactDistance);
}

// Disabling sleep wakes all bodies; enabling it starts normal island rest checks.
void PhysicsWorld::SetSleepEnabled(bool enabled) {
    if (m_settings.sleepEnabled == enabled) {
        return;
    }
    m_settings.sleepEnabled = enabled;
    if (!enabled) {
        for (auto& entry : m_bodies) {
            entry.second.WakeUp();
        }
    }
}

// Construct and store a body under a monotonically assigned runtime id.
uint32_t PhysicsWorld::CreateRigidBody(const RigidBodyDesc& desc) {
    const uint32_t id = m_nextBodyId++;
    RigidBody body(desc);
    body.SetId(id);
    m_bodies[id] = body;
    return id;
}

// Wake sleeping contact neighbors before removing a body and its collider.
bool PhysicsWorld::DestroyRigidBody(uint32_t bodyId) {
    if (m_settings.sleepEnabled && m_bodies.find(bodyId) != m_bodies.end()) {
        Midphase& midphase = m_collisionDetector.GetMidphase();
        // Wake the opposite endpoint's dynamic island for each retained physical contact.
        midphase.ForEachPair([&](const MidphasePair& pair) {
            if (!pair.hadContact || pair.manifold.isTrigger) {
                return;
            }
            uint32_t other = 0;
            if (pair.key.bodyA == bodyId) {
                other = pair.key.bodyB;
            } else if (pair.key.bodyB == bodyId) {
                other = pair.key.bodyA;
            } else {
                return;
            }
            const auto otherIt = m_bodies.find(other);
            if (otherIt != m_bodies.end() && !otherIt->second.IsStatic() &&
                otherIt->second.IsSleeping()) {
                WakeIslandContaining(other, 0.0f);
            }
        });
    }
    const bool bodyRemoved = m_bodies.erase(bodyId) > 0;
    m_colliders.erase(bodyId);
    return bodyRemoved;
}

// Test membership in the body-id map.
bool PhysicsWorld::HasRigidBody(uint32_t bodyId) const {
    return m_bodies.find(bodyId) != m_bodies.end();
}

// Return mutable body storage or nullptr for an unknown id.
RigidBody* PhysicsWorld::GetRigidBody(uint32_t bodyId) {
    auto it = m_bodies.find(bodyId);
    if (it == m_bodies.end()) {
        return nullptr;
    }
    return &it->second;
}

// Return read-only body storage or nullptr for an unknown id.
const RigidBody* PhysicsWorld::GetRigidBody(uint32_t bodyId) const {
    auto it = m_bodies.find(bodyId);
    if (it == m_bodies.end()) {
        return nullptr;
    }
    return &it->second;
}

// Attach a valid shape with a fresh collider identity and wake affected islands.
bool PhysicsWorld::AttachCollider(uint32_t bodyId, const ColliderDesc& desc) {
    auto bodyIt = m_bodies.find(bodyId);
    if (bodyIt == m_bodies.end() || !desc.shape) {
        return false;
    }

    Collider collider(desc);
    collider.SetBodyId(bodyId);
    // Identity distinguishes replacement even when property revisions coincide.
    collider.SetIdentity(m_nextColliderIdentity++);
    m_colliders[bodyId] = std::move(collider);
    if (m_settings.sleepEnabled) {
        if (bodyIt->second.IsStatic()) {
            WakeStaticAdjacency(bodyId);
        } else {
            WakeIslandContaining(bodyId, 0.0f);
        }
    }
    return true;
}

// Wake the body's island or its sleeping static-support neighbors before detaching.
bool PhysicsWorld::RemoveCollider(uint32_t bodyId) {
    if (m_colliders.find(bodyId) == m_colliders.end()) {
        return false;
    }
    if (m_settings.sleepEnabled) {
        // Persistent pairs remain until the next detection sweep publishes Exit.
        const auto bodyIt = m_bodies.find(bodyId);
        if (bodyIt != m_bodies.end() && bodyIt->second.IsStatic()) {
            Midphase& midphase = m_collisionDetector.GetMidphase();
            // Wake sleeping dynamic islands whose retained contact loses this support.
            midphase.ForEachPair([&](const MidphasePair& pair) {
                if (!pair.hadContact || pair.manifold.isTrigger) {
                    return;
                }
                uint32_t other = 0;
                if (pair.key.bodyA == bodyId) {
                    other = pair.key.bodyB;
                } else if (pair.key.bodyB == bodyId) {
                    other = pair.key.bodyA;
                } else {
                    return;
                }
                const auto otherIt = m_bodies.find(other);
                if (otherIt != m_bodies.end() && !otherIt->second.IsStatic() &&
                    otherIt->second.IsSleeping()) {
                    WakeIslandContaining(other, 0.0f);
                }
            });
        } else {
            WakeIslandContaining(bodyId, 0.0f);
        }
    }
    return m_colliders.erase(bodyId) > 0;
}

// Test whether a collider is attached to the body id.
bool PhysicsWorld::HasCollider(uint32_t bodyId) const {
    return m_colliders.find(bodyId) != m_colliders.end();
}

// Return mutable attached collider storage or nullptr when absent.
Collider* PhysicsWorld::GetCollider(uint32_t bodyId) {
    auto it = m_colliders.find(bodyId);
    if (it == m_colliders.end()) {
        return nullptr;
    }
    return &it->second;
}

// Return read-only attached collider storage or nullptr when absent.
const Collider* PhysicsWorld::GetCollider(uint32_t bodyId) const {
    auto it = m_colliders.find(bodyId);
    if (it == m_colliders.end()) {
        return nullptr;
    }
    return &it->second;
}

// Run floor((accumulator + deltaTime) / fixedTimeStep) ticks and retain the remainder.
void PhysicsWorld::Step(float deltaTime) {
    if (deltaTime <= 0.0f) {
        return;
    }

    using Clock = std::chrono::steady_clock;
    const auto stepStart = Clock::now();
    m_lastStepStats = {};
    // Events cover every fixed step executed by this Step call.
    m_contactEvents.clear();
    for (const auto& entry : m_bodies) {
        if (entry.second.IsStatic()) {
            ++m_lastStepStats.staticBodyCount;
        } else {
            ++m_lastStepStats.dynamicBodyCount;
        }
    }

    m_accumulator += deltaTime;
    while (m_accumulator >= m_fixedTimeStep) {
        FixedStep(m_fixedTimeStep);
        m_accumulator -= m_fixedTimeStep;
        ++m_lastStepStats.fixedStepCount;
    }
    // Activity counts reflect the state after all ticks in this call.
    for (const auto& entry : m_bodies) {
        if (entry.second.IsStatic()) {
            continue;
        }
        if (entry.second.IsSleeping()) {
            ++m_lastStepStats.sleepingBodyCount;
        } else {
            ++m_lastStepStats.awakeBodyCount;
        }
    }
    m_lastStepStats.totalMilliseconds =
        std::chrono::duration<double, std::milli>(Clock::now() - stepStart).count();
}

// Replace the broad-phase implementation only when the comparison mode changes.
void PhysicsWorld::SetLegacyBroadPhaseEnabled(bool enabled) {
    if (m_legacyBroadPhase == enabled) {
        return;
    }
    m_legacyBroadPhase = enabled;
    if (enabled) {
        m_collisionDetector.SetBroadPhase(std::make_unique<DynamicBvhBroadPhase>());
    } else {
        m_collisionDetector.SetBroadPhase(std::make_unique<HybridBvhBroadPhase>());
    }
}

// Integrate forces and gravity for awake bodies; accumulate integration timing.
void PhysicsWorld::IntegrateBodyVelocities(float dt) {
    if (dt <= 0.0f) {
        return;
    }
    const auto start = std::chrono::steady_clock::now();
    for (auto& kv : m_bodies) {
        if (kv.second.IsSleeping()) {
            continue;
        }
        kv.second.IntegrateVelocity(dt, m_gravity);
    }
    m_lastStepStats.integrationMilliseconds +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

// Advance awake positions and orientations using their solved velocities.
void PhysicsWorld::IntegrateBodyPositions(float dt) {
    if (dt <= 0.0f) {
        return;
    }
    const auto start = std::chrono::steady_clock::now();
    for (auto& kv : m_bodies) {
        if (kv.second.IsSleeping()) {
            continue;
        }
        kv.second.IntegratePositions(dt);
    }
    m_lastStepStats.integrationMilliseconds +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

// Detect, wake, and solve at one pose; CCD may supply a certified impact patch.
void PhysicsWorld::DetectAndSolveContacts(float substepDt, bool solve, bool warmStart,
    const ContactManifold* impactContact) {
    // Contact wakes catch up the full fixed-step force/gravity increment,
    // not the shorter remaining TOI horizon; each sleeper is compensated once.
    WakeRequestCallback wakeCallback{};
    if (m_settings.sleepEnabled) {
        wakeCallback.user = this;
        // Bridge the detector's wake request to the owning world's island traversal.
        wakeCallback.invoke = [](void* user, uint32_t sleepingBodyId) {
            auto* self = static_cast<PhysicsWorld*>(user);
            return self->WakeIslandContaining(sleepingBodyId, self->m_wakeCompensationDt);
        };
    }
    const CollisionDetectionResult& result = m_collisionDetector.Detect(
        m_colliders, m_bodies, m_fixedStepId, ++m_queryEpoch, substepDt,
        wakeCallback, impactContact);
    AccumulateDetectionStats(result);
    BuildIslands(result.touchingPairs);
    if (!solve) {
        // Detect-only must not replace solved snapshots with zeroed impulse telemetry.
        return;
    }
    if (!result.touchingPairs.empty()) {
        const auto solverStart = std::chrono::steady_clock::now();
        // Jobs solve disjoint islands; cache commit remains serial and stable.
        PrepareIslandConstraints(result.touchingPairs, substepDt, warmStart, impactContact);
        SolveIslandConstraints();
        m_lastStepStats.velocitySolverPassCount += static_cast<uint64_t>(m_solverIterations);
        CommitSolvedContacts();
        m_lastStepStats.solverMilliseconds +=
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - solverStart).count();
    }
    PublishTouchingContacts(result.touchingPairs);
}

// Correct final poses with two nonlinear Gauss-Seidel iterations per island.
void PhysicsWorld::SolveContactPositions() {
    if (m_preparedConstraints.empty()) {
        return;
    }
    ++m_lastStepStats.positionSolverPassCount;
    Midphase& midphase = m_collisionDetector.GetMidphase();

    // Reuse velocity jobs; each applies sequential corrections to disjoint islands.
    const auto processJobs = [&](std::size_t begin, std::size_t end) {
        for (std::size_t job = begin; job < end; ++job) {
            for (std::size_t entry = m_islandJobs[job].scheduleBegin;
                 entry < m_islandJobs[job].scheduleEnd; ++entry) {
                const IslandConstraintRange range =
                    m_islandConstraintRanges[m_islandSchedule[entry]];
                for (int iteration = 0; iteration < 2; ++iteration) {
                    for (std::size_t index = range.begin; index < range.end; ++index) {
                        PreparedContactConstraint& constraint = m_preparedConstraints[index];
                        m_contactSolver.ResolvePosition(
                            constraint,
                            midphase.PairAt(constraint.slotIndex).manifold);
                    }
                }
            }
        }
    };

    Core::JobSystem& jobSystem = Core::JobSystem::Get();
    if (m_settings.parallelIslandSolverEnabled && jobSystem.WorkerCount() > 1 && !m_islandJobs.empty()) {
        jobSystem.ParallelForRange(m_islandJobs.size(), 1, processJobs);
    } else if (!m_islandJobs.empty()) {
        processJobs(0, m_islandJobs.size());
    }
}

// Apply forces once, solve velocities, advance through impacts, then correct poses and evaluate rest.
void PhysicsWorld::FixedStep(float dt) {
    if (!m_integrator) {
        return;
    }

    ++m_fixedStepId;
    // Wake externally modified islands before the single force integration.
    ProcessExternalActivityWakes();
    // Mid-step wakes missed exactly dt of force/gravity velocity integration.
    m_wakeCompensationDt = dt;
    // Keep one snapshot per pair touched by any solving pass in this tick.
    m_contacts.clear();
    m_stepContactIndex.clear();
    if (!m_enableCcd) {
        // Forces -> velocity constraints at current poses -> advancement -> pose correction.
        IntegrateBodyVelocities(dt);
        DetectAndSolveContacts(dt, true);
        IntegrateBodyPositions(dt);
        SolveContactPositions();
    } else {
        // Integrate forces once; split only pose advancement and impact velocity solves.
        IntegrateBodyVelocities(dt);
        DetectAndSolveContacts(dt, true);

        float remaining = dt;
        int subStep = 0;
        bool triggerInvolvedInToi = false;
        while (remaining > 1e-6f && subStep < m_ccdMaxSubSteps) {
            // True initial touches are solver-owned; separated speculative pairs
            // still need a sweep when rotation can bring a different feature into contact.
            const TimeOfImpact toi = m_continuousCollision.FindEarliestImpact(
                m_colliders, m_bodies, m_collisionDetector.GetMidphase(), remaining,
                m_settings.ccdMotionThreshold);
            ++m_lastStepStats.ccdSubStepCount;
            if (!toi.hit) {
                break;
            }
            ++m_lastStepStats.ccdToiHitCount;
            if (m_colliders.at(toi.bodyA).IsTrigger() || m_colliders.at(toi.bodyB).IsTrigger()) {
                triggerInvolvedInToi = true;
            }

            const float advance = std::clamp(toi.toi, 0.0f, remaining);
            IntegrateBodyPositions(advance);
            remaining -= advance;
            if (remaining <= 1e-6f) {
                // An end-of-window touch is solved at the start of the next tick.
                remaining = 0.0f;
                break;
            }
            // Bias uses the remaining motion horizon; do not reapply warm-start
            // impulses already in the velocities or run position recovery at impact.
            DetectAndSolveContacts(remaining, true, false, &toi.contact);
            ++subStep;
        }
        if (remaining > 1e-6f && subStep == m_ccdMaxSubSteps)
            ++m_lastStepStats.ccdBudgetExhaustionCount;
        // Preserve remaining motion even if the sweep budget is exhausted.
        IntegrateBodyPositions(remaining);
        if (triggerInvolvedInToi) {
            // Final detection publishes same-tick Exit for trigger pass-throughs.
            DetectAndSolveContacts(remaining, false);
            // Only solving passes publish contact snapshots; refresh final touches too.
            DetectAndSolveContacts(std::max(remaining, 1e-6f), true, false);
        }
        // Re-evaluate separation at final poses, never inside the TOI loop.
        SolveContactPositions();
    }

    // Publish the fixed-step summary in stable PairKey order.
    std::sort(m_contacts.begin(), m_contacts.end(), [](const ContactManifold& a, const ContactManifold& b) {
        return MakePairKey(a.bodyA, a.bodyB) < MakePairKey(b.bodyA, b.bodyB);
    });

    EvaluateIslandSleep(dt);

    // Consume pending forces only in the first completed tick of Step().
    for (auto& entry : m_bodies) {
        entry.second.ClearForces();
        // Activity has now fed both wake processing and rest evaluation.
        entry.second.ClearExternalActivity();
    }
}

// Sum query work while treating tree and persistent-pair counts as last-query snapshots.
void PhysicsWorld::AccumulateDetectionStats(const CollisionDetectionResult& result) {
    const CollisionDetectionStats& detectionStats = m_collisionDetector.LastStats();
    ++m_lastStepStats.collisionDetectionPassCount;
    m_lastStepStats.staticBvhLeafCount = detectionStats.staticLeafCount;
    m_lastStepStats.dynamicBvhLeafCount = detectionStats.dynamicLeafCount;
    m_lastStepStats.broadPhaseCandidateCount += detectionStats.broadPhaseCandidateCount;
    m_lastStepStats.narrowPhaseTestCount += detectionStats.narrowPhaseTestCount;
    m_lastStepStats.satCallCount += detectionStats.satCallCount;
    m_lastStepStats.primitiveCallCount += detectionStats.primitiveCallCount;
    m_lastStepStats.gjkCallCount += detectionStats.gjkCallCount;
    m_lastStepStats.gjkFailureCount += detectionStats.gjkFailureCount;
    m_lastStepStats.epaCallCount += detectionStats.epaCallCount;
    m_lastStepStats.epaFailureCount += detectionStats.epaFailureCount;
    m_lastStepStats.manifoldCount += detectionStats.manifoldCount;
    m_lastStepStats.contactPointCount += detectionStats.contactPointCount;
    // Pair pool sizes are absolute snapshots, not per-query addends.
    m_lastStepStats.midphaseActivePairCount = detectionStats.midphaseActivePairCount;
    m_lastStepStats.midphaseNewPairCount = detectionStats.midphaseNewPairCount;
    m_lastStepStats.midphaseRemovedPairCount += detectionStats.midphaseRemovedPairCount;
    m_lastStepStats.broadPhaseMilliseconds += detectionStats.broadPhaseMilliseconds;
    m_lastStepStats.narrowPhaseMilliseconds += detectionStats.narrowPhaseMilliseconds;

    // Sum job/time work; retain maximum worker participation across queries.
    const NarrowPhaseParallelStats& parallelStats = m_collisionDetector.LastParallelStats();
    m_lastStepStats.narrowPhaseJobCount += parallelStats.jobCount;
    m_lastStepStats.narrowPhaseWorkerCount = std::max<std::size_t>(
        m_lastStepStats.narrowPhaseWorkerCount,
        parallelStats.workerParticipation);
    m_lastStepStats.narrowPhaseWorkerBusyMilliseconds += parallelStats.workerBusyMilliseconds;
    m_lastStepStats.narrowPhaseTailWaitMilliseconds += parallelStats.tailWaitMilliseconds;

    m_contactEvents.insert(
        m_contactEvents.end(),
        result.events.begin(),
        result.events.end());
}

// Upsert the last solved geometry and fixed-step impulse totals by canonical pair id.
void PhysicsWorld::PublishTouchingContacts(const std::vector<uint32_t>& touchingSlots) {
    Midphase& midphase = m_collisionDetector.GetMidphase();
    for (const uint32_t slotIndex : touchingSlots) {
        const MidphasePair& pair = midphase.PairAt(slotIndex);
        ContactManifold published = pair.manifold;
        published.fixedStepNormalImpulse = pair.fixedStepNormalImpulse;
        published.fixedStepTangentImpulse = pair.fixedStepTangentImpulse;
        const auto it = m_stepContactIndex.find(pair.key);
        if (it == m_stepContactIndex.end()) {
            m_stepContactIndex.emplace(pair.key, m_contacts.size());
            m_contacts.push_back(std::move(published));
        } else {
            m_contacts[it->second] = std::move(published);
        }
    }
}

// Prepare island-local rows and keep cache units fixed across incremental TOI solves.
void PhysicsWorld::PrepareIslandConstraints(const std::vector<uint32_t>& touchingSlots, float substepDt, bool warmStart,
    const ContactManifold* impactContact) {
    Midphase& midphase = m_collisionDetector.GetMidphase();
    m_preparedConstraints.clear();
    const std::vector<PhysicsIsland>& islands = m_islandBuilder.Islands();
    m_islandConstraintRanges.clear();
    m_islandConstraintRanges.resize(islands.size());
    m_islandSchedule.clear();

    // Solve high contacts first and low supports last: the last Gauss-Seidel
    // rows dominate the residual, so landing loads must not undo the floor
    // constraint at the end of every iteration. Stable ties retain PairKey order.
    for (std::size_t islandIndex = 0; islandIndex < islands.size(); ++islandIndex) {
        const auto& members = islands[islandIndex].bodies;
        const bool affected = !impactContact ||
            std::find(members.begin(), members.end(), impactContact->bodyA) != members.end() ||
            std::find(members.begin(), members.end(), impactContact->bodyB) != members.end();
        IslandConstraintRange range;
        range.begin = m_preparedConstraints.size();
        m_islandContactOrder = islands[islandIndex].contacts;
        if (glm::dot(m_gravity, m_gravity) > 1e-10f) {
            // Project the patch centroid onto gravity to order upper contacts first.
            const auto height = [&](uint32_t contactIndex) {
                const ContactManifold& contact = midphase.PairAt(touchingSlots[contactIndex]).manifold;
                if (contact.pointCount == 0)
                    throw std::logic_error("A touching manifold must contain a contact point");
                glm::vec3 center(0.0f);
                for (std::size_t p = 0; p < contact.pointCount; ++p) center += contact.Point(p).position;
                return glm::dot(center / static_cast<float>(contact.pointCount), m_gravity);
            };
            std::stable_sort(m_islandContactOrder.begin(), m_islandContactOrder.end(),
                [&](uint32_t a, uint32_t b) { return height(a) < height(b); });
        }
        for (const uint32_t contactIndex : m_islandContactOrder) {
            // Island contact indices map directly to the detection's touching slots.
            const uint32_t slotIndex = touchingSlots[contactIndex];
            ContactManifold& contact = midphase.PairAt(slotIndex).manifold;
            auto bodyItA = m_bodies.find(contact.bodyA);
            auto bodyItB = m_bodies.find(contact.bodyB);
            auto colliderItA = m_colliders.find(contact.bodyA);
            auto colliderItB = m_colliders.find(contact.bodyB);
            if (bodyItA == m_bodies.end() || bodyItB == m_bodies.end() ||
                colliderItA == m_colliders.end() || colliderItB == m_colliders.end()) {
                continue;
            }

            m_preparedConstraints.emplace_back();
            m_contactSolver.Prepare(
                m_preparedConstraints.back(),
                slotIndex,
                contact,
                bodyItA->second,
                bodyItB->second,
                colliderItA->second,
                colliderItB->second,
                substepDt,
                m_gravity);
            // TOI velocities already contain previous impulses; keep them only as
            // cache bases: lambdaBase = lambdaCached * fixedDt / cachedDt.
            if (!warmStart) {
                PreparedContactConstraint& prepared = m_preparedConstraints.back();
                const bool solvedThisStep = midphase.PairAt(slotIndex).lastSolvedFixedStepId == m_fixedStepId;
                for (std::size_t point = 0; point < prepared.pointCount; ++point) {
                    const ContactPoint& cached = contact.Point(point);
                    if (solvedThisStep && cached.cachedDt > 0.0f)
                        prepared.points[point].cacheBaseNormal =
                            cached.accumulatedNormalImpulse * (m_fixedTimeStep / cached.cachedDt);
                    prepared.points[point].accumulatedNormal = 0.0f;
                }
                if (solvedThisStep && contact.Point(0).cachedDt > 0.0f) {
                    const ContactPoint& cached = contact.Point(0);
                    const glm::vec3 tangent = cached.cachedTangent1 * cached.accumulatedTangentImpulse.x +
                        cached.cachedTangent2 * cached.accumulatedTangentImpulse.y;
                    const float scale = m_fixedTimeStep / cached.cachedDt;
                    prepared.cacheBaseTangent = scale * glm::vec2(
                        glm::dot(tangent, prepared.tangent1), glm::dot(tangent, prepared.tangent2));
                    prepared.cacheBaseSpin = scale * cached.accumulatedSpinImpulse;
                }
                prepared.accumulatedTangent = glm::vec2(0.0f);
                prepared.accumulatedSpin = 0.0f;
            }
            m_preparedConstraints.back().cacheDt = m_fixedTimeStep;
            m_preparedConstraints.back().velocitySolveEnabled = affected;
        }
        range.end = m_preparedConstraints.size();
        m_islandConstraintRanges[islandIndex] = range;
        if (range.end > range.begin) {
            m_islandSchedule.push_back(islandIndex);
        }
    }

    // Largest islands first; accumulate whole islands until each job reaches its target.
    std::stable_sort(
        m_islandSchedule.begin(),
        m_islandSchedule.end(),
        [this](std::size_t a, std::size_t b) {
            const std::size_t countA =
                m_islandConstraintRanges[a].end - m_islandConstraintRanges[a].begin;
            const std::size_t countB =
                m_islandConstraintRanges[b].end - m_islandConstraintRanges[b].begin;
            return countA > countB;
        });
    m_islandJobs.clear();
    const std::size_t minPerJob = std::max<std::size_t>(1, m_settings.islandSolverMinConstraintsPerJob);
    std::size_t jobBegin = 0;
    while (jobBegin < m_islandSchedule.size()) {
        std::size_t constraintCount = 0;
        std::size_t jobEnd = jobBegin;
        while (jobEnd < m_islandSchedule.size() && (constraintCount < minPerJob || jobEnd == jobBegin)) {
            const IslandConstraintRange range = m_islandConstraintRanges[m_islandSchedule[jobEnd]];
            constraintCount += range.end - range.begin;
            ++jobEnd;
        }
        m_islandJobs.push_back({jobBegin, jobEnd});
        jobBegin = jobEnd;
    }

#ifndef NDEBUG
    // Build ownership indices before dispatch; tasks stamp only their own bodies.
    m_islandBodyIndex.clear();
    for (std::size_t index = 0; index < m_dynamicBodyIds.size(); ++index) {
        m_islandBodyIndex.emplace(m_dynamicBodyIds[index], static_cast<uint32_t>(index));
    }
    if (m_islandBodyClaimCapacity < m_dynamicBodyIds.size()) {
        m_islandBodyClaims = std::make_unique<std::atomic<uint32_t>[]>(m_dynamicBodyIds.size());
        m_islandBodyClaimCapacity = m_dynamicBodyIds.size();
    }
#endif
}

// Solve affected islands with exclusive body ownership, then merge worker timings.
void PhysicsWorld::SolveIslandConstraints() {
    using Clock = std::chrono::steady_clock;
    const auto solveStart = Clock::now();
    const std::vector<PhysicsIsland>& islands = m_islandBuilder.Islands();
    const std::size_t jobCount = m_islandJobs.size();
    const int iterations = m_solverIterations;

    // Reserve spare records for participant callbacks, including empty final ranges.
    m_islandJobRecords.clear();
    m_islandJobRecords.resize(
        jobCount + 2 + std::max<uint32_t>(1, Core::JobSystem::Get().WorkerCount()));
    std::atomic<uint32_t> claimedRecords{0};
#ifndef NDEBUG
    const uint32_t claimTicket = static_cast<uint32_t>(m_queryEpoch) + 1u;
#endif

    // Run each job's islands in fixed row order; shared static bodies are read-only.
    const auto processJobs = [&](std::size_t begin, std::size_t end) {
        const uint32_t recordIndex = claimedRecords.fetch_add(1, std::memory_order_relaxed);
        IslandJobRecord& record = m_islandJobRecords[recordIndex];
        record.threadId = std::this_thread::get_id();
        record.startMilliseconds =
            std::chrono::duration<double, std::milli>(Clock::now() - solveStart).count();

        for (std::size_t job = begin; job < end; ++job) {
            for (std::size_t entry = m_islandJobs[job].scheduleBegin;
                 entry < m_islandJobs[job].scheduleEnd; ++entry) {
                const std::size_t islandIndex = m_islandSchedule[entry];
                const IslandConstraintRange range = m_islandConstraintRanges[islandIndex];
                // A TOI changes only its connected island; all islands still get the final NGS pass.
                if (!m_preparedConstraints[range.begin].velocitySolveEnabled) continue;
#ifndef NDEBUG
                for (const uint32_t bodyId : islands[islandIndex].bodies) {
                    const uint32_t claimIndex = m_islandBodyIndex.find(bodyId)->second;
                    const uint32_t previous = m_islandBodyClaims[claimIndex].exchange(
                        claimTicket, std::memory_order_relaxed);
                    assert(previous != claimTicket &&
                           "two island solver tasks write the same dynamic body");
                    (void)previous;
                }
#endif
                for (std::size_t index = range.begin; index < range.end; ++index) {
                    ContactSolver::WarmStart(m_preparedConstraints[index]);
                }
                for (int iteration = 0; iteration < iterations; ++iteration) {
                    for (std::size_t index = range.begin; index < range.end; ++index) {
                        ContactSolver::SolveVelocityIteration(m_preparedConstraints[index]);
                    }
                }
            }
        }

        record.endMilliseconds =
            std::chrono::duration<double, std::milli>(Clock::now() - solveStart).count();
    };

    Core::JobSystem& jobSystem = Core::JobSystem::Get();
    if (m_settings.parallelIslandSolverEnabled && jobSystem.WorkerCount() > 1 && jobCount > 0) {
        // One job per chunk: batching already happened at schedule time.
        jobSystem.ParallelForRange(jobCount, 1, processJobs);
    } else if (jobCount > 0) {
        // Serial deterministic fallback: run every job in-line.
        processJobs(0, jobCount);
    }

    // Merge task-private chunk records on this thread (post barrier).
    const uint32_t recordsUsed = claimedRecords.load(std::memory_order_relaxed);
    const double barrierMilliseconds =
        std::chrono::duration<double, std::milli>(Clock::now() - solveStart).count();
    std::vector<std::thread::id> threadIds;
    std::vector<double> busyPerThread;
    std::vector<double> lastEndPerThread;
    for (uint32_t record = 0; record < recordsUsed; ++record) {
        const IslandJobRecord& entry = m_islandJobRecords[record];
        std::size_t threadIndex = threadIds.size();
        for (std::size_t index = 0; index < threadIds.size(); ++index) {
            if (threadIds[index] == entry.threadId) {
                threadIndex = index;
                break;
            }
        }
        if (threadIndex == threadIds.size()) {
            threadIds.push_back(entry.threadId);
            busyPerThread.push_back(0.0);
            lastEndPerThread.push_back(0.0);
        }
        busyPerThread[threadIndex] += entry.endMilliseconds - entry.startMilliseconds;
        if (entry.endMilliseconds > lastEndPerThread[threadIndex]) {
            lastEndPerThread[threadIndex] = entry.endMilliseconds;
        }
    }
    // Sum work and maximum per-pass tail wait; retain maximum participant count.
    m_lastStepStats.islandSolverJobCount += recordsUsed;
    m_lastStepStats.islandSolverWorkerCount = std::max<std::size_t>(
        m_lastStepStats.islandSolverWorkerCount, threadIds.size());
    double substepTailWait = 0.0;
    for (std::size_t index = 0; index < threadIds.size(); ++index) {
        m_lastStepStats.islandSolverWorkerBusyMilliseconds += busyPerThread[index];
        substepTailWait = std::max(substepTailWait, barrierMilliseconds - lastEndPerThread[index]);
    }
    m_lastStepStats.islandSolverTailWaitMilliseconds += substepTailWait;
}

// Commit cache totals and account for each newly applied impulse exactly once.
void PhysicsWorld::CommitSolvedContacts() {
    Midphase& midphase = m_collisionDetector.GetMidphase();
    for (PreparedContactConstraint& constraint : m_preparedConstraints) {
        if (!constraint.velocitySolveEnabled) continue;
        ContactSolver::CommitSolvedImpulses(constraint);
        // Jstep += sum(lambdaNormal); Jtangent += t1 * lambda1 + t2 * lambda2.
        // Accumulate each solve's applied impulse, not every iteration's lambda.
        MidphasePair& pair = midphase.PairAt(constraint.slotIndex);
        pair.lastSolvedFixedStepId = m_fixedStepId;
        for (std::size_t index = 0; index < constraint.pointCount; ++index) {
            pair.fixedStepNormalImpulse += constraint.points[index].accumulatedNormal;
        }
        pair.fixedStepTangentImpulse +=
            constraint.tangent1 * constraint.accumulatedTangent.x +
            constraint.tangent2 * constraint.accumulatedTangent.y;
    }
}

// Partition awake dynamic bodies by non-trigger contacts and record the latest sizes.
void PhysicsWorld::BuildIslands(const std::vector<uint32_t>& touchingSlots) {
    const auto start = std::chrono::steady_clock::now();
    m_dynamicBodyIds.clear();
    for (const auto& entry : m_bodies) {
        // Sleepers retain persistent adjacency but have no solver work.
        if (!entry.second.IsStatic() && !entry.second.IsSleeping()) {
            m_dynamicBodyIds.push_back(entry.first);
        }
    }
    m_islandContacts.clear();
    const Midphase& midphase = m_collisionDetector.GetMidphase();
    for (const uint32_t slotIndex : touchingSlots) {
        const ContactManifold& manifold = midphase.PairAt(slotIndex).manifold;
        m_islandContacts.push_back({manifold.bodyA, manifold.bodyB, manifold.isTrigger});
    }
    m_islandBuilder.Build(m_dynamicBodyIds, m_islandContacts);
    // Sizes are snapshots; build time accumulates across queries.
    const IslandBuildStats& stats = m_islandBuilder.LastStats();
    m_lastStepStats.islandCount = stats.islandCount;
    m_lastStepStats.islandMaxBodyCount = stats.maxIslandBodyCount;
    m_lastStepStats.islandBuildMilliseconds +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

// Convert pending body activity into full-island wakes before force integration.
void PhysicsWorld::ProcessExternalActivityWakes() {
    if (!m_settings.sleepEnabled) {
        return;
    }
    for (auto& entry : m_bodies) {
        if (!entry.second.HasExternalActivity()) {
            continue;
        }
        if (entry.second.IsStatic()) {
            WakeStaticAdjacency(entry.first);
        } else {
            WakeIslandContaining(entry.first, 0.0f);
        }
    }
}

// Traverse persistent dynamic contact edges and compensate each newly woken sleeper once.
bool PhysicsWorld::WakeIslandContaining(uint32_t bodyId, float compensateVelocityDt) {
    const Midphase& midphase = m_collisionDetector.GetMidphase();
    bool wokeAny = false;
    m_wakeVisited.clear();
    m_wakeStack.clear();
    m_wakeStack.push_back(bodyId);
    while (!m_wakeStack.empty()) {
        const uint32_t current = m_wakeStack.back();
        m_wakeStack.pop_back();
        if (!m_wakeVisited.insert(current).second) {
            continue;
        }
        const auto bodyIt = m_bodies.find(current);
        if (bodyIt == m_bodies.end() || bodyIt->second.IsStatic()) {
            continue;
        }
        RigidBody& body = bodyIt->second;
        if (body.IsSleeping()) {
            body.WakeUp();
            wokeAny = true;
            if (compensateVelocityDt > 0.0f) {
                body.IntegrateVelocity(compensateVelocityDt, m_gravity);
            }
        }
        // Follow retained non-trigger contacts even through already-awake members.
        // Enqueue unvisited dynamic neighbors of the current body.
        midphase.ForEachPair([&](const MidphasePair& pair) {
            if (!pair.hadContact || pair.manifold.isTrigger) {
                return;
            }
            uint32_t other = 0;
            if (pair.key.bodyA == current) {
                other = pair.key.bodyB;
            } else if (pair.key.bodyB == current) {
                other = pair.key.bodyA;
            } else {
                return;
            }
            const auto otherIt = m_bodies.find(other);
            if (otherIt != m_bodies.end() && !otherIt->second.IsStatic() &&
                m_wakeVisited.find(other) == m_wakeVisited.end()) {
                m_wakeStack.push_back(other);
            }
        });
    }
    return wokeAny;
}

// Wake static-linked sleepers and sleeping dynamic leaves overlapping current static bounds.
void PhysicsWorld::WakeStaticAdjacency(uint32_t staticBodyId) {
    const Midphase& midphase = m_collisionDetector.GetMidphase();
    // Persistent adjacency covers moved or removed supports.
    midphase.ForEachPair([&](const MidphasePair& pair) {
        uint32_t other = 0;
        if (pair.key.bodyA == staticBodyId) {
            other = pair.key.bodyB;
        } else if (pair.key.bodyB == staticBodyId) {
            other = pair.key.bodyA;
        } else {
            return;
        }
        const auto otherIt = m_bodies.find(other);
        if (otherIt != m_bodies.end() && !otherIt->second.IsStatic() &&
            otherIt->second.IsSleeping()) {
            WakeIslandContaining(other, 0.0f);
        }
    });

    // Region overlap also finds sleepers without a pre-existing static pair.
    const auto bodyIt = m_bodies.find(staticBodyId);
    const auto colliderIt = m_colliders.find(staticBodyId);
    if (bodyIt == m_bodies.end() || colliderIt == m_colliders.end()) {
        return;
    }
    ShapeTransform transform;
    transform.position = bodyIt->second.Position();
    transform.orientation = bodyIt->second.Orientation();
    const AABB region = colliderIt->second.ComputeAABB(transform);
    m_wakeRegionResults.clear();
    m_collisionDetector.CollectDynamicLeafOverlaps(region, m_bodies, m_wakeRegionResults);
    for (const uint32_t dynamicId : m_wakeRegionResults) {
        const auto dynamicIt = m_bodies.find(dynamicId);
        if (dynamicIt != m_bodies.end() && dynamicIt->second.IsSleeping()) {
            WakeIslandContaining(dynamicId, 0.0f);
        }
    }
}

// Sleep an entire island only after its shape-sized motion probes remain bounded for the rest window.
void PhysicsWorld::EvaluateIslandSleep(float fixedDt) {
    if (!m_settings.sleepEnabled) {
        return;
    }

    // Contact instability of this fixed step: any Enter/Exit resets the
    // owning island (a contact must persist quietly before it can sleep).
    m_stepTransitionPairs.clear();
    for (const ContactEvent& event : m_contactEvents) {
        if (event.fixedStepId == m_fixedStepId && event.type != ContactEventType::Stay) {
            m_stepTransitionPairs.insert(event.pair);
        }
    }

	// Shape-sized probes catch angular drift without using instantaneous speed.
	const float maxDisplacement = m_settings.sleepMaxDisplacement;
	for (const PhysicsIsland& island : m_islandBuilder.Islands()) {
		bool restful = true;
		for (const uint32_t bodyId : island.bodies) {
			RigidBody& body = m_bodies.find(bodyId)->second;
            float probeArm = 0.5f;
            const auto collider = m_colliders.find(bodyId);
            if (collider != m_colliders.end()) {
                const AABB bounds = collider->second.ComputeAABB(ShapeTransform{});
                const glm::vec3 extent = glm::max(glm::abs(bounds.min), glm::abs(bounds.max));
                probeArm = std::max({extent.x, extent.y, extent.z});
            }
			if (!body.AllowSleep() || body.HasExternalActivity() ||
				!body.SleepDisplacementWithin(maxDisplacement, probeArm)) {
				restful = false;
				break;
			}
		}
		if (restful) {
			for (const uint32_t contactIndex : island.contacts) {
				const IslandContact& contact = m_islandContacts[contactIndex];
				if (m_stepTransitionPairs.find(MakePairKey(contact.bodyA, contact.bodyB)) !=
					m_stepTransitionPairs.end()) {
					restful = false;
					break;
				}
			}
		}

		if (!restful) {
			for (const uint32_t bodyId : island.bodies) {
				RigidBody& body = m_bodies.find(bodyId)->second;
				body.ResetSleepTimer();
				body.ResetSleepDisplacement();
			}
			continue;
		}
        bool allReady = !island.bodies.empty();
        for (const uint32_t bodyId : island.bodies) {
            RigidBody& body = m_bodies.find(bodyId)->second;
            body.AdvanceSleepTimer(fixedDt);
            // Sleep only when min(member rest timers) >= sleepTimeThreshold.
            if (body.SleepTimer() < m_settings.sleepTimeThreshold) {
                allReady = false;
            }
        }
        if (allReady) {
            for (const uint32_t bodyId : island.bodies) {
                // Zero residual motion without removing contact adjacency or publishing Exit.
                m_bodies.find(bodyId)->second.EnterSleep();
            }
        }
    }
}

} // namespace Physics
} // namespace Runtime
