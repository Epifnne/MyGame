#include "Physics/PhysicsWorld.h"

#include <algorithm>
#include <cassert>
#include <atomic>
#include <chrono>
#include <thread>
#include <utility>

#include "Core/JobSystem.h"

namespace Runtime {
namespace Physics {

PhysicsWorld::PhysicsWorld()
    : m_integrator(std::make_unique<SemiImplicitEulerIntegrator>()) {
    // Apply the default settings to the shared fixed job pool (0 = hardware
    // concurrency) and the collision detector.
    Core::JobSystem::Get().Initialize(m_settings.workerCount);
    m_collisionDetector.SetParallelNarrowphaseEnabled(m_settings.parallelNarrowphaseEnabled);
    m_collisionDetector.SetNarrowphaseMinPairsPerJob(m_settings.narrowphaseMinPairsPerJob);
}

void PhysicsWorld::SetPhysicsWorkerCount(uint32_t workerCount) {
    m_settings.workerCount = workerCount;
    Core::JobSystem::Get().Initialize(workerCount);
}

uint32_t PhysicsWorld::PhysicsWorkerCount() const {
    return Core::JobSystem::Get().WorkerCount();
}

void PhysicsWorld::SetParallelNarrowphaseEnabled(bool enabled) {
    m_settings.parallelNarrowphaseEnabled = enabled;
    m_collisionDetector.SetParallelNarrowphaseEnabled(enabled);
}

void PhysicsWorld::SetParallelIslandSolverEnabled(bool enabled) {
    m_settings.parallelIslandSolverEnabled = enabled;
}

uint32_t PhysicsWorld::CreateRigidBody(const RigidBodyDesc& desc) {
    const uint32_t id = m_nextBodyId++;
    RigidBody body(desc);
    body.SetId(id);
    m_bodies[id] = body;
    return id;
}

bool PhysicsWorld::DestroyRigidBody(uint32_t bodyId) {
    const bool bodyRemoved = m_bodies.erase(bodyId) > 0;
    m_colliders.erase(bodyId);
    return bodyRemoved;
}

bool PhysicsWorld::HasRigidBody(uint32_t bodyId) const {
    return m_bodies.find(bodyId) != m_bodies.end();
}

RigidBody* PhysicsWorld::GetRigidBody(uint32_t bodyId) {
    auto it = m_bodies.find(bodyId);
    if (it == m_bodies.end()) {
        return nullptr;
    }
    return &it->second;
}

const RigidBody* PhysicsWorld::GetRigidBody(uint32_t bodyId) const {
    auto it = m_bodies.find(bodyId);
    if (it == m_bodies.end()) {
        return nullptr;
    }
    return &it->second;
}

bool PhysicsWorld::AttachCollider(uint32_t bodyId, const ColliderDesc& desc) {
    auto bodyIt = m_bodies.find(bodyId);
    if (bodyIt == m_bodies.end() || !desc.shape) {
        return false;
    }

    Collider collider(desc);
    collider.SetBodyId(bodyId);
    // Fresh identity per attach: a replacement collider is always
    // distinguishable by the midphase, even if revisions coincided.
    collider.SetIdentity(m_nextColliderIdentity++);
    m_colliders[bodyId] = std::move(collider);
    return true;
}

bool PhysicsWorld::RemoveCollider(uint32_t bodyId) {
    return m_colliders.erase(bodyId) > 0;
}

bool PhysicsWorld::HasCollider(uint32_t bodyId) const {
    return m_colliders.find(bodyId) != m_colliders.end();
}

Collider* PhysicsWorld::GetCollider(uint32_t bodyId) {
    auto it = m_colliders.find(bodyId);
    if (it == m_colliders.end()) {
        return nullptr;
    }
    return &it->second;
}

const Collider* PhysicsWorld::GetCollider(uint32_t bodyId) const {
    auto it = m_colliders.find(bodyId);
    if (it == m_colliders.end()) {
        return nullptr;
    }
    return &it->second;
}

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
    m_lastStepStats.totalMilliseconds =
        std::chrono::duration<double, std::milli>(Clock::now() - stepStart).count();
}

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

void PhysicsWorld::IntegrateBodies(float dt) {
    if (dt <= 0.0f || !m_integrator) {
        return;
    }
    const auto start = std::chrono::steady_clock::now();
    for (auto& kv : m_bodies) {
        m_integrator->Integrate(kv.second, dt, m_gravity);
    }
    m_lastStepStats.integrationMilliseconds +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

void PhysicsWorld::DetectAndSolveContacts(float substepDt, bool isToiSubstep) {
    const CollisionDetectionResult& result = m_collisionDetector.Detect(
        m_colliders, m_bodies, m_fixedStepId, ++m_queryEpoch, substepDt, isToiSubstep);
    AccumulateDetectionStats(result);
    BuildIslands(result.touchingPairs);
    if (!result.touchingPairs.empty()) {
        const auto solverStart = std::chrono::steady_clock::now();
        // Prepare once per sub-step (grouped by island), then per island:
        // warm start once, N accumulated incremental velocity iterations and
        // a single position pass. Independent islands solve as job-system
        // tasks; the commit below runs on this thread in stable order.
        PrepareIslandConstraints(result.touchingPairs, substepDt);
        SolveIslandConstraints();
        m_lastStepStats.velocitySolverPassCount += static_cast<uint64_t>(m_solverIterations);
        ++m_lastStepStats.positionSolverPassCount;
        CommitSolvedContacts();
        m_lastStepStats.solverMilliseconds +=
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - solverStart).count();
    }
    PublishTouchingContacts(result.touchingPairs);
}

void PhysicsWorld::FixedStep(float dt) {
    if (!m_integrator) {
        return;
    }

    ++m_fixedStepId;
    // The fixed-step contact summary is rebuilt per step; pairs that touched in
    // any sub-step of this fixed step are upserted by PairKey.
    m_contacts.clear();
    m_stepContactIndex.clear();
    if (!m_enableCcd) {
        IntegrateBodies(dt);
        DetectAndSolveContacts(dt, false);
    } else {
        float remaining = dt;
        int subStep = 0;

        while (remaining > 1e-6f && subStep < m_ccdMaxSubSteps) {
            const TimeOfImpact toi = m_continuousCollision.FindEarliestImpact(m_colliders, m_bodies, remaining);

            float advance = remaining;
            if (toi.hit) {
                advance = std::clamp(toi.toi, 0.0f, remaining);
                if (advance < 1e-5f) {
                    advance = std::min(remaining, 1e-4f);
                }
            }

            IntegrateBodies(advance);
            remaining -= advance;

            DetectAndSolveContacts(advance, toi.hit);

            if (!toi.hit) {
                break;
            }

            ++subStep;
        }

        if (remaining > 1e-6f) {
            IntegrateBodies(remaining);
            DetectAndSolveContacts(remaining, false);
        }
    }

    // Publish the fixed-step summary in stable PairKey order.
    std::sort(m_contacts.begin(), m_contacts.end(), [](const ContactManifold& a, const ContactManifold& b) {
        return MakePairKey(a.bodyA, a.bodyB) < MakePairKey(b.bodyA, b.bodyB);
    });

    // External forces/torques are locked when the fixed step starts: every sub-step
    // integrates the same locked forces with its own dt, and they are cleared once
    // here at fixed-step end. Forces accumulated before Step() are consumed only by
    // the first actual fixed step; instantaneous impulses stay single-application.
    for (auto& entry : m_bodies) {
        entry.second.ClearForces();
    }
}

void PhysicsWorld::AccumulateDetectionStats(const CollisionDetectionResult& result) {
    const CollisionDetectionStats& detectionStats = m_collisionDetector.LastStats();
    ++m_lastStepStats.collisionDetectionPassCount;
    m_lastStepStats.staticBvhLeafCount = detectionStats.staticLeafCount;
    m_lastStepStats.dynamicBvhLeafCount = detectionStats.dynamicLeafCount;
    m_lastStepStats.broadPhaseCandidateCount += detectionStats.broadPhaseCandidateCount;
    m_lastStepStats.narrowPhaseTestCount += detectionStats.narrowPhaseTestCount;
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

    // Phase 6 parallel narrow-phase telemetry: job/busy/tail are per-query
    // addends, worker participation is a snapshot maximum.
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

void PhysicsWorld::PublishTouchingContacts(const std::vector<uint32_t>& touchingSlots) {
    Midphase& midphase = m_collisionDetector.GetMidphase();
    for (const uint32_t slotIndex : touchingSlots) {
        const MidphasePair& pair = midphase.PairAt(slotIndex);
        // The published snapshot carries the fixed-step net impulse totals
        // (warm start plus subsequent deltas of every sub-step this step);
        // geometry and normalImpulse come from the last touching sub-step.
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

void PhysicsWorld::PrepareIslandConstraints(const std::vector<uint32_t>& touchingSlots, float substepDt) {
    Midphase& midphase = m_collisionDetector.GetMidphase();
    m_preparedConstraints.clear();
    const std::vector<PhysicsIsland>& islands = m_islandBuilder.Islands();
    m_islandConstraintRanges.clear();
    m_islandConstraintRanges.resize(islands.size());
    m_islandSchedule.clear();

    // Constraints are stored island by island in the builder's stable island
    // order. Within an island the contacts keep their stable work-list
    // (PairKey) order, so the Gauss-Seidel sequence inside each island is
    // identical to the previous flat serial order; islands share no writable
    // body, so the cross-island execution order cannot change the result.
    for (std::size_t islandIndex = 0; islandIndex < islands.size(); ++islandIndex) {
        IslandConstraintRange range;
        range.begin = m_preparedConstraints.size();
        for (const uint32_t contactIndex : islands[islandIndex].contacts) {
            // Island contacts index the BuildIslands contact array, which is
            // parallel to the detection result's touching slot list. The
            // builder already dropped trigger and static-static contacts, so
            // every island contact is a solver-relevant constraint.
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
                substepDt);
        }
        range.end = m_preparedConstraints.size();
        m_islandConstraintRanges[islandIndex] = range;
        if (range.end > range.begin) {
            m_islandSchedule.push_back(islandIndex);
        }
    }

    // Scheduling: islands with more constraints first (stable ties keep the
    // builder order); an island reaching the configured minimum becomes a
    // dedicated job, smaller islands are batched until the job reaches it.
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
    // Acceptance gate setup: the body-ownership stamp table is rebuilt on
    // this thread before dispatch; tasks only stamp their own bodies.
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

void PhysicsWorld::SolveIslandConstraints() {
    using Clock = std::chrono::steady_clock;
    const auto solveStart = Clock::now();
    Midphase& midphase = m_collisionDetector.GetMidphase();
    const std::vector<PhysicsIsland>& islands = m_islandBuilder.Islands();
    const std::size_t jobCount = m_islandJobs.size();
    const int iterations = m_solverIterations;

    // Chunk record slots are claimed atomically. Claims exceed the actual
    // chunk count (one final out-of-range claim per participant, same as the
    // parallel narrow-phase), so size for jobCount chunks plus participants.
    m_islandJobRecords.clear();
    m_islandJobRecords.resize(
        jobCount + 2 + std::max<uint32_t>(1, Core::JobSystem::Get().WorkerCount()));
    std::atomic<uint32_t> claimedRecords{0};
#ifndef NDEBUG
    const uint32_t claimTicket = static_cast<uint32_t>(m_queryEpoch) + 1u;
#endif

    // Each task exclusively owns its islands' dynamic bodies. Shared static
    // bodies are never written by the solver (the impulse entries are
    // static-guarded and position correction skips statics), so no
    // synchronization beyond the barrier is needed. The per-island
    // constraint order is fixed, keeping results bit-identical for any
    // worker count.
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
                for (std::size_t index = range.begin; index < range.end; ++index) {
                    PreparedContactConstraint& constraint = m_preparedConstraints[index];
                    m_contactSolver.ResolvePosition(
                        midphase.PairAt(constraint.slotIndex).manifold,
                        *constraint.bodyA,
                        *constraint.bodyB);
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
    // Job count and busy time are per-sub-step addends, worker participation
    // is a snapshot maximum, tail wait adds each sub-step's maximum (the same
    // accounting as the parallel narrow-phase telemetry).
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

void PhysicsWorld::CommitSolvedContacts() {
    Midphase& midphase = m_collisionDetector.GetMidphase();
    for (PreparedContactConstraint& constraint : m_preparedConstraints) {
        ContactSolver::CommitSolvedImpulses(constraint);
        // Fixed-step totals: each sub-step contributes its final accumulated
        // lambda (warm start plus deltas), never a per-iteration sum.
        MidphasePair& pair = midphase.PairAt(constraint.slotIndex);
        for (std::size_t index = 0; index < constraint.pointCount; ++index) {
            const PreparedContactPoint& point = constraint.points[index];
            pair.fixedStepNormalImpulse += point.accumulatedNormal;
            pair.fixedStepTangentImpulse +=
                constraint.tangent1 * point.accumulatedTangent.x +
                constraint.tangent2 * point.accumulatedTangent.y;
        }
    }
}

void PhysicsWorld::BuildIslands(const std::vector<uint32_t>& touchingSlots) {
    const auto start = std::chrono::steady_clock::now();
    m_dynamicBodyIds.clear();
    for (const auto& entry : m_bodies) {
        if (!entry.second.IsStatic()) {
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
    // Island counts are absolute snapshots of the latest sub-step's build
    // (like the pair pool sizes), the build time is a per-sub-step addend.
    const IslandBuildStats& stats = m_islandBuilder.LastStats();
    m_lastStepStats.islandCount = stats.islandCount;
    m_lastStepStats.islandMaxBodyCount = stats.maxIslandBodyCount;
    m_lastStepStats.islandBuildMilliseconds +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

} // namespace Physics
} // namespace Runtime
