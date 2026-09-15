#include "Physics/PhysicsWorld.h"

#include <algorithm>
#include <chrono>
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
    if (!result.touchingPairs.empty()) {
        const auto solverStart = std::chrono::steady_clock::now();
        // Prepare once per sub-step, warm start once, then N accumulated
        // incremental velocity iterations and a single position pass.
        PrepareContactConstraints(result.touchingPairs, substepDt);
        for (PreparedContactConstraint& constraint : m_preparedConstraints) {
            ContactSolver::WarmStart(constraint);
        }
        for (int iteration = 0; iteration < m_solverIterations; ++iteration) {
            for (PreparedContactConstraint& constraint : m_preparedConstraints) {
                ContactSolver::SolveVelocityIteration(constraint);
            }
            ++m_lastStepStats.velocitySolverPassCount;
        }
        ResolvePositionContacts(result.touchingPairs);
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

void PhysicsWorld::PrepareContactConstraints(const std::vector<uint32_t>& touchingSlots, float substepDt) {
    Midphase& midphase = m_collisionDetector.GetMidphase();
    m_preparedConstraints.clear();
    for (const uint32_t slotIndex : touchingSlots) {
        ContactManifold& contact = midphase.PairAt(slotIndex).manifold;
        auto bodyItA = m_bodies.find(contact.bodyA);
        auto bodyItB = m_bodies.find(contact.bodyB);
        auto colliderItA = m_colliders.find(contact.bodyA);
        auto colliderItB = m_colliders.find(contact.bodyB);
        if (bodyItA == m_bodies.end() || bodyItB == m_bodies.end() ||
            colliderItA == m_colliders.end() || colliderItB == m_colliders.end()) {
            continue;
        }
        // Trigger and static-static contacts are never solved; their impulse
        // fields stay zero.
        if (contact.isTrigger || (bodyItA->second.IsStatic() && bodyItB->second.IsStatic())) {
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

void PhysicsWorld::ResolvePositionContacts(const std::vector<uint32_t>& touchingSlots) {
    Midphase& midphase = m_collisionDetector.GetMidphase();
    for (const uint32_t slotIndex : touchingSlots) {
        ContactManifold& contact = midphase.PairAt(slotIndex).manifold;
        auto bodyItA = m_bodies.find(contact.bodyA);
        auto bodyItB = m_bodies.find(contact.bodyB);
        if (bodyItA == m_bodies.end() || bodyItB == m_bodies.end()) {
            continue;
        }
        m_contactSolver.ResolvePosition(contact, bodyItA->second, bodyItB->second);
    }
}

} // namespace Physics
} // namespace Runtime
