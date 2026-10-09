#include "Physics/CollisionDetector.h"

#include <chrono>
#include <utility>

#include "Core/JobSystem.h"

namespace Runtime {
namespace Physics {

// Install split-tree broad phase and analytic/SAT/GJK/EPA narrow phase.
CollisionDetector::CollisionDetector()
    : m_broadPhase(std::make_unique<HybridBvhBroadPhase>()),
      m_narrowPhase(std::make_unique<GjkEpaNarrowPhase>()) {}

// Destroy owned stages, pair storage, and reusable work buffers.
CollisionDetector::~CollisionDetector() = default;

// Repeat detection only to close same-pose wake propagation, preserving the supplied TOI geometry.
const CollisionDetectionResult& CollisionDetector::Detect(
    const std::unordered_map<uint32_t, Collider>& colliders,
    const std::unordered_map<uint32_t, RigidBody>& bodies,
    uint64_t fixedStepId,
    uint64_t queryEpoch,
    float substepDt,
    WakeRequestCallback wakeCallback, const ContactManifold* impactContact) {
    using Clock = std::chrono::steady_clock;

    m_lastStats = {};
    m_lastResult.touchingPairs.clear();
    m_lastResult.events.clear();
    if (!m_broadPhase || !m_narrowPhase) {
        return m_lastResult;
    }

    const std::size_t maxRounds = wakeCallback.invoke ? bodies.size() + 1 : 1;
    for (std::size_t round = 0; round < maxRounds; ++round) {
        const bool wokeAny = RunDetectionRound(
            colliders, bodies, fixedStepId, queryEpoch, substepDt, wakeCallback, impactContact);
        if (!wokeAny) {
            break;
        }
    }
    return m_lastResult;
}

// Update candidate lifetime before committing immutable narrow-phase work outputs.
bool CollisionDetector::RunDetectionRound(
    const std::unordered_map<uint32_t, Collider>& colliders,
    const std::unordered_map<uint32_t, RigidBody>& bodies,
    uint64_t fixedStepId,
    uint64_t queryEpoch,
    float substepDt,
    WakeRequestCallback wakeCallback, const ContactManifold* impactContact) {
    using Clock = std::chrono::steady_clock;
    (void)substepDt;
    // Only the final round's touching set matters: earlier rounds' pairs are
    // re-evaluated (geometry never moved) and re-committed by the last one.
    m_lastResult.touchingPairs.clear();

    const auto broadPhaseStart = Clock::now();
    const BroadPhaseQueryResult queryResult = m_broadPhase->ComputePairs(colliders, bodies);
    const auto broadPhaseEnd = Clock::now();
    m_lastStats.broadPhaseCandidateCount += queryResult.pairs.size();
    m_lastStats.broadPhaseMilliseconds +=
        std::chrono::duration<double, std::milli>(broadPhaseEnd - broadPhaseStart).count();

    // Sample leaf counts after the query synchronized the trees.
    const BroadPhaseLeafCounts leafCounts = m_broadPhase->GetLeafCounts();
    if (leafCounts.tracked) {
        // Report the synchronized split-tree membership.
        m_lastStats.staticLeafCount = leafCounts.staticLeaves;
        m_lastStats.dynamicLeafCount = leafCounts.dynamicLeaves;
    } else {
        // Legacy single-tree broad-phase keeps no per-class trees; fall back to
        // the logical collider/body classification baseline.
        for (const auto& entry : colliders) {
            const auto bodyIt = bodies.find(entry.first);
            if (bodyIt == bodies.end()) {
                continue;
            }
            if (bodyIt->second.IsStatic()) {
                ++m_lastStats.staticLeafCount;
            } else {
                ++m_lastStats.dynamicLeafCount;
            }
        }
    }

    m_midphase.BeginQuery(queryEpoch, fixedStepId, queryResult.coverage);
    for (const PairKey& pair : queryResult.pairs) {
        const auto colliderItA = colliders.find(pair.bodyA);
        const auto colliderItB = colliders.find(pair.bodyB);
        const auto bodyItA = bodies.find(pair.bodyA);
        const auto bodyItB = bodies.find(pair.bodyB);
        if (colliderItA == colliders.end() || colliderItB == colliders.end() ||
            bodyItA == bodies.end() || bodyItB == bodies.end()) {
            continue;
        }
        m_midphase.RegisterCandidate(
            pair,
            colliderItA->second,
            bodyItA->second,
            colliderItB->second,
            bodyItB->second);
    }
    m_midphase.FinishQuery(bodies, colliders, m_lastResult.events);
    const MidphaseStats& midphaseStats = m_midphase.LastStats();
    // Pair pool sizes are absolute snapshots (last round wins); new/removed
    // counts accumulate across wake rounds.
    m_lastStats.midphaseActivePairCount = midphaseStats.activePairCount;
    m_lastStats.midphaseNewPairCount += midphaseStats.newPairCount;
    m_lastStats.midphaseRemovedPairCount += midphaseStats.removedPairCount;

    // Stage 3: narrow-phase generates fresh geometry per work-list slot
    // (serially or across the job system); this thread then commits each
    // result into the persistent pair pool in stable PairKey order.
    return ExecuteNarrowphase(colliders, bodies, wakeCallback, impactContact);
}

// Query pairs in parallel, then serially publish geometry, wake requests, and cache transitions.
bool CollisionDetector::ExecuteNarrowphase(
    const std::unordered_map<uint32_t, Collider>& colliders,
    const std::unordered_map<uint32_t, RigidBody>& bodies,
    WakeRequestCallback wakeCallback, const ContactManifold* impactContact) {
    using Clock = std::chrono::steady_clock;
    bool wokeAny = false;

    const auto narrowPhaseStart = Clock::now();
    const std::vector<uint32_t>& workList = m_midphase.WorkList();
    const std::size_t workCount = workList.size();
    m_lastParallelStats = {};
    if (workCount > 0) {
        m_workOutputs.clear();
        m_workOutputs.resize(workCount);

        // Reserve chunk records with participant slack; each processRange call claims one.
        m_chunkRecords.clear();
        m_chunkRecords.resize(
            workCount / m_minPairsPerJob + 2 +
            std::max<uint32_t>(1, Core::JobSystem::Get().WorkerCount()));
        std::atomic<uint32_t> claimedChunks{0};

        // Read inputs and persistent pairs; write only this range's outputs and timing record.
        const auto processRange = [&](std::size_t begin, std::size_t end) {
            const uint32_t recordIndex = claimedChunks.fetch_add(1, std::memory_order_relaxed);
            ChunkRecord& record = m_chunkRecords[recordIndex];
            record.threadId = std::this_thread::get_id();
            record.startMilliseconds =
                std::chrono::duration<double, std::milli>(Clock::now() - narrowPhaseStart).count();

            for (std::size_t item = begin; item < end; ++item) {
                const uint32_t slotIndex = workList[item];
                WorkOutput& output = m_workOutputs[item];
                const MidphasePair& pair = m_midphase.PairAt(slotIndex);
                const auto colliderItA = colliders.find(pair.key.bodyA);
                const auto colliderItB = colliders.find(pair.key.bodyB);
                const auto bodyItA = bodies.find(pair.key.bodyA);
                const auto bodyItB = bodies.find(pair.key.bodyB);
                if (colliderItA == colliders.end() || colliderItB == colliders.end() ||
                    bodyItA == bodies.end() || bodyItB == bodies.end()) {
                    output.status = WorkOutput::Status::MissingInput;
                    continue;
                }

                output.manifold = ContactManifold{};
                output.manifold.bodyA = pair.key.bodyA;
                output.manifold.bodyB = pair.key.bodyB;
                output.isTrigger =
                    colliderItA->second.IsTrigger() || colliderItB->second.IsTrigger();
                // Carry the certified CCD patch into the ordinary cache/wake/solver path.
                // Its numerical tolerance must not depend on the speculative band being enabled.
                if (impactContact && pair.key == MakePairKey(impactContact->bodyA, impactContact->bodyB)) {
                    const bool generated = m_narrowPhase->GenerateContact(
                        colliderItA->second, bodyItA->second, colliderItB->second, bodyItB->second,
                        output.manifold, output.stats);
                    if (!generated) output.manifold = *impactContact;
                    output.status = output.stats.gjkFailureCount + output.stats.epaFailureCount != 0 ?
                        WorkOutput::Status::QueryFailure : WorkOutput::Status::Contact;
                    continue;
                }
                const bool reused = m_reuseDefaultNarrowphase && !output.isTrigger &&
                    pair.TryReuseContact(
                        bodyItA->second, bodyItB->second, output.manifold,
                        m_speculativeContactDistance);
                output.reused = reused;
                const bool generatedContact = reused || m_narrowPhase->GenerateContact(
                    colliderItA->second,
                    bodyItA->second,
                    colliderItB->second,
                    bodyItB->second,
                    output.manifold,
                    output.stats);
                const uint32_t failures =
                    output.stats.gjkFailureCount + output.stats.epaFailureCount;
                if (failures != 0) {
                    output.status = WorkOutput::Status::QueryFailure;
                } else if (!generatedContact) {
                    output.status = WorkOutput::Status::Separated;
                } else {
                    output.status = WorkOutput::Status::Contact;
                }
            }

            record.endMilliseconds =
                std::chrono::duration<double, std::milli>(Clock::now() - narrowPhaseStart).count();
        };

        Core::JobSystem& jobSystem = Core::JobSystem::Get();
        if (m_parallelNarrowphaseEnabled && jobSystem.WorkerCount() > 1) {
            jobSystem.ParallelForRange(workCount, m_minPairsPerJob, processRange);
        } else {
            // Serial deterministic fallback: run the whole range in-line.
            processRange(0, workCount);
        }

        // Merge task-private chunk records on this thread (post barrier).
        const uint32_t chunksUsed = claimedChunks.load(std::memory_order_relaxed);
        const double barrierMilliseconds =
            std::chrono::duration<double, std::milli>(Clock::now() - narrowPhaseStart).count();
        std::vector<std::thread::id> threadIds;
        std::vector<double> lastEndPerThread;
        for (uint32_t chunk = 0; chunk < chunksUsed; ++chunk) {
            const ChunkRecord& record = m_chunkRecords[chunk];
            std::size_t threadIndex = threadIds.size();
            for (std::size_t index = 0; index < threadIds.size(); ++index) {
                if (threadIds[index] == record.threadId) {
                    threadIndex = index;
                    break;
                }
            }
            if (threadIndex == threadIds.size()) {
                threadIds.push_back(record.threadId);
                m_lastParallelStats.jobCountPerThread.push_back(0);
                m_lastParallelStats.busyMillisecondsPerThread.push_back(0.0);
                lastEndPerThread.push_back(0.0);
            }
            ++m_lastParallelStats.jobCountPerThread[threadIndex];
            m_lastParallelStats.busyMillisecondsPerThread[threadIndex] +=
                record.endMilliseconds - record.startMilliseconds;
            if (record.endMilliseconds > lastEndPerThread[threadIndex]) {
                lastEndPerThread[threadIndex] = record.endMilliseconds;
            }
        }
        m_lastParallelStats.jobCount = chunksUsed;
        m_lastParallelStats.workerParticipation = static_cast<uint32_t>(threadIds.size());
        for (std::size_t index = 0; index < threadIds.size(); ++index) {
            m_lastParallelStats.workerBusyMilliseconds +=
                m_lastParallelStats.busyMillisecondsPerThread[index];
            const double tailWait = barrierMilliseconds - lastEndPerThread[index];
            if (tailWait > m_lastParallelStats.tailWaitMilliseconds) {
                m_lastParallelStats.tailWaitMilliseconds = tailWait;
            }
        }
    }

    // Main-thread commit in stable work-list (PairKey) order: match old
    // manifold points, transfer impulse caches and publish contact events.
    for (std::size_t item = 0; item < workCount; ++item) {
        const uint32_t slotIndex = workList[item];
        WorkOutput& output = m_workOutputs[item];
        if (output.status == WorkOutput::Status::MissingInput) {
            continue;
        }

        ++m_lastStats.narrowPhaseTestCount;
        m_lastStats.satCallCount += output.stats.satCallCount;
        m_lastStats.primitiveCallCount += output.stats.primitiveCallCount;
        m_lastStats.gjkCallCount += output.stats.gjkCallCount;
        m_lastStats.gjkFailureCount += output.stats.gjkFailureCount;
        m_lastStats.epaCallCount += output.stats.epaCallCount;
        m_lastStats.epaFailureCount += output.stats.epaFailureCount;

        if (output.status == WorkOutput::Status::QueryFailure) {
            // Algorithm failure never poses as a normal separation: no Exit,
            // no touching-state change, pair excluded from this solver input.
            m_midphase.CommitQueryFailure(slotIndex);
            continue;
        }
        if (output.status == WorkOutput::Status::Separated) {
            m_midphase.CommitSeparation(slotIndex, m_lastResult.events);
            continue;
        }

        const MidphasePair& pair = m_midphase.PairAt(slotIndex);
        const RigidBody& bodyA = bodies.find(pair.key.bodyA)->second;
        const RigidBody& bodyB = bodies.find(pair.key.bodyB)->second;
        // Request a wake for the inactive endpoint of any non-trigger contact
        // with exactly one active endpoint; the callback decides whether a wake occurs.
        if (wakeCallback.invoke && !output.isTrigger) {
            const bool activeA = BroadPhaseBodyIsActive(bodyA);
            const bool activeB = BroadPhaseBodyIsActive(bodyB);
            if (activeA != activeB &&
                wakeCallback.invoke(wakeCallback.user, activeA ? pair.key.bodyB : pair.key.bodyA)) {
                // The wake re-runs the whole detection round so the woken
                // island's own pairs join this sub-step's solver input.
                wokeAny = true;
            }
        }
        m_midphase.CommitContact(
            slotIndex,
            bodyA,
            bodyB,
            output.isTrigger,
            std::move(output.manifold),
            m_lastResult.events, output.reused);
        m_lastStats.contactPointCount += m_midphase.PairAt(slotIndex).manifold.pointCount;
        m_lastResult.touchingPairs.push_back(slotIndex);
    }

    m_lastStats.manifoldCount = m_lastResult.touchingPairs.size();
    m_lastStats.narrowPhaseMilliseconds +=
        std::chrono::duration<double, std::milli>(Clock::now() - narrowPhaseStart).count();
    return wokeAny;
}

} // namespace Physics
} // namespace Runtime
