#include "Physics/CollisionDetector.h"

#include <chrono>
#include <utility>

#include "Core/JobSystem.h"

namespace Runtime {
namespace Physics {

CollisionDetector::CollisionDetector()
    : m_broadPhase(std::make_unique<HybridBvhBroadPhase>()),
      m_narrowPhase(std::make_unique<GjkEpaNarrowPhase>()) {}

CollisionDetector::~CollisionDetector() = default;

const CollisionDetectionResult& CollisionDetector::Detect(
    const std::unordered_map<uint32_t, Collider>& colliders,
    const std::unordered_map<uint32_t, RigidBody>& bodies,
    uint64_t fixedStepId,
    uint64_t queryEpoch,
    float substepDt,
    bool isToiSubstep) {
    using Clock = std::chrono::steady_clock;

    m_lastStats = {};
    m_lastResult.touchingPairs.clear();
    m_lastResult.events.clear();
    if (!m_broadPhase || !m_narrowPhase) {
        return m_lastResult;
    }

    // Stage 1: broad-phase submits normalized pairs plus query coverage.
    const auto broadPhaseStart = Clock::now();
    const BroadPhaseQueryResult queryResult = m_broadPhase->ComputePairs(colliders, bodies);
    const auto broadPhaseEnd = Clock::now();
    m_lastStats.broadPhaseCandidateCount = queryResult.pairs.size();
    m_lastStats.broadPhaseMilliseconds =
        std::chrono::duration<double, std::milli>(broadPhaseEnd - broadPhaseStart).count();

    // Sample leaf counts after the query synchronized the trees.
    const BroadPhaseLeafCounts leafCounts = m_broadPhase->GetLeafCounts();
    if (leafCounts.tracked) {
        // Phase 1: report the actual leaf counts of the two BVH trees.
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

    // Stage 2: midphase updates the persistent pair lifecycle and builds the
    // continuous narrow-phase work list. Candidates missing from a FullScene
    // query are swept (touching ones publish Exit).
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
    m_midphase.FinishQuery(m_lastResult.events);
    const MidphaseStats& midphaseStats = m_midphase.LastStats();
    m_lastStats.midphaseActivePairCount = midphaseStats.activePairCount;
    m_lastStats.midphaseNewPairCount = midphaseStats.newPairCount;
    m_lastStats.midphaseRemovedPairCount = midphaseStats.removedPairCount;

    // Stage 3: narrow-phase generates fresh geometry per work-list slot
    // (serially or across the job system); this thread then commits each
    // result into the persistent pair pool in stable PairKey order.
    ExecuteNarrowphase(colliders, bodies, isToiSubstep);

    return m_lastResult;
}

void CollisionDetector::ExecuteNarrowphase(
    const std::unordered_map<uint32_t, Collider>& colliders,
    const std::unordered_map<uint32_t, RigidBody>& bodies,
    bool isToiSubstep) {
    using Clock = std::chrono::steady_clock;

    const auto narrowPhaseStart = Clock::now();
    const std::vector<uint32_t>& workList = m_midphase.WorkList();
    const std::size_t workCount = workList.size();
    m_lastParallelStats = {};
    if (workCount > 0) {
        m_workOutputs.clear();
        m_workOutputs.resize(workCount);

        // Chunk record slots are claimed atomically. Claims exceed the actual
        // chunk count: every participating thread performs one final
        // out-of-range claim before exiting, so size for
        // ceil(workCount / minItemsPerJob) chunks plus all participants.
        m_chunkRecords.clear();
        m_chunkRecords.resize(
            workCount / m_minPairsPerJob + 2 +
            std::max<uint32_t>(1, Core::JobSystem::Get().WorkerCount()));
        std::atomic<uint32_t> claimedChunks{0};

        // Workers only read the frozen collider/body snapshots and write
        // their own output slots plus task-private statistics; nothing here
        // touches the BVH, the pair map, the midphase or event lists.
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
                const bool generatedContact = m_narrowPhase->GenerateContact(
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
        m_midphase.CommitContact(
            slotIndex,
            bodyA,
            bodyB,
            output.isTrigger,
            std::move(output.manifold),
            isToiSubstep,
            m_lastResult.events);
        m_lastStats.contactPointCount += m_midphase.PairAt(slotIndex).manifold.pointCount;
        m_lastResult.touchingPairs.push_back(slotIndex);
    }

    m_lastStats.manifoldCount = m_lastResult.touchingPairs.size();
    m_lastStats.narrowPhaseMilliseconds =
        std::chrono::duration<double, std::milli>(Clock::now() - narrowPhaseStart).count();
}

} // namespace Physics
} // namespace Runtime
