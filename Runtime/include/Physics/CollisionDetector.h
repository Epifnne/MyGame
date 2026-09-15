#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

#include "BroadPhase.h"
#include "ContactManifold.h"
#include "Midphase.h"
#include "NarrowPhase.h"

namespace Runtime {
namespace Physics {

struct CollisionDetectionStats {
	std::size_t staticLeafCount = 0;
	std::size_t dynamicLeafCount = 0;
	std::size_t broadPhaseCandidateCount = 0;
	std::size_t narrowPhaseTestCount = 0;
	std::size_t gjkCallCount = 0;
	std::size_t gjkFailureCount = 0;
	std::size_t epaCallCount = 0;
	std::size_t epaFailureCount = 0;
	std::size_t manifoldCount = 0;
	std::size_t contactPointCount = 0;
	std::size_t midphaseActivePairCount = 0;
	std::size_t midphaseNewPairCount = 0;
	std::size_t midphaseRemovedPairCount = 0;
	double broadPhaseMilliseconds = 0.0;
	double narrowPhaseMilliseconds = 0.0;
};

// Phase 6 parallel narrow-phase telemetry of the last Detect call. The wall
// clock stays in CollisionDetectionStats::narrowPhaseMilliseconds; this
// records how the work was distributed across the job system.
struct NarrowPhaseParallelStats {
	// Chunks actually executed and distinct threads that ran them.
	uint32_t jobCount = 0;
	uint32_t workerParticipation = 0;
	// Summed per-chunk execution time across all threads (CPU time, not wall).
	double workerBusyMilliseconds = 0.0;
	// Longest idle wait of a thread between finishing its last chunk and the
	// frame barrier returning (tail latency of the slowest straggler).
	double tailWaitMilliseconds = 0.0;
	std::vector<uint32_t> jobCountPerThread;
	std::vector<double> busyMillisecondsPerThread;
};

// Output of one detection query. Touching pairs are referenced by midphase
// slot index (valid until the next query's synchronization point); events are
// published per (fixedStepId, queryEpoch, PairKey).
struct CollisionDetectionResult {
	// Slots with a touching contact this query, in stable PairKey order.
	std::vector<uint32_t> touchingPairs;
	std::vector<ContactEvent> events;
};

// Collision pipeline split into three stages: the broad-phase submits
// normalized PairKeys plus query coverage, the midphase owns the persistent
// pair lifecycle and builds a continuous work list, and the narrow-phase
// produces fresh geometry per work item. The main thread then commits the
// results: matches old manifold points, transfers impulse caches and
// publishes contact events.
class CollisionDetector {
public:
	// Create detector with default broad-phase and narrow-phase implementations.
	CollisionDetector();
	// Virtual resources managed by unique_ptr members.
	~CollisionDetector();

	// Override broad-phase stage implementation.
	void SetBroadPhase(std::unique_ptr<BroadPhase> broadPhase) {
		if (broadPhase) {
			m_broadPhase = std::move(broadPhase);
		}
	}

	// Override narrow-phase stage implementation.
	void SetNarrowPhase(std::unique_ptr<NarrowPhase> narrowPhase) {
		if (narrowPhase) {
			m_narrowPhase = std::move(narrowPhase);
		}
	}

	// Run one detection query for a sub-step and commit it into the persistent
	// pair pool. The result reference stays valid until the next Detect call.
	const CollisionDetectionResult& Detect(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies,
		uint64_t fixedStepId,
		uint64_t queryEpoch,
		float substepDt,
		bool isToiSubstep);

	// Persistent pair pool; solvers read/write the persistent manifolds of the
	// touching slots reported by Detect.
	Midphase& GetMidphase() { return m_midphase; }
	const Midphase& GetMidphase() const { return m_midphase; }

	const CollisionDetectionStats& LastStats() const { return m_lastStats; }
	const NarrowPhaseParallelStats& LastParallelStats() const { return m_lastParallelStats; }

	// Phase 6: parallel narrow-phase controls. Disabled forces in-line
	// execution on the calling thread; the job system worker count (1 = fully
	// serial deterministic fallback) is owned by PhysicsWorld.
	void SetParallelNarrowphaseEnabled(bool enabled) { m_parallelNarrowphaseEnabled = enabled; }
	bool ParallelNarrowphaseEnabled() const { return m_parallelNarrowphaseEnabled; }
	void SetNarrowphaseMinPairsPerJob(std::size_t minPairsPerJob) {
		m_minPairsPerJob = minPairsPerJob > 0 ? minPairsPerJob : 1;
	}
	std::size_t NarrowphaseMinPairsPerJob() const { return m_minPairsPerJob; }

private:
	// Per-work-item output slot; exactly one worker writes slot i (its own
	// chunk), nobody else reads it until after the frame barrier.
	struct WorkOutput {
		enum class Status : uint8_t {
			MissingInput,
			Separated,
			Contact,
			QueryFailure,
		};
		Status status = Status::MissingInput;
		bool isTrigger = false;
		NarrowPhaseQueryStats stats;
		ContactManifold manifold;
	};
	// Per-chunk execution record; slots are claimed atomically (no shared
	// container writes) and merged on the calling thread after the barrier.
	struct ChunkRecord {
		std::thread::id threadId;
		double startMilliseconds = 0.0;
		double endMilliseconds = 0.0;
	};

	// Stage 3: run the narrow-phase over the midphase work list (serially or
	// through Core::JobSystem) and commit outputs in stable PairKey order.
	void ExecuteNarrowphase(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies,
		bool isToiSubstep);

	std::unique_ptr<BroadPhase> m_broadPhase;
	std::unique_ptr<NarrowPhase> m_narrowPhase;
	Midphase m_midphase;
	CollisionDetectionStats m_lastStats;
	CollisionDetectionResult m_lastResult;
	NarrowPhaseParallelStats m_lastParallelStats;
	bool m_parallelNarrowphaseEnabled = true;
	std::size_t m_minPairsPerJob = 32;
	// Reused buffers; sized to the work list, capacity survives across frames.
	std::vector<WorkOutput> m_workOutputs;
	std::vector<ChunkRecord> m_chunkRecords;
};

} // namespace Physics
} // namespace Runtime
