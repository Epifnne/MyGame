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

// Detection counters and stage timings, accumulated across same-query wake rounds.
struct CollisionDetectionStats {
	std::size_t staticLeafCount = 0;
	std::size_t dynamicLeafCount = 0;
	std::size_t broadPhaseCandidateCount = 0;
	std::size_t narrowPhaseTestCount = 0;
	std::size_t satCallCount = 0;
	std::size_t primitiveCallCount = 0;
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

// Parallel narrow-phase telemetry of the latest round. The wall
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

// Non-owning synchronous callback on the commit thread for non-trigger contacts
// with exactly one awake dynamic endpoint. The inactive endpoint is passed even
// if static; the callback decides whether a wake occurs. Speculative/reused
// contacts also qualify. A successful wake requests another same-pose detection
// round (bounded by bodyCount+1); candidate proximity alone never invokes it.
struct WakeRequestCallback {
	void* user;
	// Returns true when the call actually woke at least one sleeping body.
	bool (*invoke)(void* user, uint32_t sleepingBodyId);
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
	// Create hybrid-BVH broad phase and analytic/SAT/GJK/EPA narrow phase.
	CollisionDetector();
	// Destroy the owned stages, persistent pool, and work buffers.
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
			m_reuseDefaultNarrowphase = false;
		}
	}

	// Clamp the speculative band to nonnegative meters and forward it to the
	// current narrow phase; also used for reuse. No fat-margin clamp is done here.
	void SetSpeculativeContactDistance(float distance) {
		m_speculativeContactDistance = distance > 0.0f ? distance : 0.0f;
		if (m_narrowPhase) {
			m_narrowPhase->SetSpeculativeContactDistance(m_speculativeContactDistance);
		}
	}
	// Read the clamped speculative band configured on this detector.
	float SpeculativeContactDistance() const { return m_speculativeContactDistance; }

	// Run one detection query for a sub-step and commit it into the persistent
	// pair pool. The result reference stays valid until the next Detect call.
	const CollisionDetectionResult& Detect(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies,
		uint64_t fixedStepId,
		uint64_t queryEpoch,
		float substepDt,
		WakeRequestCallback wakeCallback = {}, const ContactManifold* impactContact = nullptr);

	// Pass through the broad-phase dynamic-leaf region query
	// (sleeping bodies included) used for static-modification wakes.
	void CollectDynamicLeafOverlaps(
		const AABB& region,
		const std::unordered_map<uint32_t, RigidBody>& bodies,
		std::vector<uint32_t>& outBodyIds) const {
		if (m_broadPhase) {
			m_broadPhase->CollectDynamicLeafOverlaps(region, bodies, outBodyIds);
		}
	}

	// Persistent pair pool; solvers read/write the persistent manifolds of the
	// touching slots reported by Detect.
	Midphase& GetMidphase() { return m_midphase; }
	// Read the persistent pair pool without permitting mutation.
	const Midphase& GetMidphase() const { return m_midphase; }

	// Read counters and timings from the latest Detect call.
	const CollisionDetectionStats& LastStats() const { return m_lastStats; }
	// Read worker telemetry from the latest narrow-phase round.
	const NarrowPhaseParallelStats& LastParallelStats() const { return m_lastParallelStats; }

	// Parallel narrow-phase controls. Disabled forces in-line
	// execution on the calling thread; the job system worker count (1 = fully
	// serial deterministic fallback) is owned by PhysicsWorld.
	void SetParallelNarrowphaseEnabled(bool enabled) { m_parallelNarrowphaseEnabled = enabled; }
	// Read whether job-system execution is permitted.
	bool ParallelNarrowphaseEnabled() const { return m_parallelNarrowphaseEnabled; }
	// Store the requested minimum chunk size, clamping zero to one.
	void SetNarrowphaseMinPairsPerJob(std::size_t minPairsPerJob) {
		m_minPairsPerJob = minPairsPerJob > 0 ? minPairsPerJob : 1;
	}
	// Read the minimum work items requested per chunk.
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
		bool reused = false;
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

	// One full pass over the three stages. Returns true when a confirmed
	// awake-sleeping contact woke an island during the commit, in which case
	// Detect re-runs the round (wake propagation closure).
	bool RunDetectionRound(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies,
		uint64_t fixedStepId,
		uint64_t queryEpoch,
		float substepDt,
		WakeRequestCallback wakeCallback, const ContactManifold* impactContact);

	// Stage 3: run the narrow-phase over the midphase work list (serially or
	// through Core::JobSystem) and commit outputs in stable PairKey order.
	// Non-trigger contacts with exactly one active endpoint invoke the wake
	// callback on this thread before CommitContact.
	// Returns true when any wake occurred during the commit.
	bool ExecuteNarrowphase(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies,
		WakeRequestCallback wakeCallback, const ContactManifold* impactContact);

	std::unique_ptr<BroadPhase> m_broadPhase;
	std::unique_ptr<NarrowPhase> m_narrowPhase;
	bool m_reuseDefaultNarrowphase = true;
	float m_speculativeContactDistance = 0.0f;
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
