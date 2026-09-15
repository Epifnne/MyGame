#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include "BroadPhase.h"
#include "Collider.h"
#include "ContactManifold.h"
#include "PhysicsMaterial.h"
#include "RigidBody.h"

namespace Runtime {
namespace Physics {

// Candidate pair lifecycle. Candidate removal and touching separation are
// distinct: a pair whose fat AABBs still overlap stays a candidate even when
// the narrow-phase reports separation (touching ends, candidate persists).
enum class PairLifecycleState : uint8_t {
	New,
	Persisting,
	Removed,
};

enum class ContactEventType : uint8_t {
	Enter,
	Stay,
	Exit,
};

// Contact events are published per (fixedStepId, queryEpoch, PairKey). A
// transient touch inside one fixed step may produce Enter followed by Exit;
// Stay is emitted at most once per fixed step per persisting pair.
struct ContactEvent {
	ContactEventType type = ContactEventType::Enter;
	PairKey pair;
	uint64_t fixedStepId = 0;
	uint64_t queryEpoch = 0;
};

// Generational handle into the midphase slot pool. Handles obtained before a
// synchronization point must not be used after slots were recycled.
struct PairHandle {
	static constexpr uint32_t kInvalidIndex = std::numeric_limits<uint32_t>::max();

	uint32_t index = kInvalidIndex;
	uint32_t generation = 0;

	bool IsValid() const { return index != kInvalidIndex; }
};

// Snapshot of external state from the frozen invalidation matrix. Any change
// clears the pair's contact caches (accumulated impulses) but does not by
// itself end the touching state; only a narrow-phase separation, pair removal
// or destruction publishes Exit.
struct PairBindingSnapshot {
	uint32_t colliderIdentityA = 0;
	uint32_t colliderIdentityB = 0;
	uint32_t colliderRevisionA = 0;
	uint32_t colliderRevisionB = 0;
	uint32_t poseRevisionA = 0;
	uint32_t poseRevisionB = 0;
	uint32_t structureRevisionA = 0;
	uint32_t structureRevisionB = 0;
	// Compared by value because the mutable Material() accessor cannot bump
	// the collider revision.
	PhysicsMaterial materialA;
	PhysicsMaterial materialB;

	bool operator==(const PairBindingSnapshot& other) const {
		return colliderIdentityA == other.colliderIdentityA &&
			colliderIdentityB == other.colliderIdentityB &&
			colliderRevisionA == other.colliderRevisionA &&
			colliderRevisionB == other.colliderRevisionB &&
			poseRevisionA == other.poseRevisionA &&
			poseRevisionB == other.poseRevisionB &&
			structureRevisionA == other.structureRevisionA &&
			structureRevisionB == other.structureRevisionB &&
			materialA == other.materialA &&
			materialB == other.materialB;
	}
	bool operator!=(const PairBindingSnapshot& other) const { return !(*this == other); }
};

struct MidphasePair {
	PairKey key;
	PairLifecycleState state = PairLifecycleState::New;
	// Query epoch the pair was last returned by the broad-phase.
	uint64_t lastSeenQueryEpoch = 0;
	uint64_t createdFixedStepId = 0;
	// Last fixed step with a touching contact on this pair.
	uint64_t lastTouchFixedStepId = 0;
	// Last fixed step a Stay event was published (at most one per step).
	uint64_t lastStayFixedStepId = 0;
	// Touching state as of the latest commit; independent from candidacy.
	bool hadContact = false;
	bool needsNarrowphase = false;
	// Set when this query's binding check cleared the impulse caches.
	bool cacheClearedThisQuery = false;
	PairBindingSnapshot binding;
	// Persistent manifold: fresh geometry is committed every touching query and
	// matched against the previous points to transfer the impulse caches.
	ContactManifold manifold;
	// Fixed-step total impulse summary (telemetry contract): the net impulse
	// actually applied across this fixed step's sub-steps (each sub-step's
	// final accumulated lambda, warm start included). Accumulated by the
	// solver through PhysicsWorld and reset on the first touch of a new fixed
	// step; copied onto the published manifold snapshot.
	float fixedStepNormalImpulse = 0.0f;
	glm::vec3 fixedStepTangentImpulse = glm::vec3(0.0f);
};

struct MidphaseStats {
	std::size_t activePairCount = 0;
	std::size_t newPairCount = 0;
	std::size_t removedPairCount = 0;
	std::size_t workListSize = 0;
};

// Midphase: owns the persistent pair pool between broad-phase and narrow-phase.
// Candidates live in a continuous slot array with a free-list; an unordered
// map resolves PairKey to a generational handle. The narrow-phase consumes a
// continuous work list of slot indices and never traverses the map. Removed
// slots are recycled at the next query's begin (the synchronization point), so
// handles captured during a query stay valid until that query is committed.
class Midphase {
public:
	Midphase() = default;

	// Begin a new query cycle; recycles slots removed before the sync point.
	void BeginQuery(uint64_t queryEpoch, uint64_t fixedStepId, BroadPhaseQueryCoverage coverage);

	// Register one broad-phase candidate and append it to the work list.
	// Returns the slot index. Revalidates the binding snapshot against the
	// frozen invalidation matrix and clears stale contact caches.
	uint32_t RegisterCandidate(
		const PairKey& key,
		const Collider& colliderA,
		const RigidBody& bodyA,
		const Collider& colliderB,
		const RigidBody& bodyB);

	// Sweep candidates not re-seen by this query. A missing pair only justifies
	// removal when it belonged to the query coverage (FullScene here); a skipped
	// (e.g. sleeping) query never implies separation. Removal of a touching pair
	// publishes Exit with the stable PairKey. Removed slots are recycled at the
	// next BeginQuery, not here.
	void FinishQuery(std::vector<ContactEvent>& events);

	// Commit a touching narrow-phase result: computes local anchors, matches the
	// previous points to transfer impulse caches (with their interpretation
	// data: old tangent basis, cachedDt, TOI origin flag), stamps the fresh
	// TOI flag, resets the per-sub-step normalImpulse base, restarts the
	// fixed-step impulse totals on the first touch of a new fixed step and
	// publishes Enter/Stay transitions. The solver re-stamps the current
	// tangent basis and cachedDt when it stores the solved cache.
	void CommitContact(
		uint32_t slotIndex,
		const RigidBody& bodyA,
		const RigidBody& bodyB,
		bool isTrigger,
		ContactManifold freshManifold,
		bool isToiImpact,
		std::vector<ContactEvent>& events);

	// Commit a clean narrow-phase separation: keeps the candidate pair, clears
	// the contact caches and publishes Exit when the pair was touching.
	void CommitSeparation(uint32_t slotIndex, std::vector<ContactEvent>& events);

	// GJK/EPA algorithm failure policy (frozen contract): a failure never poses
	// as a normal separation. Touching state, caches and events stay untouched;
	// the pair is excluded from this query's solver input by the caller.
	void CommitQueryFailure(uint32_t slotIndex);

	// Continuous narrow-phase work list of slot indices, in stable PairKey
	// order (it follows the sorted broad-phase output).
	const std::vector<uint32_t>& WorkList() const { return m_workList; }

	MidphasePair& PairAt(uint32_t slotIndex) { return m_slots[slotIndex].pair; }
	const MidphasePair& PairAt(uint32_t slotIndex) const { return m_slots[slotIndex].pair; }

	const MidphasePair* FindPair(const PairKey& key) const;
	MidphasePair* FindPair(const PairKey& key);

	bool IsHandleValid(const PairHandle& handle) const;

	std::size_t ActivePairCount() const { return m_pairToSlot.size(); }
	const MidphaseStats& LastStats() const { return m_stats; }

private:
	struct Slot {
		MidphasePair pair;
		uint32_t generation = 1;
		bool occupied = false;
	};

	uint32_t AllocateSlot();
	void RecyclePendingRemovals();
	static PairBindingSnapshot CaptureBinding(
		const Collider& colliderA,
		const RigidBody& bodyA,
		const Collider& colliderB,
		const RigidBody& bodyB);
	static void ClearContactCaches(MidphasePair& pair);
	void PushEvent(
		std::vector<ContactEvent>& events,
		ContactEventType type,
		const PairKey& key) const;

	std::vector<Slot> m_slots;
	std::vector<uint32_t> m_freeSlots;
	std::vector<uint32_t> m_pendingRemoval;
	std::unordered_map<PairKey, PairHandle, PairKeyHash> m_pairToSlot;
	std::vector<uint32_t> m_workList;
	uint64_t m_queryEpoch = 0;
	uint64_t m_fixedStepId = 0;
	BroadPhaseQueryCoverage m_coverage = BroadPhaseQueryCoverage::FullScene;
	MidphaseStats m_stats;
};

} // namespace Physics
} // namespace Runtime
