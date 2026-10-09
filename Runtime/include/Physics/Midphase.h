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

	// Check the index sentinel only; pool occupancy/generation need IsHandleValid.
	bool IsValid() const { return index != kInvalidIndex; }
};

// Snapshot of external identities, revisions, and materials. Any change
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

	// Compare both endpoints' identities/revisions and material values.
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
	// Negate the complete binding comparison.
	bool operator!=(const PairBindingSnapshot& other) const { return !(*this == other); }
};

// Persistent candidate, touching state, reusable surface anchors, and fixed-step impulse totals.
struct MidphasePair {
	PairKey key;
	PairLifecycleState state = PairLifecycleState::New;
	// Query epoch the pair was last returned by the broad-phase.
	uint64_t lastSeenQueryEpoch = 0;
	uint64_t createdFixedStepId = 0;
	// Last fixed step with a touching contact on this pair.
	uint64_t lastTouchFixedStepId = 0;
	uint64_t lastSolvedFixedStepId = 0;
	// Last fixed step a Stay event was published (at most one per step).
	uint64_t lastStayFixedStepId = 0;
	// Touching state as of the latest commit; independent from candidacy.
	bool hadContact = false;
	bool needsNarrowphase = false;
	// Set when this query's binding check cleared the impulse caches.
	bool cacheClearedThisQuery = false;
	PairBindingSnapshot binding;
	// Persistent manifold: either refreshed by narrow-phase or rebuilt from
	// nearly stationary anchors, then matched to transfer impulse caches.
	ContactManifold manifold;
	glm::vec3 contactNormal = glm::vec3(0.0f, 1.0f, 0.0f);
	glm::vec3 contactPositionA = glm::vec3(0.0f);
	glm::vec3 contactPositionB = glm::vec3(0.0f);
	glm::quat contactOrientationA = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	glm::quat contactOrientationB = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	// Rebuild anchors for near-identical relative motion: <=1 mm translation,
	// absolute orientation-dot >=0.9998477, <=1 cm tangential drift, gap<=maxSeparation.
	// The caller supplies an empty output; late rejection clears its points.
	bool TryReuseContact(
		const RigidBody& bodyA, const RigidBody& bodyB,
		ContactManifold& outManifold, float maxSeparation) const;
	// Fixed-step total impulse summary (telemetry contract): the net impulse
	// actually applied across this fixed step's sub-steps (each sub-step's
	// final accumulated lambda, warm start included). Accumulated by the
	// solver through PhysicsWorld and reset on the first touch of a new fixed
	// step; copied onto the published manifold snapshot.
	float fixedStepNormalImpulse = 0.0f;
	glm::vec3 fixedStepTangentImpulse = glm::vec3(0.0f);
};

// Pair-lifecycle counts and work-list size for the latest query round.
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
	// Initialize an empty pool with no pending removals or query work.
	Midphase() = default;

	// Begin a new query cycle; recycles slots removed before the sync point.
	void BeginQuery(uint64_t queryEpoch, uint64_t fixedStepId, BroadPhaseQueryCoverage coverage);

	// Register one broad-phase candidate and append it to the work list.
	// Returns the slot index. Revalidates the binding snapshot against the
	// endpoint identities/revisions/materials and clears stale contact caches.
	uint32_t RegisterCandidate(
		const PairKey& key,
		const Collider& colliderA,
		const RigidBody& bodyA,
		const Collider& colliderB,
		const RigidBody& bodyB);

	// Sweep candidates not re-seen by this query. Under FullScene coverage every
	// missing pair is removed. Under ActiveDynamics coverage a missing
	// pair is removed only when it belonged to the query: an endpoint whose
	// collider or body disappeared is always covered; otherwise the pair is
	// covered only while neither endpoint is a sleeping dynamic body. Sleeping
	// pairs are retained untouched (a skipped query never implies separation).
	// Removal of a touching pair publishes Exit with the stable PairKey.
	// Removed slots are recycled at the next BeginQuery, not here.
	void FinishQuery(
		const std::unordered_map<uint32_t, RigidBody>& bodies,
		const std::unordered_map<uint32_t, Collider>& colliders,
		std::vector<ContactEvent>& events);

	// Commit a touching narrow-phase result: computes local anchors, matches the
	// previous points to transfer impulse caches (with their interpretation
	// data: old tangent basis, cachedDt), resets the per-sub-step normalImpulse
	// base, restarts the fixed-step impulse totals on the first touch of a new
	// fixed step and publishes Enter/Stay transitions. The solver re-stamps the
	// current tangent basis and cachedDt when it stores the solved cache.
	void CommitContact(
		uint32_t slotIndex,
		const RigidBody& bodyA,
		const RigidBody& bodyB,
		bool isTrigger,
		ContactManifold freshManifold,
		std::vector<ContactEvent>& events, bool reused = false);

	// Commit a clean narrow-phase separation: keeps the candidate pair, clears
	// the contact caches and publishes Exit when the pair was touching.
	void CommitSeparation(uint32_t slotIndex, std::vector<ContactEvent>& events);

	// GJK/EPA algorithm failure policy: a failure never poses
	// as a normal separation. Touching state, caches and events stay untouched;
	// the pair is excluded from this query's solver input by the caller.
	void CommitQueryFailure(uint32_t slotIndex);

	// Continuous narrow-phase work list of slot indices, in stable PairKey
	// order (it follows the sorted broad-phase output).
	const std::vector<uint32_t>& WorkList() const { return m_workList; }

	// Access a caller-validated slot directly without bounds or generation checks.
	MidphasePair& PairAt(uint32_t slotIndex) { return m_slots[slotIndex].pair; }
	// Read a caller-validated slot directly without handle checks.
	const MidphasePair& PairAt(uint32_t slotIndex) const { return m_slots[slotIndex].pair; }

	// Resolve a live key to its persistent pair, or return null.
	const MidphasePair* FindPair(const PairKey& key) const;
	// Resolve a live key for mutation, or return null.
	MidphasePair* FindPair(const PairKey& key);

	// Visit every live pair (slots pending removal are skipped). Templated so
	// the callback inlines; used by the wake closure to walk the
	// contact adjacency of sleeping islands.
	template <typename Fn>
	void ForEachPair(Fn&& fn) const {
		for (const Slot& slot : m_slots) {
			if (slot.occupied && slot.pair.state != PairLifecycleState::Removed) {
				fn(slot.pair);
			}
		}
	}

	// Require a valid in-range index, occupied slot, and matching generation.
	bool IsHandleValid(const PairHandle& handle) const;

	// Count keys still present in the live-pair map.
	std::size_t ActivePairCount() const { return m_pairToSlot.size(); }
	// Read statistics reset by the latest BeginQuery.
	const MidphaseStats& LastStats() const { return m_stats; }

private:
	// Pool entry whose generation changes when deferred removal is recycled.
	struct Slot {
		MidphasePair pair;
		uint32_t generation = 1;
		bool occupied = false;
	};

	// Mark a free slot occupied, or append a fresh occupied entry.
	uint32_t AllocateSlot();
	// Reset deferred slots, increment generations, and return indices to the free list.
	void RecyclePendingRemovals();
	// Copy endpoint identities, revisions, and material values for cache invalidation.
	static PairBindingSnapshot CaptureBinding(
		const Collider& colliderA,
		const RigidBody& bodyA,
		const Collider& colliderB,
		const RigidBody& bodyB);
	// Zero per-point cached impulses, tangent basis, and cached timestep.
	static void ClearContactCaches(MidphasePair& pair);
	// Append an event stamped with the current fixed step and query epoch.
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
