#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include "BvhTree.h"
#include "Collider.h"
#include "RigidBody.h"

namespace Runtime {
namespace Physics {

// Ordered body identifiers; MakePairKey sorts endpoints (equal IDs remain equal).
struct PairKey {
	uint32_t bodyA = 0;
	uint32_t bodyB = 0;

	// Compare both endpoints for pair identity.
	bool operator==(const PairKey& other) const {
		return bodyA == other.bodyA && bodyB == other.bodyB;
	}
	// Lexicographically order pairs for deterministic output.
	bool operator<(const PairKey& other) const {
		return bodyA < other.bodyA || (bodyA == other.bodyA && bodyB < other.bodyB);
	}
	// Pack A in the high 32 bits and B in the low 32 bits.
	uint64_t ToUint64() const {
		return (static_cast<uint64_t>(bodyA) << 32) | static_cast<uint64_t>(bodyB);
	}
};

// Build a canonical PairKey from an unordered body id pair.
PairKey MakePairKey(uint32_t a, uint32_t b);

// Hash for PairKey-keyed containers (midphase map, fixed-step contact summary).
struct PairKeyHash {
	// Hash the packed endpoint identifiers.
	std::size_t operator()(const PairKey& key) const {
		return std::hash<uint64_t>{}(key.ToUint64());
	}
};

// Query coverage states which bodies a query re-checked.
// A missing pair may only justify pair removal when the pair belonged to the
// query coverage; a skipped (e.g. sleeping) query never implies separation.
enum class BroadPhaseQueryCoverage {
	// Full re-query of every collider-bearing body.
	FullScene,
	// Only pairs with at least one awake dynamic endpoint are generated.
	// Sleeping leaves participate in traversal, but inactive-only pairs are filtered out.
	ActiveDynamics,
};

// Activity predicate shared by both broad-phase implementations: a
// body actively queries when it is dynamic and not sleeping.
inline bool BroadPhaseBodyIsActive(const RigidBody& body) {
	return !body.IsStatic() && !body.IsSleeping();
}

// Result of one broad-phase query.
struct BroadPhaseQueryResult {
	// Normalized (bodyA < bodyB), deduplicated and stable-sorted candidate pairs.
	std::vector<PairKey> pairs;
	BroadPhaseQueryCoverage coverage = BroadPhaseQueryCoverage::FullScene;
};

// Actual leaf counts of the backing tree(s). tracked == false means the
// implementation keeps no per-class trees; callers then fall back to logical
// collider/body classification for statistics.
struct BroadPhaseLeafCounts {
	std::size_t staticLeaves = 0;
	std::size_t dynamicLeaves = 0;
	bool tracked = false;
};

// Interface for candidate generation and dynamic-leaf region queries.
class BroadPhase {
public:
	// Allow derived tree resources to be destroyed through the interface.
	virtual ~BroadPhase() = default;

	// Compute potentially colliding body pairs with query coverage metadata.
	virtual BroadPhaseQueryResult ComputePairs(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies) = 0;

	// Actual tree leaf counts; the default reports untracked.
	virtual BroadPhaseLeafCounts GetLeafCounts() const { return {}; }

	// Collect the dynamic leaves (sleeping bodies included; they stay
	// in the dynamic tree) whose fat AABB overlaps the region. Used to find
	// sleeping bodies affected by a static body's modification.
	virtual void CollectDynamicLeafOverlaps(
		const AABB& region,
		const std::unordered_map<uint32_t, RigidBody>& bodies,
		std::vector<uint32_t>& outBodyIds) const = 0;
};

// Legacy single-tree broad-phase: one BVH holds static and dynamic leaves.
// Retained only as the A/B comparison path for the hybrid broad-phase.
class DynamicBvhBroadPhase final : public BroadPhase {
public:
	// Start with an empty mixed static/dynamic tree.
	DynamicBvhBroadPhase() = default;
	// Release the mixed tree's node storage.
	~DynamicBvhBroadPhase() override = default;

	// Refresh the mixed tree, self-query, filter inactive/masked pairs, and sort.
	BroadPhaseQueryResult ComputePairs(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies) override;

	// Single tree holds every leaf; dynamic overlaps are filtered by body.
	void CollectDynamicLeafOverlaps(
		const AABB& region,
		const std::unordered_map<uint32_t, RigidBody>& bodies,
		std::vector<uint32_t>& outBodyIds) const override;

private:
	BvhTree m_tree;
};

// Hybrid broad-phase managing one static and one dynamic BVH.
// Dynamic leaves cover all non-static bodies, sleeping ones included
// (sleeping bodies keep their leaf but never initiate queries).
// Both trees refit contained bounds and reinsert on escape or excessive area growth.
class HybridBvhBroadPhase final : public BroadPhase {
public:
	// Start with empty static and dynamic trees.
	HybridBvhBroadPhase() = default;
	// Release both trees' node storage.
	~HybridBvhBroadPhase() override = default;

	// Dynamic-dynamic self query plus dynamic-static cross query; static-static
	// pairs are never generated. Output is normalized, deduplicated, sorted.
	// Pairs without an awake dynamic endpoint are filtered out; report ActiveDynamics coverage.
	BroadPhaseQueryResult ComputePairs(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies) override;

	// Report actual leaf membership in each tree.
	BroadPhaseLeafCounts GetLeafCounts() const override {
		return {m_staticTree.LeafCount(), m_dynamicTree.LeafCount(), true};
	}

	// Dynamic-tree region query (sleeping leaves included).
	void CollectDynamicLeafOverlaps(
		const AABB& region,
		const std::unordered_map<uint32_t, RigidBody>& bodies,
		std::vector<uint32_t>& outBodyIds) const override;

private:
	BvhTree m_staticTree;
	BvhTree m_dynamicTree;
};

} // namespace Physics
} // namespace Runtime
