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

// Canonical broad-phase pair identifier with bodyA < bodyB.
// Frozen pair identity contract consumed by the Phase 2 Midphase.
struct PairKey {
	uint32_t bodyA = 0;
	uint32_t bodyB = 0;

	bool operator==(const PairKey& other) const {
		return bodyA == other.bodyA && bodyB == other.bodyB;
	}
	bool operator<(const PairKey& other) const {
		return bodyA < other.bodyA || (bodyA == other.bodyA && bodyB < other.bodyB);
	}
	uint64_t ToUint64() const {
		return (static_cast<uint64_t>(bodyA) << 32) | static_cast<uint64_t>(bodyB);
	}
};

// Build a canonical PairKey from an unordered body id pair.
PairKey MakePairKey(uint32_t a, uint32_t b);

// Hash for PairKey-keyed containers (midphase map, fixed-step contact summary).
struct PairKeyHash {
	std::size_t operator()(const PairKey& key) const {
		return std::hash<uint64_t>{}(key.ToUint64());
	}
};

// Frozen query coverage contract: states which bodies a query re-checked.
// A missing pair may only justify pair removal when the pair belonged to the
// query coverage; a skipped (e.g. sleeping) query never implies separation.
enum class BroadPhaseQueryCoverage {
	// Full re-query of every collider-bearing body; the only mode before sleep lands.
	FullScene,
};

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

class BroadPhase {
public:
	virtual ~BroadPhase() = default;

	// Compute potentially colliding body pairs with query coverage metadata.
	virtual BroadPhaseQueryResult ComputePairs(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies) = 0;

	// Actual tree leaf counts; the default reports untracked.
	virtual BroadPhaseLeafCounts GetLeafCounts() const { return {}; }
};

// Legacy single-tree broad-phase: one BVH holds static and dynamic leaves.
// Retained only as the A/B comparison path for the hybrid broad-phase.
class DynamicBvhBroadPhase final : public BroadPhase {
public:
	DynamicBvhBroadPhase() = default;
	~DynamicBvhBroadPhase() override = default;

	// Build candidate pairs using single-tree BVH self traversal.
	BroadPhaseQueryResult ComputePairs(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies) override;

private:
	BvhTree m_tree;
};

// Hybrid broad-phase managing one static and one dynamic BVH.
// Dynamic leaves cover all non-static bodies (all active before sleep lands).
// The static tree only reinserts leaves whose tight AABB exits the stored fat
// AABB (creation, destruction, collider/shape/transform change, static switch).
class HybridBvhBroadPhase final : public BroadPhase {
public:
	HybridBvhBroadPhase() = default;
	~HybridBvhBroadPhase() override = default;

	// Dynamic-dynamic self query plus dynamic-static cross query; static-static
	// pairs are never generated. Output is normalized, deduplicated, sorted.
	BroadPhaseQueryResult ComputePairs(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies) override;

	BroadPhaseLeafCounts GetLeafCounts() const override {
		return {m_staticTree.LeafCount(), m_dynamicTree.LeafCount(), true};
	}

private:
	BvhTree m_staticTree;
	BvhTree m_dynamicTree;
};

} // namespace Physics
} // namespace Runtime
