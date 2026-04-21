#pragma once

#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Collider.h"
#include "RigidBody.h"

namespace Runtime {
namespace Physics {

using BroadPhasePair = std::pair<uint32_t, uint32_t>;

class BroadPhase {
public:
	virtual ~BroadPhase() = default;

	// Compute potentially colliding body id pairs.
	virtual std::vector<BroadPhasePair> ComputePairs(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies) = 0;
};

class DynamicBvhBroadPhase final : public BroadPhase {
public:
	DynamicBvhBroadPhase() = default;
	~DynamicBvhBroadPhase() override = default;

	// Build candidate pairs using a dynamic BVH traversal.
	std::vector<BroadPhasePair> ComputePairs(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies) override;

private:
	struct Node {
		AABB aabb;
		bool active = false;
		int parent = -1;
		int left = -1;
		int right = -1;
		uint32_t bodyId = 0;

		// Leaf nodes store one body id; internal nodes have two children.
		bool IsLeaf() const { return left < 0 && right < 0; }
	};

	// Node pool allocation helpers.
	int AllocateNode();
	void ReleaseNode(int node);

	// Synchronize BVH leaves with live collider/body data.
	void SyncLeaves(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies);

	// BVH topology maintenance operations.
	void InsertLeaf(int leaf);
	void RemoveLeaf(int leaf);
	void FixUpwardTree(int node);

	std::vector<Node> m_nodes;
	std::vector<int> m_freeNodes;
	std::unordered_map<uint32_t, int> m_bodyToLeaf;
	int m_root = -1;
	float m_fatMargin = 0.08f;
};

} // namespace Physics
} // namespace Runtime
