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

	virtual std::vector<BroadPhasePair> ComputePairs(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies) = 0;
};

class DynamicBvhBroadPhase final : public BroadPhase {
public:
	DynamicBvhBroadPhase() = default;
	~DynamicBvhBroadPhase() override = default;

	std::vector<BroadPhasePair> ComputePairs(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies) override;

private:
	struct Node {
		AABB aabb;
		int parent = -1;
		int left = -1;
		int right = -1;
		uint32_t bodyId = 0;

		bool IsLeaf() const { return left < 0 && right < 0; }
	};

	int AllocateNode();

	void BuildTree(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies);

	void InsertLeaf(int leaf);
	void FixUpwardTree(int node);

	std::vector<Node> m_nodes;
	std::vector<int> m_leafNodes;
	int m_root = -1;
};

} // namespace Physics
} // namespace Runtime
