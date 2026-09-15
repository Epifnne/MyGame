#pragma once

#include <cstdint>
#include <stack>
#include <unordered_map>
#include <vector>

#include "CollisionShape.h"

namespace Runtime {
namespace Physics {

// Single incremental BVH over body leaves with fat AABBs.
// Extracted from DynamicBvhBroadPhase; the hybrid broad-phase owns two of them.
class BvhTree {
public:
	// Insert or refresh the leaf of bodyId:
	// - new body: insert a fat leaf.
	// - tight AABB still inside the stored fat AABB: refit in place, updating the
	//   leaf AABB and refitting ancestors without touching the topology.
	// - growth below RefitSurfaceAreaGrowthThreshold(): cheap refit keeps topology.
	// - growth above it, or the tight AABB leaves the fat AABB: remove + reinsert.
	void UpsertLeaf(uint32_t bodyId, const AABB& tightAabb);

	// Remove the leaf of bodyId if present.
	void RemoveLeaf(uint32_t bodyId);

	bool Contains(uint32_t bodyId) const;
	size_t LeafCount() const { return m_bodyToLeaf.size(); }

	// Snapshot of stored body ids for external stale-leaf pruning.
	std::vector<uint32_t> CollectBodyIds() const;

	// Emit each overlapping leaf pair inside this tree exactly once.
	// Records query metrics retrievable via LastSelfQueryMetrics().
	// Templated callback: the per-pair emit inlines into the query loop
	// (thousands of calls per frame on the broad-phase hot path).
	template <typename OnPair>
	void SelfQueryPairs(OnPair&& onPair) {
		m_lastSelfQueryMetrics = {};
		ComputeSurfaceAreas(m_lastSelfQueryMetrics.totalNodeSurfaceArea, m_lastSelfQueryMetrics.leafSurfaceArea);
		if (m_root < 0 || m_bodyToLeaf.size() < 2) {
			return;
		}

		for (const auto& entry : m_bodyToLeaf) {
			const int leafIndex = entry.second;
			if (leafIndex < 0 || leafIndex >= static_cast<int>(m_nodes.size())) {
				continue;
			}

			const Node& leaf = m_nodes[leafIndex];
			if (!leaf.active) {
				continue;
			}

			// Query current leaf against the whole tree using DFS.
			std::stack<int> stack;
			stack.push(m_root);
			while (!stack.empty()) {
				const int nodeIndex = stack.top();
				stack.pop();

				if (nodeIndex == leafIndex || nodeIndex < 0 || nodeIndex >= static_cast<int>(m_nodes.size())) {
					continue;
				}

				const Node& node = m_nodes[nodeIndex];
				if (!node.active) {
					continue;
				}
				++m_lastSelfQueryMetrics.visitedNodes;

				// Broad-phase pruning: skip disjoint AABBs early.
				if (!leaf.aabb.Intersects(node.aabb)) {
					continue;
				}

				if (!node.IsLeaf()) {
					// Internal node: keep descending until reaching candidate leaves.
					stack.push(node.left);
					stack.push(node.right);
					continue;
				}

				// Body id ordering emits each unordered pair exactly once.
				if (node.bodyId > leaf.bodyId) {
					++m_lastSelfQueryMetrics.candidateLeafPairs;
					onPair(leaf.bodyId, node.bodyId);
				}
			}
		}
	}

	// Emit every overlapping (leaf of this tree, leaf of other) pair.
	// Records query metrics on this tree (LastCrossQueryMetrics()).
	template <typename OnPair>
	void QueryPairsAgainst(const BvhTree& other, OnPair&& onPair) {
		m_lastCrossQueryMetrics = {};
		ComputeSurfaceAreas(m_lastCrossQueryMetrics.totalNodeSurfaceArea, m_lastCrossQueryMetrics.leafSurfaceArea);
		if (other.m_root < 0) {
			return;
		}

		for (const auto& entry : m_bodyToLeaf) {
			const int leafIndex = entry.second;
			if (leafIndex < 0 || leafIndex >= static_cast<int>(m_nodes.size())) {
				continue;
			}

			const Node& leaf = m_nodes[leafIndex];
			if (!leaf.active) {
				continue;
			}

			// Query each leaf of this tree against the other tree using DFS.
			std::stack<int> stack;
			stack.push(other.m_root);
			while (!stack.empty()) {
				const int nodeIndex = stack.top();
				stack.pop();
				if (nodeIndex < 0 || nodeIndex >= static_cast<int>(other.m_nodes.size())) {
					continue;
				}

				const Node& node = other.m_nodes[nodeIndex];
				if (!node.active) {
					continue;
				}
				++m_lastCrossQueryMetrics.visitedNodes;
				if (!leaf.aabb.Intersects(node.aabb)) {
					continue;
				}
				if (!node.IsLeaf()) {
					stack.push(node.left);
					stack.push(node.right);
					continue;
				}
				++m_lastCrossQueryMetrics.candidateLeafPairs;
				onPair(leaf.bodyId, node.bodyId);
			}
		}
	}

	// Rebuild the whole tree from current leaves via top-down median splitting.
	// Restores tree quality after many incremental updates degraded it.
	void Rebuild();

	void SetFatMargin(float margin) { m_fatMargin = margin; }
	float FatMargin() const { return m_fatMargin; }

	// Max tolerated surface-area growth ratio of one refit before it is upgraded
	// to a remove + reinsert. Non-positive disables the check (always refit).
	void SetRefitSurfaceAreaGrowthThreshold(float ratio) { m_refitGrowthThreshold = ratio; }
	float RefitSurfaceAreaGrowthThreshold() const { return m_refitGrowthThreshold; }

	// Number of leaf remove + reinsert operations that triggers a full Rebuild().
	// Zero disables the automatic rebuild.
	void SetRebuildUpdateThreshold(uint32_t count) { m_rebuildUpdateThreshold = count; }
	uint32_t RebuildUpdateThreshold() const { return m_rebuildUpdateThreshold; }

	// Chaos metrics: sums over the most recent query call. Surface areas let
	// callers compare the actual tree against a brute-force leaf AABB baseline.
	struct QueryMetrics {
		std::size_t visitedNodes = 0;
		std::size_t candidateLeafPairs = 0;
		double totalNodeSurfaceArea = 0.0;
		double leafSurfaceArea = 0.0;
	};
	QueryMetrics LastSelfQueryMetrics() const { return m_lastSelfQueryMetrics; }
	QueryMetrics LastCrossQueryMetrics() const { return m_lastCrossQueryMetrics; }

	// Raw quality gauges, valid whenever the tree is non-empty.
	double TotalNodeSurfaceArea() const;
	double LeafSurfaceArea() const;

	uint32_t UpdatesSinceRebuild() const { return m_updatesSinceRebuild; }

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

	// BVH topology maintenance operations.
	void InsertLeaf(int leaf);
	void RemoveLeafNode(int leaf);
	void FixUpwardTree(int node);

	// Update a leaf AABB in place and refit its ancestors; topology untouched.
	void RefitLeaf(int leaf, const AABB& aabb);

	// Compute total/leaf surface areas without touching query instrumentation.
	void ComputeSurfaceAreas(double& total, double& leaves) const;

	std::vector<Node> m_nodes;
	std::vector<int> m_freeNodes;
	std::unordered_map<uint32_t, int> m_bodyToLeaf;
	int m_root = -1;
	float m_fatMargin = 0.08f;
	float m_refitGrowthThreshold = 0.25f;
	uint32_t m_rebuildUpdateThreshold = 128;
	uint32_t m_updatesSinceRebuild = 0;
	QueryMetrics m_lastSelfQueryMetrics;
	QueryMetrics m_lastCrossQueryMetrics;
};

} // namespace Physics
} // namespace Runtime
