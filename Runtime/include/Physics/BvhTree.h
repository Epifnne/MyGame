#pragma once

#include <cstdint>
#include <stack>
#include <unordered_map>
#include <vector>

#include "CollisionShape.h"

namespace Runtime {
namespace Physics {

// Incremental pooled BVH with fat body bounds, area-growth insertion, and ancestor refits.
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

	// Test whether the body-to-leaf map contains this identifier.
	bool Contains(uint32_t bodyId) const;
	// Return mapped leaf count, not the total node-pool size.
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

	// Clear the node pool and reinsert current fat leaves with the area-growth heuristic.
	void Rebuild();

	// DFS-prune disjoint stored fat bounds and emit matching leaves without sorting.
	template <typename OnLeaf>
	void QueryLeafOverlaps(const AABB& region, OnLeaf&& onLeaf) const {
		if (m_root < 0) {
			return;
		}
		std::stack<int> stack;
		stack.push(m_root);
		while (!stack.empty()) {
			const int nodeIndex = stack.top();
			stack.pop();
			if (nodeIndex < 0 || nodeIndex >= static_cast<int>(m_nodes.size())) {
				continue;
			}
			const Node& node = m_nodes[nodeIndex];
			if (!node.active || !node.aabb.Intersects(region)) {
				continue;
			}
			if (!node.IsLeaf()) {
				stack.push(node.left);
				stack.push(node.right);
				continue;
			}
			onLeaf(node.bodyId);
		}
	}

	// Store padding; AABB::Expanded clamps negative margins to zero during updates.
	void SetFatMargin(float margin) { m_fatMargin = margin; }
	// Return the configured, unclamped padding.
	float FatMargin() const { return m_fatMargin; }

	// Max tolerated surface-area growth ratio of one refit before it is upgraded
	// to a remove + reinsert. Non-positive disables the check (always refit).
	void SetRefitSurfaceAreaGrowthThreshold(float ratio) { m_refitGrowthThreshold = ratio; }
	// Return the fractional area-growth limit (newArea-oldArea)/oldArea.
	float RefitSurfaceAreaGrowthThreshold() const { return m_refitGrowthThreshold; }

	// Number of leaf remove + reinsert operations that triggers a full Rebuild().
	// Zero disables the automatic rebuild.
	void SetRebuildUpdateThreshold(uint32_t count) { m_rebuildUpdateThreshold = count; }
	// Return the reinsertion count limit.
	uint32_t RebuildUpdateThreshold() const { return m_rebuildUpdateThreshold; }

	// Metrics for the most recent query call. Surface areas let
	// callers compare the actual tree against a brute-force leaf AABB baseline.
	struct QueryMetrics {
		std::size_t visitedNodes = 0;
		std::size_t candidateLeafPairs = 0;
		double totalNodeSurfaceArea = 0.0;
		double leafSurfaceArea = 0.0;
	};
	// Copy instrumentation from the last self-query.
	QueryMetrics LastSelfQueryMetrics() const { return m_lastSelfQueryMetrics; }
	// Copy instrumentation from the last cross-query.
	QueryMetrics LastCrossQueryMetrics() const { return m_lastCrossQueryMetrics; }

	// Sum active-node surface areas; an empty tree returns zero.
	double TotalNodeSurfaceArea() const;
	// Sum active-leaf surface areas only.
	double LeafSurfaceArea() const;

	// Count reinsertions since the last rebuild; insertions/refits/removals are excluded.
	uint32_t UpdatesSinceRebuild() const { return m_updatesSinceRebuild; }

private:
	// Pooled fat-bound node; negative child indices identify a body leaf.
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

	// Reuse a free slot or append a reset active node.
	int AllocateNode();
	// Reset an active node and put its index on the free list.
	void ReleaseNode(int node);

	// Descend by minimal area growth and join the leaf with the selected sibling.
	void InsertLeaf(int leaf);
	// Detach a leaf, promote its sibling, and release the obsolete parent.
	void RemoveLeafNode(int leaf);
	// Merge child bounds on the path to the root.
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
