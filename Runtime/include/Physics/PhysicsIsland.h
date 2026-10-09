#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace Runtime {
namespace Physics {

// Body-pair build input; triggers are ignored and only supplied dynamic ids become graph nodes.
struct IslandContact {
	uint32_t bodyA = 0;
	uint32_t bodyB = 0;
	bool isTrigger = false;
};

// Forward-star edge: compact destination node, contact input index and next edge in the source chain.
// kNoNode marks an endpoint omitted from the supplied dynamic node set.
struct IslandEdge {
	static constexpr uint32_t kNoNode = std::numeric_limits<uint32_t>::max();

	uint32_t to = kNoNode;
	uint32_t contact = 0;
	uint32_t next = kNoNode;
};

// Connected dynamic-body component: ascending body ids and contact indices ordered by pair key, then index.
// Components are emitted in ascending smallest-body-id order; triggers never belong to them.
struct PhysicsIsland {
	std::vector<uint32_t> bodies;
	std::vector<uint32_t> contacts;
};

// Counts from the latest graph build, including isolated dynamic nodes and directed boundary edges.
struct IslandBuildStats {
	// Participating dynamic bodies (forward-star node count).
	std::size_t nodeCount = 0;
	// Two edges when both endpoints are nodes, one when exactly one endpoint is a node.
	std::size_t edgeCount = 0;
	std::size_t islandCount = 0;
	// Largest island's dynamic body count.
	std::size_t maxIslandBodyCount = 0;
};

// Builds forward-star components using iterative depth-first traversal and reusable graph buffers.
// Membership is determined solely by supplied body ids/contacts; this builder does not inspect sleep state.
class PhysicsIslandBuilder {
public:
	// Initialize empty graph/output buffers and zero build statistics.
	PhysicsIslandBuilder() = default;

	// Rebuild from unique dynamic ids; isolated ids form single-body islands.
	// Ignore triggers and contacts with no node endpoints; external endpoints do not join components.
	// Return builder-owned output that is replaced by the next Build.
	const std::vector<PhysicsIsland>& Build(
		const std::vector<uint32_t>& dynamicBodyIds,
		const std::vector<IslandContact>& contacts);

	// Return builder-owned components from the latest Build.
	const std::vector<PhysicsIsland>& Islands() const { return m_islands; }
	// Return node/edge/component counts and largest component size from the latest Build.
	const IslandBuildStats& LastStats() const { return m_stats; }

private:
	static constexpr uint32_t kInvalidEdge = std::numeric_limits<uint32_t>::max();

	// Compact node indexing: sorted dynamic id array plus id -> node map.
	std::vector<uint32_t> m_nodeIds;
	std::unordered_map<uint32_t, uint32_t> m_idToNode;
	// Chained forward star.
	std::vector<uint32_t> m_head;
	std::vector<IslandEdge> m_edges;
	// Traversal scratch.
	std::vector<uint8_t> m_visited;
	std::vector<uint8_t> m_contactClaimed;
	std::vector<uint32_t> m_bfsStack;
	// Surviving output vectors retain capacity; shrinking the island list destroys discarded vectors.
	std::vector<PhysicsIsland> m_islands;
	IslandBuildStats m_stats;
};

} // namespace Physics
} // namespace Runtime
