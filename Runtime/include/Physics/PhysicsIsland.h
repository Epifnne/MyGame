#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace Runtime {
namespace Physics {

// Island-build input: one solver-relevant contact between two bodies.
// Trigger contacts never form edges and are dropped by the builder. A contact
// with a static endpoint is assigned to the dynamic body's island (the static
// body is not a propagation node); static-static contacts are ignored.
struct IslandContact {
	uint32_t bodyA = 0;
	uint32_t bodyB = 0;
	bool isTrigger = false;
};

// Forward-star edge of the island graph. Edges of one body are chained through
// `next`; `to` is the compact dynamic-body index, or kNoNode for a
// dynamic-static boundary edge (the contact still belongs to the island).
struct IslandEdge {
	static constexpr uint32_t kNoNode = std::numeric_limits<uint32_t>::max();

	uint32_t to = kNoNode;
	uint32_t contact = 0;
	uint32_t next = kNoNode;
};

// One extracted physics island: the dynamic bodies it contains (body ids,
// ascending) and the contacts it owns (indices into the Build() contact
// array, ascending; trigger contacts excluded). The island list itself is
// ordered by each island's smallest body id, so identical inputs always
// produce identical output regardless of container iteration order.
struct PhysicsIsland {
	std::vector<uint32_t> bodies;
	std::vector<uint32_t> contacts;
};

struct IslandBuildStats {
	// Participating dynamic bodies (forward-star node count).
	std::size_t nodeCount = 0;
	// Forward-star edge count: 2 per dynamic-dynamic contact plus 1 per
	// dynamic-static contact, i.e. memory grows as O(V + E).
	std::size_t edgeCount = 0;
	std::size_t islandCount = 0;
	// Largest island's dynamic body count.
	std::size_t maxIslandBodyCount = 0;
};

// Phase 7: builds physics islands over the dynamic bodies that currently
// participate in solving, using a chained forward star (head array plus one
// continuous IslandEdge array). Extraction is an iterative BFS (no recursion);
// islands, bodies and contacts come out in stable sorted order. Internal
// buffers (and the per-island body/contact vectors) keep their capacity
// across frames, so steady-state builds perform no allocation growth.
//
// Kinematic bodies are out of scope for this version. Sleeping islands (Phase
// 9) keep their membership and contact adjacency by simply not being removed
// here: the caller decides which bodies participate, the builder never drops
// adjacency information on its own.
class PhysicsIslandBuilder {
public:
	PhysicsIslandBuilder() = default;

	// Rebuild the island set. dynamicBodyIds lists every dynamic body that
	// participates in solving this sub-step (contactless bodies included:
	// each forms a single-body island). The result reference stays valid
	// until the next Build call.
	const std::vector<PhysicsIsland>& Build(
		const std::vector<uint32_t>& dynamicBodyIds,
		const std::vector<IslandContact>& contacts);

	// Islands of the latest Build call.
	const std::vector<PhysicsIsland>& Islands() const { return m_islands; }
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
	// Reused output storage; per-island vectors keep capacity across builds.
	std::vector<PhysicsIsland> m_islands;
	IslandBuildStats m_stats;
};

} // namespace Physics
} // namespace Runtime
