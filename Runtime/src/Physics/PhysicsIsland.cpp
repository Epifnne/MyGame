#include "Physics/PhysicsIsland.h"

#include <algorithm>

namespace Runtime {
namespace Physics {

const std::vector<PhysicsIsland>& PhysicsIslandBuilder::Build(
	const std::vector<uint32_t>& dynamicBodyIds,
	const std::vector<IslandContact>& contacts) {
	// Compact indexing: nodes are the participating dynamic bodies in
	// ascending id order, which also defines the stable output order.
	m_nodeIds = dynamicBodyIds;
	std::sort(m_nodeIds.begin(), m_nodeIds.end());
	m_idToNode.clear();
	m_idToNode.reserve(m_nodeIds.size());
	for (uint32_t node = 0; node < m_nodeIds.size(); ++node) {
		m_idToNode.emplace(m_nodeIds[node], node);
	}

	// Chained forward star. Dynamic-dynamic non-trigger contacts write both
	// directions; dynamic-static contacts write a single boundary edge on the
	// dynamic body (static bodies are never propagation nodes).
	m_head.assign(m_nodeIds.size(), kInvalidEdge);
	m_edges.clear();
	m_edges.reserve(contacts.size() * 2);
	m_contactClaimed.assign(contacts.size(), 0);
	for (uint32_t contactIndex = 0; contactIndex < contacts.size(); ++contactIndex) {
		const IslandContact& contact = contacts[contactIndex];
		if (contact.isTrigger) {
			continue;
		}
		const auto itA = m_idToNode.find(contact.bodyA);
		const auto itB = m_idToNode.find(contact.bodyB);
		const bool hasA = itA != m_idToNode.end();
		const bool hasB = itB != m_idToNode.end();
		if (hasA && hasB) {
			m_edges.push_back({itB->second, contactIndex, m_head[itA->second]});
			m_head[itA->second] = static_cast<uint32_t>(m_edges.size() - 1);
			m_edges.push_back({itA->second, contactIndex, m_head[itB->second]});
			m_head[itB->second] = static_cast<uint32_t>(m_edges.size() - 1);
		} else if (hasA || hasB) {
			const uint32_t node = hasA ? itA->second : itB->second;
			m_edges.push_back({IslandEdge::kNoNode, contactIndex, m_head[node]});
			m_head[node] = static_cast<uint32_t>(m_edges.size() - 1);
		}
		// Static-static contacts carry no island information; skip them.
	}

	// Iterative BFS seeded in ascending id order: the first unvisited seed is
	// always its island's smallest id, so islands come out sorted by minimum
	// body id without an extra pass.
	m_visited.assign(m_nodeIds.size(), 0);
	std::size_t islandCount = 0;
	std::size_t maxIslandBodyCount = 0;
	for (uint32_t seed = 0; seed < m_nodeIds.size(); ++seed) {
		if (m_visited[seed]) {
			continue;
		}
		if (islandCount < m_islands.size()) {
			m_islands[islandCount].bodies.clear();
			m_islands[islandCount].contacts.clear();
		} else {
			m_islands.emplace_back();
		}
		PhysicsIsland& island = m_islands[islandCount];
		++islandCount;

		m_bfsStack.clear();
		m_visited[seed] = 1;
		m_bfsStack.push_back(seed);
		while (!m_bfsStack.empty()) {
			const uint32_t node = m_bfsStack.back();
			m_bfsStack.pop_back();
			island.bodies.push_back(m_nodeIds[node]);
			for (uint32_t edgeIndex = m_head[node]; edgeIndex != kInvalidEdge;
				 edgeIndex = m_edges[edgeIndex].next) {
				const IslandEdge& edge = m_edges[edgeIndex];
				// Each non-trigger contact is claimed exactly once: a
				// dynamic-dynamic contact appears in both endpoints' chains,
				// the claim flag keeps it in a single island's contact list.
				if (!m_contactClaimed[edge.contact]) {
					m_contactClaimed[edge.contact] = 1;
					island.contacts.push_back(edge.contact);
				}
				if (edge.to != IslandEdge::kNoNode && !m_visited[edge.to]) {
					m_visited[edge.to] = 1;
					m_bfsStack.push_back(edge.to);
				}
			}
		}
		std::sort(island.bodies.begin(), island.bodies.end());
		std::sort(island.contacts.begin(), island.contacts.end());
		maxIslandBodyCount = std::max(maxIslandBodyCount, island.bodies.size());
	}
	m_islands.resize(islandCount);

	m_stats.nodeCount = m_nodeIds.size();
	m_stats.edgeCount = m_edges.size();
	m_stats.islandCount = islandCount;
	m_stats.maxIslandBodyCount = maxIslandBodyCount;
	return m_islands;
}

} // namespace Physics
} // namespace Runtime
