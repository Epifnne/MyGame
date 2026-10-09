#include "Physics/BroadPhase.h"

#include <algorithm>

namespace Runtime {
namespace Physics {

namespace {

// Evaluate shape bounds at the body's current pose.
AABB ComputeBodyAabb(const Collider& collider, const RigidBody& body) {
    ShapeTransform tf;
    tf.position = body.Position();
    tf.orientation = body.Orientation();
    return collider.ComputeAABB(tf);
}

} // namespace

// Sort endpoints without rejecting equal identifiers.
PairKey MakePairKey(uint32_t a, uint32_t b) {
    PairKey key;
    key.bodyA = std::min(a, b);
    key.bodyB = std::max(a, b);
    return key;
}

// Synchronize and query the mixed tree; fewer than two colliders bypass synchronization.
BroadPhaseQueryResult DynamicBvhBroadPhase::ComputePairs(
    const std::unordered_map<uint32_t, Collider>& colliders,
    const std::unordered_map<uint32_t, RigidBody>& bodies) {
    BroadPhaseQueryResult result;
    // Only pairs with an awake dynamic endpoint belong to the query coverage.
    result.coverage = BroadPhaseQueryCoverage::ActiveDynamics;
    if (colliders.size() < 2) {
        return result;
    }

    // Drop leaves whose collider or body disappeared, then refresh all leaves.
    for (const uint32_t bodyId : m_tree.CollectBodyIds()) {
        if (colliders.find(bodyId) == colliders.end() || bodies.find(bodyId) == bodies.end()) {
            m_tree.RemoveLeaf(bodyId);
        }
    }
    for (const auto& kv : colliders) {
        const auto bodyIt = bodies.find(kv.first);
        if (bodyIt == bodies.end()) {
            continue;
        }
        m_tree.UpsertLeaf(kv.first, ComputeBodyAabb(kv.second, bodyIt->second));
    }

    // Reject missing inputs, incompatible masks, and pairs with no active endpoint.
    m_tree.SelfQueryPairs([&](uint32_t idA, uint32_t idB) {
        const auto colliderItA = colliders.find(idA);
        const auto colliderItB = colliders.find(idB);
        const auto bodyItA = bodies.find(idA);
        const auto bodyItB = bodies.find(idB);
        if (colliderItA == colliders.end() || colliderItB == colliders.end() ||
            bodyItA == bodies.end() || bodyItB == bodies.end()) {
            return;
        }

        // Apply layer/group mask filtering.
        if (!colliderItA->second.CanCollideWith(colliderItB->second)) {
            return;
        }

        // Pairs without an awake dynamic endpoint (static-static included)
        // are never generated: sleeping bodies do not actively query.
        if (!BroadPhaseBodyIsActive(bodyItA->second) && !BroadPhaseBodyIsActive(bodyItB->second)) {
            return;
        }

        result.pairs.push_back(MakePairKey(idA, idB));
    });

    std::sort(result.pairs.begin(), result.pairs.end());
    result.pairs.erase(std::unique(result.pairs.begin(), result.pairs.end()), result.pairs.end());
    return result;
}

// Append non-static matches from the mixed tree without clearing the destination.
void DynamicBvhBroadPhase::CollectDynamicLeafOverlaps(
    const AABB& region,
    const std::unordered_map<uint32_t, RigidBody>& bodies,
    std::vector<uint32_t>& outBodyIds) const {
    // The single tree mixes static and dynamic leaves; filter by body.
    m_tree.QueryLeafOverlaps(region, [&](uint32_t bodyId) {
        const auto bodyIt = bodies.find(bodyId);
        if (bodyIt != bodies.end() && !bodyIt->second.IsStatic()) {
            outBodyIds.push_back(bodyId);
        }
    });
}

// Synchronize split membership, then merge filtered self/cross-query candidates.
BroadPhaseQueryResult HybridBvhBroadPhase::ComputePairs(
    const std::unordered_map<uint32_t, Collider>& colliders,
    const std::unordered_map<uint32_t, RigidBody>& bodies) {
    BroadPhaseQueryResult result;
    // Only pairs with an awake dynamic endpoint belong to the query coverage.
    result.coverage = BroadPhaseQueryCoverage::ActiveDynamics;

    // Remove leaves whose collider or body disappeared from either tree.
    for (const uint32_t bodyId : m_staticTree.CollectBodyIds()) {
        if (colliders.find(bodyId) == colliders.end() || bodies.find(bodyId) == bodies.end()) {
            m_staticTree.RemoveLeaf(bodyId);
        }
    }
    for (const uint32_t bodyId : m_dynamicTree.CollectBodyIds()) {
        if (colliders.find(bodyId) == colliders.end() || bodies.find(bodyId) == bodies.end()) {
            m_dynamicTree.RemoveLeaf(bodyId);
        }
    }

    // Refresh fat bounds even for static leaves; UpsertLeaf decides refit versus reinsertion.
    for (const auto& kv : colliders) {
        const uint32_t bodyId = kv.first;
        const auto bodyIt = bodies.find(bodyId);
        if (bodyIt == bodies.end()) {
            continue;
        }

        // Static/dynamic switch migrates the leaf without leaving residuals.
        const bool isStatic = bodyIt->second.IsStatic();
        if (isStatic && m_dynamicTree.Contains(bodyId)) {
            m_dynamicTree.RemoveLeaf(bodyId);
        }
        if (!isStatic && m_staticTree.Contains(bodyId)) {
            m_staticTree.RemoveLeaf(bodyId);
        }

        const AABB tight = ComputeBodyAabb(kv.second, bodyIt->second);
        (isStatic ? m_staticTree : m_dynamicTree).UpsertLeaf(bodyId, tight);
    }

    // Emit only mask-compatible pairs with at least one awake dynamic endpoint.
    const auto tryEmit = [&](uint32_t idA, uint32_t idB) {
        const auto colliderItA = colliders.find(idA);
        const auto colliderItB = colliders.find(idB);
        if (colliderItA == colliders.end() || colliderItB == colliders.end()) {
            return;
        }

        // Apply layer/group mask filtering. Static-static pairs are impossible
        // by construction and need no explicit check.
        if (!colliderItA->second.CanCollideWith(colliderItB->second)) {
            return;
        }

        // Tree traversal includes sleeping leaves; reject pairs with no active endpoint.
        const auto bodyItA = bodies.find(idA);
        const auto bodyItB = bodies.find(idB);
        if (bodyItA == bodies.end() || bodyItB == bodies.end()) {
            return;
        }
        if (!BroadPhaseBodyIsActive(bodyItA->second) && !BroadPhaseBodyIsActive(bodyItB->second)) {
            return;
        }
        result.pairs.push_back(MakePairKey(idA, idB));
    };

    // Dynamic-dynamic self query plus dynamic-static cross query. Sleeping
    // bodies keep their dynamic leaves but are filtered above.
    m_dynamicTree.SelfQueryPairs(tryEmit);
    m_dynamicTree.QueryPairsAgainst(m_staticTree, tryEmit);

    std::sort(result.pairs.begin(), result.pairs.end());
    result.pairs.erase(std::unique(result.pairs.begin(), result.pairs.end()), result.pairs.end());
    return result;
}

// Append dynamic-tree matches, including sleeping leaves; the body map is unused.
void HybridBvhBroadPhase::CollectDynamicLeafOverlaps(
    const AABB& region,
    const std::unordered_map<uint32_t, RigidBody>& bodies,
    std::vector<uint32_t>& outBodyIds) const {
    // The dynamic tree holds exactly the dynamic bodies (sleeping included).
    (void)bodies;
    m_dynamicTree.QueryLeafOverlaps(region, [&](uint32_t bodyId) {
        outBodyIds.push_back(bodyId);
    });
}

} // namespace Physics
} // namespace Runtime
