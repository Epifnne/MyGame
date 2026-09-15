#include "Physics/BroadPhase.h"

#include <algorithm>

namespace Runtime {
namespace Physics {

namespace {

AABB ComputeBodyAabb(const Collider& collider, const RigidBody& body) {
    ShapeTransform tf;
    tf.position = body.Position();
    tf.orientation = body.Orientation();
    return collider.ComputeAABB(tf);
}

} // namespace

PairKey MakePairKey(uint32_t a, uint32_t b) {
    PairKey key;
    key.bodyA = std::min(a, b);
    key.bodyB = std::max(a, b);
    return key;
}

BroadPhaseQueryResult DynamicBvhBroadPhase::ComputePairs(
    const std::unordered_map<uint32_t, Collider>& colliders,
    const std::unordered_map<uint32_t, RigidBody>& bodies) {
    BroadPhaseQueryResult result;
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

        // Ignore static-static pairs to reduce narrow-phase work.
        if (bodyItA->second.IsStatic() && bodyItB->second.IsStatic()) {
            return;
        }

        result.pairs.push_back(MakePairKey(idA, idB));
    });

    std::sort(result.pairs.begin(), result.pairs.end());
    result.pairs.erase(std::unique(result.pairs.begin(), result.pairs.end()), result.pairs.end());
    return result;
}

BroadPhaseQueryResult HybridBvhBroadPhase::ComputePairs(
    const std::unordered_map<uint32_t, Collider>& colliders,
    const std::unordered_map<uint32_t, RigidBody>& bodies) {
    BroadPhaseQueryResult result;

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

    // Synchronize membership and leaves. Static leaves only reinsert when their
    // tight AABB exits the stored fat AABB (BvhTree::UpsertLeaf), so static
    // transform/collider changes invalidate the tree while untouched statics
    // keep their leaves untouched.
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
        result.pairs.push_back(MakePairKey(idA, idB));
    };

    // Dynamic-dynamic self query plus dynamic-static cross query. Before the
    // sleep phase lands, every dynamic body is treated as active.
    m_dynamicTree.SelfQueryPairs(tryEmit);
    m_dynamicTree.QueryPairsAgainst(m_staticTree, tryEmit);

    std::sort(result.pairs.begin(), result.pairs.end());
    result.pairs.erase(std::unique(result.pairs.begin(), result.pairs.end()), result.pairs.end());
    return result;
}

} // namespace Physics
} // namespace Runtime
