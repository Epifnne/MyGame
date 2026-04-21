#include "Physics/BroadPhase.h"

#include <algorithm>
#include <stack>
#include <unordered_set>

namespace Runtime {
namespace Physics {

std::vector<BroadPhasePair> DynamicBvhBroadPhase::ComputePairs(
    const std::unordered_map<uint32_t, Collider>& colliders,
    const std::unordered_map<uint32_t, RigidBody>& bodies) {
    std::vector<BroadPhasePair> pairs;
    if (colliders.size() < 2) {
        return pairs;
    }

    BuildTree(colliders, bodies);

    std::unordered_set<uint64_t> unique;
    unique.reserve(colliders.size() * 2);

    for (int leafIndex : m_leafNodes) {
        if (leafIndex < 0 || leafIndex >= static_cast<int>(m_nodes.size())) {
            continue;
        }

        const Node& leaf = m_nodes[leafIndex];
        std::stack<int> stack;
        if (m_root >= 0) {
            stack.push(m_root);
        }

        while (!stack.empty()) {
            const int nodeIndex = stack.top();
            stack.pop();

            if (nodeIndex == leafIndex || nodeIndex < 0 || nodeIndex >= static_cast<int>(m_nodes.size())) {
                continue;
            }

            const Node& node = m_nodes[nodeIndex];
            if (!leaf.aabb.Intersects(node.aabb)) {
                continue;
            }

            if (!node.IsLeaf()) {
                stack.push(node.left);
                stack.push(node.right);
                continue;
            }

            const uint32_t idA = leaf.bodyId;
            const uint32_t idB = node.bodyId;
            if (idA == idB) {
                continue;
            }

            auto colliderItA = colliders.find(idA);
            auto colliderItB = colliders.find(idB);
            auto bodyItA = bodies.find(idA);
            auto bodyItB = bodies.find(idB);
            if (colliderItA == colliders.end() || colliderItB == colliders.end() ||
                bodyItA == bodies.end() || bodyItB == bodies.end()) {
                continue;
            }

            if (!colliderItA->second.CanCollideWith(colliderItB->second)) {
                continue;
            }

            if (bodyItA->second.IsStatic() && bodyItB->second.IsStatic()) {
                continue;
            }

            const uint32_t lo = std::min(idA, idB);
            const uint32_t hi = std::max(idA, idB);
            const uint64_t key = (static_cast<uint64_t>(lo) << 32) | static_cast<uint64_t>(hi);
            if (unique.insert(key).second) {
                pairs.emplace_back(lo, hi);
            }
        }
    }

    return pairs;
}

int DynamicBvhBroadPhase::AllocateNode() {
    m_nodes.push_back(Node{});
    return static_cast<int>(m_nodes.size()) - 1;
}

void DynamicBvhBroadPhase::BuildTree(
    const std::unordered_map<uint32_t, Collider>& colliders,
    const std::unordered_map<uint32_t, RigidBody>& bodies) {
    m_nodes.clear();
    m_leafNodes.clear();
    m_root = -1;

    for (const auto& kv : colliders) {
        const uint32_t bodyId = kv.first;
        auto bodyIt = bodies.find(bodyId);
        if (bodyIt == bodies.end()) {
            continue;
        }

        ShapeTransform tf;
        tf.position = bodyIt->second.Position();
        tf.orientation = bodyIt->second.Orientation();
        const AABB fat = kv.second.ComputeAABB(tf).Expanded(0.08f);

        const int leaf = AllocateNode();
        m_nodes[leaf].aabb = fat;
        m_nodes[leaf].bodyId = bodyId;
        m_leafNodes.push_back(leaf);
        InsertLeaf(leaf);
    }
}

void DynamicBvhBroadPhase::InsertLeaf(int leaf) {
    if (m_root < 0) {
        m_root = leaf;
        m_nodes[leaf].parent = -1;
        return;
    }

    int sibling = m_root;
    while (!m_nodes[sibling].IsLeaf()) {
        const int left = m_nodes[sibling].left;
        const int right = m_nodes[sibling].right;

        const AABB mergedLeft = AABB::Merge(m_nodes[left].aabb, m_nodes[leaf].aabb);
        const AABB mergedRight = AABB::Merge(m_nodes[right].aabb, m_nodes[leaf].aabb);

        const float costLeft = mergedLeft.SurfaceArea() - m_nodes[left].aabb.SurfaceArea();
        const float costRight = mergedRight.SurfaceArea() - m_nodes[right].aabb.SurfaceArea();

        sibling = (costLeft < costRight) ? left : right;
    }

    const int oldParent = m_nodes[sibling].parent;
    const int newParent = AllocateNode();
    m_nodes[newParent].parent = oldParent;
    m_nodes[newParent].aabb = AABB::Merge(m_nodes[sibling].aabb, m_nodes[leaf].aabb);
    m_nodes[newParent].left = sibling;
    m_nodes[newParent].right = leaf;

    m_nodes[sibling].parent = newParent;
    m_nodes[leaf].parent = newParent;

    if (oldParent < 0) {
        m_root = newParent;
    } else {
        if (m_nodes[oldParent].left == sibling) {
            m_nodes[oldParent].left = newParent;
        } else {
            m_nodes[oldParent].right = newParent;
        }
    }

    FixUpwardTree(newParent);
}

void DynamicBvhBroadPhase::FixUpwardTree(int node) {
    int current = node;
    while (current >= 0) {
        if (!m_nodes[current].IsLeaf()) {
            const int left = m_nodes[current].left;
            const int right = m_nodes[current].right;
            m_nodes[current].aabb = AABB::Merge(m_nodes[left].aabb, m_nodes[right].aabb);
        }
        current = m_nodes[current].parent;
    }
}

} // namespace Physics
} // namespace Runtime
