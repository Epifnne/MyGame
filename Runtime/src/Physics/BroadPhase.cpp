#include "Physics/BroadPhase.h"

#include <algorithm>
#include <stack>
#include <unordered_set>

namespace Runtime {
namespace Physics {

namespace {

bool AabbContains(const AABB& outer, const AABB& inner) {
    return outer.min.x <= inner.min.x && outer.min.y <= inner.min.y && outer.min.z <= inner.min.z &&
           outer.max.x >= inner.max.x && outer.max.y >= inner.max.y && outer.max.z >= inner.max.z;
}

AABB ComputeBodyAabb(const Collider& collider, const RigidBody& body) {
    ShapeTransform tf;
    tf.position = body.Position();
    tf.orientation = body.Orientation();
    return collider.ComputeAABB(tf);
}

} // namespace

std::vector<BroadPhasePair> DynamicBvhBroadPhase::ComputePairs(
    const std::unordered_map<uint32_t, Collider>& colliders,
    const std::unordered_map<uint32_t, RigidBody>& bodies) {
    std::vector<BroadPhasePair> pairs;
    if (colliders.size() < 2) {
        return pairs;
    }

    // Incrementally synchronize tree leaves with current collider/body states.
    SyncLeaves(colliders, bodies);
    if (m_root < 0 || m_bodyToLeaf.size() < 2) {
        return pairs;
    }

    // Deduplicate potential pairs across multiple tree traversal paths.
    std::unordered_set<uint64_t> unique;
    unique.reserve(m_bodyToLeaf.size() * 2);

    for (const auto& entry : m_bodyToLeaf) {
        const int leafIndex = entry.second;
        if (leafIndex < 0 || leafIndex >= static_cast<int>(m_nodes.size())) {
            continue;
        }

        const Node& leaf = m_nodes[leafIndex];
        if (!leaf.active) {
            continue;
        }

        std::stack<int> stack;
        if (m_root >= 0) {
            // Query current leaf against the whole tree using DFS.
            stack.push(m_root);
        }

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

            // Apply layer/group mask filtering.
            if (!colliderItA->second.CanCollideWith(colliderItB->second)) {
                continue;
            }

            // Ignore static-static pairs to reduce narrow-phase work.
            if (bodyItA->second.IsStatic() && bodyItB->second.IsStatic()) {
                continue;
            }

            // Store pair in canonical order so (A,B) and (B,A) map to one key.
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
    if (!m_freeNodes.empty()) {
        const int reused = m_freeNodes.back();
        m_freeNodes.pop_back();
        m_nodes[reused] = Node{};
        m_nodes[reused].active = true;
        return reused;
    }

    m_nodes.push_back(Node{});
    m_nodes.back().active = true;
    return static_cast<int>(m_nodes.size()) - 1;
}

void DynamicBvhBroadPhase::ReleaseNode(int node) {
    if (node < 0 || node >= static_cast<int>(m_nodes.size())) {
        return;
    }

    if (!m_nodes[node].active) {
        return;
    }

    m_nodes[node] = Node{};
    m_freeNodes.push_back(node);
}

void DynamicBvhBroadPhase::SyncLeaves(
    const std::unordered_map<uint32_t, Collider>& colliders,
    const std::unordered_map<uint32_t, RigidBody>& bodies) {
    // Remove leaves whose bodies/colliders no longer exist.
    for (auto it = m_bodyToLeaf.begin(); it != m_bodyToLeaf.end();) {
        const uint32_t bodyId = it->first;
        if (colliders.find(bodyId) == colliders.end() || bodies.find(bodyId) == bodies.end()) {
            const int leaf = it->second;
            RemoveLeaf(leaf);
            ReleaseNode(leaf);
            it = m_bodyToLeaf.erase(it);
            continue;
        }
        ++it;
    }

    // Insert new leaves and reinsert moved leaves when they exit their fat AABB.
    for (const auto& kv : colliders) {
        const uint32_t bodyId = kv.first;
        auto bodyIt = bodies.find(bodyId);
        if (bodyIt == bodies.end()) {
            continue;
        }

        const AABB tight = ComputeBodyAabb(kv.second, bodyIt->second);
        const AABB fat = tight.Expanded(m_fatMargin);

        auto leafIt = m_bodyToLeaf.find(bodyId);
        if (leafIt == m_bodyToLeaf.end()) {
            const int leaf = AllocateNode();
            m_nodes[leaf].aabb = fat;
            m_nodes[leaf].bodyId = bodyId;
            InsertLeaf(leaf);
            m_bodyToLeaf[bodyId] = leaf;
            continue;
        }

        const int leaf = leafIt->second;
        if (leaf < 0 || leaf >= static_cast<int>(m_nodes.size()) || !m_nodes[leaf].active) {
            const int newLeaf = AllocateNode();
            m_nodes[newLeaf].aabb = fat;
            m_nodes[newLeaf].bodyId = bodyId;
            InsertLeaf(newLeaf);
            leafIt->second = newLeaf;
            continue;
        }

        if (!AabbContains(m_nodes[leaf].aabb, tight)) {
            RemoveLeaf(leaf);
            m_nodes[leaf].aabb = fat;
            m_nodes[leaf].bodyId = bodyId;
            InsertLeaf(leaf);
        }
    }
}

void DynamicBvhBroadPhase::InsertLeaf(int leaf) {
    if (leaf < 0 || leaf >= static_cast<int>(m_nodes.size()) || !m_nodes[leaf].active) {
        return;
    }

    m_nodes[leaf].left = -1;
    m_nodes[leaf].right = -1;

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

        // Heuristic insertion: descend toward child with lower area growth.
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
        // Sibling used to be root: new parent becomes the new root.
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

void DynamicBvhBroadPhase::RemoveLeaf(int leaf) {
    if (leaf < 0 || leaf >= static_cast<int>(m_nodes.size()) || !m_nodes[leaf].active) {
        return;
    }

    if (leaf == m_root) {
        m_root = -1;
        m_nodes[leaf].parent = -1;
        return;
    }

    const int parent = m_nodes[leaf].parent;
    if (parent < 0 || parent >= static_cast<int>(m_nodes.size()) || !m_nodes[parent].active) {
        m_nodes[leaf].parent = -1;
        return;
    }

    const int grandParent = m_nodes[parent].parent;
    const int sibling = (m_nodes[parent].left == leaf) ? m_nodes[parent].right : m_nodes[parent].left;

    if (grandParent < 0) {
        m_root = sibling;
        if (sibling >= 0 && sibling < static_cast<int>(m_nodes.size()) && m_nodes[sibling].active) {
            m_nodes[sibling].parent = -1;
        }
    } else {
        if (m_nodes[grandParent].left == parent) {
            m_nodes[grandParent].left = sibling;
        } else {
            m_nodes[grandParent].right = sibling;
        }

        if (sibling >= 0 && sibling < static_cast<int>(m_nodes.size()) && m_nodes[sibling].active) {
            m_nodes[sibling].parent = grandParent;
        }

        FixUpwardTree(grandParent);
    }

    m_nodes[leaf].parent = -1;
    ReleaseNode(parent);
}

void DynamicBvhBroadPhase::FixUpwardTree(int node) {
    int current = node;
    while (current >= 0) {
        if (current >= static_cast<int>(m_nodes.size()) || !m_nodes[current].active) {
            break;
        }

        if (!m_nodes[current].IsLeaf()) {
            const int left = m_nodes[current].left;
            const int right = m_nodes[current].right;
            if (left < 0 || right < 0 || left >= static_cast<int>(m_nodes.size()) ||
                right >= static_cast<int>(m_nodes.size()) || !m_nodes[left].active || !m_nodes[right].active) {
                break;
            }

            // Refit ancestor bounds after local topology/leaf changes.
            m_nodes[current].aabb = AABB::Merge(m_nodes[left].aabb, m_nodes[right].aabb);
        }
        current = m_nodes[current].parent;
    }
}

} // namespace Physics
} // namespace Runtime
