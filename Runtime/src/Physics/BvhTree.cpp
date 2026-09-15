#include "Physics/BvhTree.h"

namespace Runtime {
namespace Physics {

namespace {

bool AabbContains(const AABB& outer, const AABB& inner) {
    return outer.min.x <= inner.min.x && outer.min.y <= inner.min.y && outer.min.z <= inner.min.z &&
           outer.max.x >= inner.max.x && outer.max.y >= inner.max.y && outer.max.z >= inner.max.z;
}

} // namespace

int BvhTree::AllocateNode() {
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

void BvhTree::ReleaseNode(int node) {
    if (node < 0 || node >= static_cast<int>(m_nodes.size())) {
        return;
    }

    if (!m_nodes[node].active) {
        return;
    }

    m_nodes[node] = Node{};
    m_freeNodes.push_back(node);
}

void BvhTree::UpsertLeaf(uint32_t bodyId, const AABB& tightAabb) {
    const AABB fat = tightAabb.Expanded(m_fatMargin);

    auto leafIt = m_bodyToLeaf.find(bodyId);
    if (leafIt == m_bodyToLeaf.end()) {
        const int leaf = AllocateNode();
        m_nodes[leaf].aabb = fat;
        m_nodes[leaf].bodyId = bodyId;
        InsertLeaf(leaf);
        m_bodyToLeaf[bodyId] = leaf;
        return;
    }

    const int leaf = leafIt->second;
    if (leaf < 0 || leaf >= static_cast<int>(m_nodes.size()) || !m_nodes[leaf].active) {
        const int newLeaf = AllocateNode();
        m_nodes[newLeaf].aabb = fat;
        m_nodes[newLeaf].bodyId = bodyId;
        InsertLeaf(newLeaf);
        leafIt->second = newLeaf;
        return;
    }

    // Fast path: the tight AABB still fits the stored fat AABB. Refit in place,
    // keeping topology. Only escalate to a remove + reinsert when the tight
    // AABB leaves the fat AABB, or when the fat-AABB surface-area growth of one
    // refit exceeds the configured threshold.
    if (AabbContains(m_nodes[leaf].aabb, tightAabb)) {
        const float oldArea = m_nodes[leaf].aabb.SurfaceArea();
        const float newArea = fat.SurfaceArea();
        const float growth = (oldArea > 1e-8f) ? (newArea - oldArea) / oldArea : 0.0f;
        if (m_refitGrowthThreshold <= 0.0f || growth <= m_refitGrowthThreshold) {
            RefitLeaf(leaf, fat);
            return;
        }
    }

    RemoveLeafNode(leaf);
    m_nodes[leaf].aabb = fat;
    m_nodes[leaf].bodyId = bodyId;
    InsertLeaf(leaf);

    // Chaos accounting: a remove + reinsert degrades the tree; too many since
    // the last rebuild triggers a full quality rebuild.
    ++m_updatesSinceRebuild;
    if (m_rebuildUpdateThreshold > 0 && m_updatesSinceRebuild >= m_rebuildUpdateThreshold) {
        Rebuild();
    }
}

void BvhTree::RemoveLeaf(uint32_t bodyId) {
    const auto leafIt = m_bodyToLeaf.find(bodyId);
    if (leafIt == m_bodyToLeaf.end()) {
        return;
    }
    const int leaf = leafIt->second;
    RemoveLeafNode(leaf);
    ReleaseNode(leaf);
    m_bodyToLeaf.erase(leafIt);
}

bool BvhTree::Contains(uint32_t bodyId) const {
    return m_bodyToLeaf.find(bodyId) != m_bodyToLeaf.end();
}

std::vector<uint32_t> BvhTree::CollectBodyIds() const {
    std::vector<uint32_t> ids;
    ids.reserve(m_bodyToLeaf.size());
    for (const auto& entry : m_bodyToLeaf) {
        ids.push_back(entry.first);
    }
    return ids;
}

void BvhTree::InsertLeaf(int leaf) {
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

void BvhTree::RemoveLeafNode(int leaf) {
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

void BvhTree::FixUpwardTree(int node) {
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

void BvhTree::RefitLeaf(int leaf, const AABB& aabb) {
    if (leaf < 0 || leaf >= static_cast<int>(m_nodes.size()) || !m_nodes[leaf].active) {
        return;
    }
    m_nodes[leaf].aabb = aabb;
    FixUpwardTree(m_nodes[leaf].parent);
}

void BvhTree::Rebuild() {
    // Snapshot the leaf data, then rebuild from an empty tree. Starting from an
    // empty node pool avoids index collisions between fresh internal nodes and
    // not-yet-reinserted leaves that would corrupt the topology (cycles).
    struct LeafData {
        uint32_t bodyId;
        AABB aabb;
    };
    std::vector<LeafData> leaves;
    leaves.reserve(m_bodyToLeaf.size());
    for (const auto& entry : m_bodyToLeaf) {
        const int leaf = entry.second;
        if (leaf >= 0 && leaf < static_cast<int>(m_nodes.size()) && m_nodes[leaf].active) {
            leaves.push_back({entry.first, m_nodes[leaf].aabb});
        }
    }

    m_nodes.clear();
    m_freeNodes.clear();
    m_bodyToLeaf.clear();
    m_root = -1;

    for (const LeafData& data : leaves) {
        const int leaf = AllocateNode();
        m_nodes[leaf].aabb = data.aabb;
        m_nodes[leaf].bodyId = data.bodyId;
        InsertLeaf(leaf);
        m_bodyToLeaf[data.bodyId] = leaf;
    }
    m_updatesSinceRebuild = 0;
}

void BvhTree::ComputeSurfaceAreas(double& total, double& leaves) const {
    total = 0.0;
    leaves = 0.0;
    for (const Node& node : m_nodes) {
        if (!node.active) {
            continue;
        }
        const double area = static_cast<double>(node.aabb.SurfaceArea());
        total += area;
        if (node.IsLeaf()) {
            leaves += area;
        }
    }
}

double BvhTree::TotalNodeSurfaceArea() const {
    double total = 0.0;
    double leaves = 0.0;
    ComputeSurfaceAreas(total, leaves);
    return total;
}

double BvhTree::LeafSurfaceArea() const {
    double total = 0.0;
    double leaves = 0.0;
    ComputeSurfaceAreas(total, leaves);
    return leaves;
}

} // namespace Physics
} // namespace Runtime
