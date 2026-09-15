#include "AI/BehaviorTree/BehaviorTree.h"

#include <utility>

namespace Runtime {
namespace AI {
namespace BehaviorTree {

void BehaviorTree::SetRoot(std::unique_ptr<BehaviorNode> root) {
    m_root = std::move(root);
}

NodeStatus BehaviorTree::Tick(float deltaTime, ECS::Entity entity, void* userData) {
    if (!m_root) {
        return NodeStatus::Failure;
    }

    BehaviorContext context;
    context.deltaTime = deltaTime;
    context.entity = entity;
    context.blackboard = &m_blackboard;
    context.userData = userData;

    return m_root->Tick(context);
}

void BehaviorTree::Reset() {
    if (m_root) {
        m_root->Reset();
    }
}

} // namespace BehaviorTree
} // namespace AI
} // namespace Runtime
