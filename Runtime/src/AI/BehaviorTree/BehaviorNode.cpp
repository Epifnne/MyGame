#include "AI/BehaviorTree/BehaviorNode.h"

#include <utility>

namespace Runtime {
namespace AI {
namespace BehaviorTree {

void CompositeNode::AddChild(std::unique_ptr<BehaviorNode> child) {
    if (!child) {
        return;
    }
    m_children.push_back(std::move(child));
}

NodeStatus SequenceNode::Tick(BehaviorContext& context) {
    if (m_children.empty()) {
        return NodeStatus::Success;
    }

    for (size_t i = m_runningChildIndex; i < m_children.size(); ++i) {
        NodeStatus status = m_children[i]->Tick(context);
        if (status == NodeStatus::Running) {
            m_runningChildIndex = i;
            return NodeStatus::Running;
        }
        if (status == NodeStatus::Failure) {
            m_runningChildIndex = 0;
            return NodeStatus::Failure;
        }
    }

    m_runningChildIndex = 0;
    return NodeStatus::Success;
}

void SequenceNode::Reset() {
    m_runningChildIndex = 0;
    for (const auto& child : m_children) {
        child->Reset();
    }
}

NodeStatus SelectorNode::Tick(BehaviorContext& context) {
    if (m_children.empty()) {
        return NodeStatus::Failure;
    }

    for (size_t i = m_runningChildIndex; i < m_children.size(); ++i) {
        NodeStatus status = m_children[i]->Tick(context);
        if (status == NodeStatus::Running) {
            m_runningChildIndex = i;
            return NodeStatus::Running;
        }
        if (status == NodeStatus::Success) {
            m_runningChildIndex = 0;
            return NodeStatus::Success;
        }
    }

    m_runningChildIndex = 0;
    return NodeStatus::Failure;
}

void SelectorNode::Reset() {
    m_runningChildIndex = 0;
    for (const auto& child : m_children) {
        child->Reset();
    }
}

InverterNode::InverterNode(std::unique_ptr<BehaviorNode> child)
    : m_child(std::move(child)) {}

NodeStatus InverterNode::Tick(BehaviorContext& context) {
    if (!m_child) {
        return NodeStatus::Failure;
    }

    NodeStatus status = m_child->Tick(context);
    if (status == NodeStatus::Success) {
        return NodeStatus::Failure;
    }
    if (status == NodeStatus::Failure) {
        return NodeStatus::Success;
    }
    return NodeStatus::Running;
}

void InverterNode::Reset() {
    if (m_child) {
        m_child->Reset();
    }
}

RepeatNode::RepeatNode(std::unique_ptr<BehaviorNode> child, int32_t repeatCount)
    : m_child(std::move(child)),
      m_repeatCount(repeatCount) {}

NodeStatus RepeatNode::Tick(BehaviorContext& context) {
    if (!m_child) {
        return NodeStatus::Failure;
    }

    if (m_repeatCount >= 0 && m_completedCount >= m_repeatCount) {
        return NodeStatus::Success;
    }

    NodeStatus status = m_child->Tick(context);
    if (status == NodeStatus::Running) {
        return NodeStatus::Running;
    }

    if (status == NodeStatus::Failure) {
        m_completedCount = 0;
        return NodeStatus::Failure;
    }

    ++m_completedCount;
    m_child->Reset();

    if (m_repeatCount >= 0 && m_completedCount >= m_repeatCount) {
        m_completedCount = 0;
        return NodeStatus::Success;
    }

    return NodeStatus::Running;
}

void RepeatNode::Reset() {
    m_completedCount = 0;
    if (m_child) {
        m_child->Reset();
    }
}

ConditionNode::ConditionNode(NodeCallback callback)
    : m_callback(std::move(callback)) {}

NodeStatus ConditionNode::Tick(BehaviorContext& context) {
    if (!m_callback) {
        return NodeStatus::Failure;
    }
    return m_callback(context);
}

ActionNode::ActionNode(NodeCallback callback)
    : m_callback(std::move(callback)) {}

NodeStatus ActionNode::Tick(BehaviorContext& context) {
    if (!m_callback) {
        return NodeStatus::Failure;
    }
    return m_callback(context);
}

} // namespace BehaviorTree
} // namespace AI
} // namespace Runtime
