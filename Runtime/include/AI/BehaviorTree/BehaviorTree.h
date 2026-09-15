#pragma once

#include <memory>

#include "AI/BehaviorTree/BehaviorNode.h"

namespace Runtime {
namespace AI {
namespace BehaviorTree {

class BehaviorTree {
public:
    void SetRoot(std::unique_ptr<BehaviorNode> root);

    [[nodiscard]] NodeStatus Tick(float deltaTime, ECS::Entity entity, void* userData = nullptr);
    void Reset();

    [[nodiscard]] Blackboard& Data() { return m_blackboard; }
    [[nodiscard]] const Blackboard& Data() const { return m_blackboard; }

private:
    Blackboard m_blackboard;
    std::unique_ptr<BehaviorNode> m_root;
};

} // namespace BehaviorTree
} // namespace AI
} // namespace Runtime
