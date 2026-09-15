#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "AI/BehaviorTree/Blackboard.h"
#include "ECS/Entity.h"

namespace Runtime {
namespace AI {
namespace BehaviorTree {

enum class NodeStatus : uint8_t {
    Success = 0,
    Failure = 1,
    Running = 2
};

struct BehaviorContext {
    float deltaTime = 0.0f;
    ECS::Entity entity = ECS::NullEntity;
    Blackboard* blackboard = nullptr;
    void* userData = nullptr;
};

class BehaviorNode {
public:
    virtual ~BehaviorNode() = default;
    virtual NodeStatus Tick(BehaviorContext& context) = 0;
    virtual void Reset() {}
};

class CompositeNode : public BehaviorNode {
public:
    void AddChild(std::unique_ptr<BehaviorNode> child);

protected:
    std::vector<std::unique_ptr<BehaviorNode>> m_children;
};

class SequenceNode : public CompositeNode {
public:
    NodeStatus Tick(BehaviorContext& context) override;
    void Reset() override;

private:
    size_t m_runningChildIndex = 0;
};

class SelectorNode : public CompositeNode {
public:
    NodeStatus Tick(BehaviorContext& context) override;
    void Reset() override;

private:
    size_t m_runningChildIndex = 0;
};

class InverterNode : public BehaviorNode {
public:
    explicit InverterNode(std::unique_ptr<BehaviorNode> child);

    NodeStatus Tick(BehaviorContext& context) override;
    void Reset() override;

private:
    std::unique_ptr<BehaviorNode> m_child;
};

class RepeatNode : public BehaviorNode {
public:
    RepeatNode(std::unique_ptr<BehaviorNode> child, int32_t repeatCount);

    NodeStatus Tick(BehaviorContext& context) override;
    void Reset() override;

private:
    std::unique_ptr<BehaviorNode> m_child;
    int32_t m_repeatCount = -1;
    int32_t m_completedCount = 0;
};

using NodeCallback = std::function<NodeStatus(BehaviorContext& context)>;

class ConditionNode : public BehaviorNode {
public:
    explicit ConditionNode(NodeCallback callback);

    NodeStatus Tick(BehaviorContext& context) override;

private:
    NodeCallback m_callback;
};

class ActionNode : public BehaviorNode {
public:
    explicit ActionNode(NodeCallback callback);

    NodeStatus Tick(BehaviorContext& context) override;

private:
    NodeCallback m_callback;
};

} // namespace BehaviorTree
} // namespace AI
} // namespace Runtime
