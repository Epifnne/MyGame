#include "AI/BehaviorTree/Blackboard.h"

namespace Runtime {
namespace AI {
namespace BehaviorTree {

void Blackboard::Clear() {
    m_values.clear();
}

bool Blackboard::HasKey(const std::string& key) const {
    return m_values.contains(key);
}

void Blackboard::RemoveKey(const std::string& key) {
    m_values.erase(key);
}

} // namespace BehaviorTree
} // namespace AI
} // namespace Runtime
