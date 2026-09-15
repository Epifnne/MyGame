#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>

namespace Runtime {
namespace AI {
namespace BehaviorTree {

using BlackboardValue = std::variant<std::monostate, bool, int64_t, double, std::string>;

class Blackboard {
public:
    void Clear();
    [[nodiscard]] bool HasKey(const std::string& key) const;
    void RemoveKey(const std::string& key);

    template<typename T>
    void SetValue(const std::string& key, T value) {
        m_values[key] = BlackboardValue(value);
    }

    template<typename T>
    [[nodiscard]] std::optional<T> GetValue(const std::string& key) const {
        auto found = m_values.find(key);
        if (found == m_values.end()) {
            return std::nullopt;
        }

        const T* value = std::get_if<T>(&found->second);
        if (!value) {
            return std::nullopt;
        }
        return *value;
    }

private:
    std::unordered_map<std::string, BlackboardValue> m_values;
};

} // namespace BehaviorTree
} // namespace AI
} // namespace Runtime
