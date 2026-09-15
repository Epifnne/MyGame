#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace Runtime {
namespace Gameplay {

enum class AttributeModifierOperation : uint8_t {
    Add = 0,
    Multiply = 1,
    Override = 2
};

struct AttributeModifier {
    AttributeModifierOperation operation = AttributeModifierOperation::Add;
    float value = 0.0f;
    uint64_t sourceId = 0;
};

class AttributeSet {
public:
    void SetBaseValue(const std::string& attributeName, float value);
    float GetBaseValue(const std::string& attributeName) const;

    void AddModifier(const std::string& attributeName, const AttributeModifier& modifier);
    void RemoveModifiersBySource(uint64_t sourceId);
    void ClearModifiers(const std::string& attributeName);

    float GetCurrentValue(const std::string& attributeName) const;

private:
    struct AttributeEntry {
        float baseValue = 0.0f;
        std::vector<AttributeModifier> modifiers;
    };

    std::unordered_map<std::string, AttributeEntry> m_attributes;
};

} // namespace Gameplay
} // namespace Runtime
