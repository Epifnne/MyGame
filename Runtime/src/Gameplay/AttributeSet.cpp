#include "Gameplay/AttributeSet.h"

#include <algorithm>

namespace Runtime {
namespace Gameplay {

void AttributeSet::SetBaseValue(const std::string& attributeName, float value) {
    m_attributes[attributeName].baseValue = value;
}

float AttributeSet::GetBaseValue(const std::string& attributeName) const {
    auto found = m_attributes.find(attributeName);
    if (found == m_attributes.end()) {
        return 0.0f;
    }
    return found->second.baseValue;
}

void AttributeSet::AddModifier(const std::string& attributeName, const AttributeModifier& modifier) {
    m_attributes[attributeName].modifiers.push_back(modifier);
}

void AttributeSet::RemoveModifiersBySource(uint64_t sourceId) {
    for (auto& kv : m_attributes) {
        auto& modifiers = kv.second.modifiers;
        modifiers.erase(
            std::remove_if(
                modifiers.begin(),
                modifiers.end(),
                [sourceId](const AttributeModifier& modifier) { return modifier.sourceId == sourceId; }),
            modifiers.end());
    }
}

void AttributeSet::ClearModifiers(const std::string& attributeName) {
    auto found = m_attributes.find(attributeName);
    if (found == m_attributes.end()) {
        return;
    }
    found->second.modifiers.clear();
}

float AttributeSet::GetCurrentValue(const std::string& attributeName) const {
    auto found = m_attributes.find(attributeName);
    if (found == m_attributes.end()) {
        return 0.0f;
    }

    const AttributeEntry& entry = found->second;
    float result = entry.baseValue;

    for (const AttributeModifier& modifier : entry.modifiers) {
        if (modifier.operation == AttributeModifierOperation::Add) {
            result += modifier.value;
        }
    }

    for (const AttributeModifier& modifier : entry.modifiers) {
        if (modifier.operation == AttributeModifierOperation::Multiply) {
            result *= modifier.value;
        }
    }

    for (const AttributeModifier& modifier : entry.modifiers) {
        if (modifier.operation == AttributeModifierOperation::Override) {
            result = modifier.value;
        }
    }

    return result;
}

} // namespace Gameplay
} // namespace Runtime
