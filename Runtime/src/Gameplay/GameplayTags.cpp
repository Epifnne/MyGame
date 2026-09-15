#include "Gameplay/GameplayTags.h"

#include <unordered_map>

namespace Runtime {
namespace Gameplay {

namespace {

std::unordered_map<std::string, uint32_t>& TagToValue() {
    static std::unordered_map<std::string, uint32_t> tags;
    return tags;
}

std::unordered_map<uint32_t, std::string>& ValueToTag() {
    static std::unordered_map<uint32_t, std::string> names;
    return names;
}

uint32_t& NextTagValue() {
    static uint32_t next = 1;
    return next;
}

} // namespace

GameplayTagRegistry& GameplayTagRegistry::Instance() noexcept {
    static GameplayTagRegistry registry;
    return registry;
}

GameplayTag GameplayTagRegistry::RegisterTag(const std::string& name) {
    auto& map = TagToValue();
    auto found = map.find(name);
    if (found != map.end()) {
        return GameplayTag{found->second};
    }

    const uint32_t value = NextTagValue()++;
    map[name] = value;
    ValueToTag()[value] = name;
    return GameplayTag{value};
}

GameplayTag GameplayTagRegistry::FindTag(const std::string& name) const {
    const auto& map = TagToValue();
    auto found = map.find(name);
    if (found == map.end()) {
        return GameplayTag{};
    }
    return GameplayTag{found->second};
}

std::string GameplayTagRegistry::FindName(GameplayTag tag) const {
    const auto& map = ValueToTag();
    auto found = map.find(tag.value);
    if (found == map.end()) {
        return std::string();
    }
    return found->second;
}

} // namespace Gameplay
} // namespace Runtime
