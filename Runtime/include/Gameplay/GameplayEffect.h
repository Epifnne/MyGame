#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "AttributeSet.h"
#include "GameplayTags.h"

namespace Runtime {
namespace Gameplay {

struct GameplayEffectModifier {
    std::string attributeName;
    AttributeModifier modifier;
};

struct GameplayEffectSpec {
    std::string effectName;
    float durationSeconds = -1.0f;
    std::vector<GameplayEffectModifier> modifiers;
    std::vector<GameplayTag> grantedTags;
};

struct ActiveGameplayEffect {
    uint64_t instanceId = 0;
    uint64_t sourceId = 0;
    float remainingSeconds = -1.0f;
    GameplayEffectSpec spec;
};

} // namespace Gameplay
} // namespace Runtime
