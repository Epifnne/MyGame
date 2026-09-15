#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "AttributeSet.h"
#include "GameplayEffect.h"
#include "GameplayTags.h"
#include "ECS/Entity.h"

namespace Runtime {
namespace Gameplay {

class GameplaySystem {
public:
    using EffectInstanceId = uint64_t;

    struct EntityGameplayState {
        AttributeSet attributes;
        std::unordered_map<uint32_t, uint32_t> tagRefCounts;
        std::vector<ActiveGameplayEffect> activeEffects;
    };

    void EnsureEntity(ECS::Entity entity);
    void RemoveEntity(ECS::Entity entity);
    bool HasEntity(ECS::Entity entity) const;

    void SetBaseAttribute(ECS::Entity entity, const std::string& attributeName, float value);
    float GetCurrentAttribute(ECS::Entity entity, const std::string& attributeName) const;

    void AddTag(ECS::Entity entity, GameplayTag tag);
    void RemoveTag(ECS::Entity entity, GameplayTag tag);
    bool HasTag(ECS::Entity entity, GameplayTag tag) const;

    EffectInstanceId ApplyEffect(ECS::Entity entity, const GameplayEffectSpec& effectSpec, uint64_t sourceId = 0);
    bool RemoveEffect(ECS::Entity entity, EffectInstanceId effectInstanceId);

    void Tick(float deltaTime);

    const EntityGameplayState* GetState(ECS::Entity entity) const;

private:
    EntityGameplayState& GetOrCreateState(ECS::Entity entity);
    bool RemoveEffectInternal(EntityGameplayState& state, EffectInstanceId effectInstanceId);

    EffectInstanceId m_nextEffectInstanceId = 1;
    std::unordered_map<ECS::Entity, EntityGameplayState> m_entityStates;
};

} // namespace Gameplay
} // namespace Runtime
