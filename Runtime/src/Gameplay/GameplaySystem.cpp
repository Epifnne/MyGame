#include "Gameplay/GameplaySystem.h"

#include <algorithm>

namespace Runtime {
namespace Gameplay {

void GameplaySystem::EnsureEntity(ECS::Entity entity) {
    if (entity == ECS::NullEntity) {
        return;
    }
    m_entityStates.try_emplace(entity);
}

void GameplaySystem::RemoveEntity(ECS::Entity entity) {
    if (entity == ECS::NullEntity) {
        return;
    }
    m_entityStates.erase(entity);
}

bool GameplaySystem::HasEntity(ECS::Entity entity) const {
    if (entity == ECS::NullEntity) {
        return false;
    }
    return m_entityStates.find(entity) != m_entityStates.end();
}

void GameplaySystem::SetBaseAttribute(ECS::Entity entity, const std::string& attributeName, float value) {
    if (entity == ECS::NullEntity) {
        return;
    }
    GetOrCreateState(entity).attributes.SetBaseValue(attributeName, value);
}

float GameplaySystem::GetCurrentAttribute(ECS::Entity entity, const std::string& attributeName) const {
    auto found = m_entityStates.find(entity);
    if (found == m_entityStates.end()) {
        return 0.0f;
    }
    return found->second.attributes.GetCurrentValue(attributeName);
}

void GameplaySystem::AddTag(ECS::Entity entity, GameplayTag tag) {
    if (entity == ECS::NullEntity || !tag.IsValid()) {
        return;
    }
    auto& counts = GetOrCreateState(entity).tagRefCounts;
    counts[tag.value] += 1;
}

void GameplaySystem::RemoveTag(ECS::Entity entity, GameplayTag tag) {
    if (entity == ECS::NullEntity || !tag.IsValid()) {
        return;
    }

    auto foundState = m_entityStates.find(entity);
    if (foundState == m_entityStates.end()) {
        return;
    }

    auto& counts = foundState->second.tagRefCounts;
    auto foundTag = counts.find(tag.value);
    if (foundTag == counts.end()) {
        return;
    }

    if (foundTag->second <= 1) {
        counts.erase(foundTag);
        return;
    }

    foundTag->second -= 1;
}

bool GameplaySystem::HasTag(ECS::Entity entity, GameplayTag tag) const {
    if (entity == ECS::NullEntity || !tag.IsValid()) {
        return false;
    }

    auto foundState = m_entityStates.find(entity);
    if (foundState == m_entityStates.end()) {
        return false;
    }

    auto foundTag = foundState->second.tagRefCounts.find(tag.value);
    if (foundTag == foundState->second.tagRefCounts.end()) {
        return false;
    }

    return foundTag->second > 0;
}

GameplaySystem::EffectInstanceId GameplaySystem::ApplyEffect(
    ECS::Entity entity,
    const GameplayEffectSpec& effectSpec,
    uint64_t sourceId) {
    if (entity == ECS::NullEntity) {
        return 0;
    }

    EntityGameplayState& state = GetOrCreateState(entity);

    const EffectInstanceId instanceId = m_nextEffectInstanceId++;

    ActiveGameplayEffect active;
    active.instanceId = instanceId;
    active.sourceId = sourceId;
    active.remainingSeconds = effectSpec.durationSeconds;
    active.spec = effectSpec;
    state.activeEffects.push_back(active);

    for (const GameplayEffectModifier& mod : effectSpec.modifiers) {
        AttributeModifier applied = mod.modifier;
        applied.sourceId = instanceId;
        state.attributes.AddModifier(mod.attributeName, applied);
    }

    for (GameplayTag tag : effectSpec.grantedTags) {
        if (!tag.IsValid()) {
            continue;
        }
        state.tagRefCounts[tag.value] += 1;
    }

    return instanceId;
}

bool GameplaySystem::RemoveEffect(ECS::Entity entity, EffectInstanceId effectInstanceId) {
    auto found = m_entityStates.find(entity);
    if (found == m_entityStates.end()) {
        return false;
    }

    return RemoveEffectInternal(found->second, effectInstanceId);
}

void GameplaySystem::Tick(float deltaTime) {
    if (deltaTime <= 0.0f) {
        return;
    }

    for (auto& kv : m_entityStates) {
        EntityGameplayState& state = kv.second;

        std::vector<EffectInstanceId> expired;
        for (ActiveGameplayEffect& effect : state.activeEffects) {
            if (effect.remainingSeconds < 0.0f) {
                continue;
            }

            effect.remainingSeconds -= deltaTime;
            if (effect.remainingSeconds <= 0.0f) {
                expired.push_back(effect.instanceId);
            }
        }

        for (EffectInstanceId id : expired) {
            RemoveEffectInternal(state, id);
        }
    }
}

const GameplaySystem::EntityGameplayState* GameplaySystem::GetState(ECS::Entity entity) const {
    auto found = m_entityStates.find(entity);
    if (found == m_entityStates.end()) {
        return nullptr;
    }
    return &found->second;
}

GameplaySystem::EntityGameplayState& GameplaySystem::GetOrCreateState(ECS::Entity entity) {
    return m_entityStates.try_emplace(entity).first->second;
}

bool GameplaySystem::RemoveEffectInternal(EntityGameplayState& state, EffectInstanceId effectInstanceId) {
    auto found = std::find_if(
        state.activeEffects.begin(),
        state.activeEffects.end(),
        [effectInstanceId](const ActiveGameplayEffect& effect) {
            return effect.instanceId == effectInstanceId;
        });

    if (found == state.activeEffects.end()) {
        return false;
    }

    state.attributes.RemoveModifiersBySource(effectInstanceId);

    for (GameplayTag tag : found->spec.grantedTags) {
        if (!tag.IsValid()) {
            continue;
        }

        auto tagIt = state.tagRefCounts.find(tag.value);
        if (tagIt == state.tagRefCounts.end()) {
            continue;
        }

        if (tagIt->second <= 1) {
            state.tagRefCounts.erase(tagIt);
        } else {
            tagIt->second -= 1;
        }
    }

    state.activeEffects.erase(found);
    return true;
}

} // namespace Gameplay
} // namespace Runtime
