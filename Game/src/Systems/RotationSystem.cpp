#include "Systems/RotationSystem.h"

#include "Components/RotationComponent.h"
#include <ECS/Registry.h>

namespace Game {
namespace Systems {

void RotationSystem::Update(Runtime::ECS::Registry& registry, float dt) {
    auto entities = registry.EntitiesWith<Components::RotationComponent>();
    for (auto entity : entities) {
        auto& rotation = registry.GetComponent<Components::RotationComponent>(entity);
        rotation.angle += rotation.speed * dt;
    }
}

} // namespace Systems
} // namespace Game
