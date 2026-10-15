#include "Systems/RotationSystem.h"

#include "Components/RotationComponent.h"
#include <ECS/Registry.h>

namespace Game {
namespace Systems {

void RotationSystem::Update(Runtime::ECS::Registry& registry, float dt) {
    registry.View<Components::RotationComponent>().Each(
        [dt](Runtime::ECS::Entity, Components::RotationComponent& rotation) {
        rotation.angle += rotation.speed * dt;
    });
}

} // namespace Systems
} // namespace Game
