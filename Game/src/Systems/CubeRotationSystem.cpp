#include "Systems/CubeRotationSystem.h"

#include "Components/CubeRenderComponent.h"
#include "Components/RotationComponent.h"
#include "Components/Transform3DComponent.h"

namespace Game {
namespace Systems {

void CubeRotationSystem::Update(Runtime::ECS::World& world, float dt) {
    auto entities = world.RegistryRef().EntitiesWith<Components::RotationComponent>();
    for (auto entity : entities) {
        if (!world.RegistryRef().HasComponent<Components::Transform3DComponent>(entity)) {
            continue;
        }
        if (!world.RegistryRef().HasComponent<Components::CubeRenderComponent>(entity)) {
            continue;
        }

        auto& transform = world.RegistryRef().GetComponent<Components::Transform3DComponent>(entity);
        auto& rotation = world.RegistryRef().GetComponent<Components::RotationComponent>(entity);
        rotation.angle += rotation.speed * dt;
        transform.rotationEuler = rotation.axis * rotation.angle;
    }
}

} // namespace Systems
} // namespace Game
