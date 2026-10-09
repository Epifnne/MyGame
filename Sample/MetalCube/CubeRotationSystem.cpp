#include "CubeRotationSystem.h"

#include "CubeRenderComponent.h"
#include <ECS/Component.h>

namespace Sample {
namespace Systems {

using Runtime::ECS::Components::Rotation;
using Runtime::ECS::Components::Transform3D;

void CubeRotationSystem::Update(Runtime::ECS::World& world, float dt) {
    auto entities = world.RegistryRef().EntitiesWith<Rotation>();
    for (auto entity : entities) {
        if (!world.RegistryRef().HasComponent<Transform3D>(entity)) {
            continue;
        }
        if (!world.RegistryRef().HasComponent<Components::CubeRenderComponent>(entity)) {
            continue;
        }

        auto& transform = world.RegistryRef().GetComponent<Transform3D>(entity);
        auto& rotation = world.RegistryRef().GetComponent<Rotation>(entity);
        rotation.angle += rotation.speed * dt;
        transform.rotationEuler = rotation.axis * rotation.angle;
    }
}

} // namespace Systems
} // namespace Sample
