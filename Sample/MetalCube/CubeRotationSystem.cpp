#include "CubeRotationSystem.h"

#include "CubeRenderComponent.h"
#include <ECS/Component.h>

namespace Sample {
namespace Systems {

using Runtime::ECS::Components::Rotation;
using Runtime::ECS::Components::Transform3D;

void CubeRotationSystem::Update(Runtime::ECS::World& world, float dt) {
    world.View<Rotation, Transform3D, Components::CubeRenderComponent>().Each(
        [dt](Runtime::ECS::Entity, Rotation& rotation, Transform3D& transform,
             Components::CubeRenderComponent&) {
        rotation.angle += rotation.speed * dt;
        transform.rotationEuler = rotation.axis * rotation.angle;
    });
}

} // namespace Systems
} // namespace Sample
