#include "Systems/MovementSystem.h"

#include "Components/TransformComponent.h"
#include "Components/VelocityComponent.h"

namespace Game {
namespace Systems {

void MovementSystem::UpdateEntity(Runtime::ECS::World& world, Runtime::ECS::Entity entity, float dt) {
    auto& transform = world.RegistryRef().GetComponent<Components::TransformComponent>(entity);
    const auto& velocity = world.RegistryRef().GetComponent<Components::VelocityComponent>(entity);

    transform.x += velocity.vx * dt;
    transform.y += velocity.vy * dt;
}

} // namespace Systems
} // namespace Game
