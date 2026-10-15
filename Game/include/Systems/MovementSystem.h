#pragma once

#include <ECS/World.h>
#include <ECS/Entity.h>
#include "Components/TransformComponent.h"
#include "Components/VelocityComponent.h"

namespace Game {
namespace Systems {

class MovementSystem {
public:
    static void Update(Components::TransformComponent& transform,
                       const Components::VelocityComponent& velocity, float dt);
    static void UpdateEntity(Runtime::ECS::World& world, Runtime::ECS::Entity entity, float dt);
};

} // namespace Systems
} // namespace Game
