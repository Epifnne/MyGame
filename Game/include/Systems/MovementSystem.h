#pragma once

#include <ECS/World.h>
#include <ECS/Entity.h>

namespace Game {
namespace Systems {

class MovementSystem {
public:
    static void UpdateEntity(Runtime::ECS::World& world, Runtime::ECS::Entity entity, float dt);
};

} // namespace Systems
} // namespace Game
