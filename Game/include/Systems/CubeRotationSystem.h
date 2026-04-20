#pragma once

#include <ECS/World.h>

namespace Game {
namespace Systems {

class CubeRotationSystem {
public:
    static void Update(Runtime::ECS::World& world, float dt);
};

} // namespace Systems
} // namespace Game
