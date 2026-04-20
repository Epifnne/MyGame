#pragma once

#include <ECS/System.h>

namespace Game {
namespace Systems {

class RotationSystem : public Runtime::ECS::System {
public:
    void Update(Runtime::ECS::Registry& registry, float dt) override;
};

} // namespace Systems
} // namespace Game
