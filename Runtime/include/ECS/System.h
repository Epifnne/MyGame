// Runtime/include/ECS/System.h
#pragma once

#include <vector>
#include "Entity.h"

namespace Runtime {
namespace ECS {
class Registry;

class System {
public:
	virtual ~System() = default;
	// Called each frame with registry and delta time
	virtual void Update(Registry& registry, float dt) = 0;
};


} // namespace ECS
} // namespace Runtime
