#pragma once

#include <cstdint>
#include <glm/glm.hpp>

namespace Runtime {
namespace Physics {

struct ContactPoint {
	glm::vec3 position = glm::vec3(0.0f);
	float penetration = 0.0f;
	float normalImpulse = 0.0f;
};

struct ContactManifold {
	uint32_t bodyA = 0;
	uint32_t bodyB = 0;
	glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
	ContactPoint point;
	bool isTrigger = false;
};

} // namespace Physics
} // namespace Runtime
