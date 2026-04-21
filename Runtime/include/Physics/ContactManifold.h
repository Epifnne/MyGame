#pragma once

#include <cstdint>
#include <glm/glm.hpp>

namespace Runtime {
namespace Physics {

struct ContactPoint {
	// Contact position, penetration depth, and cached normal impulse.
	glm::vec3 position = glm::vec3(0.0f);
	float penetration = 0.0f;
	float normalImpulse = 0.0f;
};

struct ContactManifold {
	// Body pair, contact normal, and a single representative point.
	uint32_t bodyA = 0;
	uint32_t bodyB = 0;
	glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
	ContactPoint point;
	// Trigger contacts skip impulse solving.
	bool isTrigger = false;
};

} // namespace Physics
} // namespace Runtime
