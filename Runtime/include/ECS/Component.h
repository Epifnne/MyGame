// Runtime/include/ECS/Component.h
#pragma once

#include <glm/glm.hpp>

namespace Runtime {
namespace ECS {

namespace Components {

struct Transform2D {
	float x = 0.0f;
	float y = 0.0f;
};

struct Velocity2D {
	float vx = 0.0f;
	float vy = 0.0f;
};

struct Transform3D {
	glm::vec3 position = glm::vec3(0.0f);
	glm::vec3 rotationEuler = glm::vec3(0.0f);
	glm::vec3 scale = glm::vec3(1.0f);
};

struct Rotation {
	glm::vec3 axis = glm::vec3(0.0f, 1.0f, 0.0f);
	float angle = 0.0f;
	float speed = 1.0f;
};

} // namespace Components



} // namespace ECS
} // namespace Runtime
