#pragma once

#include <algorithm>
#include <cmath>

namespace Runtime {
namespace Physics {

struct PhysicsMaterial {
	float staticFriction = 0.6f;
	float dynamicFriction = 0.5f;
	float restitution = 0.1f;

	static PhysicsMaterial Combine(const PhysicsMaterial& a, const PhysicsMaterial& b) {
		PhysicsMaterial result;
		result.staticFriction = std::sqrt(std::max(0.0f, a.staticFriction * b.staticFriction));
		result.dynamicFriction = std::sqrt(std::max(0.0f, a.dynamicFriction * b.dynamicFriction));
		result.restitution = std::clamp((a.restitution + b.restitution) * 0.5f, 0.0f, 1.0f);
		return result;
	}
};

} // namespace Physics
} // namespace Runtime
