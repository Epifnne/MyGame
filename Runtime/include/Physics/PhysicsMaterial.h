#pragma once

#include <algorithm>
#include <cmath>

namespace Runtime {
namespace Physics {

// Contact friction/restitution coefficients with geometric friction and averaged restitution blending.
struct PhysicsMaterial {
	// Coulomb friction coefficients and restitution.
	float staticFriction = 0.6f;
	float dynamicFriction = 0.5f;
	float restitution = 0.1f;

	// Blend each friction as sqrt(max(0, muA*muB)); restitution = clamp((eA+eB)/2, 0, 1).
	static PhysicsMaterial Combine(const PhysicsMaterial& a, const PhysicsMaterial& b) {
		PhysicsMaterial result;
		result.staticFriction = std::sqrt(std::max(0.0f, a.staticFriction * b.staticFriction));
		result.dynamicFriction = std::sqrt(std::max(0.0f, a.dynamicFriction * b.dynamicFriction));
		result.restitution = std::clamp((a.restitution + b.restitution) * 0.5f, 0.0f, 1.0f);
		return result;
	}

	// Compare all three float coefficients exactly for material-change detection.
	bool operator==(const PhysicsMaterial& other) const {
		return staticFriction == other.staticFriction &&
			dynamicFriction == other.dynamicFriction &&
			restitution == other.restitution;
	}
	// Return the negation of exact coefficient equality.
	bool operator!=(const PhysicsMaterial& other) const { return !(*this == other); }
};

} // namespace Physics
} // namespace Runtime
