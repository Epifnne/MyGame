#pragma once

#include <glm/glm.hpp>

#include "RigidBody.h"

namespace Runtime {
namespace Physics {

// Interface for advancing a rigid body's velocity and pose.
class Integrator {
public:
	// Destroy a concrete integrator through the interface.
	virtual ~Integrator() = default;
	// Integrate one rigid body state for dt.
	virtual void Integrate(RigidBody& body, float dt, const glm::vec3& gravity) const = 0;
};

// Semi-implicit Euler: update v first, then x' = x + dt * v'.
class SemiImplicitEulerIntegrator final : public Integrator {
public:
	// Delegate to rigid body's semi-implicit Euler integration.
	void Integrate(RigidBody& body, float dt, const glm::vec3& gravity) const override {
		body.Integrate(dt, gravity);
	}
};

} // namespace Physics
} // namespace Runtime
