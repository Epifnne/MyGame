#pragma once

#include <glm/glm.hpp>

#include "RigidBody.h"

namespace Runtime {
namespace Physics {

class Integrator {
public:
	virtual ~Integrator() = default;
	// Integrate one rigid body state for dt.
	virtual void Integrate(RigidBody& body, float dt, const glm::vec3& gravity) const = 0;
};

class SemiImplicitEulerIntegrator final : public Integrator {
public:
	// Delegate to rigid body's semi-implicit Euler integration.
	void Integrate(RigidBody& body, float dt, const glm::vec3& gravity) const override {
		body.Integrate(dt, gravity);
	}
};

} // namespace Physics
} // namespace Runtime
