#pragma once

namespace Runtime {
namespace Physics {

class Constraint {
public:
	virtual ~Constraint() = default;
	// Solve this constraint for the current time step.
	virtual void Solve(float dt) = 0;
};

} // namespace Physics
} // namespace Runtime
