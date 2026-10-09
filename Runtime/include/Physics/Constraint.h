#pragma once

namespace Runtime {
namespace Physics {

// Abstract time-step constraint; concrete constraints define their solve rule.
class Constraint {
public:
	// Destroy a concrete constraint through the interface.
	virtual ~Constraint() = default;
	// Solve this constraint for the current time step.
	virtual void Solve(float dt) = 0;
};

} // namespace Physics
} // namespace Runtime
