#pragma once

namespace Runtime {
namespace Physics {

class Constraint {
public:
	virtual ~Constraint() = default;
	virtual void Solve(float dt) = 0;
};

} // namespace Physics
} // namespace Runtime
