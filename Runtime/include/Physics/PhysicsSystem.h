#pragma once

#include "PhysicsWorld.h"

namespace Runtime {
namespace Physics {

class PhysicsSystem {
public:
	PhysicsSystem() = default;

	void Update(float dt) { m_world.Step(dt); }

	PhysicsWorld& WorldRef() { return m_world; }
	const PhysicsWorld& WorldRef() const { return m_world; }

private:
	PhysicsWorld m_world;
};

} // namespace Physics
} // namespace Runtime
