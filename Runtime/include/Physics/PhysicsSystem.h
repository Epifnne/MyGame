#pragma once

#include "PhysicsWorld.h"

namespace Runtime {
namespace Physics {

class PhysicsSystem {
public:
	// Construct physics system with an internal world instance.
	PhysicsSystem() = default;

	// Advance physics simulation by delta time.
	void Update(float dt) { m_world.Step(dt); }

	// Access the owned physics world.
	PhysicsWorld& WorldRef() { return m_world; }
	const PhysicsWorld& WorldRef() const { return m_world; }

private:
	PhysicsWorld m_world;
};

} // namespace Physics
} // namespace Runtime
