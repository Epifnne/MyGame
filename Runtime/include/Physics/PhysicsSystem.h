#pragma once

#include "PhysicsWorld.h"

namespace Runtime {
namespace Physics {

// Own a physics world and forward frame updates to its fixed-step accumulator.
class PhysicsSystem {
public:
	// Construct physics system with an internal world instance.
	PhysicsSystem() = default;

	// Advance physics simulation by delta time.
	void Update(float dt) { m_world.Step(dt); }

	// Access the owned world for configuration and body management.
	PhysicsWorld& WorldRef() { return m_world; }
	// Inspect the owned world without modifying simulation state.
	const PhysicsWorld& WorldRef() const { return m_world; }

private:
	PhysicsWorld m_world;
};

} // namespace Physics
} // namespace Runtime
