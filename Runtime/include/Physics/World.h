#pragma once

#include <cstdint>

#include "Collider.h"
#include "PhysicsWorld.h"
#include "RigidBody.h"

namespace Runtime {
namespace Physics {

// Lightweight facade for body management and fixed-step world simulation.
class World {
public:
	// Construct facade over PhysicsWorld.
	World() = default;

	// Advance physics simulation.
	void Step(float deltaTime) { m_world.Step(deltaTime); }

	// Create a body in the owned world and return its runtime id.
	uint32_t CreateRigidBody(const RigidBodyDesc& desc) { return m_world.CreateRigidBody(desc); }
	// Remove a body and its collider; return whether the body existed.
	bool DestroyRigidBody(uint32_t bodyId) { return m_world.DestroyRigidBody(bodyId); }

	// Attach collider to an existing rigid body.
	bool AttachCollider(uint32_t bodyId, const ColliderDesc& desc) {
		return m_world.AttachCollider(bodyId, desc);
	}

	// Access internal PhysicsWorld implementation.
	PhysicsWorld& Impl() { return m_world; }
	// Inspect the internal world without modifying it.
	const PhysicsWorld& Impl() const { return m_world; }

private:
	PhysicsWorld m_world;
};

} // namespace Physics
} // namespace Runtime
