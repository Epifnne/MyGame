#pragma once

#include <cstdint>

#include "Collider.h"
#include "PhysicsWorld.h"
#include "RigidBody.h"

namespace Runtime {
namespace Physics {

class World {
public:
	// Construct facade over PhysicsWorld.
	World() = default;

	// Advance physics simulation.
	void Step(float deltaTime) { m_world.Step(deltaTime); }

	// Forward body creation/removal to internal world.
	uint32_t CreateRigidBody(const RigidBodyDesc& desc) { return m_world.CreateRigidBody(desc); }
	bool DestroyRigidBody(uint32_t bodyId) { return m_world.DestroyRigidBody(bodyId); }

	// Attach collider to an existing rigid body.
	bool AttachCollider(uint32_t bodyId, const ColliderDesc& desc) {
		return m_world.AttachCollider(bodyId, desc);
	}

	// Access internal PhysicsWorld implementation.
	PhysicsWorld& Impl() { return m_world; }
	const PhysicsWorld& Impl() const { return m_world; }

private:
	PhysicsWorld m_world;
};

} // namespace Physics
} // namespace Runtime
