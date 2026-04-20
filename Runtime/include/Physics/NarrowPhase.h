#pragma once

#include <algorithm>
#include <glm/glm.hpp>

#include "Collider.h"
#include "ContactManifold.h"
#include "RigidBody.h"

namespace Runtime {
namespace Physics {

class NarrowPhase {
public:
	virtual ~NarrowPhase() = default;

	virtual bool GenerateContact(
		const Collider& colliderA,
		const RigidBody& bodyA,
		const Collider& colliderB,
		const RigidBody& bodyB,
		ContactManifold& outContact) const = 0;
};

class BasicNarrowPhase final : public NarrowPhase {
public:
	bool GenerateContact(
		const Collider& colliderA,
		const RigidBody& bodyA,
		const Collider& colliderB,
		const RigidBody& bodyB,
		ContactManifold& outContact) const override {
		const AABB a = colliderA.ComputeAABB(bodyA.Position());
		const AABB b = colliderB.ComputeAABB(bodyB.Position());
		if (!a.Intersects(b)) {
			return false;
		}

		const float overlapX = std::min(a.max.x, b.max.x) - std::max(a.min.x, b.min.x);
		const float overlapY = std::min(a.max.y, b.max.y) - std::max(a.min.y, b.min.y);
		const float overlapZ = std::min(a.max.z, b.max.z) - std::max(a.min.z, b.min.z);

		if (overlapX <= 0.0f || overlapY <= 0.0f || overlapZ <= 0.0f) {
			return false;
		}

		const glm::vec3 centerDelta = bodyB.Position() - bodyA.Position();
		outContact.normal = glm::vec3(0.0f, 1.0f, 0.0f);
		outContact.point.penetration = overlapY;

		if (overlapX <= overlapY && overlapX <= overlapZ) {
			outContact.normal = glm::vec3(centerDelta.x >= 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f);
			outContact.point.penetration = overlapX;
		} else if (overlapY <= overlapZ) {
			outContact.normal = glm::vec3(0.0f, centerDelta.y >= 0.0f ? 1.0f : -1.0f, 0.0f);
			outContact.point.penetration = overlapY;
		} else {
			outContact.normal = glm::vec3(0.0f, 0.0f, centerDelta.z >= 0.0f ? 1.0f : -1.0f);
			outContact.point.penetration = overlapZ;
		}

		outContact.point.position = (bodyA.Position() + bodyB.Position()) * 0.5f;
		outContact.point.normalImpulse = 0.0f;
		return true;
	}
};

} // namespace Physics
} // namespace Runtime
