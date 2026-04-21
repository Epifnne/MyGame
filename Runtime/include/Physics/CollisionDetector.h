#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "BroadPhase.h"
#include "ContactManifold.h"
#include "NarrowPhase.h"

namespace Runtime {
namespace Physics {

class CollisionDetector {
public:
	// Create detector with default broad-phase and narrow-phase implementations.
	CollisionDetector();
	// Virtual resources managed by unique_ptr members.
	~CollisionDetector();

	// Override broad-phase stage implementation.
	void SetBroadPhase(std::unique_ptr<BroadPhase> broadPhase) {
		if (broadPhase) {
			m_broadPhase = std::move(broadPhase);
		}
	}

	// Override narrow-phase stage implementation.
	void SetNarrowPhase(std::unique_ptr<NarrowPhase> narrowPhase) {
		if (narrowPhase) {
			m_narrowPhase = std::move(narrowPhase);
		}
	}

	// Run collision pipeline and return generated contact manifolds.
	std::vector<ContactManifold> Detect(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies);

private:
	std::unique_ptr<BroadPhase> m_broadPhase;
	std::unique_ptr<NarrowPhase> m_narrowPhase;
};

} // namespace Physics
} // namespace Runtime
