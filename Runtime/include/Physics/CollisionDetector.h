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
	CollisionDetector();
	~CollisionDetector();

	void SetBroadPhase(std::unique_ptr<BroadPhase> broadPhase) {
		if (broadPhase) {
			m_broadPhase = std::move(broadPhase);
		}
	}

	void SetNarrowPhase(std::unique_ptr<NarrowPhase> narrowPhase) {
		if (narrowPhase) {
			m_narrowPhase = std::move(narrowPhase);
		}
	}

	std::vector<ContactManifold> Detect(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies);

private:
	std::unique_ptr<BroadPhase> m_broadPhase;
	std::unique_ptr<NarrowPhase> m_narrowPhase;
};

} // namespace Physics
} // namespace Runtime
