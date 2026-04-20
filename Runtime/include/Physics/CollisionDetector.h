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
	CollisionDetector()
		: m_broadPhase(std::make_unique<NaiveBroadPhase>()),
		  m_narrowPhase(std::make_unique<BasicNarrowPhase>()) {}

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
		const std::unordered_map<uint32_t, RigidBody>& bodies) const {
		std::vector<ContactManifold> contacts;
		if (!m_broadPhase || !m_narrowPhase) {
			return contacts;
		}

		const std::vector<BroadPhasePair> candidatePairs = m_broadPhase->ComputePairs(colliders, bodies);
		contacts.reserve(candidatePairs.size());

		for (const auto& pair : candidatePairs) {
			const auto colliderItA = colliders.find(pair.first);
			const auto colliderItB = colliders.find(pair.second);
			const auto bodyItA = bodies.find(pair.first);
			const auto bodyItB = bodies.find(pair.second);

			if (colliderItA == colliders.end() || colliderItB == colliders.end() ||
				bodyItA == bodies.end() || bodyItB == bodies.end()) {
				continue;
			}

			ContactManifold manifold;
			manifold.bodyA = pair.first;
			manifold.bodyB = pair.second;
			manifold.isTrigger = colliderItA->second.IsTrigger() || colliderItB->second.IsTrigger();

			if (m_narrowPhase->GenerateContact(
					colliderItA->second,
					bodyItA->second,
					colliderItB->second,
					bodyItB->second,
					manifold)) {
				contacts.push_back(manifold);
			}
		}

		return contacts;
	}

private:
	std::unique_ptr<BroadPhase> m_broadPhase;
	std::unique_ptr<NarrowPhase> m_narrowPhase;
};

} // namespace Physics
} // namespace Runtime
