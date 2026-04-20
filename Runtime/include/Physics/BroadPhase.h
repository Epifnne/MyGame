#pragma once

#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Collider.h"
#include "RigidBody.h"

namespace Runtime {
namespace Physics {

using BroadPhasePair = std::pair<uint32_t, uint32_t>;

class BroadPhase {
public:
	virtual ~BroadPhase() = default;

	virtual std::vector<BroadPhasePair> ComputePairs(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies) const = 0;
};

class NaiveBroadPhase final : public BroadPhase {
public:
	std::vector<BroadPhasePair> ComputePairs(
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies) const override {
		std::vector<BroadPhasePair> pairs;
		if (colliders.size() < 2) {
			return pairs;
		}

		std::vector<uint32_t> ids;
		ids.reserve(colliders.size());
		for (const auto& kv : colliders) {
			ids.push_back(kv.first);
		}

		for (size_t i = 0; i < ids.size(); ++i) {
			const uint32_t idA = ids[i];
			const auto colliderItA = colliders.find(idA);
			const auto bodyItA = bodies.find(idA);
			if (colliderItA == colliders.end() || bodyItA == bodies.end()) {
				continue;
			}

			for (size_t j = i + 1; j < ids.size(); ++j) {
				const uint32_t idB = ids[j];
				const auto colliderItB = colliders.find(idB);
				const auto bodyItB = bodies.find(idB);
				if (colliderItB == colliders.end() || bodyItB == bodies.end()) {
					continue;
				}

				const Collider& colliderA = colliderItA->second;
				const Collider& colliderB = colliderItB->second;
				if (!colliderA.CanCollideWith(colliderB)) {
					continue;
				}

				const AABB aabbA = colliderA.ComputeAABB(bodyItA->second.Position());
				const AABB aabbB = colliderB.ComputeAABB(bodyItB->second.Position());
				if (aabbA.Intersects(aabbB)) {
					pairs.emplace_back(std::min(idA, idB), std::max(idA, idB));
				}
			}
		}

		return pairs;
	}
};

} // namespace Physics
} // namespace Runtime
