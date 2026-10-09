#pragma once

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdint>
#include <unordered_map>

#include <glm/glm.hpp>

#include "Collider.h"
#include "RigidBody.h"

namespace Runtime {
namespace Physics {

// World-space ray input; Cast normalizes its direction before measuring distance.
struct Ray {
	// Ray start point and cast direction in world space.
	glm::vec3 origin = glm::vec3(0.0f);
	glm::vec3 direction = glm::vec3(0.0f, -1.0f, 0.0f);
};

// Nearest AABB hit; a miss retains the requested maximum distance.
struct RaycastHit {
	// Closest hit result data.
	bool hit = false;
	uint32_t bodyId = 0;
	glm::vec3 point = glm::vec3(0.0f);
	glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
	float distance = 0.0f;
};

// Linear scan against collider world AABBs, not exact shape surfaces.
class Raycast {
public:
	// Return the nearest AABB slab hit in [0,maxDistance], ignoring collider masks/trigger flags.
	// Rays starting inside bounds hit at distance zero; equal-distance hits overwrite earlier ones.
	static RaycastHit Cast(
		const Ray& ray,
		float maxDistance,
		const std::unordered_map<uint32_t, Collider>& colliders,
		const std::unordered_map<uint32_t, RigidBody>& bodies) {
		RaycastHit bestHit;
		bestHit.distance = maxDistance;

		const float directionLen = glm::length(ray.direction);
		if (directionLen <= 1e-6f) {
			return bestHit;
		}
		const glm::vec3 direction = ray.direction / directionLen;

		for (const auto& kv : colliders) {
			const uint32_t id = kv.first;
			const auto bodyIt = bodies.find(id);
			if (bodyIt == bodies.end()) {
				continue;
			}

			ShapeTransform tf;
			tf.position = bodyIt->second.Position();
			tf.orientation = bodyIt->second.Orientation();
			const AABB aabb = kv.second.ComputeAABB(tf);
			float tNear = 0.0f;
			float tFar = maxDistance;

			if (!IntersectRayAABB(ray.origin, direction, aabb, tNear, tFar)) {
				continue;
			}

			if (tNear < 0.0f || tNear > bestHit.distance) {
				continue;
			}

			bestHit.hit = true;
			bestHit.bodyId = id;
			bestHit.distance = tNear;
			bestHit.point = ray.origin + direction * tNear;
			bestHit.normal = EstimateNormal(aabb, bestHit.point);
		}

		return bestHit;
	}

private:
	// Intersect t intervals [(min-origin)/d,(max-origin)/d] on each axis.
	// Parallel axes require the origin inside that slab; tNear/tFar are clipped in place.
	static bool IntersectRayAABB(
		const glm::vec3& origin,
		const glm::vec3& direction,
		const AABB& aabb,
		float& tNear,
		float& tFar) {
		constexpr float epsilon = 1e-6f;

		for (int axis = 0; axis < 3; ++axis) {
			const float d = direction[axis];
			const float o = origin[axis];
			const float minV = aabb.min[axis];
			const float maxV = aabb.max[axis];

			if (std::abs(d) < epsilon) {
				if (o < minV || o > maxV) {
					return false;
				}
				continue;
			}

			const float invD = 1.0f / d;
			float t1 = (minV - o) * invD;
			float t2 = (maxV - o) * invD;
			if (t1 > t2) {
				std::swap(t1, t2);
			}

			tNear = std::max(tNear, t1);
			tFar = std::min(tFar, t2);
			if (tNear > tFar) {
				return false;
			}
		}

		return true;
	}

	// Choose the signed axis of greatest absolute center offset, without scaling by extents.
	static glm::vec3 EstimateNormal(const AABB& aabb, const glm::vec3& point) {
		const glm::vec3 center = (aabb.min + aabb.max) * 0.5f;
		const glm::vec3 local = point - center;
		const glm::vec3 absLocal = glm::abs(local);

		if (absLocal.x >= absLocal.y && absLocal.x >= absLocal.z) {
			return glm::vec3(local.x >= 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f);
		}
		if (absLocal.y >= absLocal.z) {
			return glm::vec3(0.0f, local.y >= 0.0f ? 1.0f : -1.0f, 0.0f);
		}
		return glm::vec3(0.0f, 0.0f, local.z >= 0.0f ? 1.0f : -1.0f);
	}
};

} // namespace Physics
} // namespace Runtime
