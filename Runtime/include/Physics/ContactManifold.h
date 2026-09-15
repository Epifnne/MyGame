#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <glm/glm.hpp>

namespace Runtime {
namespace Physics {

// Matching thresholds for persistent contact point matching.
inline constexpr float kContactAnchorMatchDistance = 0.05f;
// cos(30 degrees): a normal rotation beyond this clears the impulse caches.
inline constexpr float kContactNormalCacheMinDot = 0.8660254f;

struct ContactPoint {
	// Contact position, penetration depth, and solved normal impulse.
	// normalImpulse semantics (frozen telemetry contract): the accumulated
	// normal impulse actually applied within the most recent touching sub-step.
	// It is NOT a cross-sub-step sum and NOT a cross-frame history total.
	glm::vec3 position = glm::vec3(0.0f);
	float penetration = 0.0f;
	float normalImpulse = 0.0f;

	// Persistent local anchors on both bodies, used for cross-frame matching.
	glm::vec3 localPointA = glm::vec3(0.0f);
	glm::vec3 localPointB = glm::vec3(0.0f);

	// Persistent solver cache transferred across frames by anchor matching.
	// The Phase 4 solver warm-starts from these fields: they carry the last
	// sub-step's accumulated solution forward as the next sub-step's cache,
	// scaled by dtNew/cachedDt and clamped before application.
	float accumulatedNormalImpulse = 0.0f;
	glm::vec2 accumulatedTangentImpulse = glm::vec2(0.0f);
	// Tangent basis under which the cached tangent impulse was accumulated.
	// The interpretation data travels with the cache through matching; the
	// Prepare stage reprojects the cached 2D impulse onto the new basis and
	// the old components are never reused directly in a new basis. The solver
	// re-stamps the current basis when it stores the solved cache.
	glm::vec3 cachedTangent1 = glm::vec3(0.0f);
	glm::vec3 cachedTangent2 = glm::vec3(0.0f);
	// Sub-step dt the cache was accumulated with (drives dtNew/cachedDt warm
	// start scaling). Zero means no usable cache.
	float cachedDt = 0.0f;
	// Whether THIS sub-step is a TOI impact sub-step (stamped fresh at commit).
	bool isToiImpact = false;
	// Whether the transferred cache originated from a TOI impact sub-step
	// (transferred by matching). Either flag suppresses warm start: TOI impact
	// contacts never replay their cache across sub-steps.
	bool cacheFromToiImpact = false;
};

struct ContactManifold {
	static constexpr std::size_t kMaxContactPoints = 4;

	// Body pair, contact normal, and up to four representative points.
	uint32_t bodyA = 0;
	uint32_t bodyB = 0;
	glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
	// The primary point remains public for compatibility with existing callers.
	ContactPoint point;
	std::array<ContactPoint, kMaxContactPoints - 1> additionalPoints{};
	std::size_t pointCount = 0;
	// Trigger contacts skip impulse solving; every impulse field stays zero.
	bool isTrigger = false;

	// Fixed-step total impulse summary (frozen telemetry contract): the net
	// impulse actually applied across all sub-steps of the fixed step, i.e.
	// the sum of each sub-step's final accumulated lambda (warm start plus
	// subsequent deltas), never a per-iteration accumulation of lambda.
	// Stamped by PhysicsWorld at publish time; zero for triggers.
	float fixedStepNormalImpulse = 0.0f;
	// World-space net tangent impulse sum over the fixed step's sub-steps.
	glm::vec3 fixedStepTangentImpulse = glm::vec3(0.0f);

	void ClearPoints() { pointCount = 0; }

	bool AddPoint(const ContactPoint& contactPoint) {
		if (pointCount >= kMaxContactPoints) {
			return false;
		}
		if (pointCount == 0) {
			point = contactPoint;
		} else {
			additionalPoints[pointCount - 1] = contactPoint;
		}
		++pointCount;
		return true;
	}

	ContactPoint& Point(std::size_t index) {
		return index == 0 ? point : additionalPoints[index - 1];
	}

	const ContactPoint& Point(std::size_t index) const {
		return index == 0 ? point : additionalPoints[index - 1];
	}
};

// Deterministic orthonormal tangent basis for a unit contact normal.
void BuildContactTangentBasis(
	const glm::vec3& normal,
	glm::vec3& outTangent1,
	glm::vec3& outTangent2);

// Deterministic one-to-one matching of new manifold points against old points
// by two-sided local anchor distance; exact distance ties resolve to the
// lowest old index. Transfers the accumulated impulse caches together with
// their interpretation data (old tangent basis, cachedDt, TOI origin flag);
// geometry and the current sub-step's isToiImpact are stamped fresh by the
// caller. A significant normal flip transfers nothing, i.e. clears the cache.
void MatchPersistentContactPoints(
	const ContactManifold& oldManifold,
	ContactManifold& newManifold);

} // namespace Physics
} // namespace Runtime
