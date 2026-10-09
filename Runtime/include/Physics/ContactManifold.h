#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <glm/glm.hpp>

namespace Runtime {
namespace Physics {

// Labels the geometric feature pairing used to describe a contact patch.
enum class ContactTopology {
	Unknown,
	FaceFace,
	FaceEdge,
	FaceVertex,
	EdgeEdge,
};

// Match when the sum of both squared local-anchor distances is strictly below distance^2.
inline constexpr float kContactAnchorMatchDistance = 0.01f;
// cos(30 degrees): a larger normal rotation prevents transferring impulse caches.
inline constexpr float kContactNormalCacheMinDot = 0.8660254f;

// Contact/surface anchors, penetration, substep telemetry and persistent warm-start data.
struct ContactPoint {
	// normalImpulse is this point's most recently committed substep lambda, including warm start.
	glm::vec3 position = glm::vec3(0.0f);
	glm::vec3 surfacePointA = glm::vec3(0.0f);
	glm::vec3 surfacePointB = glm::vec3(0.0f);
	float penetration = 0.0f;
	float normalImpulse = 0.0f;

	// Persistent local anchors on both bodies, used for cross-frame matching.
	glm::vec3 localPointA = glm::vec3(0.0f);
	glm::vec3 localPointB = glm::vec3(0.0f);
	glm::vec3 surfaceLocalA = glm::vec3(0.0f);
	glm::vec3 surfaceLocalB = glm::vec3(0.0f);

	// Persistent normal cache; Prepare scales by dtNew/cachedDt when cachedDt > 0.
	float accumulatedNormalImpulse = 0.0f;
	// Patch tangent/spin cache: Commit duplicates it on every point; Prepare reads point zero.
	glm::vec2 accumulatedTangentImpulse = glm::vec2(0.0f);
	float accumulatedSpinImpulse = 0.0f;
	// Cached tangent basis; Prepare reconstructs world impulse and projects onto the new basis.
	glm::vec3 cachedTangent1 = glm::vec3(0.0f);
	glm::vec3 cachedTangent2 = glm::vec3(0.0f);
	// Cache time scale; nonpositive values disable Prepare's cache initialization.
	float cachedDt = 0.0f;
};

// Body-pair contact patch with at most four points, trigger flag and fixed-step impulse summaries.
struct ContactManifold {
	static constexpr std::size_t kMaxContactPoints = 4;

	// Body pair, contact normal, and up to four representative points.
	uint32_t bodyA = 0;
	uint32_t bodyB = 0;
	glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
	ContactTopology topology = ContactTopology::Unknown;
	// Face contacts transport their normal with the actual reference face.
	bool normalOnB = false;
	// The primary point remains public for compatibility with existing callers.
	ContactPoint point;
	std::array<ContactPoint, kMaxContactPoints - 1> additionalPoints{};
	std::size_t pointCount = 0;
	// Marks a trigger for caller-side velocity-solve exclusion and ResolvePosition's early exit.
	bool isTrigger = false;

	// Published fixed-step sum of final substep normal lambdas, not per-iteration lambda sums.
	float fixedStepNormalImpulse = 0.0f;
	// World-space net tangent impulse sum over the fixed step's sub-steps.
	glm::vec3 fixedStepTangentImpulse = glm::vec3(0.0f);

	// Reset active count only; leave point storage and impulse summaries intact.
	void ClearPoints() { pointCount = 0; }

	// Append into the primary/extra point storage; return false without changes when full.
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

	// Return mutable primary/extra storage for index; caller must supply an in-range index.
	ContactPoint& Point(std::size_t index) {
		return index == 0 ? point : additionalPoints[index - 1];
	}

	// Return const primary/extra storage for index; caller must supply an in-range index.
	const ContactPoint& Point(std::size_t index) const {
		return index == 0 ? point : additionalPoints[index - 1];
	}
};

// For a unit normal, choose the least-aligned world axis and set t1 = normalize(n cross axis), t2 = n cross t1.
void BuildContactTangentBasis(
	const glm::vec3& normal,
	glm::vec3& outTangent1,
	glm::vec3& outTangent2);

// Match new points in order to the nearest unused old point with |dA|^2 + |dB|^2 < 0.01^2.
// Equal distances keep the lowest old index; dot(nOld,nNew) < cos(30 degrees) transfers nothing.
// Copy only impulse caches, tangent basis and cachedDt; unmatched new data remains untouched.
void MatchPersistentContactPoints(
	const ContactManifold& oldManifold,
	ContactManifold& newManifold);

} // namespace Physics
} // namespace Runtime
