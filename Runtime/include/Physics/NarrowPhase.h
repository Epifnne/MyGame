#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include <glm/glm.hpp>

#include "Collider.h"
#include "ContactManifold.h"
#include "RigidBody.h"

namespace Runtime {
namespace Physics {

// Per-query algorithm counters supplied by the caller to avoid shared mutable telemetry.
struct NarrowPhaseQueryStats {
	uint32_t satCallCount = 0;
	uint32_t primitiveCallCount = 0;
	uint32_t gjkCallCount = 0;
	uint32_t gjkFailureCount = 0;
	uint32_t epaCallCount = 0;
	uint32_t epaFailureCount = 0;
};

// Contact-generation interface with optional speculative-distance configuration.
class NarrowPhase {
public:
	// Destroy concrete narrow-phase implementations safely through the interface.
	virtual ~NarrowPhase() = default;

	// Generate touching or supported speculative geometry; false may mean separation or failure.
	// outStats is reset at the start of the call and accumulates this query
	// only, so concurrent calls on the same instance never alias statistics.
	virtual bool GenerateContact(
		const Collider& colliderA,
		const RigidBody& bodyA,
		const Collider& colliderB,
		const RigidBody& bodyB,
		ContactManifold& outContact,
		NarrowPhaseQueryStats& outStats) const = 0;

	// Speculative contact band in meters; 0 disables. No-op by default so
	// custom narrow phases without speculative support simply ignore it.
	virtual void SetSpeculativeContactDistance(float distance) { (void)distance; }
};

// Analytic sphere queries, 15-axis box SAT, and support-mapped GJK/EPA for other convex pairs.
class GjkEpaNarrowPhase final : public NarrowPhase {
public:
	// Initialize the speculative band to zero.
	GjkEpaNarrowPhase() = default;
	// Destroy the implementation; query scratch storage is local to each call.
	~GjkEpaNarrowPhase() override = default;

	// Dispatch primitives to analytic/SAT queries and other convex shapes to GJK/EPA.
	bool GenerateContact(
		const Collider& colliderA,
		const RigidBody& bodyA,
		const Collider& colliderB,
		const RigidBody& bodyB,
		ContactManifold& outContact,
		NarrowPhaseQueryStats& outStats) const override;

	// Speculative contact band in meters; 0 disables speculative contacts.
	void SetSpeculativeContactDistance(float distance) override;

	// Return an A-to-B plane/gap via primitives, SAT, or GJK distance.
	// Primitives/SAT return true even at overlap; generic distance returns false
	// for touching/overlap or nonconvergence, not an overlap classification.
	bool GetSeparation(
		const Collider& colliderA, const ShapeTransform& tfA,
		const Collider& colliderB, const ShapeTransform& tfB,
		glm::vec3& normal, float& gap) const;

private:
	// Paired support witnesses and their Minkowski difference point A-B.
	struct SupportPoint {
		glm::vec3 pointA = glm::vec3(0.0f);
		glm::vec3 pointB = glm::vec3(0.0f);
		glm::vec3 point = glm::vec3(0.0f);
	};

	using Simplex = std::vector<SupportPoint>;

	// Polytope triangle with an origin-facing distance and outward unit normal.
	struct EpaFace {
		int a = 0;
		int b = 0;
		int c = 0;
		glm::vec3 normal = glm::vec3(0.0f);
		float distance = 0.0f;
		bool valid = false;
	};

	// EPA plane depth and barycentrically interpolated shape witnesses.
	struct EpaResult {
		glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
		float penetration = 0.0f;
		glm::vec3 contactPoint = glm::vec3(0.0f);
		glm::vec3 witnessA = glm::vec3(0.0f);
		glm::vec3 witnessB = glm::vec3(0.0f);
	};

	enum class QueryResult {
		Separated,
		Intersecting,
		Failed,
	};

	// Output of the GJK distance variant for a separated pair.
	struct GjkDistanceResult {
		// Separation axis A -> B (contact normal convention:
		// dot(surfaceA - surfaceB, normal) = -gap).
		glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
		glm::vec3 witnessA = glm::vec3(0.0f);
		glm::vec3 witnessB = glm::vec3(0.0f);
		float gap = 0.0f;
	};

	// Speculative contact band in meters; 0 disables.
	float m_speculativeContactDistance = 0.0f;

	static constexpr int kMaxGjkIterations = 32;
	static constexpr int kMaxEpaIterations = 48;
	static constexpr std::size_t kMaxEpaVertices = 64;
	static constexpr std::size_t kMaxEpaFaces = 128;
	static constexpr float kEpsilon = 1e-5f;

	// Support mapping on Minkowski difference A-B.
	SupportPoint Support(
		const Collider& a,
		const ShapeTransform& tfA,
		const Collider& b,
		const ShapeTransform& tfB,
		const glm::vec3& direction) const;

	// GJK intersection test and simplex generation.
	QueryResult RunGjk(
		const Collider& a,
		const ShapeTransform& tfA,
		const Collider& b,
		const ShapeTransform& tfB,
		Simplex& simplex) const;
	// Dispatch two-, three-, and four-point simplex updates.
	bool UpdateSimplex(Simplex& simplex, glm::vec3& direction) const;
	// Retain the edge or newest vertex; use the triple product for the next direction.
	bool HandleLine(Simplex& simplex, glm::vec3& direction) const;
	// Test edge half-spaces, otherwise search along the oriented triangle normal.
	bool HandleTriangle(Simplex& simplex, glm::vec3& direction) const;
	// Test the three faces adjacent to the newest vertex; reduce or report containment.
	bool HandleTetrahedron(Simplex& simplex, glm::vec3& direction) const;
	// Build one EPA face with consistent outward normal.
	EpaFace BuildFace(const std::vector<SupportPoint>& vertices, int a, int b, int c) const;
	// Interpolate shape witness points at the closest point on an EPA face.
	bool BuildEpaResult(
		const std::vector<SupportPoint>& vertices,
		const EpaFace& face,
		EpaResult& out) const;
	// Require a tetrahedral simplex, expand visible faces, and extract the closest converged plane.
	QueryResult RunEpa(
		const Collider& a,
		const ShapeTransform& tfA,
		const Collider& b,
		const ShapeTransform& tfB,
		const Simplex& simplex,
		EpaResult& out) const;

	// GJK distance variant: iteratively converges the simplex to the feature
	// of the Minkowski difference closest to the origin. Returns true when the
	// pair converges, with barycentric witnesses. maxDistance rejects a support-plane
	// lower bound beyond the band; the final witness distance is not rechecked.
	bool RunGjkDistance(
		const Collider& a,
		const ShapeTransform& tfA,
		const Collider& b,
		const ShapeTransform& tfB,
		float maxDistance,
		GjkDistanceResult& out) const;
	// Project the origin onto the simplex, compact it to the supporting
	// feature and return the closest point; outWeights[i] aligns with the
	// compacted simplex[i] (barycentric weights, sum = 1).
	glm::vec3 ProjectOriginOntoSimplex(Simplex& simplex, float* outWeights) const;
	// Separated-but-close path of GenerateContact: builds a negative
	// penetration manifold from the GJK distance result.
	bool GenerateSpeculativeContact(
		const Collider& colliderA,
		const ShapeTransform& tfA,
		const Collider& colliderB,
		const ShapeTransform& tfB,
		ContactManifold& outContact,
		NarrowPhaseQueryStats& outStats) const;
};

} // namespace Physics
} // namespace Runtime
