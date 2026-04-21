#pragma once

#include <array>
#include <vector>

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

class GjkEpaNarrowPhase final : public NarrowPhase {
public:
	GjkEpaNarrowPhase() = default;
	~GjkEpaNarrowPhase() override = default;

	bool GenerateContact(
		const Collider& colliderA,
		const RigidBody& bodyA,
		const Collider& colliderB,
		const RigidBody& bodyB,
		ContactManifold& outContact) const override;

private:
	struct SupportPoint {
		glm::vec3 pointA = glm::vec3(0.0f);
		glm::vec3 pointB = glm::vec3(0.0f);
		glm::vec3 point = glm::vec3(0.0f);
	};

	using Simplex = std::vector<SupportPoint>;

	struct EpaFace {
		int a = 0;
		int b = 0;
		int c = 0;
		glm::vec3 normal = glm::vec3(0.0f);
		float distance = 0.0f;
	};

	struct EpaResult {
		glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
		float penetration = 0.0f;
		glm::vec3 contactPoint = glm::vec3(0.0f);
	};

	static constexpr int kMaxGjkIterations = 32;
	static constexpr int kMaxEpaIterations = 48;
	static constexpr float kEpsilon = 1e-5f;

	SupportPoint Support(
		const Collider& a,
		const ShapeTransform& tfA,
		const Collider& b,
		const ShapeTransform& tfB,
		const glm::vec3& direction) const;

	bool RunGjk(
		const Collider& a,
		const ShapeTransform& tfA,
		const Collider& b,
		const ShapeTransform& tfB,
		Simplex& simplex) const;
	bool UpdateSimplex(Simplex& simplex, glm::vec3& direction) const;
	bool HandleLine(Simplex& simplex, glm::vec3& direction) const;
	bool HandleTriangle(Simplex& simplex, glm::vec3& direction) const;
	bool HandleTetrahedron(Simplex& simplex, glm::vec3& direction) const;
	EpaFace BuildFace(const std::vector<SupportPoint>& vertices, int a, int b, int c) const;
	bool RunEpa(
		const Collider& a,
		const ShapeTransform& tfA,
		const Collider& b,
		const ShapeTransform& tfB,
		const Simplex& simplex,
		EpaResult& out) const;
};

} // namespace Physics
} // namespace Runtime
