#pragma once

#include <algorithm>
#include <glm/glm.hpp>

namespace Runtime {
namespace Physics {

struct AABB {
	glm::vec3 min = glm::vec3(0.0f);
	glm::vec3 max = glm::vec3(0.0f);

	bool Intersects(const AABB& other) const {
		return min.x <= other.max.x && max.x >= other.min.x &&
			   min.y <= other.max.y && max.y >= other.min.y &&
			   min.z <= other.max.z && max.z >= other.min.z;
	}
};

class CollisionShape {
public:
	virtual ~CollisionShape() = default;
	virtual AABB ComputeAABB(const glm::vec3& center) const = 0;
};

class SphereShape final : public CollisionShape {
public:
	explicit SphereShape(float radius) : m_radius(std::max(0.0f, radius)) {}

	float Radius() const { return m_radius; }

	AABB ComputeAABB(const glm::vec3& center) const override {
		const glm::vec3 extents(m_radius);
		return {center - extents, center + extents};
	}

private:
	float m_radius = 0.5f;
};

class BoxShape final : public CollisionShape {
public:
	explicit BoxShape(const glm::vec3& halfExtents)
		: m_halfExtents(glm::max(halfExtents, glm::vec3(0.0f))) {}

	const glm::vec3& HalfExtents() const { return m_halfExtents; }

	AABB ComputeAABB(const glm::vec3& center) const override {
		return {center - m_halfExtents, center + m_halfExtents};
	}

private:
	glm::vec3 m_halfExtents = glm::vec3(0.5f);
};

} // namespace Physics
} // namespace Runtime
