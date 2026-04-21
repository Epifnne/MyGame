#pragma once

#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

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

	static AABB Merge(const AABB& a, const AABB& b) {
		return {
			glm::min(a.min, b.min),
			glm::max(a.max, b.max)
		};
	}

	float SurfaceArea() const {
		const glm::vec3 ext = glm::max(max - min, glm::vec3(0.0f));
		return 2.0f * (ext.x * ext.y + ext.y * ext.z + ext.z * ext.x);
	}

	AABB Expanded(float margin) const {
		const glm::vec3 pad(std::max(0.0f, margin));
		return {min - pad, max + pad};
	}
};

struct ShapeTransform {
	glm::vec3 position = glm::vec3(0.0f);
	glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
};

class CollisionShape {
public:
	virtual ~CollisionShape() = default;
	virtual AABB ComputeAABB(const ShapeTransform& transform) const = 0;
	virtual glm::vec3 Support(const ShapeTransform& transform, const glm::vec3& direction) const = 0;
};

class SphereShape final : public CollisionShape {
public:
	explicit SphereShape(float radius) : m_radius(std::max(0.0f, radius)) {}

	float Radius() const { return m_radius; }

	AABB ComputeAABB(const ShapeTransform& transform) const override {
		const glm::vec3 extents(m_radius);
		return {transform.position - extents, transform.position + extents};
	}

	glm::vec3 Support(const ShapeTransform& transform, const glm::vec3& direction) const override {
		const float len2 = glm::dot(direction, direction);
		if (len2 <= 1e-12f) {
			return transform.position;
		}
		return transform.position + glm::normalize(direction) * m_radius;
	}

private:
	float m_radius = 0.5f;
};

class BoxShape final : public CollisionShape {
public:
	explicit BoxShape(const glm::vec3& halfExtents)
		: m_halfExtents(glm::max(halfExtents, glm::vec3(0.0f))) {}

	const glm::vec3& HalfExtents() const { return m_halfExtents; }

	AABB ComputeAABB(const ShapeTransform& transform) const override {
		const glm::mat3 r = glm::mat3_cast(glm::normalize(transform.orientation));
		const glm::mat3 absR = glm::mat3(
			glm::abs(r[0]),
			glm::abs(r[1]),
			glm::abs(r[2]));
		const glm::vec3 extents = absR * m_halfExtents;
		return {transform.position - extents, transform.position + extents};
	}

	glm::vec3 Support(const ShapeTransform& transform, const glm::vec3& direction) const override {
		const glm::quat q = glm::normalize(transform.orientation);
		const glm::vec3 localDir = glm::inverse(q) * direction;
		const glm::vec3 localSupport(
			localDir.x >= 0.0f ? m_halfExtents.x : -m_halfExtents.x,
			localDir.y >= 0.0f ? m_halfExtents.y : -m_halfExtents.y,
			localDir.z >= 0.0f ? m_halfExtents.z : -m_halfExtents.z);
		return transform.position + (q * localSupport);
	}

private:
	glm::vec3 m_halfExtents = glm::vec3(0.5f);
};

} // namespace Physics
} // namespace Runtime
