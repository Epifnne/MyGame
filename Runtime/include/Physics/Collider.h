#pragma once

#include <cstdint>
#include <memory>

#include "CollisionShape.h"
#include "PhysicsMaterial.h"

namespace Runtime {
namespace Physics {

struct ColliderDesc {
	std::shared_ptr<CollisionShape> shape;
	PhysicsMaterial material;
	bool isTrigger = false;
	bool oneSided = false;
	glm::vec3 oneSidedNormalLocal = glm::vec3(0.0f, 1.0f, 0.0f);
	uint32_t layer = 1;
	uint32_t mask = 0xFFFFFFFFu;
};

class Collider {
public:
	Collider() = default;

	explicit Collider(const ColliderDesc& desc)
		: m_shape(desc.shape),
		  m_material(desc.material),
		  m_isTrigger(desc.isTrigger),
		  m_oneSided(desc.oneSided),
		  m_oneSidedNormalLocal(desc.oneSidedNormalLocal),
		  m_layer(desc.layer),
		  m_mask(desc.mask) {}

	bool IsValid() const { return static_cast<bool>(m_shape); }

	uint32_t BodyId() const { return m_bodyId; }
	void SetBodyId(uint32_t bodyId) { m_bodyId = bodyId; }

	const std::shared_ptr<CollisionShape>& Shape() const { return m_shape; }
	void SetShape(std::shared_ptr<CollisionShape> shape) { m_shape = std::move(shape); }

	const PhysicsMaterial& Material() const { return m_material; }
	PhysicsMaterial& Material() { return m_material; }

	bool IsTrigger() const { return m_isTrigger; }
	void SetTrigger(bool isTrigger) { m_isTrigger = isTrigger; }

	bool IsOneSided() const { return m_oneSided; }
	void SetOneSided(bool oneSided) { m_oneSided = oneSided; }

	const glm::vec3& OneSidedNormalLocal() const { return m_oneSidedNormalLocal; }
	void SetOneSidedNormalLocal(const glm::vec3& normal) {
		if (glm::dot(normal, normal) > 1e-8f) {
			m_oneSidedNormalLocal = glm::normalize(normal);
		}
	}

	uint32_t Layer() const { return m_layer; }
	void SetLayer(uint32_t layer) { m_layer = layer; }

	uint32_t Mask() const { return m_mask; }
	void SetMask(uint32_t mask) { m_mask = mask; }

	bool CanCollideWith(const Collider& other) const {
		return (m_mask & other.m_layer) != 0u && (other.m_mask & m_layer) != 0u;
	}

	AABB ComputeAABB(const ShapeTransform& transform) const {
		if (!m_shape) {
			return {};
		}
		return m_shape->ComputeAABB(transform);
	}

	glm::vec3 Support(const ShapeTransform& transform, const glm::vec3& direction) const {
		if (!m_shape) {
			return transform.position;
		}
		return m_shape->Support(transform, direction);
	}

private:
	uint32_t m_bodyId = 0;
	std::shared_ptr<CollisionShape> m_shape;
	PhysicsMaterial m_material;
	bool m_isTrigger = false;
	bool m_oneSided = false;
	glm::vec3 m_oneSidedNormalLocal = glm::vec3(0.0f, 1.0f, 0.0f);
	uint32_t m_layer = 1;
	uint32_t m_mask = 0xFFFFFFFFu;
};

} // namespace Physics
} // namespace Runtime
