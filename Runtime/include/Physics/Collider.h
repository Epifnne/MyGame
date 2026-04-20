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

	uint32_t Layer() const { return m_layer; }
	void SetLayer(uint32_t layer) { m_layer = layer; }

	uint32_t Mask() const { return m_mask; }
	void SetMask(uint32_t mask) { m_mask = mask; }

	bool CanCollideWith(const Collider& other) const {
		return (m_mask & other.m_layer) != 0u && (other.m_mask & m_layer) != 0u;
	}

	AABB ComputeAABB(const glm::vec3& center) const {
		if (!m_shape) {
			return {};
		}
		return m_shape->ComputeAABB(center);
	}

private:
	uint32_t m_bodyId = 0;
	std::shared_ptr<CollisionShape> m_shape;
	PhysicsMaterial m_material;
	bool m_isTrigger = false;
	uint32_t m_layer = 1;
	uint32_t m_mask = 0xFFFFFFFFu;
};

} // namespace Physics
} // namespace Runtime
