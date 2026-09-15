#pragma once

#include <cstdint>
#include <memory>

#include "CollisionShape.h"
#include "PhysicsMaterial.h"

namespace Runtime {
namespace Physics {

struct ColliderDesc {
	// Geometric shape used for broad-phase and narrow-phase queries.
	std::shared_ptr<CollisionShape> shape;
	// Surface response properties used during contact solving.
	PhysicsMaterial material;
	// Trigger colliders report overlaps but do not generate physical impulses.
	bool isTrigger = false;
	// One-sided colliders only react when hit from the front side.
	bool oneSided = false;
	glm::vec3 oneSidedNormalLocal = glm::vec3(0.0f, 1.0f, 0.0f);
	// Layer/mask control collision filtering.
	uint32_t layer = 1;
	uint32_t mask = 0xFFFFFFFFu;
};

class Collider {
public:
	Collider() = default;

	// Construct collider from descriptor data.
	explicit Collider(const ColliderDesc& desc)
		: m_shape(desc.shape),
		  m_material(desc.material),
		  m_isTrigger(desc.isTrigger),
		  m_oneSided(desc.oneSided),
		  m_oneSidedNormalLocal(desc.oneSidedNormalLocal),
		  m_layer(desc.layer),
		  m_mask(desc.mask) {}

	// Collider is valid when it owns a collision shape.
	bool IsValid() const { return static_cast<bool>(m_shape); }

	// Body id this collider is attached to.
	uint32_t BodyId() const { return m_bodyId; }
	void SetBodyId(uint32_t bodyId) { m_bodyId = bodyId; }

	// Unique identity assigned by PhysicsWorld on attach. A replacement collider
	// always receives a fresh identity so the midphase can detect replacement
	// even when revision counters would coincidentally match.
	uint32_t Identity() const { return m_identity; }
	void SetIdentity(uint32_t identity) { m_identity = identity; }

	// Bumped by every property setter below. The mutable Material() accessor
	// cannot bump it, so the midphase compares material values instead.
	uint32_t Revision() const { return m_revision; }

	// Access or replace underlying collision shape.
	const std::shared_ptr<CollisionShape>& Shape() const { return m_shape; }
	void SetShape(std::shared_ptr<CollisionShape> shape) {
		m_shape = std::move(shape);
		++m_revision;
	}

	// Access material parameters.
	const PhysicsMaterial& Material() const { return m_material; }
	PhysicsMaterial& Material() { return m_material; }

	// Trigger flag controls overlap-only behavior.
	bool IsTrigger() const { return m_isTrigger; }
	void SetTrigger(bool isTrigger) {
		m_isTrigger = isTrigger;
		++m_revision;
	}

	// One-sided flag and local normal controls directional collision response.
	bool IsOneSided() const { return m_oneSided; }
	void SetOneSided(bool oneSided) {
		m_oneSided = oneSided;
		++m_revision;
	}

	const glm::vec3& OneSidedNormalLocal() const { return m_oneSidedNormalLocal; }
	void SetOneSidedNormalLocal(const glm::vec3& normal) {
		if (glm::dot(normal, normal) > 1e-8f) {
			m_oneSidedNormalLocal = glm::normalize(normal);
			++m_revision;
		}
	}

	// Collision filter group and mask accessors.
	uint32_t Layer() const { return m_layer; }
	void SetLayer(uint32_t layer) {
		m_layer = layer;
		++m_revision;
	}

	uint32_t Mask() const { return m_mask; }
	void SetMask(uint32_t mask) {
		m_mask = mask;
		++m_revision;
	}

	// Return true when both colliders pass layer/mask filtering.
	bool CanCollideWith(const Collider& other) const {
		return (m_mask & other.m_layer) != 0u && (other.m_mask & m_layer) != 0u;
	}

	// Compute world-space bounds and support point through the shape.
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
	uint32_t m_identity = 0;
	uint32_t m_revision = 0;
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
