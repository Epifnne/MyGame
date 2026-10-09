#pragma once

#include <cstdint>
#include <memory>

#include "CollisionShape.h"
#include "PhysicsMaterial.h"

namespace Runtime {
namespace Physics {

// Shape, material, directional response, and symmetric mask-filter settings for construction.
struct ColliderDesc {
	// Geometric shape used for broad-phase and narrow-phase queries.
	std::shared_ptr<CollisionShape> shape;
	// Surface response properties used during contact solving.
	PhysicsMaterial material;
	// Trigger colliders report overlaps but do not generate physical impulses.
	bool isTrigger = false;
	// Store one-sided response intent; the wrapper itself does not reject back-face queries.
	bool oneSided = false;
	glm::vec3 oneSidedNormalLocal = glm::vec3(0.0f, 1.0f, 0.0f);
	// Layer/mask control collision filtering.
	uint32_t layer = 1;
	uint32_t mask = 0xFFFFFFFFu;
};

// Body-bound shape wrapper with filtering properties and cache-invalidation revisions.
class Collider {
public:
	// Create a shapeless collider with default material and filtering.
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
	// Assign the body binding without changing the property revision.
	void SetBodyId(uint32_t bodyId) { m_bodyId = bodyId; }

	// Unique identity assigned by PhysicsWorld on attach. A replacement collider
	// always receives a fresh identity so the midphase can detect replacement
	// even when revision counters would coincidentally match.
	uint32_t Identity() const { return m_identity; }
	// Assign replacement identity without changing the property revision.
	void SetIdentity(uint32_t identity) { m_identity = identity; }

	// Return the property revision; material edits require separate value comparison.
	uint32_t Revision() const { return m_revision; }

	// Read the shared shape ownership reference.
	const std::shared_ptr<CollisionShape>& Shape() const { return m_shape; }
	// Move in the replacement shape and increment the property revision.
	void SetShape(std::shared_ptr<CollisionShape> shape) {
		m_shape = std::move(shape);
		++m_revision;
	}

	// Access material parameters.
	const PhysicsMaterial& Material() const { return m_material; }
	// Expose editable material without incrementing the revision.
	PhysicsMaterial& Material() { return m_material; }

	// Trigger flag controls overlap-only behavior.
	bool IsTrigger() const { return m_isTrigger; }
	// Set overlap-only response and increment the revision, even if unchanged.
	void SetTrigger(bool isTrigger) {
		m_isTrigger = isTrigger;
		++m_revision;
	}

	// Read directional-response intent; narrow-phase queries here do not enforce it.
	bool IsOneSided() const { return m_oneSided; }
	// Store directional-response intent and increment the revision.
	void SetOneSided(bool oneSided) {
		m_oneSided = oneSided;
		++m_revision;
	}

	// Read the configured local front direction.
	const glm::vec3& OneSidedNormalLocal() const { return m_oneSidedNormalLocal; }
	// Normalize n when dot(n,n)>1e-8 and increment the revision; ignore shorter normals.
	void SetOneSidedNormalLocal(const glm::vec3& normal) {
		if (glm::dot(normal, normal) > 1e-8f) {
			m_oneSidedNormalLocal = glm::normalize(normal);
			++m_revision;
		}
	}

	// Collision filter group and mask accessors.
	uint32_t Layer() const { return m_layer; }
	// Replace layer bits and increment the revision.
	void SetLayer(uint32_t layer) {
		m_layer = layer;
		++m_revision;
	}

	// Read the accepted peer-layer bits.
	uint32_t Mask() const { return m_mask; }
	// Replace accepted layer bits and increment the revision.
	void SetMask(uint32_t mask) {
		m_mask = mask;
		++m_revision;
	}

	// Return true when both colliders pass layer/mask filtering.
	bool CanCollideWith(const Collider& other) const {
		return (m_mask & other.m_layer) != 0u && (other.m_mask & m_layer) != 0u;
	}

	// Delegate world bounds to the shape; a missing shape returns the zero AABB.
	AABB ComputeAABB(const ShapeTransform& transform) const {
		if (!m_shape) {
			return {};
		}
		return m_shape->ComputeAABB(transform);
	}

	// Delegate support mapping; a missing shape returns the supplied position.
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
