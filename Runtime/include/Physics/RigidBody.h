#pragma once

#include <algorithm>
#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace Runtime {
namespace Physics {

struct RigidBodyDesc {
	// Initial transform.
	glm::vec3 position = glm::vec3(0.0f);
	glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	// Initial velocities.
	glm::vec3 linearVelocity = glm::vec3(0.0f);
	glm::vec3 angularVelocity = glm::vec3(0.0f);
	// Principal inertia diagonal in local space.
	glm::vec3 inertiaTensorDiagonal = glm::vec3(1.0f);
	float mass = 1.0f;
	bool isStatic = false;
	bool useGravity = true;
};

class RigidBody {
public:
	RigidBody() = default;

	// Initialize body from descriptor and precompute inertia terms.
	explicit RigidBody(const RigidBodyDesc& desc);

	// Runtime id accessors.
	uint32_t Id() const { return m_id; }
	void SetId(uint32_t id) { m_id = id; }

	// Pose accessors.
	const glm::vec3& Position() const { return m_position; }
	void SetPosition(const glm::vec3& position) { m_position = position; }

	const glm::quat& Orientation() const { return m_orientation; }
	void SetOrientation(const glm::quat& orientation);

	// Linear and angular velocity/momentum accessors.
	const glm::vec3& LinearVelocity() const { return m_linearVelocity; }
	void SetLinearVelocity(const glm::vec3& velocity) { m_linearVelocity = velocity; }

	const glm::vec3& AngularVelocity() const { return m_angularVelocity; }
	void SetAngularVelocity(const glm::vec3& velocity);

	const glm::vec3& AngularMomentum() const { return m_angularMomentum; }
	void SetAngularMomentum(const glm::vec3& momentum);

	// Mass and inertia read-only accessors.
	float Mass() const { return m_mass; }
	float InverseMass() const { return m_inverseMass; }

	const glm::vec3& InertiaTensorDiagonal() const { return m_inertiaTensorDiagonal; }
	const glm::vec3& InverseInertiaTensorDiagonal() const { return m_inverseInertiaTensorDiagonal; }

	// Inverse inertia tensor transformed into world space.
	glm::mat3 InverseInertiaTensorWorld() const;

	// Set mass for dynamic bodies; static bodies keep zero inverse mass.
	void SetMass(float mass);

	// Set principal inertia diagonal and update derived angular velocity.
	void SetInertiaTensorDiagonal(const glm::vec3& inertiaDiagonal);

	// Toggle static state and keep physical invariants consistent.
	bool IsStatic() const { return m_isStatic; }
	void SetStatic(bool isStatic);

	// Enable/disable gravity force contribution.
	bool UseGravity() const { return m_useGravity; }
	void SetUseGravity(bool useGravity) { m_useGravity = useGravity; }

	// External force/torque accumulation for next integration step.
	void ApplyForce(const glm::vec3& force);

	void ApplyTorque(const glm::vec3& torque);

	// Instant velocity changes in world space.
	void ApplyLinearImpulse(const glm::vec3& impulse);

	void ApplyAngularImpulse(const glm::vec3& impulse);

	// Clear accumulated force and torque buffers.
	void ClearForces();

	// Integrate body state for one time step.
	void Integrate(float dt, const glm::vec3& gravity);

private:
	// Return world-space inertia tensor (not inverted).
	glm::mat3 InverseInertiaTensorWorldInverse() const;

	// Update angular velocity from current angular momentum.
	void UpdateAngularVelocityFromMomentum();

	uint32_t m_id = 0;
	glm::vec3 m_position = glm::vec3(0.0f);
	glm::quat m_orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	glm::vec3 m_linearVelocity = glm::vec3(0.0f);
	glm::vec3 m_angularVelocity = glm::vec3(0.0f);
	glm::vec3 m_angularMomentum = glm::vec3(0.0f);
	glm::vec3 m_accumulatedForce = glm::vec3(0.0f);
	glm::vec3 m_accumulatedTorque = glm::vec3(0.0f);
	glm::vec3 m_inertiaTensorDiagonal = glm::vec3(1.0f);
	glm::vec3 m_inverseInertiaTensorDiagonal = glm::vec3(1.0f);
	float m_mass = 1.0f;
	float m_inverseMass = 1.0f;
	bool m_isStatic = false;
	bool m_useGravity = true;
};

} // namespace Physics
} // namespace Runtime
