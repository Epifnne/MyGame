#pragma once

#include <algorithm>
#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace Runtime {
namespace Physics {

struct RigidBodyDesc {
	glm::vec3 position = glm::vec3(0.0f);
	glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	glm::vec3 linearVelocity = glm::vec3(0.0f);
	glm::vec3 angularVelocity = glm::vec3(0.0f);
	glm::vec3 inertiaTensorDiagonal = glm::vec3(1.0f);
	float mass = 1.0f;
	bool isStatic = false;
	bool useGravity = true;
};

class RigidBody {
public:
	RigidBody() = default;

	explicit RigidBody(const RigidBodyDesc& desc)
		: m_position(desc.position),
		  m_orientation(glm::normalize(desc.orientation)),
		  m_linearVelocity(desc.linearVelocity),
		  m_angularVelocity(glm::vec3(0.0f)),
		  m_isStatic(desc.isStatic),
		  m_useGravity(desc.useGravity) {
		SetInertiaTensorDiagonal(desc.inertiaTensorDiagonal);
		SetMass(desc.mass);
		if (!m_isStatic) {
			m_angularVelocity = desc.angularVelocity;
			m_angularMomentum = InverseInertiaTensorWorldInverse() * m_angularVelocity;
		}
	}

	uint32_t Id() const { return m_id; }
	void SetId(uint32_t id) { m_id = id; }

	const glm::vec3& Position() const { return m_position; }
	void SetPosition(const glm::vec3& position) { m_position = position; }

	const glm::quat& Orientation() const { return m_orientation; }
	void SetOrientation(const glm::quat& orientation) {
		m_orientation = glm::normalize(orientation);
		UpdateAngularVelocityFromMomentum();
	}

	const glm::vec3& LinearVelocity() const { return m_linearVelocity; }
	void SetLinearVelocity(const glm::vec3& velocity) { m_linearVelocity = velocity; }

	const glm::vec3& AngularVelocity() const { return m_angularVelocity; }
	void SetAngularVelocity(const glm::vec3& velocity) {
		m_angularVelocity = velocity;
		m_angularMomentum = InverseInertiaTensorWorldInverse() * m_angularVelocity;
	}

	const glm::vec3& AngularMomentum() const { return m_angularMomentum; }
	void SetAngularMomentum(const glm::vec3& momentum) {
		m_angularMomentum = momentum;
		UpdateAngularVelocityFromMomentum();
	}

	float Mass() const { return m_mass; }
	float InverseMass() const { return m_inverseMass; }

	const glm::vec3& InertiaTensorDiagonal() const { return m_inertiaTensorDiagonal; }
	const glm::vec3& InverseInertiaTensorDiagonal() const { return m_inverseInertiaTensorDiagonal; }

	glm::mat3 InverseInertiaTensorWorld() const {
		if (m_isStatic) {
			return glm::mat3(0.0f);
		}
		const glm::mat3 r = glm::mat3_cast(glm::normalize(m_orientation));
		const glm::mat3 iBodyInv = glm::mat3(
			glm::vec3(m_inverseInertiaTensorDiagonal.x, 0.0f, 0.0f),
			glm::vec3(0.0f, m_inverseInertiaTensorDiagonal.y, 0.0f),
			glm::vec3(0.0f, 0.0f, m_inverseInertiaTensorDiagonal.z));
		return r * iBodyInv * glm::transpose(r);
	}

	void SetMass(float mass) {
		if (m_isStatic) {
			m_mass = 0.0f;
			m_inverseMass = 0.0f;
			return;
		}

		m_mass = std::max(0.0001f, mass);
		m_inverseMass = 1.0f / m_mass;
	}

	void SetInertiaTensorDiagonal(const glm::vec3& inertiaDiagonal) {
		m_inertiaTensorDiagonal = glm::max(inertiaDiagonal, glm::vec3(0.0001f));
		m_inverseInertiaTensorDiagonal = glm::vec3(
			1.0f / m_inertiaTensorDiagonal.x,
			1.0f / m_inertiaTensorDiagonal.y,
			1.0f / m_inertiaTensorDiagonal.z);
		UpdateAngularVelocityFromMomentum();
	}

	bool IsStatic() const { return m_isStatic; }
	void SetStatic(bool isStatic) {
		m_isStatic = isStatic;
		if (m_isStatic) {
			m_mass = 0.0f;
			m_inverseMass = 0.0f;
			m_accumulatedForce = glm::vec3(0.0f);
			m_accumulatedTorque = glm::vec3(0.0f);
			m_linearVelocity = glm::vec3(0.0f);
			m_angularVelocity = glm::vec3(0.0f);
			m_angularMomentum = glm::vec3(0.0f);
		} else if (m_mass <= 0.0f) {
			SetMass(1.0f);
		}
	}

	bool UseGravity() const { return m_useGravity; }
	void SetUseGravity(bool useGravity) { m_useGravity = useGravity; }

	void ApplyForce(const glm::vec3& force) {
		if (m_isStatic) {
			return;
		}
		m_accumulatedForce += force;
	}

	void ApplyTorque(const glm::vec3& torque) {
		if (m_isStatic) {
			return;
		}
		m_accumulatedTorque += torque;
	}

	void ApplyLinearImpulse(const glm::vec3& impulse) {
		if (m_isStatic) {
			return;
		}
		m_linearVelocity += impulse * m_inverseMass;
	}

	void ApplyAngularImpulse(const glm::vec3& impulse) {
		if (m_isStatic) {
			return;
		}
		m_angularMomentum += impulse;
		UpdateAngularVelocityFromMomentum();
	}

	void ClearForces() {
		m_accumulatedForce = glm::vec3(0.0f);
		m_accumulatedTorque = glm::vec3(0.0f);
	}

	void Integrate(float dt, const glm::vec3& gravity) {
		if (m_isStatic || dt <= 0.0f) {
			ClearForces();
			return;
		}

		glm::vec3 acceleration = m_accumulatedForce * m_inverseMass;
		if (m_useGravity) {
			acceleration += gravity;
		}

		// Semi-implicit Euler: update velocity first, then position.
		m_linearVelocity += acceleration * dt;
		m_position += m_linearVelocity * dt;

		m_angularMomentum += m_accumulatedTorque * dt;
		UpdateAngularVelocityFromMomentum();

		const glm::quat omega(0.0f, m_angularVelocity.x, m_angularVelocity.y, m_angularVelocity.z);
		m_orientation += 0.5f * dt * (omega * m_orientation);
		m_orientation = glm::normalize(m_orientation);
		ClearForces();
	}

private:
	glm::mat3 InverseInertiaTensorWorldInverse() const {
		if (m_isStatic) {
			return glm::mat3(0.0f);
		}
		const glm::mat3 r = glm::mat3_cast(glm::normalize(m_orientation));
		const glm::mat3 iBody = glm::mat3(
			glm::vec3(m_inertiaTensorDiagonal.x, 0.0f, 0.0f),
			glm::vec3(0.0f, m_inertiaTensorDiagonal.y, 0.0f),
			glm::vec3(0.0f, 0.0f, m_inertiaTensorDiagonal.z));
		return r * iBody * glm::transpose(r);
	}

	void UpdateAngularVelocityFromMomentum() {
		if (m_isStatic) {
			m_angularVelocity = glm::vec3(0.0f);
			return;
		}
		m_angularVelocity = InverseInertiaTensorWorld() * m_angularMomentum;
	}

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
