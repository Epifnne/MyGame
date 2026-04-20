#pragma once

#include <algorithm>
#include <cstdint>
#include <glm/glm.hpp>

namespace Runtime {
namespace Physics {

struct RigidBodyDesc {
	glm::vec3 position = glm::vec3(0.0f);
	glm::vec3 linearVelocity = glm::vec3(0.0f);
	float mass = 1.0f;
	bool isStatic = false;
	bool useGravity = true;
};

class RigidBody {
public:
	RigidBody() = default;

	explicit RigidBody(const RigidBodyDesc& desc)
		: m_position(desc.position),
		  m_linearVelocity(desc.linearVelocity),
		  m_isStatic(desc.isStatic),
		  m_useGravity(desc.useGravity) {
		SetMass(desc.mass);
	}

	uint32_t Id() const { return m_id; }
	void SetId(uint32_t id) { m_id = id; }

	const glm::vec3& Position() const { return m_position; }
	void SetPosition(const glm::vec3& position) { m_position = position; }

	const glm::vec3& LinearVelocity() const { return m_linearVelocity; }
	void SetLinearVelocity(const glm::vec3& velocity) { m_linearVelocity = velocity; }

	float Mass() const { return m_mass; }
	float InverseMass() const { return m_inverseMass; }

	void SetMass(float mass) {
		if (m_isStatic) {
			m_mass = 0.0f;
			m_inverseMass = 0.0f;
			return;
		}

		m_mass = std::max(0.0001f, mass);
		m_inverseMass = 1.0f / m_mass;
	}

	bool IsStatic() const { return m_isStatic; }
	void SetStatic(bool isStatic) {
		m_isStatic = isStatic;
		if (m_isStatic) {
			m_mass = 0.0f;
			m_inverseMass = 0.0f;
			m_accumulatedForce = glm::vec3(0.0f);
			m_linearVelocity = glm::vec3(0.0f);
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

	void ClearForces() { m_accumulatedForce = glm::vec3(0.0f); }

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
		ClearForces();
	}

private:
	uint32_t m_id = 0;
	glm::vec3 m_position = glm::vec3(0.0f);
	glm::vec3 m_linearVelocity = glm::vec3(0.0f);
	glm::vec3 m_accumulatedForce = glm::vec3(0.0f);
	float m_mass = 1.0f;
	float m_inverseMass = 1.0f;
	bool m_isStatic = false;
	bool m_useGravity = true;
};

} // namespace Physics
} // namespace Runtime
