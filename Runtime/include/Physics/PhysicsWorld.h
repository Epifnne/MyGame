#pragma once

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <vector>

#include <glm/glm.hpp>

#include "Collider.h"
#include "CollisionDetector.h"
#include "ContactSolver.h"
#include "ContinuousCollision.h"
#include "ContactManifold.h"
#include "Integrator.h"
#include "PhysicsMaterial.h"
#include "RigidBody.h"

namespace Runtime {
namespace Physics {

class PhysicsWorld {
public:
	PhysicsWorld()
		: m_integrator(std::make_unique<SemiImplicitEulerIntegrator>()) {}

	void SetGravity(const glm::vec3& gravity) { m_gravity = gravity; }
	const glm::vec3& Gravity() const { return m_gravity; }

	void SetFixedTimeStep(float dt) { m_fixedTimeStep = std::max(0.0001f, dt); }
	float FixedTimeStep() const { return m_fixedTimeStep; }

	void SetContinuousCollisionEnabled(bool enabled) { m_enableCcd = enabled; }
	bool ContinuousCollisionEnabled() const { return m_enableCcd; }

	void SetCcdMaxSubSteps(int steps) { m_ccdMaxSubSteps = std::max(1, steps); }
	int CcdMaxSubSteps() const { return m_ccdMaxSubSteps; }

	void SetSolverIterations(int iterations) { m_solverIterations = std::max(1, iterations); }
	int SolverIterations() const { return m_solverIterations; }

	uint32_t CreateRigidBody(const RigidBodyDesc& desc) {
		const uint32_t id = m_nextBodyId++;
		RigidBody body(desc);
		body.SetId(id);
		m_bodies[id] = body;
		return id;
	}

	bool DestroyRigidBody(uint32_t bodyId) {
		const bool bodyRemoved = m_bodies.erase(bodyId) > 0;
		m_colliders.erase(bodyId);
		return bodyRemoved;
	}

	bool HasRigidBody(uint32_t bodyId) const {
		return m_bodies.find(bodyId) != m_bodies.end();
	}

	RigidBody* GetRigidBody(uint32_t bodyId) {
		auto it = m_bodies.find(bodyId);
		if (it == m_bodies.end()) {
			return nullptr;
		}
		return &it->second;
	}

	const RigidBody* GetRigidBody(uint32_t bodyId) const {
		auto it = m_bodies.find(bodyId);
		if (it == m_bodies.end()) {
			return nullptr;
		}
		return &it->second;
	}

	bool AttachCollider(uint32_t bodyId, const ColliderDesc& desc) {
		auto bodyIt = m_bodies.find(bodyId);
		if (bodyIt == m_bodies.end() || !desc.shape) {
			return false;
		}

		Collider collider(desc);
		collider.SetBodyId(bodyId);
		m_colliders[bodyId] = std::move(collider);
		return true;
	}

	bool RemoveCollider(uint32_t bodyId) {
		return m_colliders.erase(bodyId) > 0;
	}

	bool HasCollider(uint32_t bodyId) const {
		return m_colliders.find(bodyId) != m_colliders.end();
	}

	Collider* GetCollider(uint32_t bodyId) {
		auto it = m_colliders.find(bodyId);
		if (it == m_colliders.end()) {
			return nullptr;
		}
		return &it->second;
	}

	const Collider* GetCollider(uint32_t bodyId) const {
		auto it = m_colliders.find(bodyId);
		if (it == m_colliders.end()) {
			return nullptr;
		}
		return &it->second;
	}

	void Step(float deltaTime) {
		if (deltaTime <= 0.0f) {
			return;
		}

		m_accumulator += deltaTime;
		while (m_accumulator >= m_fixedTimeStep) {
			FixedStep(m_fixedTimeStep);
			m_accumulator -= m_fixedTimeStep;
		}
	}

	const std::vector<ContactManifold>& Contacts() const { return m_contacts; }

	const std::unordered_map<uint32_t, RigidBody>& Bodies() const { return m_bodies; }
	const std::unordered_map<uint32_t, Collider>& Colliders() const { return m_colliders; }

private:
	void IntegrateBodies(float dt) {
		if (dt <= 0.0f || !m_integrator) {
			return;
		}
		for (auto& kv : m_bodies) {
			m_integrator->Integrate(kv.second, dt, m_gravity);
		}
	}

	void SolveContactsIterative() {
		for (int i = 0; i < m_solverIterations; ++i) {
			m_contacts = m_collisionDetector.Detect(m_colliders, m_bodies);
			if (m_contacts.empty()) {
				break;
			}
			ResolveContacts();
		}
	}

	void FixedStep(float dt) {
		if (!m_integrator) {
			return;
		}

		if (!m_enableCcd) {
			IntegrateBodies(dt);
			SolveContactsIterative();
			return;
		}

		float remaining = dt;
		int subStep = 0;
		m_contacts.clear();

		while (remaining > 1e-6f && subStep < m_ccdMaxSubSteps) {
			const TimeOfImpact toi = m_continuousCollision.FindEarliestImpact(m_colliders, m_bodies, remaining);

			float advance = remaining;
			if (toi.hit) {
				advance = std::clamp(toi.toi, 0.0f, remaining);
				if (advance < 1e-5f) {
					advance = std::min(remaining, 1e-4f);
				}
			}

			IntegrateBodies(advance);
			remaining -= advance;

			SolveContactsIterative();

			if (!toi.hit) {
				break;
			}

			++subStep;
		}

		if (remaining > 1e-6f) {
			IntegrateBodies(remaining);
			SolveContactsIterative();
		}
	}

	void ResolveContacts() {
		for (ContactManifold& contact : m_contacts) {
			auto bodyItA = m_bodies.find(contact.bodyA);
			auto bodyItB = m_bodies.find(contact.bodyB);
			auto colliderItA = m_colliders.find(contact.bodyA);
			auto colliderItB = m_colliders.find(contact.bodyB);
			if (bodyItA == m_bodies.end() || bodyItB == m_bodies.end() ||
				colliderItA == m_colliders.end() || colliderItB == m_colliders.end()) {
				continue;
			}

			m_contactSolver.Resolve(
				contact,
				bodyItA->second,
				bodyItB->second,
				colliderItA->second,
				colliderItB->second);
		}
	}

	glm::vec3 m_gravity = glm::vec3(0.0f, -9.81f, 0.0f);
	float m_fixedTimeStep = 1.0f / 60.0f;
	float m_accumulator = 0.0f;
	uint32_t m_nextBodyId = 1;

	std::unordered_map<uint32_t, RigidBody> m_bodies;
	std::unordered_map<uint32_t, Collider> m_colliders;
	std::vector<ContactManifold> m_contacts;
	bool m_enableCcd = true;
	int m_ccdMaxSubSteps = 8;
	int m_solverIterations = 8;

	CollisionDetector m_collisionDetector;
	ContinuousCollisionDetector m_continuousCollision;
	ContactSolver m_contactSolver;
	std::unique_ptr<Integrator> m_integrator;
};

} // namespace Physics
} // namespace Runtime
