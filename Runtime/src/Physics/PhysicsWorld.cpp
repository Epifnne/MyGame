#include "Physics/PhysicsWorld.h"

#include <algorithm>
#include <utility>

namespace Runtime {
namespace Physics {

PhysicsWorld::PhysicsWorld()
    : m_integrator(std::make_unique<SemiImplicitEulerIntegrator>()) {}

uint32_t PhysicsWorld::CreateRigidBody(const RigidBodyDesc& desc) {
    const uint32_t id = m_nextBodyId++;
    RigidBody body(desc);
    body.SetId(id);
    m_bodies[id] = body;
    return id;
}

bool PhysicsWorld::DestroyRigidBody(uint32_t bodyId) {
    const bool bodyRemoved = m_bodies.erase(bodyId) > 0;
    m_colliders.erase(bodyId);
    return bodyRemoved;
}

bool PhysicsWorld::HasRigidBody(uint32_t bodyId) const {
    return m_bodies.find(bodyId) != m_bodies.end();
}

RigidBody* PhysicsWorld::GetRigidBody(uint32_t bodyId) {
    auto it = m_bodies.find(bodyId);
    if (it == m_bodies.end()) {
        return nullptr;
    }
    return &it->second;
}

const RigidBody* PhysicsWorld::GetRigidBody(uint32_t bodyId) const {
    auto it = m_bodies.find(bodyId);
    if (it == m_bodies.end()) {
        return nullptr;
    }
    return &it->second;
}

bool PhysicsWorld::AttachCollider(uint32_t bodyId, const ColliderDesc& desc) {
    auto bodyIt = m_bodies.find(bodyId);
    if (bodyIt == m_bodies.end() || !desc.shape) {
        return false;
    }

    Collider collider(desc);
    collider.SetBodyId(bodyId);
    m_colliders[bodyId] = std::move(collider);
    return true;
}

bool PhysicsWorld::RemoveCollider(uint32_t bodyId) {
    return m_colliders.erase(bodyId) > 0;
}

bool PhysicsWorld::HasCollider(uint32_t bodyId) const {
    return m_colliders.find(bodyId) != m_colliders.end();
}

Collider* PhysicsWorld::GetCollider(uint32_t bodyId) {
    auto it = m_colliders.find(bodyId);
    if (it == m_colliders.end()) {
        return nullptr;
    }
    return &it->second;
}

const Collider* PhysicsWorld::GetCollider(uint32_t bodyId) const {
    auto it = m_colliders.find(bodyId);
    if (it == m_colliders.end()) {
        return nullptr;
    }
    return &it->second;
}

void PhysicsWorld::Step(float deltaTime) {
    if (deltaTime <= 0.0f) {
        return;
    }

    m_accumulator += deltaTime;
    while (m_accumulator >= m_fixedTimeStep) {
        FixedStep(m_fixedTimeStep);
        m_accumulator -= m_fixedTimeStep;
    }
}

void PhysicsWorld::IntegrateBodies(float dt) {
    if (dt <= 0.0f || !m_integrator) {
        return;
    }
    for (auto& kv : m_bodies) {
        m_integrator->Integrate(kv.second, dt, m_gravity);
    }
}

void PhysicsWorld::SolveContactsIterative() {
    for (int i = 0; i < m_solverIterations; ++i) {
        m_contacts = m_collisionDetector.Detect(m_colliders, m_bodies);
        if (m_contacts.empty()) {
            break;
        }
        ResolveContacts();
    }
}

void PhysicsWorld::FixedStep(float dt) {
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

void PhysicsWorld::ResolveContacts() {
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

} // namespace Physics
} // namespace Runtime
