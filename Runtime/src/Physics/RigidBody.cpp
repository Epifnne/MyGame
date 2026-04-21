#include "Physics/RigidBody.h"

#include <algorithm>

namespace Runtime {
namespace Physics {

RigidBody::RigidBody(const RigidBodyDesc& desc)
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

void RigidBody::SetOrientation(const glm::quat& orientation) {
    m_orientation = glm::normalize(orientation);
    UpdateAngularVelocityFromMomentum();
}

void RigidBody::SetAngularVelocity(const glm::vec3& velocity) {
    m_angularVelocity = velocity;
    m_angularMomentum = InverseInertiaTensorWorldInverse() * m_angularVelocity;
}

void RigidBody::SetAngularMomentum(const glm::vec3& momentum) {
    m_angularMomentum = momentum;
    UpdateAngularVelocityFromMomentum();
}

glm::mat3 RigidBody::InverseInertiaTensorWorld() const {
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

void RigidBody::SetMass(float mass) {
    if (m_isStatic) {
        m_mass = 0.0f;
        m_inverseMass = 0.0f;
        return;
    }

    m_mass = std::max(0.0001f, mass);
    m_inverseMass = 1.0f / m_mass;
}

void RigidBody::SetInertiaTensorDiagonal(const glm::vec3& inertiaDiagonal) {
    m_inertiaTensorDiagonal = glm::max(inertiaDiagonal, glm::vec3(0.0001f));
    m_inverseInertiaTensorDiagonal = glm::vec3(
        1.0f / m_inertiaTensorDiagonal.x,
        1.0f / m_inertiaTensorDiagonal.y,
        1.0f / m_inertiaTensorDiagonal.z);
    UpdateAngularVelocityFromMomentum();
}

void RigidBody::SetStatic(bool isStatic) {
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

void RigidBody::ApplyForce(const glm::vec3& force) {
    if (m_isStatic) {
        return;
    }
    m_accumulatedForce += force;
}

void RigidBody::ApplyTorque(const glm::vec3& torque) {
    if (m_isStatic) {
        return;
    }
    m_accumulatedTorque += torque;
}

void RigidBody::ApplyLinearImpulse(const glm::vec3& impulse) {
    if (m_isStatic) {
        return;
    }
    m_linearVelocity += impulse * m_inverseMass;
}

void RigidBody::ApplyAngularImpulse(const glm::vec3& impulse) {
    if (m_isStatic) {
        return;
    }
    m_angularMomentum += impulse;
    UpdateAngularVelocityFromMomentum();
}

void RigidBody::ClearForces() {
    m_accumulatedForce = glm::vec3(0.0f);
    m_accumulatedTorque = glm::vec3(0.0f);
}

void RigidBody::Integrate(float dt, const glm::vec3& gravity) {
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

glm::mat3 RigidBody::InverseInertiaTensorWorldInverse() const {
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

void RigidBody::UpdateAngularVelocityFromMomentum() {
    if (m_isStatic) {
        m_angularVelocity = glm::vec3(0.0f);
        return;
    }
    m_angularVelocity = InverseInertiaTensorWorld() * m_angularMomentum;
}

} // namespace Physics
} // namespace Runtime
