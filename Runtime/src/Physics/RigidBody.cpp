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
    ++m_poseRevision;
    RefreshInverseInertiaTensorWorld();
    UpdateAngularVelocityFromMomentum();
}

void RigidBody::SetAngularVelocity(const glm::vec3& velocity) {
    // L is authoritative: derive momentum from the requested velocity, then
    // re-derive omega through the cached inverse inertia (static bodies end
    // up with zero L and zero omega).
    m_angularMomentum = InverseInertiaTensorWorldInverse() * velocity;
    UpdateAngularVelocityFromMomentum();
}

void RigidBody::SetAngularMomentum(const glm::vec3& momentum) {
    m_angularMomentum = momentum;
    UpdateAngularVelocityFromMomentum();
}

void RigidBody::RefreshInverseInertiaTensorWorld() {
    if (m_isStatic) {
        m_inverseInertiaTensorWorld = glm::mat3(0.0f);
        return;
    }
    const glm::mat3 r = glm::mat3_cast(glm::normalize(m_orientation));
    const glm::mat3 iBodyInv = glm::mat3(
        glm::vec3(m_inverseInertiaTensorDiagonal.x, 0.0f, 0.0f),
        glm::vec3(0.0f, m_inverseInertiaTensorDiagonal.y, 0.0f),
        glm::vec3(0.0f, 0.0f, m_inverseInertiaTensorDiagonal.z));
    m_inverseInertiaTensorWorld = r * iBodyInv * glm::transpose(r);
}

void RigidBody::SetMass(float mass) {
    ++m_structureRevision;
    if (m_isStatic) {
        m_mass = 0.0f;
        m_inverseMass = 0.0f;
        return;
    }

    m_mass = std::max(0.0001f, mass);
    m_inverseMass = 1.0f / m_mass;
}

void RigidBody::SetInertiaTensorDiagonal(const glm::vec3& inertiaDiagonal) {
    ++m_structureRevision;
    m_inertiaTensorDiagonal = glm::max(inertiaDiagonal, glm::vec3(0.0001f));
    m_inverseInertiaTensorDiagonal = glm::vec3(
        1.0f / m_inertiaTensorDiagonal.x,
        1.0f / m_inertiaTensorDiagonal.y,
        1.0f / m_inertiaTensorDiagonal.z);
    RefreshInverseInertiaTensorWorld();
    UpdateAngularVelocityFromMomentum();
}

void RigidBody::SetStatic(bool isStatic) {
    ++m_structureRevision;
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
    RefreshInverseInertiaTensorWorld();
    UpdateAngularVelocityFromMomentum();
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
    // Incremental form of omega = Iworld^-1 * L using the cached matrix.
    m_angularMomentum += impulse;
    m_angularVelocity += m_inverseInertiaTensorWorld * impulse;
}

void RigidBody::ApplySolverAngularImpulse(const glm::vec3& angularImpulse, const glm::vec3& deltaAngularVelocity) {
    if (m_isStatic) {
        return;
    }
    // The solver supplies delta omega precomputed with the same inverse
    // inertia matrix it used for the effective mass, keeping the applied
    // response consistent with the predicted one.
    m_angularMomentum += angularImpulse;
    m_angularVelocity += deltaAngularVelocity;
}

void RigidBody::ClearForces() {
    m_accumulatedForce = glm::vec3(0.0f);
    m_accumulatedTorque = glm::vec3(0.0f);
}

void RigidBody::Integrate(float dt, const glm::vec3& gravity) {
    if (m_isStatic || dt <= 0.0f) {
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
    // Orientation changed: refresh the cached inverse inertia and re-derive
    // omega from the authoritative momentum so they cannot diverge.
    RefreshInverseInertiaTensorWorld();
    UpdateAngularVelocityFromMomentum();
    // Accumulated forces are NOT cleared here: PhysicsWorld clears them once at
    // fixed-step end so CCD sub-steps each integrate the locked forces by own dt.
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
    m_angularVelocity = m_inverseInertiaTensorWorld * m_angularMomentum;
}

} // namespace Physics
} // namespace Runtime
