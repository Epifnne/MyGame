#include "Physics/RigidBody.h"

#include <algorithm>

namespace Runtime {
namespace Physics {

// Normalize pose, initialize clamped mass/inertia and derive dynamic L = Iworld * omega.
RigidBody::RigidBody(const RigidBodyDesc& desc)
    : m_position(desc.position),
      m_orientation(glm::normalize(desc.orientation)),
      m_linearVelocity(desc.linearVelocity),
      m_angularVelocity(glm::vec3(0.0f)),
      m_allowSleep(desc.allowSleep),
      m_isStatic(desc.isStatic),
      m_useGravity(desc.useGravity) {
    SetInertiaTensorDiagonal(desc.inertiaTensorDiagonal);
    SetMass(desc.mass);
    if (!m_isStatic) {
        m_angularVelocity = desc.angularVelocity;
        m_angularMomentum = InverseInertiaTensorWorldInverse() * m_angularVelocity;
    }
    // Construction-time setter side effects are not external activity.
    m_externalActivity = false;
}

// External teleport: normalize q, increment pose/transform versions and refresh inertia/omega.
void RigidBody::SetOrientation(const glm::quat& orientation) {
    m_orientation = glm::normalize(orientation);
    ++m_poseRevision;
    ++m_transformVersion;
    m_externalActivity = true;
    RefreshInverseInertiaTensorWorld();
    UpdateAngularVelocityFromMomentum();
}

// Set L = Iworld * velocity and omega = Iworld^-1 * L; mark external activity.
void RigidBody::SetAngularVelocity(const glm::vec3& velocity) {
    m_angularMomentum = InverseInertiaTensorWorldInverse() * velocity;
    UpdateAngularVelocityFromMomentum();
    m_externalActivity = true;
}

// Assign world momentum and refresh omega; static bodies keep the assigned L with zero omega.
void RigidBody::SetAngularMomentum(const glm::vec3& momentum) {
    m_angularMomentum = momentum;
    UpdateAngularVelocityFromMomentum();
    m_externalActivity = true;
}

// Cache R * diag(1/I) * R^T for a normalized orientation, or zero for static bodies.
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

// Increment structural revision/activity; store max(mass, 0.0001) and its reciprocal, or static zeros.
void RigidBody::SetMass(float mass) {
    ++m_structureRevision;
    m_externalActivity = true;
    if (m_isStatic) {
        m_mass = 0.0f;
        m_inverseMass = 0.0f;
        return;
    }

    m_mass = std::max(0.0001f, mass);
    m_inverseMass = 1.0f / m_mass;
}

// Clamp principal inertia to >= 0.0001, refresh reciprocal/world inertia and omega, and mark activity.
void RigidBody::SetInertiaTensorDiagonal(const glm::vec3& inertiaDiagonal) {
    ++m_structureRevision;
    m_externalActivity = true;
    m_inertiaTensorDiagonal = glm::max(inertiaDiagonal, glm::vec3(0.0001f));
    m_inverseInertiaTensorDiagonal = glm::vec3(
        1.0f / m_inertiaTensorDiagonal.x,
        1.0f / m_inertiaTensorDiagonal.y,
        1.0f / m_inertiaTensorDiagonal.z);
    RefreshInverseInertiaTensorWorld();
    UpdateAngularVelocityFromMomentum();
}

// Mark a structural write, reset sleep state and inertia; static bodies lose forces and motion.
// Returning to dynamic restores unit mass only when stored mass is nonpositive.
void RigidBody::SetStatic(bool isStatic) {
    ++m_structureRevision;
    m_externalActivity = true;
    m_isStatic = isStatic;
    m_sleepState = SleepState::Awake;
    m_sleepTimer = 0.0f;
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

// Add world force and mark external activity unless static.
void RigidBody::ApplyForce(const glm::vec3& force) {
    if (m_isStatic) {
        return;
    }
    m_accumulatedForce += force;
    m_externalActivity = true;
}

// Add world torque and mark external activity unless static.
void RigidBody::ApplyTorque(const glm::vec3& torque) {
    if (m_isStatic) {
        return;
    }
    m_accumulatedTorque += torque;
    m_externalActivity = true;
}

// Dynamic response: v += impulse/m, with external activity.
void RigidBody::ApplyLinearImpulse(const glm::vec3& impulse) {
    if (m_isStatic) {
        return;
    }
    m_linearVelocity += impulse * m_inverseMass;
    m_externalActivity = true;
}

// Dynamic response: L += impulse and omega += Iworld^-1 * impulse, with external activity.
void RigidBody::ApplyAngularImpulse(const glm::vec3& impulse) {
    if (m_isStatic) {
        return;
    }
    m_angularMomentum += impulse;
    m_angularVelocity += m_inverseInertiaTensorWorld * impulse;
    m_externalActivity = true;
}

// Dynamic response: v += impulse/m without external activity or sleep-state changes.
void RigidBody::ApplySolverLinearImpulse(const glm::vec3& impulse) {
    if (m_isStatic) {
        return;
    }
    m_linearVelocity += impulse * m_inverseMass;
}

// Add the supplied dynamic dL and domega without waking; consistency depends on caller-precomputed inertia.
void RigidBody::ApplySolverAngularImpulse(const glm::vec3& angularImpulse, const glm::vec3& deltaAngularVelocity) {
    if (m_isStatic) {
        return;
    }
    m_angularMomentum += angularImpulse;
    m_angularVelocity += deltaAngularVelocity;
}

// Zero accumulated force and torque while preserving the external-activity flag.
void RigidBody::ClearForces() {
    m_accumulatedForce = glm::vec3(0.0f);
    m_accumulatedTorque = glm::vec3(0.0f);
}

// Semi-implicit Euler: integrate velocity then pose for dynamic bodies with dt > 0.
void RigidBody::Integrate(float dt, const glm::vec3& gravity) {
    if (m_isStatic || dt <= 0.0f) {
        return;
    }
    IntegrateVelocity(dt, gravity);
    IntegratePositions(dt);
    // Forces remain locked for subsequent substeps until ClearForces.
}

// Evaluate a = F/m + g using the same force state as the fixed-step integrator.
glm::vec3 RigidBody::LinearAcceleration(const glm::vec3& gravity) const {
    if (m_isStatic) return glm::vec3(0.0f);
    return m_accumulatedForce * m_inverseMass + (m_useGravity ? gravity : glm::vec3(0.0f));
}

// For dynamic bodies and dt > 0: v += a*dt, L += torque*dt, omega = Iworld^-1 * L.
void RigidBody::IntegrateVelocity(float dt, const glm::vec3& gravity) {
    if (m_isStatic || dt <= 0.0f) {
        return;
    }
    m_linearVelocity += LinearAcceleration(gravity) * dt;
    m_angularMomentum += m_accumulatedTorque * dt;
    UpdateAngularVelocityFromMomentum();
}

// For dynamic bodies and dt > 0: x += v*dt, q = normalize(q + 0.5*dt*(0,omega)*q).
// Refresh inertia and omega at the new pose without changing L or external revisions.
void RigidBody::IntegratePositions(float dt) {
    if (m_isStatic || dt <= 0.0f) {
        return;
    }
    m_position += m_linearVelocity * dt;
    ++m_transformVersion;

    const glm::quat omega(0.0f, m_angularVelocity.x, m_angularVelocity.y, m_angularVelocity.z);
    m_orientation += 0.5f * dt * (omega * m_orientation);
    m_orientation = glm::normalize(m_orientation);
    RefreshInverseInertiaTensorWorld();
    UpdateAngularVelocityFromMomentum();
}

// Dynamic pose correction: x += dx, q = normalize(q + 0.5*(0,dtheta)*q); keep v, omega and L.
// Each nonzero translation/rotation increments transform version; rotation refreshes only inertia.
void RigidBody::ApplyPositionStep(const glm::vec3& deltaPosition, const glm::vec3& deltaRotation) {
    if (m_isStatic) {
        return;
    }
    if (glm::dot(deltaPosition, deltaPosition) > 0.0f) {
        m_position += deltaPosition;
        ++m_transformVersion;
    }
    if (glm::dot(deltaRotation, deltaRotation) > 0.0f) {
        m_orientation = glm::normalize(
            m_orientation + 0.5f * glm::quat(0.0f, deltaRotation.x, deltaRotation.y, deltaRotation.z) * m_orientation);
        ++m_transformVersion;
        RefreshInverseInertiaTensorWorld();
    }
}

// Update sleep permission and raise activity only on a value change; do not transition sleep state.
void RigidBody::SetAllowSleep(bool allowSleep) {
    if (m_allowSleep == allowSleep) {
        return;
    }
    m_allowSleep = allowSleep;
    m_externalActivity = true;
}

// Add dt unless static/sleeping; a positive resulting timer marks Candidate.
void RigidBody::AdvanceSleepTimer(float dt) {
    if (m_isStatic || m_sleepState == SleepState::Sleeping) {
        return;
    }
    m_sleepTimer += dt;
    if (m_sleepTimer > 0.0f) {
        m_sleepState = SleepState::Candidate;
    }
}

// Clear rest time and change Candidate to Awake, preserving Sleeping.
void RigidBody::ResetSleepTimer() {
    m_sleepTimer = 0.0f;
    if (m_sleepState == SleepState::Candidate) {
        m_sleepState = SleepState::Awake;
    }
}

// Test center/probe displacement against maxDisplacement^2; reanchor and succeed if the arm changes.
// A failed test leaves reference pose and timer unchanged for the caller to handle.
bool RigidBody::SleepDisplacementWithin(float maxDisplacement, float probeArmLength) {
    if (m_sleepProbeArm != probeArmLength) {
        m_sleepProbeArm = probeArmLength;
        m_sleepRefPosition = m_position;
        m_sleepRefOrientation = m_orientation;
        return true;
    }
    const glm::vec3 centerDelta = m_position - m_sleepRefPosition;
    if (glm::dot(centerDelta, centerDelta) > maxDisplacement * maxDisplacement) {
        return false;
    }
    // World probe displacement combines translation and rotation: dx + (R - Rref)*r.
    if (probeArmLength > 0.0f) {
        for (int axis = 0; axis < 3; ++axis) {
            glm::vec3 probeLocal(0.0f);
            probeLocal[axis] = probeArmLength;
            const glm::vec3 probeDelta = centerDelta +
                (m_orientation * probeLocal) - (m_sleepRefOrientation * probeLocal);
            if (glm::dot(probeDelta, probeDelta) > maxDisplacement * maxDisplacement)
                return false;
        }
    }
    return true;
}

// Set the sleep reference pose to the current pose while retaining probe length.
void RigidBody::ResetSleepDisplacement() {
    m_sleepRefPosition = m_position;
    m_sleepRefOrientation = m_orientation;
}

// Enter Sleeping for a dynamic body, clear rest time and zero v, omega and L; retain forces.
void RigidBody::EnterSleep() {
    if (m_isStatic) {
        return;
    }
    m_sleepState = SleepState::Sleeping;
    m_sleepTimer = 0.0f;
    m_linearVelocity = glm::vec3(0.0f);
    m_angularVelocity = glm::vec3(0.0f);
    m_angularMomentum = glm::vec3(0.0f);
}

// Set Awake and clear rest time without touching velocities, forces or the displacement reference.
void RigidBody::WakeUp() {
    m_sleepState = SleepState::Awake;
    m_sleepTimer = 0.0f;
}

// Build Iworld = R * diag(I) * R^T, or return zero for static bodies.
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

// Set omega = cached Iworld^-1 * L, or zero if static; never modify L.
void RigidBody::UpdateAngularVelocityFromMomentum() {
    if (m_isStatic) {
        m_angularVelocity = glm::vec3(0.0f);
        return;
    }
    m_angularVelocity = m_inverseInertiaTensorWorld * m_angularMomentum;
}

} // namespace Physics
} // namespace Runtime
