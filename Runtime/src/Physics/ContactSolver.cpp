#include "Physics/ContactSolver.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/quaternion.hpp>

namespace Runtime {
namespace Physics {

namespace {
constexpr float kCorrectionRatio = 0.8f;
constexpr float kPenetrationSlop = 0.001f;
// Below this incoming normal speed restitution is suppressed, so resting and
// slowly closing contacts do not micro-bounce.
constexpr float kRestitutionVelocityThreshold = 1.0f;
constexpr float kEffectiveMassEpsilon = 1e-6f;

void BuildContactArms(
    const ContactManifold& contact,
    const ContactPoint& point,
    const RigidBody& bodyA,
    const RigidBody& bodyB,
    const Collider& colliderA,
    const Collider& colliderB,
    glm::vec3& outRa,
    glm::vec3& outRb) {
    outRa = point.position - bodyA.Position();
    outRb = point.position - bodyB.Position();

    const auto* sphereA = dynamic_cast<const SphereShape*>(colliderA.Shape().get());
    if (sphereA) {
        outRa = contact.normal * sphereA->Radius();
    }

    const auto* sphereB = dynamic_cast<const SphereShape*>(colliderB.Shape().get());
    if (sphereB) {
        outRb = -contact.normal * sphereB->Radius();
    }
}
} // namespace

void ContactSolver::Prepare(
    PreparedContactConstraint& outConstraint,
    uint32_t slotIndex,
    ContactManifold& contact,
    RigidBody& bodyA,
    RigidBody& bodyB,
    const Collider& colliderA,
    const Collider& colliderB,
    float substepDt) const {
    outConstraint = PreparedContactConstraint{};
    outConstraint.slotIndex = slotIndex;
    outConstraint.bodyA = &bodyA;
    outConstraint.bodyB = &bodyB;
    outConstraint.substepDt = substepDt;

    const PhysicsMaterial material = PhysicsMaterial::Combine(colliderA.Material(), colliderB.Material());
    outConstraint.friction = material.dynamicFriction;

    // One-sided static surfaces override the contact normal before any arm,
    // basis or effective-mass computation (and before position correction).
    ApplyOneSidedNormal(contact, bodyA, bodyB, colliderA, colliderB);
    outConstraint.normal = contact.normal;
    BuildContactTangentBasis(outConstraint.normal, outConstraint.tangent1, outConstraint.tangent2);

    const float invMassA = bodyA.InverseMass();
    const float invMassB = bodyB.InverseMass();
    const glm::mat3& invInertiaA = bodyA.InverseInertiaTensorWorld();
    const glm::mat3& invInertiaB = bodyB.InverseInertiaTensorWorld();

    outConstraint.pointCount = contact.pointCount;
    for (std::size_t index = 0; index < contact.pointCount; ++index) {
        ContactPoint& point = contact.Point(index);
        PreparedContactPoint& prepared = outConstraint.points[index];
        prepared.persistent = &point;
        BuildContactArms(contact, point, bodyA, bodyB, colliderA, colliderB, prepared.ra, prepared.rb);

        prepared.normalAxis = BuildAxis(
            outConstraint.normal, prepared.ra, prepared.rb,
            invMassA, invMassB, invInertiaA, invInertiaB);
        prepared.tangentAxis1 = BuildAxis(
            outConstraint.tangent1, prepared.ra, prepared.rb,
            invMassA, invMassB, invInertiaA, invInertiaB);
        prepared.tangentAxis2 = BuildAxis(
            outConstraint.tangent2, prepared.ra, prepared.rb,
            invMassA, invMassB, invInertiaA, invInertiaB);

        // Restitution is a one-shot velocity bias taken from the incoming
        // velocity BEFORE warm start; it never re-applies inside iterations.
        const glm::vec3 incomingVelocity =
            RelativeContactVelocity(bodyA, bodyB, prepared.ra, prepared.rb);
        const float incomingNormalVelocity = glm::dot(incomingVelocity, outConstraint.normal);
        if (incomingNormalVelocity < -kRestitutionVelocityThreshold) {
            prepared.restitutionBias = -material.restitution * incomingNormalVelocity;
        }

        // Warm start initial lambdas per the frozen cache contract: TOI
        // impact contacts (current sub-step or cache origin) start at zero;
        // persistent contacts scale by dtNew/cachedDt, reproject the cached
        // tangent impulse onto the new basis and clamp to the friction disc.
        if (!point.isToiImpact && !point.cacheFromToiImpact && point.cachedDt > 0.0f) {
            const float scale = substepDt / point.cachedDt;
            prepared.accumulatedNormal = std::max(point.accumulatedNormalImpulse * scale, 0.0f);
            const glm::vec3 worldTangentImpulse =
                point.cachedTangent1 * point.accumulatedTangentImpulse.x +
                point.cachedTangent2 * point.accumulatedTangentImpulse.y;
            prepared.accumulatedTangent = glm::vec2(
                glm::dot(worldTangentImpulse, outConstraint.tangent1),
                glm::dot(worldTangentImpulse, outConstraint.tangent2)) * scale;

            const float maxFriction = outConstraint.friction * prepared.accumulatedNormal;
            const float tangentLengthSq = glm::dot(prepared.accumulatedTangent, prepared.accumulatedTangent);
            if (tangentLengthSq > maxFriction * maxFriction && tangentLengthSq > 0.0f) {
                prepared.accumulatedTangent *= maxFriction / std::sqrt(tangentLengthSq);
            }
        }
    }
}

void ContactSolver::WarmStart(PreparedContactConstraint& constraint) {
    RigidBody& bodyA = *constraint.bodyA;
    RigidBody& bodyB = *constraint.bodyB;
    for (std::size_t index = 0; index < constraint.pointCount; ++index) {
        const PreparedContactPoint& point = constraint.points[index];
        ApplyAxisImpulse(bodyA, bodyB, constraint.normal, point.normalAxis, point.accumulatedNormal);
        ApplyAxisImpulse(bodyA, bodyB, constraint.tangent1, point.tangentAxis1, point.accumulatedTangent.x);
        ApplyAxisImpulse(bodyA, bodyB, constraint.tangent2, point.tangentAxis2, point.accumulatedTangent.y);
    }
}

void ContactSolver::SolveVelocityIteration(PreparedContactConstraint& constraint) {
    RigidBody& bodyA = *constraint.bodyA;
    RigidBody& bodyB = *constraint.bodyB;
    for (std::size_t index = 0; index < constraint.pointCount; ++index) {
        PreparedContactPoint& point = constraint.points[index];

        // Normal: clamp the accumulated lambda to lambda_n >= 0 and apply
        // only the delta (a negative delta retracts an over-large warm start
        // or an outdated restitution bias; separating velocity never skips).
        glm::vec3 relativeVelocity = RelativeContactVelocity(bodyA, bodyB, point.ra, point.rb);
        const float normalVelocity = glm::dot(relativeVelocity, constraint.normal);
        const float normalDelta =
            (point.restitutionBias - normalVelocity) * point.normalAxis.effectiveMass;
        const float oldNormal = point.accumulatedNormal;
        point.accumulatedNormal = std::max(oldNormal + normalDelta, 0.0f);
        ApplyAxisImpulse(
            bodyA, bodyB, constraint.normal, point.normalAxis,
            point.accumulatedNormal - oldNormal);

        // Friction: solve both tangents against the updated state, clamp the
        // accumulated vector to the disc of radius mu * lambda_n, apply the
        // per-axis differences.
        relativeVelocity = RelativeContactVelocity(bodyA, bodyB, point.ra, point.rb);
        const glm::vec2 tangentDelta(
            -glm::dot(relativeVelocity, constraint.tangent1) * point.tangentAxis1.effectiveMass,
            -glm::dot(relativeVelocity, constraint.tangent2) * point.tangentAxis2.effectiveMass);
        const glm::vec2 oldTangent = point.accumulatedTangent;
        glm::vec2 newTangent = oldTangent + tangentDelta;
        const float maxFriction = constraint.friction * point.accumulatedNormal;
        const float tangentLengthSq = glm::dot(newTangent, newTangent);
        if (tangentLengthSq > maxFriction * maxFriction && tangentLengthSq > 0.0f) {
            newTangent *= maxFriction / std::sqrt(tangentLengthSq);
        }
        ApplyAxisImpulse(
            bodyA, bodyB, constraint.tangent1, point.tangentAxis1, newTangent.x - oldTangent.x);
        ApplyAxisImpulse(
            bodyA, bodyB, constraint.tangent2, point.tangentAxis2, newTangent.y - oldTangent.y);
        point.accumulatedTangent = newTangent;
    }
}

void ContactSolver::ResolvePosition(
    ContactManifold& contact,
    RigidBody& bodyA,
    RigidBody& bodyB) const {
    if (contact.isTrigger || (bodyA.IsStatic() && bodyB.IsStatic())) {
        return;
    }
    ApplyPositionalCorrection(contact, bodyA, bodyB);
}

void ContactSolver::CommitSolvedImpulses(PreparedContactConstraint& constraint) {
    for (std::size_t index = 0; index < constraint.pointCount; ++index) {
        const PreparedContactPoint& prepared = constraint.points[index];
        ContactPoint& point = *prepared.persistent;
        // The cache is now expressed in this sub-step's tangent basis and dt.
        point.accumulatedNormalImpulse = prepared.accumulatedNormal;
        point.accumulatedTangentImpulse = prepared.accumulatedTangent;
        point.cachedTangent1 = constraint.tangent1;
        point.cachedTangent2 = constraint.tangent2;
        point.cachedDt = constraint.substepDt;
        // Telemetry: accumulated lambda actually applied within this sub-step
        // (warm start included), not a cross-sub-step sum.
        point.normalImpulse = prepared.accumulatedNormal;
    }
}

void ContactSolver::ApplyOneSidedNormal(
    ContactManifold& contact,
    const RigidBody& bodyA,
    const RigidBody& bodyB,
    const Collider& colliderA,
    const Collider& colliderB) {
    if (bodyA.IsStatic() && colliderA.IsOneSided()) {
        const glm::vec3 worldNormal = glm::mat3_cast(bodyA.Orientation()) * colliderA.OneSidedNormalLocal();
        if (glm::dot(worldNormal, worldNormal) > 1e-8f) {
            contact.normal = glm::normalize(worldNormal);
        }
        return;
    }

    if (bodyB.IsStatic() && colliderB.IsOneSided()) {
        const glm::vec3 worldNormal = glm::mat3_cast(bodyB.Orientation()) * colliderB.OneSidedNormalLocal();
        if (glm::dot(worldNormal, worldNormal) > 1e-8f) {
            contact.normal = -glm::normalize(worldNormal);
        }
    }
}

PreparedContactAxis ContactSolver::BuildAxis(
    const glm::vec3& axis,
    const glm::vec3& ra,
    const glm::vec3& rb,
    float invMassA,
    float invMassB,
    const glm::mat3& invInertiaA,
    const glm::mat3& invInertiaB) {
    PreparedContactAxis axisData;
    axisData.raCrossAxis = glm::cross(ra, axis);
    axisData.rbCrossAxis = glm::cross(rb, axis);
    axisData.invInertiaATimesRaCross = invInertiaA * axisData.raCrossAxis;
    axisData.invInertiaBTimesRbCross = invInertiaB * axisData.rbCrossAxis;

    // Full physical angular response: the denominator matches exactly the
    // delta-v/delta-omega applied by ApplyAxisImpulse (no angular scaling).
    const float angularTerm = glm::dot(
        axis,
        glm::cross(axisData.invInertiaATimesRaCross, ra) +
            glm::cross(axisData.invInertiaBTimesRbCross, rb));
    const float denominator = invMassA + invMassB + angularTerm;
    if (denominator > kEffectiveMassEpsilon) {
        axisData.effectiveMass = 1.0f / denominator;
    }
    return axisData;
}

void ContactSolver::ApplyAxisImpulse(
    RigidBody& bodyA,
    RigidBody& bodyB,
    const glm::vec3& axis,
    const PreparedContactAxis& axisData,
    float lambda) {
    if (lambda == 0.0f) {
        return;
    }
    const glm::vec3 impulse = axis * lambda;
    bodyA.ApplyLinearImpulse(-impulse);
    bodyB.ApplyLinearImpulse(impulse);
    // One solver-only call accumulates the angular impulse into L and the
    // precomputed delta omega into omega, so L/omega cannot diverge.
    bodyA.ApplySolverAngularImpulse(
        -lambda * axisData.raCrossAxis, -lambda * axisData.invInertiaATimesRaCross);
    bodyB.ApplySolverAngularImpulse(
        lambda * axisData.rbCrossAxis, lambda * axisData.invInertiaBTimesRbCross);
}

glm::vec3 ContactSolver::RelativeContactVelocity(
    const RigidBody& bodyA,
    const RigidBody& bodyB,
    const glm::vec3& ra,
    const glm::vec3& rb) {
    const glm::vec3 velocityA = bodyA.LinearVelocity() + glm::cross(bodyA.AngularVelocity(), ra);
    const glm::vec3 velocityB = bodyB.LinearVelocity() + glm::cross(bodyB.AngularVelocity(), rb);
    return velocityB - velocityA;
}

void ContactSolver::ApplyPositionalCorrection(
    ContactManifold& contact,
    RigidBody& bodyA,
    RigidBody& bodyB) {
    float deepestPenetration = 0.0f;
    for (std::size_t index = 0; index < contact.pointCount; ++index) {
        deepestPenetration = std::max(deepestPenetration, contact.Point(index).penetration);
    }
    const float penetration = std::max(deepestPenetration - kPenetrationSlop, 0.0f);
    if (penetration <= 0.0f) {
        return;
    }

    const float invMassA = bodyA.InverseMass();
    const float invMassB = bodyB.InverseMass();
    const float invMassSum = invMassA + invMassB;
    if (invMassSum <= 1e-6f) {
        return;
    }

    const glm::vec3 correction = (penetration / invMassSum) * kCorrectionRatio * contact.normal;
    // Internal solver write: no teleport semantics, pair caches stay valid.
    if (!bodyA.IsStatic()) {
        bodyA.SetPositionInternal(bodyA.Position() - correction * invMassA);
    }
    if (!bodyB.IsStatic()) {
        bodyB.SetPositionInternal(bodyB.Position() + correction * invMassB);
    }
}

} // namespace Physics
} // namespace Runtime
