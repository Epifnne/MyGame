#include "Physics/ContactSolver.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/quaternion.hpp>

namespace Runtime {
namespace Physics {

namespace {
// NGS correction fraction beta.
constexpr float kCorrectionRatio = 0.2f;
// Pose correction permits 2 cm penetration and caps the corrected error at 20 cm.
constexpr float kPenetrationSlop = 0.02f;
constexpr float kMaxCorrectionDistance = 0.2f;
// Restitution requires incoming v_n < -1 m/s.
constexpr float kRestitutionVelocityThreshold = 1.0f;
constexpr float kEffectiveMassEpsilon = 1e-6f;

// Use point-minus-center arms, except spheres use +/- normal*radius for their surface arms.
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

// Freeze velocity Jacobians and bias targets; lambda = M_eff * (target - relativeSpeed).
void ContactSolver::Prepare(
    PreparedContactConstraint& outConstraint,
    uint32_t slotIndex,
    ContactManifold& contact,
    RigidBody& bodyA,
    RigidBody& bodyB,
    const Collider& colliderA,
    const Collider& colliderB,
    float substepDt,
    const glm::vec3& gravity) const {
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
        // Per-point normal arms preserve the angular contact velocity omega cross r.
        prepared.normalRa = prepared.ra;
        prepared.normalRb = prepared.rb;

        prepared.normalAxis = BuildAxis(
            outConstraint.normal, prepared.normalRa, prepared.normalRb,
            invMassA, invMassB, invInertiaA, invInertiaB);

        // Allow separated points to close by at most gap/dt: v_n >= min(0, penetration/dt).
        prepared.restitutionBias = std::min(0.0f, point.penetration / substepDt);

        // Freeze restitution before warm start only if v_n < -1 and v_n*dt < penetration.
        const glm::vec3 incomingVelocity =
            RelativeContactVelocity(bodyA, bodyB, prepared.normalRa, prepared.normalRb);
        const float incomingNormalVelocity = glm::dot(incomingVelocity, outConstraint.normal);
        if (incomingNormalVelocity < -kRestitutionVelocityThreshold &&
            incomingNormalVelocity * substepDt < point.penetration) {
            // Compensate only closing relative force acceleration:
            // bias = -e * (v_n - min(0, dot(aB-aA,n)*dt)).
            const float forceDelta = glm::dot(
                bodyB.LinearAcceleration(gravity) - bodyA.LinearAcceleration(gravity),
                outConstraint.normal) * substepDt;
            const float compensated = incomingNormalVelocity - std::min(0.0f, forceDelta);
            prepared.restitutionBias = -material.restitution * compensated;
        }

        // Initial normal lambda = max(0, cachedLambda * dt/cachedDt).
        if (point.cachedDt > 0.0f) {
            prepared.accumulatedNormal =
                std::max(point.accumulatedNormalImpulse * (substepDt / point.cachedDt), 0.0f);
        }
    }

    // One centroid patch carries two tangent axes and, for multiple points, a spin axis.
    if (outConstraint.pointCount > 0) {
        glm::vec3 centroid(0.0f);
        for (std::size_t index = 0; index < contact.pointCount; ++index) {
            centroid += contact.Point(index).position;
        }
        centroid /= static_cast<float>(contact.pointCount);
        outConstraint.frictionCenterRa = centroid - bodyA.Position();
        outConstraint.frictionCenterRb = centroid - bodyB.Position();
        for (std::size_t index = 0; index < contact.pointCount; ++index) {
            const glm::vec3 offset = contact.Point(index).position - centroid;
            outConstraint.points[index].distanceToFrictionCenter =
                glm::length(offset - glm::dot(offset, outConstraint.normal) * outConstraint.normal);
        }
        outConstraint.tangentAxis1 = BuildAxis(
            outConstraint.tangent1, outConstraint.frictionCenterRa, outConstraint.frictionCenterRb,
            invMassA, invMassB, invInertiaA, invInertiaB);
        outConstraint.tangentAxis2 = BuildAxis(
            outConstraint.tangent2, outConstraint.frictionCenterRa, outConstraint.frictionCenterRb,
            invMassA, invMassB, invInertiaA, invInertiaB);
        outConstraint.frictionActive = true;
        outConstraint.spinFrictionActive = contact.pointCount > 1;
        if (outConstraint.spinFrictionActive) {
            // Pure torque: dL = +/- lambda*n, domega = I^-1*dL.
            // Spin M_eff = 1 / dot(n, (I_A^-1 + I_B^-1)*n) when the denominator > epsilon.
            const glm::vec3 responseA = invInertiaA * outConstraint.normal;
            const glm::vec3 responseB = invInertiaB * outConstraint.normal;
            outConstraint.spinAxis.raCrossAxis = outConstraint.normal;
            outConstraint.spinAxis.rbCrossAxis = outConstraint.normal;
            outConstraint.spinAxis.invInertiaATimesRaCross = responseA;
            outConstraint.spinAxis.invInertiaBTimesRbCross = responseB;
            const float spinDenominator = glm::dot(outConstraint.normal, responseA + responseB);
            if (spinDenominator > kEffectiveMassEpsilon) {
                outConstraint.spinAxis.effectiveMass = 1.0f / spinDenominator;
            }
        }
        // Friction warm start: the whole patch's cached tangent impulse is
        // reprojected onto the new basis and clamped to the disc.
        const ContactPoint& first = contact.Point(0);
        if (first.cachedDt > 0.0f) {
            const float scale = substepDt / first.cachedDt;
            const glm::vec3 worldTangentImpulse =
                first.cachedTangent1 * first.accumulatedTangentImpulse.x +
                first.cachedTangent2 * first.accumulatedTangentImpulse.y;
            outConstraint.accumulatedTangent = glm::vec2(
                glm::dot(worldTangentImpulse, outConstraint.tangent1),
                glm::dot(worldTangentImpulse, outConstraint.tangent2)) * scale;
            outConstraint.accumulatedSpin = first.accumulatedSpinImpulse * scale;
            float normalSum = 0.0f, spinSum = 0.0f;
            for (std::size_t index = 0; index < outConstraint.pointCount; ++index) {
                normalSum += outConstraint.points[index].accumulatedNormal;
                spinSum += outConstraint.points[index].distanceToFrictionCenter *
                    outConstraint.points[index].accumulatedNormal;
            }
            const float maxFriction = outConstraint.friction * normalSum;
            const float maxSpin = outConstraint.friction * spinSum;
            outConstraint.accumulatedSpin = outConstraint.spinFrictionActive ?
                glm::clamp(outConstraint.accumulatedSpin, -maxSpin, maxSpin) : 0.0f;
            const float tangentLengthSq =
                glm::dot(outConstraint.accumulatedTangent, outConstraint.accumulatedTangent);
            if (tangentLengthSq > maxFriction * maxFriction && tangentLengthSq > 0.0f) {
                outConstraint.accumulatedTangent *= maxFriction / std::sqrt(tangentLengthSq);
            }
        }
    }
}

// Apply supplied cached tangent/spin impulses before per-point normal impulses, without wake activity.
void ContactSolver::WarmStart(PreparedContactConstraint& constraint) {
    RigidBody& bodyA = *constraint.bodyA;
    RigidBody& bodyB = *constraint.bodyB;
    if (constraint.frictionActive) {
        ApplyAxisImpulse(bodyA, bodyB, constraint.tangent1, constraint.tangentAxis1,
            constraint.accumulatedTangent.x);
        ApplyAxisImpulse(bodyA, bodyB, constraint.tangent2, constraint.tangentAxis2,
            constraint.accumulatedTangent.y);
        if (constraint.spinFrictionActive && constraint.accumulatedSpin != 0.0f) {
            bodyA.ApplySolverAngularImpulse(
                -constraint.accumulatedSpin * constraint.spinAxis.raCrossAxis,
                -constraint.accumulatedSpin * constraint.spinAxis.invInertiaATimesRaCross);
            bodyB.ApplySolverAngularImpulse(
                constraint.accumulatedSpin * constraint.spinAxis.raCrossAxis,
                constraint.accumulatedSpin * constraint.spinAxis.invInertiaBTimesRbCross);
        }
    }
    for (std::size_t index = 0; index < constraint.pointCount; ++index) {
        const PreparedContactPoint& point = constraint.points[index];
        ApplyAxisImpulse(bodyA, bodyB, constraint.normal, point.normalAxis, point.accumulatedNormal);
    }
}

// Solve centroid tangent/spin friction then point normals, applying only clamped lambda changes.
void ContactSolver::SolveVelocityIteration(PreparedContactConstraint& constraint) {
    RigidBody& bodyA = *constraint.bodyA;
    RigidBody& bodyB = *constraint.bodyB;

    // Friction bounds use the current load before this iteration updates normals:
    // |lambda_t| <= mu*sum(lambda_n), |lambda_spin| <= mu*sum(d_i*lambda_n_i).
    if (constraint.frictionActive) {
        float normalSum = 0.0f;
        float spinSum = 0.0f;
        for (std::size_t index = 0; index < constraint.pointCount; ++index) {
            normalSum += constraint.points[index].accumulatedNormal;
            spinSum += constraint.points[index].distanceToFrictionCenter *
                constraint.points[index].accumulatedNormal;
        }
        const float maxLinear = constraint.friction * normalSum;
        const float maxSpin = constraint.friction * spinSum;

        // Compute both dlambda_t components from the same center velocity, then project onto the disc.
        const glm::vec3 centerVelocity = RelativeContactVelocity(
            bodyA, bodyB, constraint.frictionCenterRa, constraint.frictionCenterRb);
        const glm::vec2 tangentDelta(
            -glm::dot(centerVelocity, constraint.tangent1) * constraint.tangentAxis1.effectiveMass,
            -glm::dot(centerVelocity, constraint.tangent2) * constraint.tangentAxis2.effectiveMass);
        const glm::vec2 oldTangent = constraint.accumulatedTangent;
        glm::vec2 newTangent = oldTangent + tangentDelta;
        const float tangentLengthSq = glm::dot(newTangent, newTangent);
        if (tangentLengthSq > maxLinear * maxLinear && tangentLengthSq > 0.0f) {
            newTangent *= maxLinear / std::sqrt(tangentLengthSq);
        }
        ApplyAxisImpulse(bodyA, bodyB, constraint.tangent1, constraint.tangentAxis1,
            newTangent.x - oldTangent.x);
        ApplyAxisImpulse(bodyA, bodyB, constraint.tangent2, constraint.tangentAxis2,
            newTangent.y - oldTangent.y);
        constraint.accumulatedTangent = newTangent;

        // Torsional friction: cancel the relative spin about the normal,
        // clamped by the distance-weighted normal load.
        if (constraint.spinFrictionActive) {
            const float relativeSpin = glm::dot(
                bodyB.AngularVelocity() - bodyA.AngularVelocity(), constraint.normal);
            const float spinDelta = -relativeSpin * constraint.spinAxis.effectiveMass;
            const float oldSpin = constraint.accumulatedSpin;
            const float newSpin = glm::clamp(oldSpin + spinDelta, -maxSpin, maxSpin);
            const float applied = newSpin - oldSpin;
            if (applied != 0.0f) {
                bodyA.ApplySolverAngularImpulse(
                    -applied * constraint.spinAxis.raCrossAxis,
                    -applied * constraint.spinAxis.invInertiaATimesRaCross);
                bodyB.ApplySolverAngularImpulse(
                    applied * constraint.spinAxis.raCrossAxis,
                    applied * constraint.spinAxis.invInertiaBTimesRbCross);
            }
            constraint.accumulatedSpin = newSpin;
        }
    }

    // Normal rows: lambda' = max(0, lambda + M_eff*(bias-v_n)).
    for (std::size_t index = 0; index < constraint.pointCount; ++index) {
        PreparedContactPoint& point = constraint.points[index];

        // Negative deltas retract excessive impulses even if the point is separating.
        const glm::vec3 relativeVelocity = RelativeContactVelocity(bodyA, bodyB, point.normalRa, point.normalRb);
        const float normalVelocity = glm::dot(relativeVelocity, constraint.normal);
        const float normalDelta =
            (point.restitutionBias - normalVelocity) * point.normalAxis.effectiveMass;
        const float oldNormal = point.accumulatedNormal;
        point.accumulatedNormal = std::max(oldNormal + normalDelta, 0.0f);
        ApplyAxisImpulse(
            bodyA, bodyB, constraint.normal, point.normalAxis,
            point.accumulatedNormal - oldNormal);
    }
}

// Correct C = dot(pB-pA,n)+slop using a freshly evaluated pose Jacobian, without changing velocity.
void ContactSolver::ResolvePosition(
    PreparedContactConstraint& constraint,
    ContactManifold& contact) const {
    RigidBody& bodyA = *constraint.bodyA;
    RigidBody& bodyB = *constraint.bodyB;
    if (contact.isTrigger || (bodyA.IsStatic() && bodyB.IsStatic())) {
        return;
    }

    // NGS relinearizes both the separation and effective mass at each corrected pose.
    const float invMassA = bodyA.InverseMass();
    const float invMassB = bodyB.InverseMass();
    for (std::size_t index = 0; index < contact.pointCount; ++index) {
        const ContactPoint& point = contact.Point(index);
        const glm::vec3 anchorA = bodyA.Position() + bodyA.Orientation() * point.surfaceLocalA;
        const glm::vec3 anchorB = bodyB.Position() + bodyB.Orientation() * point.surfaceLocalB;
        const glm::vec3& normal = constraint.normal;
        // C = separation + slop; positive means close enough, clamped below
        // so a deep penetration cannot apply a huge correction:
        //   lambda = -K^-1 * beta * C   (C < 0 pushes the bodies apart)
        const float separation = std::max(
            glm::dot(anchorB - anchorA, normal) + kPenetrationSlop,
            -kMaxCorrectionDistance);
        if (separation >= 0.0f) {
            continue;
        }
        const glm::vec3 midpoint = 0.5f * (anchorA + anchorB);
        const PreparedContactAxis axis = BuildAxis(normal,
            midpoint - bodyA.Position(), midpoint - bodyB.Position(), invMassA, invMassB,
            bodyA.InverseInertiaTensorWorld(), bodyB.InverseInertiaTensorWorld());
        const float lambda = -axis.effectiveMass * kCorrectionRatio * separation;
        bodyA.ApplyPositionStep(
            -lambda * invMassA * normal,
            -lambda * axis.invInertiaATimesRaCross);
        bodyB.ApplyPositionStep(
            lambda * invMassB * normal,
            lambda * axis.invInertiaBTimesRbCross);
        constraint.ngsDeltaLinearB += lambda * invMassB * normal;
        constraint.ngsAppliedLambda += lambda;
    }
}

// Cache the fixed-step total while publishing only the newly applied substep impulse.
void ContactSolver::CommitSolvedImpulses(PreparedContactConstraint& constraint) {
    for (std::size_t index = 0; index < constraint.pointCount; ++index) {
        const PreparedContactPoint& prepared = constraint.points[index];
        ContactPoint& point = *prepared.persistent;
        point.accumulatedNormalImpulse = prepared.cacheBaseNormal + prepared.accumulatedNormal;
        // Duplicate patch friction on every point; Prepare reads point zero.
        point.accumulatedTangentImpulse = constraint.cacheBaseTangent + constraint.accumulatedTangent;
        point.accumulatedSpinImpulse = constraint.cacheBaseSpin + constraint.accumulatedSpin;
        point.cachedTangent1 = constraint.tangent1;
        point.cachedTangent2 = constraint.tangent2;
        point.cachedDt = constraint.cacheDt > 0.0f ? constraint.cacheDt : constraint.substepDt;
        // Telemetry excludes cacheBase: only this substep's accumulated normal lambda.
        point.normalImpulse = prepared.accumulatedNormal;
    }
}

// Prefer one-sided static A's normal, otherwise negated B's; set normalOnB with nondegenerate normals.
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
            contact.normalOnB = false;
        }
        return;
    }

    if (bodyB.IsStatic() && colliderB.IsOneSided()) {
        const glm::vec3 worldNormal = glm::mat3_cast(bodyB.Orientation()) * colliderB.OneSidedNormalLocal();
        if (glm::dot(worldNormal, worldNormal) > 1e-8f) {
            contact.normal = -glm::normalize(worldNormal);
            contact.normalOnB = true;
        }
    }
}

// Build axis responses; M_eff = 1/(invMassA + invMassB + sum((r cross axis) dot I^-1*(r cross axis))).
// Leave M_eff zero when the denominator <= 1e-6.
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

// Apply opposite linear impulses +/- lambda*axis and precomputed angular dL/domega, without waking.
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
    bodyA.ApplySolverLinearImpulse(-impulse);
    bodyB.ApplySolverLinearImpulse(impulse);
    bodyA.ApplySolverAngularImpulse(
        -lambda * axisData.raCrossAxis, -lambda * axisData.invInertiaATimesRaCross);
    bodyB.ApplySolverAngularImpulse(
        lambda * axisData.rbCrossAxis, lambda * axisData.invInertiaBTimesRbCross);
}

// Return (vB + omegaB cross rb) - (vA + omegaA cross ra) in world space.
glm::vec3 ContactSolver::RelativeContactVelocity(
    const RigidBody& bodyA,
    const RigidBody& bodyB,
    const glm::vec3& ra,
    const glm::vec3& rb) {
    const glm::vec3 velocityA = bodyA.LinearVelocity() + glm::cross(bodyA.AngularVelocity(), ra);
    const glm::vec3 velocityB = bodyB.LinearVelocity() + glm::cross(bodyB.AngularVelocity(), rb);
    return velocityB - velocityA;
}

} // namespace Physics
} // namespace Runtime
