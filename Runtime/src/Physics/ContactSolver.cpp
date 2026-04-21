#include "Physics/ContactSolver.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/quaternion.hpp>

namespace Runtime {
namespace Physics {

namespace {
constexpr float kOneSidedAngularScale = 0.12f;
constexpr float kCorrectionRatio = 0.8f;
constexpr float kPenetrationSlop = 0.001f;

void BuildContactArms(
    const ContactManifold& contact,
    const RigidBody& bodyA,
    const RigidBody& bodyB,
    const Collider& colliderA,
    const Collider& colliderB,
    glm::vec3& outRa,
    glm::vec3& outRb) {
    outRa = contact.point.position - bodyA.Position();
    outRb = contact.point.position - bodyB.Position();

    const auto* sphereA = dynamic_cast<const SphereShape*>(colliderA.Shape().get());
    if (sphereA) {
        outRa = contact.normal * sphereA->Radius();
    }

    const auto* sphereB = dynamic_cast<const SphereShape*>(colliderB.Shape().get());
    if (sphereB) {
        outRb = -contact.normal * sphereB->Radius();
    }
}
}

void ContactSolver::Resolve(
    ContactManifold& contact,
    RigidBody& bodyA,
    RigidBody& bodyB,
    const Collider& colliderA,
    const Collider& colliderB) const {
    if (contact.isTrigger || (bodyA.IsStatic() && bodyB.IsStatic())) {
        return;
    }

    const PhysicsMaterial material = PhysicsMaterial::Combine(colliderA.Material(), colliderB.Material());
    const OneSidedContext oneSided = ConfigureOneSidedNormal(contact, bodyA, bodyB, colliderA, colliderB);

    glm::vec3 ra;
    glm::vec3 rb;
    BuildContactArms(contact, bodyA, bodyB, colliderA, colliderB, ra, rb);

    const glm::vec3 velocityA = bodyA.LinearVelocity() + glm::cross(bodyA.AngularVelocity(), ra);
    const glm::vec3 velocityB = bodyB.LinearVelocity() + glm::cross(bodyB.AngularVelocity(), rb);
    const glm::vec3 relativeVelocity = velocityB - velocityA;
    const float linearVelocityAlongNormal =
        glm::dot(bodyB.LinearVelocity() - bodyA.LinearVelocity(), contact.normal);

    float velocityAlongNormal = 0.0f;
    if (!EnsureClosingVelocity(contact, relativeVelocity, linearVelocityAlongNormal, velocityAlongNormal)) {
        ApplyPositionalCorrection(contact, bodyA, bodyB);
        return;
    }

    const float invMassA = bodyA.InverseMass();
    const float invMassB = bodyB.InverseMass();
    const glm::mat3 invInertiaA = bodyA.InverseInertiaTensorWorld();
    const glm::mat3 invInertiaB = bodyB.InverseInertiaTensorWorld();

    float normalDenom = ComputeAxisDenominator(
        contact.normal,
        ra,
        rb,
        invMassA,
        invMassB,
        invInertiaA,
        invInertiaB);
    if (normalDenom <= 1e-6f) {
        ApplyPositionalCorrection(contact, bodyA, bodyB);
        return;
    }

    const float angularOnly = normalDenom - (invMassA + invMassB);
    normalDenom = invMassA + invMassB + angularOnly * oneSided.angularScale;
    if (normalDenom <= 1e-6f) {
        ApplyPositionalCorrection(contact, bodyA, bodyB);
        return;
    }

    const float normalImpulseMagnitude =
        -(1.0f + material.restitution) * velocityAlongNormal / normalDenom;
    const glm::vec3 normalImpulse = normalImpulseMagnitude * contact.normal;
    ApplyImpulsePair(bodyA, bodyB, ra, rb, normalImpulse);

    const glm::vec3 postVelA = bodyA.LinearVelocity() + glm::cross(bodyA.AngularVelocity(), ra);
    const glm::vec3 postVelB = bodyB.LinearVelocity() + glm::cross(bodyB.AngularVelocity(), rb);
    const glm::vec3 postRelative = postVelB - postVelA;

    glm::vec3 tangent = postRelative - glm::dot(postRelative, contact.normal) * contact.normal;
    if (glm::dot(tangent, tangent) > 1e-8f) {
        tangent = glm::normalize(tangent);
        const float tangentDenom = ComputeAxisDenominator(
            tangent,
            ra,
            rb,
            invMassA,
            invMassB,
            invInertiaA,
            invInertiaB);

        if (tangentDenom > 1e-6f) {
            float tangentImpulseMagnitude = -glm::dot(postRelative, tangent) / tangentDenom;
            const float frictionLimit = std::abs(normalImpulseMagnitude) * material.dynamicFriction;
            tangentImpulseMagnitude = std::clamp(tangentImpulseMagnitude, -frictionLimit, frictionLimit);
            ApplyImpulsePair(bodyA, bodyB, ra, rb, tangentImpulseMagnitude * tangent);
        }
    }

    ApplyPositionalCorrection(contact, bodyA, bodyB);
    ApplyOneSidedBounceBoost(oneSided, material);
    contact.point.normalImpulse = normalImpulseMagnitude;
}

ContactSolver::OneSidedContext ContactSolver::ConfigureOneSidedNormal(
    ContactManifold& contact,
    RigidBody& bodyA,
    RigidBody& bodyB,
    const Collider& colliderA,
    const Collider& colliderB) {
    OneSidedContext context;

    auto setupContext = [&](RigidBody& dynamicBody, const glm::vec3& surfaceNormal) {
        context.dynamicBody = &dynamicBody;
        context.surfaceNormal = surfaceNormal;
        context.preNormalVelocity = glm::dot(dynamicBody.LinearVelocity(), surfaceNormal);
        context.angularScale = kOneSidedAngularScale;
    };

    if (bodyA.IsStatic() && colliderA.IsOneSided()) {
        const glm::vec3 worldNormal = glm::mat3_cast(bodyA.Orientation()) * colliderA.OneSidedNormalLocal();
        if (glm::dot(worldNormal, worldNormal) > 1e-8f) {
            contact.normal = glm::normalize(worldNormal);
            if (!bodyB.IsStatic()) {
                setupContext(bodyB, contact.normal);
            }
        }
        return context;
    }

    if (bodyB.IsStatic() && colliderB.IsOneSided()) {
        const glm::vec3 worldNormal = glm::mat3_cast(bodyB.Orientation()) * colliderB.OneSidedNormalLocal();
        if (glm::dot(worldNormal, worldNormal) > 1e-8f) {
            contact.normal = -glm::normalize(worldNormal);
            if (!bodyA.IsStatic()) {
                setupContext(bodyA, -contact.normal);
            }
        }
    }

    return context;
}

bool ContactSolver::EnsureClosingVelocity(
    ContactManifold& contact,
    const glm::vec3& relativeVelocity,
    float linearVelocityAlongNormal,
    float& velocityAlongNormal) {
    const float contactVelocityAlongNormal = glm::dot(relativeVelocity, contact.normal);
    if (contactVelocityAlongNormal < 0.0f) {
        velocityAlongNormal = contactVelocityAlongNormal;
        return true;
    }

    if (linearVelocityAlongNormal < 0.0f) {
        velocityAlongNormal = linearVelocityAlongNormal;
        return true;
    }

    velocityAlongNormal = contactVelocityAlongNormal;
    return false;
}

float ContactSolver::ComputeAxisDenominator(
    const glm::vec3& axis,
    const glm::vec3& ra,
    const glm::vec3& rb,
    float invMassA,
    float invMassB,
    const glm::mat3& invInertiaA,
    const glm::mat3& invInertiaB) {
    const glm::vec3 raCrossAxis = glm::cross(ra, axis);
    const glm::vec3 rbCrossAxis = glm::cross(rb, axis);
    const float angularTerm = glm::dot(
        axis,
        glm::cross(invInertiaA * raCrossAxis, ra) + glm::cross(invInertiaB * rbCrossAxis, rb));
    return invMassA + invMassB + angularTerm;
}

void ContactSolver::ApplyImpulsePair(
    RigidBody& bodyA,
    RigidBody& bodyB,
    const glm::vec3& ra,
    const glm::vec3& rb,
    const glm::vec3& impulse) {
    bodyA.ApplyLinearImpulse(-impulse);
    bodyB.ApplyLinearImpulse(impulse);
    bodyA.ApplyAngularImpulse(-glm::cross(ra, impulse));
    bodyB.ApplyAngularImpulse(glm::cross(rb, impulse));
}

void ContactSolver::ApplyPositionalCorrection(
    ContactManifold& contact,
    RigidBody& bodyA,
    RigidBody& bodyB) {
    const float penetration = std::max(contact.point.penetration - kPenetrationSlop, 0.0f);
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
    if (!bodyA.IsStatic()) {
        bodyA.SetPosition(bodyA.Position() - correction * invMassA);
    }
    if (!bodyB.IsStatic()) {
        bodyB.SetPosition(bodyB.Position() + correction * invMassB);
    }
}

void ContactSolver::ApplyOneSidedBounceBoost(
    const OneSidedContext& oneSided,
    const PhysicsMaterial& material) {
    if (!oneSided.dynamicBody || glm::dot(oneSided.surfaceNormal, oneSided.surfaceNormal) <= 1e-8f) {
        return;
    }
    if (oneSided.preNormalVelocity >= -0.35f) {
        return;
    }

    const float postNormalVelocity = glm::dot(oneSided.dynamicBody->LinearVelocity(), oneSided.surfaceNormal);
    const float targetBounceVelocity = std::max(-oneSided.preNormalVelocity * material.restitution, 0.0f);
    if (postNormalVelocity >= targetBounceVelocity * 0.55f) {
        return;
    }

    const float deltaV = targetBounceVelocity - postNormalVelocity;
    if (deltaV > 0.0f) {
        oneSided.dynamicBody->SetLinearVelocity(
            oneSided.dynamicBody->LinearVelocity() + oneSided.surfaceNormal * deltaV);
    }
}

} // namespace Physics
} // namespace Runtime
