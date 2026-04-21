#pragma once

#include <glm/glm.hpp>

#include "Collider.h"
#include "ContactManifold.h"
#include "PhysicsMaterial.h"
#include "RigidBody.h"

namespace Runtime {
namespace Physics {

class ContactSolver {
public:
    void Resolve(
        ContactManifold& contact,
        RigidBody& bodyA,
        RigidBody& bodyB,
        const Collider& colliderA,
        const Collider& colliderB) const;

private:
    struct OneSidedContext {
        RigidBody* dynamicBody = nullptr;
        glm::vec3 surfaceNormal = glm::vec3(0.0f);
        float preNormalVelocity = 0.0f;
        float angularScale = 1.0f;
    };

    static OneSidedContext ConfigureOneSidedNormal(
        ContactManifold& contact,
        RigidBody& bodyA,
        RigidBody& bodyB,
        const Collider& colliderA,
        const Collider& colliderB);

    static bool EnsureClosingVelocity(
        ContactManifold& contact,
        const glm::vec3& relativeVelocity,
        float linearVelocityAlongNormal,
        float& velocityAlongNormal);

    static float ComputeAxisDenominator(
        const glm::vec3& axis,
        const glm::vec3& ra,
        const glm::vec3& rb,
        float invMassA,
        float invMassB,
        const glm::mat3& invInertiaA,
        const glm::mat3& invInertiaB);

    static void ApplyImpulsePair(
        RigidBody& bodyA,
        RigidBody& bodyB,
        const glm::vec3& ra,
        const glm::vec3& rb,
        const glm::vec3& impulse);

    static void ApplyPositionalCorrection(
        ContactManifold& contact,
        RigidBody& bodyA,
        RigidBody& bodyB);

    static void ApplyOneSidedBounceBoost(
        const OneSidedContext& oneSided,
        const PhysicsMaterial& material);
};

} // namespace Physics
} // namespace Runtime
