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
    // Solve one contact manifold by applying impulses and penetration correction.
    void Resolve(
        ContactManifold& contact,
        RigidBody& bodyA,
        RigidBody& bodyB,
        const Collider& colliderA,
        const Collider& colliderB) const;

private:
    struct OneSidedContext {
        // Dynamic body affected by one-sided plane logic, if any.
        RigidBody* dynamicBody = nullptr;
        glm::vec3 surfaceNormal = glm::vec3(0.0f);
        float preNormalVelocity = 0.0f;
        float angularScale = 1.0f;
    };

    // Reorient normal and setup one-sided metadata when needed.
    static OneSidedContext ConfigureOneSidedNormal(
        ContactManifold& contact,
        RigidBody& bodyA,
        RigidBody& bodyB,
        const Collider& colliderA,
        const Collider& colliderB);

    // Ensure objects are closing along contact normal before impulse solve.
    static bool EnsureClosingVelocity(
        ContactManifold& contact,
        const glm::vec3& relativeVelocity,
        float linearVelocityAlongNormal,
        float& velocityAlongNormal);

    // Compute effective mass denominator for an impulse axis.
    static float ComputeAxisDenominator(
        const glm::vec3& axis,
        const glm::vec3& ra,
        const glm::vec3& rb,
        float invMassA,
        float invMassB,
        const glm::mat3& invInertiaA,
        const glm::mat3& invInertiaB);

    // Apply equal and opposite impulse to a body pair.
    static void ApplyImpulsePair(
        RigidBody& bodyA,
        RigidBody& bodyB,
        const glm::vec3& ra,
        const glm::vec3& rb,
        const glm::vec3& impulse);

    // Remove residual penetration by positional correction.
    static void ApplyPositionalCorrection(
        ContactManifold& contact,
        RigidBody& bodyA,
        RigidBody& bodyB);

    // Add a small rebound boost for one-sided surface interactions.
    static void ApplyOneSidedBounceBoost(
        const OneSidedContext& oneSided,
        const PhysicsMaterial& material);
};

} // namespace Physics
} // namespace Runtime
