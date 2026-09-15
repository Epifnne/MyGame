#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <glm/glm.hpp>

#include "Collider.h"
#include "ContactManifold.h"
#include "PhysicsMaterial.h"
#include "RigidBody.h"

namespace Runtime {
namespace Physics {

// Substep-temporary prepared data for one impulse axis (normal or tangent).
// All response terms are precomputed at Prepare time from the frozen body
// state and stay valid only for the current sub-step; they are never
// persisted across pose changes.
struct PreparedContactAxis {
    // cross(ra, axis) / cross(rb, axis): angular impulse per unit lambda.
    glm::vec3 raCrossAxis = glm::vec3(0.0f);
    glm::vec3 rbCrossAxis = glm::vec3(0.0f);
    // invInertiaWorld * (r cross axis): delta omega per unit lambda, using the
    // same inverse inertia matrices as the effective-mass denominator so the
    // predicted and the actually applied velocity response stay identical.
    glm::vec3 invInertiaATimesRaCross = glm::vec3(0.0f);
    glm::vec3 invInertiaBTimesRbCross = glm::vec3(0.0f);
    // 1 / K with K the projected inverse mass along the axis; zero when the
    // axis cannot transmit impulse (both bodies immovable along it).
    float effectiveMass = 0.0f;
};

// Substep-temporary prepared data for one contact point. The accumulated
// lambdas live here during the sub-step (initialized from the warm start
// cache) and are stored back into the persistent ContactPoint at sub-step
// end via CommitSolvedImpulses.
struct PreparedContactPoint {
    // Write-back target in the persistent midphase manifold.
    ContactPoint* persistent = nullptr;
    glm::vec3 ra = glm::vec3(0.0f);
    glm::vec3 rb = glm::vec3(0.0f);
    // One-shot restitution target velocity computed at Prepare from the
    // pre-warm-start incoming velocity (zero below the low-speed threshold).
    // Penetration has no velocity bias; position correction handles it.
    float restitutionBias = 0.0f;
    // Accumulated impulse of this sub-step (warm start value included).
    float accumulatedNormal = 0.0f;
    // Accumulated friction impulse expressed in (tangent1, tangent2).
    glm::vec2 accumulatedTangent = glm::vec2(0.0f);
    PreparedContactAxis normalAxis;
    PreparedContactAxis tangentAxis1;
    PreparedContactAxis tangentAxis2;
};

// Substep-temporary prepared constraint for one contact manifold.
struct PreparedContactConstraint {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;
    // Effective contact normal after one-sided reorientation, with its
    // deterministic orthonormal tangent basis.
    glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 tangent1 = glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 tangent2 = glm::vec3(0.0f, 0.0f, 1.0f);
    // Combined dynamic friction coefficient (friction disc radius factor).
    float friction = 0.0f;
    float substepDt = 0.0f;
    // Midphase slot this constraint was prepared from (for cache store-back
    // and fixed-step impulse accounting).
    uint32_t slotIndex = 0;
    std::size_t pointCount = 0;
    std::array<PreparedContactPoint, ContactManifold::kMaxContactPoints> points{};
};

// Sequential impulse contact solver (Phase 4 pipeline):
//   Prepare once -> WarmStart once -> SolveVelocityIteration N times
//   -> ResolvePosition once -> CommitSolvedImpulses once.
// Velocity solving uses the accumulated incremental form: each iteration
// computes a delta lambda, clamps the ACCUMULATED normal impulse to
// lambda_n >= 0 and the accumulated tangent impulse to the friction disc
// ||lambda_t|| <= mu * lambda_n, then applies only the difference to the
// accumulated value (negative deltas retract an over-large warm start).
class ContactSolver {
public:
    // Build the substep-temporary constraint from the persistent manifold and
    // the frozen body state: contact arms, stable tangent basis, per-axis
    // cross products and effective masses, restitution bias and the warm
    // start initial lambdas (dt scaling, tangent reprojection, disc clamp).
    // Also applies the one-sided normal reorientation to the manifold.
    void Prepare(
        PreparedContactConstraint& outConstraint,
        uint32_t slotIndex,
        ContactManifold& contact,
        RigidBody& bodyA,
        RigidBody& bodyB,
        const Collider& colliderA,
        const Collider& colliderB,
        float substepDt) const;

    // Apply the warm start initial lambdas exactly once per sub-step. TOI
    // impact contacts (current or cache origin) start from zero.
    static void WarmStart(PreparedContactConstraint& constraint);

    // One accumulated-increment velocity iteration over all prepared points.
    static void SolveVelocityIteration(PreparedContactConstraint& constraint);

    // Remove residual penetration by positional correction (unchanged split:
    // slop/ratio based, runs once per sub-step, no re-detection).
    void ResolvePosition(
        ContactManifold& contact,
        RigidBody& bodyA,
        RigidBody& bodyB) const;

    // Store the solved accumulated lambdas back into the persistent manifold
    // points together with the current tangent basis and sub-step dt, and set
    // the telemetry normalImpulse to this sub-step's accumulated lambda.
    static void CommitSolvedImpulses(PreparedContactConstraint& constraint);

private:
    // Reorient the contact normal for one-sided static colliders (front/back
    // determination is the closing-velocity check along this normal).
    static void ApplyOneSidedNormal(
        ContactManifold& contact,
        const RigidBody& bodyA,
        const RigidBody& bodyB,
        const Collider& colliderA,
        const Collider& colliderB);

    // Precompute one impulse axis against the frozen body state.
    static PreparedContactAxis BuildAxis(
        const glm::vec3& axis,
        const glm::vec3& ra,
        const glm::vec3& rb,
        float invMassA,
        float invMassB,
        const glm::mat3& invInertiaA,
        const glm::mat3& invInertiaB);

    // Apply axis * lambda to the body pair: linear velocity, angular velocity
    // and L change together through the solver-only entry (no external wake
    // request), with the precomputed angular response.
    static void ApplyAxisImpulse(
        RigidBody& bodyA,
        RigidBody& bodyB,
        const glm::vec3& axis,
        const PreparedContactAxis& axisData,
        float lambda);

    // Contact-point relative velocity (vB + wB x rb) - (vA + wA x ra).
    static glm::vec3 RelativeContactVelocity(
        const RigidBody& bodyA,
        const RigidBody& bodyB,
        const glm::vec3& ra,
        const glm::vec3& rb);

    // Remove residual penetration by positional correction.
    static void ApplyPositionalCorrection(
        ContactManifold& contact,
        RigidBody& bodyA,
        RigidBody& bodyB);
};

} // namespace Physics
} // namespace Runtime
