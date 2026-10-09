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

// Frozen velocity-axis Jacobian, angular responses and scalar effective mass for one substep.
struct PreparedContactAxis {
    // cross(ra, axis) / cross(rb, axis): angular impulse per unit lambda.
    glm::vec3 raCrossAxis = glm::vec3(0.0f);
    glm::vec3 rbCrossAxis = glm::vec3(0.0f);
    // Iworld^-1 * (r cross axis): angular velocity response per unit lambda.
    glm::vec3 invInertiaATimesRaCross = glm::vec3(0.0f);
    glm::vec3 invInertiaBTimesRbCross = glm::vec3(0.0f);
    // 1/K when K > 1e-6; otherwise zero (no response).
    float effectiveMass = 0.0f;
};

// Per-point arms, normal constraint and warm-started lambda; friction belongs to the manifold.
struct PreparedContactPoint {
    // Write-back target in the persistent midphase manifold.
    ContactPoint* persistent = nullptr;
    glm::vec3 ra = glm::vec3(0.0f);
    glm::vec3 rb = glm::vec3(0.0f);
    // Normal velocity/effective-mass arms, currently copied from ra/rb.
    glm::vec3 normalRa = glm::vec3(0.0f);
    glm::vec3 normalRb = glm::vec3(0.0f);
    // Target min(0, penetration/dt), replaced by restitution for fast arriving contacts.
    // Positive penetration has no velocity bias; pose correction handles recovery.
    float restitutionBias = 0.0f;
    // Accumulated normal impulse of this sub-step (warm start value included).
    float accumulatedNormal = 0.0f;
    // Impulse already applied earlier in this fixed step; cache only, never warm-started again.
    float cacheBaseNormal = 0.0f;
    // Planar centroid distance d_i used in |lambda_spin| <= mu * sum(d_i * lambda_n_i).
    float distanceToFrictionCenter = 0.0f;
    PreparedContactAxis normalAxis;
};

// Substep manifold constraint: point normals, centroid friction/spin, cache bases and pose diagnostics.
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
    float cacheDt = 0.0f;
    glm::vec2 cacheBaseTangent = glm::vec2(0.0f);
    float cacheBaseSpin = 0.0f;
    bool velocitySolveEnabled = true;
    // Midphase slot this constraint was prepared from (for cache store-back
    // and fixed-step impulse accounting).
    uint32_t slotIndex = 0;
    std::size_t pointCount = 0;
    std::array<PreparedContactPoint, ContactManifold::kMaxContactPoints> points{};

	// Centroid friction uses the normal load available before this iteration's normal rows:
	//   |lambda_t| <= mu * sum_i lambda_n_i
	//   |lambda_spin| <= mu * sum_i d_i * lambda_n_i
	glm::vec3 frictionCenterRa = glm::vec3(0.0f); // centroid minus position A
	glm::vec3 frictionCenterRb = glm::vec3(0.0f); // centroid minus position B
	bool frictionActive = false;
	// Enable torsional friction only for patches with more than one point.
	bool spinFrictionActive = false;
	glm::vec2 accumulatedTangent = glm::vec2(0.0f);
	float accumulatedSpin = 0.0f;
	PreparedContactAxis tangentAxis1;
	PreparedContactAxis tangentAxis2;
	PreparedContactAxis spinAxis;
	// Net body-B translation and summed lambda from this constraint's pose corrections.
	glm::vec3 ngsDeltaLinearB = glm::vec3(0.0f);
	float ngsAppliedLambda = 0.0f;
};

// Sequential impulse solver: prepare, warm start, iterate friction then normals, and correct poses.
// Velocity rows freeze their Jacobians; pose correction recomputes arms and mass at each point.
class ContactSolver {
public:
    // Reset output, reorient one-sided normals and build axes, bias and centroid friction.
    // With dt > 0, scale caches by dt/cachedDt, reproject tangent impulse and clamp friction bounds.
    // Restitution uses incoming speed before warm start, compensated for closing force/gravity.
    void Prepare(
        PreparedContactConstraint& outConstraint,
        uint32_t slotIndex,
        ContactManifold& contact,
        RigidBody& bodyA,
        RigidBody& bodyB,
        const Collider& colliderA,
        const Collider& colliderB,
        float substepDt,
        const glm::vec3& gravity) const;

    // Apply the supplied accumulated patch tangent/spin impulses, then point normal impulses.
    // Caller controls cache suppression and must invoke this only once for each initial guess.
    static void WarmStart(PreparedContactConstraint& constraint);

    // Update/clamp patch tangent and spin impulses first; update point normal impulses last.
    // lambda_n' = max(0, lambda_n + M_eff*(bias-v_n)); apply only each accumulated-impulse delta.
    static void SolveVelocityIteration(PreparedContactConstraint& constraint);

    // For each point, recompute surface anchors and midpoint arms at the corrected pose.
    // C = max(dot(pB-pA,n)+0.02, -0.2); when C < 0, apply lambda = -0.2*M_eff*C.
    // Recompute M_eff per point using the prepared normal; skip triggers/static-static pairs.
    void ResolvePosition(
        PreparedContactConstraint& constraint,
        ContactManifold& contact) const;

    // Store cacheBase + solved impulses, tangent basis and cacheDt (or substepDt if unset).
    // Duplicate patch friction on each point; normalImpulse reports only this substep's lambda.
    static void CommitSolvedImpulses(PreparedContactConstraint& constraint);

private:
    // Prefer static one-sided A's world normal, else negate B's; stamp the reference side.
    // An eligible A returns even if its local normal is degenerate.
    static void ApplyOneSidedNormal(
        ContactManifold& contact,
        const RigidBody& bodyA,
        const RigidBody& bodyB,
        const Collider& colliderA,
        const Collider& colliderB);

    // Compute K = invMassA + invMassB + sum((r cross axis) dot I^-1*(r cross axis)).
    // Store M_eff = 1/K only when K > 1e-6, plus angular impulse/velocity responses.
    static PreparedContactAxis BuildAxis(
        const glm::vec3& axis,
        const glm::vec3& ra,
        const glm::vec3& rb,
        float invMassA,
        float invMassB,
        const glm::mat3& invInertiaA,
        const glm::mat3& invInertiaB);

    // Apply opposite +/- lambda*axis linear impulses and cached angular responses without waking.
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
};

} // namespace Physics
} // namespace Runtime
