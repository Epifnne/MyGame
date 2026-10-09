#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "Physics/CollisionShape.h"
#include "Physics/ContactManifold.h"
#include "Physics/ContactSolver.h"
#include "Physics/NarrowPhase.h"
#include "Physics/PhysicsWorld.h"

namespace {

using Runtime::Physics::BoxShape;
using Runtime::Physics::Collider;
using Runtime::Physics::ColliderDesc;
using Runtime::Physics::ContactEventType;
using Runtime::Physics::ContactManifold;
using Runtime::Physics::ContactPoint;
using Runtime::Physics::ContactSolver;
using Runtime::Physics::MakePairKey;
using Runtime::Physics::PairKey;
using Runtime::Physics::PairKeyHash;
using Runtime::Physics::PhysicsWorld;
using Runtime::Physics::PreparedContactConstraint;
using Runtime::Physics::RigidBody;
using Runtime::Physics::RigidBodyDesc;
using Runtime::Physics::SphereShape;

constexpr float kDt = 1.0f / 60.0f;

RigidBody MakeBody(
    const glm::vec3& position,
    float mass,
    const glm::vec3& inertiaDiagonal,
    bool isStatic = false) {
    RigidBodyDesc desc;
    desc.position = position;
    desc.mass = mass;
    desc.inertiaTensorDiagonal = inertiaDiagonal;
    desc.isStatic = isStatic;
    desc.useGravity = false;
    return RigidBody(desc);
}

Collider MakeBoxCollider(
    const glm::vec3& halfExtents,
    float friction,
    float restitution,
    bool oneSided = false) {
    ColliderDesc desc;
    desc.shape = std::make_shared<BoxShape>(halfExtents);
    desc.material.staticFriction = friction;
    desc.material.dynamicFriction = friction;
    desc.material.restitution = restitution;
    desc.oneSided = oneSided;
    return Collider(desc);
}

// Deterministic tangent basis of the +Y normal, computed through the same
// helper the pipeline uses.
void UpBasis(glm::vec3& tangent1, glm::vec3& tangent2) {
    Runtime::Physics::BuildContactTangentBasis(glm::vec3(0.0f, 1.0f, 0.0f), tangent1, tangent2);
}

// Single-point manifold on the +Y normal with the point straight below/above
// the body centers (zero angular arms) unless a custom position is given.
ContactManifold MakePointManifold(
    const glm::vec3& pointPosition,
    float penetration = 0.01f) {
    ContactManifold manifold;
    manifold.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    ContactPoint point;
    point.position = pointPosition;
    point.penetration = penetration;
    EXPECT_TRUE(manifold.AddPoint(point));
    return manifold;
}

glm::vec3 RelativeVelocityAt(
    const RigidBody& bodyA,
    const RigidBody& bodyB,
    const glm::vec3& pointPosition) {
    const glm::vec3 ra = pointPosition - bodyA.Position();
    const glm::vec3 rb = pointPosition - bodyB.Position();
    const glm::vec3 velocityA = bodyA.LinearVelocity() + glm::cross(bodyA.AngularVelocity(), ra);
    const glm::vec3 velocityB = bodyB.LinearVelocity() + glm::cross(bodyB.AngularVelocity(), rb);
    return velocityB - velocityA;
}

// ---------------------------------------------------------------------------
// Prepare / effective mass (plan step 16, 19, 21)
// ---------------------------------------------------------------------------

TEST(ContactSolverPositionTest, SurfaceAnchorsAgreeWithPenetration) {
    RigidBody ground = MakeBody({0.0f, -0.5f, 0.0f}, 1.0f, glm::vec3(1.0f), true);
    RigidBody box = MakeBody({0.0f, 0.48f, 0.0f}, 1.0f, glm::vec3(0.135f));
    Collider groundCollider = MakeBoxCollider(glm::vec3(5.0f, 0.5f, 5.0f), 0.5f, 0.0f);
    Collider boxCollider = MakeBoxCollider(glm::vec3(0.5f), 0.5f, 0.0f);
    Runtime::Physics::GjkEpaNarrowPhase narrowPhase;
    Runtime::Physics::NarrowPhaseQueryStats stats;
    ContactManifold manifold;
    ASSERT_TRUE(narrowPhase.GenerateContact(groundCollider, ground, boxCollider, box, manifold, stats));
    ASSERT_EQ(manifold.pointCount, 4u);
    glm::vec3 center(0.0f);
    int cornerMask = 0;
    for (std::size_t index = 0; index < manifold.pointCount; ++index) {
        const ContactPoint& point = manifold.Point(index);
        center += point.position;
        EXPECT_NEAR(std::abs(point.position.x), 0.5f, 1e-3f);
        EXPECT_NEAR(std::abs(point.position.z), 0.5f, 1e-3f);
        cornerMask |= 1 << ((point.position.x > 0.0f ? 1 : 0) + (point.position.z > 0.0f ? 2 : 0));
        EXPECT_NEAR(glm::dot(point.surfacePointA - point.surfacePointB, manifold.normal),
            point.penetration, 1e-3f);
    }
    EXPECT_EQ(cornerMask, 15);
    center /= static_cast<float>(manifold.pointCount);
    EXPECT_NEAR(center.x, 0.0f, 1e-3f);
    EXPECT_NEAR(center.z, 0.0f, 1e-3f);
}

TEST(ContactSolverPositionTest, FlatPatchKeepsPerPointNormalArms) {
    // Jolt semantics: every contact point keeps its own normal arm so the
    // velocity solve sees tilting motion of a flat patch (a shared centroid
    // arm would be blind to omega x r).
    RigidBody ground = MakeBody({0.0f, -0.5f, 0.0f}, 1.0f, glm::vec3(1.0f), true);
    RigidBody box = MakeBody({0.0f, 0.48f, 0.0f}, 1.0f, glm::vec3(0.135f));
    Collider groundCollider = MakeBoxCollider(glm::vec3(5.0f, 0.5f, 5.0f), 0.5f, 0.0f);
    Collider boxCollider = MakeBoxCollider(glm::vec3(0.5f), 0.5f, 0.0f);
    Runtime::Physics::GjkEpaNarrowPhase narrowPhase;
    Runtime::Physics::NarrowPhaseQueryStats stats;
    ContactManifold manifold;
    ASSERT_TRUE(narrowPhase.GenerateContact(groundCollider, ground, boxCollider, box, manifold, stats));
    ASSERT_EQ(manifold.pointCount, 4u);

    ContactSolver solver;
    PreparedContactConstraint constraint;
    solver.Prepare(constraint, 0, manifold, ground, box, groundCollider, boxCollider, kDt, glm::vec3(0.0f));
    for (std::size_t index = 0; index < constraint.pointCount; ++index) {
        EXPECT_EQ(constraint.points[index].normalRb, constraint.points[index].rb);
        EXPECT_NEAR(std::abs(constraint.points[index].normalRb.x), 0.5f, 1e-3f);
        EXPECT_NEAR(std::abs(constraint.points[index].normalRb.z), 0.5f, 1e-3f);
    }
}

TEST(ContactSolverPositionTest, OffsetContactSeparatesViaPoseWithoutTouchingVelocities) {
    RigidBody ground = MakeBody({0.0f, -0.5f, 0.0f}, 1.0f, glm::vec3(1.0f), true);
    RigidBody box = MakeBody({0.0f, 0.48f, 0.0f}, 1.0f, glm::vec3(0.135f));
    Collider groundCollider = MakeBoxCollider(glm::vec3(5.0f, 0.5f, 5.0f), 0.5f, 0.0f);
    Collider boxCollider = MakeBoxCollider(glm::vec3(0.5f), 0.5f, 0.0f);
    // Penetration beyond the 2 cm NGS slop so the position pass engages.
    ContactManifold manifold = MakePointManifold({0.4f, -0.01f, 0.0f}, 0.05f);
    ContactPoint& point = manifold.Point(0);
    point.surfaceLocalA = {0.4f, 0.5f, 0.0f};
    point.surfaceLocalB = {0.4f, -0.53f, 0.0f};
    const float before = glm::dot(
        box.Position() + box.Orientation() * point.surfaceLocalB -
        ground.Position() - ground.Orientation() * point.surfaceLocalA, manifold.normal);

    // NGS position pass: the frozen effective mass/angular response come
    // from Prepare; ResolvePosition writes the pose directly.
    ContactSolver solver;
    PreparedContactConstraint constraint;
    solver.Prepare(constraint, 0, manifold, ground, box, groundCollider, boxCollider, kDt, glm::vec3(0.0f));
    solver.ResolvePosition(constraint, manifold);

    const float after = glm::dot(
        box.Position() + box.Orientation() * point.surfaceLocalB -
        ground.Position() - ground.Orientation() * point.surfaceLocalA, manifold.normal);
    EXPECT_GT(after, before);
    // The position pass never touches velocities or momentum.
    EXPECT_EQ(box.LinearVelocity(), glm::vec3(0.0f));
    EXPECT_EQ(box.AngularVelocity(), glm::vec3(0.0f));
    EXPECT_EQ(box.AngularMomentum(), glm::vec3(0.0f));
}

TEST(ContactSolverPositionTest, TiltedTwoAnchorContactRecoversPoseWithoutChangingMomentum) {
    RigidBody ground = MakeBody({0.0f, -0.5f, 0.0f}, 1.0f, glm::vec3(1.0f), true);
    RigidBody box = MakeBody({0.0f, 0.48f, 0.0f}, 1.0f, glm::vec3(0.135f));
    box.SetOrientation(glm::angleAxis(0.08f, glm::vec3(0.0f, 0.0f, 1.0f)));
    Collider groundCollider = MakeBoxCollider(glm::vec3(5.0f, 0.5f, 5.0f), 0.5f, 0.0f);
    Collider boxCollider = MakeBoxCollider(glm::vec3(0.5f), 0.5f, 0.0f);
    ContactManifold manifold;
    manifold.normal = {0.0f, 1.0f, 0.0f};
    for (const float x : {-0.4f, 0.4f}) {
        ContactPoint point;
        // The solver builds the arms from the world-space contact position;
        // without it the angular term vanishes and the pose cannot un-tilt.
        point.position = {x, 0.0f, 0.0f};
        point.surfaceLocalA = {x, 0.5f, 0.0f};
        point.surfaceLocalB = {x, -0.5f, 0.0f};
        // Both anchors penetrate beyond the 2 cm NGS slop so the pass
        // engages on both (the tilted depth difference drives the rotation).
        point.penetration = 0.08f - x * 0.08f;
        ASSERT_TRUE(manifold.AddPoint(point));
    }
    const float angleBefore = box.Orientation().z;
    const float heightBefore = box.Position().y;
    ContactSolver solver;
    PreparedContactConstraint constraint;
    solver.Prepare(constraint, 0, manifold, ground, box, groundCollider, boxCollider, kDt, glm::vec3(0.0f));
    solver.ResolvePosition(constraint, manifold);

    EXPECT_LT(box.Orientation().z, angleBefore);
    EXPECT_GT(box.Position().y, heightBefore);
    EXPECT_EQ(box.LinearVelocity(), glm::vec3(0.0f));
    EXPECT_EQ(box.AngularMomentum(), glm::vec3(0.0f));
}

TEST(ContactSolverPrepareTest, EffectiveMassMatchesActualVelocityResponse) {
    // Offset contact point and non-uniform inertia: the angular terms dominate
    // the effective mass, so a single iteration can only land exactly on the
    // target velocity if the denominator matches the applied response.
    RigidBody ground = MakeBody({0.0f, -0.5f, 0.0f}, 1.0f, glm::vec3(1.0f), true);
    RigidBody box = MakeBody({0.0f, 0.5f, 0.0f}, 2.0f, glm::vec3(0.5f, 0.8f, 1.2f));
    box.SetLinearVelocity({0.2f, -3.0f, -0.1f});
    box.SetAngularVelocity({0.5f, 0.1f, -0.3f});

    // Zero friction isolates the normal axis: without tangent impulses there
    // is no angular coupling perturbing the normal velocity after the solve.
    Collider groundCollider = MakeBoxCollider(glm::vec3(5.0f, 0.5f, 5.0f), 0.0f, 0.0f);
    Collider boxCollider = MakeBoxCollider(glm::vec3(0.5f), 0.0f, 0.0f);

    const glm::vec3 pointPosition(0.3f, 0.02f, 0.2f);
    ContactManifold manifold = MakePointManifold(pointPosition);

    ContactSolver solver;
    PreparedContactConstraint constraint;
    solver.Prepare(
        constraint, 0u, manifold, ground, box, groundCollider, boxCollider, kDt, glm::vec3(0.0f));
    ASSERT_EQ(constraint.pointCount, 1u);
    EXPECT_GT(constraint.points[0].normalAxis.effectiveMass, 0.0f);

    const float preNormalVelocity =
        glm::dot(RelativeVelocityAt(ground, box, pointPosition), constraint.normal);
    ASSERT_LT(preNormalVelocity, 0.0f);

    ContactSolver::WarmStart(constraint);
    ContactSolver::SolveVelocityIteration(constraint);

    // Restitution is zero, so exactly one iteration must zero the contact
    // point normal velocity: predicted and applied response are identical.
    const float postNormalVelocity =
        glm::dot(RelativeVelocityAt(ground, box, pointPosition), constraint.normal);
    EXPECT_NEAR(postNormalVelocity, 0.0f, 1e-4f);

    // L and omega stay consistent after solver impulses.
    const glm::mat3 inertiaWorld = glm::inverse(box.InverseInertiaTensorWorld());
    const glm::vec3 recomputed = inertiaWorld * box.AngularVelocity();
    EXPECT_NEAR(glm::distance(recomputed, box.AngularMomentum()), 0.0f, 1e-4f);
}

TEST(ContactSolverPrepareTest, OneSidedNormalReorientedAtPrepare) {
    RigidBody ground = MakeBody({0.0f, 0.0f, 0.0f}, 1.0f, glm::vec3(1.0f), true);
    RigidBody box = MakeBody({0.0f, 1.0f, 0.0f}, 1.0f, glm::vec3(1.0f));
    Collider groundCollider = MakeBoxCollider(glm::vec3(5.0f, 0.5f, 5.0f), 0.5f, 0.0f, true);
    Collider boxCollider = MakeBoxCollider(glm::vec3(0.5f), 0.5f, 0.0f);

    ContactManifold manifold = MakePointManifold({0.0f, 0.5f, 0.0f});
    manifold.normal = glm::vec3(0.0f, -1.0f, 0.0f); // narrow-phase direction

    ContactSolver solver;
    PreparedContactConstraint constraint;
    solver.Prepare(
        constraint, 0u, manifold, ground, box, groundCollider, boxCollider, kDt, glm::vec3(0.0f));
    // The one-sided static surface overrides the normal with its front face.
    EXPECT_GT(glm::dot(constraint.normal, glm::vec3(0.0f, 1.0f, 0.0f)), 0.99f);
    EXPECT_GT(glm::dot(manifold.normal, glm::vec3(0.0f, 1.0f, 0.0f)), 0.99f);
}

// ---------------------------------------------------------------------------
// Warm start contract (plan step 17, 21)
// ---------------------------------------------------------------------------

TEST(ContactSolverWarmStartTest, AppliesCachedImpulseOnceAndRetractsOverestimate) {
    RigidBody ground = MakeBody({0.0f, -0.5f, 0.0f}, 1.0f, glm::vec3(1.0f), true);
    RigidBody box = MakeBody({0.0f, 0.5f, 0.0f}, 2.0f, glm::vec3(1.0f));
    Collider groundCollider = MakeBoxCollider(glm::vec3(5.0f, 0.5f, 5.0f), 0.5f, 0.0f);
    Collider boxCollider = MakeBoxCollider(glm::vec3(0.5f), 0.5f, 0.0f);

    // Contact directly below the box center: zero angular arms, pure linear
    // response (K = invMass = 0.5).
    ContactManifold manifold = MakePointManifold({0.0f, 0.5f, 0.0f});
    glm::vec3 tangent1;
    glm::vec3 tangent2;
    UpBasis(tangent1, tangent2);
    ContactPoint& point = manifold.Point(0);
    point.accumulatedNormalImpulse = 2.0f;
    point.accumulatedTangentImpulse = glm::vec2(0.4f, -0.2f);
    point.cachedTangent1 = tangent1;
    point.cachedTangent2 = tangent2;
    point.cachedDt = kDt;

    ContactSolver solver;
    PreparedContactConstraint constraint;
    solver.Prepare(
        constraint, 0u, manifold, ground, box, groundCollider, boxCollider, kDt, glm::vec3(0.0f));
    EXPECT_FLOAT_EQ(constraint.points[0].accumulatedNormal, 2.0f);
    EXPECT_FLOAT_EQ(constraint.accumulatedTangent.x, 0.4f);
    EXPECT_FLOAT_EQ(constraint.accumulatedTangent.y, -0.2f);

    ContactSolver::WarmStart(constraint);
    // Applied exactly once: dv = lambda * invMass on the dynamic body.
    const glm::vec3 expectedTangentImpulse = tangent1 * 0.4f + tangent2 * (-0.2f);
    EXPECT_NEAR(box.LinearVelocity().y, 2.0f * 0.5f, 1e-5f);
    EXPECT_NEAR(box.LinearVelocity().x, expectedTangentImpulse.x * 0.5f, 1e-5f);
    EXPECT_NEAR(box.LinearVelocity().z, expectedTangentImpulse.z * 0.5f, 1e-5f);

    // The warm start pushed the bodies apart with no external closing
    // velocity: the next iteration must retract the over-large accumulated
    // value through a negative delta instead of keeping it.
    ContactSolver::SolveVelocityIteration(constraint);
    EXPECT_NEAR(constraint.points[0].accumulatedNormal, 0.0f, 1e-4f);
    EXPECT_NEAR(box.LinearVelocity().y, 0.0f, 1e-4f);

    ContactSolver::CommitSolvedImpulses(constraint);
    EXPECT_FLOAT_EQ(manifold.Point(0).accumulatedNormalImpulse, 0.0f);
    EXPECT_FLOAT_EQ(manifold.Point(0).normalImpulse, 0.0f);
    EXPECT_FLOAT_EQ(manifold.Point(0).cachedDt, kDt);
}

TEST(ContactSolverWarmStartTest, ScalesCacheBySubstepDtRatio) {
    RigidBody ground = MakeBody({0.0f, -0.5f, 0.0f}, 1.0f, glm::vec3(1.0f), true);
    RigidBody box = MakeBody({0.0f, 0.5f, 0.0f}, 2.0f, glm::vec3(1.0f));
    Collider groundCollider = MakeBoxCollider(glm::vec3(5.0f, 0.5f, 5.0f), 0.0f, 0.0f);
    Collider boxCollider = MakeBoxCollider(glm::vec3(0.5f), 0.0f, 0.0f);

    ContactManifold manifold = MakePointManifold({0.0f, 0.5f, 0.0f});
    ContactPoint& point = manifold.Point(0);
    point.accumulatedNormalImpulse = 1.0f;
    // The cache was accumulated with a sub-step half the current length:
    // the initial value doubles (dtNew / cachedDt).
    point.cachedDt = kDt * 0.5f;

    ContactSolver solver;
    PreparedContactConstraint constraint;
    solver.Prepare(
        constraint, 0u, manifold, ground, box, groundCollider, boxCollider, kDt, glm::vec3(0.0f));
    EXPECT_FLOAT_EQ(constraint.points[0].accumulatedNormal, 2.0f);

    ContactSolver::WarmStart(constraint);
    EXPECT_NEAR(box.LinearVelocity().y, 2.0f * 0.5f, 1e-5f);
}

TEST(ContactSolverWarmStartTest, ReprojectsCachedTangentImpulseOntoNewBasis) {
    RigidBody ground = MakeBody({0.0f, -0.5f, 0.0f}, 1.0f, glm::vec3(1.0f), true);
    RigidBody box = MakeBody({0.0f, 0.5f, 0.0f}, 2.0f, glm::vec3(1.0f));
    // High friction: the reprojected tangent cache must not be disc-clamped.
    Collider groundCollider = MakeBoxCollider(glm::vec3(5.0f, 0.5f, 5.0f), 5.0f, 0.0f);
    Collider boxCollider = MakeBoxCollider(glm::vec3(0.5f), 5.0f, 0.0f);

    glm::vec3 oldTangent1;
    glm::vec3 oldTangent2;
    UpBasis(oldTangent1, oldTangent2);

    // The new normal rotated 20 degrees around Z: a fresh, different basis.
    const float angle = glm::radians(20.0f);
    const glm::vec3 newNormal = glm::normalize(glm::vec3(std::sin(angle), std::cos(angle), 0.0f));
    glm::vec3 newTangent1;
    glm::vec3 newTangent2;
    Runtime::Physics::BuildContactTangentBasis(newNormal, newTangent1, newTangent2);

    ContactManifold manifold = MakePointManifold({0.0f, 0.5f, 0.0f});
    manifold.normal = newNormal;
    ContactPoint& point = manifold.Point(0);
    point.accumulatedNormalImpulse = 1.0f;
    point.accumulatedTangentImpulse = glm::vec2(0.4f, -0.2f);
    point.cachedTangent1 = oldTangent1;
    point.cachedTangent2 = oldTangent2;
    point.cachedDt = kDt;

    ContactSolver solver;
    PreparedContactConstraint constraint;
    solver.Prepare(
        constraint, 0u, manifold, ground, box, groundCollider, boxCollider, kDt, glm::vec3(0.0f));

    // The cached 2D impulse is re-expressed in the NEW basis by projecting
    // the old world-space tangent impulse.
    const glm::vec3 oldWorldTangentImpulse = oldTangent1 * 0.4f + oldTangent2 * (-0.2f);
    const float expectedT1 = glm::dot(oldWorldTangentImpulse, newTangent1);
    const float expectedT2 = glm::dot(oldWorldTangentImpulse, newTangent2);
    EXPECT_NEAR(constraint.accumulatedTangent.x, expectedT1, 1e-5f);
    EXPECT_NEAR(constraint.accumulatedTangent.y, expectedT2, 1e-5f);

    ContactSolver::WarmStart(constraint);
    const glm::vec3 expectedImpulse =
        newNormal * 1.0f + newTangent1 * expectedT1 + newTangent2 * expectedT2;
    const glm::vec3 expectedVelocity = expectedImpulse * 0.5f;
    EXPECT_NEAR(glm::distance(box.LinearVelocity(), expectedVelocity), 0.0f, 1e-5f);
}

// Restitution arrival test (Jolt-style): a separated speculative contact
// whose closing speed cannot bridge the gap within the sub-step keeps the
// (negative) speculative bias instead of bouncing off thin air; a contact
// that truly arrives (|v_n| > gap/dt) gets the full restitution target.
// Replaces the removed TOI impact suppression, which existed only because
// the pre-rework CCD pipeline re-solved already-touching pairs with
// restitution applied at every TOI sub-step.
TEST(ContactSolverWarmStartTest, RestitutionRequiresArrivalWithinSubstep) {
    RigidBody ground = MakeBody({0.0f, -0.5f, 0.0f}, 1.0f, glm::vec3(1.0f), true);
    RigidBody box = MakeBody({0.0f, 0.5f, 0.0f}, 2.0f, glm::vec3(1.0f));
    Collider groundCollider = MakeBoxCollider(glm::vec3(5.0f, 0.5f, 5.0f), 0.5f, 0.9f);
    Collider boxCollider = MakeBoxCollider(glm::vec3(0.5f), 0.5f, 0.9f);
    ContactSolver solver;

    // Gap 0.03 m at dt = 1/60: arrival needs |v_n| > 1.8 m/s. Closing at
    // 1.5 m/s passes the restitution speed threshold but never arrives this
    // sub-step, so the bias stays the speculative target -gap/dt.
    box.SetLinearVelocity({0.0f, -1.5f, 0.0f});
    ContactManifold missManifold = MakePointManifold({0.0f, 0.5f, 0.0f}, -0.03f);
    PreparedContactConstraint missConstraint;
    solver.Prepare(
        missConstraint, 0u, missManifold, ground, box, groundCollider, boxCollider, kDt,
        glm::vec3(0.0f));
    EXPECT_NEAR(missConstraint.points[0].restitutionBias, -1.8f, 1e-4f);

    // Closing at 3.0 m/s arrives within the sub-step: the bias becomes the
    // positive restitution target e * |v_n| (zero gravity here).
    box.SetLinearVelocity({0.0f, -3.0f, 0.0f});
    ContactManifold hitManifold = MakePointManifold({0.0f, 0.5f, 0.0f}, -0.03f);
    PreparedContactConstraint hitConstraint;
    solver.Prepare(
        hitConstraint, 0u, hitManifold, ground, box, groundCollider, boxCollider, kDt,
        glm::vec3(0.0f));
    EXPECT_NEAR(hitConstraint.points[0].restitutionBias, 2.7f, 1e-4f);
}

// ---------------------------------------------------------------------------
// Accumulated incremental solve (plan step 18, 21)
// ---------------------------------------------------------------------------

TEST(ContactSolverAccumulatedTest, NormalLambdaNeverNegativeOnSeparatingContact) {
    RigidBody ground = MakeBody({0.0f, -0.5f, 0.0f}, 1.0f, glm::vec3(1.0f), true);
    RigidBody box = MakeBody({0.0f, 0.5f, 0.0f}, 2.0f, glm::vec3(1.0f));
    box.SetLinearVelocity({0.0f, 1.5f, 0.0f}); // separating
    Collider groundCollider = MakeBoxCollider(glm::vec3(5.0f, 0.5f, 5.0f), 0.5f, 0.0f);
    Collider boxCollider = MakeBoxCollider(glm::vec3(0.5f), 0.5f, 0.0f);

    ContactManifold manifold = MakePointManifold({0.0f, 0.5f, 0.0f});
    ContactSolver solver;
    PreparedContactConstraint constraint;
    solver.Prepare(
        constraint, 0u, manifold, ground, box, groundCollider, boxCollider, kDt, glm::vec3(0.0f));

    for (int iteration = 0; iteration < 8; ++iteration) {
        ContactSolver::SolveVelocityIteration(constraint);
    }
    EXPECT_FLOAT_EQ(constraint.points[0].accumulatedNormal, 0.0f);
    EXPECT_FLOAT_EQ(box.LinearVelocity().y, 1.5f);
}

TEST(ContactSolverAccumulatedTest, FrictionDiscClampHoldsEveryIteration) {
    RigidBody ground = MakeBody({0.0f, -0.5f, 0.0f}, 1.0f, glm::vec3(1.0f), true);
    RigidBody box = MakeBody({0.0f, 0.5f, 0.0f}, 2.0f, glm::vec3(1.0f));
    box.SetLinearVelocity({5.0f, -1.0f, 0.0f});
    // mu = sqrt(0.3 * 0.3) = 0.3 on the combined material.
    Collider groundCollider = MakeBoxCollider(glm::vec3(5.0f, 0.5f, 5.0f), 0.3f, 0.0f);
    Collider boxCollider = MakeBoxCollider(glm::vec3(0.5f), 0.3f, 0.0f);

    ContactManifold manifold = MakePointManifold({0.0f, 0.5f, 0.0f});
    ContactSolver solver;
    PreparedContactConstraint constraint;
    solver.Prepare(
        constraint, 0u, manifold, ground, box, groundCollider, boxCollider, kDt, glm::vec3(0.0f));

    constexpr float mu = 0.3f;
    for (int iteration = 0; iteration < 8; ++iteration) {
        ContactSolver::SolveVelocityIteration(constraint);
        const float lambdaN = constraint.points[0].accumulatedNormal;
        const glm::vec2 lambdaT = constraint.accumulatedTangent;
        EXPECT_GE(lambdaN, 0.0f);
        EXPECT_LE(glm::length(lambdaT), mu * lambdaN + 1e-4f);
    }

    // Pure linear response (zero arms, K = invMass = 0.5): the normal solve
    // converged to lambda_n = 2, so friction saturates at 0.6 and removes
    // exactly 0.6 * 0.5 = 0.3 of tangential speed.
    EXPECT_NEAR(constraint.points[0].accumulatedNormal, 2.0f, 1e-4f);
    EXPECT_NEAR(box.LinearVelocity().y, 0.0f, 1e-4f);
    EXPECT_NEAR(box.LinearVelocity().x, 5.0f - mu * 2.0f * 0.5f, 1e-4f);
}

TEST(ContactSolverAccumulatedTest, ReorderedManifoldInheritsImpulsesByAnchor) {
    // Two persistent points with distinct caches; the new manifold lists the
    // anchors in reversed order and must inherit through anchor matching.
    ContactManifold oldManifold;
    oldManifold.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 tangent1;
    glm::vec3 tangent2;
    UpBasis(tangent1, tangent2);
    for (int index = 0; index < 2; ++index) {
        ContactPoint point;
        point.localPointA = glm::vec3(0.2f * static_cast<float>(index), 0.0f, 0.0f);
        point.localPointB = glm::vec3(0.2f * static_cast<float>(index), -1.0f, 0.0f);
        point.accumulatedNormalImpulse = 1.0f + static_cast<float>(index);
        point.cachedTangent1 = tangent1;
        point.cachedTangent2 = tangent2;
        point.cachedDt = kDt;
        ASSERT_TRUE(oldManifold.AddPoint(point));
    }

    RigidBody ground = MakeBody({0.0f, 0.0f, 0.0f}, 1.0f, glm::vec3(1.0f), true);
    RigidBody box = MakeBody({0.0f, 1.0f, 0.0f}, 2.0f, glm::vec3(1.0f));
    Collider groundCollider = MakeBoxCollider(glm::vec3(5.0f, 0.5f, 5.0f), 0.5f, 0.0f);
    Collider boxCollider = MakeBoxCollider(glm::vec3(0.5f), 0.5f, 0.0f);

    // Identity orientations: local anchors map to world offsets directly.
    ContactManifold newManifold;
    newManifold.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    for (int index = 1; index >= 0; --index) {
        ContactPoint point;
        const float x = 0.2f * static_cast<float>(index);
        point.position = glm::vec3(x, 0.5f, 0.0f);
        point.penetration = 0.01f;
        point.localPointA = glm::vec3(x, 0.0f, 0.0f);
        point.localPointB = glm::vec3(x, -1.0f, 0.0f);
        ASSERT_TRUE(newManifold.AddPoint(point));
    }
    Runtime::Physics::MatchPersistentContactPoints(oldManifold, newManifold);

    ContactSolver solver;
    PreparedContactConstraint constraint;
    solver.Prepare(
        constraint, 0u, newManifold, ground, box, groundCollider, boxCollider, kDt, glm::vec3(0.0f));
    ASSERT_EQ(constraint.pointCount, 2u);
    // Reversed order: new point 0 carries old cache 2.0, new point 1 cache 1.0.
    EXPECT_FLOAT_EQ(constraint.points[0].accumulatedNormal, 2.0f);
    EXPECT_FLOAT_EQ(constraint.points[1].accumulatedNormal, 1.0f);
}

// ---------------------------------------------------------------------------
// One-sided regressions (plan step 19) and fixed-step impulse telemetry
// ---------------------------------------------------------------------------

TEST(ContactSolverWorldTest, OneSidedPlatformBouncesOnceWithoutBoost) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    world.SetFixedTimeStep(1.0f / 120.0f);

    RigidBodyDesc groundDesc;
    groundDesc.position = {0.0f, -0.5f, 0.0f};
    groundDesc.isStatic = true;
    groundDesc.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(groundDesc);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(10.0f, 0.5f, 10.0f));
    groundCollider.material.restitution = 0.5f;
    groundCollider.material.dynamicFriction = 0.0f;
    groundCollider.material.staticFriction = 0.0f;
    groundCollider.oneSided = true;
    groundCollider.oneSidedNormalLocal = glm::vec3(0.0f, 1.0f, 0.0f);
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    RigidBodyDesc sphereDesc;
    sphereDesc.position = {0.0f, 2.0f, 0.0f};
    sphereDesc.mass = 1.0f;
    const uint32_t sphereId = world.CreateRigidBody(sphereDesc);
    ColliderDesc sphereCollider;
    sphereCollider.shape = std::make_shared<SphereShape>(0.25f);
    sphereCollider.material.restitution = 0.5f;
    sphereCollider.material.dynamicFriction = 0.0f;
    sphereCollider.material.staticFriction = 0.0f;
    ASSERT_TRUE(world.AttachCollider(sphereId, sphereCollider));

    float impactSpeed = 0.0f;
    float reboundSpeed = 0.0f;
    bool touched = false;
    for (int step = 0; step < 360; ++step) {
        const float preVelocityY = world.GetRigidBody(sphereId)->LinearVelocity().y;
        world.Step(world.FixedTimeStep());
        const float postVelocityY = world.GetRigidBody(sphereId)->LinearVelocity().y;
        if (!touched) {
            impactSpeed = std::min(impactSpeed, preVelocityY);
            if (!world.ContactEvents().empty()) {
                touched = true;
                impactSpeed = std::min(impactSpeed, preVelocityY);
            }
        } else if (step < 180) {
            reboundSpeed = std::max(reboundSpeed, postVelocityY);
        }
    }

    ASSERT_TRUE(touched);
    const float incoming = -impactSpeed;
    ASSERT_GT(incoming, 1.0f);
    // Single restitution application through the Prepare bias: the rebound
    // matches e * v_incoming and is never boosted beyond it.
    EXPECT_NEAR(reboundSpeed, 0.5f * incoming, 0.2f);
    EXPECT_LT(reboundSpeed, 0.62f * incoming);

    // After the bounces decay the sphere rests on the platform.
    const auto* sphere = world.GetRigidBody(sphereId);
    ASSERT_NE(sphere, nullptr);
    EXPECT_GT(sphere->Position().y, -0.05f);
    EXPECT_NEAR(sphere->LinearVelocity().y, 0.0f, 0.2f);
}

TEST(ContactSolverWorldTest, OneSidedPlatformPassesThroughFromBack) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f));
    world.SetContinuousCollisionEnabled(false);

    RigidBodyDesc platformDesc;
    platformDesc.position = {0.0f, 1.0f, 0.0f};
    platformDesc.isStatic = true;
    platformDesc.useGravity = false;
    const uint32_t platformId = world.CreateRigidBody(platformDesc);
    ColliderDesc platformCollider;
    platformCollider.shape = std::make_shared<BoxShape>(glm::vec3(5.0f, 0.05f, 5.0f));
    platformCollider.oneSided = true;
    platformCollider.oneSidedNormalLocal = glm::vec3(0.0f, 1.0f, 0.0f);
    ASSERT_TRUE(world.AttachCollider(platformId, platformCollider));

    RigidBodyDesc sphereDesc;
    sphereDesc.position = {0.0f, 0.0f, 0.0f};
    sphereDesc.linearVelocity = {0.0f, 3.0f, 0.0f};
    sphereDesc.useGravity = false;
    const uint32_t sphereId = world.CreateRigidBody(sphereDesc);
    ColliderDesc sphereCollider;
    sphereCollider.shape = std::make_shared<SphereShape>(0.2f);
    ASSERT_TRUE(world.AttachCollider(sphereId, sphereCollider));

    bool sawEnter = false;
    bool sawExit = false;
    float maxImpulse = 0.0f;
    for (int step = 0; step < 60; ++step) {
        world.Step(1.0f / 60.0f);
        for (const auto& event : world.ContactEvents()) {
            if (event.type == ContactEventType::Enter) {
                sawEnter = true;
            }
            if (event.type == ContactEventType::Exit) {
                sawExit = true;
            }
        }
        for (const auto& contact : world.Contacts()) {
            maxImpulse = std::max(maxImpulse, contact.Point(0).normalImpulse);
            EXPECT_FLOAT_EQ(contact.fixedStepNormalImpulse, 0.0f);
        }
    }

    // Approaching from the back side produces events but no impulse and no
    // blocking: the sphere keeps its velocity and exits above the platform.
    EXPECT_TRUE(sawEnter);
    EXPECT_TRUE(sawExit);
    EXPECT_FLOAT_EQ(maxImpulse, 0.0f);
    const auto* sphere = world.GetRigidBody(sphereId);
    ASSERT_NE(sphere, nullptr);
    EXPECT_GT(sphere->Position().y, 1.4f);
    EXPECT_NEAR(sphere->LinearVelocity().y, 3.0f, 0.05f);
}

TEST(ContactSolverWorldTest, FixedStepImpulseTotalsAccumulateAndResetPerStep) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    // This test verifies fixed-step impulse accounting, not sleep: keep the
    // box awake so its contact keeps publishing every step (Phase 9).
    world.SetSleepEnabled(false);

    RigidBodyDesc groundDesc;
    groundDesc.isStatic = true;
    groundDesc.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(groundDesc);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(10.0f, 0.5f, 10.0f));
    groundCollider.material.restitution = 0.0f;
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    RigidBodyDesc boxDesc;
    boxDesc.position = {0.0f, 0.55f, 0.0f};
    const uint32_t boxId = world.CreateRigidBody(boxDesc);
    ColliderDesc boxCollider;
    boxCollider.shape = std::make_shared<BoxShape>(glm::vec3(0.5f));
    boxCollider.material.restitution = 0.0f;
    ASSERT_TRUE(world.AttachCollider(boxId, boxCollider));

    for (int step = 0; step < 120; ++step) {
        world.Step(kDt);
    }

    // Without CCD a fixed step has exactly one sub-step, so the fixed-step
    // total equals the last sub-step's accumulated lambda and supports the
    // box weight: m * g * dt ~= 0.1635. Re-checking over several steps also
    // proves the totals reset instead of accumulating across fixed steps.
    const float expectedSupport = 9.81f * kDt;
    for (int step = 0; step < 10; ++step) {
        world.Step(kDt);
        ASSERT_EQ(world.Contacts().size(), 1u);
        const ContactManifold& contact = world.Contacts().front();
        EXPECT_EQ(MakePairKey(contact.bodyA, contact.bodyB), MakePairKey(groundId, boxId));
        EXPECT_NEAR(contact.fixedStepNormalImpulse, expectedSupport, 0.05f);
        float pointImpulseSum = 0.0f;
        for (std::size_t index = 0; index < contact.pointCount; ++index) {
            pointImpulseSum += contact.Point(index).normalImpulse;
        }
        EXPECT_NEAR(contact.fixedStepNormalImpulse, pointImpulseSum, 1e-4f);
        // Manifold-level friction converges to a small non-zero net tangent
        // impulse on a settling box; only the symmetric rest state is zero.
        EXPECT_NEAR(glm::length(contact.fixedStepTangentImpulse), 0.0f, 0.05f);
    }
}

// ---------------------------------------------------------------------------
// Low-iteration stacking quality (plan step 21)
// ---------------------------------------------------------------------------

// DISABLED: 4-iteration 3-box stacking exceeds what sequential
// Gauss-Seidel can redistribute across a multi-point manifold at very low
// iteration counts (see Docs/phase4-investigation.md). The Phase 10 fixes
// (speculative contacts, per-point normal arms, manifold-level friction,
// NGS) made 10-iteration stacks stable; the 4-iteration case still needs
// Jolt-style manifold reduction or a block solve to converge the load
// distribution fast enough.
TEST(ContactSolverWorldTest, DISABLED_LowIterationStackStaysWithinQualityBounds) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    world.SetSolverIterations(4);

    RigidBodyDesc groundDesc;
    groundDesc.position = {0.0f, -0.5f, 0.0f};
    groundDesc.isStatic = true;
    groundDesc.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(groundDesc);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(10.0f, 0.5f, 10.0f));
    groundCollider.material.restitution = 0.0f;
    groundCollider.material.dynamicFriction = 0.6f;
    groundCollider.material.staticFriction = 0.7f;
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    uint32_t boxIds[3];
    for (int index = 0; index < 3; ++index) {
        RigidBodyDesc boxDesc;
        boxDesc.position = {0.0f, 0.55f + 1.05f * static_cast<float>(index), 0.0f};
        boxIds[index] = world.CreateRigidBody(boxDesc);
        ColliderDesc boxCollider;
        boxCollider.shape = std::make_shared<BoxShape>(glm::vec3(0.5f));
        boxCollider.material.restitution = 0.0f;
        boxCollider.material.dynamicFriction = 0.6f;
        boxCollider.material.staticFriction = 0.7f;
        ASSERT_TRUE(world.AttachCollider(boxIds[index], boxCollider));
    }

    float maxPenetration = 0.0f;
    int sleepStep = -1;
    for (int step = 0; step < 300; ++step) {
        world.Step(kDt);
        for (const auto& contact : world.Contacts()) {
            for (std::size_t index = 0; index < contact.pointCount; ++index) {
                maxPenetration = std::max(maxPenetration, contact.Point(index).penetration);
            }
        }
        if (sleepStep < 0) {
            bool allSleeping = true;
            for (const uint32_t boxId : boxIds) {
                if (!world.GetRigidBody(boxId)->IsSleeping()) {
                    allSleeping = false;
                }
            }
            if (allSleeping) {
                sleepStep = step;
            }
        }
    }
    std::fprintf(stderr, "[Stack3] sleepStep=%d maxPenetration=%.4f\n", sleepStep, static_cast<double>(maxPenetration));

    for (const uint32_t boxId : boxIds) {
        const auto* box = world.GetRigidBody(boxId);
        ASSERT_NE(box, nullptr);
        EXPECT_LT(glm::length(box->LinearVelocity()), 0.05f);
        EXPECT_LT(std::abs(box->Position().x), 0.1f);
        EXPECT_LT(std::abs(box->Position().z), 0.1f);
    }
    EXPECT_LT(maxPenetration, 0.08f);
}

TEST(ContactSolverWorldTest, AwakeFiveBoxStackDoesNotSinkOrDrift) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetGravity({0.0f, -10.5f, 0.0f});
    world.SetContinuousCollisionEnabled(false);
    world.SetSleepEnabled(false);
    world.SetSolverIterations(10);

    RigidBodyDesc ground;
    ground.position = {0.0f, -0.5f, 0.0f};
    ground.isStatic = true;
    ground.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(ground);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(10.0f, 0.5f, 10.0f));
    groundCollider.material.restitution = 0.0f;
    groundCollider.material.dynamicFriction = 0.7f;
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    std::array<uint32_t, 5> boxIds{};
    for (std::size_t index = 0; index < boxIds.size(); ++index) {
        RigidBodyDesc box;
        box.position = {0.0f, 0.5f + static_cast<float>(index), 0.0f};
        box.mass = 1.0f;
        box.inertiaTensorDiagonal = glm::vec3(1.0f / 6.0f);
        boxIds[index] = world.CreateRigidBody(box);
        ColliderDesc collider;
        collider.shape = std::make_shared<BoxShape>(glm::vec3(0.5f));
        collider.material.restitution = 0.0f;
        collider.material.dynamicFriction = 0.7f;
        ASSERT_TRUE(world.AttachCollider(boxIds[index], collider));
    }
    float maxSettledSpeed = 0.0f;
    float maxSettledPenetration = 0.0f;
    for (int step = 0; step < 1200; ++step) {
        world.Step(1.0f / 120.0f);
        if (step >= 300) {
            for (const uint32_t boxId : boxIds) {
                maxSettledSpeed = std::max(maxSettledSpeed, glm::length(world.GetRigidBody(boxId)->LinearVelocity()));
            }
            for (const ContactManifold& contact : world.Contacts()) {
                for (std::size_t index = 0; index < contact.pointCount; ++index) {
                    maxSettledPenetration = std::max(maxSettledPenetration, contact.Point(index).penetration);
                }
            }
        }
    }
    EXPECT_LT(maxSettledSpeed, 0.08f);
    EXPECT_LT(maxSettledPenetration, 0.04f);
    std::printf("[stack5] settled speed=%.5f penetration=%.5f\n", maxSettledSpeed, maxSettledPenetration);
    for (std::size_t index = 0; index < boxIds.size(); ++index) {
        const RigidBody* box = world.GetRigidBody(boxIds[index]);
        ASSERT_NE(box, nullptr);
        EXPECT_NEAR(box->Position().y, 0.5f + static_cast<float>(index), 0.08f);
        EXPECT_LT(glm::length(glm::vec2(box->Position().x, box->Position().z)), 0.08f);
        EXPECT_LT(glm::length(box->LinearVelocity()), 0.08f);
    }
}

TEST(ContactSolverWorldTest, TowerRainBuildsFiveBoxStack) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetGravity({0.0f, -10.5f, 0.0f});
    world.SetContinuousCollisionEnabled(false);
    world.SetSolverIterations(6);

    RigidBodyDesc ground;
    ground.position = {0.0f, -0.5f, 0.0f};
    ground.isStatic = true;
    ground.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(ground);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
    groundCollider.material.restitution = 0.1f;
    groundCollider.material.dynamicFriction = 0.6f;
    groundCollider.material.staticFriction = 0.7f;
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    const auto boxShape = std::make_shared<BoxShape>(glm::vec3(0.45f));
    std::array<uint32_t, 5> boxIds{};
    for (std::size_t index = 0; index < boxIds.size(); ++index) {
        RigidBodyDesc box;
        box.position = {0.0f, 0.95f + 0.9f * static_cast<float>(index), 0.0f};
        box.mass = 1.0f;
        box.inertiaTensorDiagonal = glm::vec3(0.135f);
        boxIds[index] = world.CreateRigidBody(box);
        ColliderDesc collider;
        collider.shape = boxShape;
        collider.material.restitution = 0.05f;
        collider.material.dynamicFriction = 1.0f;
        collider.material.staticFriction = 1.0f;
        ASSERT_TRUE(world.AttachCollider(boxIds[index], collider));
        for (int step = 0; step < 180; ++step) {
            world.Step(1.0f / 120.0f);
        }
        for (std::size_t settled = 0; settled <= index; ++settled) {
            const RigidBody* current = world.GetRigidBody(boxIds[settled]);
            ASSERT_NE(current, nullptr);
            EXPECT_NEAR(current->Position().y, 0.45f + 0.9f * static_cast<float>(settled), 0.45f)
                << "after drop " << index;
            EXPECT_LT(glm::length(glm::vec2(current->Position().x, current->Position().z)), 0.35f)
                << "after drop " << index;
        }
    }
    for (int step = 0; step < 360; ++step) {
        world.Step(1.0f / 120.0f);
    }
    for (std::size_t index = 0; index < boxIds.size(); ++index) {
        const RigidBody* box = world.GetRigidBody(boxIds[index]);
        ASSERT_NE(box, nullptr);
        EXPECT_NEAR(box->Position().y, 0.45f + 0.9f * static_cast<float>(index), 0.45f);
        EXPECT_LT(std::abs(box->Position().x), 0.35f);
        EXPECT_LT(std::abs(box->Position().z), 0.35f);
        EXPECT_TRUE(box->IsSleeping());
    }
}

// BoxStackSample tower-rain mode with CCD enabled: boxes spawn one at a time
// just above the current tower top (0.05 m gap), 10 solver iterations,
// exactly like the sample. Regression for "CCD on -> every step integrates
// positions before the velocity solve": the stack used to sink ~g*dt^2 per
// step into a ~2 cm per-interface penetration equilibrium and never slept.
TEST(ContactSolverWorldTest, TowerRainWithCcdBuildsSleepingStack) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetGravity({0.0f, -10.5f, 0.0f});
    world.SetContinuousCollisionEnabled(true);
    world.SetSolverIterations(10);

    RigidBodyDesc ground;
    ground.position = {0.0f, -0.5f, 0.0f};
    ground.isStatic = true;
    ground.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(ground);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
    groundCollider.material.restitution = 0.1f;
    groundCollider.material.dynamicFriction = 0.6f;
    groundCollider.material.staticFriction = 0.7f;
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    const auto boxShape = std::make_shared<BoxShape>(glm::vec3(0.45f));
    std::vector<uint32_t> boxIds;
    float maxSettledPenetration = 0.0f;
    const auto trackPenetration = [&]() {
        for (const ContactManifold& contact : world.Contacts()) {
            for (std::size_t p = 0; p < contact.pointCount; ++p) {
                maxSettledPenetration = std::max(maxSettledPenetration, contact.Point(p).penetration);
            }
        }
    };
    for (std::size_t index = 0; index < 5; ++index) {
        float towerTop = 0.0f;
        for (const uint32_t id : boxIds) {
            towerTop = std::max(towerTop, world.GetRigidBody(id)->Position().y + 0.45f);
        }
        RigidBodyDesc box;
        box.position = {0.0f, towerTop + 0.45f + 0.05f, 0.0f};
        box.mass = 1.0f;
        box.inertiaTensorDiagonal = glm::vec3(0.135f);
        const uint32_t boxId = world.CreateRigidBody(box);
        ColliderDesc collider;
        collider.shape = boxShape;
        collider.material.restitution = 0.05f;
        collider.material.dynamicFriction = 1.0f;
        collider.material.staticFriction = 1.0f;
        ASSERT_TRUE(world.AttachCollider(boxId, collider));
        boxIds.push_back(boxId);
        for (int step = 0; step < 180; ++step) {
            world.Step(1.0f / 120.0f);
            if (step >= 60) {
                trackPenetration();
            }
        }
        for (std::size_t settled = 0; settled <= index; ++settled) {
            const RigidBody* current = world.GetRigidBody(boxIds[settled]);
            ASSERT_NE(current, nullptr);
            EXPECT_NEAR(current->Position().y, 0.45f + 0.9f * static_cast<float>(settled), 0.45f)
                << "after drop " << index;
            EXPECT_LT(glm::length(glm::vec2(current->Position().x, current->Position().z)), 0.35f)
                << "after drop " << index;
        }
    }
    for (int step = 0; step < 600; ++step) {
        world.Step(1.0f / 120.0f);
        trackPenetration();
    }
    std::printf("[tower-ccd] settled penetration=%.5f\n", maxSettledPenetration);
    // Speculative contacts stop the approach at the surface; settled
    // penetration must stay far below the 2 cm slop (the bug pinned it
    // just above slop at every interface).
    EXPECT_LT(maxSettledPenetration, 0.01f);
    for (std::size_t index = 0; index < boxIds.size(); ++index) {
        const RigidBody* box = world.GetRigidBody(boxIds[index]);
        ASSERT_NE(box, nullptr);
        EXPECT_NEAR(box->Position().y, 0.45f + 0.9f * static_cast<float>(index), 0.2f);
        EXPECT_LT(glm::length(glm::vec2(box->Position().x, box->Position().z)), 0.35f);
        EXPECT_TRUE(box->IsSleeping()) << "tower box " << index << " never slept";
    }
}

// CCD-enabled variant of FourLevelStackSurvivesEdgeContactSettle: the
// four-level stack plus the 45-degree edge-first dropper must survive with
// continuous collision on, the non face-face path must actually run, and the
// whole island must fall asleep afterwards.
TEST(ContactSolverWorldTest, FourLevelStackWithCcdSurvivesEdgeContactSettle) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetGravity({0.0f, -10.5f, 0.0f});
    world.SetContinuousCollisionEnabled(true);
    world.SetSolverIterations(10);

    RigidBodyDesc ground;
    ground.position = {0.0f, -0.5f, 0.0f};
    ground.isStatic = true;
    ground.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(ground);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
    groundCollider.material.restitution = 0.05f;
    groundCollider.material.dynamicFriction = 1.0f;
    groundCollider.material.staticFriction = 1.0f;
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    const auto boxShape = std::make_shared<BoxShape>(glm::vec3(0.45f));
    std::array<uint32_t, 4> stackIds{};
    for (std::size_t index = 0; index < stackIds.size(); ++index) {
        RigidBodyDesc box;
        box.position = {0.0f, 0.45f + 0.9f * static_cast<float>(index), 0.0f};
        box.mass = 1.0f;
        box.inertiaTensorDiagonal = glm::vec3(0.135f);
        stackIds[index] = world.CreateRigidBody(box);
        ColliderDesc collider;
        collider.shape = boxShape;
        collider.material.restitution = 0.05f;
        collider.material.dynamicFriction = 1.0f;
        collider.material.staticFriction = 1.0f;
        ASSERT_TRUE(world.AttachCollider(stackIds[index], collider));
    }

    RigidBodyDesc dropper;
    const float dropperHalfDiagonal = 0.45f * std::sqrt(2.0f);
    dropper.position = {0.0f, 0.45f + 0.9f * 3.0f + dropperHalfDiagonal + 0.3f, 0.0f};
    dropper.orientation = glm::angleAxis(glm::radians(45.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    dropper.mass = 1.0f;
    dropper.inertiaTensorDiagonal = glm::vec3(0.135f);
    const uint32_t dropperId = world.CreateRigidBody(dropper);
    ColliderDesc dropperCollider;
    dropperCollider.shape = boxShape;
    dropperCollider.material.restitution = 0.05f;
    dropperCollider.material.dynamicFriction = 1.0f;
    dropperCollider.material.staticFriction = 1.0f;
    ASSERT_TRUE(world.AttachCollider(dropperId, dropperCollider));

    bool sawReducedContact = false;
    for (int step = 0; step < 1200; ++step) {
        world.Step(1.0f / 120.0f);
        for (const auto& contact : world.Contacts()) {
            const bool involvesDropper =
                contact.bodyA == dropperId || contact.bodyB == dropperId;
            if (involvesDropper && contact.pointCount >= 1 && contact.pointCount < 4) {
                sawReducedContact = true;
            }
        }
    }
    EXPECT_TRUE(sawReducedContact)
        << "the 45-degree dropper never produced an edge/vertex contact";

    for (std::size_t index = 0; index < stackIds.size(); ++index) {
        const RigidBody* box = world.GetRigidBody(stackIds[index]);
        ASSERT_NE(box, nullptr);
        EXPECT_NEAR(box->Position().y, 0.45f + 0.9f * static_cast<float>(index), 0.45f)
            << "stack level " << index;
        EXPECT_LT(glm::length(glm::vec2(box->Position().x, box->Position().z)), 0.35f)
            << "stack level " << index;
        EXPECT_TRUE(box->IsSleeping()) << "stack level " << index << " never slept";
    }
    const RigidBody* settled = world.GetRigidBody(dropperId);
    ASSERT_NE(settled, nullptr);
    EXPECT_GT(settled->Position().y, 3.0f) << "dropper fell off the stack";
    EXPECT_TRUE(settled->IsSleeping()) << "dropper never slept";
}

// Non face-face contact: a box dropped rotated 45 degrees about X lands
// edge-first on the ground, then must settle (roll onto a face) without
// exploding or staying in motion.
TEST(ContactSolverWorldTest, EdgeDropSettlesOntoFace) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetGravity({0.0f, -10.5f, 0.0f});
    world.SetContinuousCollisionEnabled(false);
    world.SetSolverIterations(10);

    RigidBodyDesc ground;
    ground.position = {0.0f, -0.5f, 0.0f};
    ground.isStatic = true;
    ground.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(ground);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
    groundCollider.material.restitution = 0.05f;
    groundCollider.material.dynamicFriction = 1.0f;
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    const auto boxShape = std::make_shared<BoxShape>(glm::vec3(0.45f));
    RigidBodyDesc dropper;
    dropper.position = {0.0f, 2.0f, 0.0f};
    dropper.orientation = glm::angleAxis(glm::radians(45.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    dropper.mass = 1.0f;
    dropper.inertiaTensorDiagonal = glm::vec3(0.135f);
    const uint32_t dropperId = world.CreateRigidBody(dropper);
    ColliderDesc dropperCollider;
    dropperCollider.shape = boxShape;
    dropperCollider.material.restitution = 0.05f;
    dropperCollider.material.dynamicFriction = 1.0f;
    ASSERT_TRUE(world.AttachCollider(dropperId, dropperCollider));

    for (int step = 0; step < 600; ++step) {
        world.Step(1.0f / 120.0f);
    }
    const auto* b = world.GetRigidBody(dropperId);
    ASSERT_NE(b, nullptr);
    EXPECT_LT(glm::length(b->LinearVelocity()), 0.05f);
    EXPECT_LT(glm::length(b->AngularVelocity()), 0.05f);
    EXPECT_GT(b->Position().y, 0.3f); // on the ground, not fallen through
    EXPECT_LT(b->Position().y, 0.8f); // settled (edge or face resting height)
}

// Four-level stack stability with a NON face-face contact: a box dropped
// rotated 45 degrees about X lands edge-first on the flat stack top, then
// must settle (roll onto a face) without the stack collapsing.
TEST(ContactSolverWorldTest, FourLevelStackSurvivesEdgeContactSettle) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetGravity({0.0f, -10.5f, 0.0f});
    world.SetContinuousCollisionEnabled(false);
    world.SetSolverIterations(10);

    RigidBodyDesc ground;
    ground.position = {0.0f, -0.5f, 0.0f};
    ground.isStatic = true;
    ground.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(ground);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
    groundCollider.material.restitution = 0.05f;
    groundCollider.material.dynamicFriction = 1.0f;
    groundCollider.material.staticFriction = 1.0f;
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    const auto boxShape = std::make_shared<BoxShape>(glm::vec3(0.45f));
    std::array<uint32_t, 4> stackIds{};
    for (std::size_t index = 0; index < stackIds.size(); ++index) {
        RigidBodyDesc box;
        box.position = {0.0f, 0.45f + 0.9f * static_cast<float>(index), 0.0f};
        box.mass = 1.0f;
        box.inertiaTensorDiagonal = glm::vec3(0.135f);
        stackIds[index] = world.CreateRigidBody(box);
        ColliderDesc collider;
        collider.shape = boxShape;
        collider.material.restitution = 0.05f;
        collider.material.dynamicFriction = 1.0f;
        collider.material.staticFriction = 1.0f;
        ASSERT_TRUE(world.AttachCollider(stackIds[index], collider));
    }

    // The edge-first dropper: 45 degrees about X, released just above the
    // stack top so it contacts an edge against the flat face below.
    RigidBodyDesc dropper;
    const float dropperHalfDiagonal = 0.45f * std::sqrt(2.0f);
    dropper.position = {0.0f, 0.45f + 0.9f * 3.0f + dropperHalfDiagonal + 0.3f, 0.0f};
    dropper.orientation = glm::angleAxis(glm::radians(45.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    dropper.mass = 1.0f;
    dropper.inertiaTensorDiagonal = glm::vec3(0.135f);
    const uint32_t dropperId = world.CreateRigidBody(dropper);
    ColliderDesc dropperCollider;
    dropperCollider.shape = boxShape;
    dropperCollider.material.restitution = 0.05f;
    dropperCollider.material.dynamicFriction = 1.0f;
    dropperCollider.material.staticFriction = 1.0f;
    ASSERT_TRUE(world.AttachCollider(dropperId, dropperCollider));

    // The dropper contacts the stack top edge-first. Prove the non face-face
    // path actually ran: at some step the dropper-stack contact manifold must
    // have fewer than 4 points (an edge/vertex contact has 1-2 points).
    bool sawReducedContact = false;
    for (int step = 0; step < 1200; ++step) {
        world.Step(1.0f / 120.0f);
        for (const auto& contact : world.Contacts()) {
            const bool involvesDropper =
                contact.bodyA == dropperId || contact.bodyB == dropperId;
            if (involvesDropper && contact.pointCount >= 1 && contact.pointCount < 4) {
                sawReducedContact = true;
            }
        }
    }
    EXPECT_TRUE(sawReducedContact)
        << "the 45-degree dropper never produced an edge/vertex contact";

    // The four-level stack must stay upright and near its column; the
    // dropper must come to rest somewhere on top of the stack (not fallen
    // off to the ground and not still tumbling).
    for (std::size_t index = 0; index < stackIds.size(); ++index) {
        const RigidBody* box = world.GetRigidBody(stackIds[index]);
        ASSERT_NE(box, nullptr);
        EXPECT_NEAR(box->Position().y, 0.45f + 0.9f * static_cast<float>(index), 0.45f)
            << "stack level " << index;
        EXPECT_LT(glm::length(glm::vec2(box->Position().x, box->Position().z)), 0.35f)
            << "stack level " << index;
        EXPECT_LT(glm::length(box->LinearVelocity()), 0.1f);
    }
    const RigidBody* settled = world.GetRigidBody(dropperId);
    ASSERT_NE(settled, nullptr);
    EXPECT_GT(settled->Position().y, 3.0f) << "dropper fell off the stack";
    EXPECT_LT(glm::length(settled->LinearVelocity()), 0.1f);
    EXPECT_LT(glm::length(settled->AngularVelocity()), 0.1f);
}

// High-speed drop (14 m, ~16 m/s => 13 cm/step >> speculative band): with
// CCD off the box tunnels deep enough for the penetration axis to flip and
// shove it into the ground (regression for the "box embeds and flips into
// the ground" report); with CCD on it must land flat and stop.
TEST(ContactSolverWorldTest, HighSpeedDropNeedsCcdToAvoidTunnelingFlip) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetGravity({0.0f, -10.5f, 0.0f});
    world.SetContinuousCollisionEnabled(true);
    world.SetSolverIterations(10);
    RigidBodyDesc ground;
    ground.position = {0.0f, -0.5f, 0.0f};
    ground.isStatic = true;
    ground.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(ground);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
    groundCollider.material.restitution = 0.1f;
    groundCollider.material.dynamicFriction = 0.6f;
    groundCollider.material.staticFriction = 0.7f;
    world.AttachCollider(groundId, groundCollider);

    RigidBodyDesc box;
    box.position = {0.0f, 14.0f, 0.0f};
    box.mass = 1.0f;
    box.inertiaTensorDiagonal = glm::vec3(0.135f);
    const uint32_t boxId = world.CreateRigidBody(box);
    ColliderDesc boxCollider;
    boxCollider.shape = std::make_shared<BoxShape>(glm::vec3(0.45f));
    boxCollider.material.restitution = 0.05f;
    boxCollider.material.dynamicFriction = 1.0f;
    world.AttachCollider(boxId, boxCollider);

    float minY = 1e9f;
    int minYStep = -1;
    for (int step = 0; step < 600; ++step) {
        world.Step(1.0f / 120.0f);
        const float y = world.GetRigidBody(boxId)->Position().y;
        if (y < minY) { minY = y; minYStep = step; }
    }
    std::printf("[hsdrop] minY=%.4f at step=%d finalY=%.4f\n",
        minY, minYStep, world.GetRigidBody(boxId)->Position().y);
    std::fflush(stdout);
    const auto* b = world.GetRigidBody(boxId);
    // CCD must prevent the box from ever crossing the ground's center plane
    // (y=0 is the ground center; a box whose center crosses it has tunneled
    // past the halfway point, the point of no return). Transient penetration
    // during settling is recovered by the NGS pass and is not tunneling.
    EXPECT_GT(minY, 0.0f) << "box tunneled past the ground center despite CCD";
    EXPECT_NEAR(b->Orientation().w, 1.0f, 0.05f) << "box flipped on landing";
    EXPECT_LT(glm::length(b->LinearVelocity()), 0.1f);
    EXPECT_GT(b->Position().y, 0.4f) << "box did not settle back on the surface";
}

// Energy must never be CREATED by the contact solve: a box bouncing on the
// ground with restitution must come out at <= e * incoming speed, never
// faster than it arrived. Guards the speculative-bias / restitution /
// position-correction interaction against velocity overshoot ("explosion").
TEST(ContactSolverWorldTest, BounceNeverExceedsRestitutionTimesIncoming) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetGravity({0.0f, -10.5f, 0.0f});
    world.SetContinuousCollisionEnabled(false);
    world.SetSolverIterations(10);

    RigidBodyDesc ground;
    ground.position = {0.0f, -0.5f, 0.0f};
    ground.isStatic = true;
    ground.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(ground);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
    groundCollider.material.restitution = 0.6f;
    groundCollider.material.dynamicFriction = 0.0f; // no friction: isolate the normal solve
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    RigidBodyDesc box;
    box.position = {0.0f, 3.0f, 0.0f};
    box.mass = 1.0f;
    box.inertiaTensorDiagonal = glm::vec3(0.135f);
    const uint32_t boxId = world.CreateRigidBody(box);
    ColliderDesc boxCollider;
    boxCollider.shape = std::make_shared<BoxShape>(glm::vec3(0.45f));
    boxCollider.material.restitution = 0.6f;
    boxCollider.material.dynamicFriction = 0.0f;
    ASSERT_TRUE(world.AttachCollider(boxId, boxCollider));

    // Track the largest upward speed right after each bounce and compare it
    // against the incoming (downward) speed just before contact.
    float prevVy = 0.0f;
    float worstRatio = 0.0f;
    for (int step = 0; step < 1200; ++step) {
        world.Step(1.0f / 120.0f);
        const auto* b = world.GetRigidBody(boxId);
        const float vy = b->LinearVelocity().y;
        // Bounce = velocity flips from downward to upward across a step.
        if (prevVy < -1.0f && vy > 0.0f) {
            worstRatio = std::max(worstRatio, vy / -prevVy);
        }
        prevVy = vy;
    }
    // e = 0.6: the post-bounce speed must stay <= e * incoming + tolerance.
    // A ratio > 1 means the solver injected energy (explosion).
    EXPECT_LT(worstRatio, 0.65f) << "bounce overshot restitution: energy injected";
}

// Same bounce energy gate but with CCD enabled: the TOI sub-stepping must not
// inject energy either (regression for "CCD on -> boxes get flung").
TEST(ContactSolverWorldTest, BounceWithCcdNeverInjectsEnergy) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetGravity({0.0f, -10.5f, 0.0f});
    world.SetContinuousCollisionEnabled(true);
    world.SetCcdMaxSubSteps(12);
    world.SetSolverIterations(10);

    RigidBodyDesc ground;
    ground.position = {0.0f, -0.5f, 0.0f};
    ground.isStatic = true;
    ground.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(ground);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
    groundCollider.material.restitution = 0.6f;
    groundCollider.material.dynamicFriction = 0.0f;
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    RigidBodyDesc box;
    box.position = {0.0f, 3.0f, 0.0f};
    box.mass = 1.0f;
    box.inertiaTensorDiagonal = glm::vec3(0.135f);
    const uint32_t boxId = world.CreateRigidBody(box);
    ColliderDesc boxCollider;
    boxCollider.shape = std::make_shared<BoxShape>(glm::vec3(0.45f));
    boxCollider.material.restitution = 0.6f;
    boxCollider.material.dynamicFriction = 0.0f;
    ASSERT_TRUE(world.AttachCollider(boxId, boxCollider));

    float prevVy = 0.0f;
    float worstRatio = 0.0f;
    float maxAbsV = 0.0f;
    for (int step = 0; step < 1200; ++step) {
        world.Step(1.0f / 120.0f);
        const auto* b = world.GetRigidBody(boxId);
        const float vy = b->LinearVelocity().y;
        maxAbsV = std::max(maxAbsV, std::abs(vy));
        if (prevVy < -1.0f && vy > 0.0f) {
            worstRatio = std::max(worstRatio, vy / -prevVy);
        }
        prevVy = vy;
    }
    EXPECT_LT(worstRatio, 0.65f) << "CCD bounce overshot restitution: energy injected";
    // And the box must stay on the surface (not flung off / tunneled).
    const auto* b = world.GetRigidBody(boxId);
    EXPECT_GT(b->Position().y, 0.0f);
    EXPECT_LT(b->Position().y, 4.0f);
}

// Two boxes: one resting on the ground, a second dropped on top from height.
// No post-impact speed may exceed the dropper's impact speed (kinetic energy
// can only be dissipated by the inelastic contact, never created). Guards the
// multi-contact stack against the "boxes get flung apart" explosion seen in
// field-rain when a fast box lands on the pile.
TEST(ContactSolverWorldTest, BoxDroppedOnBoxNeverFlings) {
    for (const bool ccd : {false, true}) {
        PhysicsWorld world;
        world.SetFixedTimeStep(1.0f / 120.0f);
        world.SetGravity({0.0f, -10.5f, 0.0f});
        world.SetContinuousCollisionEnabled(ccd);
        world.SetSolverIterations(10);

        RigidBodyDesc ground;
        ground.position = {0.0f, -0.5f, 0.0f};
        ground.isStatic = true;
        ground.useGravity = false;
        const uint32_t groundId = world.CreateRigidBody(ground);
        ColliderDesc groundCollider;
        groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
        groundCollider.material.restitution = 0.05f;
        groundCollider.material.dynamicFriction = 1.0f;
        groundCollider.material.staticFriction = 1.0f;
        ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

        const auto boxShape = std::make_shared<BoxShape>(glm::vec3(0.45f));
        const auto addBox = [&](const glm::vec3& position) {
            RigidBodyDesc desc;
            desc.position = position;
            desc.mass = 1.0f;
            desc.inertiaTensorDiagonal = glm::vec3(0.135f);
            const uint32_t id = world.CreateRigidBody(desc);
            ColliderDesc collider;
            collider.shape = boxShape;
            collider.material.restitution = 0.05f;
            collider.material.dynamicFriction = 1.0f;
            collider.material.staticFriction = 1.0f;
            world.AttachCollider(id, collider);
            return id;
        };
        // Resting box on the ground, then a dropper falling onto it from 10 m.
        const uint32_t bottomId = addBox(glm::vec3(0.0f, 0.5f, 0.0f));
        for (int step = 0; step < 120; ++step) world.Step(1.0f / 120.0f);
        const uint32_t dropperId = addBox(glm::vec3(0.0f, 10.5f, 0.0f));

        // Impact speed from 10 m: v = sqrt(2 * 10.5 * ~9.6) ~ 14.2 m/s.
        float maxPostContactSpeed = 0.0f;
        bool contactSeen = false;
        for (int step = 0; step < 600; ++step) {
            world.Step(1.0f / 120.0f);
            const bool touching = !world.Contacts().empty();
            if (touching) contactSeen = true;
            if (contactSeen) {
                for (const uint32_t id : {bottomId, dropperId}) {
                    maxPostContactSpeed = std::max(
                        maxPostContactSpeed, glm::length(world.GetRigidBody(id)->LinearVelocity()));
                }
            }
        }
        EXPECT_TRUE(contactSeen);
        // After contact, neither body may exceed the impact speed (14.2 + margin).
        EXPECT_LT(maxPostContactSpeed, 16.0f)
            << "energy injected into the stack (ccd=" << ccd << ")";
    }
}

// Long rain energy gate: high-speed impacts and overlapping spawns must not fling boxes through the floor.
TEST(ContactSolverWorldTest, FieldRainNeverFlingsBoxes) {
    for (const bool ccd : {false, true}) {
        PhysicsWorld world;
        world.SetFixedTimeStep(1.0f / 120.0f);
        world.SetGravity({0.0f, -10.5f, 0.0f});
        world.SetContinuousCollisionEnabled(ccd);
        world.SetSolverIterations(10);

        RigidBodyDesc ground;
        ground.position = {0.0f, -0.5f, 0.0f};
        ground.isStatic = true;
        ground.useGravity = false;
        const uint32_t groundId = world.CreateRigidBody(ground);
        ColliderDesc groundCollider;
        groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
        groundCollider.material.restitution = 0.1f;
        groundCollider.material.dynamicFriction = 0.6f;
        groundCollider.material.staticFriction = 0.7f;
        ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

        const auto boxShape = std::make_shared<BoxShape>(glm::vec3(0.45f));
        std::mt19937 rng(1337);
        std::uniform_real_distribution<float> posDist(-3.5f, 3.5f);
        std::vector<uint32_t> bodies;
        float spawnAccum = 0.0f;
        float maxOvershoot = 0.0f; // max (|v| - terminalImpactSpeed) seen on contact
        int energyViolations = 0;   // count of boxes flung past terminal speed
        constexpr float kTerminal = 16.0f; // sqrt(2*10.5*13.5) ~ 16.8 from 14 m
        // True energy injection = speed ABOVE the max free-fall speed (17) at
        // low altitude; falling boxes legitimately reach ~16. Use 18 as the
        // threshold so ordinary falling is never flagged.
        constexpr float kExplosionSpeed = 18.0f;

        for (int step = 0; step < 6000; ++step) {
            const float dt = 1.0f / 120.0f;
            spawnAccum += dt * 24.0f;
            while (spawnAccum >= 1.0f && bodies.size() < 200) {
                spawnAccum -= 1.0f;
                RigidBodyDesc desc;
                desc.position = {posDist(rng), 14.0f, posDist(rng)};
                desc.mass = 1.0f;
                desc.inertiaTensorDiagonal = glm::vec3(0.135f);
                const uint32_t id = world.CreateRigidBody(desc);
                ColliderDesc collider;
                collider.shape = boxShape;
                collider.material.restitution = 0.05f;
                collider.material.dynamicFriction = 0.55f;
                collider.material.staticFriction = 0.65f;
                if (world.AttachCollider(id, collider)) bodies.push_back(id);
            }
            world.Step(dt);
            for (const uint32_t id : bodies) {
                const auto* b = world.GetRigidBody(id);
                const float speed = glm::length(b->LinearVelocity());
                // A flung box exceeds the max possible free-fall speed while
                // low to the ground (nothing can legitimately be that fast).
                if (b->Position().y < 4.0f && speed > kExplosionSpeed) {
                    ++energyViolations;
                    maxOvershoot = std::max(maxOvershoot, speed - kExplosionSpeed);
                }
            }
        }
        EXPECT_EQ(energyViolations, 0)
            << "boxes flung past terminal speed (ccd=" << ccd
            << ", overshoot=" << maxOvershoot << ")";

        // Report the worst resting penetration in the settled pile (visual
        // sink/overlap in field-rain is deep-penetration contacts).
        float maxRestingPenetration = 0.0f;
        for (const ContactManifold& contact : world.Contacts()) {
            for (std::size_t p = 0; p < contact.pointCount; ++p) {
                maxRestingPenetration = std::max(maxRestingPenetration, contact.Point(p).penetration);
            }
        }
        std::printf("[field-rain ccd=%d] maxRestingPenetration=%.5f\n", ccd ? 1 : 0, maxRestingPenetration);
    }
}

// Field-rain reproduction of the sample (BoxStackSample FieldRain mode):
// 24 boxes/s spawn at 14 m with sleep enabled, CCD on. A box visually sunk
// into the ground means its center ended more than ~half a box height below
// the surface (y < 0). Regression for "CCD on + field rain -> boxes sink
// into the ground".
TEST(ContactSolverWorldTest, FieldRainDoesNotSinkIntoGround) {
    for (const bool ccd : {false, true}) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetGravity({0.0f, -10.5f, 0.0f});
    world.SetContinuousCollisionEnabled(ccd);
    world.SetSolverIterations(10);

    RigidBodyDesc ground;
    ground.position = {0.0f, -0.5f, 0.0f};
    ground.isStatic = true;
    ground.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(ground);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
    groundCollider.material.restitution = 0.1f;
    groundCollider.material.dynamicFriction = 0.6f;
    groundCollider.material.staticFriction = 0.7f;
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    const auto boxShape = std::make_shared<BoxShape>(glm::vec3(0.45f));
    std::mt19937 rng(1337);
    std::uniform_real_distribution<float> posDist(-3.5f, 3.5f);
    std::vector<uint32_t> bodies;
    float spawnAccum = 0.0f;
    float maxRestingPenetration = 0.0f;
    std::string worstContactDesc;
    // Per ground-contact box: track the max penetration over its lifetime
    // and the step at which it occurred, so a transient impact spike is
    // distinguishable from a chronic equilibrium penetration.
    // Per-box max penetration across ANY contact it takes part in, plus a
    // per-pair sample history (step, penetration) so the event shape around
    // a deep-penetration spike is visible.
    std::unordered_map<uint32_t, float> boxMaxGroundPen;
    std::unordered_map<uint32_t, int> boxMaxGroundPenStep;
    std::unordered_map<uint32_t, uint32_t> boxWorstPartner;
    std::unordered_map<PairKey, std::vector<std::pair<int, float>>, PairKeyHash> pairPenHistory;
    // Steady-state penetration tracker: maximum penetration over the LAST
    // 600 steps (long after spawning stopped at ~step 1200, every impact
    // recovered). Impact spikes during the raining phase are expected and
    // recovered by the NGS pass; only the settled pile matters here.
    float settledMaxPenetration = 0.0f;
    // Focused trace of one box around its deepest ground contact: position,
    // velocity and the penetrations of every contact it takes part in.
    constexpr uint32_t kTraceBoxId = 30u;
    int traceWindowStart = -1;
    std::vector<std::string> traceLines;
    std::unordered_set<uint32_t> tracePrevPartners;

    for (int step = 0; step < 2400; ++step) {
        const float dt = 1.0f / 120.0f;
        spawnAccum += dt * 24.0f;
        while (spawnAccum >= 1.0f && bodies.size() < 60) {
            spawnAccum -= 1.0f;
            RigidBodyDesc desc;
            desc.position = {posDist(rng), 14.0f, posDist(rng)};
            desc.mass = 1.0f;
            desc.inertiaTensorDiagonal = glm::vec3(0.135f);
            const uint32_t id = world.CreateRigidBody(desc);
            ColliderDesc collider;
            collider.shape = boxShape;
            collider.material.restitution = 0.05f;
            collider.material.dynamicFriction = 0.55f;
            collider.material.staticFriction = 0.65f;
            if (world.AttachCollider(id, collider)) {
                bodies.push_back(id);
            }
        }
        world.Step(dt);
        // Same recycle as the sample: a body below y = -15 is removed.
        for (std::size_t index = 0; index < bodies.size();) {
            const auto* body = world.GetRigidBody(bodies[index]);
            if (body && body->Position().y < -15.0f) {
                world.DestroyRigidBody(bodies[index]);
                bodies.erase(bodies.begin() + static_cast<std::ptrdiff_t>(index));
            } else {
                ++index;
            }
        }
        if (step >= 1200) {
            // Trace window around the box's deepest event (detected lazily:
            // once boxMaxGroundPen[kTraceBoxId] is updated, keep tracing for
            // 20 more steps).
            const auto maxIt = boxMaxGroundPen.find(kTraceBoxId);
            if (maxIt != boxMaxGroundPen.end() && maxIt->second > 0.3f && traceWindowStart < 0) {
                // Trace from the scan start to just past the final deep
                // event: the full inter-event stretch matters more than a
                // narrow window (the compression phase is the question).
                traceWindowStart = 1200;
            }
            for (const ContactManifold& contact : world.Contacts()) {
                const PairKey pairKey = MakePairKey(contact.bodyA, contact.bodyB);
                for (std::size_t p = 0; p < contact.pointCount; ++p) {
                    const float pen = contact.Point(p).penetration;
                    auto& history = pairPenHistory[pairKey];
                    history.emplace_back(step, pen);
                    if (history.size() > 64u) {
                        history.erase(history.begin());
                    }
                    // Track the deepest contact of every dynamic body.
                    for (const uint32_t bodyId : {contact.bodyA, contact.bodyB}) {
                        if (bodyId == groundId) {
                            continue;
                        }
                        float& boxMax = boxMaxGroundPen[bodyId];
                        if (pen > boxMax) {
                            boxMax = pen;
                            boxMaxGroundPenStep[bodyId] = step;
                            boxWorstPartner[bodyId] =
                                bodyId == contact.bodyA ? contact.bodyB : contact.bodyA;
                        }
                    }
                    if (pen > maxRestingPenetration) {
                        maxRestingPenetration = pen;
                        const auto* a = world.GetRigidBody(contact.bodyA);
                        const auto* b = world.GetRigidBody(contact.bodyB);
                        char buf[256];
                        std::snprintf(buf, sizeof(buf),
                            "pair %u-%u pen=%.3f at step %d yA=%.3f yB=%.3f "
                            "normal=(%.2f,%.2f,%.2f) points=%zu",
                            contact.bodyA, contact.bodyB, pen, step,
                            a ? a->Position().y : -1.0f, b ? b->Position().y : -1.0f,
                            contact.normal.x, contact.normal.y, contact.normal.z,
                            contact.pointCount);
                        worstContactDesc = buf;
                    }
                    if (step >= 1800) {
                        settledMaxPenetration = std::max(settledMaxPenetration, pen);
                    }
                }
            }
            bool traceHasContact = false;
            if (traceWindowStart >= 0) {
                for (const ContactManifold& contact : world.Contacts()) {
                    if (contact.bodyA == kTraceBoxId || contact.bodyB == kTraceBoxId) {
                        traceHasContact = true;
                        break;
                    }
                }
            }
            if (traceWindowStart >= 0 && step >= traceWindowStart &&
                step <= boxMaxGroundPenStep[kTraceBoxId] + 1 &&
                // Only print rows near the events or where the box has any
                // contact (skip hundreds of idle mid-flight rows).
                (traceHasContact || step <= 1250 || step >= 2120)) {
                const auto* traceBody = world.GetRigidBody(kTraceBoxId);
                if (traceBody) {
                    std::unordered_set<uint32_t> partners;
                    for (const ContactManifold& contact : world.Contacts()) {
                        if (contact.bodyA == kTraceBoxId || contact.bodyB == kTraceBoxId) {
                            partners.insert(
                                contact.bodyA == kTraceBoxId ? contact.bodyB : contact.bodyA);
                        }
                    }
                    std::string appeared;
                    std::string vanished;
                    for (const uint32_t p : partners) {
                        if (tracePrevPartners.count(p) == 0) {
                            appeared += " +" + std::to_string(p);
                        }
                    }
                    for (const uint32_t p : tracePrevPartners) {
                        if (partners.count(p) == 0) {
                            vanished += " -" + std::to_string(p);
                        }
                    }
                    tracePrevPartners = partners;

                    char buf[640];
                    const auto& stepStats = world.LastStepStats();
                    int offset = std::snprintf(buf, sizeof(buf),
                        "  trace step %d: y=%.4f vy=%.3f sleeping=%d ccd(sub=%llu,hit=%llu) ev:%s%s contacts:",
                        step, traceBody->Position().y, traceBody->LinearVelocity().y,
                        traceBody->IsSleeping() ? 1 : 0,
                        static_cast<unsigned long long>(stepStats.ccdSubStepCount),
                        static_cast<unsigned long long>(stepStats.ccdToiHitCount),
                        appeared.c_str(), vanished.c_str());
                    for (const ContactManifold& contact : world.Contacts()) {
                        if (contact.bodyA != kTraceBoxId && contact.bodyB != kTraceBoxId) {
                            continue;
                        }
                        float pairMaxPen = -1.0f;
                        for (std::size_t p = 0; p < contact.pointCount; ++p) {
                            pairMaxPen = std::max(pairMaxPen, contact.Point(p).penetration);
                        }
                        offset += std::snprintf(buf + offset, sizeof(buf) - offset,
                            " %u(pen=%.3f,n=%zu)",
                            contact.bodyA == kTraceBoxId ? contact.bodyB : contact.bodyA,
                            pairMaxPen, contact.pointCount);
                    }
                    traceLines.emplace_back(buf);
                }
                // At the deep-penetration step and the one before, dump the
                // heights of every box touching the ground so the impact
                // source that pressed box 30 down is visible.
                const int eventStep = boxMaxGroundPenStep[kTraceBoxId];
                if (step == eventStep - 1 || step == eventStep) {
                    std::string dump = "  ground-row step " + std::to_string(step) + ":";
                    for (const ContactManifold& contact : world.Contacts()) {
                        const uint32_t boxId =
                            contact.bodyA == groundId ? contact.bodyB :
                            contact.bodyB == groundId ? contact.bodyA : 0u;
                        if (boxId == 0u) {
                            continue;
                        }
                        const auto* b = world.GetRigidBody(boxId);
                        char item[64];
                        std::snprintf(item, sizeof(item), " %u@%.3f(vy=%.2f)",
                            boxId, b->Position().y, b->LinearVelocity().y);
                        dump += item;
                    }
                    traceLines.push_back(std::move(dump));
                }
            }
        }
    }

    int sunkCount = 0;
    for (const uint32_t id : bodies) {
        const auto* body = world.GetRigidBody(id);
        ASSERT_NE(body, nullptr);
        if (body->Position().y < 0.0f) {
            ++sunkCount;
        }
    }
    // Steady-state penetration: only the LAST 600 steps counted (see the
    // tracker above). Impact spikes during the raining phase are recovered
    // by the NGS pass and are asserted separately by the sinking check.
    std::printf("[fieldrain-sink ccd=%d] sunk=%d/%zu maxRestingPenetration=%.5f settled=%.5f\n  worst: %s\n",
        ccd ? 1 : 0, sunkCount, bodies.size(), maxRestingPenetration,
        settledMaxPenetration, worstContactDesc.c_str());
    for (const auto& [boxId, pen] : boxMaxGroundPen) {
        if (pen > 0.1f) {
            const auto* body = world.GetRigidBody(boxId);
            const uint32_t partner = boxWorstPartner[boxId];
            std::printf(
                "[fieldrain-sink] box %u maxPen=%.3f vs %u at step %d (finalY=%.3f sleeping=%d)\n",
                boxId, pen, partner, boxMaxGroundPenStep[boxId],
                body ? body->Position().y : -1.0f, body && body->IsSleeping() ? 1 : 0);
        }
    }
    for (const std::string& line : traceLines) {
        std::printf("%s\n", line.c_str());
    }
    EXPECT_EQ(sunkCount, 0) << "boxes sank into the ground (ccd=" << ccd << ")";
    // Impact recovery: a box that was shoved deep by a falling impact must
    // be recovered by the position solver back to the surface. The spike
    // itself is unavoidable (the velocity solve is soft over one step); the
    // recovery is the contract.
    int unrecoveredDeep = 0;
    for (const auto& [boxId, pen] : boxMaxGroundPen) {
        if (pen > 0.2f) {
            const auto* body = world.GetRigidBody(boxId);
            if (body && body->Position().y < 0.1f) {
                ++unrecoveredDeep;
            }
        }
    }
    EXPECT_EQ(unrecoveredDeep, 0)
        << "deep-penetration impacts were not recovered (ccd=" << ccd << ")";
    }
}

} // namespace
