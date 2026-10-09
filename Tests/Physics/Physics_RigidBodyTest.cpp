// Phase 3 focused tests: cached world inverse inertia, L-authoritative angular
// state and the solver-only precomputed angular response entry.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <glm/gtc/quaternion.hpp>

#include "Physics/RigidBody.h"

using Runtime::Physics::RigidBody;
using Runtime::Physics::RigidBodyDesc;

namespace {

glm::mat3 ComputeWorldInverseInertia(const glm::quat& orientation, const glm::vec3& inverseDiagonal) {
    const glm::mat3 r = glm::mat3_cast(glm::normalize(orientation));
    const glm::mat3 iBodyInv = glm::mat3(
        glm::vec3(inverseDiagonal.x, 0.0f, 0.0f),
        glm::vec3(0.0f, inverseDiagonal.y, 0.0f),
        glm::vec3(0.0f, 0.0f, inverseDiagonal.z));
    return r * iBodyInv * glm::transpose(r);
}

glm::mat3 ComputeWorldInertia(const glm::quat& orientation, const glm::vec3& diagonal) {
    const glm::mat3 r = glm::mat3_cast(glm::normalize(orientation));
    const glm::mat3 iBody = glm::mat3(
        glm::vec3(diagonal.x, 0.0f, 0.0f),
        glm::vec3(0.0f, diagonal.y, 0.0f),
        glm::vec3(0.0f, 0.0f, diagonal.z));
    return r * iBody * glm::transpose(r);
}

void ExpectMat3Near(const glm::mat3& actual, const glm::mat3& expected, float tolerance) {
    for (int column = 0; column < 3; ++column) {
        for (int row = 0; row < 3; ++row) {
            EXPECT_NEAR(actual[column][row], expected[column][row], tolerance);
        }
    }
}

void ExpectVec3Near(const glm::vec3& actual, const glm::vec3& expected, float tolerance) {
    EXPECT_NEAR(actual.x, expected.x, tolerance);
    EXPECT_NEAR(actual.y, expected.y, tolerance);
    EXPECT_NEAR(actual.z, expected.z, tolerance);
}

RigidBody MakeDynamicBody() {
    RigidBodyDesc desc;
    desc.position = glm::vec3(1.0f, 2.0f, 3.0f);
    desc.orientation = glm::normalize(glm::quat(0.9f, 0.2f, 0.3f, 0.1f));
    desc.inertiaTensorDiagonal = glm::vec3(2.0f, 3.0f, 4.0f);
    desc.angularVelocity = glm::vec3(0.5f, -1.0f, 0.25f);
    desc.useGravity = false;
    return RigidBody(desc);
}

void ExpectMomentumVelocityConsistent(const RigidBody& body, float tolerance) {
    const glm::mat3 worldInertia = ComputeWorldInertia(body.Orientation(), body.InertiaTensorDiagonal());
    ExpectVec3Near(worldInertia * body.AngularVelocity(), body.AngularMomentum(), tolerance);
    ExpectVec3Near(body.InverseInertiaTensorWorld() * body.AngularMomentum(), body.AngularVelocity(), tolerance);
}

} // namespace

TEST(RigidBodyAngularStateTest, CachedInverseInertiaMatchesRotationFormula) {
    RigidBody body = MakeDynamicBody();
    ExpectMat3Near(
        body.InverseInertiaTensorWorld(),
        ComputeWorldInverseInertia(body.Orientation(), body.InverseInertiaTensorDiagonal()),
        1e-5f);

    // Pose change refreshes the cache.
    const glm::quat rotated = glm::normalize(
        glm::angleAxis(0.7f, glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f))) * body.Orientation());
    body.SetOrientation(rotated);
    ExpectMat3Near(
        body.InverseInertiaTensorWorld(),
        ComputeWorldInverseInertia(rotated, body.InverseInertiaTensorDiagonal()),
        1e-5f);
}

TEST(RigidBodyAngularStateTest, SolverAngularResponseMatchesApplyAngularImpulse) {
    RigidBody reference = MakeDynamicBody();
    RigidBody solverPath = MakeDynamicBody();

    const glm::vec3 impulses[] = {
        glm::vec3(0.3f, -0.2f, 0.1f),
        glm::vec3(-0.05f, 0.4f, 0.0f),
        glm::vec3(0.0f, 0.0f, -0.6f),
    };
    for (const glm::vec3& impulse : impulses) {
        reference.ApplyAngularImpulse(impulse);
        solverPath.ApplySolverAngularImpulse(
            impulse, solverPath.InverseInertiaTensorWorld() * impulse);

        ExpectVec3Near(solverPath.AngularMomentum(), reference.AngularMomentum(), 1e-6f);
        ExpectVec3Near(solverPath.AngularVelocity(), reference.AngularVelocity(), 1e-6f);
    }
}

TEST(RigidBodyAngularStateTest, MomentumStaysAuthoritativeAcrossImpulsesPoseAndIntegration) {
    RigidBody body = MakeDynamicBody();
    ExpectMomentumVelocityConsistent(body, 1e-5f);

    // Multiple impulses at a fixed pose keep L == Iworld * omega.
    body.ApplyAngularImpulse(glm::vec3(0.2f, 0.1f, -0.3f));
    body.ApplyAngularImpulse(glm::vec3(-0.1f, 0.05f, 0.4f));
    ExpectMomentumVelocityConsistent(body, 1e-5f);
    const glm::vec3 momentumAfterImpulses = body.AngularMomentum();

    // External pose change keeps L and re-derives omega at the new pose.
    body.SetOrientation(glm::normalize(
        glm::angleAxis(-0.4f, glm::vec3(0.0f, 1.0f, 0.0f)) * body.Orientation()));
    ExpectVec3Near(body.AngularMomentum(), momentumAfterImpulses, 0.0f);
    ExpectMomentumVelocityConsistent(body, 1e-5f);

    // Inertia change keeps L and re-derives omega through the new inertia.
    body.SetInertiaTensorDiagonal(glm::vec3(1.0f, 2.5f, 3.5f));
    ExpectVec3Near(body.AngularMomentum(), momentumAfterImpulses, 0.0f);
    ExpectMomentumVelocityConsistent(body, 1e-5f);

    // Torque-free integration rotates the pose; omega is re-derived from the
    // untouched L after every orientation update.
    for (int step = 0; step < 240; ++step) {
        body.Integrate(1.0f / 120.0f, glm::vec3(0.0f));
    }
    ExpectVec3Near(body.AngularMomentum(), momentumAfterImpulses, 0.0f);
    ExpectMomentumVelocityConsistent(body, 1e-4f);
}

TEST(RigidBodyAngularStateTest, StaticBodiesKeepZeroInverseInertiaCache) {
    RigidBodyDesc desc;
    desc.isStatic = true;
    desc.inertiaTensorDiagonal = glm::vec3(2.0f, 3.0f, 4.0f);
    desc.angularVelocity = glm::vec3(1.0f, 1.0f, 1.0f);
    RigidBody body(desc);

    ExpectMat3Near(body.InverseInertiaTensorWorld(), glm::mat3(0.0f), 0.0f);
    ExpectVec3Near(body.AngularVelocity(), glm::vec3(0.0f), 0.0f);
    ExpectVec3Near(body.AngularMomentum(), glm::vec3(0.0f), 0.0f);

    // Static -> dynamic refreshes the cache for the current pose.
    body.SetStatic(false);
    ExpectMat3Near(
        body.InverseInertiaTensorWorld(),
        ComputeWorldInverseInertia(body.Orientation(), body.InverseInertiaTensorDiagonal()),
        1e-6f);

    body.SetAngularVelocity(glm::vec3(0.5f, -0.25f, 1.0f));
    ExpectVec3Near(
        body.AngularVelocity(),
        body.InverseInertiaTensorWorld() * body.AngularMomentum(),
        1e-5f);

    // Dynamic -> static zeroes the cache again.
    body.SetStatic(true);
    ExpectMat3Near(body.InverseInertiaTensorWorld(), glm::mat3(0.0f), 0.0f);
    ExpectVec3Near(body.AngularVelocity(), glm::vec3(0.0f), 0.0f);
    ExpectVec3Near(body.AngularMomentum(), glm::vec3(0.0f), 0.0f);
}

TEST(RigidBodyAngularStateTest, CcdInterpolatedCopyRefreshesCache) {
    const RigidBody source = MakeDynamicBody();
    // Mirror ContinuousCollisionDetector::GenerateContactAtTime: copy the body
    // and teleport the copy to the interpolated transform. The copy must not
    // reuse the source pose's cached inverse inertia.
    RigidBody copy = source;
    const glm::quat sweptOrientation = glm::normalize(
        glm::angleAxis(0.35f, glm::normalize(glm::vec3(0.2f, 1.0f, 0.4f))) * source.Orientation());
    copy.SetPosition(source.Position() + glm::vec3(1.0f, 0.0f, 0.0f));
    copy.SetOrientation(sweptOrientation);

    ExpectMat3Near(
        copy.InverseInertiaTensorWorld(),
        ComputeWorldInverseInertia(sweptOrientation, copy.InverseInertiaTensorDiagonal()),
        1e-6f);
    ExpectVec3Near(copy.AngularMomentum(), source.AngularMomentum(), 0.0f);
    ExpectVec3Near(
        copy.AngularVelocity(),
        copy.InverseInertiaTensorWorld() * copy.AngularMomentum(),
        1e-6f);

    // The source body keeps its own cache untouched.
    ExpectMat3Near(
        source.InverseInertiaTensorWorld(),
        ComputeWorldInverseInertia(source.Orientation(), source.InverseInertiaTensorDiagonal()),
        1e-6f);
}

TEST(RigidBodyAngularStateTest, FreeRotationConservesMomentumAndBoundsEnergyDrift) {
    RigidBodyDesc desc;
    desc.inertiaTensorDiagonal = glm::vec3(1.0f, 2.0f, 4.0f);
    desc.angularVelocity = glm::vec3(2.5f, 0.8f, -0.4f);
    desc.useGravity = false;
    RigidBody body(desc);

    constexpr float dt = 1.0f / 120.0f;
    constexpr int stepCount = 600;  // 5 seconds of torque-free rotation.

    const glm::vec3 initialMomentum = body.AngularMomentum();
    const float initialEnergy = 0.5f * glm::dot(body.AngularVelocity(), initialMomentum);
    ASSERT_GT(initialEnergy, 0.0f);

    float maxRelativeEnergyDrift = 0.0f;
    for (int step = 0; step < stepCount; ++step) {
        body.Integrate(dt, glm::vec3(0.0f));
        const float energy = 0.5f * glm::dot(body.AngularVelocity(), body.AngularMomentum());
        maxRelativeEnergyDrift = std::max(
            maxRelativeEnergyDrift, std::abs(energy - initialEnergy) / initialEnergy);
    }

    // No torque: the authoritative momentum is untouched exactly.
    ExpectVec3Near(body.AngularMomentum(), initialMomentum, 0.0f);

    std::printf(
        "[FreeRotation] dt=%.6f steps=%d maxRelativeEnergyDrift=%.6e\n",
        static_cast<double>(dt), stepCount, static_cast<double>(maxRelativeEnergyDrift));
    // Recorded tolerance: measured max relative drift is ~2.8e-2 with
    // dt = 1/120 over 5 simulated seconds of non-uniform free rotation
    // (first-order semi-implicit orientation integration); bound set to 5e-2.
    EXPECT_LT(maxRelativeEnergyDrift, 5e-2f);
}

// Phase 9: the velocity-only compensation used when a sleeping body is woken
// by a confirmed contact applies exactly one dt of force/gravity and torque
// increments and never moves the pose.
TEST(RigidBodySleepTest, IntegrateVelocityCompensatesExactlyOnce) {
    RigidBodyDesc desc;
    desc.mass = 2.0f;
    desc.inertiaTensorDiagonal = glm::vec3(1.0f);
    RigidBody body(desc);
    const glm::vec3 initialPosition = body.Position();
    const glm::quat initialOrientation = body.Orientation();

    body.ApplyForce(glm::vec3(4.0f, 0.0f, 0.0f));
    body.ApplyTorque(glm::vec3(0.0f, 0.0f, 3.0f));
    body.IntegrateVelocity(0.5f, glm::vec3(0.0f, -10.0f, 0.0f));

    // v += (F / m + g) * dt = ((2,0,0) + (0,-10,0)) * 0.5 = (1, -5, 0).
    ExpectVec3Near(body.LinearVelocity(), glm::vec3(1.0f, -5.0f, 0.0f), 1e-6f);
    // L += torque * dt = (0,0,1.5); identity inertia/orientation -> omega = L.
    ExpectVec3Near(body.AngularMomentum(), glm::vec3(0.0f, 0.0f, 1.5f), 1e-6f);
    ExpectVec3Near(body.AngularVelocity(), glm::vec3(0.0f, 0.0f, 1.5f), 1e-6f);
    // Velocity-only: the pose must not move.
    ExpectVec3Near(body.Position(), initialPosition, 0.0f);
    EXPECT_EQ(body.Orientation(), initialOrientation);
}

// Phase 9: sleep transitions zero the residual motion state consistently and
// never apply to static bodies.
TEST(RigidBodySleepTest, EnterSleepZeroesMotionStateAndStaticNeverSleeps) {
    RigidBody body = MakeDynamicBody();
    body.EnterSleep();
    EXPECT_TRUE(body.IsSleeping());
    ExpectVec3Near(body.LinearVelocity(), glm::vec3(0.0f), 0.0f);
    ExpectVec3Near(body.AngularVelocity(), glm::vec3(0.0f), 0.0f);
    ExpectVec3Near(body.AngularMomentum(), glm::vec3(0.0f), 0.0f);

    body.WakeUp();
    EXPECT_FALSE(body.IsSleeping());
    EXPECT_EQ(body.GetSleepState(), Runtime::Physics::SleepState::Awake);
    EXPECT_EQ(body.SleepTimer(), 0.0f);

    RigidBodyDesc staticDesc;
    staticDesc.isStatic = true;
    RigidBody staticBody(staticDesc);
    staticBody.EnterSleep();
    EXPECT_FALSE(staticBody.IsSleeping());
}
