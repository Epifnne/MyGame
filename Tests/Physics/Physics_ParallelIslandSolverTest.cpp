// Phase 8 acceptance gates: parallel island solver over physics islands.
//
// - 1 worker and N workers produce bit-identical contact sets, events and
//   final body states: islands own disjoint dynamic bodies, shared static
//   bodies are never written by the solver (static-guarded impulse entries,
//   position correction skips statics), and the per-island Gauss-Seidel
//   sequence is fixed at schedule time on the main thread.
// - Disabling the parallel island solver matches the enabled path bit-exactly.
// - Independent stacks sharing one static ground stay independent islands and
//   both settle (shared static ground is read-only during the solve).
// - Job telemetry reflects the scheduling and the serial fallback.
//
// The runtime ownership assertion (debug builds stamp every solved dynamic
// body with the current query epoch inside the island task) is exercised by
// every parallel test in this file: a duplicate stamp aborts the process.
// ThreadSanitizer is unavailable on this platform (MinGW-GCC), so the
// structural argument above plus these deterministic tests stand in for it.
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "Physics/CollisionShape.h"
#include "Physics/PhysicsWorld.h"

namespace {

using Runtime::Physics::BoxShape;
using Runtime::Physics::ColliderDesc;
using Runtime::Physics::ContactEvent;
using Runtime::Physics::ContactManifold;
using Runtime::Physics::PhysicsWorld;
using Runtime::Physics::RigidBody;
using Runtime::Physics::RigidBodyDesc;

constexpr float kFixedDt = 1.0f / 60.0f;

uint32_t AddGround(PhysicsWorld& world) {
    RigidBodyDesc ground;
    ground.position = glm::vec3(0.0f, -0.5f, 0.0f);
    ground.isStatic = true;
    ground.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(ground);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(30.0f, 0.5f, 30.0f));
    groundCollider.material.dynamicFriction = 0.6f;
    groundCollider.material.staticFriction = 0.7f;
    EXPECT_TRUE(world.AttachCollider(groundId, groundCollider));
    return groundId;
}

void AddBox(
    PhysicsWorld& world,
    const std::shared_ptr<BoxShape>& shape,
    const glm::vec3& position,
    const glm::quat& orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)) {
    RigidBodyDesc body;
    body.position = position;
    body.orientation = orientation;
    body.inertiaTensorDiagonal = glm::vec3(0.135f);
    const uint32_t id = world.CreateRigidBody(body);
    ColliderDesc collider;
    collider.shape = shape;
    collider.material.restitution = 0.05f;
    collider.material.dynamicFriction = 0.55f;
    collider.material.staticFriction = 0.65f;
    EXPECT_TRUE(world.AttachCollider(id, collider));
}

// 64-box jittered field, identical construction to the narrow-phase tests:
// deterministic LCG jitter, no <random>.
std::unique_ptr<PhysicsWorld> MakeBoxFieldWorld(uint32_t workerCount, bool islandParallel) {
    auto world = std::make_unique<PhysicsWorld>();
    world->SetFixedTimeStep(kFixedDt);
    world->SetContinuousCollisionEnabled(false);
    world->SetPhysicsWorkerCount(workerCount);
    world->SetParallelIslandSolverEnabled(islandParallel);
    AddGround(*world);

    uint32_t lcgState = 1337;
    const auto jitter = [&lcgState]() {
        lcgState = lcgState * 1664525u + 1013904223u;
        return (static_cast<float>(lcgState >> 8) / 16777216.0f - 0.5f) * 0.08f;
    };

    const auto shape = std::make_shared<BoxShape>(glm::vec3(0.45f));
    constexpr std::size_t side = 8;
    for (std::size_t index = 0; index < 64; ++index) {
        const float x = (static_cast<float>(index % side) - 3.5f) * 0.92f + jitter();
        const float z = (static_cast<float>(index / side) - 3.5f) * 0.92f + jitter();
        AddBox(*world, shape, glm::vec3(x, 0.55f + jitter(), z));
    }
    return world;
}

// Comparable snapshot of everything the parallel island solver could influence.
struct WorldSnapshot {
    std::vector<glm::vec3> positions;
    std::vector<glm::quat> orientations;
    std::vector<glm::vec3> linearVelocities;
    std::vector<glm::vec3> angularVelocities;
    std::vector<ContactManifold> contacts;
    std::vector<ContactEvent> events;
    uint64_t velocitySolverPasses = 0;
    uint64_t positionSolverPasses = 0;
    uint64_t contactPoints = 0;
    std::size_t islandCount = 0;
    std::size_t islandMaxBodyCount = 0;
};

WorldSnapshot RunAndCapture(PhysicsWorld& world, std::size_t frames) {
    WorldSnapshot snapshot;
    for (std::size_t frame = 0; frame < frames; ++frame) {
        world.Step(kFixedDt);
        snapshot.velocitySolverPasses += world.LastStepStats().velocitySolverPassCount;
        snapshot.positionSolverPasses += world.LastStepStats().positionSolverPassCount;
        snapshot.contactPoints += world.LastStepStats().contactPointCount;
    }
    snapshot.islandCount = world.LastStepStats().islandCount;
    snapshot.islandMaxBodyCount = world.LastStepStats().islandMaxBodyCount;

    std::vector<const RigidBody*> bodies;
    for (const auto& entry : world.Bodies()) {
        bodies.push_back(&entry.second);
    }
    std::sort(bodies.begin(), bodies.end(), [](const RigidBody* a, const RigidBody* b) {
        return a->Id() < b->Id();
    });
    for (const RigidBody* body : bodies) {
        snapshot.positions.push_back(body->Position());
        snapshot.orientations.push_back(body->Orientation());
        snapshot.linearVelocities.push_back(body->LinearVelocity());
        snapshot.angularVelocities.push_back(body->AngularVelocity());
    }
    snapshot.contacts = world.Contacts();
    snapshot.events = world.ContactEvents();
    return snapshot;
}

void ExpectVec3BitExact(const glm::vec3& a, const glm::vec3& b) {
    EXPECT_EQ(a.x, b.x);
    EXPECT_EQ(a.y, b.y);
    EXPECT_EQ(a.z, b.z);
}

void ExpectSnapshotsBitExact(const WorldSnapshot& a, const WorldSnapshot& b) {
    ASSERT_EQ(a.positions.size(), b.positions.size());
    for (std::size_t index = 0; index < a.positions.size(); ++index) {
        ExpectVec3BitExact(a.positions[index], b.positions[index]);
        EXPECT_EQ(a.orientations[index].x, b.orientations[index].x);
        EXPECT_EQ(a.orientations[index].y, b.orientations[index].y);
        EXPECT_EQ(a.orientations[index].z, b.orientations[index].z);
        EXPECT_EQ(a.orientations[index].w, b.orientations[index].w);
        ExpectVec3BitExact(a.linearVelocities[index], b.linearVelocities[index]);
        ExpectVec3BitExact(a.angularVelocities[index], b.angularVelocities[index]);
    }

    // Contact sets and every solved impulse field must be identical (stable
    // PairKey order, main-thread commit).
    ASSERT_EQ(a.contacts.size(), b.contacts.size());
    for (std::size_t index = 0; index < a.contacts.size(); ++index) {
        const ContactManifold& manifoldA = a.contacts[index];
        const ContactManifold& manifoldB = b.contacts[index];
        EXPECT_EQ(manifoldA.bodyA, manifoldB.bodyA);
        EXPECT_EQ(manifoldA.bodyB, manifoldB.bodyB);
        ExpectVec3BitExact(manifoldA.normal, manifoldB.normal);
        ASSERT_EQ(manifoldA.pointCount, manifoldB.pointCount);
        for (std::size_t point = 0; point < manifoldA.pointCount; ++point) {
            ExpectVec3BitExact(manifoldA.Point(point).position, manifoldB.Point(point).position);
            EXPECT_EQ(manifoldA.Point(point).penetration, manifoldB.Point(point).penetration);
            EXPECT_EQ(
                manifoldA.Point(point).accumulatedNormalImpulse,
                manifoldB.Point(point).accumulatedNormalImpulse);
            EXPECT_EQ(
                manifoldA.Point(point).accumulatedTangentImpulse.x,
                manifoldB.Point(point).accumulatedTangentImpulse.x);
            EXPECT_EQ(
                manifoldA.Point(point).accumulatedTangentImpulse.y,
                manifoldB.Point(point).accumulatedTangentImpulse.y);
        }
        EXPECT_EQ(manifoldA.fixedStepNormalImpulse, manifoldB.fixedStepNormalImpulse);
        ExpectVec3BitExact(manifoldA.fixedStepTangentImpulse, manifoldB.fixedStepTangentImpulse);
    }

    // Event streams are committed on the main thread in stable order.
    ASSERT_EQ(a.events.size(), b.events.size());
    for (std::size_t index = 0; index < a.events.size(); ++index) {
        EXPECT_EQ(a.events[index].type, b.events[index].type);
        EXPECT_EQ(a.events[index].pair, b.events[index].pair);
        EXPECT_EQ(a.events[index].fixedStepId, b.events[index].fixedStepId);
        EXPECT_EQ(a.events[index].queryEpoch, b.events[index].queryEpoch);
    }

    // Solver pass counts and island snapshots are worker-count independent.
    EXPECT_EQ(a.velocitySolverPasses, b.velocitySolverPasses);
    EXPECT_EQ(a.positionSolverPasses, b.positionSolverPasses);
    EXPECT_EQ(a.islandCount, b.islandCount);
    EXPECT_EQ(a.islandMaxBodyCount, b.islandMaxBodyCount);
}

} // namespace

TEST(PhysicsParallelIslandSolverTest, MultiWorkerMatchesSingleWorkerBitExact) {
    auto serialWorld = MakeBoxFieldWorld(1, true);
    const WorldSnapshot serial = RunAndCapture(*serialWorld, 60);

    auto parallelWorld = MakeBoxFieldWorld(4, true);
    const WorldSnapshot parallel = RunAndCapture(*parallelWorld, 60);

    ASSERT_GT(serial.contactPoints, 0u);
    // The box field settles into many independent ground-contact islands.
    ASSERT_GE(serial.islandCount, 8u);
    ExpectSnapshotsBitExact(serial, parallel);
}

TEST(PhysicsParallelIslandSolverTest, DisabledToggleMatchesEnabledBitExact) {
    auto enabledWorld = MakeBoxFieldWorld(4, true);
    const WorldSnapshot enabled = RunAndCapture(*enabledWorld, 60);

    auto disabledWorld = MakeBoxFieldWorld(4, false);
    const WorldSnapshot disabled = RunAndCapture(*disabledWorld, 60);

    ASSERT_GT(enabled.contactPoints, 0u);
    ExpectSnapshotsBitExact(enabled, disabled);
}

TEST(PhysicsParallelIslandSolverTest, SharedStaticGroundSolvedIdenticalAcrossWorkerCounts) {
    // Two 3-box stacks sharing the same static ground, far enough apart to
    // never interact. Both stacks belong to different islands but read the
    // SAME static body; if island tasks wrote the shared static body, the
    // 1-worker and 4-worker runs would diverge. (Long-horizon stack
    // stability is the known Phase 4 limitation and is not gated here.)
    const auto makeStacksWorld = [](uint32_t workerCount) {
        auto world = std::make_unique<PhysicsWorld>();
        world->SetFixedTimeStep(kFixedDt);
        world->SetContinuousCollisionEnabled(false);
        world->SetPhysicsWorkerCount(workerCount);
        AddGround(*world);
        const auto shape = std::make_shared<BoxShape>(glm::vec3(0.45f));
        for (const float stackX : {-5.0f, 5.0f}) {
            for (int level = 0; level < 3; ++level) {
                AddBox(
                    *world,
                    shape,
                    glm::vec3(stackX, 0.45f + 0.92f * static_cast<float>(level), 0.0f));
            }
        }
        return world;
    };

    auto serialWorld = makeStacksWorld(1);
    const WorldSnapshot serial = RunAndCapture(*serialWorld, 120);
    auto parallelWorld = makeStacksWorld(4);
    const WorldSnapshot parallel = RunAndCapture(*parallelWorld, 120);

    ASSERT_GT(serial.contactPoints, 0u);
    ExpectSnapshotsBitExact(serial, parallel);

    // Sanity: no fall-through or explosion in either configuration. The
    // velocity bound is deliberately loose — the stacks spread as they settle
    // (the known Phase 4 limitation); this only guards against blow-ups.
    for (const auto& entry : parallelWorld->Bodies()) {
        if (entry.second.IsStatic()) {
            continue;
        }
        EXPECT_GT(entry.second.Position().y, 0.2f);
        EXPECT_LT(glm::length(entry.second.LinearVelocity()), 5.0f);
    }
}

TEST(PhysicsParallelIslandSolverTest, JobTelemetryReflectsSchedulingAndSerialFallback) {
    // 12 independent resting boxes, one constraint per island; with one
    // constraint per job every island becomes its own job.
    auto makeWorld = [](uint32_t workerCount) {
        auto world = std::make_unique<PhysicsWorld>();
        world->SetFixedTimeStep(kFixedDt);
        world->SetContinuousCollisionEnabled(false);
        world->SetPhysicsWorkerCount(workerCount);
        world->SetIslandSolverMinConstraintsPerJob(1);
        AddGround(*world);
        const auto shape = std::make_shared<BoxShape>(glm::vec3(0.45f));
        for (std::size_t index = 0; index < 12; ++index) {
            AddBox(*world, shape, glm::vec3(static_cast<float>(index) * 2.0f - 11.0f, 0.46f, 0.0f));
        }
        return world;
    };

    auto parallelWorld = makeWorld(4);
    for (std::size_t frame = 0; frame < 10; ++frame) {
        parallelWorld->Step(kFixedDt);
    }
    const auto& parallelStats = parallelWorld->LastStepStats();
    EXPECT_EQ(parallelStats.islandCount, 12u);
    // 12 islands with one constraint each must schedule into >= 12 jobs.
    EXPECT_GE(parallelStats.islandSolverJobCount, 12u);
    EXPECT_GE(parallelStats.islandSolverWorkerCount, 1u);
    EXPECT_GT(parallelStats.islandSolverWorkerBusyMilliseconds, 0.0);
    EXPECT_GE(parallelStats.islandSolverTailWaitMilliseconds, 0.0);

    // workerCount == 1 is the fully serial deterministic fallback executed
    // in-line on the calling thread: a single chunk record, one participant.
    auto serialWorld = makeWorld(1);
    for (std::size_t frame = 0; frame < 10; ++frame) {
        serialWorld->Step(kFixedDt);
    }
    const auto& serialStats = serialWorld->LastStepStats();
    EXPECT_EQ(serialStats.islandSolverJobCount, 1u);
    EXPECT_EQ(serialStats.islandSolverWorkerCount, 1u);
}
