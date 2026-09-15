// Phase 6 acceptance gates: parallel narrow-phase over the midphase work list.
//
// - 1 worker and N workers produce identical pairs, contact points, normals,
//   events and final body states (bit-exact: workers only write private output
//   slots and the main thread commits in stable PairKey order).
// - Disabling the parallel path matches the enabled path bit-exactly.
// - Query statistics use task-private counters merged after the barrier, so
//   merged totals equal the serial totals.
// - Job telemetry records per-thread job counts, busy time and tail wait.
//
// ThreadSanitizer is part of the acceptance gates on platforms that support
// it; MinGW-GCC on Windows does not ship TSan, so this platform relies on the
// structural race-freedom arguments (frozen inputs, private output slots,
// atomic chunk claiming, post-barrier merge) exercised by these tests.
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "Core/JobSystem.h"
#include "Physics/CollisionDetector.h"
#include "Physics/CollisionShape.h"
#include "Physics/Midphase.h"
#include "Physics/PhysicsWorld.h"

namespace {

using Runtime::Physics::BoxShape;
using Runtime::Physics::Collider;
using Runtime::Physics::ColliderDesc;
using Runtime::Physics::CollisionDetectionResult;
using Runtime::Physics::CollisionDetectionStats;
using Runtime::Physics::CollisionDetector;
using Runtime::Physics::ContactEvent;
using Runtime::Physics::ContactManifold;
using Runtime::Physics::NarrowPhaseParallelStats;
using Runtime::Physics::PhysicsWorld;
using Runtime::Physics::RigidBody;
using Runtime::Physics::RigidBodyDesc;

constexpr float kFixedDt = 1.0f / 60.0f;
constexpr std::size_t kSceneBodies = 64;
constexpr std::size_t kSimFrames = 60;

std::unique_ptr<PhysicsWorld> MakeBoxFieldWorld(uint32_t workerCount, bool parallelEnabled) {
    auto world = std::make_unique<PhysicsWorld>();
    world->SetFixedTimeStep(kFixedDt);
    world->SetContinuousCollisionEnabled(false);
    world->SetPhysicsWorkerCount(workerCount);
    world->SetParallelNarrowphaseEnabled(parallelEnabled);

    RigidBodyDesc ground;
    ground.position = glm::vec3(0.0f, -0.5f, 0.0f);
    ground.isStatic = true;
    ground.useGravity = false;
    const uint32_t groundId = world->CreateRigidBody(ground);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
    groundCollider.material.dynamicFriction = 0.6f;
    groundCollider.material.staticFriction = 0.7f;
    EXPECT_TRUE(world->AttachCollider(groundId, groundCollider));

    // Deterministic jitter without <random>: a tiny LCG keeps construction
    // identical across runs and worker configurations.
    uint32_t lcgState = 1337;
    const auto jitter = [&lcgState]() {
        lcgState = lcgState * 1664525u + 1013904223u;
        return (static_cast<float>(lcgState >> 8) / 16777216.0f - 0.5f) * 0.08f;
    };

    const auto shape = std::make_shared<BoxShape>(glm::vec3(0.45f));
    constexpr std::size_t side = 8;
    for (std::size_t index = 0; index < kSceneBodies; ++index) {
        const float x = (static_cast<float>(index % side) - 3.5f) * 0.92f + jitter();
        const float z = (static_cast<float>(index / side) - 3.5f) * 0.92f + jitter();
        RigidBodyDesc body;
        body.position = glm::vec3(x, 0.55f + jitter(), z);
        body.inertiaTensorDiagonal = glm::vec3(0.135f);
        const uint32_t id = world->CreateRigidBody(body);
        ColliderDesc collider;
        collider.shape = shape;
        collider.material.dynamicFriction = 0.55f;
        collider.material.staticFriction = 0.65f;
        EXPECT_TRUE(world->AttachCollider(id, collider));
    }
    return world;
}

// Comparable snapshot of everything the parallel narrow-phase could influence.
struct WorldSnapshot {
    std::vector<glm::vec3> positions;
    std::vector<glm::quat> orientations;
    std::vector<glm::vec3> linearVelocities;
    std::vector<glm::vec3> angularVelocities;
    std::vector<ContactManifold> contacts;
    std::vector<ContactEvent> events;
    uint64_t narrowPhaseTests = 0;
    uint64_t gjkCalls = 0;
    uint64_t epaCalls = 0;
    uint64_t manifolds = 0;
    uint64_t contactPoints = 0;
};

WorldSnapshot RunAndCapture(PhysicsWorld& world) {
    WorldSnapshot snapshot;
    for (std::size_t frame = 0; frame < kSimFrames; ++frame) {
        world.Step(kFixedDt);
        snapshot.narrowPhaseTests += world.LastStepStats().narrowPhaseTestCount;
        snapshot.gjkCalls += world.LastStepStats().gjkCallCount;
        snapshot.epaCalls += world.LastStepStats().epaCallCount;
        snapshot.manifolds += world.LastStepStats().manifoldCount;
        snapshot.contactPoints += world.LastStepStats().contactPointCount;
    }

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

    // Pair/contact point/normal sets must be identical (stable PairKey order).
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
        }
        EXPECT_EQ(manifoldA.fixedStepNormalImpulse, manifoldB.fixedStepNormalImpulse);
    }

    // Event streams are committed on the main thread in stable order.
    ASSERT_EQ(a.events.size(), b.events.size());
    for (std::size_t index = 0; index < a.events.size(); ++index) {
        EXPECT_EQ(a.events[index].type, b.events[index].type);
        EXPECT_EQ(a.events[index].pair, b.events[index].pair);
        EXPECT_EQ(a.events[index].fixedStepId, b.events[index].fixedStepId);
        EXPECT_EQ(a.events[index].queryEpoch, b.events[index].queryEpoch);
    }

    // Task-private statistics merged after the barrier equal the serial totals.
    EXPECT_EQ(a.narrowPhaseTests, b.narrowPhaseTests);
    EXPECT_EQ(a.gjkCalls, b.gjkCalls);
    EXPECT_EQ(a.epaCalls, b.epaCalls);
    EXPECT_EQ(a.manifolds, b.manifolds);
    EXPECT_EQ(a.contactPoints, b.contactPoints);
}

// Builds a direct-detector scene: one static ground slab plus `boxCount`
// dynamic boxes resting on it, i.e. exactly boxCount candidate pairs.
void BuildDetectorScene(
    std::size_t boxCount,
    std::unordered_map<uint32_t, RigidBody>& bodies,
    std::unordered_map<uint32_t, Collider>& colliders) {
    uint32_t nextId = 1;
    RigidBodyDesc groundDesc;
    groundDesc.position = glm::vec3(0.0f, -0.5f, 0.0f);
    groundDesc.isStatic = true;
    groundDesc.useGravity = false;
    RigidBody ground(groundDesc);
    ground.SetId(nextId);
    bodies.emplace(nextId, ground);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(50.0f, 0.5f, 50.0f));
    Collider groundCol(groundCollider);
    groundCol.SetBodyId(nextId);
    groundCol.SetIdentity(nextId);
    colliders.emplace(nextId, std::move(groundCol));
    ++nextId;

    for (std::size_t index = 0; index < boxCount; ++index) {
        RigidBodyDesc desc;
        desc.position = glm::vec3(static_cast<float>(index) * 1.5f, 0.45f, 0.0f);
        desc.useGravity = false;
        RigidBody body(desc);
        body.SetId(nextId);
        bodies.emplace(nextId, body);
        ColliderDesc colliderDesc;
        colliderDesc.shape = std::make_shared<BoxShape>(glm::vec3(0.45f));
        Collider collider(colliderDesc);
        collider.SetBodyId(nextId);
        collider.SetIdentity(nextId);
        colliders.emplace(nextId, std::move(collider));
        ++nextId;
    }
}

} // namespace

TEST(PhysicsParallelNarrowphaseTest, MultiWorkerMatchesSingleWorkerBitExact) {
    auto serialWorld = MakeBoxFieldWorld(1, true);
    const WorldSnapshot serial = RunAndCapture(*serialWorld);

    auto parallelWorld = MakeBoxFieldWorld(4, true);
    const WorldSnapshot parallel = RunAndCapture(*parallelWorld);

    ASSERT_GT(serial.contactPoints, 0u);
    ExpectSnapshotsBitExact(serial, parallel);
}

TEST(PhysicsParallelNarrowphaseTest, DisabledToggleMatchesEnabledBitExact) {
    auto enabledWorld = MakeBoxFieldWorld(4, true);
    const WorldSnapshot enabled = RunAndCapture(*enabledWorld);

    auto disabledWorld = MakeBoxFieldWorld(4, false);
    const WorldSnapshot disabled = RunAndCapture(*disabledWorld);

    ASSERT_GT(enabled.contactPoints, 0u);
    ExpectSnapshotsBitExact(enabled, disabled);
}

TEST(PhysicsParallelNarrowphaseTest, JobTelemetryReflectsChunkingAndSerialFallback) {
    constexpr std::size_t kBoxes = 40;
    std::unordered_map<uint32_t, RigidBody> bodies;
    std::unordered_map<uint32_t, Collider> colliders;
    BuildDetectorScene(kBoxes, bodies, colliders);

    CollisionDetector detector;
    detector.SetNarrowphaseMinPairsPerJob(8);

    Runtime::Core::JobSystem::Get().Initialize(4);
    detector.Detect(colliders, bodies, 1, 1, kFixedDt, false);
    const NarrowPhaseParallelStats parallel = detector.LastParallelStats();
    // 40 pairs with at most 8 pairs per chunk must split into >= 5 jobs.
    EXPECT_GE(parallel.jobCount, 5u);
    EXPECT_GE(parallel.workerParticipation, 1u);
    EXPECT_EQ(parallel.jobCountPerThread.size(), parallel.workerParticipation);
    EXPECT_EQ(parallel.busyMillisecondsPerThread.size(), parallel.workerParticipation);
    const uint32_t summedJobs = std::accumulate(
        parallel.jobCountPerThread.begin(), parallel.jobCountPerThread.end(), 0u);
    EXPECT_EQ(summedJobs, parallel.jobCount);
    EXPECT_GT(parallel.workerBusyMilliseconds, 0.0);
    EXPECT_GE(parallel.tailWaitMilliseconds, 0.0);

    // workerCount == 1 is the fully serial deterministic fallback executed
    // in-line on the calling thread.
    Runtime::Core::JobSystem::Get().Initialize(1);
    detector.Detect(colliders, bodies, 1, 2, kFixedDt, false);
    const NarrowPhaseParallelStats serial = detector.LastParallelStats();
    EXPECT_EQ(serial.jobCount, 1u);
    EXPECT_EQ(serial.workerParticipation, 1u);
}

TEST(PhysicsParallelNarrowphaseTest, MergedStatsAndContactsMatchSerial) {
    constexpr std::size_t kBoxes = 40;
    std::unordered_map<uint32_t, RigidBody> bodies;
    std::unordered_map<uint32_t, Collider> colliders;
    BuildDetectorScene(kBoxes, bodies, colliders);

    CollisionDetector parallelDetector;
    parallelDetector.SetNarrowphaseMinPairsPerJob(8);
    Runtime::Core::JobSystem::Get().Initialize(4);
    const CollisionDetectionResult& parallelResult =
        parallelDetector.Detect(colliders, bodies, 1, 1, kFixedDt, false);
    const CollisionDetectionStats parallelStats = parallelDetector.LastStats();

    CollisionDetector serialDetector;
    Runtime::Core::JobSystem::Get().Initialize(1);
    const CollisionDetectionResult& serialResult =
        serialDetector.Detect(colliders, bodies, 1, 1, kFixedDt, false);
    const CollisionDetectionStats serialStats = serialDetector.LastStats();

    EXPECT_EQ(parallelStats.narrowPhaseTestCount, serialStats.narrowPhaseTestCount);
    EXPECT_EQ(parallelStats.gjkCallCount, serialStats.gjkCallCount);
    EXPECT_EQ(parallelStats.gjkFailureCount, serialStats.gjkFailureCount);
    EXPECT_EQ(parallelStats.epaCallCount, serialStats.epaCallCount);
    EXPECT_EQ(parallelStats.epaFailureCount, serialStats.epaFailureCount);
    EXPECT_EQ(parallelStats.contactPointCount, serialStats.contactPointCount);
    ASSERT_GT(serialStats.contactPointCount, 0u);

    // Touching sets and committed manifold geometry are identical in stable
    // PairKey (work list) order.
    ASSERT_EQ(parallelResult.touchingPairs.size(), serialResult.touchingPairs.size());
    for (std::size_t index = 0; index < parallelResult.touchingPairs.size(); ++index) {
        const auto& pairA = parallelDetector.GetMidphase().PairAt(parallelResult.touchingPairs[index]);
        const auto& pairB = serialDetector.GetMidphase().PairAt(serialResult.touchingPairs[index]);
        EXPECT_EQ(pairA.key, pairB.key);
        ExpectVec3BitExact(pairA.manifold.normal, pairB.manifold.normal);
        ASSERT_EQ(pairA.manifold.pointCount, pairB.manifold.pointCount);
        for (std::size_t point = 0; point < pairA.manifold.pointCount; ++point) {
            ExpectVec3BitExact(
                pairA.manifold.Point(point).position,
                pairB.manifold.Point(point).position);
        }
    }
}
