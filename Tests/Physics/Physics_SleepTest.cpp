// Phase 9 acceptance gates: island sleep and wake.
//
// - A stable resting stack finally sleeps: support impulses and position
//   correction must not reset the island rest timer.
// - Sleeping bodies stay in the dynamic BVH but skip integration, active
//   broad-phase queries, sleeping-inactive narrow-phase pairs and the solver;
//   their pairs and island adjacency are retained (a skipped query never
//   implies separation, so sleep publishes no Exit).
// - A confirmed awake-sleeping contact wakes the whole island inside the same
//   sub-step before solving (broad-phase proximity alone never wakes, so
//   nearby but non-touching bodies can all sleep); the woken bodies catch up
//   the missed force/gravity velocity increment exactly once.
// - External forces/impulses/explicit velocity and pose writes, collider
//   replacement and static support removal/modification all wake the
//   affected islands; woken islands never lose contacts or double-consume
//   forces.
// - Sleep judgment is island-wide; sleep results are bit-identical across
//   worker counts.
//
// These tests retain discrete-path coverage. Physics_StackPipelineTest also
// runs the default CCD path for sixty seconds, with sleep enabled and disabled.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <memory>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "Physics/CollisionShape.h"
#include "Physics/PhysicsWorld.h"

namespace {

using Runtime::Physics::BoxShape;
using Runtime::Physics::ColliderDesc;
using Runtime::Physics::ContactEventType;
using Runtime::Physics::MakePairKey;
using Runtime::Physics::PhysicsWorld;
using Runtime::Physics::RigidBodyDesc;
using Runtime::Physics::SleepState;
using Runtime::Physics::SphereShape;

constexpr float kDt = 1.0f / 60.0f;
// Default PhysicsSettings thresholds: 0.05 m/s, 0.05 rad/s, 0.5 s.
constexpr int kMinStepsToSleep = 29; // 0.5 s at 60 Hz, minus tolerance.

uint32_t AddStaticBox(PhysicsWorld& world, const glm::vec3& position, const glm::vec3& halfExtents) {
    RigidBodyDesc desc;
    desc.position = position;
    desc.isStatic = true;
    desc.useGravity = false;
    const uint32_t id = world.CreateRigidBody(desc);
    ColliderDesc collider;
    collider.shape = std::make_shared<BoxShape>(halfExtents);
    collider.material.restitution = 0.0f;
    EXPECT_TRUE(world.AttachCollider(id, collider));
    return id;
}

uint32_t AddDynamicBox(
    PhysicsWorld& world,
    const glm::vec3& position,
    float halfExtent = 0.5f,
    bool useGravity = true) {
    RigidBodyDesc desc;
    desc.position = position;
    desc.useGravity = useGravity;
    const uint32_t id = world.CreateRigidBody(desc);
    ColliderDesc collider;
    collider.shape = std::make_shared<BoxShape>(glm::vec3(halfExtent));
    collider.material.restitution = 0.0f;
    EXPECT_TRUE(world.AttachCollider(id, collider));
    return id;
}

// Step until every dynamic body sleeps; returns the 0-based step index of the
// first fully sleeping step, or -1 when maxSteps ran out.
int StepUntilAllSleep(PhysicsWorld& world, int maxSteps) {
    for (int step = 0; step < maxSteps; ++step) {
        world.Step(kDt);
        bool allSleeping = true;
        bool anyDynamic = false;
        for (const auto& entry : world.Bodies()) {
            if (entry.second.IsStatic()) {
                continue;
            }
            anyDynamic = true;
            if (!entry.second.IsSleeping()) {
                allSleeping = false;
                break;
            }
        }
        if (anyDynamic && allSleeping) {
            return step;
        }
    }
    return -1;
}

bool AnyEventOfType(const PhysicsWorld& world, ContactEventType type) {
    for (const auto& event : world.ContactEvents()) {
        if (event.type == type) {
            return true;
        }
    }
    return false;
}

// Standard scene: static ground (top at y = 0.5) plus a dynamic box dropped
// from the given height. Discrete sleep acceptance is complemented by CCD stack tests.
void BuildGround(PhysicsWorld& world) {
    AddStaticBox(world, glm::vec3(0.0f), glm::vec3(10.0f, 0.5f, 10.0f));
}

TEST(SleepTest, RestingBoxEventuallySleepsAndStaysStable) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    BuildGround(world);
    const uint32_t boxId = AddDynamicBox(world, glm::vec3(0.0f, 1.2f, 0.0f));

    float maxPenetration = 0.0f;
    int sleepStep = -1;
    for (int step = 0; step < 300 && sleepStep < 0; ++step) {
        world.Step(kDt);
        for (const auto& contact : world.Contacts()) {
            for (std::size_t index = 0; index < contact.pointCount; ++index) {
                maxPenetration = std::max(maxPenetration, contact.Point(index).penetration);
            }
        }
        const auto* box = world.GetRigidBody(boxId);
        if (box->IsSleeping()) {
            sleepStep = step;
        }
    }

    // Sleep timing record: rest accumulation starts only after the landing
    // Enter settled, so total time stays above the 0.5 s threshold.
    ASSERT_GE(sleepStep, 0) << "box never fell asleep";
    EXPECT_GE(sleepStep + 1, kMinStepsToSleep);
    EXPECT_LT(maxPenetration, 0.05f) << "excessive penetration before sleep";

    const auto* box = world.GetRigidBody(boxId);
    // EnterSleep zeroes the residual motion state exactly.
    EXPECT_EQ(box->LinearVelocity(), glm::vec3(0.0f));
    EXPECT_EQ(box->AngularVelocity(), glm::vec3(0.0f));
    EXPECT_EQ(box->AngularMomentum(), glm::vec3(0.0f));

    // Sleeping body skips integration, queries, narrow-phase and solver; the
    // pair is retained without being re-registered.
    const glm::vec3 sleptPosition = box->Position();
    for (int step = 0; step < 30; ++step) {
        world.Step(kDt);
        const auto& stats = world.LastStepStats();
        EXPECT_EQ(stats.sleepingBodyCount, 1u);
        EXPECT_EQ(stats.awakeBodyCount, 0u);
        EXPECT_EQ(stats.islandCount, 0u);
        EXPECT_EQ(stats.broadPhaseCandidateCount, 0u);
        EXPECT_EQ(stats.narrowPhaseTestCount, 0u);
        EXPECT_EQ(stats.velocitySolverPassCount, 0u);
        EXPECT_EQ(stats.midphaseActivePairCount, 1u);
        EXPECT_EQ(stats.midphaseNewPairCount, 0u);
        EXPECT_EQ(stats.midphaseRemovedPairCount, 0u);
    }
    EXPECT_EQ(world.GetRigidBody(boxId)->Position(), sleptPosition);
}

TEST(SleepTest, SleepingPairPublishesNoEventsAndIsRetainedUnqueried) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    BuildGround(world);
    const uint32_t boxId = AddDynamicBox(world, glm::vec3(0.0f, 1.2f, 0.0f));
    ASSERT_GE(StepUntilAllSleep(world, 300), 0);

    // No Enter/Stay/Exit while asleep (sleep itself never publishes Exit),
    // and the fixed-step contact summary no longer lists the pair because it
    // was not evaluated this step.
    for (int step = 0; step < 30; ++step) {
        world.Step(kDt);
        EXPECT_TRUE(world.ContactEvents().empty());
        EXPECT_TRUE(world.Contacts().empty());
        EXPECT_EQ(world.LastStepStats().midphaseActivePairCount, 1u);
    }

    // A gentle sideways nudge wakes the island; the retained pair keeps its
    // touching state, so the contact persists without a new Enter.
    world.GetRigidBody(boxId)->ApplyLinearImpulse(glm::vec3(0.2f, 0.0f, 0.0f));
    world.Step(kDt);
    EXPECT_FALSE(world.GetRigidBody(boxId)->IsSleeping());
    EXPECT_EQ(world.Contacts().size(), 1u);
    EXPECT_FALSE(AnyEventOfType(world, ContactEventType::Enter));
    EXPECT_FALSE(AnyEventOfType(world, ContactEventType::Exit));
}

TEST(SleepTest, ExternalImpulseWakesWholeIsland) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    BuildGround(world);
    const uint32_t bottom = AddDynamicBox(world, glm::vec3(0.0f, 1.05f, 0.0f));
    const uint32_t middle = AddDynamicBox(world, glm::vec3(0.0f, 2.1f, 0.0f));
    const uint32_t top = AddDynamicBox(world, glm::vec3(0.0f, 3.15f, 0.0f));
    ASSERT_GE(StepUntilAllSleep(world, 300), 0);

    // Poking one member wakes the whole island (the island slept atomically).
    world.GetRigidBody(middle)->ApplyLinearImpulse(glm::vec3(0.0f, 1.0f, 0.0f));
    world.Step(kDt);
    EXPECT_FALSE(world.GetRigidBody(bottom)->IsSleeping());
    EXPECT_FALSE(world.GetRigidBody(middle)->IsSleeping());
    EXPECT_FALSE(world.GetRigidBody(top)->IsSleeping());
    EXPECT_GE(world.LastStepStats().islandCount, 1u);
}

TEST(SleepTest, ExternalForceConsumedExactlyOnceAfterWake) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f)); // zero-g: a lone floating body can rest
    world.SetContinuousCollisionEnabled(false);
    const uint32_t boxId = AddDynamicBox(world, glm::vec3(0.0f), 0.5f, /*useGravity=*/false);
    ASSERT_GE(StepUntilAllSleep(world, 300), 0);

    // The force is locked at fixed-step start and integrated exactly once
    // (not lost by sleep, not duplicated by the wake compensation).
    world.GetRigidBody(boxId)->ApplyForce(glm::vec3(60.0f, 0.0f, 0.0f));
    world.Step(kDt);
    const auto* box = world.GetRigidBody(boxId);
    EXPECT_FALSE(box->IsSleeping());
    EXPECT_NEAR(box->LinearVelocity().x, 60.0f * kDt, 1e-5f);

    world.Step(kDt);
    EXPECT_NEAR(world.GetRigidBody(boxId)->LinearVelocity().x, 60.0f * kDt, 1e-5f);
}

TEST(SleepTest, BroadPhaseProximityAloneDoesNotWakeSleeper) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    BuildGround(world);
    const uint32_t sleeperId = AddDynamicBox(world, glm::vec3(0.0f, 1.2f, 0.0f));
    ASSERT_GE(StepUntilAllSleep(world, 300), 0);

    // A zero-g box hovering inside the sleeper's fat AABB (fat margin 0.08
    // per side, tight gap 0.1) without ever touching it: the broad-phase hit
    // alone must NOT wake the sleeper, otherwise two nearby but non-touching
    // bodies would keep waking each other and never both sleep.
    const uint32_t hoverId = AddDynamicBox(world, glm::vec3(0.0f, 2.1f, 0.0f), 0.5f, /*useGravity=*/false);
    int stepsBothAwakeOverlapEvaluated = 0;
    bool sleeperWoke = false;
    for (int step = 0; step < 120; ++step) {
        world.Step(kDt);
        if (!world.GetRigidBody(hoverId)->IsSleeping()) {
            // While the hover box is awake, the mixed pair is still evaluated
            // by the narrow-phase (read-only frozen geometry) and separated.
            EXPECT_GE(world.LastStepStats().narrowPhaseTestCount, 1u);
            ++stepsBothAwakeOverlapEvaluated;
        }
        if (!world.GetRigidBody(sleeperId)->IsSleeping()) {
            sleeperWoke = true;
        }
    }
    EXPECT_FALSE(sleeperWoke) << "mere fat-AABB proximity woke the sleeper";
    EXPECT_GT(stepsBothAwakeOverlapEvaluated, 0);
    // Eventually both sleep at once and the pair stops being queried.
    EXPECT_TRUE(world.GetRigidBody(sleeperId)->IsSleeping());
    EXPECT_TRUE(world.GetRigidBody(hoverId)->IsSleeping());
    world.Step(kDt);
    EXPECT_EQ(world.LastStepStats().broadPhaseCandidateCount, 0u);
    EXPECT_EQ(world.LastStepStats().narrowPhaseTestCount, 0u);
}

TEST(SleepTest, ConfirmedContactWakesSleepingIslandInSameSubstep) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    BuildGround(world);
    const uint32_t bottom = AddDynamicBox(world, glm::vec3(0.0f, 1.05f, 0.0f));
    const uint32_t top = AddDynamicBox(world, glm::vec3(0.0f, 2.1f, 0.0f));
    ASSERT_GE(StepUntilAllSleep(world, 300), 0);

    // Drop a box from just above the sleeping stack: the confirmed contact
    // wakes the whole island in the very sub-step that detects it, so the
    // island's own contacts already participate in that sub-step's solve.
    const uint32_t dropperId = AddDynamicBox(world, glm::vec3(0.0f, 3.2f, 0.0f));
    bool sawEnter = false;
    for (int step = 0; step < 60 && !sawEnter; ++step) {
        world.Step(kDt);
        if (AnyEventOfType(world, ContactEventType::Enter)) {
            sawEnter = true;
            // Wake completed before this sub-step's solve: the whole island
            // is awake and the impact produced no deep penetration.
            EXPECT_FALSE(world.GetRigidBody(bottom)->IsSleeping());
            EXPECT_FALSE(world.GetRigidBody(top)->IsSleeping());
            for (const auto& contact : world.Contacts()) {
                for (std::size_t index = 0; index < contact.pointCount; ++index) {
                    EXPECT_LT(contact.Point(index).penetration, 0.05f);
                }
            }
        }
    }
    ASSERT_TRUE(sawEnter) << "dropped box never touched the sleeping stack";

    // The merged island settles and sleeps again.
    (void)dropperId;
    EXPECT_GE(StepUntilAllSleep(world, 400), 0);
}

TEST(SleepTest, ExplicitPoseWriteWakesIsland) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    BuildGround(world);
    const uint32_t bottom = AddDynamicBox(world, glm::vec3(0.0f, 1.05f, 0.0f));
    const uint32_t top = AddDynamicBox(world, glm::vec3(0.0f, 2.1f, 0.0f));
    ASSERT_GE(StepUntilAllSleep(world, 300), 0);

    world.GetRigidBody(bottom)->SetPosition(glm::vec3(0.0f, 1.0f, 0.0f));
    world.Step(kDt);
    EXPECT_FALSE(world.GetRigidBody(bottom)->IsSleeping());
    EXPECT_FALSE(world.GetRigidBody(top)->IsSleeping());
}

TEST(SleepTest, ColliderReplacementWakesIsland) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    BuildGround(world);
    const uint32_t boxId = AddDynamicBox(world, glm::vec3(0.0f, 1.2f, 0.0f));
    ASSERT_GE(StepUntilAllSleep(world, 300), 0);

    ColliderDesc replacement;
    replacement.shape = std::make_shared<BoxShape>(glm::vec3(0.5f));
    ASSERT_TRUE(world.AttachCollider(boxId, replacement));
    world.Step(kDt);
    EXPECT_FALSE(world.GetRigidBody(boxId)->IsSleeping());
}

TEST(SleepTest, RemovingGroundWakesSupportedIslandAndPublishesExit) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    BuildGround(world);
    const uint32_t groundId = 1; // first body created by BuildGround
    const uint32_t boxId = AddDynamicBox(world, glm::vec3(0.0f, 1.2f, 0.0f));
    ASSERT_GE(StepUntilAllSleep(world, 300), 0);
    const float sleptY = world.GetRigidBody(boxId)->Position().y;

    ASSERT_TRUE(world.DestroyRigidBody(groundId));
    world.Step(kDt);
    EXPECT_FALSE(world.GetRigidBody(boxId)->IsSleeping());
    EXPECT_TRUE(AnyEventOfType(world, ContactEventType::Exit));

    // Unsupported now: the box starts falling within a few steps.
    for (int step = 0; step < 10; ++step) {
        world.Step(kDt);
    }
    EXPECT_LT(world.GetRigidBody(boxId)->Position().y, sleptY);
}

TEST(SleepTest, MovingGroundAwayWakesSupportedIsland) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    BuildGround(world);
    const uint32_t groundId = 1;
    const uint32_t boxId = AddDynamicBox(world, glm::vec3(0.0f, 1.2f, 0.0f));
    ASSERT_GE(StepUntilAllSleep(world, 300), 0);
    const float sleptY = world.GetRigidBody(boxId)->Position().y;

    world.GetRigidBody(groundId)->SetPosition(glm::vec3(0.0f, -5.0f, 0.0f));
    world.Step(kDt);
    EXPECT_FALSE(world.GetRigidBody(boxId)->IsSleeping());
    for (int step = 0; step < 10; ++step) {
        world.Step(kDt);
    }
    EXPECT_LT(world.GetRigidBody(boxId)->Position().y, sleptY);
}

TEST(SleepTest, StaticMovedIntoSleeperWakesItViaRegionQuery) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    BuildGround(world);
    const uint32_t boxId = AddDynamicBox(world, glm::vec3(0.0f, 1.2f, 0.0f));
    // Far-away static wall: no pair with the box ever existed.
    const uint32_t wallId = AddStaticBox(world, glm::vec3(100.0f, 1.0f, 0.0f), glm::vec3(0.5f, 2.0f, 2.0f));
    ASSERT_GE(StepUntilAllSleep(world, 300), 0);

    // The wall teleports into the sleeping box. There is no pair to scan, so
    // only the dynamic-BVH region query of the static modification finds it.
    world.GetRigidBody(wallId)->SetPosition(glm::vec3(0.0f, 1.0f, 0.0f));
    world.Step(kDt);
    EXPECT_FALSE(world.GetRigidBody(boxId)->IsSleeping());
}

TEST(SleepTest, HighSpeedHitWakesIslandBeforeSolve) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    BuildGround(world);
    const uint32_t bottom = AddDynamicBox(world, glm::vec3(0.0f, 1.05f, 0.0f));
    const uint32_t top = AddDynamicBox(world, glm::vec3(0.0f, 2.1f, 0.0f));
    ASSERT_GE(StepUntilAllSleep(world, 300), 0);

    // Enable CCD and fire a fast sphere horizontally at the sleeping stack.
    world.SetContinuousCollisionEnabled(true);
    world.SetCcdMaxSubSteps(12);
    RigidBodyDesc sphereDesc;
    sphereDesc.position = {-5.0f, 1.5f, 0.0f};
    sphereDesc.linearVelocity = {150.0f, 0.0f, 0.0f};
    sphereDesc.useGravity = false;
    const uint32_t sphereId = world.CreateRigidBody(sphereDesc);
    ColliderDesc sphereCollider;
    sphereCollider.shape = std::make_shared<SphereShape>(0.25f);
    sphereCollider.material.restitution = 0.0f;
    ASSERT_TRUE(world.AttachCollider(sphereId, sphereCollider));

    // The first step that wakes any stack body must wake the whole island:
    // the TOI sub-step's detection runs the wake closure before solving.
    bool woke = false;
    for (int step = 0; step < 30 && !woke; ++step) {
        world.Step(kDt);
        if (!world.GetRigidBody(bottom)->IsSleeping() || !world.GetRigidBody(top)->IsSleeping()) {
            woke = true;
            EXPECT_FALSE(world.GetRigidBody(bottom)->IsSleeping());
            EXPECT_FALSE(world.GetRigidBody(top)->IsSleeping());
        }
    }
    ASSERT_TRUE(woke);

    // No tunneling means the impact interacted: the sphere lost speed and
    // the stack gained momentum instead of being passed through.
    for (int step = 0; step < 5; ++step) {
        world.Step(kDt);
    }
    const float sphereSpeed = glm::length(world.GetRigidBody(sphereId)->LinearVelocity());
    EXPECT_LT(sphereSpeed, 140.0f);
    bool stackMoved = false;
    for (const uint32_t id : {bottom, top}) {
        const auto* body = world.GetRigidBody(id);
        if (std::abs(body->Position().x) > 0.05f || body->LinearVelocity().x > 0.1f) {
            stackMoved = true;
        }
    }
    EXPECT_TRUE(stackMoved) << "sphere passed through without moving the stack";
}

TEST(SleepTest, IslandAdjacencyRetainedAcrossSleep) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    BuildGround(world);
    const uint32_t bottom = AddDynamicBox(world, glm::vec3(0.0f, 1.05f, 0.0f));
    const uint32_t top = AddDynamicBox(world, glm::vec3(0.0f, 2.1f, 0.0f));

    const auto islandOf = [&](uint32_t bodyId) -> int {
        const auto& islands = world.LastIslands();
        for (std::size_t index = 0; index < islands.size(); ++index) {
            for (const uint32_t member : islands[index].bodies) {
                if (member == bodyId) {
                    return static_cast<int>(index);
                }
            }
        }
        return -1;
    };

    world.Step(kDt);
    // Same island before sleep...
    for (int step = 0; step < 10; ++step) {
        world.Step(kDt);
    }
    const int beforeBottom = islandOf(bottom);
    const int beforeTop = islandOf(top);
    ASSERT_GE(beforeBottom, 0);
    EXPECT_EQ(beforeBottom, beforeTop);

    ASSERT_GE(StepUntilAllSleep(world, 300), 0);
    // Sleeping islands keep no solver island but retain their midphase
    // adjacency; a wake rebuilds the same membership.
    world.GetRigidBody(bottom)->ApplyLinearImpulse(glm::vec3(0.2f, 0.0f, 0.0f));
    world.Step(kDt);
    const int afterBottom = islandOf(bottom);
    const int afterTop = islandOf(top);
    ASSERT_GE(afterBottom, 0);
    EXPECT_EQ(afterBottom, afterTop);
}

TEST(SleepTest, EnterContactResetsIslandSleepTimer) {
    PhysicsWorld world;
    // Zero-g isolates the contact-stability rule from the speed thresholds:
    // every body stays below the limits for the whole test.
    world.SetGravity(glm::vec3(0.0f));
    world.SetContinuousCollisionEnabled(false);
    BuildGround(world);
    // Box A rests exactly on the ground (top at y = 0.5, half extent 0.5).
    const uint32_t boxA = AddDynamicBox(world, glm::vec3(0.0f, 1.0f, 0.0f), 0.5f, false);

    // Let A accumulate some rest time but not yet sleep.
    int steps = 0;
    while (steps < 300) {
        world.Step(kDt);
        ++steps;
        const auto* body = world.GetRigidBody(boxA);
        if (body->GetSleepState() == SleepState::Candidate && body->SleepTimer() >= 0.15f) {
            break;
        }
    }
    ASSERT_LT(steps, 300);
    ASSERT_EQ(world.GetRigidBody(boxA)->GetSleepState(), SleepState::Candidate);

    // Box B drifts down slowly (below the speed threshold) and touches A:
    // the Enter event resets the island-wide timer even though every body
    // stays under the speed limits. Gap and speed are chosen so B touches A
    // within 0.25 s, before either body can reach the 0.5 s sleep threshold.
    const uint32_t boxB = AddDynamicBox(world, glm::vec3(0.0f, 2.01f, 0.0f), 0.5f, false);
    world.GetRigidBody(boxB)->SetLinearVelocity(glm::vec3(0.0f, -0.04f, 0.0f));
    bool sawReset = false;
    for (int step = 0; step < 60; ++step) {
        world.Step(kDt);
        if (AnyEventOfType(world, ContactEventType::Enter)) {
            EXPECT_EQ(world.GetRigidBody(boxA)->SleepTimer(), 0.0f);
            EXPECT_EQ(world.GetRigidBody(boxB)->SleepTimer(), 0.0f);
            EXPECT_FALSE(world.GetRigidBody(boxA)->IsSleeping());
            sawReset = true;
            break;
        }
    }
    ASSERT_TRUE(sawReset) << "slow box never touched the candidate box";

    // After the contact stabilizes the merged island sleeps normally.
    EXPECT_GE(StepUntilAllSleep(world, 300), 0);
}

TEST(SleepTest, AllowSleepFalseKeepsBodyAwakeAndReEnableWakes) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    BuildGround(world);
    const uint32_t noSleepBox = AddDynamicBox(world, glm::vec3(-3.0f, 1.2f, 0.0f));
    const uint32_t normalBox = AddDynamicBox(world, glm::vec3(3.0f, 1.2f, 0.0f));
    world.GetRigidBody(noSleepBox)->SetAllowSleep(false);

    for (int step = 0; step < 150; ++step) {
        world.Step(kDt);
    }
    // The allowSleep=false member blocks nothing but itself; the other box
    // sleeps while this one keeps simulating (its pair keeps re-checking).
    EXPECT_TRUE(world.GetRigidBody(normalBox)->IsSleeping());
    EXPECT_FALSE(world.GetRigidBody(noSleepBox)->IsSleeping());
    EXPECT_GT(world.LastStepStats().narrowPhaseTestCount, 0u);

    // Toggling the policy on a sleeping body wakes it at the next step.
    world.GetRigidBody(normalBox)->SetAllowSleep(false);
    world.Step(kDt);
    EXPECT_FALSE(world.GetRigidBody(normalBox)->IsSleeping());
}

TEST(SleepTest, DisablingSleepWakesEverythingImmediately) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    BuildGround(world);
    const uint32_t boxId = AddDynamicBox(world, glm::vec3(0.0f, 1.2f, 0.0f));
    ASSERT_GE(StepUntilAllSleep(world, 300), 0);

    world.SetSleepEnabled(false);
    // The toggle itself wakes immediately, before the next step.
    EXPECT_FALSE(world.GetRigidBody(boxId)->IsSleeping());
    world.Step(kDt);
    EXPECT_GT(world.LastStepStats().narrowPhaseTestCount, 0u);
    EXPECT_EQ(world.Contacts().size(), 1u);
}

TEST(SleepTest, SleepResultsBitIdenticalAcrossWorkerCounts) {
    const auto runScene = [](uint32_t workers) {
        PhysicsWorld world;
        world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
        world.SetContinuousCollisionEnabled(false);
        world.SetPhysicsWorkerCount(workers);
        BuildGround(world);
        // Three independent stacks plus one falling box that joins late.
        for (int stack = 0; stack < 3; ++stack) {
            const float x = -6.0f + 6.0f * static_cast<float>(stack);
            AddDynamicBox(world, glm::vec3(x, 1.05f, 0.0f));
            AddDynamicBox(world, glm::vec3(x, 2.1f, 0.0f));
            AddDynamicBox(world, glm::vec3(x, 3.15f, 0.0f));
        }
        AddDynamicBox(world, glm::vec3(0.0f, 8.0f, 0.0f));
        for (int step = 0; step < 200; ++step) {
            world.Step(kDt);
        }
        // Sort by body id: container iteration order must not leak into the
        // determinism comparison.
        std::vector<std::pair<uint32_t, glm::vec3>> snapshot;
        for (const auto& entry : world.Bodies()) {
            if (entry.second.IsStatic()) {
                continue;
            }
            snapshot.emplace_back(entry.first * 3u + 0u, entry.second.Position());
            snapshot.emplace_back(entry.first * 3u + 1u, entry.second.LinearVelocity());
            snapshot.emplace_back(
                entry.first * 3u + 2u,
                entry.second.IsSleeping() ? glm::vec3(1.0f) : glm::vec3(0.0f));
        }
        std::sort(snapshot.begin(), snapshot.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });
        return snapshot;
    };

    const auto serial = runScene(1);
    const auto parallel = runScene(4);
    ASSERT_EQ(serial.size(), parallel.size());
    for (std::size_t index = 0; index < serial.size(); ++index) {
        ASSERT_EQ(serial[index].first, parallel[index].first);
        EXPECT_EQ(serial[index].second, parallel[index].second) << "state divergence at slot " << index;
    }
}

// Regression test for the four-supports explosion: boxes landing gently
// (impact speed * dt inside the speculative band) used to get a single-point
// speculative manifold because the box support feature degenerated under
// the GJK distance normal noise; the midphase then reused that one-point
// manifold forever (translation < band), leaving no anti-tilt support and
// letting the wedge contacts with the neighboring supports blow the scene
// up. The box now answers a full dominant face for every query direction.
TEST(SleepTest, FourSupportsLandOnSpeculativeBandAndSleep) {
    constexpr float kDt120 = 1.0f / 120.0f;
    constexpr float kHalf = 0.45f;
    PhysicsWorld world;
    world.SetFixedTimeStep(kDt120);
    world.SetGravity(glm::vec3(0.0f, -10.5f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    world.SetSolverIterations(10);
    {
        RigidBodyDesc groundDesc;
        groundDesc.position = glm::vec3(0.0f, -0.5f, 0.0f);
        groundDesc.isStatic = true;
        groundDesc.useGravity = false;
        const uint32_t groundId = world.CreateRigidBody(groundDesc);
        ColliderDesc groundCollider;
        groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
        groundCollider.material.restitution = 0.1f;
        groundCollider.material.dynamicFriction = 0.6f;
        groundCollider.material.staticFriction = 0.7f;
        world.AttachCollider(groundId, groundCollider);
    }
    const auto addBox = [&](const glm::vec3& position) {
        RigidBodyDesc desc;
        desc.position = position;
        desc.mass = 1.0f;
        desc.inertiaTensorDiagonal = glm::vec3(0.135f);
        const uint32_t id = world.CreateRigidBody(desc);
        ColliderDesc collider;
        collider.shape = std::make_shared<BoxShape>(glm::vec3(kHalf));
        collider.material.restitution = 0.05f;
        collider.material.dynamicFriction = 1.0f;
        collider.material.staticFriction = 1.0f;
        world.AttachCollider(id, collider);
        return id;
    };
    constexpr float supportOffset = kHalf + 0.01f;
    uint32_t ids[4];
    int n = 0;
    for (const float x : {-supportOffset, supportOffset}) {
        for (const float z : {-supportOffset, supportOffset}) {
            ids[n++] = addBox(glm::vec3(x, kHalf + 0.05f, z));
        }
    }
    for (int step = 0; step < 330; ++step) {
        // Drive like the sample's render loop (165 fps frame dt through the
        // world's fixed-step accumulator).
        world.Step(1.0f / 165.0f);
    }
    for (const uint32_t id : ids) {
        const auto* body = world.GetRigidBody(id);
        EXPECT_LT(glm::length(body->LinearVelocity()), 0.05f);
        EXPECT_LT(glm::length(body->AngularVelocity()), 0.05f);
        EXPECT_GT(body->Position().y, 0.4f);
        EXPECT_TRUE(body->IsSleeping());
    }
}

// After spawning stops, a settled pile must come to rest and sleep. This is
// the "big island never sleeps" complaint: while boxes keep raining, the
// island's Enter events legitimately reset it, but once spawning stops the
// pile must settle and the whole island must sleep.
TEST(SleepTest, StoppedFieldRainPileSleeps) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetGravity(glm::vec3(0.0f, -10.5f, 0.0f));
    world.SetContinuousCollisionEnabled(false);
    world.SetSolverIterations(10);

    RigidBodyDesc groundDesc;
    groundDesc.position = glm::vec3(0.0f, -0.5f, 0.0f);
    groundDesc.isStatic = true;
    groundDesc.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(groundDesc);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
    groundCollider.material.restitution = 0.1f;
    groundCollider.material.dynamicFriction = 0.6f;
    groundCollider.material.staticFriction = 0.7f;
    world.AttachCollider(groundId, groundCollider);

    // Drop 60 boxes from moderate height in a tight cluster so they form one
    // big connected island, then STOP spawning and let it settle.
    const auto boxShape = std::make_shared<BoxShape>(glm::vec3(0.45f));
    std::mt19937 rng(1337);
    std::uniform_real_distribution<float> posDist(-1.5f, 1.5f);
    std::vector<uint32_t> bodies;
    float spawnAccum = 0.0f;
    for (int step = 0; step < 2400; ++step) {
        const float dt = 1.0f / 120.0f;
        if (bodies.size() < 60) {
            spawnAccum += dt * 30.0f;
            while (spawnAccum >= 1.0f) {
                spawnAccum -= 1.0f;
                RigidBodyDesc desc;
                desc.position = {posDist(rng), 4.0f, posDist(rng)};
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
        }
        world.Step(dt);
    }

    std::size_t sleeping = 0;
    float maxSpeed = 0.0f;
    float maxPen = 0.0f;
    for (const uint32_t id : bodies) {
        const auto* b = world.GetRigidBody(id);
        if (b->IsSleeping()) ++sleeping;
        maxSpeed = std::max(maxSpeed, glm::length(b->LinearVelocity()));
    }
    for (const auto& c : world.Contacts()) {
        for (std::size_t p = 0; p < c.pointCount; ++p) {
            maxPen = std::max(maxPen, c.Point(p).penetration);
        }
    }
    std::printf("[stopped-pile] sleeping=%zu/%zu maxSpeed=%.4f maxPen=%.5f\n",
        sleeping, bodies.size(), maxSpeed, maxPen);
    // The pile must come to rest and (nearly) all of it must sleep.
    EXPECT_LT(maxSpeed, 0.2f);
    EXPECT_GT(sleeping, bodies.size() * 3 / 4);
}

} // namespace
