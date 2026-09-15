#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include <glm/glm.hpp>

#include "Physics/CollisionDetector.h"
#include "Physics/CollisionShape.h"
#include "Physics/Midphase.h"
#include "Physics/PhysicsWorld.h"

namespace {

using Runtime::Physics::BroadPhaseQueryCoverage;
using Runtime::Physics::BoxShape;
using Runtime::Physics::Collider;
using Runtime::Physics::ColliderDesc;
using Runtime::Physics::CollisionDetectionResult;
using Runtime::Physics::CollisionDetector;
using Runtime::Physics::ContactEvent;
using Runtime::Physics::ContactEventType;
using Runtime::Physics::ContactManifold;
using Runtime::Physics::ContactPoint;
using Runtime::Physics::MakePairKey;
using Runtime::Physics::Midphase;
using Runtime::Physics::MidphasePair;
using Runtime::Physics::PairKey;
using Runtime::Physics::PairLifecycleState;
using Runtime::Physics::PhysicsWorld;
using Runtime::Physics::RigidBody;
using Runtime::Physics::RigidBodyDesc;
using Runtime::Physics::SphereShape;

constexpr float kFixedDt = 1.0f / 60.0f;

// Minimal body/collider scene driving the detector or the midphase directly.
struct DetectorScene {
    std::unordered_map<uint32_t, RigidBody> bodies;
    std::unordered_map<uint32_t, Collider> colliders;
    uint32_t nextId = 1;
    uint32_t nextIdentity = 1;

    uint32_t AddBox(
        const glm::vec3& position,
        const glm::vec3& halfExtents,
        bool isStatic,
        bool isTrigger = false) {
        RigidBodyDesc desc;
        desc.position = position;
        desc.isStatic = isStatic;
        desc.useGravity = false;
        const uint32_t id = nextId++;
        RigidBody body(desc);
        body.SetId(id);
        bodies.emplace(id, body);

        ColliderDesc colliderDesc;
        colliderDesc.shape = std::make_shared<BoxShape>(halfExtents);
        colliderDesc.isTrigger = isTrigger;
        Collider collider(colliderDesc);
        collider.SetBodyId(id);
        collider.SetIdentity(nextIdentity++);
        colliders.emplace(id, std::move(collider));
        return id;
    }

    // Attach a replacement collider with a fresh identity, like the world does.
    void ReplaceCollider(uint32_t id, const glm::vec3& halfExtents) {
        ColliderDesc colliderDesc;
        colliderDesc.shape = std::make_shared<BoxShape>(halfExtents);
        Collider collider(colliderDesc);
        collider.SetBodyId(id);
        collider.SetIdentity(nextIdentity++);
        colliders.at(id) = std::move(collider);
    }
};

std::size_t CountEvents(const std::vector<ContactEvent>& events, ContactEventType type) {
    std::size_t count = 0;
    for (const ContactEvent& event : events) {
        if (event.type == type) {
            ++count;
        }
    }
    return count;
}

void SeedContactCaches(MidphasePair& pair, float normalImpulse) {
    for (std::size_t index = 0; index < pair.manifold.pointCount; ++index) {
        ContactPoint& point = pair.manifold.Point(index);
        point.accumulatedNormalImpulse = normalImpulse;
        point.accumulatedTangentImpulse = glm::vec2(0.5f, -0.25f);
        // Interpretation data the cache is expressed in (Phase 4: it travels
        // with the cache through matching until the solver re-stamps it).
        point.cachedTangent1 = glm::vec3(1.0f, 0.0f, 0.0f);
        point.cachedTangent2 = glm::vec3(0.0f, 0.0f, 1.0f);
        point.cachedDt = kFixedDt;
    }
}

void ExpectCachesCleared(const MidphasePair& pair) {
    for (std::size_t index = 0; index < pair.manifold.pointCount; ++index) {
        const ContactPoint& point = pair.manifold.Point(index);
        EXPECT_FLOAT_EQ(point.accumulatedNormalImpulse, 0.0f);
        EXPECT_FLOAT_EQ(point.accumulatedTangentImpulse.x, 0.0f);
        EXPECT_FLOAT_EQ(point.accumulatedTangentImpulse.y, 0.0f);
    }
}

} // namespace

// ---------------------------------------------------------------------------
// Persistent point matching (plan step 8)
// ---------------------------------------------------------------------------

TEST(ContactManifoldMatchTest, TransfersImpulsesByAnchorsWhenPointsReordered) {
    ContactManifold oldManifold;
    oldManifold.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    for (int index = 0; index < 3; ++index) {
        ContactPoint point;
        point.localPointA = glm::vec3(0.2f * static_cast<float>(index), 0.0f, 0.0f);
        point.localPointB = glm::vec3(0.1f + 0.2f * static_cast<float>(index), 0.0f, 0.0f);
        point.accumulatedNormalImpulse = 1.0f + static_cast<float>(index);
        point.accumulatedTangentImpulse = glm::vec2(static_cast<float>(index), 0.0f);
        ASSERT_TRUE(oldManifold.AddPoint(point));
    }

    // Same anchors, reversed point order: impulses must follow the anchors,
    // never the array position.
    ContactManifold newManifold;
    newManifold.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    for (int index = 2; index >= 0; --index) {
        ContactPoint point;
        point.localPointA = glm::vec3(0.2f * static_cast<float>(index), 0.0f, 0.0f);
        point.localPointB = glm::vec3(0.1f + 0.2f * static_cast<float>(index), 0.0f, 0.0f);
        ASSERT_TRUE(newManifold.AddPoint(point));
    }

    Runtime::Physics::MatchPersistentContactPoints(oldManifold, newManifold);

    EXPECT_FLOAT_EQ(newManifold.Point(0).accumulatedNormalImpulse, 3.0f);
    EXPECT_FLOAT_EQ(newManifold.Point(1).accumulatedNormalImpulse, 2.0f);
    EXPECT_FLOAT_EQ(newManifold.Point(2).accumulatedNormalImpulse, 1.0f);
    EXPECT_FLOAT_EQ(newManifold.Point(0).accumulatedTangentImpulse.x, 2.0f);
}

TEST(ContactManifoldMatchTest, ExactDistanceTieResolvesToLowestOldIndex) {
    ContactManifold oldManifold;
    oldManifold.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    for (int index = 0; index < 2; ++index) {
        ContactPoint point;
        point.localPointA = glm::vec3(0.0f);
        point.localPointB = glm::vec3(0.0f);
        point.accumulatedNormalImpulse = 1.0f + static_cast<float>(index);
        ASSERT_TRUE(oldManifold.AddPoint(point));
    }

    ContactManifold newManifold;
    newManifold.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    for (int index = 0; index < 2; ++index) {
        ContactPoint point;
        point.localPointA = glm::vec3(0.0f);
        point.localPointB = glm::vec3(0.0f);
        ASSERT_TRUE(newManifold.AddPoint(point));
    }

    Runtime::Physics::MatchPersistentContactPoints(oldManifold, newManifold);

    // First new point claims old index 0; one-to-one matching leaves old index
    // 1 for the second new point.
    EXPECT_FLOAT_EQ(newManifold.Point(0).accumulatedNormalImpulse, 1.0f);
    EXPECT_FLOAT_EQ(newManifold.Point(1).accumulatedNormalImpulse, 2.0f);
}

TEST(ContactManifoldMatchTest, AnchorsBeyondThresholdDoNotTransfer) {
    ContactManifold oldManifold;
    oldManifold.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    ContactPoint oldPoint;
    oldPoint.localPointA = glm::vec3(0.0f);
    oldPoint.localPointB = glm::vec3(0.0f);
    oldPoint.accumulatedNormalImpulse = 4.0f;
    ASSERT_TRUE(oldManifold.AddPoint(oldPoint));

    ContactManifold newManifold;
    newManifold.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    ContactPoint newPoint;
    newPoint.localPointA = glm::vec3(1.0f, 0.0f, 0.0f);
    newPoint.localPointB = glm::vec3(1.0f, 0.0f, 0.0f);
    ASSERT_TRUE(newManifold.AddPoint(newPoint));

    Runtime::Physics::MatchPersistentContactPoints(oldManifold, newManifold);
    EXPECT_FLOAT_EQ(newManifold.Point(0).accumulatedNormalImpulse, 0.0f);
}

TEST(ContactManifoldMatchTest, SignificantNormalFlipTransfersNothing) {
    ContactManifold oldManifold;
    oldManifold.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    ContactPoint oldPoint;
    oldPoint.localPointA = glm::vec3(0.0f);
    oldPoint.localPointB = glm::vec3(0.0f);
    oldPoint.accumulatedNormalImpulse = 4.0f;
    ASSERT_TRUE(oldManifold.AddPoint(oldPoint));

    ContactManifold newManifold;
    newManifold.normal = glm::vec3(0.0f, -1.0f, 0.0f);
    ContactPoint newPoint;
    newPoint.localPointA = glm::vec3(0.0f);
    newPoint.localPointB = glm::vec3(0.0f);
    ASSERT_TRUE(newManifold.AddPoint(newPoint));

    Runtime::Physics::MatchPersistentContactPoints(oldManifold, newManifold);
    EXPECT_FLOAT_EQ(newManifold.Point(0).accumulatedNormalImpulse, 0.0f);
}

TEST(ContactManifoldMatchTest, TangentBasisIsOrthonormalAndDeterministic) {
    const glm::vec3 normal = glm::normalize(glm::vec3(0.3f, 1.0f, -0.2f));
    glm::vec3 tangent1A;
    glm::vec3 tangent2A;
    glm::vec3 tangent1B;
    glm::vec3 tangent2B;
    Runtime::Physics::BuildContactTangentBasis(normal, tangent1A, tangent2A);
    Runtime::Physics::BuildContactTangentBasis(normal, tangent1B, tangent2B);

    EXPECT_NEAR(glm::dot(tangent1A, normal), 0.0f, 1e-6f);
    EXPECT_NEAR(glm::dot(tangent2A, normal), 0.0f, 1e-6f);
    EXPECT_NEAR(glm::dot(tangent1A, tangent2A), 0.0f, 1e-6f);
    EXPECT_NEAR(glm::length(tangent1A), 1.0f, 1e-6f);
    EXPECT_NEAR(glm::length(tangent2A), 1.0f, 1e-6f);
    EXPECT_EQ(tangent1A, tangent1B);
    EXPECT_EQ(tangent2A, tangent2B);
}

// ---------------------------------------------------------------------------
// Pair pool lifecycle (plan step 7)
// ---------------------------------------------------------------------------

TEST(MidphaseTest, PairLifecycleNewPersistingRemovedWithSlotReuse) {
    DetectorScene scene;
    const uint32_t ground = scene.AddBox({0.0f, 0.0f, 0.0f}, {5.0f, 0.5f, 5.0f}, true);
    const uint32_t box = scene.AddBox({0.0f, 0.75f, 0.0f}, {0.5f, 0.5f, 0.5f}, false);
    const PairKey key = MakePairKey(ground, box);

    Midphase midphase;
    std::vector<ContactEvent> events;

    midphase.BeginQuery(1, 1, BroadPhaseQueryCoverage::FullScene);
    const uint32_t slot1 = midphase.RegisterCandidate(
        key, scene.colliders.at(ground), scene.bodies.at(ground),
        scene.colliders.at(box), scene.bodies.at(box));
    ASSERT_NE(midphase.FindPair(key), nullptr);
    EXPECT_EQ(midphase.FindPair(key)->state, PairLifecycleState::New);
    midphase.FinishQuery(events);
    EXPECT_TRUE(events.empty());
    EXPECT_EQ(midphase.ActivePairCount(), 1u);

    midphase.BeginQuery(2, 1, BroadPhaseQueryCoverage::FullScene);
    const uint32_t slot2 = midphase.RegisterCandidate(
        key, scene.colliders.at(ground), scene.bodies.at(ground),
        scene.colliders.at(box), scene.bodies.at(box));
    EXPECT_EQ(slot2, slot1);
    EXPECT_EQ(midphase.FindPair(key)->state, PairLifecycleState::Persisting);
    midphase.FinishQuery(events);

    // Not returned by a FullScene query: candidate removed. The pair never
    // touched, so no Exit is published.
    midphase.BeginQuery(3, 2, BroadPhaseQueryCoverage::FullScene);
    midphase.FinishQuery(events);
    EXPECT_TRUE(events.empty());
    EXPECT_EQ(midphase.LastStats().removedPairCount, 1u);
    EXPECT_EQ(midphase.ActivePairCount(), 0u);
    EXPECT_EQ(midphase.FindPair(key), nullptr);

    // After the synchronization point the freed slot is recycled for a new
    // registration of the same key.
    midphase.BeginQuery(4, 2, BroadPhaseQueryCoverage::FullScene);
    const uint32_t slot3 = midphase.RegisterCandidate(
        key, scene.colliders.at(ground), scene.bodies.at(ground),
        scene.colliders.at(box), scene.bodies.at(box));
    EXPECT_EQ(slot3, slot1);
    EXPECT_EQ(midphase.FindPair(key)->state, PairLifecycleState::New);
    midphase.FinishQuery(events);
}

// ---------------------------------------------------------------------------
// Detector-integrated cache and lifecycle behavior (plan steps 9-10)
// ---------------------------------------------------------------------------

TEST(MidphaseDetectorTest, ImpulseCachesTransferAcrossFramesByAnchorMatching) {
    DetectorScene scene;
    scene.AddBox({0.0f, 0.0f, 0.0f}, {5.0f, 0.5f, 5.0f}, true);
    const uint32_t box = scene.AddBox({0.0f, 0.75f, 0.0f}, {0.5f, 0.5f, 0.5f}, false);

    CollisionDetector detector;
    const CollisionDetectionResult& result1 =
        detector.Detect(scene.colliders, scene.bodies, 1, 1, kFixedDt, false);
    ASSERT_EQ(result1.touchingPairs.size(), 1u);
    ASSERT_EQ(CountEvents(result1.events, ContactEventType::Enter), 1u);

    MidphasePair& pair = detector.GetMidphase().PairAt(result1.touchingPairs.front());
    const std::size_t pointCount = pair.manifold.pointCount;
    ASSERT_GT(pointCount, 0u);
    SeedContactCaches(pair, 3.25f);

    // Internal integration-style motion: no external revision bump, so the
    // caches must survive and reattach to the matched points.
    scene.bodies.at(box).SetPositionInternal(
        scene.bodies.at(box).Position() + glm::vec3(0.01f, 0.0f, 0.0f));

    const CollisionDetectionResult& result2 =
        detector.Detect(scene.colliders, scene.bodies, 2, 2, kFixedDt, false);
    ASSERT_EQ(result2.touchingPairs.size(), 1u);
    EXPECT_EQ(CountEvents(result2.events, ContactEventType::Enter), 0u);
    EXPECT_EQ(CountEvents(result2.events, ContactEventType::Stay), 1u);

    const MidphasePair& pair2 = detector.GetMidphase().PairAt(result2.touchingPairs.front());
    ASSERT_EQ(pair2.manifold.pointCount, pointCount);
    for (std::size_t index = 0; index < pair2.manifold.pointCount; ++index) {
        const ContactPoint& point = pair2.manifold.Point(index);
        EXPECT_FLOAT_EQ(point.accumulatedNormalImpulse, 3.25f);
        EXPECT_FLOAT_EQ(point.accumulatedTangentImpulse.x, 0.5f);
        EXPECT_FLOAT_EQ(point.accumulatedTangentImpulse.y, -0.25f);
        // Sub-step accumulation base is reset for the solver-visible field.
        EXPECT_FLOAT_EQ(point.normalImpulse, 0.0f);
        // Interpretation data is transferred with the cache (the solver
        // re-stamps the fresh basis/dt when it stores the solved cache).
        EXPECT_FLOAT_EQ(point.cachedDt, kFixedDt);
        EXPECT_FALSE(point.isToiImpact);
        EXPECT_GT(glm::length(point.cachedTangent1), 0.5f);
    }
}

TEST(MidphaseDetectorTest, ExternalTeleportClearsImpulseCaches) {
    DetectorScene scene;
    scene.AddBox({0.0f, 0.0f, 0.0f}, {5.0f, 0.5f, 5.0f}, true);
    const uint32_t box = scene.AddBox({0.0f, 0.75f, 0.0f}, {0.5f, 0.5f, 0.5f}, false);

    CollisionDetector detector;
    const CollisionDetectionResult& result1 =
        detector.Detect(scene.colliders, scene.bodies, 1, 1, kFixedDt, false);
    ASSERT_EQ(result1.touchingPairs.size(), 1u);
    SeedContactCaches(detector.GetMidphase().PairAt(result1.touchingPairs.front()), 3.25f);

    // External pose write (teleport): bumps the pose revision even over a
    // distance small enough that anchor matching alone would keep the points.
    scene.bodies.at(box).SetPosition(
        scene.bodies.at(box).Position() + glm::vec3(0.02f, 0.0f, 0.0f));

    const CollisionDetectionResult& result2 =
        detector.Detect(scene.colliders, scene.bodies, 2, 2, kFixedDt, false);
    ASSERT_EQ(result2.touchingPairs.size(), 1u);
    ExpectCachesCleared(detector.GetMidphase().PairAt(result2.touchingPairs.front()));
}

TEST(MidphaseDetectorTest, ColliderReplacementClearsImpulseCaches) {
    DetectorScene scene;
    scene.AddBox({0.0f, 0.0f, 0.0f}, {5.0f, 0.5f, 5.0f}, true);
    const uint32_t box = scene.AddBox({0.0f, 0.75f, 0.0f}, {0.5f, 0.5f, 0.5f}, false);

    CollisionDetector detector;
    const CollisionDetectionResult& result1 =
        detector.Detect(scene.colliders, scene.bodies, 1, 1, kFixedDt, false);
    ASSERT_EQ(result1.touchingPairs.size(), 1u);
    SeedContactCaches(detector.GetMidphase().PairAt(result1.touchingPairs.front()), 3.25f);

    // Same geometry, fresh collider identity: replacement must be detected.
    scene.ReplaceCollider(box, {0.5f, 0.5f, 0.5f});

    const CollisionDetectionResult& result2 =
        detector.Detect(scene.colliders, scene.bodies, 2, 2, kFixedDt, false);
    ASSERT_EQ(result2.touchingPairs.size(), 1u);
    ExpectCachesCleared(detector.GetMidphase().PairAt(result2.touchingPairs.front()));
}

TEST(MidphaseDetectorTest, MaterialValueChangeClearsImpulseCaches) {
    DetectorScene scene;
    scene.AddBox({0.0f, 0.0f, 0.0f}, {5.0f, 0.5f, 5.0f}, true);
    const uint32_t box = scene.AddBox({0.0f, 0.75f, 0.0f}, {0.5f, 0.5f, 0.5f}, false);

    CollisionDetector detector;
    const CollisionDetectionResult& result1 =
        detector.Detect(scene.colliders, scene.bodies, 1, 1, kFixedDt, false);
    ASSERT_EQ(result1.touchingPairs.size(), 1u);
    SeedContactCaches(detector.GetMidphase().PairAt(result1.touchingPairs.front()), 3.25f);

    // The mutable Material() accessor cannot bump the revision; the midphase
    // compares material values at sync time instead.
    scene.colliders.at(box).Material().restitution = 0.9f;

    const CollisionDetectionResult& result2 =
        detector.Detect(scene.colliders, scene.bodies, 2, 2, kFixedDt, false);
    ASSERT_EQ(result2.touchingPairs.size(), 1u);
    ExpectCachesCleared(detector.GetMidphase().PairAt(result2.touchingPairs.front()));
}

TEST(MidphaseDetectorTest, SeparationKeepsCandidatePairAndPublishesExit) {
    DetectorScene scene;
    scene.AddBox({0.0f, 0.0f, 0.0f}, {5.0f, 0.5f, 5.0f}, true);
    const uint32_t box = scene.AddBox({0.0f, 0.75f, 0.0f}, {0.5f, 0.5f, 0.5f}, false);
    const PairKey key = MakePairKey(1u, box);

    CollisionDetector detector;
    const CollisionDetectionResult& result1 =
        detector.Detect(scene.colliders, scene.bodies, 1, 1, kFixedDt, false);
    ASSERT_EQ(result1.touchingPairs.size(), 1u);

    // Lift the box so the shapes separate by 0.02: inside the fat AABB margin
    // (0.08 per leaf), so the candidate pair persists while touching ends.
    scene.bodies.at(box).SetPositionInternal(
        scene.bodies.at(box).Position() + glm::vec3(0.0f, 0.27f, 0.0f));

    const CollisionDetectionResult& result2 =
        detector.Detect(scene.colliders, scene.bodies, 2, 2, kFixedDt, false);
    EXPECT_TRUE(result2.touchingPairs.empty());
    EXPECT_EQ(CountEvents(result2.events, ContactEventType::Exit), 1u);

    // The candidate pair is still tracked although it is no longer touching.
    const MidphasePair* pair = detector.GetMidphase().FindPair(key);
    ASSERT_NE(pair, nullptr);
    EXPECT_FALSE(pair->hadContact);
    EXPECT_EQ(pair->manifold.pointCount, 0u);

    // Moving fully out of the fat AABBs removes the candidate; already
    // separated, so no second Exit.
    scene.bodies.at(box).SetPositionInternal(
        scene.bodies.at(box).Position() + glm::vec3(0.0f, 5.0f, 0.0f));
    const CollisionDetectionResult& result3 =
        detector.Detect(scene.colliders, scene.bodies, 3, 3, kFixedDt, false);
    EXPECT_EQ(CountEvents(result3.events, ContactEventType::Exit), 0u);
    EXPECT_EQ(detector.GetMidphase().FindPair(key), nullptr);
}

TEST(MidphaseDetectorTest, EnterThenExitWithinOneFixedStepIsNotSwallowed) {
    DetectorScene scene;
    scene.AddBox({0.0f, 0.0f, 0.0f}, {5.0f, 0.5f, 5.0f}, true);
    const uint32_t box = scene.AddBox({0.0f, 0.75f, 0.0f}, {0.5f, 0.5f, 0.5f}, false, true);

    CollisionDetector detector;
    // Simulates a CCD fixed step: a TOI sub-step touches, a later sub-step of
    // the same fixed step separates. Both events must survive.
    constexpr uint64_t fixedStepId = 7;
    const CollisionDetectionResult& result1 =
        detector.Detect(scene.colliders, scene.bodies, fixedStepId, 1, kFixedDt * 0.25f, true);
    ASSERT_EQ(result1.touchingPairs.size(), 1u);
    ASSERT_EQ(result1.events.size(), 1u);
    EXPECT_EQ(result1.events.front().type, ContactEventType::Enter);
    EXPECT_EQ(result1.events.front().fixedStepId, fixedStepId);
    EXPECT_EQ(result1.events.front().queryEpoch, 1u);

    scene.bodies.at(box).SetPositionInternal(
        scene.bodies.at(box).Position() + glm::vec3(0.0f, 5.0f, 0.0f));
    const CollisionDetectionResult& result2 =
        detector.Detect(scene.colliders, scene.bodies, fixedStepId, 2, kFixedDt * 0.75f, false);
    ASSERT_EQ(result2.events.size(), 1u);
    EXPECT_EQ(result2.events.front().type, ContactEventType::Exit);
    EXPECT_EQ(result2.events.front().fixedStepId, fixedStepId);
    EXPECT_GT(result2.events.front().queryEpoch, 1u);
}

// ---------------------------------------------------------------------------
// World-level integration (plan steps 10-11)
// ---------------------------------------------------------------------------

TEST(MidphaseWorldTest, TriggerEnterStayExitSequenceAcrossSteps) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f));
    world.SetContinuousCollisionEnabled(false);

    RigidBodyDesc groundDesc;
    groundDesc.isStatic = true;
    groundDesc.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(groundDesc);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(5.0f, 0.5f, 5.0f));
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    RigidBodyDesc boxDesc;
    boxDesc.position = {0.0f, 0.75f, 0.0f};
    boxDesc.useGravity = false;
    const uint32_t boxId = world.CreateRigidBody(boxDesc);
    ColliderDesc boxCollider;
    boxCollider.shape = std::make_shared<BoxShape>(glm::vec3(0.5f));
    boxCollider.isTrigger = true;
    ASSERT_TRUE(world.AttachCollider(boxId, boxCollider));

    // First touch: Enter only.
    world.Step(kFixedDt);
    EXPECT_EQ(CountEvents(world.ContactEvents(), ContactEventType::Enter), 1u);
    EXPECT_EQ(CountEvents(world.ContactEvents(), ContactEventType::Stay), 0u);
    EXPECT_EQ(world.ContactEvents().front().pair, MakePairKey(groundId, boxId));

    // Persisting touch: exactly one Stay per fixed step, no repeated Enter.
    world.Step(kFixedDt);
    EXPECT_EQ(CountEvents(world.ContactEvents(), ContactEventType::Enter), 0u);
    EXPECT_EQ(CountEvents(world.ContactEvents(), ContactEventType::Stay), 1u);

    // One Step executing two fixed steps emits events for both.
    world.Step(2.0f * kFixedDt);
    EXPECT_EQ(CountEvents(world.ContactEvents(), ContactEventType::Stay), 2u);

    // Teleport away: Exit exactly once and the contact summary is empty.
    world.GetRigidBody(boxId)->SetPosition(glm::vec3(100.0f, 0.0f, 0.0f));
    world.Step(kFixedDt);
    EXPECT_EQ(CountEvents(world.ContactEvents(), ContactEventType::Exit), 1u);
    EXPECT_TRUE(world.Contacts().empty());
}

TEST(MidphaseWorldTest, DestroyedBodyPublishesExitAndLeavesNoDanglingPair) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f));
    world.SetContinuousCollisionEnabled(false);

    RigidBodyDesc groundDesc;
    groundDesc.isStatic = true;
    groundDesc.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(groundDesc);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(5.0f, 0.5f, 5.0f));
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    RigidBodyDesc boxDesc;
    boxDesc.position = {0.0f, 0.75f, 0.0f};
    boxDesc.useGravity = false;
    const uint32_t boxId = world.CreateRigidBody(boxDesc);
    ColliderDesc boxCollider;
    boxCollider.shape = std::make_shared<BoxShape>(glm::vec3(0.5f));
    ASSERT_TRUE(world.AttachCollider(boxId, boxCollider));

    world.Step(kFixedDt);
    ASSERT_EQ(world.Contacts().size(), 1u);

    ASSERT_TRUE(world.DestroyRigidBody(boxId));
    world.Step(kFixedDt);

    EXPECT_EQ(CountEvents(world.ContactEvents(), ContactEventType::Exit), 1u);
    EXPECT_EQ(world.ContactEvents().front().pair, MakePairKey(groundId, boxId));
    EXPECT_TRUE(world.Contacts().empty());
    EXPECT_EQ(world.LastStepStats().midphaseActivePairCount, 0u);
}

TEST(MidphaseWorldTest, LayerMaskChangePublishesExitAndRemovesPair) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f));
    world.SetContinuousCollisionEnabled(false);

    RigidBodyDesc groundDesc;
    groundDesc.isStatic = true;
    groundDesc.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(groundDesc);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(5.0f, 0.5f, 5.0f));
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    RigidBodyDesc boxDesc;
    boxDesc.position = {0.0f, 0.75f, 0.0f};
    boxDesc.useGravity = false;
    const uint32_t boxId = world.CreateRigidBody(boxDesc);
    ColliderDesc boxCollider;
    boxCollider.shape = std::make_shared<BoxShape>(glm::vec3(0.5f));
    ASSERT_TRUE(world.AttachCollider(boxId, boxCollider));

    world.Step(kFixedDt);
    ASSERT_EQ(world.Contacts().size(), 1u);

    // Filtering the pair out re-filters the old pair and ends touching.
    world.GetCollider(boxId)->SetMask(0u);
    world.Step(kFixedDt);

    EXPECT_EQ(CountEvents(world.ContactEvents(), ContactEventType::Exit), 1u);
    EXPECT_TRUE(world.Contacts().empty());
    EXPECT_EQ(world.LastStepStats().midphaseActivePairCount, 0u);
}

TEST(MidphaseWorldTest, SteadyStatePairPoolStopsAllocating) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);

    RigidBodyDesc groundDesc;
    groundDesc.isStatic = true;
    groundDesc.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(groundDesc);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(10.0f, 0.5f, 10.0f));
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    RigidBodyDesc boxDesc;
    boxDesc.position = {0.0f, 1.2f, 0.0f};
    const uint32_t boxId = world.CreateRigidBody(boxDesc);
    ColliderDesc boxCollider;
    boxCollider.shape = std::make_shared<BoxShape>(glm::vec3(0.5f));
    ASSERT_TRUE(world.AttachCollider(boxId, boxCollider));

    // Let the box land and settle onto the ground.
    for (int step = 0; step < 120; ++step) {
        world.Step(kFixedDt);
    }
    ASSERT_EQ(world.Contacts().size(), 1u);

    // Steady state: the same pair persists without new allocations or churn.
    for (int step = 0; step < 10; ++step) {
        world.Step(kFixedDt);
        const auto& stats = world.LastStepStats();
        EXPECT_EQ(stats.midphaseActivePairCount, 1u);
        EXPECT_EQ(stats.midphaseNewPairCount, 0u);
        EXPECT_EQ(stats.midphaseRemovedPairCount, 0u);
    }
}

TEST(MidphaseWorldTest, NormalImpulseIsSubstepAccumulatedAndTriggerStaysZero) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.SetContinuousCollisionEnabled(false);

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

    RigidBodyDesc triggerDesc;
    triggerDesc.position = {5.0f, 0.55f, 0.0f};
    const uint32_t triggerId = world.CreateRigidBody(triggerDesc);
    ColliderDesc triggerCollider;
    triggerCollider.shape = std::make_shared<BoxShape>(glm::vec3(0.5f));
    triggerCollider.isTrigger = true;
    ASSERT_TRUE(world.AttachCollider(triggerId, triggerCollider));

    // A static platform under the trigger box as well.
    RigidBodyDesc groundDesc2;
    groundDesc2.position = {5.0f, 0.0f, 0.0f};
    groundDesc2.isStatic = true;
    groundDesc2.useGravity = false;
    const uint32_t groundId2 = world.CreateRigidBody(groundDesc2);
    ColliderDesc groundCollider2;
    groundCollider2.shape = std::make_shared<BoxShape>(glm::vec3(1.0f, 0.5f, 1.0f));
    ASSERT_TRUE(world.AttachCollider(groundId2, groundCollider2));

    for (int step = 0; step < 30; ++step) {
        world.Step(kFixedDt);
    }

    bool checkedSolid = false;
    bool checkedTrigger = false;
    for (const ContactManifold& contact : world.Contacts()) {
        const PairKey key = MakePairKey(contact.bodyA, contact.bodyB);
        if (key == MakePairKey(groundId, boxId)) {
            checkedSolid = true;
            EXPECT_GT(contact.point.normalImpulse, 0.0f);
            // After solving, the persistent cache mirrors the sub-step
            // accumulated value (Phase 2: no warm start yet).
            EXPECT_FLOAT_EQ(contact.point.normalImpulse, contact.point.accumulatedNormalImpulse);
        }
        if (key == MakePairKey(groundId2, triggerId)) {
            checkedTrigger = true;
            EXPECT_TRUE(contact.isTrigger);
            // Trigger contacts never carry impulses, including the fixed-step
            // total summary fields.
            EXPECT_FLOAT_EQ(contact.point.normalImpulse, 0.0f);
            EXPECT_FLOAT_EQ(contact.point.accumulatedNormalImpulse, 0.0f);
            EXPECT_FLOAT_EQ(contact.point.accumulatedTangentImpulse.x, 0.0f);
            EXPECT_FLOAT_EQ(contact.point.accumulatedTangentImpulse.y, 0.0f);
            EXPECT_FLOAT_EQ(contact.fixedStepNormalImpulse, 0.0f);
            EXPECT_FLOAT_EQ(glm::length(contact.fixedStepTangentImpulse), 0.0f);
        }
    }
    EXPECT_TRUE(checkedSolid);
    EXPECT_TRUE(checkedTrigger);
}

TEST(MidphaseWorldTest, ContactSetMatchesLegacyBroadPhasePath) {
    auto buildWorld = []() {
        auto world = std::make_unique<PhysicsWorld>();
        world->SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
        world->SetContinuousCollisionEnabled(false);

        RigidBodyDesc groundDesc;
        groundDesc.isStatic = true;
        groundDesc.useGravity = false;
        const uint32_t groundId = world->CreateRigidBody(groundDesc);
        ColliderDesc groundCollider;
        groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(10.0f, 0.5f, 10.0f));
        EXPECT_TRUE(world->AttachCollider(groundId, groundCollider));

        for (int index = 0; index < 3; ++index) {
            RigidBodyDesc boxDesc;
            boxDesc.position = {0.1f * static_cast<float>(index), 0.6f + 1.1f * static_cast<float>(index), 0.0f};
            const uint32_t boxId = world->CreateRigidBody(boxDesc);
            ColliderDesc boxCollider;
            boxCollider.shape = std::make_shared<BoxShape>(glm::vec3(0.5f));
            EXPECT_TRUE(world->AttachCollider(boxId, boxCollider));
        }
        return world;
    };

    auto hybridWorld = buildWorld();
    auto legacyWorld = buildWorld();
    legacyWorld->SetLegacyBroadPhaseEnabled(true);

    for (int step = 0; step < 30; ++step) {
        hybridWorld->Step(kFixedDt);
        legacyWorld->Step(kFixedDt);

        // Same frozen input must yield the same touching pair set on both
        // broad-phase paths through the midphase.
        ASSERT_EQ(hybridWorld->Contacts().size(), legacyWorld->Contacts().size());
        for (std::size_t index = 0; index < hybridWorld->Contacts().size(); ++index) {
            const ContactManifold& a = hybridWorld->Contacts()[index];
            const ContactManifold& b = legacyWorld->Contacts()[index];
            EXPECT_EQ(MakePairKey(a.bodyA, a.bodyB), MakePairKey(b.bodyA, b.bodyB));
            EXPECT_EQ(a.pointCount, b.pointCount);
        }
    }
    EXPECT_FALSE(hybridWorld->Contacts().empty());
}

TEST(MidphaseWorldTest, CcdTriggerEnterAndExitInsideSingleFixedStep) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f));
    world.SetContinuousCollisionEnabled(true);
    world.SetCcdMaxSubSteps(12);

    RigidBodyDesc wallDesc;
    wallDesc.position = {5.0f, 0.0f, 0.0f};
    wallDesc.isStatic = true;
    wallDesc.useGravity = false;
    const uint32_t wallId = world.CreateRigidBody(wallDesc);
    ColliderDesc wallCollider;
    wallCollider.shape = std::make_shared<BoxShape>(glm::vec3(0.05f, 5.0f, 5.0f));
    ASSERT_TRUE(world.AttachCollider(wallId, wallCollider));

    RigidBodyDesc ballDesc;
    ballDesc.position = {0.0f, 0.0f, 0.0f};
    ballDesc.linearVelocity = {400.0f, 0.0f, 0.0f};
    ballDesc.useGravity = false;
    const uint32_t ballId = world.CreateRigidBody(ballDesc);
    ColliderDesc ballCollider;
    ballCollider.shape = std::make_shared<SphereShape>(0.25f);
    ballCollider.isTrigger = true;
    ASSERT_TRUE(world.AttachCollider(ballId, ballCollider));

    // The ball crosses the thin wall within this fixed step (TOI around
    // 11.75 ms of 16.67 ms): the transient touch must publish Enter AND Exit
    // inside the same fixed step instead of being swallowed by the step-end
    // separated state.
    world.Step(kFixedDt);

    const PairKey key = MakePairKey(wallId, ballId);
    const ContactEvent* enter = nullptr;
    const ContactEvent* exit = nullptr;
    for (const ContactEvent& event : world.ContactEvents()) {
        if (event.pair != key) {
            continue;
        }
        if (event.type == ContactEventType::Enter) {
            enter = &event;
        }
        if (event.type == ContactEventType::Exit) {
            exit = &event;
        }
    }
    ASSERT_NE(enter, nullptr);
    ASSERT_NE(exit, nullptr);
    EXPECT_EQ(enter->fixedStepId, exit->fixedStepId);
    EXPECT_GT(exit->queryEpoch, enter->queryEpoch);

    // The trigger ball is not impeded and flew past the wall.
    const auto* ball = world.GetRigidBody(ballId);
    ASSERT_NE(ball, nullptr);
    EXPECT_GT(ball->Position().x, 5.3f);

    // Publish contract: Contacts() reports the unique pairs that touched at any
    // sub-step of the last completed fixed step (geometry from the last touching
    // sub-step); it does not claim the pair is still touching at step end. The
    // transient TOI touch is therefore still listed here, with zero impulses
    // because the pair is a trigger.
    ASSERT_EQ(world.Contacts().size(), 1u);
    EXPECT_EQ(MakePairKey(world.Contacts().front().bodyA, world.Contacts().front().bodyB), key);
    EXPECT_TRUE(world.Contacts().front().isTrigger);
    EXPECT_FLOAT_EQ(world.Contacts().front().point.normalImpulse, 0.0f);
}
