#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "Physics/CollisionDetector.h"
#include "Physics/PhysicsWorld.h"

namespace {
using namespace Runtime::Physics;
constexpr float kHalf = 0.45f;

// Add a physical cube with Ixx = m*(width^2 + depth^2)/12.
uint32_t AddBox(PhysicsWorld& world, const glm::vec3& position,
    const glm::quat& orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)) {
    RigidBodyDesc desc;
    desc.position = position;
    desc.orientation = orientation;
    desc.inertiaTensorDiagonal = glm::vec3(0.135f);
    const uint32_t id = world.CreateRigidBody(desc);
    ColliderDesc collider;
    collider.shape = std::make_shared<BoxShape>(glm::vec3(kHalf));
    collider.material.restitution = 0.05f;
    collider.material.dynamicFriction = 1.0f;
    EXPECT_TRUE(world.AttachCollider(id, collider));
    return id;
}

// Ground top is y=0; all tests use the sample's gravity and unmodified solver.
void AddGround(PhysicsWorld& world) {
    RigidBodyDesc desc;
    desc.position = {0.0f, -0.5f, 0.0f};
    desc.isStatic = true;
    const uint32_t id = world.CreateRigidBody(desc);
    ColliderDesc collider;
    collider.shape = std::make_shared<BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
    collider.material.restitution = 0.1f;
    collider.material.dynamicFriction = 1.0f;
    ASSERT_TRUE(world.AttachCollider(id, collider));
}

// A diamond bridges two three-high columns: the fourth level rests on two edge/face patches.
TEST(StackPipelineTest, FourLevelsRetainNonFaceContactsForSixtySeconds) {
    for (const bool sleep : {false, true}) {
        for (const uint32_t workers : {1u, 4u}) {
            SCOPED_TRACE(::testing::Message() << "sleep=" << sleep << " workers=" << workers);
            PhysicsWorld world;
            world.SetPhysicsWorkerCount(workers);
            world.SetFixedTimeStep(1.0f / 120.0f);
            world.SetGravity({0.0f, -10.5f, 0.0f});
            world.SetSolverIterations(10);
            world.SetSleepEnabled(sleep);
            ASSERT_TRUE(world.ContinuousCollisionEnabled());
            AddGround(world);
            std::vector<uint32_t> ids;
            for (int level = 0; level < 3; ++level)
                for (float x : {-0.5f, 0.5f})
                    ids.push_back(AddBox(world, {x, kHalf + 0.9f * level + 0.02f, 0.0f}));
            const uint32_t cap = AddBox(world, {0.0f, 3.7f, 0.0f},
                glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f)));
            ids.push_back(cap);
            bool sawEdgeFace = false, sawSleepingEdgeFace = false;
            int sleepStep = -1;
            float maxGroundPenetration = 0.0f, maxContactPenetration = 0.0f;
            float settledSupportImpulse = 0.0f;
            std::size_t minNonFaceSupports = 2;
            std::vector<glm::vec3> reference;
            for (int step = 0; step < 7200; ++step) {
                world.Step(1.0f / 120.0f);
                if (sleepStep < 0 && world.LastStepStats().sleepingBodyCount == ids.size())
                    sleepStep = step;
                std::size_t nonFaceSupports = 0;
                for (const auto& contact : world.Contacts()) {
                    if ((contact.bodyA == cap || contact.bodyB == cap) &&
                        (contact.topology == ContactTopology::FaceEdge ||
                         contact.topology == ContactTopology::FaceVertex ||
                         contact.topology == ContactTopology::EdgeEdge)) {
                        sawEdgeFace = true;
                        if (world.GetRigidBody(cap)->IsSleeping()) sawSleepingEdgeFace = true;
                        ++nonFaceSupports;
                        if (step >= 240) settledSupportImpulse += contact.fixedStepNormalImpulse;
                    }
                    for (std::size_t p = 0; p < contact.pointCount; ++p)
                        maxContactPenetration = std::max(maxContactPenetration, contact.Point(p).penetration);
                }
                if (step >= 240 && !sleep)
                    minNonFaceSupports = std::min(minNonFaceSupports, nonFaceSupports);
                for (uint32_t id : ids) {
                    const RigidBody& body = *world.GetRigidBody(id);
                    const AABB bounds = BoxShape(glm::vec3(kHalf)).ComputeAABB(
                        {body.Position(), body.Orientation()});
                    maxGroundPenetration = std::max(maxGroundPenetration, -bounds.min.y);
                }
                if (step == 3599)
                    for (uint32_t id : ids) reference.push_back(world.GetRigidBody(id)->Position());
            }
            EXPECT_TRUE(sawEdgeFace);
            EXPECT_LT(maxGroundPenetration, 0.004f);
            EXPECT_LT(maxContactPenetration, 0.004f);
            // Gravity supplies 609 N*s after settling; mu=1 bounds |J| <= sqrt(2)*Jn.
            if (!sleep) EXPECT_GT(settledSupportImpulse, 400.0f);
            EXPECT_GE(minNonFaceSupports, 2u);
            for (std::size_t i = 0; i < 6; ++i) {
                const RigidBody& body = *world.GetRigidBody(ids[i]);
                EXPECT_NEAR(body.Position().y, kHalf + 0.9f * static_cast<float>(i / 2), 0.04f);
                EXPECT_LT(glm::length(body.Position() - reference[i]), 0.02f);
                if (sleep) EXPECT_TRUE(body.IsSleeping());
            }
            const RigidBody& top = *world.GetRigidBody(cap);
            std::printf("[cradle sleep=%d workers=%u] groundPen=%.6f contactPen=%.6f topY=%.5f sleepStep=%d\n",
                sleep ? 1 : 0, workers, maxGroundPenetration, maxContactPenetration, top.Position().y, sleepStep);
            EXPECT_GT(top.Position().y, 3.1f);
            EXPECT_LT(glm::length(top.Position() - reference.back()), 0.02f);
            EXPECT_NEAR(std::abs(glm::dot(top.Orientation(),
                glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f)))), 1.0f, 0.002f);
            ColliderDesc shapeDesc;
            shapeDesc.shape = std::make_shared<BoxShape>(glm::vec3(kHalf));
            Collider shape(shapeDesc);
            GjkEpaNarrowPhase narrow;
            narrow.SetSpeculativeContactDistance(0.004f);
            for (std::size_t i : {4u, 5u}) {
                ContactManifold contact;
                NarrowPhaseQueryStats stats;
                ASSERT_TRUE(narrow.GenerateContact(shape, *world.GetRigidBody(ids[i]),
                    shape, top, contact, stats));
                EXPECT_TRUE(contact.topology == ContactTopology::FaceEdge ||
                    contact.topology == ContactTopology::FaceVertex ||
                    contact.topology == ContactTopology::EdgeEdge);
            }
            if (sleep) {
                EXPECT_LT(sleepStep, 360);
                EXPECT_TRUE(top.IsSleeping());
                EXPECT_TRUE(sawSleepingEdgeFace) << "sleep must retain real non-face contact geometry";
            }
        }
    }
}

// A tilted reference plane must never manufacture off-surface anchors.
TEST(StackPipelineTest, TiltedBoxWitnessesStayOnBothSurfaces) {
    ColliderDesc desc;
    desc.shape = std::make_shared<BoxShape>(glm::vec3(0.45f));
    Collider a(desc), b(desc);
    RigidBodyDesc da, db;
    da.orientation = glm::angleAxis(glm::radians(25.0f), glm::normalize(glm::vec3(1.0f, 0.0f, 1.0f)));
    db.position = {0.0f, 0.7f, 0.0f};
    RigidBody ba(da), bb(db);
    GjkEpaNarrowPhase narrow;
    ContactManifold contact;
    NarrowPhaseQueryStats stats;
    ASSERT_TRUE(narrow.GenerateContact(a, ba, b, bb, contact, stats));
    for (std::size_t p = 0; p < contact.pointCount; ++p) {
        const ContactPoint& point = contact.Point(p);
        const glm::vec3 localA = glm::conjugate(ba.Orientation()) * (point.surfacePointA - ba.Position());
        const glm::vec3 localB = glm::conjugate(bb.Orientation()) * (point.surfacePointB - bb.Position());
        for (int axis = 0; axis < 3; ++axis) {
            EXPECT_LE(std::abs(localA[axis]), kHalf + 2e-5f);
            EXPECT_LE(std::abs(localB[axis]), kHalf + 2e-5f);
        }

        EXPECT_NEAR(glm::dot(point.surfacePointA - point.surfacePointB, contact.normal),
            point.penetration, 1e-5f);
    }
}

// Random OBB patches must satisfy surface bounds and the signed-separation identity.
TEST(StackPipelineTest, RandomBoxWitnessesArePhysical) {
    std::mt19937 random(1337);
    std::uniform_real_distribution<float> angle(-3.14f, 3.14f), offset(-0.8f, 0.8f);
    ColliderDesc desc;
    desc.shape = std::make_shared<BoxShape>(glm::vec3(kHalf));
    Collider a(desc), b(desc);
    GjkEpaNarrowPhase narrow;
    narrow.SetSpeculativeContactDistance(0.02f);
    for (int sample = 0; sample < 2000; ++sample) {
        SCOPED_TRACE(sample);
        RigidBodyDesc da, db;
        da.orientation = glm::quat(glm::vec3(angle(random), angle(random), angle(random)));
        db.orientation = glm::quat(glm::vec3(angle(random), angle(random), angle(random)));
        if ((sample & 1) == 0) {
            da.orientation = glm::quat(glm::vec3(1e-5f * angle(random), angle(random), 1e-5f * angle(random)));
            db.orientation = da.orientation * glm::quat(
                glm::vec3(1e-5f * angle(random), 1e-5f * angle(random), 1e-5f * angle(random)));
        }
        db.position = {offset(random), offset(random), offset(random)};
        RigidBody ba(da), bb(db);
        ContactManifold contact;
        NarrowPhaseQueryStats stats;
        if (!narrow.GenerateContact(a, ba, b, bb, contact, stats)) continue;
        for (std::size_t p = 0; p < contact.pointCount; ++p) {
            const ContactPoint& point = contact.Point(p);
            const glm::vec3 localA = glm::conjugate(ba.Orientation()) * (point.surfacePointA - ba.Position());
            const glm::vec3 localB = glm::conjugate(bb.Orientation()) * (point.surfacePointB - bb.Position());
            for (int axis = 0; axis < 3; ++axis) {
                ASSERT_LE(std::abs(localA[axis]), kHalf + 1e-4f);
                ASSERT_LE(std::abs(localB[axis]), kHalf + 1e-4f);
            }
            ASSERT_NEAR(glm::dot(point.surfacePointA - point.surfacePointB, contact.normal),
                point.penetration, 1e-4f);
        }
    }
}

// Redundant hull edge vertices must not turn a planar support face into a NaN plane.
TEST(StackPipelineTest, CollinearHullFacePrefixesStillProduceAPatch) {
    std::vector<glm::vec3> vertices;
    for (float y : {-kHalf, kHalf}) {
        vertices.insert(vertices.end(), {
            {-kHalf, y, -kHalf}, {0.0f, y, -kHalf}, {kHalf, y, -kHalf},
            {kHalf, y, kHalf}, {-kHalf, y, kHalf}});
    }
    ColliderDesc desc;
    desc.shape = std::make_shared<ConvexHullShape>(vertices);
    Collider a(desc), b(desc);
    RigidBodyDesc da, db;
    db.position.y = 0.88f;
    RigidBody ba(da), bb(db);
    GjkEpaNarrowPhase narrow;
    ContactManifold contact;
    NarrowPhaseQueryStats stats;
    ASSERT_TRUE(narrow.GenerateContact(a, ba, b, bb, contact, stats));
    EXPECT_GE(contact.pointCount, 2u);
    for (std::size_t p = 0; p < contact.pointCount; ++p) {
        EXPECT_TRUE(std::isfinite(contact.Point(p).penetration));
        EXPECT_NEAR(contact.Point(p).penetration, 0.02f, 1e-4f);
    }
}

// A body spinning about the old single probe's axis must not be frozen by sleep.
TEST(StackPipelineTest, RotationAboutXCannotPassTheSleepProbeTest) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f));
    const uint32_t id = AddBox(world, glm::vec3(0.0f));
    world.GetRigidBody(id)->SetAngularVelocity({0.2f, 0.0f, 0.0f});
    for (int step = 0; step < 240; ++step) world.Step(1.0f / 60.0f);
    EXPECT_FALSE(world.GetRigidBody(id)->IsSleeping());
    EXPECT_NEAR(world.GetRigidBody(id)->AngularVelocity().x, 0.2f, 1e-5f);
}

// Numerical TOI contact tolerance remains effective when speculative contacts are disabled.
TEST(StackPipelineTest, CcdDoesNotRequireTheSpeculativeContactBand) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f));
    world.SetSpeculativeContactDistance(0.0f);
    AddGround(world);
    const uint32_t id = AddBox(world, {0.0f, 2.0f, 0.0f});
    world.GetRigidBody(id)->SetLinearVelocity({0.0f, -450.0f, 0.0f});
    world.Step(1.0f / 60.0f);
    EXPECT_GE(world.GetRigidBody(id)->Position().y, 0.43f);
    EXPECT_GT(world.LastStepStats().ccdToiHitCount, 0u);
}

// A speculative single-corner hit must still sweep the same pair for newly rotating support features.
TEST(StackPipelineTest, CcdRechecksSeparatedSpeculativePairsAfterTheVelocitySolve) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f));
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetSolverIterations(10);
    AddGround(world);
    const glm::quat rotation = glm::quat(glm::vec3(glm::radians(2.0f), 0.0f, glm::radians(2.0f)));
    const BoxShape shape{glm::vec3(kHalf)};
    const float extentY = shape.ComputeAABB({glm::vec3(0.0f), rotation}).max.y;
    const uint32_t id = AddBox(world, {0.0f, extentY + 0.018f, 0.0f}, rotation);
    world.GetRigidBody(id)->SetLinearVelocity({0.0f, -16.0f, 0.0f});
    GjkEpaNarrowPhase narrow;
    narrow.SetSpeculativeContactDistance(0.02f);
    ContactManifold initial;
    NarrowPhaseQueryStats stats;
    ASSERT_TRUE(narrow.GenerateContact(*world.GetCollider(1), *world.GetRigidBody(1),
        *world.GetCollider(id), *world.GetRigidBody(id), initial, stats));
    ASSERT_EQ(initial.pointCount, 1u);
    world.Step(1.0f / 120.0f);
    const RigidBody& body = *world.GetRigidBody(id);
    EXPECT_GE(shape.ComputeAABB({body.Position(), body.Orientation()}).min.y, -0.02f);
    EXPECT_GT(world.LastStepStats().ccdToiHitCount, 0u);
}

// Torsional friction is part of the persistent patch cache, just like linear friction.
TEST(StackPipelineTest, PersistentMatchingTransfersTheSpinImpulse) {
    ContactManifold oldContact;
    ContactPoint point;
    point.accumulatedSpinImpulse = 0.3f;
    point.cachedDt = 1.0f / 120.0f;
    ASSERT_TRUE(oldContact.AddPoint(point));
    ContactManifold fresh;
    ASSERT_TRUE(fresh.AddPoint(ContactPoint{}));
    MatchPersistentContactPoints(oldContact, fresh);
    EXPECT_EQ(fresh.Point(0).accumulatedSpinImpulse, 0.3f);
}
} // namespace
