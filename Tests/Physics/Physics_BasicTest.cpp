#include <gtest/gtest.h>

#include <memory>
#include <limits>
#include <stdexcept>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "Physics/CollisionShape.h"
#include "Physics/PhysicsWorld.h"

using Runtime::Physics::BoxShape;
using Runtime::Physics::ColliderDesc;
using Runtime::Physics::ConvexHullShape;
using Runtime::Physics::PhysicsWorld;
using Runtime::Physics::RigidBodyDesc;
using Runtime::Physics::ShapeTransform;
using Runtime::Physics::SphereShape;
using Runtime::Physics::SupportFeatureType;

TEST(PhysicsWorldTest, CcdAndSpeculativeThresholdsAreIndependent) {
    PhysicsWorld world;
    EXPECT_FLOAT_EQ(world.CcdMotionThreshold(), 0.02f);
    EXPECT_FLOAT_EQ(world.SpeculativeContactDistance(), 0.02f);
    EXPECT_FLOAT_EQ(world.Settings().ccdMotionThreshold, 0.02f);

    world.SetSpeculativeContactDistance(0.0f);
    EXPECT_FLOAT_EQ(world.CcdMotionThreshold(), 0.02f);
    world.SetCcdMotionThreshold(0.2f);
    EXPECT_FLOAT_EQ(world.CcdMotionThreshold(), 0.2f);
    EXPECT_FLOAT_EQ(world.Settings().ccdMotionThreshold, 0.2f);
    EXPECT_FLOAT_EQ(world.SpeculativeContactDistance(), 0.0f);

    world.SetSpeculativeContactDistance(0.04f);
    EXPECT_FLOAT_EQ(world.CcdMotionThreshold(), 0.2f);
    world.SetCcdMotionThreshold(0.0f);
    EXPECT_FLOAT_EQ(world.CcdMotionThreshold(), 0.0f);
    EXPECT_FLOAT_EQ(world.SpeculativeContactDistance(), 0.04f);
}

TEST(PhysicsWorldTest, CcdMotionThresholdRejectsInvalidValues) {
    PhysicsWorld world;
    world.SetCcdMotionThreshold(0.03f);
    for (const float invalid : {-0.01f, std::numeric_limits<float>::infinity(),
                                 -std::numeric_limits<float>::infinity(),
                                 std::numeric_limits<float>::quiet_NaN()}) {
        EXPECT_THROW(world.SetCcdMotionThreshold(invalid), std::invalid_argument);
        EXPECT_FLOAT_EQ(world.CcdMotionThreshold(), 0.03f);
    }
}

TEST(PhysicsWorldTest, CcdActivationUsesItsOwnThresholdAndShapeSizeCutoff) {
    for (const float radius : {0.5f, 0.05f}) {
        for (const float band : {0.0f, 0.02f, 0.04f}) {
            for (const float threshold : {0.02f, 0.2f, 0.0f}) {
                SCOPED_TRACE(::testing::Message() << "radius=" << radius
                    << " band=" << band << " threshold=" << threshold);
                PhysicsWorld world;
                world.SetPhysicsWorkerCount(1);
                world.SetGravity(glm::vec3(0.0f));
                world.SetFixedTimeStep(1.0f / 60.0f);
                world.SetContinuousCollisionEnabled(true);
                world.SetCcdMotionThreshold(threshold);
                world.SetSpeculativeContactDistance(band);

                RigidBodyDesc wall;
                wall.isStatic = true;
                const uint32_t wallId = world.CreateRigidBody(wall);
                ColliderDesc wallCollider;
                wallCollider.shape = std::make_shared<BoxShape>(glm::vec3(0.01f, 1.0f, 1.0f));
                wallCollider.material.restitution = 0.0f;
                ASSERT_TRUE(world.AttachCollider(wallId, wallCollider));

                RigidBodyDesc moving;
                moving.position = glm::vec3(-radius - 0.06f, 0.0f, 0.0f);
                moving.linearVelocity = glm::vec3(6.0f, 0.0f, 0.0f);
                moving.useGravity = false;
                const uint32_t movingId = world.CreateRigidBody(moving);
                ColliderDesc movingCollider;
                movingCollider.shape = std::make_shared<SphereShape>(radius);
                movingCollider.material.restitution = 0.0f;
                ASSERT_TRUE(world.AttachCollider(movingId, movingCollider));

                // Initial gap 0.05 exceeds every band; travel 0.1 exceeds the small-shape cutoff.
                world.Step(1.0f / 60.0f);
                const auto* body = world.GetRigidBody(movingId);
                ASSERT_NE(body, nullptr);
                const bool expectCcd = radius == 0.05f || threshold == 0.02f;
                if (expectCcd) {
                    EXPECT_GT(world.LastStepStats().ccdToiHitCount, 0u);
                    EXPECT_LE(body->Position().x, -radius - 0.009f);
                    EXPECT_NEAR(body->LinearVelocity().x, 0.0f, 1e-4f);
                } else {
                    EXPECT_EQ(world.LastStepStats().ccdToiHitCount, 0u);
                    EXPECT_NEAR(body->Position().x, -radius + 0.04f, 1e-5f);
                    EXPECT_FLOAT_EQ(body->LinearVelocity().x, 6.0f);
                }
            }
        }
    }
}

TEST(CollisionFeatureTest, BoxAlwaysAnswersDominantFace) {
    // Jolt-style contract (BoxShape::GetSupportingFace): a box answers every
    // direction with the face whose normal is most aligned, even for
    // vertex/edge-ish directions. Tolerance-based feature collection
    // degenerates faces under the ~1e-3 rad normal noise of distance
    // queries, collapsing contact manifolds to single points; off-plane
    // vertices are rejected later by the manifold clip's separation filter.
    BoxShape box(glm::vec3(0.5f));
    Runtime::Physics::ShapeTransform transform;

    for (const glm::vec3 direction : {glm::vec3(1.0f, 1.0f, 1.0f),
                                      glm::vec3(1.0f, 1.0f, 0.0f),
                                      glm::vec3(1.0f, 0.0f, 0.0f)}) {
        const auto feature = box.GetSupportFeature(transform, direction);
        EXPECT_EQ(feature.type, SupportFeatureType::Face);
        ASSERT_EQ(feature.vertices.size(), 4u);
        // The dominant axis is +x in all three cases: every returned vertex
        // lies on the +x face plane (x = +0.5, y/z spanning both signs).
        for (const glm::vec3& vertex : feature.vertices) {
            EXPECT_FLOAT_EQ(vertex.x, 0.5f);
        }
    }
}

TEST(ConvexHullShapeTest, SupportUsesLocalVerticesAndWorldTransform) {
    ConvexHullShape hull({
        {2.0f, 0.0f, 0.0f},
        {-1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    });

    ShapeTransform transform;
    transform.position = {3.0f, 4.0f, 5.0f};
    transform.orientation = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    const glm::vec3 support = hull.Support(transform, glm::vec3(0.0f, 1.0f, 0.0f));
    EXPECT_NEAR(support.x, 3.0f, 1e-5f);
    EXPECT_NEAR(support.y, 6.0f, 1e-5f);
    EXPECT_NEAR(support.z, 5.0f, 1e-5f);
}

TEST(ConvexHullShapeTest, ComputesAabbFromRotatedVertices) {
    std::vector<glm::vec3> vertices;
    for (float x : {-1.0f, 1.0f}) {
        for (float y : {-2.0f, 2.0f}) {
            for (float z : {-3.0f, 3.0f}) {
                vertices.emplace_back(x, y, z);
            }
        }
    }
    ConvexHullShape hull(std::move(vertices));

    ShapeTransform transform;
    transform.position = {4.0f, 5.0f, 6.0f};
    transform.orientation = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    const auto bounds = hull.ComputeAABB(transform);
    EXPECT_NEAR(bounds.min.x, 2.0f, 1e-5f);
    EXPECT_NEAR(bounds.min.y, 4.0f, 1e-5f);
    EXPECT_NEAR(bounds.min.z, 3.0f, 1e-5f);
    EXPECT_NEAR(bounds.max.x, 6.0f, 1e-5f);
    EXPECT_NEAR(bounds.max.y, 6.0f, 1e-5f);
    EXPECT_NEAR(bounds.max.z, 9.0f, 1e-5f);
}

TEST(ConvexHullShapeTest, ParticipatesInCollisionDetection) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f));

    RigidBodyDesc hullBodyDesc;
    hullBodyDesc.useGravity = false;
    const uint32_t hullBodyId = world.CreateRigidBody(hullBodyDesc);

    RigidBodyDesc sphereBodyDesc;
    sphereBodyDesc.position = {0.5f, 0.0f, 0.0f};
    sphereBodyDesc.useGravity = false;
    const uint32_t sphereBodyId = world.CreateRigidBody(sphereBodyDesc);

    ColliderDesc hullCollider;
    hullCollider.shape = std::make_shared<ConvexHullShape>(std::vector<glm::vec3>{
        {1.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}, {0.0f, -1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f},
    });
    hullCollider.isTrigger = true;
    ASSERT_TRUE(world.AttachCollider(hullBodyId, hullCollider));

    ColliderDesc sphereCollider;
    sphereCollider.shape = std::make_shared<SphereShape>(0.75f);
    ASSERT_TRUE(world.AttachCollider(sphereBodyId, sphereCollider));

    world.Step(1.0f / 60.0f);

    ASSERT_EQ(world.Contacts().size(), 1u);
    EXPECT_TRUE(world.Contacts().front().isTrigger);
    EXPECT_GT(world.Contacts().front().Point(0).penetration, 0.0f);
}

TEST(ConvexHullShapeTest, RejectsSeparatedPair) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f));

    RigidBodyDesc hullBodyDesc;
    hullBodyDesc.useGravity = false;
    const uint32_t hullBodyId = world.CreateRigidBody(hullBodyDesc);

    RigidBodyDesc sphereBodyDesc;
    sphereBodyDesc.position = {5.0f, 0.0f, 0.0f};
    sphereBodyDesc.useGravity = false;
    const uint32_t sphereBodyId = world.CreateRigidBody(sphereBodyDesc);

    ColliderDesc hullCollider;
    hullCollider.shape = std::make_shared<ConvexHullShape>(std::vector<glm::vec3>{
        {1.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}, {0.0f, -1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f},
    });
    ASSERT_TRUE(world.AttachCollider(hullBodyId, hullCollider));

    ColliderDesc sphereCollider;
    sphereCollider.shape = std::make_shared<SphereShape>(0.75f);
    ASSERT_TRUE(world.AttachCollider(sphereBodyId, sphereCollider));

    world.Step(1.0f / 60.0f);
    EXPECT_TRUE(world.Contacts().empty());
}

TEST(ConvexHullShapeTest, TetrahedronOverlapsBox) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f));

    RigidBodyDesc hullBodyDesc;
    hullBodyDesc.orientation = glm::angleAxis(
        glm::radians(22.0f),
        glm::normalize(glm::vec3(1.0f, 1.0f, 0.5f)));
    hullBodyDesc.useGravity = false;
    const uint32_t hullBodyId = world.CreateRigidBody(hullBodyDesc);

    RigidBodyDesc boxBodyDesc;
    boxBodyDesc.position = {0.45f, 0.0f, 0.0f};
    boxBodyDesc.useGravity = false;
    const uint32_t boxBodyId = world.CreateRigidBody(boxBodyDesc);

    ColliderDesc hullCollider;
    hullCollider.shape = std::make_shared<ConvexHullShape>(std::vector<glm::vec3>{
        {0.8f, 0.8f, 0.8f},
        {-0.8f, -0.8f, 0.8f},
        {-0.8f, 0.8f, -0.8f},
        {0.8f, -0.8f, -0.8f},
    });
    hullCollider.isTrigger = true;
    ASSERT_TRUE(world.AttachCollider(hullBodyId, hullCollider));

    ColliderDesc boxCollider;
    boxCollider.shape = std::make_shared<BoxShape>(glm::vec3(0.5f));
    ASSERT_TRUE(world.AttachCollider(boxBodyId, boxCollider));

    world.Step(1.0f / 60.0f);

    ASSERT_EQ(world.Contacts().size(), 1u);
    EXPECT_TRUE(world.Contacts().front().isTrigger);
    EXPECT_GT(world.Contacts().front().Point(0).penetration, 0.0f);
}

TEST(ConvexHullShapeTest, RotatedOctahedronOverlapsSphere) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f));

    RigidBodyDesc hullBodyDesc;
    hullBodyDesc.orientation = glm::angleAxis(
        glm::radians(37.0f),
        glm::normalize(glm::vec3(0.5f, 1.0f, 1.0f)));
    hullBodyDesc.useGravity = false;
    const uint32_t hullBodyId = world.CreateRigidBody(hullBodyDesc);

    RigidBodyDesc sphereBodyDesc;
    sphereBodyDesc.position = {0.55f, 0.15f, 0.0f};
    sphereBodyDesc.useGravity = false;
    const uint32_t sphereBodyId = world.CreateRigidBody(sphereBodyDesc);

    ColliderDesc hullCollider;
    hullCollider.shape = std::make_shared<ConvexHullShape>(std::vector<glm::vec3>{
        {1.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}, {0.0f, -1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f},
    });
    hullCollider.isTrigger = true;
    ASSERT_TRUE(world.AttachCollider(hullBodyId, hullCollider));

    ColliderDesc sphereCollider;
    sphereCollider.shape = std::make_shared<SphereShape>(0.4f);
    ASSERT_TRUE(world.AttachCollider(sphereBodyId, sphereCollider));

    world.Step(1.0f / 60.0f);

    ASSERT_EQ(world.Contacts().size(), 1u);
    EXPECT_TRUE(world.Contacts().front().isTrigger);
    EXPECT_GT(world.Contacts().front().Point(0).penetration, 0.0f);
}

TEST(ConvexHullShapeTest, RotatedConvexHullsOverlap) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f));

    const std::vector<glm::vec3> vertices{
        {1.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}, {0.0f, -1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f},
    };

    RigidBodyDesc bodyDescA;
    bodyDescA.orientation = glm::angleAxis(glm::radians(25.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    bodyDescA.useGravity = false;
    const uint32_t bodyIdA = world.CreateRigidBody(bodyDescA);

    RigidBodyDesc bodyDescB;
    bodyDescB.position = {0.65f, 0.1f, 0.0f};
    bodyDescB.orientation = glm::angleAxis(
        glm::radians(-31.0f),
        glm::normalize(glm::vec3(1.0f, 0.5f, 1.0f)));
    bodyDescB.useGravity = false;
    const uint32_t bodyIdB = world.CreateRigidBody(bodyDescB);

    ColliderDesc colliderA;
    colliderA.shape = std::make_shared<ConvexHullShape>(vertices);
    colliderA.isTrigger = true;
    ASSERT_TRUE(world.AttachCollider(bodyIdA, colliderA));

    ColliderDesc colliderB;
    colliderB.shape = std::make_shared<ConvexHullShape>(vertices);
    ASSERT_TRUE(world.AttachCollider(bodyIdB, colliderB));

    world.Step(1.0f / 60.0f);

    ASSERT_EQ(world.Contacts().size(), 1u);
    EXPECT_TRUE(world.Contacts().front().isTrigger);
    EXPECT_GT(world.Contacts().front().Point(0).penetration, 0.0f);
}

TEST(ConvexHullShapeTest, RejectsDisjointHullsWithOverlappingAabbs) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f));

    const std::vector<glm::vec3> simplex{
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    };

    RigidBodyDesc bodyDescA;
    bodyDescA.useGravity = false;
    const uint32_t bodyIdA = world.CreateRigidBody(bodyDescA);

    RigidBodyDesc bodyDescB;
    bodyDescB.position = {0.6f, 0.6f, 0.6f};
    bodyDescB.useGravity = false;
    const uint32_t bodyIdB = world.CreateRigidBody(bodyDescB);

    ColliderDesc colliderA;
    colliderA.shape = std::make_shared<ConvexHullShape>(simplex);
    colliderA.isTrigger = true;
    ASSERT_TRUE(world.AttachCollider(bodyIdA, colliderA));

    ColliderDesc colliderB;
    colliderB.shape = std::make_shared<ConvexHullShape>(simplex);
    ASSERT_TRUE(world.AttachCollider(bodyIdB, colliderB));

    ShapeTransform transformA;
    ShapeTransform transformB;
    transformB.position = bodyDescB.position;
    EXPECT_TRUE(colliderA.shape->ComputeAABB(transformA).Intersects(
        colliderB.shape->ComputeAABB(transformB)));

    world.Step(1.0f / 60.0f);
    EXPECT_TRUE(world.Contacts().empty());
}

TEST(ConvexHullShapeTest, EpaContactUsesClosestFaceWitnessPoints) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f));

    const std::vector<glm::vec3> cubeVertices{
        {-1.0f, -1.0f, -1.0f}, {-1.0f, -1.0f, 1.0f},
        {-1.0f, 1.0f, -1.0f}, {-1.0f, 1.0f, 1.0f},
        {1.0f, -1.0f, -1.0f}, {1.0f, -1.0f, 1.0f},
        {1.0f, 1.0f, -1.0f}, {1.0f, 1.0f, 1.0f},
    };

    RigidBodyDesc bodyDescA;
    bodyDescA.useGravity = false;
    const uint32_t bodyIdA = world.CreateRigidBody(bodyDescA);

    RigidBodyDesc bodyDescB;
    bodyDescB.position = {1.5f, 0.0f, 0.0f};
    bodyDescB.useGravity = false;
    const uint32_t bodyIdB = world.CreateRigidBody(bodyDescB);

    ColliderDesc colliderDesc;
    colliderDesc.shape = std::make_shared<ConvexHullShape>(cubeVertices);
    colliderDesc.isTrigger = true;
    ASSERT_TRUE(world.AttachCollider(bodyIdA, colliderDesc));
    ASSERT_TRUE(world.AttachCollider(bodyIdB, colliderDesc));

    world.Step(1.0f / 60.0f);

    ASSERT_EQ(world.Contacts().size(), 1u);
    const auto& contact = world.Contacts().front();
    ASSERT_EQ(contact.pointCount, 4u);
    glm::vec3 center(0.0f);
    for (std::size_t index = 0; index < contact.pointCount; ++index) {
        const auto& point = contact.Point(index);
        EXPECT_GT(point.penetration, 0.0f);
        EXPECT_NEAR(point.position.x, 0.75f, 1e-3f);
        center += point.position;
    }
    center /= static_cast<float>(contact.pointCount);
    EXPECT_NEAR(center.y, 0.0f, 1e-3f);
    EXPECT_NEAR(center.z, 0.0f, 1e-3f);
}

TEST(ConvexHullShapeTest, FaceContactBuildsFourPointManifold) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f));

    RigidBodyDesc aDesc;
    aDesc.position = {0.0f, 0.0f, 0.0f};
    aDesc.useGravity = false;
    const uint32_t aId = world.CreateRigidBody(aDesc);

    RigidBodyDesc bDesc;
    bDesc.position = {0.9f, 0.0f, 0.0f};
    bDesc.useGravity = false;
    const uint32_t bId = world.CreateRigidBody(bDesc);

    ColliderDesc collider;
    collider.shape = std::make_shared<BoxShape>(glm::vec3(0.5f));
    collider.isTrigger = true;
    ASSERT_TRUE(world.AttachCollider(aId, collider));
    ASSERT_TRUE(world.AttachCollider(bId, collider));

    world.Step(1.0f / 60.0f);

    ASSERT_EQ(world.Contacts().size(), 1u);
    const auto& contact = world.Contacts().front();
    EXPECT_EQ(contact.pointCount, 4u);
    for (std::size_t index = 0; index < contact.pointCount; ++index) {
        EXPECT_NEAR(contact.Point(index).penetration, 0.1f, 1e-3f);
    }
}

TEST(PhysicsWorldTest, DynamicBodyFallsUnderGravity) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 60.0f);

    RigidBodyDesc bodyDesc;
    bodyDesc.position = {0.0f, 10.0f, 0.0f};
    const uint32_t bodyId = world.CreateRigidBody(bodyDesc);

    world.Step(1.0f);

    const auto* body = world.GetRigidBody(bodyId);
    ASSERT_NE(body, nullptr);
    EXPECT_LT(body->Position().y, 10.0f);
    EXPECT_LT(body->LinearVelocity().y, 0.0f);
}

TEST(PhysicsWorldTest, StaticBodyDoesNotMove) {
    PhysicsWorld world;

    RigidBodyDesc groundDesc;
    groundDesc.position = {0.0f, 0.0f, 0.0f};
    groundDesc.isStatic = true;
    const uint32_t groundId = world.CreateRigidBody(groundDesc);

    world.Step(0.5f);

    const auto* body = world.GetRigidBody(groundId);
    ASSERT_NE(body, nullptr);
    EXPECT_FLOAT_EQ(body->Position().x, 0.0f);
    EXPECT_FLOAT_EQ(body->Position().y, 0.0f);
    EXPECT_FLOAT_EQ(body->Position().z, 0.0f);
}

TEST(PhysicsWorldTest, OverlapGeneratesContact) {
    PhysicsWorld world;

    RigidBodyDesc aDesc;
    aDesc.position = {0.0f, 0.0f, 0.0f};
    aDesc.useGravity = false;
    const uint32_t aId = world.CreateRigidBody(aDesc);

    RigidBodyDesc bDesc;
    bDesc.position = {0.5f, 0.0f, 0.0f};
    bDesc.useGravity = false;
    const uint32_t bId = world.CreateRigidBody(bDesc);

    ColliderDesc colliderDesc;
    colliderDesc.shape = std::make_shared<BoxShape>(glm::vec3(0.5f));

    ASSERT_TRUE(world.AttachCollider(aId, colliderDesc));
    ASSERT_TRUE(world.AttachCollider(bId, colliderDesc));

    world.Step(1.0f / 60.0f);

    const auto& contacts = world.Contacts();
    ASSERT_FALSE(contacts.empty());

    bool foundPair = false;
    for (const auto& contact : contacts) {
        const bool direct = contact.bodyA == aId && contact.bodyB == bId;
        const bool reverse = contact.bodyA == bId && contact.bodyB == aId;
        if (direct || reverse) {
            foundPair = true;
            EXPECT_GT(contact.Point(0).penetration, 0.0f);
        }
    }

    EXPECT_TRUE(foundPair);
}

TEST(PhysicsWorldTest, DetectsOnceAndSeparatesSolverPassesPerSubstep) {
    PhysicsWorld world;
    world.SetGravity(glm::vec3(0.0f));
    world.SetContinuousCollisionEnabled(false);
    world.SetSolverIterations(4);

    RigidBodyDesc bodyDescA;
    bodyDescA.useGravity = false;
    const uint32_t bodyIdA = world.CreateRigidBody(bodyDescA);

    RigidBodyDesc bodyDescB;
    bodyDescB.position = {0.5f, 0.0f, 0.0f};
    bodyDescB.useGravity = false;
    const uint32_t bodyIdB = world.CreateRigidBody(bodyDescB);

    ColliderDesc colliderDesc;
    colliderDesc.shape = std::make_shared<BoxShape>(glm::vec3(0.5f));
    ASSERT_TRUE(world.AttachCollider(bodyIdA, colliderDesc));
    ASSERT_TRUE(world.AttachCollider(bodyIdB, colliderDesc));

    world.Step(world.FixedTimeStep());

    const auto& stats = world.LastStepStats();
    EXPECT_EQ(stats.fixedStepCount, 1u);
    EXPECT_EQ(stats.collisionDetectionPassCount, 1u);
    EXPECT_EQ(stats.velocitySolverPassCount, 4u);
    EXPECT_EQ(stats.positionSolverPassCount, 1u);
    EXPECT_EQ(stats.broadPhaseCandidateCount, 1u);
    EXPECT_EQ(stats.narrowPhaseTestCount, 1u);
    EXPECT_EQ(stats.staticBvhLeafCount, 0u);
    EXPECT_EQ(stats.dynamicBvhLeafCount, 2u);
    EXPECT_EQ(stats.satCallCount, 1u);
    EXPECT_EQ(stats.gjkCallCount, 0u);
    EXPECT_EQ(stats.gjkFailureCount, 0u);
    EXPECT_EQ(stats.epaCallCount, 0u);
    EXPECT_EQ(stats.epaFailureCount, 0u);
    EXPECT_EQ(stats.manifoldCount, 1u);
    EXPECT_GT(stats.contactPointCount, 0u);
}

TEST(PhysicsWorldTest, AngularVelocityUpdatesOrientation) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);

    RigidBodyDesc bodyDesc;
    bodyDesc.position = {0.0f, 0.0f, 0.0f};
    bodyDesc.useGravity = false;
    bodyDesc.angularVelocity = {0.0f, 2.5f, 0.0f};
    bodyDesc.inertiaTensorDiagonal = {1.0f, 1.0f, 1.0f};
    const uint32_t bodyId = world.CreateRigidBody(bodyDesc);

    world.Step(0.5f);

    const auto* body = world.GetRigidBody(bodyId);
    ASSERT_NE(body, nullptr);
    const glm::quat q = body->Orientation();
    EXPECT_GT(std::abs(q.y), 0.05f);
    EXPECT_NEAR(glm::length(q), 1.0f, 1e-3f);
}

TEST(PhysicsWorldTest, RotatedBoxAndSphereGenerateContact) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetGravity(glm::vec3(0.0f));

    RigidBodyDesc boxDesc;
    boxDesc.position = {0.0f, 0.0f, 0.0f};
    boxDesc.orientation = glm::angleAxis(glm::radians(35.0f), glm::normalize(glm::vec3(0.0f, 1.0f, 1.0f)));
    boxDesc.useGravity = false;
    const uint32_t boxId = world.CreateRigidBody(boxDesc);

    RigidBodyDesc sphereDesc;
    sphereDesc.position = {0.42f, 0.1f, 0.0f};
    sphereDesc.useGravity = false;
    const uint32_t sphereId = world.CreateRigidBody(sphereDesc);

    ColliderDesc boxCollider;
    boxCollider.shape = std::make_shared<BoxShape>(glm::vec3(0.5f));
    ASSERT_TRUE(world.AttachCollider(boxId, boxCollider));

    ColliderDesc sphereCollider;
    sphereCollider.shape = std::make_shared<SphereShape>(0.35f);
    ASSERT_TRUE(world.AttachCollider(sphereId, sphereCollider));

    world.Step(1.0f / 60.0f);

    const auto& contacts = world.Contacts();
    bool found = false;
    for (const auto& c : contacts) {
        const bool match = (c.bodyA == boxId && c.bodyB == sphereId) || (c.bodyA == sphereId && c.bodyB == boxId);
        if (!match) {
            continue;
        }
        found = true;
        EXPECT_GT(c.Point(0).penetration, 0.0f);
        EXPECT_GT(glm::length(c.normal), 0.5f);
    }

    EXPECT_TRUE(found);
}

TEST(PhysicsWorldTest, SpinOnGroundGeneratesSidewaysVelocity) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));

    RigidBodyDesc groundDesc;
    groundDesc.position = {0.0f, -0.2f, 0.0f};
    groundDesc.isStatic = true;
    groundDesc.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(groundDesc);

    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(3.0f, 0.2f, 3.0f));
    groundCollider.material.dynamicFriction = 0.98f;
    groundCollider.material.staticFriction = 0.99f;
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    RigidBodyDesc sphereDesc;
    sphereDesc.position = {0.0f, 1.2f, 0.0f};
    sphereDesc.mass = 1.0f;
    sphereDesc.inertiaTensorDiagonal = glm::vec3(0.4f * sphereDesc.mass * 0.25f * 0.25f);
    sphereDesc.angularVelocity = {0.0f, 0.0f, 35.0f};
    const uint32_t sphereId = world.CreateRigidBody(sphereDesc);

    ColliderDesc sphereCollider;
    sphereCollider.shape = std::make_shared<SphereShape>(0.25f);
    sphereCollider.material.dynamicFriction = 0.95f;
    sphereCollider.material.staticFriction = 0.98f;
    sphereCollider.material.restitution = 0.2f;
    ASSERT_TRUE(world.AttachCollider(sphereId, sphereCollider));

    world.Step(1.2f);

    const auto* sphere = world.GetRigidBody(sphereId);
    ASSERT_NE(sphere, nullptr);
    EXPECT_GT(std::abs(sphere->LinearVelocity().x), 0.25f);
    EXPECT_GT(std::abs(sphere->Position().x), 0.12f);
}

TEST(PhysicsWorldTest, CcdPreventsHighSpeedTunneling) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 60.0f);
    world.SetGravity(glm::vec3(0.0f));
    world.SetContinuousCollisionEnabled(true);
    world.SetCcdMaxSubSteps(12);

    RigidBodyDesc groundDesc;
    groundDesc.position = {0.0f, -0.05f, 0.0f};
    groundDesc.isStatic = true;
    groundDesc.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(groundDesc);

    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(10.0f, 0.05f, 10.0f));
    groundCollider.material.restitution = 0.0f;
    groundCollider.material.dynamicFriction = 0.8f;
    groundCollider.material.staticFriction = 0.9f;
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    RigidBodyDesc sphereDesc;
    sphereDesc.position = {0.0f, 2.0f, 0.0f};
    sphereDesc.linearVelocity = {0.0f, -450.0f, 0.0f};
    sphereDesc.mass = 1.0f;
    sphereDesc.useGravity = false;
    const uint32_t sphereId = world.CreateRigidBody(sphereDesc);

    ColliderDesc sphereCollider;
    sphereCollider.shape = std::make_shared<SphereShape>(0.25f);
    sphereCollider.material.restitution = 0.2f;
    sphereCollider.material.dynamicFriction = 0.8f;
    sphereCollider.material.staticFriction = 0.9f;
    ASSERT_TRUE(world.AttachCollider(sphereId, sphereCollider));

    world.Step(1.0f / 60.0f);

    const auto* sphere = world.GetRigidBody(sphereId);
    ASSERT_NE(sphere, nullptr);
    const float groundTop = groundDesc.position.y + 0.05f;
    EXPECT_GT(sphere->Position().y, groundTop - 0.26f);
}

TEST(PhysicsWorldTest, SpinningSphereDoesNotSinkBelowGroundTop) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetGravity(glm::vec3(0.0f, -10.5f, 0.0f));
    world.SetContinuousCollisionEnabled(true);
    world.SetCcdMaxSubSteps(12);

    RigidBodyDesc groundDesc;
    groundDesc.position = {0.0f, -1.625f, 0.0f};
    groundDesc.isStatic = true;
    groundDesc.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(groundDesc);

    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(60.0f, 0.6f, 60.0f));
    groundCollider.material.dynamicFriction = 0.95f;
    groundCollider.material.staticFriction = 0.98f;
    groundCollider.material.restitution = 0.12f;
    groundCollider.oneSided = true;
    groundCollider.oneSidedNormalLocal = glm::vec3(0.0f, 1.0f, 0.0f);
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    RigidBodyDesc sphereDesc;
    sphereDesc.position = {0.0f, 1.8f, 0.0f};
    sphereDesc.mass = 1.0f;
    sphereDesc.angularVelocity = {0.0f, 0.0f, 28.0f};
    sphereDesc.inertiaTensorDiagonal = glm::vec3(0.4f * sphereDesc.mass * 0.45f * 0.45f);
    const uint32_t sphereId = world.CreateRigidBody(sphereDesc);

    ColliderDesc sphereCollider;
    sphereCollider.shape = std::make_shared<SphereShape>(0.45f);
    sphereCollider.material.dynamicFriction = 0.92f;
    sphereCollider.material.staticFriction = 0.95f;
    sphereCollider.material.restitution = 0.62f;
    ASSERT_TRUE(world.AttachCollider(sphereId, sphereCollider));

    world.Step(2.0f);

    const auto* sphere = world.GetRigidBody(sphereId);
    ASSERT_NE(sphere, nullptr);
    const float groundTop = -1.625f + 0.6f;
    EXPECT_GT(sphere->Position().y, groundTop - 0.55f);
}

TEST(PhysicsWorldTest, CcdSubStepsConsumeForceAcrossFullFixedStep) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 60.0f);
    world.SetGravity(glm::vec3(0.0f));
    world.SetContinuousCollisionEnabled(true);
    world.SetCcdMaxSubSteps(12);

    RigidBodyDesc groundDesc;
    groundDesc.position = {0.0f, -0.05f, 0.0f};
    groundDesc.isStatic = true;
    groundDesc.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(groundDesc);

    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(10.0f, 0.05f, 10.0f));
    groundCollider.material.restitution = 0.0f;
    groundCollider.material.dynamicFriction = 0.0f;
    groundCollider.material.staticFriction = 0.0f;
    ASSERT_TRUE(world.AttachCollider(groundId, groundCollider));

    RigidBodyDesc sphereDesc;
    sphereDesc.position = {0.0f, 2.0f, 0.0f};
    sphereDesc.linearVelocity = {0.0f, -450.0f, 0.0f};
    sphereDesc.mass = 1.0f;
    sphereDesc.useGravity = false;
    const uint32_t sphereId = world.CreateRigidBody(sphereDesc);

    ColliderDesc sphereCollider;
    sphereCollider.shape = std::make_shared<SphereShape>(0.25f);
    sphereCollider.material.restitution = 0.0f;
    sphereCollider.material.dynamicFriction = 0.0f;
    sphereCollider.material.staticFriction = 0.0f;
    ASSERT_TRUE(world.AttachCollider(sphereId, sphereCollider));

    // CCD splits this fixed step at the ground impact (TOI ~4 ms into the step).
    // The locked external force must be integrated by every sub-step with its own
    // dt, i.e. over the full fixed step: dv = F / m * dt = 3000 / 60 = 50 m/s.
    auto* sphere = world.GetRigidBody(sphereId);
    ASSERT_NE(sphere, nullptr);
    sphere->ApplyForce(glm::vec3(3000.0f, 0.0f, 0.0f));
    world.Step(1.0f / 60.0f);

    EXPECT_NEAR(sphere->LinearVelocity().x, 50.0f, 2.0f);

    // Forces are cleared once at fixed-step end: a following step without a new
    // force must not gain any extra acceleration.
    world.Step(1.0f / 60.0f);
    EXPECT_NEAR(sphere->LinearVelocity().x, 50.0f, 2.0f);
}

// Phase 10 step 38: a fast-rotating long bar must be CCD-scanned by its
// swept ROTATIONAL extent, not just its center-of-mass translation. A bar
// that barely translates but whose tip sweeps through a target must not
// tunnel: the tip travels |w| * (halfLength) this step, far beyond the
// center-of-mass motion.
TEST(PhysicsWorldTest, CcdCatchesFastRotatingLongBar) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetGravity(glm::vec3(0.0f));
    world.SetContinuousCollisionEnabled(true);
    world.SetCcdMaxSubSteps(12);

    // A static sphere sitting off the bar's rotation axis, on the tip's swept
    // arc reachable THIS step. The bar spins 40 rad/s (0.33 rad per step) and
    // the tip reaches 2*sin(0.33) ~ 0.65 m. The sphere at (0.5, 0.4) sits on
    // the swept annulus: at x=0.5 the bar surface rises to ~0.4 m mid-step.
    // Away from the axis, so the rotating bar sweeps through it.
    RigidBodyDesc targetDesc;
    targetDesc.position = {0.5f, 0.4f, 0.0f};
    targetDesc.isStatic = true;
    targetDesc.useGravity = false;
    const uint32_t targetId = world.CreateRigidBody(targetDesc);
    ColliderDesc targetCollider;
    targetCollider.shape = std::make_shared<SphereShape>(0.3f);
    targetCollider.material.restitution = 0.0f;
    ASSERT_TRUE(world.AttachCollider(targetId, targetCollider));

    // A long bar spinning about Z at 40 rad/s: its tip (half length 2 m)
    // sweeps at 80 m/s, crossing the target within this step, while the bar's
    // center of mass does not move at all.
    RigidBodyDesc barDesc;
    barDesc.position = {0.0f, 0.0f, 0.0f};
    barDesc.angularVelocity = {0.0f, 0.0f, 40.0f};
    barDesc.mass = 1.0f;
    barDesc.inertiaTensorDiagonal = glm::vec3(1.0f);
    barDesc.useGravity = false;
    const uint32_t barId = world.CreateRigidBody(barDesc);
    ColliderDesc barCollider;
    barCollider.shape = std::make_shared<BoxShape>(glm::vec3(2.0f, 0.1f, 0.1f));
    barCollider.material.restitution = 0.0f;
    ASSERT_TRUE(world.AttachCollider(barId, barCollider));

    world.Step(1.0f / 120.0f);

    // The tip sweeps 40 * 2 * dt = 0.67 m this step and must strike the
    // target sphere: either a contact was published for the pair, or the
    // bar's angular momentum was absorbed by the collision (rotation slowed).
    bool sawContact = false;
    for (const auto& contact : world.Contacts()) {
        if (contact.bodyA == barId || contact.bodyB == barId) {
            if (contact.bodyA == targetId || contact.bodyB == targetId) {
                sawContact = true;
            }
        }
    }
    const auto* bar = world.GetRigidBody(barId);
    ASSERT_NE(bar, nullptr);
    const float slowed = glm::length(bar->AngularVelocity()) < 39.0f;
    EXPECT_TRUE(sawContact || slowed)
        << "rotating bar tip tunneled through the target (no contact, no slowdown)";
}
