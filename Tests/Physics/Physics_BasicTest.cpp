#include <gtest/gtest.h>

#include <memory>

#include <glm/gtc/quaternion.hpp>

#include "Physics/CollisionShape.h"
#include "Physics/PhysicsWorld.h"

using Runtime::Physics::BoxShape;
using Runtime::Physics::ColliderDesc;
using Runtime::Physics::PhysicsWorld;
using Runtime::Physics::RigidBodyDesc;
using Runtime::Physics::SphereShape;

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
            EXPECT_GT(contact.point.penetration, 0.0f);
        }
    }

    EXPECT_TRUE(foundPair);
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
        EXPECT_GT(c.point.penetration, 0.0f);
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
