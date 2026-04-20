#include <gtest/gtest.h>

#include <memory>

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
