#include <gtest/gtest.h>

#include "ECS/Component.h"
#include "ECS/Entity.h"
#include "ECS/World.h"

using Runtime::ECS::NullEntity;
using Runtime::ECS::World;
using Runtime::ECS::Components::Transform2D;

TEST(EcsWorldTest, AddGetAndDestroyComponent) {
    World world;

    const Runtime::ECS::Entity e = world.CreateEntity();
    EXPECT_NE(e, NullEntity);

    world.AddComponent<Transform2D>(e, Transform2D{1.0f, 2.0f});
    EXPECT_TRUE(world.HasComponent<Transform2D>(e));

    auto& t = world.RegistryRef().GetComponent<Transform2D>(e);
    EXPECT_FLOAT_EQ(t.x, 1.0f);
    EXPECT_FLOAT_EQ(t.y, 2.0f);

    world.DestroyEntity(e);
    EXPECT_FALSE(world.HasComponent<Transform2D>(e));
}

TEST(EcsWorldTest, NullEntityCannotReceiveComponent) {
    World world;

    // Null entity should stay untouched even if passed by mistake.
    world.AddComponent<Transform2D>(NullEntity, Transform2D{3.0f, 4.0f});
    EXPECT_FALSE(world.HasComponent<Transform2D>(NullEntity));
}

