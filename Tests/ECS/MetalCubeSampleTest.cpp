#include <gtest/gtest.h>

#include <ECS/Component.h>
#include <ECS/World.h>
#include "MetalCube/CubeRenderComponent.h"
#include "MetalCube/CubeRotationSystem.h"

TEST(MetalCubeSample, AdvancesRotationBySpeedAndDeltaTime) {
    Runtime::ECS::World world;
    const auto cube = world.CreateEntity();
    world.AddComponent<Runtime::ECS::Components::Transform3D>(cube);
    Runtime::ECS::Components::Rotation rotation;
    rotation.axis = glm::normalize(glm::vec3(0.5f, 1.0f, 0.0f));
    rotation.speed = 2.0f;
    world.AddComponent<Runtime::ECS::Components::Rotation>(cube, rotation);
    world.AddComponent<Sample::Components::CubeRenderComponent>(cube);

    Sample::Systems::CubeRotationSystem::Update(world, 0.25f);

    const auto& updated = world.RegistryRef().GetComponent<Runtime::ECS::Components::Rotation>(cube);
    const auto& transform = world.RegistryRef().GetComponent<Runtime::ECS::Components::Transform3D>(cube);
    EXPECT_FLOAT_EQ(updated.angle, 0.5f);
    EXPECT_FLOAT_EQ(transform.rotationEuler.x, rotation.axis.x * 0.5f);
    EXPECT_FLOAT_EQ(transform.rotationEuler.y, rotation.axis.y * 0.5f);
    EXPECT_FLOAT_EQ(transform.rotationEuler.z, rotation.axis.z * 0.5f);
    EXPECT_EQ(transform.position, glm::vec3(0.0f));
    EXPECT_EQ(transform.scale, glm::vec3(1.0f));

    Sample::Systems::CubeRotationSystem::Update(world, 0.0f);
    EXPECT_FLOAT_EQ(updated.angle, 0.5f);
}

TEST(MetalCubeSample, SkipsEntitiesWithoutCubeMarkerOrTransform) {
    Runtime::ECS::World world;
    const auto nonCube = world.CreateEntity();
    world.AddComponent<Runtime::ECS::Components::Rotation>(nonCube);
    world.AddComponent<Runtime::ECS::Components::Transform3D>(nonCube);
    const auto noTransform = world.CreateEntity();
    world.AddComponent<Runtime::ECS::Components::Rotation>(noTransform);
    world.AddComponent<Sample::Components::CubeRenderComponent>(noTransform);
    const auto noRotation = world.CreateEntity();
    world.AddComponent<Runtime::ECS::Components::Transform3D>(noRotation);
    world.AddComponent<Sample::Components::CubeRenderComponent>(noRotation);

    Sample::Systems::CubeRotationSystem::Update(world, 1.0f);

    EXPECT_FLOAT_EQ(world.RegistryRef().GetComponent<Runtime::ECS::Components::Rotation>(nonCube).angle, 0.0f);
    EXPECT_FLOAT_EQ(world.RegistryRef().GetComponent<Runtime::ECS::Components::Rotation>(noTransform).angle, 0.0f);
    EXPECT_EQ(world.RegistryRef().GetComponent<Runtime::ECS::Components::Transform3D>(noRotation).rotationEuler,
        glm::vec3(0.0f));
}
