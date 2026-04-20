#include <gtest/gtest.h>

#include <glm/glm.hpp>

#include "Graphics/Camera.h"
#include "Graphics/Material.h"
#include "Graphics/MeshManager.h"
#include "Graphics/TextureManager.h"

using Runtime::Graphics::Camera;
using Runtime::Graphics::Material;
using Runtime::Graphics::MeshManager;
using Runtime::Graphics::TextureManager;
using Runtime::Graphics::MeshHandle;
using Runtime::Graphics::TextureHandle;

TEST(GraphicsCameraTest, ProjectionAndViewAreConfigurable) {
    Camera cam;

    cam.SetPerspective(glm::radians(60.0f), 16.0f / 9.0f, 0.1f, 500.0f);
    const glm::mat4 perspective = cam.GetProjection();

    cam.SetOrthographic(-10.0f, 10.0f, -5.0f, 5.0f, 0.1f, 100.0f);
    const glm::mat4 orthographic = cam.GetProjection();

    EXPECT_NE(perspective, orthographic);

    cam.SetPosition(glm::vec3(2.0f, 3.0f, 4.0f));
    cam.SetTarget(glm::vec3(0.0f, 0.0f, 0.0f));

    const glm::mat4 view = cam.GetView();
    EXPECT_NE(view, glm::mat4(1.0f));
    EXPECT_EQ(cam.GetPosition(), glm::vec3(2.0f, 3.0f, 4.0f));
}

TEST(GraphicsMaterialTest, StoresPassesAndDefaultsToNoTextures) {
    Material material;
    EXPECT_EQ(material.GetAlbedo(), nullptr);
    EXPECT_EQ(material.GetNormal(), nullptr);
    EXPECT_EQ(material.GetOrm(), nullptr);

    Material::Pass extraPass;
    extraPass.name = "ShadowLikePass";
    extraPass.enabled = true;
    material.AddPass(extraPass);

    ASSERT_EQ(material.GetPassCount(), 1u);
    const Material::Pass* pass = material.GetPass(0);
    ASSERT_NE(pass, nullptr);
    EXPECT_EQ(pass->name, "ShadowLikePass");
    EXPECT_TRUE(pass->enabled);
}

TEST(GraphicsTextureManagerTest, AsyncReadWithoutResolverReturnsNullopt) {
    TextureManager manager;
    auto fut = manager.ReadBytesAsync(TextureHandle{1234});
    auto bytes = fut.get();
    EXPECT_FALSE(bytes.has_value());
}

TEST(GraphicsMeshManagerTest, AsyncReadWithoutResolverReturnsNullopt) {
    MeshManager manager;
    auto fut = manager.ReadBytesAsync(MeshHandle{5678});
    auto bytes = fut.get();
    EXPECT_FALSE(bytes.has_value());
}
