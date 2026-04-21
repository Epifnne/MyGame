#pragma once

#include <ECS/World.h>
#include <ECS/Entity.h>
#include <Graphics/Mesh.h>
#include <Graphics/MeshManager.h>
#include <Graphics/Material.h>
#include <Graphics/Camera.h>
#include <Graphics/TextureManager.h>
#include <Physics/PhysicsWorld.h>
#include <Core/Input.h>
#include <Resource/ResourceManager.h>

namespace Runtime { namespace Core { class Engine; } }

#include <glm/glm.hpp>
#include <memory>

namespace Game {
namespace Core {

class GameApp {
public:
    bool Initialize(Runtime::Core::Engine& engine);
    void Update(float dt, const Runtime::Core::Input& input);
    void Render(Runtime::Core::Engine& engine);
    void Shutdown();
    Runtime::Resource::ResourceManager& GetResourceManager() { return m_resourceManager; }
    const Runtime::Resource::ResourceManager& GetResourceManager() const { return m_resourceManager; }

    bool ShouldExit() const { return m_shouldExit; }

private:
    Runtime::ECS::World m_world;
    Runtime::ECS::Entity m_cubeEntity = Runtime::ECS::NullEntity;

    std::shared_ptr<Runtime::Graphics::Mesh> m_mesh;
    std::shared_ptr<Runtime::Graphics::Mesh> m_ballMesh;
    Runtime::Graphics::MeshManager m_meshManager;
    Runtime::Graphics::TextureManager m_textureManager;
    Runtime::Graphics::Material m_material;
    Runtime::Graphics::Material m_groundMaterial;
    Runtime::Graphics::Material m_ballMaterial;
    Runtime::Graphics::Camera m_camera;

    Runtime::Resource::ResourceManager m_resourceManager;
    Runtime::Physics::PhysicsWorld m_physicsWorld;
    uint32_t m_ballBodyId = 0;
    uint32_t m_groundBodyId = 0;

    glm::vec3 m_groundPosition = glm::vec3(0.0f, -1.625f, 0.0f);
    glm::vec3 m_groundScale = glm::vec3(16.0f, 1.2f, 16.0f);
    glm::vec3 m_ballVisualScale = glm::vec3(0.45f);
    float m_ballRadius = 0.45f;
    float m_contactSquash = 0.0f;
    float m_squashVelocity = 0.0f;
    float m_lastBallVelocityY = 0.0f;
    bool m_ballGroundContact = false;
    float m_ballTelemetryTimer = 0.0f;
    float m_ballTelemetryDuration = 6.0f;
    bool m_useAsyncResourceIO = true;

    bool m_shouldExit = false;
};

} // namespace Core
} // namespace Game
