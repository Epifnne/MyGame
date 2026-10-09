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
#include <Network/NetworkManager.h>
#include <Resource/ResourceManager.h>
#include <UI/UIManager.h>
#include <UI/Widgets/Button.h>
#include <UI/Widgets/Label.h>
#include <UI/Widgets/Panel.h>

namespace Runtime { namespace Core { class Engine; } }

#include <glm/glm.hpp>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

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
    struct FloatingTextEntry {
        std::string text;
        glm::vec3 worldPosition{0.0f, 0.0f, 0.0f};
        glm::vec4 color{1.0f, 1.0f, 1.0f, 1.0f};
        Runtime::UI::UITextRenderTechnique technique = Runtime::UI::UITextRenderTechnique::SDF;
    };

    bool InitializeUiOverlayRenderer();
    void ShutdownUiOverlayRenderer();
    void RenderUiOverlay(int viewportWidth, int viewportHeight);
    void RenderFloatingTextReserved(int viewportWidth, int viewportHeight);

public:
    // Reserved APIs for game-space floating text using SDF/MSDF backends.
    void QueueFloatingTextSdf(const std::string& text, const glm::vec3& worldPosition, const glm::vec4& color);
    void QueueFloatingTextMsdf(const std::string& text, const glm::vec3& worldPosition, const glm::vec4& color);

private:

    Runtime::ECS::World m_world;

    std::shared_ptr<Runtime::Graphics::Mesh> m_mesh;
    std::shared_ptr<Runtime::Graphics::Mesh> m_ballMesh;
    Runtime::Graphics::MeshManager m_meshManager;
    Runtime::Graphics::TextureManager m_textureManager;
    Runtime::Graphics::Material m_groundMaterial;
    Runtime::Graphics::Material m_ballMaterial;
    Runtime::Graphics::Camera m_camera;
    glm::vec3 m_cameraTarget = glm::vec3(0.0f, -0.2f, 0.0f);

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

    Runtime::Network::NetworkManager m_networkManager;
    Runtime::Network::Endpoint m_serverEndpoint{"127.0.0.1", 45000};
    bool m_networkStarted = false;
    bool m_sentInitialPing = false;
    float m_pingTimer = 0.0f;
    uint32_t m_pongCount = 0;

    Runtime::UI::UIManager m_uiManager;
    std::shared_ptr<Runtime::UI::Canvas> m_uiCanvas;
    std::shared_ptr<Runtime::UI::Panel> m_uiPanel;
    std::shared_ptr<Runtime::UI::Label> m_uiTitleLabel;
    std::shared_ptr<Runtime::UI::Label> m_uiStateLabel;
    std::shared_ptr<Runtime::UI::Button> m_uiStartButton;
    bool m_gameStarted = false;
    std::string m_windowTitle;
    uint64_t m_uiTickIndex = 0;
    int m_uiMousePixelX = 0;
    int m_uiMousePixelY = 0;
    std::string m_uiLastHitWidget;
    std::ofstream m_uiTickLog;

    unsigned int m_uiOverlayProgram = 0;
    unsigned int m_uiOverlayVao = 0;
    int m_uiOverlayRectLoc = -1;
    int m_uiOverlayColorLoc = -1;
    int m_uiOverlayViewportLoc = -1;

    unsigned int m_uiTextProgram = 0;
    unsigned int m_uiTextVao = 0;
    unsigned int m_uiTextVbo = 0;
    int m_uiTextViewportLoc = -1;
    int m_uiTextColorLoc = -1;

    std::vector<FloatingTextEntry> m_floatingTextQueue;

    bool m_shouldExit = false;
};

} // namespace Core
} // namespace Game
