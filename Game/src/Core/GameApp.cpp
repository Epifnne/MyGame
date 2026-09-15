#include "Core/GameApp.h"

#include "Components/TransformComponent.h"
#include "Components/VelocityComponent.h"
#include "Components/Transform3DComponent.h"
#include "Components/RotationComponent.h"
#include "Components/CubeRenderComponent.h"
#include "Systems/RotationSystem.h"
#include "Systems/CubeRenderSystem.h"
#include "Systems/MovementSystem.h"

#include <Core/Engine.h>
#include <Core/Input.h>
#include <Graphics/Renderer.h>
#include <Graphics/MeshManager.h>
#include <Graphics/TextureManager.h>
#include <Physics/Collider.h>
#include <Physics/CollisionShape.h>
#include <Physics/RigidBody.h>
#include <Platform/Window.h>
#include <Resource/ResourceManager.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <future>
#include <glad/gl.h>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <stb_easy_font.h>

namespace {

std::vector<float> BuildSphereVertices(int stacks, int slices) {
    std::vector<float> vertices;
    vertices.reserve(static_cast<size_t>(stacks * slices * 6) * 6);

    constexpr float pi = 3.14159265358979323846f;

    auto addVertex = [&vertices](const glm::vec3& p) {
        const glm::vec3 n = glm::normalize(p);
        vertices.push_back(p.x);
        vertices.push_back(p.y);
        vertices.push_back(p.z);
        vertices.push_back(n.x);
        vertices.push_back(n.y);
        vertices.push_back(n.z);
    };

    for (int i = 0; i < stacks; ++i) {
        const float v0 = static_cast<float>(i) / static_cast<float>(stacks);
        const float v1 = static_cast<float>(i + 1) / static_cast<float>(stacks);
        const float phi0 = pi * v0;
        const float phi1 = pi * v1;

        for (int j = 0; j < slices; ++j) {
            const float u0 = static_cast<float>(j) / static_cast<float>(slices);
            const float u1 = static_cast<float>(j + 1) / static_cast<float>(slices);
            const float theta0 = 2.0f * pi * u0;
            const float theta1 = 2.0f * pi * u1;

            const glm::vec3 p00(std::sin(phi0) * std::cos(theta0), std::cos(phi0), std::sin(phi0) * std::sin(theta0));
            const glm::vec3 p01(std::sin(phi0) * std::cos(theta1), std::cos(phi0), std::sin(phi0) * std::sin(theta1));
            const glm::vec3 p10(std::sin(phi1) * std::cos(theta0), std::cos(phi1), std::sin(phi1) * std::sin(theta0));
            const glm::vec3 p11(std::sin(phi1) * std::cos(theta1), std::cos(phi1), std::sin(phi1) * std::sin(theta1));

            addVertex(p00);
            addVertex(p10);
            addVertex(p11);

            addVertex(p00);
            addVertex(p11);
            addVertex(p01);
        }
    }

    return vertices;
}

const Runtime::Resource::AssetHandle kMetalVertHandle{1001};
const Runtime::Resource::AssetHandle kMetalFragHandle{1002};
const Runtime::Graphics::MeshHandle kCubeMeshHandle{2001};
const Runtime::Graphics::MeshHandle kBallMeshHandle{2002};

std::vector<uint8_t> BuildSolidTgaPixel(uint8_t r, uint8_t g, uint8_t b) {
    std::vector<uint8_t> bytes(18 + 3, 0);
    bytes[2] = 2;      // Uncompressed true-color image
    bytes[12] = 1;     // Width = 1
    bytes[14] = 1;     // Height = 1
    bytes[16] = 24;    // 24-bit BGR
    bytes[17] = 0x20;  // Top-left origin
    bytes[18] = b;
    bytes[19] = g;
    bytes[20] = r;
    return bytes;
}

uint8_t ToByte01(float v) {
    const float clamped = std::clamp(v, 0.0f, 1.0f);
    return static_cast<uint8_t>(clamped * 255.0f + 0.5f);
}

void ConfigurePbrMaterialFromValues(Runtime::Graphics::TextureManager& textureManager,
                                    Runtime::Graphics::Material& material,
                                    const glm::vec3& baseColor,
                                    float metallic,
                                    float roughness,
                                    float ao = 1.0f) {
    const auto albedoBytes = BuildSolidTgaPixel(ToByte01(baseColor.r), ToByte01(baseColor.g), ToByte01(baseColor.b));
    Runtime::Graphics::TextureManager::TextureLoadRequest albedoReq;
    albedoReq.bytes = albedoBytes.data();
    albedoReq.byteLength = albedoBytes.size();
    albedoReq.usage = Runtime::Graphics::TextureManager::TextureUsage::Color;
    material.SetAlbedo(textureManager.LoadSync(albedoReq));

    const auto normalBytes = BuildSolidTgaPixel(128, 128, 255);
    Runtime::Graphics::TextureManager::TextureLoadRequest normalReq;
    normalReq.bytes = normalBytes.data();
    normalReq.byteLength = normalBytes.size();
    normalReq.usage = Runtime::Graphics::TextureManager::TextureUsage::Normal;
    material.SetNormal(textureManager.LoadSync(normalReq));

    const auto ormBytes = BuildSolidTgaPixel(ToByte01(ao), ToByte01(roughness), ToByte01(metallic));
    Runtime::Graphics::TextureManager::TextureLoadRequest ormReq;
    ormReq.bytes = ormBytes.data();
    ormReq.byteLength = ormBytes.size();
    ormReq.usage = Runtime::Graphics::TextureManager::TextureUsage::ORM;
    material.SetOrm(textureManager.LoadSync(ormReq));
}

std::optional<std::vector<uint8_t>> ResolveBinaryResourceSync(void* context, Runtime::Handle handle) {
    if (!context) {
        return std::nullopt;
    }

    auto* app = static_cast<Game::Core::GameApp*>(context);
    return app->GetResourceManager().ReadBinarySync(Runtime::Resource::AssetHandle{handle.Value()});
}

std::future<std::optional<std::vector<uint8_t>>> ResolveBinaryResourceAsync(void* context, Runtime::Handle handle) {
    if (!context) {
        std::promise<std::optional<std::vector<uint8_t>>> rejected;
        rejected.set_value(std::nullopt);
        return rejected.get_future();
    }

    auto* app = static_cast<Game::Core::GameApp*>(context);
    return app->GetResourceManager().ReadBinaryAsync(Runtime::Resource::AssetHandle{handle.Value()});
}

unsigned int CompileUiOverlayShader(unsigned int type, const char* source) {
    const unsigned int shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    int ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (ok == GL_TRUE) {
        return shader;
    }

    char log[512] = {};
    glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
    std::cerr << "[UIOverlay] shader compile failed: " << log << std::endl;
    glDeleteShader(shader);
    return 0;
}

} // namespace

namespace Game {
namespace Core {

bool GameApp::Initialize(Runtime::Core::Engine& engine) {
    auto* renderer = engine.GetRenderer();
    auto* window = engine.GetWindow();
    if (!renderer || !window) {
        return false;
    }

    if (!InitializeUiOverlayRenderer()) {
        return false;
    }

    const float cubeVertices[] = {
        -0.5f,-0.5f, 0.5f,   0.0f, 0.0f, 1.0f,
         0.5f,-0.5f, 0.5f,   0.0f, 0.0f, 1.0f,
         0.5f, 0.5f, 0.5f,   0.0f, 0.0f, 1.0f,
         0.5f, 0.5f, 0.5f,   0.0f, 0.0f, 1.0f,
        -0.5f, 0.5f, 0.5f,   0.0f, 0.0f, 1.0f,
        -0.5f,-0.5f, 0.5f,   0.0f, 0.0f, 1.0f,

        -0.5f,-0.5f,-0.5f,   0.0f, 0.0f,-1.0f,
        -0.5f, 0.5f,-0.5f,   0.0f, 0.0f,-1.0f,
         0.5f, 0.5f,-0.5f,   0.0f, 0.0f,-1.0f,
         0.5f, 0.5f,-0.5f,   0.0f, 0.0f,-1.0f,
         0.5f,-0.5f,-0.5f,   0.0f, 0.0f,-1.0f,
        -0.5f,-0.5f,-0.5f,   0.0f, 0.0f,-1.0f,

        -0.5f, 0.5f, 0.5f,  -1.0f, 0.0f, 0.0f,
        -0.5f, 0.5f,-0.5f,  -1.0f, 0.0f, 0.0f,
        -0.5f,-0.5f,-0.5f,  -1.0f, 0.0f, 0.0f,
        -0.5f,-0.5f,-0.5f,  -1.0f, 0.0f, 0.0f,
        -0.5f,-0.5f, 0.5f,  -1.0f, 0.0f, 0.0f,
        -0.5f, 0.5f, 0.5f,  -1.0f, 0.0f, 0.0f,

         0.5f, 0.5f, 0.5f,   1.0f, 0.0f, 0.0f,
         0.5f,-0.5f,-0.5f,   1.0f, 0.0f, 0.0f,
         0.5f, 0.5f,-0.5f,   1.0f, 0.0f, 0.0f,
         0.5f,-0.5f,-0.5f,   1.0f, 0.0f, 0.0f,
         0.5f, 0.5f, 0.5f,   1.0f, 0.0f, 0.0f,
         0.5f,-0.5f, 0.5f,   1.0f, 0.0f, 0.0f,

        -0.5f, 0.5f,-0.5f,   0.0f, 1.0f, 0.0f,
        -0.5f, 0.5f, 0.5f,   0.0f, 1.0f, 0.0f,
         0.5f, 0.5f, 0.5f,   0.0f, 1.0f, 0.0f,
         0.5f, 0.5f, 0.5f,   0.0f, 1.0f, 0.0f,
         0.5f, 0.5f,-0.5f,   0.0f, 1.0f, 0.0f,
        -0.5f, 0.5f,-0.5f,   0.0f, 1.0f, 0.0f,

        -0.5f,-0.5f,-0.5f,   0.0f,-1.0f, 0.0f,
         0.5f,-0.5f,-0.5f,   0.0f,-1.0f, 0.0f,
         0.5f,-0.5f, 0.5f,   0.0f,-1.0f, 0.0f,
         0.5f,-0.5f, 0.5f,   0.0f,-1.0f, 0.0f,
        -0.5f,-0.5f, 0.5f,   0.0f,-1.0f, 0.0f,
        -0.5f,-0.5f,-0.5f,   0.0f,-1.0f, 0.0f
    };

    m_meshManager.SetResolvers(this, ResolveBinaryResourceSync, ResolveBinaryResourceAsync);
    m_textureManager.SetResolvers(this, ResolveBinaryResourceSync, ResolveBinaryResourceAsync);

    m_mesh = m_meshManager.CreateFromVertices(kCubeMeshHandle, cubeVertices, sizeof(cubeVertices) / sizeof(float));
    if (!m_mesh) {
        return false;
    }

    const std::vector<float> sphereVertices = BuildSphereVertices(18, 24);
    m_ballMesh = m_meshManager.CreateFromVertices(kBallMeshHandle, sphereVertices.data(), sphereVertices.size());
    if (!m_ballMesh) {
        return false;
    }

    bool mountedAssets = false;
    const std::vector<std::string> assetCandidates = {
        "assets",
        "MyGame/assets",
        "../assets"
    };
    for (const std::string& candidate : assetCandidates) {
        if (!std::filesystem::exists(std::filesystem::path(candidate))) {
            continue;
        }
        if (m_resourceManager.GetFileSystem().Mount("assets", candidate)) {
            mountedAssets = true;
            break;
        }
    }

    if (!mountedAssets) {
        std::cerr << "Failed to mount assets root for resource IO." << std::endl;
        return false;
    }

    if (!m_resourceManager.RegisterHandle(kMetalVertHandle, "assets:/shaders/metal.vert")) {
        return false;
    }
    if (!m_resourceManager.RegisterHandle(kMetalFragHandle, "assets:/shaders/metal.frag")) {
        return false;
    }
    std::optional<std::string> vertSource;
    std::optional<std::string> fragSource;
    if (m_useAsyncResourceIO) {
        vertSource = m_resourceManager.ReadTextAsync(kMetalVertHandle).get();
        fragSource = m_resourceManager.ReadTextAsync(kMetalFragHandle).get();
    } else {
        vertSource = m_resourceManager.ReadTextSync(kMetalVertHandle);
        fragSource = m_resourceManager.ReadTextSync(kMetalFragHandle);
    }

    const bool loaded = vertSource.has_value() &&
                        fragSource.has_value() &&
                        renderer->LoadShaderFromSource("metal", vertSource.value(), fragSource.value());

    if (!loaded) {
        std::cerr << "Failed to load metal shaders: "
                  << kMetalVertHandle.Value() << " | " << kMetalFragHandle.Value()
                  << std::endl;
        return false;
    }

    auto shader = renderer->GetShader("metal");
    if (!shader) {
        return false;
    }

    m_material.SetShader(shader);
    ConfigurePbrMaterialFromValues(m_textureManager, m_material, glm::vec3(0.95f, 0.95f, 0.98f), 1.0f, 0.08f);

    m_groundMaterial.SetShader(shader);
    ConfigurePbrMaterialFromValues(m_textureManager, m_groundMaterial, glm::vec3(0.22f, 0.24f, 0.28f), 0.15f, 0.75f);

    m_ballMaterial.SetShader(shader);
    ConfigurePbrMaterialFromValues(m_textureManager, m_ballMaterial, glm::vec3(0.95f, 0.45f, 0.25f), 0.35f, 0.22f);

    m_camera.SetPosition(glm::vec3(2.8f, 2.1f, 5.2f));
    m_cameraTarget = glm::vec3(0.0f, -0.2f, 0.0f);
    m_camera.SetTarget(m_cameraTarget);

    const int winW = window->GetWidth();
    const int winH = window->GetHeight();
    const float aspect = (winH > 0) ? static_cast<float>(winW) / static_cast<float>(winH) : (4.0f / 3.0f);
    m_camera.SetPerspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);

    // Keep a simple 2D movement entity for ECS regression checks.
    auto movementEntity = m_world.CreateEntity();
    m_world.AddComponent<Components::TransformComponent>(movementEntity);
    m_world.AddComponent<Components::VelocityComponent>(movementEntity, Components::VelocityComponent{60.0f, 0.0f});

    // Rotating cube as ECS entity = Transform3D + Rotation + CubeRender.
    m_cubeEntity = m_world.CreateEntity();
    m_world.AddComponent<Components::Transform3DComponent>(m_cubeEntity);
    auto& cubeTransform = m_world.RegistryRef().GetComponent<Components::Transform3DComponent>(m_cubeEntity);
    cubeTransform.position = glm::vec3(-1.6f, -0.1f, 0.0f);
    Components::RotationComponent rotation;
    rotation.axis = glm::normalize(glm::vec3(0.5f, 1.0f, 0.0f));
    rotation.speed = 1.0f;
    m_world.AddComponent<Components::RotationComponent>(m_cubeEntity, rotation);
    m_world.AddComponent<Components::CubeRenderComponent>(m_cubeEntity);

    m_world.Systems().AddSystem(std::make_unique<Systems::RotationSystem>());

    m_physicsWorld.SetFixedTimeStep(1.0f / 120.0f);
    m_physicsWorld.SetGravity(glm::vec3(0.0f, -10.5f, 0.0f));

    Runtime::Physics::RigidBodyDesc groundDesc;
    groundDesc.position = m_groundPosition;
    groundDesc.isStatic = true;
    groundDesc.useGravity = false;
    m_groundBodyId = m_physicsWorld.CreateRigidBody(groundDesc);

    Runtime::Physics::ColliderDesc groundColliderDesc;
    groundColliderDesc.shape = std::make_shared<Runtime::Physics::BoxShape>(m_groundScale * 0.5f);
    groundColliderDesc.material.restitution = 0.62f;
    groundColliderDesc.material.dynamicFriction = 0.30f;
    groundColliderDesc.material.staticFriction = 0.35f;
    groundColliderDesc.oneSided = true;
    groundColliderDesc.oneSidedNormalLocal = glm::vec3(0.0f, 1.0f, 0.0f);
    if (!m_physicsWorld.AttachCollider(m_groundBodyId, groundColliderDesc)) {
        return false;
    }

    Runtime::Physics::RigidBodyDesc ballDesc;
    ballDesc.position = glm::vec3(m_groundPosition.x, 1.8f, 0.0f);
    ballDesc.linearVelocity = glm::vec3(0.0f);
    ballDesc.mass = 1.0f;
    const float sphereInertia = 0.4f * ballDesc.mass * m_ballRadius * m_ballRadius;
    ballDesc.inertiaTensorDiagonal = glm::vec3(sphereInertia);
    // Spin around Z creates tangential contact speed on ground and should produce visible side deflection.
    ballDesc.angularVelocity = glm::vec3(0.0f, 0.0f, 12.0f);
    ballDesc.isStatic = false;
    ballDesc.useGravity = true;
    m_ballBodyId = m_physicsWorld.CreateRigidBody(ballDesc);

    Runtime::Physics::ColliderDesc ballColliderDesc;
    ballColliderDesc.shape = std::make_shared<Runtime::Physics::SphereShape>(m_ballRadius);
    ballColliderDesc.material.restitution = 0.78f;
    ballColliderDesc.material.dynamicFriction = 0.28f;
    ballColliderDesc.material.staticFriction = 0.34f;
    if (!m_physicsWorld.AttachCollider(m_ballBodyId, ballColliderDesc)) {
        return false;
    }

    m_lastBallVelocityY = 0.0f;
    m_contactSquash = 0.0f;
    m_squashVelocity = 0.0f;
    m_ballGroundContact = false;
    m_ballTelemetryTimer = 0.0f;
    m_ballTelemetryDuration = 6.0f;

    m_networkStarted = m_networkManager.StartClient(m_serverEndpoint);
    if (!m_networkStarted) {
        std::cout << "[NetworkClient] failed to start client worker" << std::endl;
    }
    m_sentInitialPing = false;
    m_pingTimer = 0.0f;
    m_pongCount = 0;

    m_uiManager.Initialize(&m_world.Events());
    m_uiCanvas = m_uiManager.CreateCanvas("StartGameTestCanvas", Runtime::UI::CanvasSpace::Screen, 100);
    if (m_uiCanvas) {
        m_uiCanvas->SetAnchor(glm::vec2(0.0f), glm::vec2(1.0f));
        m_uiCanvas->SetMargins(glm::vec2(0.0f), glm::vec2(0.0f));

        m_uiPanel = std::make_shared<Runtime::UI::Panel>("StartGamePanel");
        m_uiPanel->SetPosition(glm::vec2(24.0f, 24.0f));
        m_uiPanel->SetSize(glm::vec2(360.0f, 170.0f));
        m_uiPanel->SetLayer(100);
        m_uiCanvas->AddChild(m_uiPanel);

        m_uiTitleLabel = std::make_shared<Runtime::UI::Label>("StartGameTitle");
        m_uiTitleLabel->SetPosition(glm::vec2(18.0f, 14.0f));
        m_uiTitleLabel->SetSize(glm::vec2(320.0f, 24.0f));
        m_uiTitleLabel->SetLayer(101);
        m_uiTitleLabel->SetText("Start Game Toggle UI Test");
        m_uiPanel->AddChild(m_uiTitleLabel);

        m_uiStateLabel = std::make_shared<Runtime::UI::Label>("StartGameState");
        m_uiStateLabel->SetPosition(glm::vec2(18.0f, 52.0f));
        m_uiStateLabel->SetSize(glm::vec2(320.0f, 24.0f));
        m_uiStateLabel->SetLayer(101);
        m_uiPanel->AddChild(m_uiStateLabel);

        m_uiStartButton = std::make_shared<Runtime::UI::Button>("StartGameButton");
        m_uiStartButton->SetPosition(glm::vec2(18.0f, 90.0f));
        m_uiStartButton->SetSize(glm::vec2(180.0f, 40.0f));
        m_uiStartButton->SetLayer(101);
        m_uiPanel->AddChild(m_uiStartButton);

        auto refreshStartUiText = [this]() {
            if (m_uiStateLabel) {
                m_uiStateLabel->SetText(std::string("State: ") + (m_gameStarted ? "RUNNING" : "PAUSED"));
            }
            if (m_uiStartButton) {
                m_uiStartButton->SetText(m_gameStarted ? "Pause Game" : "Start Game");
            }
        };

        refreshStartUiText();
        m_uiStartButton->SetOnClick([this, refreshStartUiText]() {
            m_gameStarted = !m_gameStarted;
            refreshStartUiText();
        });
    }

    m_uiTickLog.open("ui_tick_pixels.log", std::ios::out | std::ios::trunc);
    if (m_uiTickLog.is_open()) {
        m_uiTickLog << "tick,mouse_x,mouse_y,hit_widget,state,ui_commands,pixel_r,pixel_g,pixel_b,pixel_a,probe_r,probe_g,probe_b,probe_a" << std::endl;
    }

    return true;
}

void GameApp::Update(float dt, const Runtime::Core::Input& input) {
    auto refreshStartUiText = [this]() {
        if (m_uiStateLabel) {
            m_uiStateLabel->SetText(std::string("State: ") + (m_gameStarted ? "RUNNING" : "PAUSED"));
        }
        if (m_uiStartButton) {
            m_uiStartButton->SetText(m_gameStarted ? "Pause Game" : "Start Game");
        }
    };

    Runtime::UI::UIEvent moveEvent;
    moveEvent.type = Runtime::UI::UIEventType::PointerMove;
    moveEvent.screenPosition = glm::vec2(static_cast<float>(input.GetMouseX()), static_cast<float>(input.GetMouseY()));
    m_uiMousePixelX = static_cast<int>(moveEvent.screenPosition.x);
    m_uiMousePixelY = static_cast<int>(moveEvent.screenPosition.y);
    m_uiManager.RouteEvent(moveEvent);

    if (input.IsKeyPressed(Runtime::Core::Mouse_Left)) {
        Runtime::UI::UIEvent downEvent;
        downEvent.type = Runtime::UI::UIEventType::PointerDown;
        downEvent.pointerButton = 0;
        downEvent.screenPosition = moveEvent.screenPosition;
        m_uiManager.RouteEvent(downEvent);
    }

    if (input.IsKeyReleased(Runtime::Core::Mouse_Left)) {
        Runtime::UI::UIEvent upEvent;
        upEvent.type = Runtime::UI::UIEventType::PointerUp;
        upEvent.pointerButton = 0;
        upEvent.screenPosition = moveEvent.screenPosition;
        m_uiManager.RouteEvent(upEvent);
    }

    if (input.IsKeyPressed(Runtime::Core::Key_Enter)) {
        m_gameStarted = !m_gameStarted;
        refreshStartUiText();
    }

    m_uiManager.Update(dt, Runtime::UI::Rect{glm::vec2(0.0f, 0.0f), glm::vec2(1280.0f, 720.0f)});
    m_uiLastHitWidget.clear();
    if (m_uiCanvas) {
        Runtime::UI::Widget* hit = m_uiCanvas->FindTopWidgetAt(moveEvent.screenPosition);
        if (hit) {
            m_uiLastHitWidget = hit->GetTypeName() + ":" + hit->GetId();
        }
    }

    // Camera test controls: WASD move camera; Q/E rotate camera yaw around world up axis.
    {
        glm::vec3 move(0.0f);
        if (input.IsKeyDown(Runtime::Core::Key_W)) {
            move.z += 1.0f;
        }
        if (input.IsKeyDown(Runtime::Core::Key_S)) {
            move.z -= 1.0f;
        }
        if (input.IsKeyDown(Runtime::Core::Key_A)) {
            move.x -= 1.0f;
        }
        if (input.IsKeyDown(Runtime::Core::Key_D)) {
            move.x += 1.0f;
        }

        float yawInput = 0.0f;
        if (input.IsKeyDown(Runtime::Core::Key_Q)) {
            yawInput += 1.0f;
        }
        if (input.IsKeyDown(Runtime::Core::Key_E)) {
            yawInput -= 1.0f;
        }

        const glm::vec3 cameraPos = m_camera.GetPosition();
        const glm::vec3 toTarget = m_cameraTarget - cameraPos;
        const float verticalOffset = toTarget.y;
        float horizontalDistance = glm::length(glm::vec2(toTarget.x, toTarget.z));
        if (horizontalDistance < 1e-3f) {
            horizontalDistance = 1.0f;
        }

        glm::vec3 forward(toTarget.x, 0.0f, toTarget.z);
        if (glm::dot(forward, forward) < 1e-5f) {
            forward = glm::vec3(0.0f, 0.0f, -1.0f);
        } else {
            forward = glm::normalize(forward);
        }

        if (yawInput != 0.0f) {
            const float yawSpeed = 1.6f;
            const float yawAngle = yawInput * yawSpeed * dt;
            const glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), yawAngle, glm::vec3(0.0f, 1.0f, 0.0f));
            forward = glm::normalize(glm::vec3(rotation * glm::vec4(forward, 0.0f)));
        }

        const glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)));

        glm::vec3 newPos = cameraPos;
        if (glm::dot(move, move) > 0.0f) {
            move = glm::normalize(move);
            const float cameraSpeed = 4.0f;
            const glm::vec3 delta = (right * move.x + forward * move.z) * (cameraSpeed * dt);
            newPos += delta;
        }

        m_cameraTarget = newPos + glm::vec3(forward.x * horizontalDistance, verticalOffset, forward.z * horizontalDistance);
        m_camera.SetPosition(newPos);
        m_camera.SetTarget(m_cameraTarget);
    }

    Runtime::Network::NetworkEvent networkEvent;
    while (m_networkManager.PollEvent(networkEvent)) {
        switch (networkEvent.type) {
        case Runtime::Network::NetworkEventType::Connected:
            std::cout << "[NetworkClient] connected to "
                      << m_serverEndpoint.address << ":" << m_serverEndpoint.port << std::endl;
            m_sentInitialPing = false;
            break;
        case Runtime::Network::NetworkEventType::Disconnected:
            std::cout << "[NetworkClient] disconnected: " << networkEvent.detail << std::endl;
            break;
        case Runtime::Network::NetworkEventType::Error:
            std::cout << "[NetworkClient] error: " << networkEvent.detail << std::endl;
            break;
        case Runtime::Network::NetworkEventType::MessageReceived:
            if (networkEvent.message.Type() == Runtime::Network::MessageType::Pong) {
                ++m_pongCount;
                std::cout << "[NetworkClient] received pong #" << m_pongCount
                          << " seq=" << networkEvent.message.header.sequence << std::endl;
            }
            break;
        }
    }

    if (m_networkManager.IsConnected()) {
        if (!m_sentInitialPing) {
            m_networkManager.Send(Runtime::Network::Message::Ping());
            m_sentInitialPing = true;
        }

        m_pingTimer += dt;
        if (m_pingTimer >= 1.0f) {
            m_pingTimer = 0.0f;
            m_networkManager.Send(Runtime::Network::Message::Ping());
        }
    }

    if (!m_gameStarted) {
        if (input.IsKeyPressed(Runtime::Core::Key_Escape)) {
            m_shouldExit = true;
        }
        return;
    }

    auto* ballBody = m_physicsWorld.GetRigidBody(m_ballBodyId);
    float impactSpeed = 0.0f;
    float prevVelocityY = m_lastBallVelocityY;
    if (ballBody) {
        impactSpeed = std::abs(ballBody->LinearVelocity().y);
        prevVelocityY = ballBody->LinearVelocity().y;

        glm::vec3 moveInput(0.0f);
        if (input.IsKeyDown(Runtime::Core::Key_Up)) {
            moveInput.z -= 1.0f;
        }
        if (input.IsKeyDown(Runtime::Core::Key_Down)) {
            moveInput.z += 1.0f;
        }
        if (input.IsKeyDown(Runtime::Core::Key_Left)) {
            moveInput.x -= 1.0f;
        }
        if (input.IsKeyDown(Runtime::Core::Key_Right)) {
            moveInput.x += 1.0f;
        }

        // Drive with force so collision impulses and friction remain effective.
        if (glm::dot(moveInput, moveInput) > 0.0f) {
            moveInput = glm::normalize(moveInput);
            const float moveAccel = 12.0f;
            const glm::vec3 driveForce = moveInput * (ballBody->Mass() * moveAccel);
            ballBody->ApplyForce(driveForce);
        }

        const glm::vec3 linearVelocity = ballBody->LinearVelocity();
        const glm::vec3 horizontalVelocity(linearVelocity.x, 0.0f, linearVelocity.z);
        const float drag = 0.35f;
        ballBody->ApplyForce(-horizontalVelocity * (ballBody->Mass() * drag));
    }

    m_physicsWorld.Step(dt);

    float targetSquash = 0.0f;
    m_ballGroundContact = false;
    float strongestPenetration = 0.0f;
    float strongestNormalImpulse = 0.0f;
    float strongestFixedStepNormalImpulse = 0.0f;
    glm::vec3 strongestNormal = glm::vec3(0.0f);
    float strongestRelVelAlongNormal = 0.0f;
    float strongestRelVelTangent = 0.0f;
    float strongestBallContactDistance = 0.0f;
    uint32_t strongestBodyA = 0;
    uint32_t strongestBodyB = 0;
    const auto& contacts = m_physicsWorld.Contacts();
    for (const auto& contact : contacts) {
        const bool ballGroundPair =
            (contact.bodyA == m_ballBodyId && contact.bodyB == m_groundBodyId) ||
            (contact.bodyA == m_groundBodyId && contact.bodyB == m_ballBodyId);
        if (!ballGroundPair) {
            continue;
        }

        m_ballGroundContact = true;
        if (contact.point.penetration >= strongestPenetration) {
            strongestPenetration = contact.point.penetration;
            strongestNormal = contact.normal;
            strongestBodyA = contact.bodyA;
            strongestBodyB = contact.bodyB;

            const auto* bodyA = m_physicsWorld.GetRigidBody(contact.bodyA);
            const auto* bodyB = m_physicsWorld.GetRigidBody(contact.bodyB);
            if (bodyA && bodyB) {
                const glm::vec3 ra = contact.point.position - bodyA->Position();
                const glm::vec3 rb = contact.point.position - bodyB->Position();
                const glm::vec3 velocityA = bodyA->LinearVelocity() + glm::cross(bodyA->AngularVelocity(), ra);
                const glm::vec3 velocityB = bodyB->LinearVelocity() + glm::cross(bodyB->AngularVelocity(), rb);
                const glm::vec3 relativeVelocity = velocityB - velocityA;
                strongestRelVelAlongNormal = glm::dot(relativeVelocity, contact.normal);
                const glm::vec3 tangent = relativeVelocity - strongestRelVelAlongNormal * contact.normal;
                strongestRelVelTangent = glm::length(tangent);

                const bool bodyAIsBall = (contact.bodyA == m_ballBodyId);
                strongestBallContactDistance = glm::length(
                    contact.point.position - (bodyAIsBall ? bodyA->Position() : bodyB->Position()));
            }
        }
        // Telemetry contract: normalImpulse is the accumulated normal impulse
        // of the pair's last touching sub-step; fixedStepNormalImpulse is the
        // net impulse actually applied across the whole fixed step (warm start
        // plus subsequent deltas of every sub-step). Never mix the two.
        strongestNormalImpulse = std::max(strongestNormalImpulse, contact.point.normalImpulse);
        strongestFixedStepNormalImpulse =
            std::max(strongestFixedStepNormalImpulse, contact.fixedStepNormalImpulse);

        const float penetrationSquash = std::clamp(contact.point.penetration * 1.4f, 0.0f, 0.35f);
        const float impactSquash = std::clamp(impactSpeed * 0.03f, 0.0f, 0.35f);
        targetSquash = std::max(targetSquash, penetrationSquash + impactSquash);
    }
    targetSquash = std::clamp(targetSquash, 0.0f, 0.45f);

    if (ballBody) {
        const float velocityY = ballBody->LinearVelocity().y;
        const float groundTopY = m_groundPosition.y + m_groundScale.y * 0.5f;
        const float distanceToGround = ballBody->Position().y - (groundTopY + m_ballRadius);
        if (velocityY < -0.1f && distanceToGround < 0.2f) {
            const float proximity = std::clamp(1.0f - distanceToGround / 0.2f, 0.0f, 1.0f);
            const float preImpactSquash = std::clamp((-velocityY) * 0.03f * proximity, 0.0f, 0.35f);
            targetSquash = std::max(targetSquash, preImpactSquash);
        }

        const bool bouncedUp = prevVelocityY < -0.3f && velocityY > 0.15f;
        if (bouncedUp) {
            const float bouncePulse = std::clamp((-prevVelocityY) * 0.035f, 0.0f, 0.4f);
            targetSquash = std::max(targetSquash, bouncePulse);

            std::cout << "[BallImpact] preVy=" << prevVelocityY
                      << " postVy=" << velocityY
                      << " penetration=" << strongestPenetration
                      << " normalImpulse=" << strongestNormalImpulse
                      << " fixedStepNormalImpulse=" << strongestFixedStepNormalImpulse
                      << std::endl;
        }
        m_lastBallVelocityY = velocityY;

        if (m_ballTelemetryDuration > 0.0f) {
            m_ballTelemetryDuration -= dt;
            m_ballTelemetryTimer += dt;
            if (m_ballTelemetryTimer >= 0.25f) {
                m_ballTelemetryTimer = 0.0f;
                const glm::vec3 p = ballBody->Position();
                const glm::vec3 lv = ballBody->LinearVelocity();
                const glm::vec3 av = ballBody->AngularVelocity();
                std::cout << "[BallTelemetry] pos=(" << p.x << ", " << p.y << ", " << p.z
                          << ") vel=(" << lv.x << ", " << lv.y << ", " << lv.z
                          << ") angVel=(" << av.x << ", " << av.y << ", " << av.z << ")"
                          << " groundContact=" << (m_ballGroundContact ? 1 : 0)
                          << " pair=(" << strongestBodyA << ", " << strongestBodyB << ")"
                          << " n=(" << strongestNormal.x << ", " << strongestNormal.y << ", " << strongestNormal.z << ")"
                          << " relN=" << strongestRelVelAlongNormal
                          << " relT=" << strongestRelVelTangent
                          << " ballCpDist=" << strongestBallContactDistance
                          << " pen=" << strongestPenetration
                          << " nImpulse=" << strongestNormalImpulse
                          << " stepNImpulse=" << strongestFixedStepNormalImpulse
                          << std::endl;
            }
        }
    }

    // Spring-damper keeps deformation visible for multiple frames.
    const float spring = 95.0f;
    const float damping = 16.0f;
    const float accel = spring * (targetSquash - m_contactSquash) - damping * m_squashVelocity;
    m_squashVelocity += accel * dt;
    m_contactSquash += m_squashVelocity * dt;
    m_contactSquash = std::clamp(m_contactSquash, 0.0f, 0.7f);
    if (m_contactSquash <= 0.0005f && targetSquash <= 0.0005f) {
        m_contactSquash = 0.0f;
        m_squashVelocity = 0.0f;
    }

    auto movementEntities = m_world.RegistryRef().EntitiesWith<Components::VelocityComponent>();
    for (auto entity : movementEntities) {
        if (m_world.RegistryRef().HasComponent<Components::TransformComponent>(entity)) {
            Systems::MovementSystem::UpdateEntity(m_world, entity, dt);
        }
    }

    m_world.Update(dt);

    if (ballBody && m_world.RegistryRef().HasComponent<Components::Transform3DComponent>(m_cubeEntity)) {
        // Keep cube and ball visually separated while ball is moving.
        auto& cubeTransform = m_world.RegistryRef().GetComponent<Components::Transform3DComponent>(m_cubeEntity);
        cubeTransform.position.x = -1.6f;
    }

    if (input.IsKeyPressed(Runtime::Core::Key_Escape)) {
        m_shouldExit = true;
    }
}

void GameApp::Render(Runtime::Core::Engine& engine) {
    auto* renderer = engine.GetRenderer();
    auto* window = engine.GetWindow();

    if (renderer) {
        renderer->Clear();

        if (m_mesh && m_ballMesh) {
            glm::mat4 groundModel = glm::mat4(1.0f);
            groundModel = glm::translate(groundModel, m_groundPosition);
            groundModel = glm::scale(groundModel, m_groundScale);
            renderer->Submit(*m_mesh, m_groundMaterial, groundModel);

            const auto* ballBody = m_physicsWorld.GetRigidBody(m_ballBodyId);
            if (ballBody) {
                const float squash = std::clamp(m_contactSquash, 0.0f, 0.65f);
                const float yScale = 1.0f - squash;
                const float xzScale = 1.0f + squash * 0.95f;
                const float anchorOffset = m_ballGroundContact ? (m_ballRadius * (1.0f - yScale) * 0.82f) : 0.0f;

                glm::mat4 ballModel = glm::mat4(1.0f);
                ballModel = glm::translate(ballModel, ballBody->Position() + glm::vec3(0.0f, -anchorOffset, 0.0f));
                ballModel *= glm::mat4_cast(ballBody->Orientation());
                ballModel = glm::scale(ballModel, glm::vec3(
                    m_ballVisualScale.x * xzScale,
                    m_ballVisualScale.y * yScale,
                    m_ballVisualScale.z * xzScale));
                renderer->Submit(*m_ballMesh, m_ballMaterial, ballModel);
            }

            Systems::CubeRenderSystem::Render(m_world, *renderer, *m_mesh, m_material, m_camera);
        }
        if (window) {
            RenderFloatingTextReserved(window->GetWidth(), window->GetHeight());
        }
    }

    m_uiManager.Render();
    if (window) {
        RenderUiOverlay(window->GetWidth(), window->GetHeight());
    }
    const size_t uiCommandCount = m_uiManager.GetRenderer() ? m_uiManager.GetRenderer()->GetStats().commandCount : 0;

    uint8_t rgba[4] = {0, 0, 0, 0};
    uint8_t probeRgba[4] = {0, 0, 0, 0};
    if (window) {
        const int winW = std::max(1, window->GetWidth());
        const int winH = std::max(1, window->GetHeight());
        const int px = std::clamp(m_uiMousePixelX, 0, winW - 1);
        const int pyTopLeft = std::clamp(m_uiMousePixelY, 0, winH - 1);
        const int pyGl = winH - 1 - pyTopLeft;
        glReadPixels(px, pyGl, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, rgba);

        // Probe a pixel inside the Start button region to verify overlay visibility.
        const int probeX = std::clamp(60, 0, winW - 1);
        const int probeYTopLeft = std::clamp(130, 0, winH - 1);
        const int probeYGl = winH - 1 - probeYTopLeft;
        glReadPixels(probeX, probeYGl, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, probeRgba);

        if (m_uiTickLog.is_open()) {
            m_uiTickLog << m_uiTickIndex << ','
                        << px << ','
                        << pyTopLeft << ','
                        << '"' << m_uiLastHitWidget << '"' << ','
                        << (m_gameStarted ? "RUNNING" : "PAUSED") << ','
                        << uiCommandCount << ','
                        << static_cast<int>(rgba[0]) << ','
                        << static_cast<int>(rgba[1]) << ','
                        << static_cast<int>(rgba[2]) << ','
                        << static_cast<int>(rgba[3]) << ','
                        << static_cast<int>(probeRgba[0]) << ','
                        << static_cast<int>(probeRgba[1]) << ','
                        << static_cast<int>(probeRgba[2]) << ','
                        << static_cast<int>(probeRgba[3])
                        << std::endl;
        }
    }
    ++m_uiTickIndex;

    if (window && window->GetNativeWindow()) {
        m_windowTitle = std::string("MyGame | UI Test: ") + (m_gameStarted ? "RUNNING" : "PAUSED") +
                " | Click Start Button or Press Enter | UI Cmds=" + std::to_string(uiCommandCount);
        glfwSetWindowTitle(window->GetNativeWindow(), m_windowTitle.c_str());
    }

    if (window) {
        window->SwapBuffers();
    }
}

void GameApp::Shutdown() {
    m_networkManager.Stop();
    m_uiManager.Shutdown();
    ShutdownUiOverlayRenderer();
    if (m_uiTickLog.is_open()) {
        m_uiTickLog.flush();
        m_uiTickLog.close();
    }

    if (m_mesh) {
        m_mesh->Destroy();
    }
    if (m_ballMesh) {
        m_ballMesh->Destroy();
    }
    m_mesh.reset();
    m_ballMesh.reset();
    m_meshManager.Clear();
    m_textureManager.Clear();
    m_shouldExit = true;
}

bool GameApp::InitializeUiOverlayRenderer() {
    static const char* kUiVert = R"glsl(
#version 330 core
uniform vec4 u_Rect;
uniform vec2 u_Viewport;
void main() {
    vec2 corners[6] = vec2[](
        vec2(0.0, 0.0), vec2(1.0, 0.0), vec2(1.0, 1.0),
        vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(0.0, 1.0)
    );
    vec2 local = corners[gl_VertexID];
    vec2 pos = vec2(u_Rect.x + local.x * u_Rect.z, u_Rect.y + local.y * u_Rect.w);
    vec2 ndc = vec2((pos.x / u_Viewport.x) * 2.0 - 1.0, 1.0 - (pos.y / u_Viewport.y) * 2.0);
    gl_Position = vec4(ndc, 0.0, 1.0);
}
)glsl";

    static const char* kUiFrag = R"glsl(
#version 330 core
uniform vec4 u_Color;
out vec4 FragColor;
void main() {
    FragColor = u_Color;
}
)glsl";

    static const char* kUiTextVert = R"glsl(
#version 330 core
layout(location = 0) in vec2 aPos;
uniform vec2 u_Viewport;
void main() {
    vec2 ndc = vec2((aPos.x / u_Viewport.x) * 2.0 - 1.0, 1.0 - (aPos.y / u_Viewport.y) * 2.0);
    gl_Position = vec4(ndc, 0.0, 1.0);
}
)glsl";

    static const char* kUiTextFrag = R"glsl(
#version 330 core
uniform vec4 u_Color;
out vec4 FragColor;
void main() {
    FragColor = u_Color;
}
)glsl";

    const unsigned int vs = CompileUiOverlayShader(GL_VERTEX_SHADER, kUiVert);
    const unsigned int fs = CompileUiOverlayShader(GL_FRAGMENT_SHADER, kUiFrag);
    if (vs == 0 || fs == 0) {
        if (vs != 0) {
            glDeleteShader(vs);
        }
        if (fs != 0) {
            glDeleteShader(fs);
        }
        return false;
    }

    m_uiOverlayProgram = glCreateProgram();
    glAttachShader(m_uiOverlayProgram, vs);
    glAttachShader(m_uiOverlayProgram, fs);
    glLinkProgram(m_uiOverlayProgram);
    glDeleteShader(vs);
    glDeleteShader(fs);

    int linked = 0;
    glGetProgramiv(m_uiOverlayProgram, GL_LINK_STATUS, &linked);
    if (linked != GL_TRUE) {
        char log[512] = {};
        glGetProgramInfoLog(m_uiOverlayProgram, sizeof(log), nullptr, log);
        std::cerr << "[UIOverlay] program link failed: " << log << std::endl;
        glDeleteProgram(m_uiOverlayProgram);
        m_uiOverlayProgram = 0;
        return false;
    }

    glGenVertexArrays(1, &m_uiOverlayVao);
    m_uiOverlayRectLoc = glGetUniformLocation(m_uiOverlayProgram, "u_Rect");
    m_uiOverlayColorLoc = glGetUniformLocation(m_uiOverlayProgram, "u_Color");
    m_uiOverlayViewportLoc = glGetUniformLocation(m_uiOverlayProgram, "u_Viewport");

    const unsigned int textVs = CompileUiOverlayShader(GL_VERTEX_SHADER, kUiTextVert);
    const unsigned int textFs = CompileUiOverlayShader(GL_FRAGMENT_SHADER, kUiTextFrag);
    if (textVs == 0 || textFs == 0) {
        if (textVs != 0) {
            glDeleteShader(textVs);
        }
        if (textFs != 0) {
            glDeleteShader(textFs);
        }
        return false;
    }

    m_uiTextProgram = glCreateProgram();
    glAttachShader(m_uiTextProgram, textVs);
    glAttachShader(m_uiTextProgram, textFs);
    glLinkProgram(m_uiTextProgram);
    glDeleteShader(textVs);
    glDeleteShader(textFs);

    linked = 0;
    glGetProgramiv(m_uiTextProgram, GL_LINK_STATUS, &linked);
    if (linked != GL_TRUE) {
        char log[512] = {};
        glGetProgramInfoLog(m_uiTextProgram, sizeof(log), nullptr, log);
        std::cerr << "[UIOverlay] text program link failed: " << log << std::endl;
        glDeleteProgram(m_uiTextProgram);
        m_uiTextProgram = 0;
        return false;
    }

    glGenVertexArrays(1, &m_uiTextVao);
    glGenBuffers(1, &m_uiTextVbo);
    glBindVertexArray(m_uiTextVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_uiTextVbo);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_STREAM_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, reinterpret_cast<void*>(0));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    m_uiTextViewportLoc = glGetUniformLocation(m_uiTextProgram, "u_Viewport");
    m_uiTextColorLoc = glGetUniformLocation(m_uiTextProgram, "u_Color");

    return m_uiOverlayVao != 0 && m_uiTextVao != 0 && m_uiTextVbo != 0;
}

void GameApp::ShutdownUiOverlayRenderer() {
    if (m_uiOverlayVao != 0) {
        glDeleteVertexArrays(1, &m_uiOverlayVao);
        m_uiOverlayVao = 0;
    }
    if (m_uiOverlayProgram != 0) {
        glDeleteProgram(m_uiOverlayProgram);
        m_uiOverlayProgram = 0;
    }
    if (m_uiTextVbo != 0) {
        glDeleteBuffers(1, &m_uiTextVbo);
        m_uiTextVbo = 0;
    }
    if (m_uiTextVao != 0) {
        glDeleteVertexArrays(1, &m_uiTextVao);
        m_uiTextVao = 0;
    }
    if (m_uiTextProgram != 0) {
        glDeleteProgram(m_uiTextProgram);
        m_uiTextProgram = 0;
    }
    m_uiOverlayRectLoc = -1;
    m_uiOverlayColorLoc = -1;
    m_uiOverlayViewportLoc = -1;
    m_uiTextViewportLoc = -1;
    m_uiTextColorLoc = -1;
}

void GameApp::RenderUiOverlay(int viewportWidth, int viewportHeight) {
    const Runtime::UI::UIRenderer* uiRenderer = m_uiManager.GetRenderer();
    if (!uiRenderer || m_uiOverlayProgram == 0 || m_uiOverlayVao == 0 || viewportWidth <= 0 || viewportHeight <= 0) {
        return;
    }

    const auto& commands = uiRenderer->GetCommands();
    if (commands.empty()) {
        return;
    }

    const GLboolean wasDepthTest = glIsEnabled(GL_DEPTH_TEST);
    const GLboolean wasBlend = glIsEnabled(GL_BLEND);
    const GLboolean wasCull = glIsEnabled(GL_CULL_FACE);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glUseProgram(m_uiOverlayProgram);
    glBindVertexArray(m_uiOverlayVao);
    glUniform2f(m_uiOverlayViewportLoc, static_cast<float>(viewportWidth), static_cast<float>(viewportHeight));

    for (const auto& cmd : commands) {
        if (cmd.type != Runtime::UI::UIDrawType::Rect) {
            continue;
        }
        glUniform4f(m_uiOverlayRectLoc, cmd.rect.position.x, cmd.rect.position.y, cmd.rect.size.x, cmd.rect.size.y);
        glUniform4f(m_uiOverlayColorLoc, cmd.color.r, cmd.color.g, cmd.color.b, cmd.color.a);
        glDrawArrays(GL_TRIANGLES, 0, 6);
    }

    glBindVertexArray(0);
    glUseProgram(0);

    if (m_uiTextProgram != 0 && m_uiTextVao != 0 && m_uiTextVbo != 0) {
        glUseProgram(m_uiTextProgram);
        glBindVertexArray(m_uiTextVao);
        glUniform2f(m_uiTextViewportLoc, static_cast<float>(viewportWidth), static_cast<float>(viewportHeight));

        for (const auto& cmd : commands) {
            if (cmd.type != Runtime::UI::UIDrawType::Text || cmd.text.empty()) {
                continue;
            }

            if (cmd.textTechnique != Runtime::UI::UITextRenderTechnique::Bitmap ||
                cmd.textUsage != Runtime::UI::UITextUsage::Overlay) {
                // Reserved for SDF/MSDF and world floating text pipeline.
                continue;
            }

            std::string text = cmd.text;
            char easyBuffer[32768] = {};
            const int quads = stb_easy_font_print(
                cmd.rect.position.x + 4.0f,
                cmd.rect.position.y + std::max(14.0f, cmd.rect.size.y * 0.55f),
                text.data(),
                nullptr,
                easyBuffer,
                static_cast<int>(sizeof(easyBuffer)));

            if (quads <= 0) {
                continue;
            }

            struct EasyVertex {
                float x;
                float y;
                float z;
                unsigned char rgba[4];
            };

            const EasyVertex* src = reinterpret_cast<const EasyVertex*>(easyBuffer);
            std::vector<float> triangles;
            triangles.reserve(static_cast<size_t>(quads) * 12);

            for (int q = 0; q < quads; ++q) {
                const EasyVertex& v0 = src[q * 4 + 0];
                const EasyVertex& v1 = src[q * 4 + 1];
                const EasyVertex& v2 = src[q * 4 + 2];
                const EasyVertex& v3 = src[q * 4 + 3];

                triangles.push_back(v0.x); triangles.push_back(v0.y);
                triangles.push_back(v1.x); triangles.push_back(v1.y);
                triangles.push_back(v2.x); triangles.push_back(v2.y);

                triangles.push_back(v0.x); triangles.push_back(v0.y);
                triangles.push_back(v2.x); triangles.push_back(v2.y);
                triangles.push_back(v3.x); triangles.push_back(v3.y);
            }

            if (triangles.empty()) {
                continue;
            }

            glUniform4f(m_uiTextColorLoc, cmd.color.r, cmd.color.g, cmd.color.b, cmd.color.a);
            glBindBuffer(GL_ARRAY_BUFFER, m_uiTextVbo);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(triangles.size() * sizeof(float)), triangles.data(), GL_STREAM_DRAW);
            glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(triangles.size() / 2));
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }

        glBindVertexArray(0);
        glUseProgram(0);
    }

    if (wasDepthTest) {
        glEnable(GL_DEPTH_TEST);
    } else {
        glDisable(GL_DEPTH_TEST);
    }
    if (wasCull) {
        glEnable(GL_CULL_FACE);
    } else {
        glDisable(GL_CULL_FACE);
    }
    if (wasBlend) {
        glEnable(GL_BLEND);
    } else {
        glDisable(GL_BLEND);
    }
}

void GameApp::RenderFloatingTextReserved(int, int) {
    // Reserved hook: integrate SDF/MSDF world text renderer here.
    // Queue is intentionally kept so gameplay can start submitting floating labels now.
    m_floatingTextQueue.clear();
}

void GameApp::QueueFloatingTextSdf(const std::string& text, const glm::vec3& worldPosition, const glm::vec4& color) {
    if (text.empty()) {
        return;
    }
    m_floatingTextQueue.push_back(FloatingTextEntry{text, worldPosition, color, Runtime::UI::UITextRenderTechnique::SDF});
}

void GameApp::QueueFloatingTextMsdf(const std::string& text, const glm::vec3& worldPosition, const glm::vec4& color) {
    if (text.empty()) {
        return;
    }
    m_floatingTextQueue.push_back(FloatingTextEntry{text, worldPosition, color, Runtime::UI::UITextRenderTechnique::MSDF});
}

} // namespace Core
} // namespace Game
