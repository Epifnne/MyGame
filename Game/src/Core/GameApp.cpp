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
#include <future>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

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

} // namespace

namespace Game {
namespace Core {

bool GameApp::Initialize(Runtime::Core::Engine& engine) {
    auto* renderer = engine.GetRenderer();
    auto* window = engine.GetWindow();
    if (!renderer || !window) {
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

    if (!m_resourceManager.GetFileSystem().Mount("assets", "assets")) {
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
    m_camera.SetTarget(glm::vec3(0.0f, -0.2f, 0.0f));

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

    return true;
}

void GameApp::Update(float dt, const Runtime::Core::Input& input) {

    auto* ballBody = m_physicsWorld.GetRigidBody(m_ballBodyId);
    float impactSpeed = 0.0f;
    float prevVelocityY = m_lastBallVelocityY;
    if (ballBody) {
        impactSpeed = std::abs(ballBody->LinearVelocity().y);
        prevVelocityY = ballBody->LinearVelocity().y;

        glm::vec3 moveInput(0.0f);
        if (input.IsKeyDown(Runtime::Core::Key_W)) {
            moveInput.z -= 1.0f;
        }
        if (input.IsKeyDown(Runtime::Core::Key_S)) {
            moveInput.z += 1.0f;
        }
        if (input.IsKeyDown(Runtime::Core::Key_A)) {
            moveInput.x -= 1.0f;
        }
        if (input.IsKeyDown(Runtime::Core::Key_D)) {
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
        strongestNormalImpulse = std::max(strongestNormalImpulse, contact.point.normalImpulse);

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
    }

    if (window) {
        window->SwapBuffers();
    }
}

void GameApp::Shutdown() {
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

} // namespace Core
} // namespace Game
