#define GLFW_INCLUDE_NONE
#include "CubeRenderComponent.h"
#include "CubeRenderSystem.h"
#include "CubeRotationSystem.h"
#include "metal.vert.h"
#include "metal.frag.h"

#include <Core/Engine.h>
#include <Core/Input.h>
#include <ECS/Component.h>
#include <Graphics/Renderer.h>
#include <Graphics/Texture.h>
#include <Platform/Window.h>

#include <array>
#include <iostream>
#include <memory>
#include <string_view>
#include <utility>
#include <vector>

namespace {

std::vector<float> BuildCubeVertices() {
    const std::array<glm::vec3, 8> corners = {
        glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(-0.5f, -0.5f, 0.5f),
        glm::vec3(-0.5f, 0.5f, -0.5f), glm::vec3(-0.5f, 0.5f, 0.5f),
        glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(0.5f, -0.5f, 0.5f),
        glm::vec3(0.5f, 0.5f, -0.5f), glm::vec3(0.5f, 0.5f, 0.5f)
    };
    const int faces[][4] = {
        {1, 5, 7, 3}, {4, 0, 2, 6}, {3, 7, 6, 2},
        {0, 4, 5, 1}, {5, 4, 6, 7}, {0, 1, 3, 2}
    };
    std::vector<float> vertices;
    vertices.reserve(36 * 6);
    for (const auto& face : faces) {
        const glm::vec3 normal = glm::normalize(glm::cross(
            corners[face[1]] - corners[face[0]], corners[face[2]] - corners[face[0]]));
        for (int index : {0, 1, 2, 0, 2, 3}) {
            const auto& position = corners[face[index]];
            vertices.insert(vertices.end(), {
                position.x, position.y, position.z, normal.x, normal.y, normal.z
            });
        }
    }
    return vertices;
}

std::shared_ptr<Runtime::Graphics::Texture> LoadSolidPixel(
    unsigned char red, unsigned char green, unsigned char blue, bool srgb) {
    std::array<unsigned char, 21> bytes{};
    bytes[2] = 2;
    bytes[12] = 1;
    bytes[14] = 1;
    bytes[16] = 24;
    bytes[17] = 0x20;
    bytes[18] = blue;
    bytes[19] = green;
    bytes[20] = red;
    auto texture = std::make_shared<Runtime::Graphics::Texture>();
    if (!texture->LoadFromMemory(bytes.data(), bytes.size(), true, srgb)) {
        std::cerr << "[MetalCubeSample] failed to create material texture\n";
        return nullptr;
    }
    return texture;
}

int RunSample(Runtime::Core::Engine& engine, bool smokeTest) {
    auto& renderer = *engine.GetRenderer();
    auto& window = *engine.GetWindow();
    if (!renderer.LoadShaderFromSource("metal", Sample::kMetalVertexShader, Sample::kMetalFragmentShader)) {
        std::cerr << "[MetalCubeSample] failed to load metal shaders\n";
        return 2;
    }

    Runtime::Graphics::Mesh mesh;
    const auto vertices = BuildCubeVertices();
    if (!mesh.Create(vertices.data(), vertices.size())) {
        std::cerr << "[MetalCubeSample] failed to create cube mesh\n";
        return 2;
    }

    Runtime::Graphics::Material material;
    material.SetShader(renderer.GetShader("metal"));
    material.SetAlbedo(LoadSolidPixel(242, 242, 250, true));
    material.SetNormal(LoadSolidPixel(128, 128, 255, false));
    material.SetOrm(LoadSolidPixel(255, 20, 255, false));
    if (!material.GetShader() || !material.GetAlbedo() || !material.GetNormal() || !material.GetOrm()) {
        std::cerr << "[MetalCubeSample] failed to initialize metal material\n";
        return 2;
    }

    Runtime::ECS::World world;
    const auto cube = world.CreateEntity();
    world.AddComponent<Runtime::ECS::Components::Transform3D>(cube);
    Runtime::ECS::Components::Rotation rotation;
    rotation.axis = glm::normalize(glm::vec3(0.5f, 1.0f, 0.0f));
    rotation.speed = 1.0f;
    world.AddComponent<Runtime::ECS::Components::Rotation>(cube, rotation);
    world.AddComponent<Sample::Components::CubeRenderComponent>(cube);

    Runtime::Graphics::Camera camera;
    camera.SetPosition(glm::vec3(2.8f, 2.1f, 5.2f));
    camera.SetTarget(glm::vec3(0.0f));

    auto& input = Runtime::Core::Input::Get();
    bool paused = false;
    int frameCount = 0;
    int result = 0;
    std::vector<unsigned char> firstFrame;
    const auto handler = input.RegisterEventHandler([&](const Runtime::Core::InputEvent& event) {
        if (event.type != Runtime::Core::InputEventType::KeyDown) {
            return;
        }
        if (event.key == Runtime::Core::Key_Escape) {
            engine.Exit();
        } else if (event.key == Runtime::Core::Key_Space) {
            paused = !paused;
        }
    });
    engine.Run(
        [&](float dt) {
            if (!paused) {
                Sample::Systems::CubeRotationSystem::Update(world, smokeTest ? 0.5f : dt);
            }
        },
        [&]() {
            int width = 0;
            int height = 0;
            glfwGetFramebufferSize(window.GetNativeWindow(), &width, &height);
            if (width <= 0 || height <= 0) {
                return;
            }
            renderer.SetViewport(0, 0, width, height);
            camera.SetPerspective(glm::radians(45.0f),
                static_cast<float>(width) / static_cast<float>(height), 0.1f, 100.0f);
            renderer.Clear();
            Sample::Systems::CubeRenderSystem::Render(world, renderer, mesh, material, camera);

            if (smokeTest) {
                std::vector<unsigned char> pixels(static_cast<size_t>(width) * height * 4);
                glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
                if (frameCount == 0) {
                    firstFrame = std::move(pixels);
                } else {
                    if (pixels.size() != firstFrame.size() || pixels == firstFrame || glGetError() != GL_NO_ERROR) {
                        std::cerr << "[MetalCubeSample] smoke test failed: no rendered rotation or GL error\n";
                        result = 3;
                    } else {
                        std::cout << "[MetalCubeSample] smoke test passed: rendered frames differ\n";
                    }
                    engine.Exit();
                }
            }
            ++frameCount;
            window.SwapBuffers();
        }
    );
    input.UnregisterEventHandler(handler);
    return result;
}

} // namespace

int main(int argc, char** argv) {
    if (argc > 2 || (argc == 2 && std::string_view(argv[1]) != "--smoke-test")) {
        std::cerr << "Usage: MetalCubeSample [--smoke-test]\n";
        return 1;
    }
    auto& engine = Runtime::Core::Engine::GetEngine();
    if (!engine.Initialize(1280, 720, "Metal Cube Sample | Space: Pause/Resume | Esc: Quit")) {
        std::cerr << "[MetalCubeSample] failed to initialize engine\n";
        return 1;
    }
    // All sample GL resources must be released before destroying the context.
    const int result = RunSample(engine, argc == 2);
    engine.Shutdown();
    return result;
}
