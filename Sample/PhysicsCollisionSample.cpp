#include <Core/Engine.h>
#include <Core/Input.h>
#include <Graphics/Camera.h>
#include <Graphics/Material.h>
#include <Graphics/MeshManager.h>
#include <Graphics/Renderer.h>
#include <Physics/Collider.h>
#include <Physics/CollisionShape.h>
#include <Physics/PhysicsWorld.h>
#include <Platform/Window.h>

#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace {

enum class ShapeType : std::size_t { Cube6, Bipyramid14, Bipyramid16, Bipyramid26, Count };

struct PolyhedronData {
    std::vector<glm::vec3> hullVertices;
    std::vector<float> renderVertices;
};

void AppendTriangle(std::vector<float>& vertices, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
    const glm::vec3 normal = glm::normalize(glm::cross(b - a, c - a));
    for (const glm::vec3& point : {a, b, c}) {
        vertices.insert(vertices.end(), {point.x, point.y, point.z, normal.x, normal.y, normal.z});
    }
}

PolyhedronData BuildCube(float halfExtent) {
    PolyhedronData data;
    data.hullVertices = {
        {-halfExtent, -halfExtent, -halfExtent}, {-halfExtent, -halfExtent, halfExtent},
        {-halfExtent, halfExtent, -halfExtent}, {-halfExtent, halfExtent, halfExtent},
        {halfExtent, -halfExtent, -halfExtent}, {halfExtent, -halfExtent, halfExtent},
        {halfExtent, halfExtent, -halfExtent}, {halfExtent, halfExtent, halfExtent},
    };
    const int faces[][4] = {
        {1, 5, 7, 3}, {4, 0, 2, 6}, {3, 7, 6, 2},
        {0, 4, 5, 1}, {5, 4, 6, 7}, {0, 1, 3, 2},
    };
    for (const auto& face : faces) {
        AppendTriangle(data.renderVertices, data.hullVertices[face[0]], data.hullVertices[face[1]], data.hullVertices[face[2]]);
        AppendTriangle(data.renderVertices, data.hullVertices[face[0]], data.hullVertices[face[2]], data.hullVertices[face[3]]);
    }
    return data;
}

PolyhedronData BuildBipyramid(int sideCount, float radius, float halfHeight) {
    PolyhedronData data;
    constexpr float pi = 3.14159265358979323846f;
    for (int index = 0; index < sideCount; ++index) {
        const float angle = 2.0f * pi * static_cast<float>(index) / static_cast<float>(sideCount);
        data.hullVertices.emplace_back(radius * std::cos(angle), 0.0f, radius * std::sin(angle));
    }
    const glm::vec3 top(0.0f, halfHeight, 0.0f);
    const glm::vec3 bottom(0.0f, -halfHeight, 0.0f);
    data.hullVertices.push_back(top);
    data.hullVertices.push_back(bottom);
    for (int index = 0; index < sideCount; ++index) {
        const glm::vec3& current = data.hullVertices[static_cast<std::size_t>(index)];
        const glm::vec3& next = data.hullVertices[static_cast<std::size_t>((index + 1) % sideCount)];
        AppendTriangle(data.renderVertices, top, current, next);
        AppendTriangle(data.renderVertices, bottom, next, current);
    }
    return data;
}

const char* ShapeName(ShapeType type) {
    switch (type) {
    case ShapeType::Cube6: return "6-face cube";
    case ShapeType::Bipyramid14: return "14-face bipyramid";
    case ShapeType::Bipyramid16: return "16-face bipyramid";
    case ShapeType::Bipyramid26: return "26-face bipyramid";
    default: return "unknown";
    }
}

class PhysicsCollisionSample {
public:
    bool Initialize(Runtime::Core::Engine& engine) {
        m_renderer = engine.GetRenderer();
        m_window = engine.GetWindow();
        if (!m_renderer || !m_window) return false;

        static const char* vertexShader = R"glsl(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
uniform mat4 u_Model;
uniform mat4 u_ModelViewProj;
out vec3 vNormal;
void main(){vNormal=mat3(transpose(inverse(u_Model)))*aNormal;gl_Position=u_ModelViewProj*vec4(aPos,1.0);}
)glsl";
        static const char* fragmentShader = R"glsl(
#version 330 core
in vec3 vNormal;
out vec4 FragColor;
void main(){float l=0.25+0.75*max(dot(normalize(vNormal),normalize(vec3(0.4,1.0,0.3))),0.0);FragColor=vec4(vec3(0.2,0.65,0.9)*l,1.0);}
)glsl";
        if (!m_renderer->LoadShaderFromSource("physics_sample", vertexShader, fragmentShader)) return false;
        m_material.SetShader(m_renderer->GetShader("physics_sample"));

        std::array<PolyhedronData, static_cast<std::size_t>(ShapeType::Count)> data = {
            BuildCube(0.35f), BuildBipyramid(7, 0.42f, 0.48f),
            BuildBipyramid(8, 0.42f, 0.48f), BuildBipyramid(13, 0.42f, 0.48f),
        };
        for (std::size_t index = 0; index < data.size(); ++index) {
            m_meshes[index] = m_meshManager.CreateFromVertices(
                Runtime::Graphics::MeshHandle{3000u + index}, data[index].renderVertices.data(), data[index].renderVertices.size());
            m_shapes[index] = std::make_shared<Runtime::Physics::ConvexHullShape>(std::move(data[index].hullVertices));
            if (!m_meshes[index]) return false;
        }

        m_camera.SetPosition(glm::vec3(8.0f, 7.0f, 11.0f));
        m_camera.SetTarget(glm::vec3(0.0f, 2.5f, 0.0f));
        const float aspect = static_cast<float>(m_window->GetWidth()) / static_cast<float>(std::max(1, m_window->GetHeight()));
        m_camera.SetPerspective(glm::radians(48.0f), aspect, 0.1f, 100.0f);
        m_world.SetFixedTimeStep(1.0f / 120.0f);
        m_world.SetGravity(glm::vec3(0.0f, -10.5f, 0.0f));
        m_world.SetContinuousCollisionEnabled(false);
        m_world.SetSolverIterations(6);

        Runtime::Physics::RigidBodyDesc ground;
        ground.position = glm::vec3(0.0f, -0.5f, 0.0f);
        ground.isStatic = true;
        ground.useGravity = false;
        m_groundId = m_world.CreateRigidBody(ground);
        Runtime::Physics::ColliderDesc collider;
        collider.shape = std::make_shared<Runtime::Physics::BoxShape>(glm::vec3(5.0f, 0.5f, 5.0f));
        collider.material.dynamicFriction = 0.6f;
        collider.material.staticFriction = 0.7f;
        return m_world.AttachCollider(m_groundId, collider);
    }

    void Update(float dt, const Runtime::Core::Input& input) {
        ShapeType requested = m_shapeType;
        if (input.IsKeyPressed(Runtime::Core::Key_1)) requested = ShapeType::Cube6;
        if (input.IsKeyPressed(Runtime::Core::Key_2)) requested = ShapeType::Bipyramid14;
        if (input.IsKeyPressed(Runtime::Core::Key_3)) requested = ShapeType::Bipyramid16;
        if (input.IsKeyPressed(Runtime::Core::Key_4)) requested = ShapeType::Bipyramid26;
        if (requested != m_shapeType) { m_shapeType = requested; Reset(); }
        if (input.IsKeyPressed(Runtime::Core::Key_Equal)) m_targetCount = std::min<std::size_t>(m_targetCount + 32, 1024);
        if (input.IsKeyPressed(Runtime::Core::Key_Minus)) m_targetCount = m_targetCount > 32 ? m_targetCount - 32 : 0;
        if (input.IsKeyPressed(Runtime::Core::Key_R)) Reset();
        if (input.IsKeyPressed(Runtime::Core::Key_C)) {
            m_ccdEnabled = !m_ccdEnabled;
            m_world.SetContinuousCollisionEnabled(m_ccdEnabled);
        }

        while (m_bodies.size() > m_targetCount) {
            m_world.DestroyRigidBody(m_bodies.back());
            m_bodies.pop_back();
        }

        UpdateFps(dt);
        if (!m_spawnLimitedByFps) {
            m_spawnAccumulator += dt * 24.0f;
            while (m_spawnAccumulator >= 1.0f && m_bodies.size() < m_targetCount) {
                m_spawnAccumulator -= 1.0f;
                SpawnBody();
            }
        }
        m_world.Step(dt);
        UpdateTitle();
    }

    void Render() {
        m_renderer->Clear();
        glm::mat4 groundModel = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -0.5f, 0.0f));
        groundModel = glm::scale(groundModel, glm::vec3(5.0f / 0.35f, 0.5f / 0.35f, 5.0f / 0.35f));
        m_renderer->Submit(*m_meshes[static_cast<std::size_t>(ShapeType::Cube6)], m_material, groundModel);

        const auto& mesh = m_meshes[static_cast<std::size_t>(m_shapeType)];
        for (uint32_t bodyId : m_bodies) {
            const auto* body = m_world.GetRigidBody(bodyId);
            if (!body) continue;
            glm::mat4 model = glm::translate(glm::mat4(1.0f), body->Position());
            model *= glm::mat4_cast(body->Orientation());
            m_renderer->Submit(*mesh, m_material, model);
        }
        m_renderer->Flush(m_camera);
        m_window->SwapBuffers();
    }

    void Shutdown() {
        for (auto& mesh : m_meshes) {
            if (mesh) mesh->Destroy();
            mesh.reset();
        }
        m_meshManager.Clear();
    }

private:
    void SpawnBody() {
        Runtime::Physics::RigidBodyDesc body;
        body.position = m_spawnPosition;
        body.orientation = glm::angleAxis(
            0.37f * static_cast<float>(m_spawnSequence++ % 11u),
            glm::normalize(glm::vec3(0.6f, 1.0f, 0.35f)));
        body.angularVelocity = glm::vec3(0.3f, 0.6f, 0.2f);
        body.mass = 1.0f;
        body.inertiaTensorDiagonal = glm::vec3(0.18f);
        const uint32_t bodyId = m_world.CreateRigidBody(body);
        Runtime::Physics::ColliderDesc collider;
        collider.shape = m_shapes[static_cast<std::size_t>(m_shapeType)];
        collider.material.restitution = 0.1f;
        collider.material.dynamicFriction = 0.55f;
        collider.material.staticFriction = 0.65f;
        if (m_world.AttachCollider(bodyId, collider)) m_bodies.push_back(bodyId);
        else m_world.DestroyRigidBody(bodyId);
    }

    void Reset() {
        for (uint32_t bodyId : m_bodies) m_world.DestroyRigidBody(bodyId);
        m_bodies.clear();
        m_spawnAccumulator = 0.0f;
        m_spawnSequence = 0;
        m_spawnLimitedByFps = false;
    }

    void UpdateFps(float dt) {
        m_fpsAccumulator += dt;
        ++m_frameCount;
        if (m_fpsAccumulator < 0.5f) return;
        m_fps = static_cast<float>(m_frameCount) / m_fpsAccumulator;
        m_fpsAccumulator = 0.0f;
        m_frameCount = 0;
        if (m_fps <= kSpawnFpsLimit && !m_bodies.empty()) m_spawnLimitedByFps = true;
        std::cout << "[PhysicsCollisionSample] shape=" << ShapeName(m_shapeType)
                  << " bodies=" << m_bodies.size() << " contacts=" << m_world.Contacts().size()
                  << " fps=" << static_cast<int>(std::round(m_fps))
                  << " spawning=" << (m_spawnLimitedByFps ? "stopped" : "active") << '\n';
    }

    void UpdateTitle() {
        if (!m_window->GetNativeWindow()) return;
        const std::string title = std::string("Physics Collision Sample | ") + ShapeName(m_shapeType) +
            " | Bodies=" + std::to_string(m_bodies.size()) + "/" + std::to_string(m_targetCount) +
            " | Contacts=" + std::to_string(m_world.Contacts().size()) +
            " | FPS=" + std::to_string(static_cast<int>(std::round(m_fps))) +
            " | Spawn=" + (m_spawnLimitedByFps ? "STOPPED@60" : "ACTIVE") +
            " | 1-4 shape, +/- count, C CCD, R reset, Esc quit";
        glfwSetWindowTitle(m_window->GetNativeWindow(), title.c_str());
    }

    static constexpr float kSpawnFpsLimit = 60.0f;
    const glm::vec3 m_spawnPosition{0.0f, 8.0f, 0.0f};
    Runtime::Graphics::Renderer* m_renderer = nullptr;
    Runtime::Platform::Window* m_window = nullptr;
    Runtime::Physics::PhysicsWorld m_world;
    Runtime::Graphics::MeshManager m_meshManager;
    Runtime::Graphics::Material m_material;
    Runtime::Graphics::Camera m_camera;
    std::array<std::shared_ptr<Runtime::Graphics::Mesh>, static_cast<std::size_t>(ShapeType::Count)> m_meshes;
    std::array<std::shared_ptr<Runtime::Physics::CollisionShape>, static_cast<std::size_t>(ShapeType::Count)> m_shapes;
    std::vector<uint32_t> m_bodies;
    ShapeType m_shapeType = ShapeType::Bipyramid14;
    std::size_t m_targetCount = 1024;
    uint32_t m_groundId = 0;
    uint32_t m_spawnSequence = 0;
    float m_spawnAccumulator = 0.0f;
    bool m_ccdEnabled = false;
    bool m_spawnLimitedByFps = false;
    float m_fps = 0.0f;
    float m_fpsAccumulator = 0.0f;
    uint32_t m_frameCount = 0;
};

} // namespace

int main() {
    auto& engine = Runtime::Core::Engine::GetEngine();
    if (!engine.Initialize(1280, 720, "Physics Collision Sample")) return 1;
    PhysicsCollisionSample sample;
    if (!sample.Initialize(engine)) { engine.Shutdown(); return 2; }
    auto& input = Runtime::Core::Input::Get();
    engine.Run(
        [&](float dt) {
            if (input.IsKeyPressed(Runtime::Core::Key_Escape)) { engine.Exit(); return; }
            sample.Update(dt, input);
        },
        [&]() { sample.Render(); });
    sample.Shutdown();
    engine.Shutdown();
    return 0;
}
