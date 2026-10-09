#define GLFW_INCLUDE_NONE
#include "instanced.vert.h"
#include "instanced.frag.h"

#include <Core/Engine.h>
#include <Core/Input.h>
#include <Graphics/Camera.h>
#include <Platform/Window.h>

#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr float kStep = 1.0f / 120.0f;
constexpr float kHalfExtent = 0.45f;
constexpr std::size_t kMaxBodies = 500;

namespace Layers {
constexpr JPH::ObjectLayer Static = 0;
constexpr JPH::ObjectLayer Dynamic = 1;
}

class PairFilter final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer first, JPH::ObjectLayer second) const override {
        return first == Layers::Dynamic || second == Layers::Dynamic;
    }
};

class BroadPhaseLayers final : public JPH::BroadPhaseLayerInterface {
public:
    JPH::uint GetNumBroadPhaseLayers() const override { return 2; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
        return JPH::BroadPhaseLayer(static_cast<JPH::BroadPhaseLayer::Type>(layer));
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override {
        return layer == JPH::BroadPhaseLayer(Layers::Static) ? "static" : "dynamic";
    }
#endif
};

class BroadPhaseFilter final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer broadPhaseLayer) const override {
        return layer == Layers::Dynamic || broadPhaseLayer == JPH::BroadPhaseLayer(Layers::Dynamic);
    }
};

class BoxField {
public:
    BoxField()
        : m_jobs(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, -1) {
        m_physics.Init(1024, 0, 8192, 8192, m_layers, m_broadPhaseFilter, m_pairFilter);
        m_physics.SetGravity(JPH::Vec3(0.0f, -10.5f, 0.0f));
        Reset();
    }

    void Reset() {
        auto& bodies = m_physics.GetBodyInterface();
        for (const JPH::BodyID id : m_boxes) {
            bodies.RemoveBody(id);
            bodies.DestroyBody(id);
        }
        m_boxes.clear();
        if (!m_ground.IsInvalid()) {
            bodies.RemoveBody(m_ground);
            bodies.DestroyBody(m_ground);
        }

        JPH::BodyCreationSettings ground(
            new JPH::BoxShape(JPH::Vec3(20.0f, 0.5f, 20.0f), 0.0f),
            JPH::RVec3(0.0f, -0.5f, 0.0f), JPH::Quat::sIdentity(),
            JPH::EMotionType::Static, Layers::Static);
        ground.mFriction = 0.7f;
        ground.mRestitution = 0.1f;
        m_ground = bodies.CreateAndAddBody(ground, JPH::EActivation::DontActivate);
        m_random.seed(1337);
        m_spawnAccumulator = 0.0f;
        m_stepAccumulator = 0.0f;
        m_stepCount = 0;
    }

    void SetSleepEnabled(bool enabled) {
        m_sleepEnabled = enabled;
        for (const JPH::BodyID id : m_boxes) {
            {
                JPH::BodyLockWrite lock(m_physics.GetBodyLockInterface(), id);
                lock.GetBody().SetAllowSleeping(enabled);
            }
            if (!enabled) m_physics.GetBodyInterface().ActivateBody(id);
        }
    }

    void Update(float dt) {
        if (!m_paused) {
            m_spawnAccumulator += dt * m_spawnRate;
            while (m_spawnAccumulator >= 1.0f && m_boxes.size() < kMaxBodies) {
                m_spawnAccumulator -= 1.0f;
                Spawn();
            }
        }

        auto& bodies = m_physics.GetBodyInterface();
        for (std::size_t index = 0; index < m_boxes.size();) {
            if (bodies.GetPosition(m_boxes[index]).GetY() < -15.0f) {
                bodies.RemoveBody(m_boxes[index]);
                bodies.DestroyBody(m_boxes[index]);
                m_boxes.erase(m_boxes.begin() + static_cast<std::ptrdiff_t>(index));
            } else {
                ++index;
            }
        }

        m_stepAccumulator += dt;
        while (m_stepAccumulator >= kStep) {
            m_physics.Update(kStep, 1, &m_allocator, &m_jobs);
            m_stepAccumulator -= kStep;
            ++m_stepCount;
        }
    }

    void Spawn() {
        auto& bodies = m_physics.GetBodyInterface();
        JPH::BodyCreationSettings box(
            new JPH::BoxShape(JPH::Vec3::sReplicate(kHalfExtent), 0.0f),
            JPH::RVec3(m_position(m_random), 14.0f, m_position(m_random)),
            JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::Dynamic);
        box.mFriction = 0.65f;
        box.mRestitution = 0.05f;
        box.mAllowSleeping = m_sleepEnabled;
        box.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        box.mMassPropertiesOverride.mMass = 1.0f;
        m_boxes.push_back(bodies.CreateAndAddBody(box, JPH::EActivation::Activate));
    }

    std::size_t SleepingCount() const {
        const auto& bodies = m_physics.GetBodyInterface();
        return std::count_if(m_boxes.begin(), m_boxes.end(), [&](JPH::BodyID id) { return !bodies.IsActive(id); });
    }

    void LogState() const {
        const auto& bodies = m_physics.GetBodyInterface();
        std::size_t resting = 0;
        double speedSum = 0.0;
        float maxSpeed = 0.0f;
        for (const JPH::BodyID id : m_boxes) {
            const JPH::RVec3 position = bodies.GetPosition(id);
            if (position.GetY() < 3.0f) {
                ++resting;
                const float speed = bodies.GetLinearVelocity(id).Length();
                speedSum += speed;
                maxSpeed = std::max(maxSpeed, speed);
            }
        }
        std::cout << "[JoltBoxFill] t=" << m_stepCount * kStep
                  << " boxes=" << m_boxes.size() << " sleeping=" << SleepingCount()
                  << " lowBoxes=" << resting << " lowMeanSpeed=" << (resting ? speedSum / resting : 0.0)
                  << " lowMaxSpeed=" << maxSpeed << std::endl;
    }

    const std::vector<JPH::BodyID>& Boxes() const { return m_boxes; }
    const JPH::BodyInterface& Bodies() const { return m_physics.GetBodyInterface(); }
    float& SpawnRate() { return m_spawnRate; }
    bool& Paused() { return m_paused; }
    bool SleepEnabled() const { return m_sleepEnabled; }
    uint64_t StepCount() const { return m_stepCount; }

private:
    BroadPhaseLayers m_layers;
    BroadPhaseFilter m_broadPhaseFilter;
    PairFilter m_pairFilter;
    JPH::TempAllocatorImpl m_allocator{16 * 1024 * 1024};
    JPH::JobSystemThreadPool m_jobs;
    JPH::PhysicsSystem m_physics;
    JPH::BodyID m_ground;
    std::vector<JPH::BodyID> m_boxes;
    std::mt19937 m_random{1337};
    std::uniform_real_distribution<float> m_position{-3.5f, 3.5f};
    float m_spawnRate = 24.0f;
    float m_spawnAccumulator = 0.0f;
    float m_stepAccumulator = 0.0f;
    uint64_t m_stepCount = 0;
    bool m_paused = false;
    bool m_sleepEnabled = true;
};

class BoxRenderer {
public:
    bool Initialize() {
        const auto compile = [](GLenum type, const char* source) {
            GLuint shader = glCreateShader(type);
            glShaderSource(shader, 1, &source, nullptr);
            glCompileShader(shader);
            GLint success = 0;
            glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if (!success) {
                char message[512];
                glGetShaderInfoLog(shader, sizeof(message), nullptr, message);
                std::cerr << message << '\n';
                glDeleteShader(shader);
                return GLuint(0);
            }
            return shader;
        };
        const GLuint vertex = compile(GL_VERTEX_SHADER, Sample::kJoltVertexShader);
        const GLuint fragment = compile(GL_FRAGMENT_SHADER, Sample::kJoltFragmentShader);
        if (!vertex || !fragment) return false;
        m_program = glCreateProgram();
        glAttachShader(m_program, vertex);
        glAttachShader(m_program, fragment);
        glLinkProgram(m_program);
        glDeleteShader(vertex);
        glDeleteShader(fragment);
        GLint linked = 0;
        glGetProgramiv(m_program, GL_LINK_STATUS, &linked);
        if (!linked) return false;

        const std::array<glm::vec3, 8> corners = {
            glm::vec3(-1,-1,-1), glm::vec3(-1,-1,1), glm::vec3(-1,1,-1), glm::vec3(-1,1,1),
            glm::vec3(1,-1,-1), glm::vec3(1,-1,1), glm::vec3(1,1,-1), glm::vec3(1,1,1)
        };
        const int faces[][4] = {{1,5,7,3}, {4,0,2,6}, {3,7,6,2}, {0,4,5,1}, {5,4,6,7}, {0,1,3,2}};
        std::vector<float> vertices;
        for (const auto& face : faces) {
            const auto append = [&](int first, int second, int third) {
                const glm::vec3 normal = glm::normalize(glm::cross(
                    corners[second] - corners[first], corners[third] - corners[first]));
                for (const int index : {first, second, third}) {
                    const glm::vec3 point = corners[index];
                    vertices.insert(vertices.end(), {point.x, point.y, point.z, normal.x, normal.y, normal.z});
                }
            };
            append(face[0], face[1], face[2]);
            append(face[0], face[2], face[3]);
        }
        glGenVertexArrays(1, &m_vao);
        glGenBuffers(1, &m_vertices);
        glGenBuffers(1, &m_instances);
        glBindVertexArray(m_vao);
        glBindBuffer(GL_ARRAY_BUFFER, m_vertices);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
        for (GLuint index = 0; index < 2; ++index) {
            glEnableVertexAttribArray(index);
            glVertexAttribPointer(index, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                reinterpret_cast<void*>(static_cast<std::size_t>(index) * 3 * sizeof(float)));
        }
        glBindBuffer(GL_ARRAY_BUFFER, m_instances);
        glBufferData(GL_ARRAY_BUFFER, 501 * 19 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
        for (GLuint index = 0; index < 4; ++index) {
            glEnableVertexAttribArray(2 + index);
            glVertexAttribPointer(2 + index, 4, GL_FLOAT, GL_FALSE, 19 * sizeof(float),
                reinterpret_cast<void*>(static_cast<std::size_t>(index) * 4 * sizeof(float)));
            glVertexAttribDivisor(2 + index, 1);
        }
        glEnableVertexAttribArray(6);
        glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, 19 * sizeof(float),
            reinterpret_cast<void*>(16 * sizeof(float)));
        glVertexAttribDivisor(6, 1);
        glBindVertexArray(0);
        return true;
    }

    void Draw(const glm::mat4& viewProjection, const std::vector<float>& instances) {
        glUseProgram(m_program);
        glUniformMatrix4fv(glGetUniformLocation(m_program, "uViewProj"), 1, GL_FALSE, &viewProjection[0][0]);
        glBindBuffer(GL_ARRAY_BUFFER, m_instances);
        glBufferSubData(GL_ARRAY_BUFFER, 0, instances.size() * sizeof(float), instances.data());
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glBindVertexArray(m_vao);
        glDrawArraysInstanced(GL_TRIANGLES, 0, 36, static_cast<GLsizei>(instances.size() / 19));
        glBindVertexArray(0);
        glUseProgram(0);
    }

    void Shutdown() {
        glDeleteProgram(m_program);
        glDeleteBuffers(1, &m_vertices);
        glDeleteBuffers(1, &m_instances);
        glDeleteVertexArrays(1, &m_vao);
    }

private:
    GLuint m_program = 0;
    GLuint m_vao = 0;
    GLuint m_vertices = 0;
    GLuint m_instances = 0;
};

class JoltBoxFillSample {
public:
    bool Initialize(Runtime::Platform::Window* window) {
        m_window = window;
        if (!m_renderer.Initialize()) return false;
        m_camera.SetPerspective(glm::radians(48.0f),
            static_cast<float>(window->GetWidth()) / std::max(1, window->GetHeight()), 0.1f, 300.0f);
        const glm::vec3 target = glm::normalize(glm::vec3(0,1,0) - m_cameraPosition);
        m_yaw = std::atan2(target.x, -target.z);
        m_pitch = std::asin(target.y);
        glfwSetInputMode(window->GetNativeWindow(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        return true;
    }

    void Update(float dt, const Runtime::Core::Input& input) {
        if (input.IsKeyPressed(Runtime::Core::Key_R)) m_field.Reset();
        if (input.IsKeyPressed(Runtime::Core::Key_S)) m_field.SetSleepEnabled(!m_field.SleepEnabled());
        if (input.IsKeyPressed(Runtime::Core::Key_Space)) m_field.Paused() = !m_field.Paused();
        if (input.IsKeyPressed(Runtime::Core::Key_Equal)) m_field.SpawnRate() = std::min(240.0f, m_field.SpawnRate() * 1.5f);
        if (input.IsKeyPressed(Runtime::Core::Key_Minus)) m_field.SpawnRate() = std::max(0.25f, m_field.SpawnRate() / 1.5f);
        if (input.IsKeyPressed(Runtime::Core::Key_L)) m_logging = !m_logging;

        const bool showCursor = glfwGetKey(m_window->GetNativeWindow(), GLFW_KEY_LEFT_ALT) == GLFW_PRESS ||
            glfwGetKey(m_window->GetNativeWindow(), GLFW_KEY_RIGHT_ALT) == GLFW_PRESS;
        if (m_cursorCaptured == showCursor) {
            m_cursorCaptured = !showCursor;
            glfwSetInputMode(m_window->GetNativeWindow(), GLFW_CURSOR,
                m_cursorCaptured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
        }
        if (m_cursorCaptured) {
            m_yaw += static_cast<float>(input.GetMouseDeltaX()) * 0.0025f;
            m_pitch -= static_cast<float>(input.GetMouseDeltaY()) * 0.0025f;
        }
        m_pitch = glm::clamp(m_pitch, -1.55f, 1.55f);
        const float scroll = input.GetScrollDelta();
        if (scroll != 0.0f) m_speed = glm::clamp(m_speed * (scroll > 0 ? 1.2f : 1.0f / 1.2f), 0.5f, 80.0f);
        const glm::vec3 forward(std::cos(m_pitch) * std::sin(m_yaw), std::sin(m_pitch),
            -std::cos(m_pitch) * std::cos(m_yaw));
        const glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0,1,0)));
        glm::vec3 move(0.0f);
        if (input.IsKeyDown(Runtime::Core::Key_W)) move += forward;
        if (input.IsKeyDown(Runtime::Core::Key_X)) move -= forward;
        if (input.IsKeyDown(Runtime::Core::Key_A)) move -= right;
        if (input.IsKeyDown(Runtime::Core::Key_D)) move += right;
        if (input.IsKeyDown(Runtime::Core::Key_Q)) move.y -= 1.0f;
        if (input.IsKeyDown(Runtime::Core::Key_E)) move.y += 1.0f;
        if (glm::dot(move, move) > 0.0f) m_cameraPosition += glm::normalize(move) * m_speed * dt;
        m_camera.SetPosition(m_cameraPosition);
        m_camera.SetTarget(m_cameraPosition + forward);
        Runtime::Core::Input::Get().UpdateMouseDelta();

        m_field.Update(dt);
        if (m_logging && m_field.StepCount() >= m_nextLog) {
            m_field.LogState();
            m_nextLog = m_field.StepCount() + 120;
        }
        const std::string title = "Jolt Box Fill | Bodies=" + std::to_string(m_field.Boxes().size()) +
            " | Sleeping=" + std::to_string(m_field.SleepingCount()) +
            " | W/X/A/D/Q/E fly, mouse look (Alt cursor), +/- rate, Space spawn, S sleep, L log, R reset";
        glfwSetWindowTitle(m_window->GetNativeWindow(), title.c_str());
    }

    void Render() {
        glClearColor(0.12f, 0.13f, 0.16f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        m_instances.clear();
        AddInstance(glm::vec3(0, -0.5f, 0), glm::quat(1,0,0,0), glm::vec3(20,0.5f,20), glm::vec3(0.45f,0.45f,0.5f));
        for (const JPH::BodyID id : m_field.Boxes()) {
            JPH::RVec3 position;
            JPH::Quat rotation;
            m_field.Bodies().GetPositionAndRotation(id, position, rotation);
            AddInstance(glm::vec3(position.GetX(), position.GetY(), position.GetZ()),
                glm::quat(rotation.GetW(), rotation.GetX(), rotation.GetY(), rotation.GetZ()),
                glm::vec3(kHalfExtent),
                m_field.Bodies().IsActive(id) ? glm::vec3(0.25f,0.70f,0.35f) : glm::vec3(0.82f,0.22f,0.22f));
        }
        m_renderer.Draw(m_camera.GetProjection() * m_camera.GetView(), m_instances);
        m_window->SwapBuffers();
    }

    void Shutdown() { m_renderer.Shutdown(); }

private:
    void AddInstance(const glm::vec3& position, const glm::quat& rotation,
        const glm::vec3& scale, const glm::vec3& color) {
        glm::mat4 model = glm::scale(glm::translate(glm::mat4(1.0f), position) * glm::mat4_cast(rotation), scale);
        for (int column = 0; column < 4; ++column)
            for (int row = 0; row < 4; ++row)
                m_instances.push_back(model[column][row]);
        m_instances.insert(m_instances.end(), {color.x, color.y, color.z});
    }

    Runtime::Platform::Window* m_window = nullptr;
    BoxField m_field;
    BoxRenderer m_renderer;
    Runtime::Graphics::Camera m_camera;
    std::vector<float> m_instances;
    glm::vec3 m_cameraPosition{14.0f, 10.0f, 14.0f};
    float m_yaw = 0.0f;
    float m_pitch = 0.0f;
    float m_speed = 10.0f;
    uint64_t m_nextLog = 120;
    bool m_cursorCaptured = true;
    bool m_logging = true;
};

} // namespace

int main(int argc, char** argv) {
    bool headless = false;
    bool noSleep = false;
    int steps = 3600;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--headless") headless = true;
        if (argument == "--no-sleep") noSleep = true;
        if (argument == "--steps" && index + 1 < argc) steps = std::stoi(argv[++index]);
    }
    JPH::RegisterDefaultAllocator();
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();
    int result = 0;
    {
        if (headless) {
            BoxField field;
            if (noSleep) field.SetSleepEnabled(false);
            for (int step = 0; step < steps; ++step) {
                field.Update(kStep);
                if ((step + 1) % 120 == 0) field.LogState();
            }
        } else {
            auto& engine = Runtime::Core::Engine::GetEngine();
            if (!engine.Initialize(1280, 720, "Jolt Box Fill")) {
                result = 1;
            } else {
                {
                    JoltBoxFillSample sample;
                    if (sample.Initialize(engine.GetWindow())) {
                        auto& input = Runtime::Core::Input::Get();
                        engine.Run([&](float dt) {
                            if (input.IsKeyPressed(Runtime::Core::Key_Escape)) engine.Exit();
                            else sample.Update(dt, input);
                        }, [&]() { sample.Render(); });
                        sample.Shutdown();
                    } else result = 2;
                }
                engine.Shutdown();
            }
        }
    }
    JPH::UnregisterTypes();
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;
    return result;
}