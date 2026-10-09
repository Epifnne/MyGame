// Box stack / rain visualization sample for Phase 9 island sleep.
//
// Field boxes rain continuously; tower modes build five-box structures, so you can watch:
//   - a stack FORM (not pre-placed) and whether/when it sleeps (red),
//   - the drift/tilt of the five-box tower when sleep is disabled with S.
//
// Rendering uses a self-contained instanced pipeline (one instanced draw for
// every box plus the ground), bypassing the per-mesh Renderer queue so a few
// hundred boxes cost a single draw call.
//
// Camera (free-fly):
//   mouse  : look around (hold Alt to show cursor)
//   W/X    : forward / backward, A/D: strafe, Q/E: down/up
//   scroll : adjust fly speed
//
// Scene keys:
//   1 / 2 / 3 / 4 : field rain / five-box tower / four supports / four-level edge cradle
//   +/-    : increase / decrease spawn rate
//   Space  : pause / resume spawning
//   S      : toggle island sleep
//   L      : toggle box state logging (once per simulated second)
//   R      : reset the scene (same seed)
//   Esc    : quit
// Window.h pulls in GLFW/glfw3.h; suppress its built-in gl.h so the glad
// loader header can be included afterwards without a conflict.
#define GLFW_INCLUDE_NONE
#include "instanced.vert.h"
#include "instanced.frag.h"

#include <Core/Engine.h>
#include <Core/Input.h>
#include <Graphics/Camera.h>
#include <Physics/Collider.h>
#include <Physics/CollisionShape.h>
#include <Physics/PhysicsWorld.h>
#include <Platform/Window.h>

#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <stdexcept>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace {

constexpr uint32_t kSeed = 1337;
constexpr float kBoxHalfExtent = 0.45f;
constexpr float kSpawnHeight = 14.0f;
constexpr float kKillY = -15.0f;

enum class SceneMode : std::size_t { FieldRain, TowerRain, FourSupports, EdgeCradle, Count };

// Return the scene identifier shown in the window title.
const char* SceneName(SceneMode mode) {
    switch (mode) {
    case SceneMode::FieldRain: return "field-rain";
    case SceneMode::TowerRain: return "tower-rain";
    case SceneMode::FourSupports: return "four-supports";
    case SceneMode::EdgeCradle: return "four-level-edge-cradle";
    default: return "unknown";
    }
}

// ---------------------------------------------------------------------------
// Self-contained instanced box renderer (one draw call for all instances).
// ---------------------------------------------------------------------------

class InstancedBoxRenderer {
public:
    // Compile shaders and upload the unit cube once.
    bool Initialize() {
        const GLuint vs = Compile(GL_VERTEX_SHADER, Sample::kInstancedVertexShader);
        const GLuint fs = Compile(GL_FRAGMENT_SHADER, Sample::kInstancedFragmentShader);
        if (vs == 0 || fs == 0) return false;
        m_program = glCreateProgram();
        glAttachShader(m_program, vs);
        glAttachShader(m_program, fs);
        glLinkProgram(m_program);
        glDeleteShader(vs);
        glDeleteShader(fs);
        GLint linked = 0;
        glGetProgramiv(m_program, GL_LINK_STATUS, &linked);
        if (linked != GL_TRUE) {
            char log[512];
            glGetProgramInfoLog(m_program, sizeof(log), nullptr, log);
            std::cerr << "[BoxStackSample] shader link failed: " << log << '\n';
            return false;
        }
        m_locViewProj = glGetUniformLocation(m_program, "u_ViewProj");

        // Unit cube (half extent 1), 36 vertices of (pos3, normal3).
        std::vector<float> vertices;
        const std::array<glm::vec3, 8> c = {
            glm::vec3(-1, -1, -1), glm::vec3(-1, -1, 1), glm::vec3(-1, 1, -1), glm::vec3(-1, 1, 1),
            glm::vec3(1, -1, -1), glm::vec3(1, -1, 1), glm::vec3(1, 1, -1), glm::vec3(1, 1, 1),
        };
        const int faces[][4] = {
            {1, 5, 7, 3}, {4, 0, 2, 6}, {3, 7, 6, 2},
            {0, 4, 5, 1}, {5, 4, 6, 7}, {0, 1, 3, 2},
        };
        const auto appendTriangle = [&](const glm::vec3& a, const glm::vec3& b, const glm::vec3& d) {
            const glm::vec3 n = glm::normalize(glm::cross(b - a, d - a));
            for (const glm::vec3& p : {a, b, d}) {
                vertices.insert(vertices.end(), {p.x, p.y, p.z, n.x, n.y, n.z});
            }
        };
        for (const auto& face : faces) {
            appendTriangle(c[face[0]], c[face[1]], c[face[2]]);
            appendTriangle(c[face[0]], c[face[2]], c[face[3]]);
        }
        m_vertexCount = static_cast<GLsizei>(vertices.size() / 6);

        glGenVertexArrays(1, &m_vao);
        glGenBuffers(1, &m_vbo);
        glGenBuffers(1, &m_instanceVbo);
        glBindVertexArray(m_vao);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));

        // Per-instance data: mat4 model (attribs 2-5) + vec3 color (attrib 6).
        glBindBuffer(GL_ARRAY_BUFFER, m_instanceVbo);
        glBufferData(GL_ARRAY_BUFFER, kMaxInstances * kInstanceFloats * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
        constexpr GLsizei stride = kInstanceFloats * sizeof(float);
        for (GLuint column = 0; column < 4; ++column) {
            const GLuint attrib = 2 + column;
            glEnableVertexAttribArray(attrib);
            glVertexAttribPointer(attrib, 4, GL_FLOAT, GL_FALSE, stride,
                reinterpret_cast<void*>(column * 4 * sizeof(float)));
            glVertexAttribDivisor(attrib, 1);
        }
        glEnableVertexAttribArray(6);
        glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(16 * sizeof(float)));
        glVertexAttribDivisor(6, 1);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        return true;
    }

    // Release the OpenGL objects while the context is alive.
    void Shutdown() {
        if (m_program) glDeleteProgram(m_program);
        if (m_vao) glDeleteVertexArrays(1, &m_vao);
        if (m_vbo) glDeleteBuffers(1, &m_vbo);
        if (m_instanceVbo) glDeleteBuffers(1, &m_instanceVbo);
        m_program = m_vao = m_vbo = m_instanceVbo = 0;
    }

    // One instanced draw for the whole frame. instanceData packs
    // (model mat4, color vec3) = 19 floats per instance.
    void Draw(const glm::mat4& viewProj, const std::vector<float>& instanceData) {
        if (m_program == 0 || instanceData.empty()) return;
        const auto instanceCount = static_cast<GLsizei>(std::min(
            instanceData.size() / kInstanceFloats, static_cast<std::size_t>(kMaxInstances)));
        if (instanceCount == 0) return;

        glUseProgram(m_program);
        glUniformMatrix4fv(m_locViewProj, 1, GL_FALSE, &viewProj[0][0]);

        glBindBuffer(GL_ARRAY_BUFFER, m_instanceVbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
            instanceCount * kInstanceFloats * sizeof(float), instanceData.data());

        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);

        glBindVertexArray(m_vao);
        glDrawArraysInstanced(GL_TRIANGLES, 0, m_vertexCount, instanceCount);
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glUseProgram(0);
    }

private:
    // Compile a shader and report the driver log on failure.
    static GLuint Compile(GLenum type, const char* source) {
        const GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);
        GLint compiled = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
        if (compiled != GL_TRUE) {
            char log[512];
            glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
            std::cerr << "[BoxStackSample] shader compile failed: " << log << '\n';
            glDeleteShader(shader);
            return 0;
        }
        return shader;
    }

    static constexpr GLsizei kMaxInstances = 4096;
    static constexpr std::size_t kInstanceFloats = 19; // mat4 + vec3

    GLuint m_program = 0;
    GLuint m_vao = 0;
    GLuint m_vbo = 0;
    GLuint m_instanceVbo = 0;
    GLint m_locViewProj = -1;
    GLsizei m_vertexCount = 0;
};

// ---------------------------------------------------------------------------
// Sample
// ---------------------------------------------------------------------------

class BoxStackSample {
public:
    // Configure the camera and build the selected deterministic scene.
    bool Initialize(Runtime::Core::Engine& engine, SceneMode mode, bool noSleep) {
        m_window = engine.GetWindow();
        if (!m_window || !m_renderer.Initialize()) return false;
        m_mode = mode;
        m_sleepEnabled = !noSleep;
        m_spawnRate = mode == SceneMode::FieldRain ? 24.0f : 2.0f / 3.0f;

        m_boxShape = std::make_shared<Runtime::Physics::BoxShape>(glm::vec3(kBoxHalfExtent));

        m_camera.SetPerspective(
            glm::radians(48.0f),
            static_cast<float>(m_window->GetWidth()) / static_cast<float>(std::max(1, m_window->GetHeight())),
            0.1f, 300.0f);
        // Start looking straight at the drop zone center from a raised view.
        m_cameraPosition = glm::vec3(14.0f, 10.0f, 14.0f);
        const glm::vec3 toTarget = glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f) - m_cameraPosition);
        m_yaw = std::atan2(toTarget.x, -toTarget.z);
        m_pitch = std::asin(glm::clamp(toTarget.y, -1.0f, 1.0f));

        glfwSetInputMode(m_window->GetNativeWindow(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        BuildScene();
        return true;
    }

    // Process input and advance physics with the real frame time.
    void Update(float dt, const Runtime::Core::Input& input) {
        const bool showCursor =
            glfwGetKey(m_window->GetNativeWindow(), GLFW_KEY_LEFT_ALT) == GLFW_PRESS ||
            glfwGetKey(m_window->GetNativeWindow(), GLFW_KEY_RIGHT_ALT) == GLFW_PRESS;
        if (m_cursorCaptured == showCursor) {
            m_cursorCaptured = !showCursor;
            m_skipMouseLookFrames = 2;
            glfwSetInputMode(m_window->GetNativeWindow(), GLFW_CURSOR,
                m_cursorCaptured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
        }
        HandleCamera(dt, input);

        SceneMode requested = m_mode;
        if (input.IsKeyPressed(Runtime::Core::Key_1)) requested = SceneMode::FieldRain;
        if (input.IsKeyPressed(Runtime::Core::Key_2)) requested = SceneMode::TowerRain;
        if (input.IsKeyPressed(Runtime::Core::Key_3)) requested = SceneMode::FourSupports;
        if (input.IsKeyPressed(Runtime::Core::Key_4)) requested = SceneMode::EdgeCradle;
        if (requested != m_mode) {
            m_mode = requested;
            m_spawnRate = m_mode == SceneMode::FieldRain ? 24.0f : 2.0f / 3.0f;
            BuildScene();
        }
        if (input.IsKeyPressed(Runtime::Core::Key_Equal)) m_spawnRate = std::min(m_spawnRate * 1.5f, 240.0f);
        if (input.IsKeyPressed(Runtime::Core::Key_Minus)) m_spawnRate = std::max(m_spawnRate / 1.5f, 0.25f);
        if (input.IsKeyPressed(Runtime::Core::Key_Space)) m_spawnPaused = !m_spawnPaused;
        if (input.IsKeyPressed(Runtime::Core::Key_R)) BuildScene();
        if (input.IsKeyPressed(Runtime::Core::Key_S)) {
            m_sleepEnabled = !m_sleepEnabled;
            m_world->SetSleepEnabled(m_sleepEnabled);
        }
        if (input.IsKeyPressed(Runtime::Core::Key_L)) m_stateLogEnabled = !m_stateLogEnabled;

        // Stop spawning at the population cap; sleeping boxes stay in place.
        if (!m_spawnPaused) {
            m_spawnAccumulator += dt * m_spawnRate;
            while (m_spawnAccumulator >= 1.0f) {
                m_spawnAccumulator -= 1.0f;
                TrySpawn();
            }
        }
        RecycleFallen();

        m_world->Step(dt);
        m_sleepingCount = 0;
        for (const uint32_t bodyId : m_bodies) {
            const auto* body = m_world->GetRigidBody(bodyId);
            if (body && body->IsSleeping()) ++m_sleepingCount;
        }

        LogBoxStates();
        UpdateFps(dt);
        UpdateTitle();
    }

    // Draw awake boxes green and sleeping boxes red in one instanced batch.
    void Render() {
        glClearColor(0.12f, 0.13f, 0.16f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        m_instanceData.clear();
        // Ground as instance 0 (static grey).
        PushInstance(glm::vec3(0.0f, -0.5f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
            glm::vec3(20.0f, 0.5f, 20.0f), glm::vec3(0.45f, 0.45f, 0.50f));

        const glm::vec3 awakeColor(0.25f, 0.70f, 0.35f);
        const glm::vec3 sleepColor(0.82f, 0.22f, 0.22f);
        for (const uint32_t bodyId : m_bodies) {
            const auto* body = m_world->GetRigidBody(bodyId);
            if (!body) continue;
            PushInstance(body->Position(), body->Orientation(), glm::vec3(kBoxHalfExtent),
                body->IsSleeping() ? sleepColor : awakeColor);
        }

        const glm::mat4 viewProj = m_camera.GetProjection() * m_camera.GetView();
        m_renderer.Draw(viewProj, m_instanceData);
        m_window->SwapBuffers();
    }

    // Release scene rendering resources before the engine shuts down.
    void Shutdown() {
        m_renderer.Shutdown();
    }

private:
    // Pack one model transform and color into the instance buffer.
    void PushInstance(
        const glm::vec3& position, const glm::quat& orientation,
        const glm::vec3& scale, const glm::vec3& color) {
        glm::mat4 model = glm::translate(glm::mat4(1.0f), position);
        model *= glm::mat4_cast(orientation);
        model = glm::scale(model, scale);
        for (int column = 0; column < 4; ++column) {
            for (int row = 0; row < 4; ++row) {
                m_instanceData.push_back(model[column][row]);
            }
        }
        m_instanceData.insert(m_instanceData.end(), {color.x, color.y, color.z});
    }

    // Update the free-fly camera independently of the physics fixed step.
    void HandleCamera(float dt, const Runtime::Core::Input& input) {
        if (m_cursorCaptured && m_skipMouseLookFrames == 0) {
            m_yaw += static_cast<float>(input.GetMouseDeltaX()) * 0.0025f;
            m_pitch -= static_cast<float>(input.GetMouseDeltaY()) * 0.0025f;
        }
        m_pitch = glm::clamp(m_pitch, -1.55f, 1.55f);

        const float scroll = input.GetScrollDelta();
        if (scroll != 0.0f) {
            m_cameraSpeed = glm::clamp(m_cameraSpeed * (scroll > 0.0f ? 1.2f : 1.0f / 1.2f), 0.5f, 80.0f);
        }

        const glm::vec3 forward(
            std::cos(m_pitch) * std::sin(m_yaw),
            std::sin(m_pitch),
            -std::cos(m_pitch) * std::cos(m_yaw));
        const glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)));

        // S is taken by the sleep toggle, so backward flight is on X.
        glm::vec3 move(0.0f);
        if (input.IsKeyDown(Runtime::Core::Key_W)) move += forward;
        if (input.IsKeyDown(Runtime::Core::Key_X)) move -= forward;
        if (input.IsKeyDown(Runtime::Core::Key_A)) move -= right;
        if (input.IsKeyDown(Runtime::Core::Key_D)) move += right;
        if (input.IsKeyDown(Runtime::Core::Key_E)) move += glm::vec3(0.0f, 1.0f, 0.0f);
        if (input.IsKeyDown(Runtime::Core::Key_Q)) move -= glm::vec3(0.0f, 1.0f, 0.0f);
        if (glm::dot(move, move) > 0.0f) {
            m_cameraPosition += glm::normalize(move) * m_cameraSpeed * dt;
        }

        m_camera.SetPosition(m_cameraPosition);
        m_camera.SetTarget(m_cameraPosition + forward);

        // Consume the per-frame mouse delta so it does not accumulate.
        Runtime::Core::Input::Get().UpdateMouseDelta();
        if (m_skipMouseLookFrames > 0) --m_skipMouseLookFrames;
    }

    // Add a static slab whose top surface is y=0.
    void AddGround() {
        Runtime::Physics::RigidBodyDesc ground;
        ground.position = glm::vec3(0.0f, -0.5f, 0.0f);
        ground.isStatic = true;
        ground.useGravity = false;
        m_groundId = m_world->CreateRigidBody(ground);
        Runtime::Physics::ColliderDesc collider;
        collider.shape = std::make_shared<Runtime::Physics::BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
        collider.material.restitution = 0.1f;
        collider.material.dynamicFriction = 0.6f;
        collider.material.staticFriction = 0.7f;
        m_world->AttachCollider(m_groundId, collider);
    }

    // Drop a cube; the cradle cap is a diamond supported by two upper edges.
    void TrySpawn() {
        const std::size_t sceneLimit = m_mode == SceneMode::EdgeCradle ? 7 : 5;
        if (m_mode != SceneMode::FieldRain && m_bodies.size() >= sceneLimit) return;
        if (m_bodies.size() >= m_maxBodies) return;

        glm::vec3 position(0.0f, kSpawnHeight, 0.0f);
        if (m_mode == SceneMode::FieldRain) {
            position.x = m_positionDist(m_rng);
            position.z = m_positionDist(m_rng);
        } else {
            float towerTop = 0.0f;
            for (const uint32_t bodyId : m_bodies) {
                const auto* existing = m_world->GetRigidBody(bodyId);
                towerTop = std::max(towerTop, existing->Position().y + kBoxHalfExtent);
            }
            position.y = towerTop + kBoxHalfExtent + 0.05f;
        }

        Runtime::Physics::RigidBodyDesc body;
        body.position = position;
        if (m_mode == SceneMode::EdgeCradle) {
            body.position.y = 3.7f;
            body.orientation = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f));
        }
        body.mass = 1.0f;
        body.inertiaTensorDiagonal = glm::vec3(0.135f);
        const uint32_t bodyId = m_world->CreateRigidBody(body);
        Runtime::Physics::ColliderDesc collider;
        collider.shape = m_boxShape;
        collider.material.restitution = 0.05f;
        collider.material.dynamicFriction = m_mode == SceneMode::FieldRain ? 0.55f : 1.0f;
        collider.material.staticFriction = m_mode == SceneMode::FieldRain ? 0.65f : 1.0f;
        if (m_world->AttachCollider(bodyId, collider)) {
            m_bodies.push_back(bodyId);
        } else {
            m_world->DestroyRigidBody(bodyId);
        }
    }

    // Remove bodies that have left the visible world below the slab.
    void RecycleFallen() {
        for (std::size_t index = 0; index < m_bodies.size();) {
            const auto* body = m_world->GetRigidBody(m_bodies[index]);
            if (body && body->Position().y < kKillY) {
                m_world->DestroyRigidBody(m_bodies[index]);
                m_bodies.erase(m_bodies.begin() + static_cast<std::ptrdiff_t>(index));
            } else {
                ++index;
            }
        }
    }

    // Rebuild the world without changing solver tuning between scene modes.
    void BuildScene() {
        // PhysicsWorld is non-copyable/non-movable: rebuild the instance.
        m_world = std::make_unique<Runtime::Physics::PhysicsWorld>();
        m_world->SetFixedTimeStep(1.0f / 120.0f);
        m_world->SetGravity(glm::vec3(0.0f, -10.5f, 0.0f));
        // Field rain drops boxes from 14 m: at 120 Hz that is ~13 cm of travel
        // per step, far beyond the speculative band, so a discrete step tunnels
        // deep enough for the penetration axis to flip and shove the box into
        // the ground. CCD keeps the drop on the contact surface.
        m_world->SetContinuousCollisionEnabled(true);
        m_world->SetSolverIterations(10);
        m_world->SetSleepEnabled(m_sleepEnabled);
        m_bodies.clear();
        m_rng.seed(kSeed);
        m_spawnAccumulator = 0.0f;
        m_nextStateLogStep = 120;
        AddGround();
        if (m_mode == SceneMode::EdgeCradle) {
            for (int level = 0; level < 3; ++level) {
                for (float x : {-0.5f, 0.5f}) {
                    Runtime::Physics::RigidBodyDesc body;
                    body.position = {x, kBoxHalfExtent + 0.9f * level + 0.02f, 0.0f};
                    body.inertiaTensorDiagonal = glm::vec3(0.135f);
                    const uint32_t id = m_world->CreateRigidBody(body);
                    Runtime::Physics::ColliderDesc collider;
                    collider.shape = m_boxShape;
                    collider.material.restitution = 0.05f;
                    collider.material.dynamicFriction = 1.0f;
                    collider.material.staticFriction = 1.0f;
                    if (!m_world->AttachCollider(id, collider))
                        throw std::runtime_error("Failed to attach edge cradle collider");
                    m_bodies.push_back(id);
                }
            }
        }
        if (m_mode == SceneMode::FourSupports) {
            constexpr float supportOffset = kBoxHalfExtent + 0.01f;
            for (const float x : {-supportOffset, supportOffset}) {
                for (const float z : {-supportOffset, supportOffset}) {
                    Runtime::Physics::RigidBodyDesc body;
                    body.position = glm::vec3(x, kBoxHalfExtent + 0.05f, z);
                    body.mass = 1.0f;
                    body.inertiaTensorDiagonal = glm::vec3(0.135f);
                    const uint32_t bodyId = m_world->CreateRigidBody(body);
                    Runtime::Physics::ColliderDesc collider;
                    collider.shape = m_boxShape;
                    collider.material.restitution = 0.05f;
                    collider.material.dynamicFriction = 1.0f;
                    collider.material.staticFriction = 1.0f;
                    m_world->AttachCollider(bodyId, collider);
                    m_bodies.push_back(bodyId);
                }
            }
        }
    }

    // Emit one compact physics summary per simulated second when requested.
    void LogBoxStates() {
        const auto step = m_world->FixedStepId();
        if (!m_stateLogEnabled || step < m_nextStateLogStep) return;
        m_nextStateLogStep = step + 120;
        const auto& stats = m_world->LastStepStats();
        std::cout << "[BoxStack] step=" << step << " sleeping=" << m_sleepingCount
            << "/" << m_bodies.size() << " physicsMs=" << stats.totalMilliseconds
            << " toi=" << stats.ccdToiHitCount << '\n';
    }

    // Average presentation FPS over a half-second window.
    void UpdateFps(float dt) {
        m_fpsAccumulator += dt;
        ++m_frameCount;
        if (m_fpsAccumulator < 0.5f) return;
        m_fps = static_cast<float>(m_frameCount) / m_fpsAccumulator;
        m_fpsAccumulator = 0.0f;
        m_frameCount = 0;
    }

    // Publish scene, sleep state, and controls without dumping individual boxes.
    void UpdateTitle() {
        if (!m_window->GetNativeWindow()) return;
        const std::string title = std::string("Box Stack Sample | ") + SceneName(m_mode) +
            " | Bodies=" + std::to_string(m_bodies.size()) +
            " | Sleeping=" + std::to_string(m_sleepingCount) +
            " | FPS=" + std::to_string(static_cast<int>(std::round(m_fps))) +
            " | Sleep=" + (m_sleepEnabled ? "ON" : "OFF") +
            " | " + (m_spawnPaused ? "PAUSED" : "raining") +
            " | PhysicsMs=" + std::to_string(m_world->LastStepStats().totalMilliseconds) +
            " | W/X/A/D/QE fly, mouse look (hold Alt for cursor), 1/2/3/4 mode, +/- rate, Space pause, S sleep, L log, R reset";
        glfwSetWindowTitle(m_window->GetNativeWindow(), title.c_str());
    }

    Runtime::Platform::Window* m_window = nullptr;
    InstancedBoxRenderer m_renderer;
    std::unique_ptr<Runtime::Physics::PhysicsWorld> m_world =
        std::make_unique<Runtime::Physics::PhysicsWorld>();
    Runtime::Graphics::Camera m_camera;
    std::shared_ptr<Runtime::Physics::CollisionShape> m_boxShape;
    std::vector<uint32_t> m_bodies;
    std::vector<float> m_instanceData;
    uint32_t m_groundId = 0;
    SceneMode m_mode = SceneMode::FieldRain;
    std::size_t m_maxBodies = 300;
    std::size_t m_sleepingCount = 0;
    bool m_sleepEnabled = true;
    bool m_spawnPaused = false;
    bool m_stateLogEnabled = false;
    bool m_cursorCaptured = true;
    int m_skipMouseLookFrames = 0;
    float m_spawnRate = 24.0f;
    float m_spawnAccumulator = 0.0f;
    uint64_t m_nextStateLogStep = 120;
    std::mt19937 m_rng{kSeed};
    std::uniform_real_distribution<float> m_positionDist{-3.5f, 3.5f};
    // Free-fly camera state.
    glm::vec3 m_cameraPosition{12.0f, 9.0f, 16.0f};
    float m_yaw = 0.0f;
    float m_pitch = 0.0f;
    float m_cameraSpeed = 10.0f;
    float m_fps = 0.0f;
    float m_fpsAccumulator = 0.0f;
    uint32_t m_frameCount = 0;
};

} // namespace

// Launch the visual sample; --edge-cradle demonstrates persistent non-face support.
int main(int argc, char** argv) {
    SceneMode mode = SceneMode::FieldRain;
    bool noSleep = false;
    for (int index = 1; index < argc; ++index) {
        if (std::string(argv[index]) == "--four-supports") mode = SceneMode::FourSupports;
        if (std::string(argv[index]) == "--tower") mode = SceneMode::TowerRain;
        if (std::string(argv[index]) == "--edge-cradle") mode = SceneMode::EdgeCradle;
        noSleep = noSleep || std::string(argv[index]) == "--no-sleep";
    }
    auto& engine = Runtime::Core::Engine::GetEngine();
    if (!engine.Initialize(1280, 720, "Box Stack Sample")) return 1;
    BoxStackSample sample;
    if (!sample.Initialize(engine, mode, noSleep)) { engine.Shutdown(); return 2; }
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
