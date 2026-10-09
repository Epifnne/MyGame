#include "physics.vert.h"
#include "physics.frag.h"

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
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace {

enum class ShapeType : std::size_t { Tetrahedron, Cube, Octahedron, Dodecahedron, Icosahedron, Count };
constexpr std::size_t kShapeCount = static_cast<std::size_t>(ShapeType::Count);
constexpr float kRadius = 0.48f;

struct PolyhedronData {
    std::vector<glm::vec3> hullVertices;
    std::vector<float> renderVertices;
    float inertiaPerMass = 0.0f;
};

// Emit an outward triangle with a flat lighting normal.
void AppendTriangle(std::vector<float>& vertices, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
    const glm::vec3 normal = glm::normalize(glm::cross(b - a, c - a));
    for (const glm::vec3& point : {a, b, c}) {
        vertices.insert(vertices.end(), {point.x, point.y, point.z, normal.x, normal.y, normal.z});
    }
}

// Enumerate the five Platonic vertex sets at a common circumradius.
PolyhedronData BuildRegularPolyhedron(ShapeType type) {
    PolyhedronData data;
    const float phi = 0.5f * (1.0f + std::sqrt(5.0f));
    if (type == ShapeType::Tetrahedron) {
        data.hullVertices = {{1, 1, 1}, {1, -1, -1}, {-1, 1, -1}, {-1, -1, 1}};
    } else if (type == ShapeType::Cube || type == ShapeType::Dodecahedron) {
        for (float x : {-1.0f, 1.0f})
            for (float y : {-1.0f, 1.0f})
                for (float z : {-1.0f, 1.0f}) data.hullVertices.emplace_back(x, y, z);
    }
    if (type == ShapeType::Octahedron) {
        data.hullVertices = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    } else if (type == ShapeType::Dodecahedron || type == ShapeType::Icosahedron) {
        const float a = type == ShapeType::Dodecahedron ? 1.0f / phi : 1.0f;
        for (float s : {-1.0f, 1.0f}) {
            for (float t : {-1.0f, 1.0f}) {
                data.hullVertices.emplace_back(0, s * a, t * phi);
                data.hullVertices.emplace_back(t * phi, 0, s * a);
                data.hullVertices.emplace_back(s * a, t * phi, 0);
            }
        }
    }
    for (auto& point : data.hullVertices) point *= kRadius / glm::length(point);

    std::vector<std::vector<std::size_t>> faces;
    std::vector<std::array<std::size_t, 2>> edges;
    double volume = 0.0, radialMoment = 0.0;
    const auto& points = data.hullVertices;
    for (std::size_t i = 0; i < points.size(); ++i) {
        for (std::size_t j = i + 1; j < points.size(); ++j) {
            for (std::size_t k = j + 1; k < points.size(); ++k) {
                glm::vec3 normal = glm::cross(points[j] - points[i], points[k] - points[i]);
                if (glm::dot(normal, normal) < 1e-10f) continue;
                normal = glm::normalize(normal);
                if (glm::dot(normal, points[i]) < 0) normal = -normal;
                const float distance = glm::dot(normal, points[i]);
                std::vector<std::size_t> face;
                bool supporting = true;
                for (std::size_t p = 0; p < points.size(); ++p) {
                    const float delta = glm::dot(normal, points[p]) - distance;
                    if (delta > 1e-5f) { supporting = false; break; }
                    if (std::abs(delta) <= 1e-5f) face.push_back(p);
                }
                if (!supporting || std::find(faces.begin(), faces.end(), face) != faces.end()) continue;
                faces.push_back(face);
                glm::vec3 center(0);
                for (std::size_t p : face) center += points[p];
                center /= static_cast<float>(face.size());
                const glm::vec3 u = glm::normalize(points[face.front()] - center);
                const glm::vec3 v = glm::cross(normal, u);
                std::sort(face.begin(), face.end(), [&](std::size_t a, std::size_t b) {
                    const glm::vec3 pa = points[a] - center, pb = points[b] - center;
                    return std::atan2(glm::dot(pa, v), glm::dot(pa, u)) <
                        std::atan2(glm::dot(pb, v), glm::dot(pb, u));
                });
                const std::array<std::size_t, kShapeCount> faceSizes{3, 4, 3, 5, 3};
                if (face.size() != faceSizes[static_cast<std::size_t>(type)])
                    throw std::runtime_error("Regular polyhedron has an invalid face");
                for (std::size_t p = 0; p < face.size(); ++p) {
                    const std::size_t a = face[p], b = face[(p + 1) % face.size()];
                    const std::array<std::size_t, 2> edge{std::min(a, b), std::max(a, b)};
                    if (std::find(edges.begin(), edges.end(), edge) == edges.end()) edges.push_back(edge);
                }
                for (std::size_t p = 1; p + 1 < face.size(); ++p) {
                    const glm::vec3 a = points[face[0]], b = points[face[p]], c = points[face[p + 1]];
                    AppendTriangle(data.renderVertices, a, b, c);
                    const double tetraVolume = glm::dot(a, glm::cross(b, c)) / 6.0;
                    volume += tetraVolume;
                    radialMoment += tetraVolume * (glm::dot(a, a) + glm::dot(b, b) + glm::dot(c, c) +
                        glm::dot(a, b) + glm::dot(a, c) + glm::dot(b, c)) / 10.0;
                }
            }
        }
    }
    // Platonic symmetry gives Ixx=Iyy=Izz=(2/3)*m*E[|r|^2].
    data.inertiaPerMass = static_cast<float>(2.0 * radialMoment / (3.0 * volume));
    const std::array<std::size_t, kShapeCount> expectedFaces{4, 6, 8, 12, 20};
    const std::array<std::size_t, kShapeCount> expectedVertices{4, 8, 6, 20, 12};
    const std::array<std::size_t, kShapeCount> expectedEdges{6, 12, 12, 30, 30};
    if (faces.size() != expectedFaces[static_cast<std::size_t>(type)] ||
        points.size() != expectedVertices[static_cast<std::size_t>(type)] ||
        edges.size() != expectedEdges[static_cast<std::size_t>(type)] ||
        volume <= 0 || !std::isfinite(data.inertiaPerMass) || data.inertiaPerMass <= 0)
        throw std::runtime_error("Invalid regular polyhedron geometry or inertia");
    const float edgeLength = glm::length(points[edges.front()[0]] - points[edges.front()[1]]);
    for (const auto& edge : edges)
        if (std::abs(glm::length(points[edge[0]] - points[edge[1]]) - edgeLength) > 1e-5f)
            throw std::runtime_error("Polyhedron edges are not regular");
    return data;
}

// Label the true regular solids rather than arbitrary bipyramids.
const char* ShapeName(ShapeType type) {
    switch (type) {
    case ShapeType::Tetrahedron: return "tetrahedron";
    case ShapeType::Cube: return "cube hull";
    case ShapeType::Octahedron: return "octahedron";
    case ShapeType::Dodecahedron: return "dodecahedron";
    case ShapeType::Icosahedron: return "icosahedron";
    case ShapeType::Count: return "mixed Platonic hulls";
    }
    throw std::logic_error("Invalid regular polyhedron type");
}

class PhysicsCollisionSample {
public:
    // Share identical geometry, materials and stepping between GUI and headless runs.
    bool InitializePhysics(bool ccd = false, bool sleep = true) {
        m_ccdEnabled = ccd;
        m_world.SetFixedTimeStep(1.0f / 120.0f);
        m_world.SetGravity(glm::vec3(0.0f, -10.5f, 0.0f));
        m_world.SetContinuousCollisionEnabled(ccd);
        m_world.SetSleepEnabled(sleep);
        m_world.SetSolverIterations(6);
        for (std::size_t index = 0; index < kShapeCount; ++index) {
            m_geometry[index] = BuildRegularPolyhedron(static_cast<ShapeType>(index));
            m_shapes[index] = std::make_shared<Runtime::Physics::ConvexHullShape>(m_geometry[index].hullVertices);
        }
        Runtime::Physics::RigidBodyDesc ground;
        ground.position = glm::vec3(0.0f, -0.5f, 0.0f);
        ground.isStatic = true;
        ground.useGravity = false;
        m_groundId = m_world.CreateRigidBody(ground);
        Runtime::Physics::ColliderDesc collider;
        collider.shape = std::make_shared<Runtime::Physics::BoxShape>(glm::vec3(5.0f, 0.5f, 5.0f));
        collider.material.dynamicFriction = 0.6f;
        collider.material.staticFriction = 0.7f;
        if (m_world.AttachCollider(m_groundId, collider)) return true;
        std::cerr << "PhysicsCollisionSample: could not attach ground collider\n";
        return false;
    }

    // Create matching render meshes without changing collision geometry.
    bool Initialize(Runtime::Core::Engine& engine) {
        m_renderer = engine.GetRenderer();
        m_window = engine.GetWindow();
        if (!m_renderer || !m_window) return false;

        if (!m_renderer->LoadShaderFromSource(
            "physics_sample", Sample::kPhysicsVertexShader, Sample::kPhysicsFragmentShader)) return false;
        m_material.SetShader(m_renderer->GetShader("physics_sample"));

        if (!InitializePhysics()) return false;
        for (std::size_t index = 0; index < kShapeCount; ++index) {
            m_meshes[index] = m_meshManager.CreateFromVertices(
                Runtime::Graphics::MeshHandle{3000u + index}, m_geometry[index].renderVertices.data(),
                m_geometry[index].renderVertices.size());
            if (!m_meshes[index]) return false;
        }

        m_camera.SetPosition(glm::vec3(8.0f, 7.0f, 11.0f));
        m_camera.SetTarget(glm::vec3(0.0f, 2.5f, 0.0f));
        const float aspect = static_cast<float>(m_window->GetWidth()) / static_cast<float>(std::max(1, m_window->GetHeight()));
        m_camera.SetPerspective(glm::radians(48.0f), aspect, 0.1f, 100.0f);
        return true;
    }

    // Select individual hulls or a mixed rain; no FPS-dependent spawning gate.
    void Update(float dt, const Runtime::Core::Input& input) {
        ShapeType requested = m_shapeType;
        if (input.IsKeyPressed(Runtime::Core::Key_1)) requested = ShapeType::Tetrahedron;
        if (input.IsKeyPressed(Runtime::Core::Key_2)) requested = ShapeType::Cube;
        if (input.IsKeyPressed(Runtime::Core::Key_3)) requested = ShapeType::Octahedron;
        if (input.IsKeyPressed(Runtime::Core::Key_4)) requested = ShapeType::Dodecahedron;
        if (input.IsKeyPressed(Runtime::Core::Key_5)) requested = ShapeType::Icosahedron;
        if (input.IsKeyPressed(Runtime::Core::Key_0)) requested = ShapeType::Count;
        if (requested != m_shapeType) { m_shapeType = requested; Reset(); }
        if (input.IsKeyPressed(Runtime::Core::Key_Equal)) m_targetCount = std::min<std::size_t>(m_targetCount + 32, 1024);
        if (input.IsKeyPressed(Runtime::Core::Key_Minus)) m_targetCount = m_targetCount > 32 ? m_targetCount - 32 : 0;
        if (input.IsKeyPressed(Runtime::Core::Key_R)) Reset();
        if (input.IsKeyPressed(Runtime::Core::Key_C)) {
            m_ccdEnabled = !m_ccdEnabled;
            m_world.SetContinuousCollisionEnabled(m_ccdEnabled);
        }
        if (input.IsKeyPressed(Runtime::Core::Key_S)) m_world.SetSleepEnabled(!m_world.SleepEnabled());
        if (input.IsKeyPressed(Runtime::Core::Key_Space)) m_paused = !m_paused;

        while (m_bodies.size() > m_targetCount) {
            m_world.DestroyRigidBody(m_bodies.back().id);
            m_bodies.pop_back();
        }

        UpdateFps(dt);
        if (!m_paused) Advance(dt);
        UpdateTitle();
    }

    // Render each body's own hull mesh, including cubes that remain ConvexHullShape.
    void Render() {
        m_renderer->Clear();
        glm::mat4 groundModel = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -0.5f, 0.0f));
        const float cubeHalf = kRadius / std::sqrt(3.0f);
        groundModel = glm::scale(groundModel, glm::vec3(5.0f / cubeHalf, 0.5f / cubeHalf, 5.0f / cubeHalf));
        m_renderer->Submit(*m_meshes[static_cast<std::size_t>(ShapeType::Cube)], m_material, groundModel);

        for (const auto& entry : m_bodies) {
            const auto* body = m_world.GetRigidBody(entry.id);
            if (!body) continue;
            glm::mat4 model = glm::translate(glm::mat4(1.0f), body->Position());
            model *= glm::mat4_cast(body->Orientation());
            m_renderer->Submit(*m_meshes[static_cast<std::size_t>(entry.type)], m_material, model);
        }
        m_renderer->Flush(m_camera);
        m_window->SwapBuffers();
    }

    // Release meshes while the GL context is still alive.
    void Shutdown() {
        for (auto& mesh : m_meshes) {
            if (mesh) mesh->Destroy();
            mesh.reset();
        }
        m_meshManager.Clear();
    }

    // Run the same rain for 60 seconds and fail explicitly on path or quality violations.
    int Verify() {
        std::size_t gjk = 0, epa = 0, sat = 0, failures = 0;
        uint64_t budgetHits = 0;
        double physicsMs = 0;
        float maxGroundPen = 0, maxContactPen = 0, settledSpeed = 0, finalHeight = 0;
        std::vector<glm::vec3> reference;
        float drift = 0;
        float retainedSpeed = 0, retainedDrift = 0, retainedSpin = 0;
        std::size_t escaped = 0, sunk = 0;
        std::array<std::size_t, kShapeCount> population{};
        for (int step = 0; step < 7200; ++step) {
            Advance(1.0f / 120.0f);
            const auto& stats = m_world.LastStepStats();
            gjk += stats.gjkCallCount;
            epa += stats.epaCallCount;
            sat += stats.satCallCount;
            failures += stats.gjkFailureCount + stats.epaFailureCount;
            budgetHits += stats.ccdBudgetExhaustionCount;
            physicsMs += stats.totalMilliseconds;
            for (const auto& contact : m_world.Contacts())
                for (std::size_t p = 0; p < contact.pointCount; ++p)
                    maxContactPen = std::max(maxContactPen, contact.Point(p).penetration);
            for (std::size_t i = 0; i < m_bodies.size(); ++i) {
                const auto& entry = m_bodies[i];
                const auto& body = *m_world.GetRigidBody(entry.id);
                const auto bounds = m_shapes[static_cast<std::size_t>(entry.type)]->ComputeAABB(
                    {body.Position(), body.Orientation()});
                if (!std::isfinite(body.Position().x) || !std::isfinite(body.Position().y) ||
                    !std::isfinite(body.Position().z) || !std::isfinite(glm::length(body.LinearVelocity())) ||
                    !std::isfinite(glm::length(body.AngularVelocity()))) {
                    std::cerr << "Verification failed: non-finite state, body=" << entry.id << '\n';
                    return 3;
                }
                const bool aboveFloor = bounds.max.x >= -5.0f && bounds.min.x <= 5.0f &&
                    bounds.max.z >= -5.0f && bounds.min.z <= 5.0f;
                if (aboveFloor) maxGroundPen = std::max(maxGroundPen, -bounds.min.y);
                if (step == 5999) reference.push_back(body.Position());
                if (step >= 6000) {
                    if (reference.size() != m_bodies.size()) {
                        std::cerr << "Verification failed: population not reached before drift window\n";
                        return 3;
                    }
                    settledSpeed = std::max(settledSpeed, glm::length(body.LinearVelocity()));
                    drift = std::max(drift, glm::length(body.Position() - reference[i]));
                    if (aboveFloor && bounds.max.y >= 0.0f) {
                        retainedSpeed = std::max(retainedSpeed, glm::length(body.LinearVelocity()));
                        retainedSpin = std::max(retainedSpin, glm::length(body.AngularVelocity()));
                        retainedDrift = std::max(retainedDrift, glm::length(body.Position() - reference[i]));
                    }
                }
                if (step == 7199) {
                    finalHeight = std::max(finalHeight, bounds.max.y);
                    ++population[static_cast<std::size_t>(entry.type)];
                    if (!aboveFloor) ++escaped;
                    else if (bounds.max.y < -0.04f) ++sunk;
                    if (bounds.max.y < 0.0f)
                        std::cout << "[fallen] body=" << entry.id << " shape=" << ShapeName(entry.type)
                                  << " x/y/z=" << body.Position().x << "/" << body.Position().y << "/"
                                  << body.Position().z << " overFloor=" << aboveFloor << '\n';
                }
            }
        }
        std::cout << "[convex-rain] seed=1337 ccd=" << m_ccdEnabled << " sleep=" << m_world.SleepEnabled()
                  << " bodies=" << m_bodies.size() << " sleeping=" << m_world.LastStepStats().sleepingBodyCount
                  << " gjk=" << gjk << " epa=" << epa << " sat=" << sat << " failures=" << failures
                  << " budgetHits=" << budgetHits << " avgPhysicsMs=" << physicsMs / 7200.0
                  << " maxGroundPen=" << maxGroundPen << " maxContactPen=" << maxContactPen
                  << " last10sSpeed=" << settledSpeed << " last10sDrift=" << drift
                  << " retainedSpeed=" << retainedSpeed << " retainedSpin=" << retainedSpin
                  << " retainedDrift=" << retainedDrift
                  << " finalHeight=" << finalHeight << " escaped=" << escaped << " sunk=" << sunk << '\n';
        for (std::size_t i = 0; i < kShapeCount; ++i)
            std::cout << "[population] " << ShapeName(static_cast<ShapeType>(i)) << "=" << population[i] << '\n';
        const bool passed = m_bodies.size() == m_targetCount && gjk > 0 && epa > 0 && sat == 0 &&
            std::all_of(population.begin(), population.end(), [](std::size_t count) { return count > 0; }) &&
            failures == 0 && budgetHits == 0 && escaped == 0 && sunk == 0 &&
            maxGroundPen < 0.04f && maxContactPen < 0.08f &&
            drift < 0.02f && settledSpeed < 0.05f &&
            (!m_world.SleepEnabled() || m_world.LastStepStats().sleepingBodyCount == m_bodies.size());
        if (!passed) std::cerr << "Convex rain verification FAILED; see the measured limits above\n";
        return passed ? 0 : 3;
    }

private:
    struct BodyEntry {
        uint32_t id;
        ShapeType type;
    };

    // Sample a reproducible uniform rotation; all solids have uniform-density isotropic inertia.
    bool SpawnBody() {
        const ShapeType type = m_shapeType == ShapeType::Count ?
            static_cast<ShapeType>(std::uniform_int_distribution<int>(0, static_cast<int>(kShapeCount) - 1)(m_random)) :
            m_shapeType;
        const std::size_t shapeIndex = static_cast<std::size_t>(type);
        std::uniform_real_distribution<float> unit(0.0f, 1.0f), offset(-0.6f, 0.6f);
        const float u = unit(m_random), a = 6.283185307f * unit(m_random), b = 6.283185307f * unit(m_random);
        Runtime::Physics::RigidBodyDesc body;
        body.position = {offset(m_random), 6.0f, offset(m_random)};
        body.orientation = glm::quat(std::sqrt(u) * std::cos(b),
            std::sqrt(1 - u) * std::sin(a), std::sqrt(1 - u) * std::cos(a), std::sqrt(u) * std::sin(b));
        body.linearVelocity = {0.0f, -3.0f, 0.0f};
        body.mass = 1.0f;
        body.inertiaTensorDiagonal = glm::vec3(m_geometry[shapeIndex].inertiaPerMass * body.mass);
        const auto spawnBounds = m_shapes[shapeIndex]->ComputeAABB({body.position, body.orientation});
        for (const auto& entry : m_bodies) {
            const auto& existing = *m_world.GetRigidBody(entry.id);
            if (spawnBounds.Intersects(m_shapes[static_cast<std::size_t>(entry.type)]->ComputeAABB(
                {existing.Position(), existing.Orientation()}))) {
                if (!m_spawnBlocked) std::cout << "[convex-rain] spawn blocked by occupied volume; retrying\n";
                m_spawnBlocked = true;
                return false;
            }
        }
        m_spawnBlocked = false;
        const uint32_t bodyId = m_world.CreateRigidBody(body);
        Runtime::Physics::ColliderDesc collider;
        collider.shape = m_shapes[shapeIndex];
        collider.material.restitution = 0.1f;
        collider.material.dynamicFriction = 0.55f;
        collider.material.staticFriction = 0.65f;
        if (!m_world.AttachCollider(bodyId, collider)) {
            m_world.DestroyRigidBody(bodyId);
            throw std::runtime_error("PhysicsCollisionSample: could not attach convex hull collider");
        }
        m_bodies.push_back({bodyId, type});
        return true;
    }

    // Advance physics and counters independently of rendering or FPS.
    void Advance(float dt) {
        m_spawnAccumulator += dt * 4.0f;
        while (m_spawnAccumulator >= 1.0f && m_bodies.size() < m_targetCount) {
            if (!SpawnBody()) { m_spawnAccumulator = 1.0f; break; }
            m_spawnAccumulator -= 1.0f;
        }
        if (m_bodies.size() == m_targetCount) m_spawnAccumulator = 0.0f;
        m_world.Step(dt);
        const auto& stats = m_world.LastStepStats();
        m_gjk += stats.gjkCallCount;
        m_epa += stats.epaCallCount;
        m_sat += stats.satCallCount;
        m_queryFailures += stats.gjkFailureCount + stats.epaFailureCount;
    }

    // Replaying a mode starts the same random sequence and clears diagnostic totals.
    void Reset() {
        for (const auto& entry : m_bodies) m_world.DestroyRigidBody(entry.id);
        m_bodies.clear();
        m_spawnAccumulator = 0.0f;
        m_random.seed(1337);
        m_spawnBlocked = false;
        m_gjk = m_epa = m_sat = m_queryFailures = 0;
    }

    // Report population and algorithm failures without stopping a slow simulation.
    void UpdateFps(float dt) {
        m_fpsAccumulator += dt;
        ++m_frameCount;
        if (m_fpsAccumulator < 0.5f) return;
        m_fps = static_cast<float>(m_frameCount) / m_fpsAccumulator;
        m_fpsAccumulator = 0.0f;
        m_frameCount = 0;
        std::cout << "[PhysicsCollisionSample] shape=" << ShapeName(m_shapeType)
                  << " bodies=" << m_bodies.size() << " contacts=" << m_world.Contacts().size()
                  << " fps=" << static_cast<int>(std::round(m_fps))
                  << " sleeping=" << m_world.LastStepStats().sleepingBodyCount
                  << " gjk=" << m_gjk << " epa=" << m_epa << " sat=" << m_sat
                  << " failures=" << m_queryFailures << '\n';
    }

    // Keep collision-path counters visible so a stable SAT fallback cannot masquerade as GJK/EPA.
    void UpdateTitle() {
        if (!m_window->GetNativeWindow()) return;
        const std::string title = std::string("Physics Collision Sample | ") + ShapeName(m_shapeType) +
            " | Bodies=" + std::to_string(m_bodies.size()) + "/" + std::to_string(m_targetCount) +
            " | Contacts=" + std::to_string(m_world.Contacts().size()) +
            " | FPS=" + std::to_string(static_cast<int>(std::round(m_fps))) +
            " | Sleeping=" + std::to_string(m_world.LastStepStats().sleepingBodyCount) +
            " | GJK/EPA/SAT=" + std::to_string(m_gjk) + "/" + std::to_string(m_epa) + "/" + std::to_string(m_sat) +
            " | Failures=" + std::to_string(m_queryFailures) +
            " | CCD=" + (m_ccdEnabled ? "ON" : "OFF") +
            " | Sleep=" + (m_world.SleepEnabled() ? "ON" : "OFF") +
            (m_paused ? " | PAUSED" : "") +
            " | 0 mixed, 1-5 solid, +/- count, C CCD, S sleep, Space pause, R reset";
        glfwSetWindowTitle(m_window->GetNativeWindow(), title.c_str());
    }

    Runtime::Graphics::Renderer* m_renderer = nullptr;
    Runtime::Platform::Window* m_window = nullptr;
    Runtime::Physics::PhysicsWorld m_world;
    Runtime::Graphics::MeshManager m_meshManager;
    Runtime::Graphics::Material m_material;
    Runtime::Graphics::Camera m_camera;
    std::array<std::shared_ptr<Runtime::Graphics::Mesh>, static_cast<std::size_t>(ShapeType::Count)> m_meshes;
    std::array<std::shared_ptr<Runtime::Physics::CollisionShape>, static_cast<std::size_t>(ShapeType::Count)> m_shapes;
    std::array<PolyhedronData, kShapeCount> m_geometry;
    std::vector<BodyEntry> m_bodies;
    ShapeType m_shapeType = ShapeType::Count;
    std::size_t m_targetCount = 96;
    uint32_t m_groundId = 0;
    std::mt19937 m_random{1337};
    std::size_t m_gjk = 0, m_epa = 0, m_sat = 0, m_queryFailures = 0;
    float m_spawnAccumulator = 0.0f;
    bool m_ccdEnabled = false;
    bool m_spawnBlocked = false;
    bool m_paused = false;
    float m_fps = 0.0f;
    float m_fpsAccumulator = 0.0f;
    uint32_t m_frameCount = 0;
};

} // namespace

// Headless validation uses the exact sample world, not a separate benchmark scene.
int main(int argc, char** argv) {
    bool verify = false, ccd = false, sleep = true;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--verify") verify = true;
        else if (arg == "--ccd") ccd = true;
        else if (arg == "--no-ccd") ccd = false;
        else if (arg == "--no-sleep") sleep = false;
        else {
            std::cerr << "Unknown argument: " << arg << "\nUsage: PhysicsCollisionSample [--verify [--ccd] [--no-ccd] [--no-sleep]]\n";
            return 1;
        }
    }
    if (!verify && (ccd || !sleep)) {
        std::cerr << "--ccd and --no-sleep require --verify; use C/S in the interactive sample\n";
        return 1;
    }
    PhysicsCollisionSample sample;
    if (verify) return sample.InitializePhysics(ccd, sleep) ? sample.Verify() : 2;
    auto& engine = Runtime::Core::Engine::GetEngine();
    if (!engine.Initialize(1280, 720, "Physics Collision Sample")) return 1;
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
