#include <Physics/Collider.h>
#include <Physics/CollisionShape.h>
#include <Physics/PhysicsWorld.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace {

struct Options {
    std::string scenario = "boxfield";
    std::size_t bodyCount = 100;
    std::size_t frameCount = 240;
    std::size_t warmupFrames = 30;
    uint32_t seed = 1337;
    std::string output = "physics_benchmark.json";
    // Phase 6: physics worker count (0 = hardware concurrency, 1 = fully
    // serial deterministic fallback) and the parallel narrow-phase toggle.
    // Phase 8 adds the parallel island solver toggle.
    uint32_t workers = 0;
    bool serialNarrowphase = false;
    bool serialIslands = false;
};

Options ParseOptions(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--serial-narrowphase") {
            options.serialNarrowphase = true;
            continue;
        }
        if (argument == "--serial-islands") {
            options.serialIslands = true;
            continue;
        }
        if (index + 1 >= argc) {
            throw std::runtime_error("Missing value for " + argument);
        }
        const std::string value = argv[++index];
        if (argument == "--scenario") {
            options.scenario = value;
        } else if (argument == "--bodies") {
            options.bodyCount = std::stoull(value);
        } else if (argument == "--frames") {
            options.frameCount = std::stoull(value);
        } else if (argument == "--warmup") {
            options.warmupFrames = std::stoull(value);
        } else if (argument == "--seed") {
            options.seed = static_cast<uint32_t>(std::stoul(value));
        } else if (argument == "--workers") {
            options.workers = static_cast<uint32_t>(std::stoul(value));
        } else if (argument == "--output") {
            options.output = value;
        } else {
            throw std::runtime_error("Unknown option: " + argument);
        }
    }
    if (options.bodyCount == 0 || options.frameCount == 0) {
        throw std::runtime_error("Body and frame counts must be positive");
    }
    return options;
}

nlohmann::json Percentiles(std::vector<double> samples) {
    std::sort(samples.begin(), samples.end());
    auto percentile = [&](double fraction) {
        const double scaled = fraction * static_cast<double>(samples.size() - 1);
        return samples[static_cast<std::size_t>(std::ceil(scaled))];
    };
    return {
        {"p50", percentile(0.50)},
        {"p95", percentile(0.95)},
        {"p99", percentile(0.99)},
    };
}

uint32_t AddGround(Runtime::Physics::PhysicsWorld& world, float restitution = 0.1f) {
    Runtime::Physics::RigidBodyDesc body;
    body.position = glm::vec3(0.0f, -0.5f, 0.0f);
    body.isStatic = true;
    body.useGravity = false;
    const uint32_t id = world.CreateRigidBody(body);

    Runtime::Physics::ColliderDesc collider;
    collider.shape = std::make_shared<Runtime::Physics::BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
    collider.material.restitution = restitution;
    collider.material.dynamicFriction = 0.6f;
    collider.material.staticFriction = 0.7f;
    if (!world.AttachCollider(id, collider)) {
        throw std::runtime_error("Failed to attach ground collider");
    }
    return id;
}

void AddBoxField(Runtime::Physics::PhysicsWorld& world, const Options& options) {
    std::mt19937 random(options.seed);
    std::uniform_real_distribution<float> jitter(-0.04f, 0.04f);
    const std::size_t side = static_cast<std::size_t>(std::ceil(std::sqrt(static_cast<double>(options.bodyCount))));
    const auto shape = std::make_shared<Runtime::Physics::BoxShape>(glm::vec3(0.45f));

    for (std::size_t index = 0; index < options.bodyCount; ++index) {
        const std::size_t x = index % side;
        const std::size_t z = (index / side) % side;
        const std::size_t layer = index / (side * side);

        Runtime::Physics::RigidBodyDesc body;
        body.position = glm::vec3(
            (static_cast<float>(x) - static_cast<float>(side) * 0.5f) * 0.92f + jitter(random),
            0.5f + static_cast<float>(layer) * 0.92f + jitter(random),
            (static_cast<float>(z) - static_cast<float>(side) * 0.5f) * 0.92f + jitter(random));
        body.orientation = glm::normalize(glm::quat(1.0f, jitter(random), jitter(random), jitter(random)));
        body.inertiaTensorDiagonal = glm::vec3(0.135f);
        const uint32_t id = world.CreateRigidBody(body);

        Runtime::Physics::ColliderDesc collider;
        collider.shape = shape;
        collider.material.restitution = 0.05f;
        collider.material.dynamicFriction = 0.55f;
        collider.material.staticFriction = 0.65f;
        if (!world.AttachCollider(id, collider)) {
            throw std::runtime_error("Failed to attach dynamic collider");
        }
    }
}

void AddBoxStacks(Runtime::Physics::PhysicsWorld& world, const Options& options, std::size_t stackHeight) {
    std::mt19937 random(options.seed);
    std::uniform_real_distribution<float> jitter(-0.02f, 0.02f);
    const std::size_t stackCount = std::max<std::size_t>(1, options.bodyCount / stackHeight);
    const std::size_t side = static_cast<std::size_t>(std::ceil(std::sqrt(static_cast<double>(stackCount))));
    const auto shape = std::make_shared<Runtime::Physics::BoxShape>(glm::vec3(0.45f));

    for (std::size_t stack = 0; stack < stackCount; ++stack) {
        const float centerX = (static_cast<float>(stack % side) - static_cast<float>(side) * 0.5f) * 1.6f;
        const float centerZ = (static_cast<float>((stack / side) % side) - static_cast<float>(side) * 0.5f) * 1.6f;
        for (std::size_t level = 0; level < stackHeight; ++level) {
            Runtime::Physics::RigidBodyDesc body;
            body.position = glm::vec3(
                centerX + jitter(random),
                0.45f + static_cast<float>(level) * 0.92f + jitter(random),
                centerZ + jitter(random));
            body.inertiaTensorDiagonal = glm::vec3(0.135f);
            const uint32_t id = world.CreateRigidBody(body);

            Runtime::Physics::ColliderDesc collider;
            collider.shape = shape;
            collider.material.restitution = 0.05f;
            collider.material.dynamicFriction = 0.55f;
            collider.material.staticFriction = 0.65f;
            if (!world.AttachCollider(id, collider)) {
                throw std::runtime_error("Failed to attach stack collider");
            }
        }
    }
}

std::shared_ptr<Runtime::Physics::ConvexHullShape> MakeHullShape(uint32_t seed) {
    std::mt19937 random(seed);
    std::uniform_real_distribution<float> coordinate(-1.0f, 1.0f);
    std::vector<glm::vec3> vertices;
    vertices.reserve(32);
    for (int index = 0; index < 32; ++index) {
        glm::vec3 point(coordinate(random), coordinate(random), coordinate(random));
        if (glm::dot(point, point) < 1e-6f) {
            point = glm::vec3(0.5f);
        }
        vertices.push_back(glm::normalize(point) * 0.4f);
    }
    return std::make_shared<Runtime::Physics::ConvexHullShape>(std::move(vertices));
}

void AddConvexHulls(Runtime::Physics::PhysicsWorld& world, const Options& options) {
    std::mt19937 random(options.seed);
    std::uniform_real_distribution<float> jitter(-0.05f, 0.05f);
    const std::size_t side = static_cast<std::size_t>(std::ceil(std::sqrt(static_cast<double>(options.bodyCount))));
    const auto shape = MakeHullShape(options.seed ^ 0x9e3779b9u);

    for (std::size_t index = 0; index < options.bodyCount; ++index) {
        const std::size_t x = index % side;
        const std::size_t z = (index / side) % side;
        const std::size_t layer = index / (side * side);

        Runtime::Physics::RigidBodyDesc body;
        body.position = glm::vec3(
            (static_cast<float>(x) - static_cast<float>(side) * 0.5f) * 1.1f + jitter(random),
            0.6f + static_cast<float>(layer) * 1.1f + jitter(random),
            (static_cast<float>(z) - static_cast<float>(side) * 0.5f) * 1.1f + jitter(random));
        body.orientation = glm::normalize(glm::quat(1.0f, jitter(random), jitter(random), jitter(random)));
        body.inertiaTensorDiagonal = glm::vec3(0.1f);
        const uint32_t id = world.CreateRigidBody(body);

        Runtime::Physics::ColliderDesc collider;
        collider.shape = shape;
        collider.material.restitution = 0.1f;
        collider.material.dynamicFriction = 0.5f;
        collider.material.staticFriction = 0.6f;
        if (!world.AttachCollider(id, collider)) {
            throw std::runtime_error("Failed to attach hull collider");
        }
    }
}

void AddTriggerScene(Runtime::Physics::PhysicsWorld& world, const Options& options) {
    std::mt19937 random(options.seed);
    std::uniform_real_distribution<float> jitter(-0.4f, 0.4f);
    const std::size_t triggerCount = std::max<std::size_t>(1, options.bodyCount / 4);
    const std::size_t side = static_cast<std::size_t>(std::ceil(std::sqrt(static_cast<double>(triggerCount))));
    const auto triggerPosition = [&](std::size_t index) {
        return glm::vec3(
            (static_cast<float>(index % side) - static_cast<float>(side) * 0.5f) * 3.0f,
            1.4f,
            (static_cast<float>((index / side) % side) - static_cast<float>(side) * 0.5f) * 3.0f);
    };

    const auto triggerShape = std::make_shared<Runtime::Physics::BoxShape>(glm::vec3(1.2f, 0.8f, 1.2f));
    for (std::size_t index = 0; index < triggerCount; ++index) {
        Runtime::Physics::RigidBodyDesc body;
        body.position = triggerPosition(index);
        body.isStatic = true;
        body.useGravity = false;
        const uint32_t id = world.CreateRigidBody(body);

        Runtime::Physics::ColliderDesc collider;
        collider.shape = triggerShape;
        collider.isTrigger = true;
        if (!world.AttachCollider(id, collider)) {
            throw std::runtime_error("Failed to attach trigger collider");
        }
    }

    const auto sphereShape = std::make_shared<Runtime::Physics::SphereShape>(0.3f);
    for (std::size_t index = 0; index < options.bodyCount; ++index) {
        const glm::vec3 base = triggerPosition(index % triggerCount);
        Runtime::Physics::RigidBodyDesc body;
        body.position = glm::vec3(
            base.x + jitter(random),
            base.y + 1.6f + 0.15f * static_cast<float>(index / triggerCount) + jitter(random) * 0.2f,
            base.z + jitter(random));
        body.inertiaTensorDiagonal = glm::vec3(0.036f);
        const uint32_t id = world.CreateRigidBody(body);

        Runtime::Physics::ColliderDesc collider;
        collider.shape = sphereShape;
        collider.material.restitution = 0.1f;
        collider.material.dynamicFriction = 0.4f;
        collider.material.staticFriction = 0.5f;
        if (!world.AttachCollider(id, collider)) {
            throw std::runtime_error("Failed to attach trigger-scene collider");
        }
    }
}

struct CcdState {
    std::vector<glm::vec3> spawnPositions;
    std::vector<uint32_t> bodyIds;
};

// Relaunch settled spheres so the CCD workload stays active for every sampled
// frame instead of spending the whole run in a single one-shot impact.
// Restitution is zero, so a sphere settles the frame after impact and is
// relaunched on the next tick; staggered spawn heights spread impacts across
// three frame phases.
void RelaunchCcdBodies(Runtime::Physics::PhysicsWorld& world, const CcdState& state) {
    for (std::size_t index = 0; index < state.bodyIds.size(); ++index) {
        Runtime::Physics::RigidBody* body = world.GetRigidBody(state.bodyIds[index]);
        if (!body) {
            continue;
        }
        if (glm::dot(body->LinearVelocity(), body->LinearVelocity()) < 25.0f) {
            body->SetPosition(state.spawnPositions[index]);
            body->SetLinearVelocity(glm::vec3(0.0f, -250.0f, 0.0f));
            body->SetAngularVelocity(glm::vec3(0.0f));
        }
    }
}

void AddCcdScene(Runtime::Physics::PhysicsWorld& world, const Options& options, CcdState& state) {
    std::mt19937 random(options.seed);
    std::uniform_real_distribution<float> jitter(-0.1f, 0.1f);
    const std::size_t side = static_cast<std::size_t>(std::ceil(std::sqrt(static_cast<double>(options.bodyCount))));
    const auto shape = std::make_shared<Runtime::Physics::SphereShape>(0.25f);

    for (std::size_t index = 0; index < options.bodyCount; ++index) {
        const std::size_t x = index % side;
        const std::size_t z = (index / side) % side;
        const std::size_t layer = index / (side * side);

        Runtime::Physics::RigidBodyDesc body;
        body.position = glm::vec3(
            (static_cast<float>(x) - static_cast<float>(side) * 0.5f) * 1.5f + jitter(random),
            1.5f + static_cast<float>(index % 3) * 2.08f + static_cast<float>(layer) * 1.0f + jitter(random),
            (static_cast<float>(z) - static_cast<float>(side) * 0.5f) * 1.5f + jitter(random));
        body.linearVelocity = glm::vec3(0.0f, -250.0f, 0.0f);
        body.inertiaTensorDiagonal = glm::vec3(0.025f);
        const uint32_t id = world.CreateRigidBody(body);

        Runtime::Physics::ColliderDesc collider;
        collider.shape = shape;
        collider.material.restitution = 0.0f;
        collider.material.dynamicFriction = 0.6f;
        collider.material.staticFriction = 0.7f;
        if (!world.AttachCollider(id, collider)) {
            throw std::runtime_error("Failed to attach CCD collider");
        }
        state.spawnPositions.push_back(body.position);
        state.bodyIds.push_back(id);
    }
}

struct ChurnState {
    std::deque<uint32_t> liveBodies;
    std::mt19937 random;
    std::shared_ptr<Runtime::Physics::CollisionShape> shape;
    std::size_t spawnCounter = 0;
};

void SpawnChurnBody(Runtime::Physics::PhysicsWorld& world, ChurnState& state) {
    std::uniform_real_distribution<float> jitter(-0.3f, 0.3f);
    const std::size_t spawnIndex = state.spawnCounter++;

    Runtime::Physics::RigidBodyDesc body;
    body.position = glm::vec3(
        (static_cast<float>(spawnIndex % 16) - 8.0f) * 0.9f + jitter(state.random),
        6.0f + static_cast<float>((spawnIndex / 16) % 8) * 0.8f + jitter(state.random),
        (static_cast<float>((spawnIndex / 128) % 16) - 8.0f) * 0.9f + jitter(state.random));
    body.inertiaTensorDiagonal = glm::vec3(0.036f);
    const uint32_t id = world.CreateRigidBody(body);

    Runtime::Physics::ColliderDesc collider;
    collider.shape = state.shape;
    collider.material.restitution = 0.1f;
    collider.material.dynamicFriction = 0.5f;
    collider.material.staticFriction = 0.6f;
    if (!world.AttachCollider(id, collider)) {
        throw std::runtime_error("Failed to attach churn collider");
    }
    state.liveBodies.push_back(id);
}

struct ScenarioDef {
    std::string name;
    bool enableCcd = false;
    int ccdMaxSubSteps = 8;
    float groundRestitution = 0.1f;
    std::function<void(Runtime::Physics::PhysicsWorld&, const Options&)> build;
    // Optional per-frame hook executed after every world.Step, warmup included.
    std::function<void(Runtime::Physics::PhysicsWorld&, std::size_t)> tick;
};

constexpr const char* kScenarioNames[] = {
    "boxfield", "stack3", "stack5", "hull", "trigger", "churn", "ccd",
};

ScenarioDef MakeChurnScenario(const Options& options) {
    ScenarioDef def;
    def.name = "churn";
    auto state = std::make_shared<ChurnState>();
    state->random.seed(options.seed);
    state->shape = std::make_shared<Runtime::Physics::BoxShape>(glm::vec3(0.3f));
    def.build = [state](Runtime::Physics::PhysicsWorld& world, const Options& opts) {
        for (std::size_t index = 0; index < opts.bodyCount; ++index) {
            SpawnChurnBody(world, *state);
        }
    };
    def.tick = [state](Runtime::Physics::PhysicsWorld& world, std::size_t) {
        const std::size_t churnCount = std::max<std::size_t>(1, state->liveBodies.size() / 20);
        for (std::size_t count = 0; count < churnCount; ++count) {
            world.DestroyRigidBody(state->liveBodies.front());
            state->liveBodies.pop_front();
        }
        for (std::size_t count = 0; count < churnCount; ++count) {
            SpawnChurnBody(world, *state);
        }
    };
    return def;
}

ScenarioDef MakeCcdScenario() {
    ScenarioDef def;
    def.name = "ccd";
    def.enableCcd = true;
    def.ccdMaxSubSteps = 12;
    // Zero ground restitution so spheres settle right after impact and are
    // relaunched on the next tick, keeping every sampled frame CCD-active.
    def.groundRestitution = 0.0f;
    auto state = std::make_shared<CcdState>();
    def.build = [state](Runtime::Physics::PhysicsWorld& world, const Options& opts) {
        AddCcdScene(world, opts, *state);
    };
    def.tick = [state](Runtime::Physics::PhysicsWorld& world, std::size_t) {
        RelaunchCcdBodies(world, *state);
    };
    return def;
}

ScenarioDef BuildScenario(const std::string& name, const Options& options) {
    if (name == "churn") {
        return MakeChurnScenario(options);
    }
    if (name == "ccd") {
        return MakeCcdScenario();
    }

    ScenarioDef def;
    def.name = name;
    if (name == "boxfield") {
        def.build = [](Runtime::Physics::PhysicsWorld& world, const Options& opts) { AddBoxField(world, opts); };
    } else if (name == "stack3") {
        def.build = [](Runtime::Physics::PhysicsWorld& world, const Options& opts) { AddBoxStacks(world, opts, 3); };
    } else if (name == "stack5") {
        def.build = [](Runtime::Physics::PhysicsWorld& world, const Options& opts) { AddBoxStacks(world, opts, 5); };
    } else if (name == "hull") {
        def.build = [](Runtime::Physics::PhysicsWorld& world, const Options& opts) { AddConvexHulls(world, opts); };
    } else if (name == "trigger") {
        def.build = [](Runtime::Physics::PhysicsWorld& world, const Options& opts) { AddTriggerScene(world, opts); };
    } else {
        throw std::runtime_error(
            "Unknown scenario: " + name +
            " (expected boxfield, stack3, stack5, hull, trigger, churn, ccd or all)");
    }
    return def;
}

nlohmann::json VersionInfo() {
    const std::time_t now = std::time(nullptr);
    char timestamp[32] = "unknown";
    if (const std::tm* utc = std::gmtime(&now)) {
        std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", utc);
    }
    return {
        {"project", MYGAME_PROJECT_VERSION},
        {"compiler", std::string(MYGAME_COMPILER_ID) + " " + MYGAME_COMPILER_VERSION},
        {"buildType", MYGAME_BUILD_TYPE},
        {"gitCommit", MYGAME_GIT_COMMIT},
        {"gitDirty", MYGAME_GIT_DIRTY != 0},
        {"generatedAtUtc", timestamp},
    };
}

void WriteReport(const nlohmann::json& report, const std::string& outputPath) {
    std::ofstream output(outputPath);
    if (!output) {
        throw std::runtime_error("Failed to open output: " + outputPath);
    }
    output << report.dump(2) << '\n';
}

nlohmann::json RunScenario(const ScenarioDef& def, const Options& options) {
    constexpr float fixedTimeStep = 1.0f / 120.0f;

    Runtime::Physics::PhysicsWorld world;
    world.SetFixedTimeStep(fixedTimeStep);
    world.SetGravity(glm::vec3(0.0f, -10.5f, 0.0f));
    world.SetContinuousCollisionEnabled(def.enableCcd);
    world.SetCcdMaxSubSteps(def.ccdMaxSubSteps);
    world.SetSolverIterations(6);
    world.SetPhysicsWorkerCount(options.workers);
    world.SetParallelNarrowphaseEnabled(!options.serialNarrowphase);
    world.SetParallelIslandSolverEnabled(!options.serialIslands);
    AddGround(world, def.groundRestitution);
    def.build(world, options);

    for (std::size_t frame = 0; frame < options.warmupFrames; ++frame) {
        world.Step(fixedTimeStep);
        if (def.tick) {
            def.tick(world, frame);
        }
    }

    std::vector<double> integrationSamples;
    std::vector<double> broadPhaseSamples;
    std::vector<double> narrowPhaseSamples;
    std::vector<double> solverSamples;
    std::vector<double> islandBuildSamples;
    std::vector<double> totalSamples;
    integrationSamples.reserve(options.frameCount);
    broadPhaseSamples.reserve(options.frameCount);
    narrowPhaseSamples.reserve(options.frameCount);
    solverSamples.reserve(options.frameCount);
    islandBuildSamples.reserve(options.frameCount);
    totalSamples.reserve(options.frameCount);

    uint64_t candidates = 0;
    uint64_t narrowPhaseTests = 0;
    uint64_t gjkCalls = 0;
    uint64_t gjkFailures = 0;
    uint64_t epaCalls = 0;
    uint64_t epaFailures = 0;
    uint64_t manifolds = 0;
    uint64_t contactPoints = 0;
    uint64_t midphaseNewPairs = 0;
    uint64_t midphaseRemovedPairs = 0;
    uint64_t narrowPhaseJobs = 0;
    double narrowPhaseWorkerBusyMs = 0.0;
    double narrowPhaseTailWaitMs = 0.0;
    uint64_t islandSolverJobs = 0;
    double islandSolverWorkerBusyMs = 0.0;
    double islandSolverTailWaitMs = 0.0;
    Runtime::Physics::PhysicsStepStats lastStats;

    for (std::size_t frame = 0; frame < options.frameCount; ++frame) {
        world.Step(fixedTimeStep);
        if (def.tick) {
            def.tick(world, options.warmupFrames + frame);
        }
        lastStats = world.LastStepStats();
        integrationSamples.push_back(lastStats.integrationMilliseconds);
        broadPhaseSamples.push_back(lastStats.broadPhaseMilliseconds);
        narrowPhaseSamples.push_back(lastStats.narrowPhaseMilliseconds);
        solverSamples.push_back(lastStats.solverMilliseconds);
        islandBuildSamples.push_back(lastStats.islandBuildMilliseconds);
        totalSamples.push_back(lastStats.totalMilliseconds);
        candidates += lastStats.broadPhaseCandidateCount;
        narrowPhaseTests += lastStats.narrowPhaseTestCount;
        gjkCalls += lastStats.gjkCallCount;
        gjkFailures += lastStats.gjkFailureCount;
        epaCalls += lastStats.epaCallCount;
        epaFailures += lastStats.epaFailureCount;
        manifolds += lastStats.manifoldCount;
        contactPoints += lastStats.contactPointCount;
        midphaseNewPairs += lastStats.midphaseNewPairCount;
        midphaseRemovedPairs += lastStats.midphaseRemovedPairCount;
        narrowPhaseJobs += lastStats.narrowPhaseJobCount;
        narrowPhaseWorkerBusyMs += lastStats.narrowPhaseWorkerBusyMilliseconds;
        narrowPhaseTailWaitMs += lastStats.narrowPhaseTailWaitMilliseconds;
        islandSolverJobs += lastStats.islandSolverJobCount;
        islandSolverWorkerBusyMs += lastStats.islandSolverWorkerBusyMilliseconds;
        islandSolverTailWaitMs += lastStats.islandSolverTailWaitMilliseconds;
    }

    glm::dvec3 positionChecksum(0.0);
    for (const auto& entry : world.Bodies()) {
        positionChecksum += glm::dvec3(entry.second.Position());
    }

    return {
        {"config", {
            {"scenario", def.name},
            {"seed", options.seed},
            {"bodyCount", options.bodyCount},
            {"frameCount", options.frameCount},
            {"warmupFrames", options.warmupFrames},
            {"fixedTimeStep", fixedTimeStep},
            {"solverIterations", world.SolverIterations()},
            {"ccdEnabled", world.ContinuousCollisionEnabled()},
            {"physicsWorkers", world.PhysicsWorkerCount()},
            {"parallelNarrowphase", !options.serialNarrowphase},
            {"parallelIslandSolver", !options.serialIslands},
        }},
        {"leaves", {
            {"static", lastStats.staticBvhLeafCount},
            {"dynamic", lastStats.dynamicBvhLeafCount},
        }},
        {"totals", {
            {"broadPhaseCandidates", candidates},
            {"narrowPhaseTests", narrowPhaseTests},
            {"gjkCalls", gjkCalls},
            {"gjkFailures", gjkFailures},
            {"epaCalls", epaCalls},
            {"epaFailures", epaFailures},
            {"manifolds", manifolds},
            {"contactPoints", contactPoints},
        }},
        {"timingsMilliseconds", {
            {"integration", Percentiles(integrationSamples)},
            {"broadPhase", Percentiles(broadPhaseSamples)},
            {"narrowPhase", Percentiles(narrowPhaseSamples)},
            {"solver", Percentiles(solverSamples)},
            {"islandBuild", Percentiles(islandBuildSamples)},
            {"total", Percentiles(totalSamples)},
        }},
        {"futureMetrics", {
            // Phase 2 landed: report the real persistent pair pool metrics.
            {"midphase", {
                {"available", true},
                {"activePairs", lastStats.midphaseActivePairCount},
                {"newPairs", midphaseNewPairs},
                {"removedPairs", midphaseRemovedPairs},
            }},
            // Phase 6 landed: report real parallel narrow-phase telemetry.
            // workerBusy/tailWait are summed over sampled fixed steps; wall
            // time stays in timingsMilliseconds.narrowPhase (never mix the
            // worker CPU-time sum with wall-clock percentiles).
            {"parallelNarrowphase", {
                {"available", true},
                {"workerCount", lastStats.narrowPhaseWorkerCount},
                {"totalJobs", narrowPhaseJobs},
                {"workerBusyMsTotal", narrowPhaseWorkerBusyMs},
                {"tailWaitMsTotal", narrowPhaseTailWaitMs},
            }},
            // Phase 7 landed: report real island build metrics. Counts are
            // the last sampled fixed step's snapshot; the island build wall
            // time is in timingsMilliseconds.islandBuild.
            {"islands", {
                {"available", true},
                {"islandCount", lastStats.islandCount},
                {"maxIslandBodyCount", lastStats.islandMaxBodyCount},
            }},
            // Phase 8 landed: report real island solver telemetry (same
            // wall-clock vs worker-CPU-time split as the narrow-phase block).
            {"parallelIslandSolver", {
                {"available", true},
                {"workerCount", lastStats.islandSolverWorkerCount},
                {"totalJobs", islandSolverJobs},
                {"workerBusyMsTotal", islandSolverWorkerBusyMs},
                {"tailWaitMsTotal", islandSolverTailWaitMs},
            }},
            {"sleep", {{"available", false}}},
        }},
        {"positionChecksum", {positionChecksum.x, positionChecksum.y, positionChecksum.z}},
    };
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Options options = ParseOptions(argc, argv);
        if (options.scenario == "all") {
            nlohmann::json scenarios = nlohmann::json::object();
            for (const char* name : kScenarioNames) {
                scenarios[name] = RunScenario(BuildScenario(name, options), options);
            }
            const nlohmann::json report = {
                {"schemaVersion", 2},
                {"version", VersionInfo()},
                {"scenarios", scenarios},
            };
            WriteReport(report, options.output);
            std::cout << "Wrote all-scenario baseline to " << options.output << '\n';
            return 0;
        }

        nlohmann::json report = RunScenario(BuildScenario(options.scenario, options), options);
        report["schemaVersion"] = 2;
        report["version"] = VersionInfo();
        WriteReport(report, options.output);
        std::cout << report.dump(2) << '\n';
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "PhysicsBenchmark: " << exception.what() << '\n';
        return 1;
    }
}