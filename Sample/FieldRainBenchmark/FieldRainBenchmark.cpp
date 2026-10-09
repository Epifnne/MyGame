// Headless FieldRain performance / stability benchmark.
// Runs the exact BoxStackSample FieldRain scene (no rendering) for a fixed
// number of frames and reports physics step time, sink count and the maximum
// speed seen (energy explosion detector).
#include <Physics/CollisionShape.h>
#include <Physics/PhysicsWorld.h>

#include <chrono>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

// Run the sample's deterministic rain, including intentionally overlapping random spawns.
int main(int argc, char** argv) {
    using namespace Runtime::Physics;
    // argv[1] == "0" zeroes restitution to isolate the energy source.
    const float restitution = (argc > 1 && std::string(argv[1]) == "0") ? 0.0f : -1.0f;
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetGravity({0.0f, -10.5f, 0.0f});
    world.SetContinuousCollisionEnabled(true);
    world.SetSolverIterations(10);

    RigidBodyDesc ground;
    ground.position = {0.0f, -0.5f, 0.0f};
    ground.isStatic = true;
    ground.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(ground);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(20.0f, 0.5f, 20.0f));
    groundCollider.material.restitution = restitution < 0.0f ? 0.1f : restitution;
    groundCollider.material.dynamicFriction = 0.6f;
    groundCollider.material.staticFriction = 0.7f;
    world.AttachCollider(groundId, groundCollider);

    const auto boxShape = std::make_shared<BoxShape>(glm::vec3(0.45f));
    std::mt19937 rng(1337);
    std::uniform_real_distribution<float> posDist(-3.5f, 3.5f);
    std::vector<uint32_t> bodies;
    float spawnAccum = 0.0f;
    float maxSpeed = 0.0f;
    float maxPenetration = 0.0f;
    int sunkCount = 0;
    int worstFrame = 0;
    uint32_t worstA = 0, worstB = 0;
    glm::vec3 worstNormal(0.0f);
    glm::vec3 worstLocalA(0.0f), worstLocalB(0.0f);
    float worstYA = 0.0f, worstYB = 0.0f;
    uint64_t toiHits = 0, budgetHits = 0;

    const int kFrames = argc > 2 ? std::stoi(argv[2]) : 1800;
    if (kFrames <= 0) {
        std::cerr << "Frame count must be positive\n";
        return 2;
    }
    std::vector<double> timings;
    timings.reserve(static_cast<std::size_t>(kFrames));
    float worstGroundPenetration = 0.0f;
    float settledGroundPenetration = 0.0f;
    int groundFrame = 0;
    uint32_t groundBody = 0;
    float groundY = 0.0f, groundSpeed = 0.0f, groundSpin = 0.0f;
    uint64_t groundToi = 0;
    std::size_t groundPoints = 0;
    float groundContactPen = 0.0f;
    const auto start = std::chrono::steady_clock::now();
    double physicsMsTotal = 0.0;
    for (int frame = 0; frame < kFrames; ++frame) {
        const float dt = 1.0f / 120.0f;
        spawnAccum += dt * 24.0f;
        while (spawnAccum >= 1.0f && bodies.size() < 300) {
            spawnAccum -= 1.0f;
            RigidBodyDesc desc;
            desc.position = {posDist(rng), 14.0f, posDist(rng)};
            desc.mass = 1.0f;
            desc.inertiaTensorDiagonal = glm::vec3(0.135f);
            const uint32_t id = world.CreateRigidBody(desc);
            ColliderDesc collider;
            collider.shape = boxShape;
            collider.material.restitution = restitution < 0.0f ? 0.05f : restitution;
            collider.material.dynamicFriction = 0.55f;
            collider.material.staticFriction = 0.65f;
            if (world.AttachCollider(id, collider)) {
                bodies.push_back(id);
            }
        }
        const auto stepStart = std::chrono::steady_clock::now();
        world.Step(dt);
        toiHits += world.LastStepStats().ccdToiHitCount;
        budgetHits += world.LastStepStats().ccdBudgetExhaustionCount;
        const double stepMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - stepStart).count();
        physicsMsTotal += stepMs;
        timings.push_back(stepMs);

        for (std::size_t index = 0; index < bodies.size();) {
            const auto* body = world.GetRigidBody(bodies[index]);
            if (body && body->Position().y < -15.0f) {
                world.DestroyRigidBody(bodies[index]);
                bodies.erase(bodies.begin() + static_cast<std::ptrdiff_t>(index));
            } else {
                ++index;
            }
        }
        for (const uint32_t id : bodies) {
            const auto* body = world.GetRigidBody(id);
            const float speed = glm::length(body->LinearVelocity());
            if (speed > 25.0f && maxSpeed <= 25.0f) {
                // First explosion event: dump the box's context.
                std::printf(
                    "[explosion] frame=%d box=%u y=%.3f vy=%.2f speed=%.2f sleeping=%d contacts:",
                    frame, id, body->Position().y, body->LinearVelocity().y, speed,
                    body->IsSleeping() ? 1 : 0);
                for (const ContactManifold& contact : world.Contacts()) {
                    if (contact.bodyA != id && contact.bodyB != id) {
                        continue;
                    }
                    float maxPen = -1.0f;
                    for (std::size_t p = 0; p < contact.pointCount; ++p) {
                        maxPen = std::max(maxPen, contact.Point(p).penetration);
                    }
                    std::printf(" %u(pen=%.3f,n=%zu,J=%.1f)",
                        contact.bodyA == id ? contact.bodyB : contact.bodyA,
                        maxPen, contact.pointCount,
                        static_cast<double>(contact.fixedStepNormalImpulse));
                }
                std::printf("\n");
            }
            maxSpeed = std::max(maxSpeed, speed);
            const AABB bounds = boxShape->ComputeAABB({body->Position(), body->Orientation()});
            if (bounds.min.x >= -20.0f && bounds.max.x <= 20.0f &&
                bounds.min.z >= -20.0f && bounds.max.z <= 20.0f) {
                if (-bounds.min.y > worstGroundPenetration) {
                    worstGroundPenetration = -bounds.min.y;
                    groundFrame = frame;
                    groundBody = id;
                    groundY = body->Position().y;
                    groundSpeed = glm::length(body->LinearVelocity());
                    groundSpin = glm::length(body->AngularVelocity());
                    groundToi = world.LastStepStats().ccdToiHitCount;
                    groundPoints = 0;
                    groundContactPen = 0.0f;
                    for (const auto& contact : world.Contacts())
                        if (MakePairKey(contact.bodyA, contact.bodyB) == MakePairKey(groundId, id)) {
                            groundPoints = contact.pointCount;
                            for (std::size_t p = 0; p < contact.pointCount; ++p)
                                groundContactPen = std::max(groundContactPen, contact.Point(p).penetration);
                        }
                }
                if (frame >= 2400)
                    settledGroundPenetration = std::max(settledGroundPenetration, -bounds.min.y);
            }
        }
        for (const ContactManifold& contact : world.Contacts()) {
            for (std::size_t p = 0; p < contact.pointCount; ++p) {
                if (contact.Point(p).penetration > maxPenetration) {
                    maxPenetration = contact.Point(p).penetration;
                    worstFrame = frame;
                    worstA = contact.bodyA;
                    worstB = contact.bodyB;
                    worstNormal = contact.normal;
                    worstLocalA = contact.Point(p).surfaceLocalA;
                    worstLocalB = contact.Point(p).surfaceLocalB;
                    worstYA = world.GetRigidBody(worstA)->Position().y;
                    worstYB = world.GetRigidBody(worstB)->Position().y;
                }
            }
        }
    }
    for (const uint32_t id : bodies) {
        if (world.GetRigidBody(id)->Position().y < 0.0f) {
            ++sunkCount;
        }
    }
    const double wallMs = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    std::printf(
        "[fieldrain-bench e=%.2f] frames=%d bodies=%zu wallMs=%.1f physicsMsTotal=%.1f "
        "avgPhysicsMs=%.3f maxSpeed=%.2f maxPenetration=%.4f sunk=%d\n",
        restitution < 0.0f ? 0.075f : restitution,
        kFrames, bodies.size(), wallMs, physicsMsTotal,
        physicsMsTotal / kFrames, maxSpeed, maxPenetration, sunkCount);
    std::printf("[worst] frame=%d pair=%u-%u y=%.4f/%.4f normal=%.3f,%.3f,%.3f toiHits=%llu budgetHits=%llu sleeping=%zu\n",
        worstFrame, worstA, worstB, worstYA, worstYB, worstNormal.x, worstNormal.y, worstNormal.z,
        static_cast<unsigned long long>(toiHits), static_cast<unsigned long long>(budgetHits),
        world.LastStepStats().sleepingBodyCount);
    std::printf("[worst-anchors] a=%.3f,%.3f,%.3f b=%.3f,%.3f,%.3f\n",
        worstLocalA.x, worstLocalA.y, worstLocalA.z, worstLocalB.x, worstLocalB.y, worstLocalB.z);
    std::sort(timings.begin(), timings.end());
    std::printf("[quality] p95Ms=%.3f peakMs=%.3f groundPen=%.5f groundPenAfter20s=%.5f\n",
        timings[static_cast<std::size_t>(0.95 * (timings.size() - 1))], timings.back(),
        worstGroundPenetration, settledGroundPenetration);
    std::printf("[ground-worst] frame=%d body=%u y=%.4f speed=%.3f spin=%.3f toi=%llu points=%zu pen=%.4f\n",
        groundFrame, groundBody, groundY, groundSpeed, groundSpin,
        static_cast<unsigned long long>(groundToi), groundPoints, groundContactPen);
    return sunkCount == 0 && maxSpeed < 25.0f && settledGroundPenetration < 0.04f ? 0 : 1;
}
