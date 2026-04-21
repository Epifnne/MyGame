#include "Physics/ContinuousCollision.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "Physics/NarrowPhase.h"

namespace Runtime {
namespace Physics {

ShapeTransform ContinuousCollisionDetector::InterpolateTransform(const RigidBody& body, float t) {
    ShapeTransform tf;
    tf.position = body.Position() + body.LinearVelocity() * t;

    const glm::vec3 angularVelocity = body.AngularVelocity();
    const float speed = glm::length(angularVelocity);
    if (speed <= 1e-6f) {
        tf.orientation = body.Orientation();
        return tf;
    }

    const glm::vec3 axis = angularVelocity / speed;
    const float angle = speed * t;
    const glm::quat dq = glm::angleAxis(angle, axis);
    tf.orientation = glm::normalize(dq * body.Orientation());
    return tf;
}

AABB ContinuousCollisionDetector::SweptAabb(const Collider& collider, const RigidBody& body, float maxTime) {
    const ShapeTransform t0 = InterpolateTransform(body, 0.0f);
    const ShapeTransform t1 = InterpolateTransform(body, maxTime);
    return AABB::Merge(collider.ComputeAABB(t0), collider.ComputeAABB(t1));
}

bool ContinuousCollisionDetector::GenerateContactAtTime(
    const Collider& colliderA,
    const RigidBody& bodyA,
    const Collider& colliderB,
    const RigidBody& bodyB,
    float t,
    ContactManifold& outContact) const {
    RigidBody interpolatedA = bodyA;
    RigidBody interpolatedB = bodyB;

    const ShapeTransform tfA = InterpolateTransform(bodyA, t);
    const ShapeTransform tfB = InterpolateTransform(bodyB, t);
    interpolatedA.SetPosition(tfA.position);
    interpolatedA.SetOrientation(tfA.orientation);
    interpolatedB.SetPosition(tfB.position);
    interpolatedB.SetOrientation(tfB.orientation);

    GjkEpaNarrowPhase narrow;
    return narrow.GenerateContact(colliderA, interpolatedA, colliderB, interpolatedB, outContact);
}

TimeOfImpact ContinuousCollisionDetector::FindEarliestImpact(
    const std::unordered_map<uint32_t, Collider>& colliders,
    const std::unordered_map<uint32_t, RigidBody>& bodies,
    float maxTime) const {
    TimeOfImpact best;
    best.toi = maxTime;

    if (maxTime <= 0.0f || colliders.size() < 2) {
        return best;
    }

    std::vector<uint32_t> ids;
    ids.reserve(colliders.size());
    for (const auto& kv : colliders) {
        ids.push_back(kv.first);
    }

    for (size_t i = 0; i < ids.size(); ++i) {
        const uint32_t idA = ids[i];
        auto colliderItA = colliders.find(idA);
        auto bodyItA = bodies.find(idA);
        if (colliderItA == colliders.end() || bodyItA == bodies.end()) {
            continue;
        }

        for (size_t j = i + 1; j < ids.size(); ++j) {
            const uint32_t idB = ids[j];
            auto colliderItB = colliders.find(idB);
            auto bodyItB = bodies.find(idB);
            if (colliderItB == colliders.end() || bodyItB == bodies.end()) {
                continue;
            }

            const Collider& colliderA = colliderItA->second;
            const Collider& colliderB = colliderItB->second;
            const RigidBody& bodyA = bodyItA->second;
            const RigidBody& bodyB = bodyItB->second;

            if (!colliderA.CanCollideWith(colliderB)) {
                continue;
            }
            if (bodyA.IsStatic() && bodyB.IsStatic()) {
                continue;
            }

            const AABB sweptA = SweptAabb(colliderA, bodyA, maxTime);
            const AABB sweptB = SweptAabb(colliderB, bodyB, maxTime);
            if (!sweptA.Intersects(sweptB)) {
                continue;
            }

            ContactManifold contactAtStart;
            if (GenerateContactAtTime(colliderA, bodyA, colliderB, bodyB, 0.0f, contactAtStart)) {
                best.hit = true;
                best.toi = 0.0f;
                best.bodyA = std::min(idA, idB);
                best.bodyB = std::max(idA, idB);
                best.contact = contactAtStart;
                return best;
            }

            // Temporal sampling catches transient "hit then separate" events in high-speed motion.
            constexpr int kTimeSamples = 48;
            float prevT = 0.0f;
            bool foundPairImpact = false;
            float pairToi = maxTime;
            ContactManifold pairContact;

            for (int s = 1; s <= kTimeSamples; ++s) {
                const float t = maxTime * (static_cast<float>(s) / static_cast<float>(kTimeSamples));
                ContactManifold sampledContact;
                if (!GenerateContactAtTime(colliderA, bodyA, colliderB, bodyB, t, sampledContact)) {
                    prevT = t;
                    continue;
                }

                float lo = prevT;
                float hi = t;
                ContactManifold bestContactForPair = sampledContact;

                for (int iter = 0; iter < 20; ++iter) {
                    const float mid = 0.5f * (lo + hi);
                    ContactManifold midContact;
                    if (GenerateContactAtTime(colliderA, bodyA, colliderB, bodyB, mid, midContact)) {
                        hi = mid;
                        bestContactForPair = midContact;
                    } else {
                        lo = mid;
                    }
                }

                foundPairImpact = true;
                pairToi = hi;
                pairContact = bestContactForPair;
                break;
            }

            if (foundPairImpact && pairToi < best.toi) {
                best.hit = true;
                best.toi = pairToi;
                best.bodyA = std::min(idA, idB);
                best.bodyB = std::max(idA, idB);
                best.contact = pairContact;
            }
        }
    }

    return best;
}

} // namespace Physics
} // namespace Runtime
