#pragma once

#include <cstdint>
#include <unordered_map>

#include "Collider.h"
#include "ContactManifold.h"
#include "RigidBody.h"

namespace Runtime {
namespace Physics {

struct TimeOfImpact {
    // Whether an impact is found in [0, maxTime].
    bool hit = false;
    // Earliest impact time from interval start.
    float toi = 0.0f;
    uint32_t bodyA = 0;
    uint32_t bodyB = 0;
    // Contact generated at impact time.
    ContactManifold contact;
};

class ContinuousCollisionDetector {
public:
    // Search earliest impact for all collider pairs within the time window.
    TimeOfImpact FindEarliestImpact(
        const std::unordered_map<uint32_t, Collider>& colliders,
        const std::unordered_map<uint32_t, RigidBody>& bodies,
        float maxTime) const;

private:
    // Approximate body transform at time t using current velocities.
    static ShapeTransform InterpolateTransform(const RigidBody& body, float t);
    // Build swept bounds from t=0 to t=maxTime for quick rejection.
    static AABB SweptAabb(const Collider& collider, const RigidBody& body, float maxTime);

    // Generate contact for a specific pair at a specific time sample.
    bool GenerateContactAtTime(
        const Collider& colliderA,
        const RigidBody& bodyA,
        const Collider& colliderB,
        const RigidBody& bodyB,
        float t,
        ContactManifold& outContact) const;
};

} // namespace Physics
} // namespace Runtime
