#pragma once

#include <cstdint>
#include <unordered_map>

#include "Collider.h"
#include "ContactManifold.h"
#include "RigidBody.h"
#include "BvhTree.h"

namespace Runtime {
namespace Physics {

class Midphase;

// Earliest positive-time impact patch with sorted body IDs; misses retain the search horizon.
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

// Swept-BVH candidate search followed by conservative advancement with rotational bounds.
class ContinuousCollisionDetector {
public:
    // Search pairs reached by fast awake dynamic bodies within the time window.
    // Initial overlaps are owned by the discrete solver. Separated speculative
    // pairs are still swept: their current patch may miss a new rotational feature.
    // Gate travel by min(half the smallest half extent,motionThreshold), if threshold>0.
    // Zero uses only the shape-size cutoff; this is independent of the speculative band.
    TimeOfImpact FindEarliestImpact(
        const std::unordered_map<uint32_t, Collider>& colliders,
        const std::unordered_map<uint32_t, RigidBody>& bodies,
        const Midphase& midphase,
        float maxTime, float motionThreshold = 0.02f) const;

private:
    // Refit swept leaves per window; fast bodies query this tree instead of all pairs.
    mutable BvhTree m_sweptTree;
    // Use p(t)=p+v*t and q(t)=normalize(q+0.5*t*(omega*q)).
    static ShapeTransform InterpolateTransform(const RigidBody& body, float t);
    // Merge endpoint bounds and pad by |omega|*body-relative radius*maxTime.
    static AABB SweptAabb(const Collider& collider, const RigidBody& body, float maxTime);

    // Query copied bodies at time t with a 0.0002 contact band; throw on GJK/EPA failure.
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
