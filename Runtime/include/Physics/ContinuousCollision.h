#pragma once

#include <cstdint>
#include <unordered_map>

#include "Collider.h"
#include "ContactManifold.h"
#include "RigidBody.h"

namespace Runtime {
namespace Physics {

struct TimeOfImpact {
    bool hit = false;
    float toi = 0.0f;
    uint32_t bodyA = 0;
    uint32_t bodyB = 0;
    ContactManifold contact;
};

class ContinuousCollisionDetector {
public:
    TimeOfImpact FindEarliestImpact(
        const std::unordered_map<uint32_t, Collider>& colliders,
        const std::unordered_map<uint32_t, RigidBody>& bodies,
        float maxTime) const;

private:
    static ShapeTransform InterpolateTransform(const RigidBody& body, float t);
    static AABB SweptAabb(const Collider& collider, const RigidBody& body, float maxTime);

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
