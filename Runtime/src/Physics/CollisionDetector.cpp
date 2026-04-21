#include "Physics/CollisionDetector.h"

namespace Runtime {
namespace Physics {

CollisionDetector::CollisionDetector()
    : m_broadPhase(std::make_unique<DynamicBvhBroadPhase>()),
      m_narrowPhase(std::make_unique<GjkEpaNarrowPhase>()) {}

CollisionDetector::~CollisionDetector() = default;

std::vector<ContactManifold> CollisionDetector::Detect(
    const std::unordered_map<uint32_t, Collider>& colliders,
    const std::unordered_map<uint32_t, RigidBody>& bodies) {
    std::vector<ContactManifold> contacts;
    if (!m_broadPhase || !m_narrowPhase) {
        return contacts;
    }

    const std::vector<BroadPhasePair> candidatePairs = m_broadPhase->ComputePairs(colliders, bodies);
    contacts.reserve(candidatePairs.size());

    for (const auto& pair : candidatePairs) {
        const auto colliderItA = colliders.find(pair.first);
        const auto colliderItB = colliders.find(pair.second);
        const auto bodyItA = bodies.find(pair.first);
        const auto bodyItB = bodies.find(pair.second);

        if (colliderItA == colliders.end() || colliderItB == colliders.end() ||
            bodyItA == bodies.end() || bodyItB == bodies.end()) {
            continue;
        }

        ContactManifold manifold;
        manifold.bodyA = pair.first;
        manifold.bodyB = pair.second;
        manifold.isTrigger = colliderItA->second.IsTrigger() || colliderItB->second.IsTrigger();

        if (m_narrowPhase->GenerateContact(
                colliderItA->second,
                bodyItA->second,
                colliderItB->second,
                bodyItB->second,
                manifold)) {
            contacts.push_back(manifold);
        }
    }

    return contacts;
}

} // namespace Physics
} // namespace Runtime
