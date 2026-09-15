#include "Physics/ContactManifold.h"

#include <array>
#include <cmath>

namespace Runtime {
namespace Physics {

void BuildContactTangentBasis(
    const glm::vec3& normal,
    glm::vec3& outTangent1,
    glm::vec3& outTangent2) {
    // Pick the world axis least aligned with the normal for a stable,
    // deterministic orthonormal basis.
    const glm::vec3 axis =
        (std::abs(normal.x) <= std::abs(normal.y) && std::abs(normal.x) <= std::abs(normal.z))
            ? glm::vec3(1.0f, 0.0f, 0.0f)
            : (std::abs(normal.y) <= std::abs(normal.z) ? glm::vec3(0.0f, 1.0f, 0.0f)
                                                        : glm::vec3(0.0f, 0.0f, 1.0f));
    outTangent1 = glm::normalize(glm::cross(normal, axis));
    outTangent2 = glm::cross(normal, outTangent1);
}

void MatchPersistentContactPoints(
    const ContactManifold& oldManifold,
    ContactManifold& newManifold) {
    if (oldManifold.pointCount == 0 || newManifold.pointCount == 0) {
        return;
    }
    // A significant normal flip invalidates the impulse caches entirely.
    if (glm::dot(oldManifold.normal, newManifold.normal) < kContactNormalCacheMinDot) {
        return;
    }

    const float maxDistanceSq = kContactAnchorMatchDistance * kContactAnchorMatchDistance;
    std::array<bool, ContactManifold::kMaxContactPoints> matched{};

    // Deterministic one-to-one matching: each new point claims the closest
    // unmatched old point by two-sided local anchor distance; exact ties keep
    // the lowest old index (strict < never replaces an equal best).
    for (std::size_t newIndex = 0; newIndex < newManifold.pointCount; ++newIndex) {
        ContactPoint& newPoint = newManifold.Point(newIndex);
        float bestDistanceSq = maxDistanceSq;
        std::size_t bestOldIndex = ContactManifold::kMaxContactPoints;
        for (std::size_t oldIndex = 0; oldIndex < oldManifold.pointCount; ++oldIndex) {
            if (matched[oldIndex]) {
                continue;
            }
            const ContactPoint& oldPoint = oldManifold.Point(oldIndex);
            const glm::vec3 deltaA = newPoint.localPointA - oldPoint.localPointA;
            const glm::vec3 deltaB = newPoint.localPointB - oldPoint.localPointB;
            const float distanceSq = glm::dot(deltaA, deltaA) + glm::dot(deltaB, deltaB);
            if (distanceSq <= maxDistanceSq && distanceSq < bestDistanceSq) {
                bestDistanceSq = distanceSq;
                bestOldIndex = oldIndex;
            }
        }
        if (bestOldIndex == ContactManifold::kMaxContactPoints) {
            continue;
        }
        matched[bestOldIndex] = true;
        const ContactPoint& oldPoint = oldManifold.Point(bestOldIndex);
        newPoint.accumulatedNormalImpulse = oldPoint.accumulatedNormalImpulse;
        newPoint.accumulatedTangentImpulse = oldPoint.accumulatedTangentImpulse;
        // The cache interpretation data travels with the cache: the tangent
        // impulse stays expressed in the basis it was accumulated under until
        // Prepare reprojects it, cachedDt drives the dtNew/cachedDt scaling
        // and the TOI origin flag suppresses warm start replay of impacts.
        newPoint.cachedTangent1 = oldPoint.cachedTangent1;
        newPoint.cachedTangent2 = oldPoint.cachedTangent2;
        newPoint.cachedDt = oldPoint.cachedDt;
        newPoint.cacheFromToiImpact = oldPoint.isToiImpact;
    }
}

} // namespace Physics
} // namespace Runtime
