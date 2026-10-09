#include "Physics/ContactManifold.h"

#include <array>
#include <cmath>

namespace Runtime {
namespace Physics {

// Choose the least-aligned world axis; for unit n, t1 = normalize(n cross axis), t2 = n cross t1.
void BuildContactTangentBasis(
    const glm::vec3& normal,
    glm::vec3& outTangent1,
    glm::vec3& outTangent2) {
    const glm::vec3 axis =
        (std::abs(normal.x) <= std::abs(normal.y) && std::abs(normal.x) <= std::abs(normal.z))
            ? glm::vec3(1.0f, 0.0f, 0.0f)
            : (std::abs(normal.y) <= std::abs(normal.z) ? glm::vec3(0.0f, 1.0f, 0.0f)
                                                        : glm::vec3(0.0f, 0.0f, 1.0f));
    outTangent1 = glm::normalize(glm::cross(normal, axis));
    outTangent2 = glm::cross(normal, outTangent1);
}

// Greedily match new anchors to unused old anchors by |dA|^2 + |dB|^2 < 0.01^2; ties favor old order.
// Transfer impulse/basis/dt only; incompatible normals and unmatched points leave new data unchanged.
void MatchPersistentContactPoints(
    const ContactManifold& oldManifold,
    ContactManifold& newManifold) {
    if (oldManifold.pointCount == 0 || newManifold.pointCount == 0) {
        return;
    }
    // Incompatible normals suppress transfer; this function does not clear existing new caches.
    if (glm::dot(oldManifold.normal, newManifold.normal) < kContactNormalCacheMinDot) {
        return;
    }

    const float maxDistanceSq = kContactAnchorMatchDistance * kContactAnchorMatchDistance;
    std::array<bool, ContactManifold::kMaxContactPoints> matched{};

    // The strict best-distance comparison also excludes points exactly at the threshold.
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
        newPoint.accumulatedSpinImpulse = oldPoint.accumulatedSpinImpulse;
        // Preserve the old basis and dt so Prepare can reproject and rescale the impulse.
        newPoint.cachedTangent1 = oldPoint.cachedTangent1;
        newPoint.cachedTangent2 = oldPoint.cachedTangent2;
        newPoint.cachedDt = oldPoint.cachedDt;
    }
}

} // namespace Physics
} // namespace Runtime
