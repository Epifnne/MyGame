#include "Physics/ContinuousCollision.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "Physics/Midphase.h"
#include "Physics/NarrowPhase.h"

namespace Runtime {
namespace Physics {
namespace {
constexpr float kCastContactTolerance = 0.0002f;
constexpr int kMaxCastIterations = 512;

// Bound every surface point relative to the body's rotation center, including offset hulls.
float RadiusAboutBody(const AABB& bounds, const glm::vec3& center) {
    return glm::length(glm::max(glm::abs(bounds.min - center), glm::abs(bounds.max - center)));
}
}

// Match pose integration: q(t) = normalize(q + 0.5*t*(omega*q)).
ShapeTransform ContinuousCollisionDetector::InterpolateTransform(const RigidBody& body, float t) {
    const glm::vec3 w = body.AngularVelocity();
    return {body.Position() + t * body.LinearVelocity(),
        glm::normalize(body.Orientation() +
            0.5f * t * glm::quat(0.0f, w.x, w.y, w.z) * body.Orientation())};
}

// Bound translation and all intermediate rotations by the tip travel |omega|*r*dt.
AABB ContinuousCollisionDetector::SweptAabb(
    const Collider& collider, const RigidBody& body, float maxTime) {
    const AABB initial = collider.ComputeAABB(InterpolateTransform(body, 0.0f));
    const AABB final = collider.ComputeAABB(InterpolateTransform(body, maxTime));
    const float radius = RadiusAboutBody(initial, body.Position());
    return AABB::Merge(initial, final).Expanded(
        glm::length(body.AngularVelocity()) * radius * maxTime);
}

// Build an impact patch inside the numerical TOI tolerance, without applying forces.
bool ContinuousCollisionDetector::GenerateContactAtTime(
    const Collider& colliderA, const RigidBody& bodyA,
    const Collider& colliderB, const RigidBody& bodyB,
    float t, ContactManifold& outContact) const {
    RigidBody a = bodyA, b = bodyB;
    const ShapeTransform tfA = InterpolateTransform(bodyA, t);
    const ShapeTransform tfB = InterpolateTransform(bodyB, t);
    a.SetPosition(tfA.position);
    a.SetOrientation(tfA.orientation);
    b.SetPosition(tfB.position);
    b.SetOrientation(tfB.orientation);
    GjkEpaNarrowPhase narrow;
    narrow.SetSpeculativeContactDistance(kCastContactTolerance);
    NarrowPhaseQueryStats stats;
    const bool hit = narrow.GenerateContact(colliderA, a, colliderB, b, outContact, stats);
    if (stats.gjkFailureCount || stats.epaFailureCount)
        throw std::runtime_error("CCD impact manifold query failed");
    return hit;
}

// Search swept candidates, advance quadratic gap bounds, and deterministically break equal-time ties.
TimeOfImpact ContinuousCollisionDetector::FindEarliestImpact(
    const std::unordered_map<uint32_t, Collider>& colliders,
    const std::unordered_map<uint32_t, RigidBody>& bodies,
    const Midphase& midphase, float maxTime, float motionThreshold) const {
    TimeOfImpact best;
    best.toi = maxTime;
    if (maxTime <= 0.0f || colliders.size() < 2) return best;

    for (uint32_t id : m_sweptTree.CollectBodyIds())
        if (colliders.find(id) == colliders.end() || bodies.find(id) == bodies.end())
            m_sweptTree.RemoveLeaf(id);

    std::vector<uint32_t> fast;
    for (const auto& [id, collider] : colliders) {
        const auto it = bodies.find(id);
        if (it == bodies.end()) continue;
        const RigidBody& body = it->second;
        const AABB tight = collider.ComputeAABB(InterpolateTransform(body, 0.0f));
        const glm::vec3 extent = 0.5f * (tight.max - tight.min);
        float innerRadius = std::min({extent.x, extent.y, extent.z});
        if (const auto* box = dynamic_cast<const BoxShape*>(collider.Shape().get()))
            innerRadius = std::min({box->HalfExtents().x, box->HalfExtents().y, box->HalfExtents().z});
        const float travel = maxTime * (glm::length(body.LinearVelocity()) +
            glm::length(body.AngularVelocity()) * RadiusAboutBody(tight, body.Position()));
        const float cutoff = motionThreshold > 0.0f ?
            std::min(0.5f * innerRadius, motionThreshold) : 0.5f * innerRadius;
        if (!body.IsStatic() && !body.IsSleeping() && travel > cutoff)
            fast.push_back(id);
        m_sweptTree.UpsertLeaf(id, SweptAabb(collider, body, maxTime));
    }
    std::sort(fast.begin(), fast.end());
    GjkEpaNarrowPhase narrow;
    for (uint32_t idA : fast) {
        const Collider& a = colliders.at(idA);
        const RigidBody& bodyA = bodies.at(idA);
        const AABB region = SweptAabb(a, bodyA, maxTime);
        // Sweep each unordered fast-body pair once; masks filter candidates before advancement.
        m_sweptTree.QueryLeafOverlaps(region, [&](uint32_t idB) {
            if (idA == idB || (idB < idA && std::binary_search(fast.begin(), fast.end(), idB))) return;
            const Collider& b = colliders.at(idB);
            const RigidBody& bodyB = bodies.at(idB);
            if (!a.CanCollideWith(b)) return;
            // Bound surface distance from the rotation center using the initial AABB corners.
            const auto radius = [&](const Collider& collider, const RigidBody& body) {
                const AABB bounds = collider.ComputeAABB(InterpolateTransform(body, 0.0f));
                return RadiusAboutBody(bounds, body.Position());
            };
            const float radiusA = radius(a, bodyA);
            const float radiusB = radius(b, bodyB);
            float time = 0.0f;
            float lastGap = 0.0f;
            for (int iteration = 0; iteration < kMaxCastIterations; ++iteration) {
                glm::vec3 normal;
                float gap = 0.0f;
                const ShapeTransform tfA = InterpolateTransform(bodyA, time);
                const ShapeTransform tfB = InterpolateTransform(bodyB, time);
                const bool separated = narrow.GetSeparation(a, tfA, b, tfB, normal, gap);
                lastGap = gap;
                if (!separated || gap <= kCastContactTolerance) {
                    const MidphasePair* pair = midphase.FindPair(MakePairKey(idA, idB));
                    if (time == 0.0f && pair && pair->hadContact && separated) return;
                    ContactManifold contact;
                    if (GenerateContactAtTime(a, bodyA, b, bodyB, time, contact)) {
                        // An initial overlap belongs to discrete detection, not a zero-time impact.
                        if (time > 0.0f && (time < best.toi ||
                            (time == best.toi && MakePairKey(idA, idB) < MakePairKey(best.bodyA, best.bodyB)))) {
                            best.hit = true;
                            best.toi = time;
                            best.bodyA = std::min(idA, idB);
                            best.bodyB = std::max(idA, idB);
                            if (idA > idB) {
                                contact.normal = -contact.normal;
                                contact.normalOnB = !contact.normalOnB;
                                for (std::size_t p = 0; p < contact.pointCount; ++p)
                                    std::swap(contact.Point(p).surfacePointA, contact.Point(p).surfacePointB);
                            }
                            contact.bodyA = best.bodyA;
                            contact.bodyB = best.bodyB;
                            best.contact = contact;
                        }
                        return;
                    }
                    if (!separated) throw std::runtime_error("CCD separation query failed");
                }
                // Bound the changing support plane: gap(t+h) >= gap - closing*h - a*h^2/2.
                const glm::vec3 axisA = glm::cross(normal, bodyA.AngularVelocity());
                const glm::vec3 axisB = glm::cross(-normal, bodyB.AngularVelocity());
                const float closing = glm::dot(bodyA.LinearVelocity() - bodyB.LinearVelocity(), normal) +
                    glm::dot(a.Shape()->Support(tfA, axisA) - tfA.position, axisA) +
                    glm::dot(b.Shape()->Support(tfB, axisB) - tfB.position, axisB);
                const float curvature = glm::length(axisA) * glm::length(bodyA.AngularVelocity()) * radiusA +
                    glm::length(axisB) * glm::length(bodyB.AngularVelocity()) * radiusB;
                const float distance = std::max(gap - 0.5f * kCastContactTolerance, 0.00001f);
                float advance;
                // Solve closing*h+curvature*h^2/2=distance; rationalize for positive closing.
                if (curvature > 1e-6f) {
                    const float root = std::sqrt(closing * closing + 2.0f * curvature * distance);
                    advance = closing > 0.0f ? 2.0f * distance / (closing + root) :
                        (root - closing) / curvature;
                } else {
                    if (closing <= 1e-6f) return;
                    advance = distance / closing;
                }
                time += advance;
                if (time > best.toi) return;
            }
            throw std::runtime_error("CCD conservative advancement did not converge: pair=" +
                std::to_string(idA) + "-" + std::to_string(idB) + " time=" + std::to_string(time) +
                " gap=" + std::to_string(lastGap) + " horizon=" + std::to_string(best.toi));
        });
    }
    return best;
}

} // namespace Physics
} // namespace Runtime
