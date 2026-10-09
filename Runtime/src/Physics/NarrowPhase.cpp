#include "Physics/NarrowPhase.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace Runtime {
namespace Physics {

namespace {
// Clamp the sphere center in box space: gap=distance-radius outside,
// or -nearestFaceClearance-radius inside; transform witnesses back to world space.
float SphereBoxSeparation(const BoxShape& box, const ShapeTransform& tfBox,
    const SphereShape& sphere, const ShapeTransform& tfSphere,
    glm::vec3& normal, glm::vec3& boxWitness, glm::vec3& sphereWitness) {
    const glm::vec3 center = glm::conjugate(tfBox.orientation) * (tfSphere.position - tfBox.position);
    glm::vec3 closest = glm::clamp(center, -box.HalfExtents(), box.HalfExtents());
    const glm::vec3 delta = center - closest;
    const float distance = glm::length(delta);
    float gap;
    if (distance > 1e-8f) {
        normal = tfBox.orientation * (delta / distance);
        gap = distance - sphere.Radius();
    } else {
        const glm::vec3 clearance = box.HalfExtents() - glm::abs(center);
        int axis = 0;
        if (clearance.y < clearance[axis]) axis = 1;
        if (clearance.z < clearance[axis]) axis = 2;
        glm::vec3 localNormal(0.0f);
        localNormal[axis] = center[axis] < 0.0f ? -1.0f : 1.0f;
        normal = tfBox.orientation * localNormal;
        closest[axis] = localNormal[axis] * box.HalfExtents()[axis];
        gap = -clearance[axis] - sphere.Radius();
    }
    boxWitness = tfBox.position + tfBox.orientation * closest;
    sphereWitness = tfSphere.position - normal * sphere.Radius();
    return gap;
}

// Handle sphere/sphere and sphere/box; sphere gap=|centerB-centerA|-rA-rB.
// Return false for unsupported shape combinations, not for primitive overlap.
bool PrimitiveSeparation(const Collider& a, const ShapeTransform& tfA,
    const Collider& b, const ShapeTransform& tfB, glm::vec3& normal,
    float& gap, glm::vec3& witnessA, glm::vec3& witnessB) {
    const auto* sphereA = dynamic_cast<const SphereShape*>(a.Shape().get());
    const auto* sphereB = dynamic_cast<const SphereShape*>(b.Shape().get());
    const auto* boxA = dynamic_cast<const BoxShape*>(a.Shape().get());
    const auto* boxB = dynamic_cast<const BoxShape*>(b.Shape().get());
    if (sphereA && sphereB) {
        const glm::vec3 delta = tfB.position - tfA.position;
        const float distance = glm::length(delta);
        normal = distance > 1e-8f ? delta / distance : glm::vec3(0.0f, 1.0f, 0.0f);
        gap = distance - sphereA->Radius() - sphereB->Radius();
        witnessA = tfA.position + normal * sphereA->Radius();
        witnessB = tfB.position - normal * sphereB->Radius();
        return true;
    }
    if (boxA && sphereB) {
        gap = SphereBoxSeparation(*boxA, tfA, *sphereB, tfB, normal, witnessA, witnessB);
        return true;
    }
    if (sphereA && boxB) {
        gap = SphereBoxSeparation(*boxB, tfB, *sphereA, tfA, normal, witnessB, witnessA);
        normal = -normal;
        return true;
    }
    return false;
}

// Test the 15 OBB axes; gap = |dot(centerDelta,n)| - radiusA(n) - radiusB(n).
float BoxSeparation(
    const BoxShape& a, const ShapeTransform& tfA,
    const BoxShape& b, const ShapeTransform& tfB,
    glm::vec3& normal, int& edgeA, int& edgeB) {
    const glm::mat3 axesA = glm::mat3_cast(tfA.orientation);
    const glm::mat3 axesB = glm::mat3_cast(tfB.orientation);
    const glm::vec3 delta = tfB.position - tfA.position;
    float best = std::numeric_limits<float>::lowest();
    edgeA = edgeB = -1;
    // Normalize usable axes and maximize the projected gap, preserving face-axis ties.
    const auto testAxis = [&](glm::vec3 axis, int ia, int ib) {
        const float lengthSq = glm::dot(axis, axis);
        if (lengthSq < 1e-10f) return;
        axis /= std::sqrt(lengthSq);
        float radiusA = 0.0f, radiusB = 0.0f;
        for (int i = 0; i < 3; ++i) {
            radiusA += a.HalfExtents()[i] * std::abs(glm::dot(axis, axesA[i]));
            radiusB += b.HalfExtents()[i] * std::abs(glm::dot(axis, axesB[i]));
        }
        const float projection = glm::dot(delta, axis);
        const float gap = std::abs(projection) - radiusA - radiusB;
        if (gap > best + 1e-6f) {
            best = gap;
            normal = projection < 0.0f ? -axis : axis;
            edgeA = ia;
            edgeB = ib;
        }
    };
    for (int i = 0; i < 3; ++i) {
        testAxis(axesA[i], -1, -1);
        testAxis(axesB[i], -1, -1);
    }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            testAxis(glm::cross(axesA[i], axesB[j]), i, j);
    return best;
}

// Return the support edge parallel to the selected OBB axis.
void BoxSupportEdge(const BoxShape& box, const ShapeTransform& tf,
    const glm::vec3& normal, int axis, glm::vec3& p0, glm::vec3& p1) {
    const glm::vec3 localNormal = glm::conjugate(tf.orientation) * normal;
    glm::vec3 point = box.HalfExtents();
    for (int i = 0; i < 3; ++i)
        if (localNormal[i] < 0.0f) point[i] = -point[i];
    point[axis] = -box.HalfExtents()[axis];
    p0 = tf.position + tf.orientation * point;
    point[axis] = box.HalfExtents()[axis];
    p1 = tf.position + tf.orientation * point;
}

// A support direction with two/one/zero zero components selects a face/edge/vertex.
int BoxSupportDimension(const ShapeTransform& tf, const glm::vec3& normal) {
    const glm::vec3 direction = glm::abs(glm::conjugate(tf.orientation) * normal);
    return static_cast<int>(direction.x < 1e-5f) +
        static_cast<int>(direction.y < 1e-5f) + static_cast<int>(direction.z < 1e-5f);
}

// Sort vertices by atan2 in a tangent plane around their centroid.
void SortFace(std::vector<glm::vec3>& face, const glm::vec3& normal) {
    if (face.size() < 3) {
        return;
    }

    glm::vec3 center(0.0f);
    for (const glm::vec3& vertex : face) {
        center += vertex;
    }
    center /= static_cast<float>(face.size());

    const glm::vec3 tangent = glm::normalize(
        std::abs(normal.x) < 0.8f
            ? glm::cross(normal, glm::vec3(1.0f, 0.0f, 0.0f))
            : glm::cross(normal, glm::vec3(0.0f, 1.0f, 0.0f)));
    const glm::vec3 bitangent = glm::cross(normal, tangent);
    // Compare polar angles in the same tangent/bitangent basis.
    std::sort(face.begin(), face.end(), [&](const glm::vec3& lhs, const glm::vec3& rhs) {
        const glm::vec3 lhsOffset = lhs - center;
        const glm::vec3 rhsOffset = rhs - center;
        const float lhsAngle = std::atan2(glm::dot(lhsOffset, bitangent), glm::dot(lhsOffset, tangent));
        const float rhsAngle = std::atan2(glm::dot(rhsOffset, bitangent), glm::dot(rhsOffset, tangent));
        return lhsAngle < rhsAngle;
    });
}

// Classify and intersect against the same tolerance-shifted plane, so 0 <= edge fraction <= 1.
std::vector<glm::vec3> ClipPolygonAgainstPlane(
    const std::vector<glm::vec3>& polygon,
    const glm::vec3& planePoint,
    const glm::vec3& planeNormal) {
    std::vector<glm::vec3> clipped;
    if (polygon.empty()) {
        return clipped;
    }

    glm::vec3 previous = polygon.back();
    float previousDistance = glm::dot(planeNormal, previous - planePoint) - 1e-5f;
    for (const glm::vec3& current : polygon) {
        const float currentDistance = glm::dot(planeNormal, current - planePoint) - 1e-5f;
        const bool previousInside = previousDistance <= 0.0f;
        const bool currentInside = currentDistance <= 0.0f;
        if (previousInside != currentInside) {
            const float denominator = previousDistance - currentDistance;
            clipped.push_back(previous + (current - previous) * (previousDistance / denominator));
        }
        if (currentInside) {
            clipped.push_back(current);
        }
        previous = current;
        previousDistance = currentDistance;
    }
    return clipped;
}

// Keep at most four candidates using depth-weighted spread and opposite-side area extremes.
void AddReducedContactPoints(
    const std::vector<ContactPoint>& candidates,
    const glm::vec3& normal,
    ContactManifold& manifold) {
    if (candidates.size() <= ContactManifold::kMaxContactPoints) {
        for (const ContactPoint& candidate : candidates) {
            manifold.AddPoint(candidate);
        }
        return;
    }

    // Seed selection favors distance from the centroid times squared signed penetration.
    glm::vec3 centroid(0.0f);
    for (const ContactPoint& c : candidates) {
        centroid += c.position;
    }
    centroid /= static_cast<float>(candidates.size());

    // weight_i = max(eps, |p_i - centroid|^2) * max(eps, penetration_i^2)
    const auto weight = [&](const ContactPoint& c) {
        const glm::vec3 d = c.position - centroid;
        const float distSq = std::max(glm::dot(d, d), 1e-6f);
        const float depthSq = std::max(c.penetration * c.penetration, 1e-6f);
        return distSq * depthSq;
    };

    const std::size_t n = candidates.size();
    std::vector<bool> kept(n, false);

    // Point 1: highest weight.
    std::size_t p1 = 0;
    float best = weight(candidates[0]);
    for (std::size_t i = 1; i < n; ++i) {
        const float w = weight(candidates[i]);
        if (w > best) { best = w; p1 = i; }
    }
    kept[p1] = true;

    // Point 2: furthest from point 1 (weighted by depth).
    std::size_t p2 = n;
    best = -1.0f;
    for (std::size_t i = 0; i < n; ++i) {
        if (kept[i]) continue;
        const glm::vec3 d = candidates[i].position - candidates[p1].position;
        const float w = std::max(glm::dot(d, d), 1e-6f) *
            std::max(candidates[i].penetration * candidates[i].penetration, 1e-6f);
        if (w > best) { best = w; p2 = i; }
    }
    if (p2 < n) kept[p2] = true;

    // Points 3 & 4: furthest on both sides of the p1-p2 line (maximize area).
    const glm::vec3 axis = candidates[p2].position - candidates[p1].position;
    const glm::vec3 perp = glm::cross(axis, normal);
    std::size_t p3 = n, p4 = n;
    float minSide = 0.0f, maxSide = 0.0f;
    for (std::size_t i = 0; i < n; ++i) {
        if (kept[i]) continue;
        const float side = glm::dot(perp, candidates[i].position - candidates[p1].position);
        if (side < minSide) { minSide = side; p3 = i; }
        else if (side > maxSide) { maxSide = side; p4 = i; }
    }
    if (p3 < n) kept[p3] = true;
    if (p4 < n) kept[p4] = true;

    for (std::size_t i = 0; i < n; ++i) {
        if (kept[i]) manifold.AddPoint(candidates[i]);
    }
}

// Solve line parameters with denominator=aa*bb-ab^2, clamp to [0,1],
// then readjust A after clamping B; degenerate lengths use endpoint zero.
void ClosestPointsOnSegments(
    const glm::vec3& a0,
    const glm::vec3& a1,
    const glm::vec3& b0,
    const glm::vec3& b1,
    glm::vec3& closestA,
    glm::vec3& closestB) {
    const glm::vec3 edgeA = a1 - a0;
    const glm::vec3 edgeB = b1 - b0;
    const glm::vec3 offset = a0 - b0;
    const float aa = glm::dot(edgeA, edgeA);
    const float bb = glm::dot(edgeB, edgeB);
    const float ab = glm::dot(edgeA, edgeB);
    const float ar = glm::dot(edgeA, offset);
    const float br = glm::dot(edgeB, offset);
    const float denominator = aa * bb - ab * ab;
    float parameterA = denominator > 1e-8f ? std::clamp((ab * br - bb * ar) / denominator, 0.0f, 1.0f) : 0.0f;
    float parameterB = bb > 1e-8f ? std::clamp((ab * parameterA + br) / bb, 0.0f, 1.0f) : 0.0f;
    if (aa > 1e-8f) {
        parameterA = std::clamp((ab * parameterB - ar) / aa, 0.0f, 1.0f);
    }
    closestA = a0 + parameterA * edgeA;
    closestB = b0 + parameterB * edgeB;
}

// Closest simplex feature with source indices and barycentric weights for compaction.
struct OriginFeature {
    glm::vec3 closest = glm::vec3(0.0f); // closest simplex point to origin
    int index[4] = {0, 0, 0, 0};         // simplex slots carrying weight
    float weight[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    int count = 1;
    bool containsOrigin = false; // origin inside a non-degenerate tetrahedron
};

// Represent one vertex with unit barycentric weight.
OriginFeature MakeVertexFeature(const glm::vec3& point, int index) {
    OriginFeature feature;
    feature.closest = point;
    feature.index[0] = index;
    return feature;
}

// Edge a-b with closest = a + t(b-a), weights (1-t, t); t in (0, 1).
OriginFeature MakeEdgeFeature(const glm::vec3& a, const glm::vec3& b, int ia, int ib, float t) {
    OriginFeature feature;
    feature.count = 2;
    feature.index[0] = ia;
    feature.index[1] = ib;
    feature.weight[0] = 1.0f - t;
    feature.weight[1] = t;
    feature.closest = a + t * (b - a);
    return feature;
}

// Segment: t = clamp(-a.(b-a) / |b-a|^2, 0, 1).
OriginFeature ProjectSegmentOrigin(const glm::vec3& a, const glm::vec3& b) {
    const glm::vec3 edge = b - a;
    const float lengthSq = glm::dot(edge, edge);
    if (lengthSq <= 1e-12f) {
        return MakeVertexFeature(a, 0);
    }
    const float t = glm::clamp(-glm::dot(a, edge) / lengthSq, 0.0f, 1.0f);
    if (t <= 0.0f) {
        return MakeVertexFeature(a, 0);
    }
    if (t >= 1.0f) {
        return MakeVertexFeature(b, 1);
    }
    return MakeEdgeFeature(a, b, 0, 1, t);
}

// Triangle: Voronoi region tests; interior weights are the barycentric
// sub-triangle ratios w1 = vb/(va+vb+vc), w2 = vc/(va+vb+vc).
OriginFeature ProjectTriangleOrigin(
    const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2,
    int i0, int i1, int i2) {
    const glm::vec3 ab = p1 - p0;
    const glm::vec3 ac = p2 - p0;
    const glm::vec3 ao = -p0;
    const float d1 = glm::dot(ab, ao);
    const float d2 = glm::dot(ac, ao);
    if (d1 <= 0.0f && d2 <= 0.0f) {
        return MakeVertexFeature(p0, i0);
    }
    const glm::vec3 bo = -p1;
    const float d3 = glm::dot(ab, bo);
    const float d4 = glm::dot(ac, bo);
    if (d3 >= 0.0f && d4 <= d3) {
        return MakeVertexFeature(p1, i1);
    }
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        return MakeEdgeFeature(p0, p1, i0, i1, d1 / (d1 - d3));
    }
    const glm::vec3 co = -p2;
    const float d5 = glm::dot(ab, co);
    const float d6 = glm::dot(ac, co);
    if (d6 >= 0.0f && d5 <= d6) {
        return MakeVertexFeature(p2, i2);
    }
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        return MakeEdgeFeature(p0, p2, i0, i2, d2 / (d2 - d6));
    }
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        return MakeEdgeFeature(p1, p2, i1, i2, (d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    const float denominator = 1.0f / (va + vb + vc);
    const float w1 = vb * denominator;
    const float w2 = vc * denominator;
    OriginFeature feature;
    feature.count = 3;
    feature.index[0] = i0;
    feature.index[1] = i1;
    feature.index[2] = i2;
    feature.weight[0] = 1.0f - w1 - w2;
    feature.weight[1] = w1;
    feature.weight[2] = w2;
    feature.closest = p0 + ab * w1 + ac * w2;
    return feature;
}

// Project onto the first face whose origin/opposite-vertex signs differ.
// Flat tetrahedra choose the closest face; otherwise report containment.
OriginFeature ProjectTetrahedronOrigin(
    const glm::vec3& p0, const glm::vec3& p1,
    const glm::vec3& p2, const glm::vec3& p3) {
    const glm::vec3 points[4] = {p0, p1, p2, p3};
    const int faces[4][4] = {
        {0, 1, 2, 3}, {0, 2, 3, 1}, {0, 3, 1, 2}, {1, 2, 3, 0},
    };
    float maxEdgeSq = 0.0f;
    for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 4; ++j) {
            const glm::vec3 edge = points[j] - points[i];
            maxEdgeSq = std::max(maxEdgeSq, glm::dot(edge, edge));
        }
    }
    for (const auto& face : faces) {
        const glm::vec3 faceNormal = glm::cross(
            points[face[1]] - points[face[0]], points[face[2]] - points[face[0]]);
        const float sideOpposite = glm::dot(faceNormal, points[face[3]] - points[face[0]]);
        const float sideOrigin = glm::dot(faceNormal, -points[face[0]]);
        if (sideOrigin * sideOpposite < 0.0f) {
            return ProjectTriangleOrigin(
                points[face[0]], points[face[1]], points[face[2]],
                face[0], face[1], face[2]);
        }
    }
    // Degenerate (flat) tetrahedron: no strict outside test fired; fall back
    // to the closest face projection instead of claiming containment.
    const float volume = glm::dot(glm::cross(p1 - p0, p2 - p0), p3 - p0);
    const float volumeEpsilon = 1e-8f * std::max(1.0f, maxEdgeSq * std::sqrt(maxEdgeSq));
    if (std::abs(volume) <= volumeEpsilon) {
        OriginFeature best;
        float bestDistanceSq = std::numeric_limits<float>::max();
        for (const auto& face : faces) {
            OriginFeature candidate = ProjectTriangleOrigin(
                points[face[0]], points[face[1]], points[face[2]],
                face[0], face[1], face[2]);
            const float distanceSq = glm::dot(candidate.closest, candidate.closest);
            if (distanceSq < bestDistanceSq) {
                bestDistanceSq = distanceSq;
                best = candidate;
            }
        }
        return best;
    }
    OriginFeature feature;
    feature.containsOrigin = true;
    return feature;
}

// Find an oriented plane even when a hull's supporting feature starts with collinear vertices.
bool GetFacePlaneNormal(const SupportFeature& face, const glm::vec3& outward, glm::vec3& normal) {
    for (std::size_t i = 1; i + 1 < face.vertices.size(); ++i) {
        for (std::size_t j = i + 1; j < face.vertices.size(); ++j) {
            const glm::vec3 axis = glm::cross(
                face.vertices[i] - face.vertices[0], face.vertices[j] - face.vertices[0]);
            const float lengthSq = glm::dot(axis, axis);
            if (lengthSq > 1e-12f) {
                normal = axis / std::sqrt(lengthSq);
                if (glm::dot(normal, outward) < 0.0f) normal = -normal;
                return true;
            }
        }
    }
    return false;
}

// Clip incident features along the collision normal onto the actual reference face plane.
bool BuildFeatureManifold(
    const Collider& colliderA,
    const ShapeTransform& tfA,
    const Collider& colliderB,
    const ShapeTransform& tfB,
    const glm::vec3& normal,
    ContactManifold& manifold,
    float maxSeparation) {
    SupportFeature featureA = colliderA.Shape()->GetSupportFeature(tfA, normal);
    SupportFeature featureB = colliderB.Shape()->GetSupportFeature(tfB, -normal);
    if (featureA.vertices.empty() || featureB.vertices.empty()) {
        return false;
    }

    if (featureA.type == SupportFeatureType::Edge && featureB.type == SupportFeatureType::Edge) {
        glm::vec3 closestA;
        glm::vec3 closestB;
        ClosestPointsOnSegments(
            featureA.vertices[0], featureA.vertices[1],
            featureB.vertices[0], featureB.vertices[1],
            closestA, closestB);
        // Signed separation along the normal: negative penetration = gap.
        const float signedPenetration = glm::dot(closestA - closestB, normal);
        if (signedPenetration < -maxSeparation) {
            return false; // beyond the accepted band; caller falls back
        }
        manifold.ClearPoints();
        ContactPoint point;
        point.position = 0.5f * (closestA + closestB);
        point.surfacePointA = closestA;
        point.surfacePointB = closestB;
        point.penetration = signedPenetration;
        manifold.AddPoint(point);
        return true;
    }

    glm::vec3 faceNormalA, faceNormalB;
    const bool aIsFace = featureA.type == SupportFeatureType::Face &&
        GetFacePlaneNormal(featureA, normal, faceNormalA);
    const bool bIsFace = featureB.type == SupportFeatureType::Face &&
        GetFacePlaneNormal(featureB, -normal, faceNormalB);
    if (!aIsFace && !bIsFace) {
        return false;
    }

    const bool useAAsReference = aIsFace &&
        (!bIsFace || glm::dot(faceNormalA, normal) >= glm::dot(faceNormalB, -normal));
    manifold.normalOnB = !useAAsReference;
    std::vector<glm::vec3> reference = useAAsReference
        ? std::move(featureA.vertices)
        : std::move(featureB.vertices);
    std::vector<glm::vec3> incident = useAAsReference
        ? std::move(featureB.vertices)
        : std::move(featureA.vertices);
    const glm::vec3 referenceNormal = useAAsReference ? normal : -normal;
    const glm::vec3 planeNormal = useAAsReference ? faceNormalA : faceNormalB;
    SortFace(reference, referenceNormal);
    if (incident.size() >= 3) {
        SortFace(incident, -referenceNormal);
    }

    for (std::size_t index = 0; index < reference.size() && !incident.empty(); ++index) {
        const glm::vec3& edgeStart = reference[index];
        const glm::vec3& edgeEnd = reference[(index + 1) % reference.size()];
        const glm::vec3 sideNormal = glm::cross(edgeEnd - edgeStart, referenceNormal);
        if (glm::dot(sideNormal, sideNormal) > 1e-10f) {
            incident = ClipPolygonAgainstPlane(incident, edgeStart, glm::normalize(sideNormal));
        }
    }

    std::vector<ContactPoint> candidates;
    candidates.reserve(incident.size());
    const glm::vec3 referencePoint = reference.front();
    for (const glm::vec3& incidentPoint : incident) {
        // Intersect the ray p - s*n with the real reference plane:
        // s = dot(p - p0, faceNormal) / dot(n, faceNormal).
        const float separation = glm::dot(incidentPoint - referencePoint, planeNormal) /
            glm::dot(referenceNormal, planeNormal);
        // Accept incident points up to maxSeparation in FRONT of the
        // reference plane (speculative band); penetration stays signed:
        //   penetration = -separation   (negative = remaining gap)
        if (separation <= maxSeparation) {
            ContactPoint point;
            point.position = incidentPoint - 0.5f * separation * referenceNormal;
            const glm::vec3 referenceSurface = incidentPoint - separation * referenceNormal;
            point.surfacePointA = useAAsReference ? referenceSurface : incidentPoint;
            point.surfacePointB = useAAsReference ? incidentPoint : referenceSurface;
            point.penetration = -separation;
            candidates.push_back(point);
        }
    }

    if (candidates.empty()) {
        return false;
    }

    manifold.ClearPoints();
    AddReducedContactPoints(candidates, normal, manifold);
    return manifold.pointCount > 0;
}

// Use sphere shell points along the normal: midpoint for two spheres,
// one shell point for a single sphere, otherwise retain the supplied point.
glm::vec3 RefineContactPointForSpheres(
    const Collider& colliderA,
    const ShapeTransform& tfA,
    const Collider& colliderB,
    const ShapeTransform& tfB,
    const glm::vec3& normal,
    const glm::vec3& defaultPoint) {
    const auto* sphereA = dynamic_cast<const SphereShape*>(colliderA.Shape().get());
    const auto* sphereB = dynamic_cast<const SphereShape*>(colliderB.Shape().get());

    if (!sphereA && !sphereB) {
        return defaultPoint;
    }

    if (sphereA && sphereB) {
        const glm::vec3 pa = tfA.position + normal * sphereA->Radius();
        const glm::vec3 pb = tfB.position - normal * sphereB->Radius();
        return 0.5f * (pa + pb);
    }

    if (sphereA) {
        return tfA.position + normal * sphereA->Radius();
    }

    return tfB.position - normal * sphereB->Radius();
}

} // namespace

// Query primitive pairs directly; retain GJK/EPA for general convex geometry.
bool GjkEpaNarrowPhase::GenerateContact(
    const Collider& colliderA,
    const RigidBody& bodyA,
    const Collider& colliderB,
    const RigidBody& bodyB,
    ContactManifold& outContact,
    NarrowPhaseQueryStats& outStats) const {
    outStats = {};
    ShapeTransform tfA;
    tfA.position = bodyA.Position();
    tfA.orientation = bodyA.Orientation();

    ShapeTransform tfB;
    tfB.position = bodyB.Position();
    tfB.orientation = bodyB.Orientation();

    // Inflate only A's rejection bounds by the configured speculative band.
    AABB boundsA = colliderA.ComputeAABB(tfA);
    if (m_speculativeContactDistance > 0.0f) {
        boundsA = boundsA.Expanded(m_speculativeContactDistance);
    }
    if (!boundsA.Intersects(colliderB.ComputeAABB(tfB))) {
        return false;
    }

    ContactPoint primitivePoint;
    float primitiveGap = 0.0f;
    if (PrimitiveSeparation(colliderA, tfA, colliderB, tfB, outContact.normal,
        primitiveGap, primitivePoint.surfacePointA, primitivePoint.surfacePointB)) {
        ++outStats.primitiveCallCount;
        if (primitiveGap > m_speculativeContactDistance) return false;
        primitivePoint.penetration = -primitiveGap;
        primitivePoint.position = 0.5f * (primitivePoint.surfacePointA + primitivePoint.surfacePointB);
        outContact.ClearPoints();
        outContact.AddPoint(primitivePoint);
        return true;
    }
    const auto* boxA = dynamic_cast<const BoxShape*>(colliderA.Shape().get());
    const auto* boxB = dynamic_cast<const BoxShape*>(colliderB.Shape().get());
    if (boxA && boxB) {
        ++outStats.satCallCount;
        int edgeA, edgeB;
        const float gap = BoxSeparation(*boxA, tfA, *boxB, tfB, outContact.normal, edgeA, edgeB);
        if (gap > m_speculativeContactDistance) return false;
        outContact.ClearPoints();
        if (edgeA >= 0) {
            outContact.topology = ContactTopology::EdgeEdge;
            glm::vec3 a0, a1, b0, b1;
            BoxSupportEdge(*boxA, tfA, outContact.normal, edgeA, a0, a1);
            BoxSupportEdge(*boxB, tfB, -outContact.normal, edgeB, b0, b1);
            ContactPoint point;
            ClosestPointsOnSegments(a0, a1, b0, b1, point.surfacePointA, point.surfacePointB);
            point.position = 0.5f * (point.surfacePointA + point.surfacePointB);
            point.penetration = -gap;
            outContact.AddPoint(point);
            return true;
        }
        const int dimension = std::min(
            BoxSupportDimension(tfA, outContact.normal), BoxSupportDimension(tfB, outContact.normal));
        outContact.topology = dimension == 2 ? ContactTopology::FaceFace :
            dimension == 1 ? ContactTopology::FaceEdge : ContactTopology::FaceVertex;
        if (BuildFeatureManifold(colliderA, tfA, colliderB, tfB, outContact.normal,
            outContact, std::max(1e-5f, m_speculativeContactDistance))) return true;
        // Near a corner the face polygons may not overlap yet; use actual distance witnesses.
        GjkDistanceResult distance;
        ++outStats.gjkCallCount;
        if (!RunGjkDistance(colliderA, tfA, colliderB, tfB,
            m_speculativeContactDistance, distance)) return false;
        ContactPoint point;
        point.surfacePointA = distance.witnessA;
        point.surfacePointB = distance.witnessB;
        point.position = 0.5f * (distance.witnessA + distance.witnessB);
        point.penetration = -distance.gap;
        outContact.normal = distance.normal;
        outContact.AddPoint(point);
        return true;
    }

    Simplex simplex;
    ++outStats.gjkCallCount;
    const QueryResult gjkResult = RunGjk(colliderA, tfA, colliderB, tfB, simplex);
    if (gjkResult == QueryResult::Separated) {
        // Separated: emit a speculative contact when the surfaces are within
        // the band (a velocity-level pre-contact, penetration stays < 0).
        return GenerateSpeculativeContact(colliderA, tfA, colliderB, tfB, outContact, outStats);
    }
    if (gjkResult == QueryResult::Failed) {
        ++outStats.gjkFailureCount;
        return false;
    }

    EpaResult epa;
    ++outStats.epaCallCount;
    const QueryResult epaResult = RunEpa(colliderA, tfA, colliderB, tfB, simplex, epa);
    if (epaResult == QueryResult::Separated) {
        return false;
    }
    if (epaResult == QueryResult::Failed) {
        ++outStats.epaFailureCount;
        return false;
    }

    outContact.normal = epa.normal;
    outContact.ClearPoints();
    ContactPoint primaryPoint;
    primaryPoint.penetration = epa.penetration;
    primaryPoint.surfacePointA = epa.witnessA;
    primaryPoint.surfacePointB = epa.witnessB;
    primaryPoint.position = RefineContactPointForSpheres(
        colliderA,
        tfA,
        colliderB,
        tfB,
        outContact.normal,
        epa.contactPoint);
    outContact.AddPoint(primaryPoint);
    // A tilted penetrating contact also accepts incident points up to the
    // speculative band in front of the reference plane: they carry negative
    // penetration and act as anti-tilt speculative support.
    BuildFeatureManifold(
        colliderA, tfA, colliderB, tfB, outContact.normal, outContact,
        std::max(1e-4f, m_speculativeContactDistance));
    return true;
}

// Clamp nonpositive distances to zero.
void GjkEpaNarrowPhase::SetSpeculativeContactDistance(float distance) {
    m_speculativeContactDistance = distance > 0.0f ? distance : 0.0f;
}

// SAT supplies a conservative OBB separating plane; other convex pairs use GJK witnesses.
bool GjkEpaNarrowPhase::GetSeparation(
    const Collider& colliderA, const ShapeTransform& tfA,
    const Collider& colliderB, const ShapeTransform& tfB,
    glm::vec3& normal, float& gap) const {
    glm::vec3 witnessA, witnessB;
    if (PrimitiveSeparation(colliderA, tfA, colliderB, tfB, normal, gap, witnessA, witnessB))
        return true;
    const auto* boxA = dynamic_cast<const BoxShape*>(colliderA.Shape().get());
    const auto* boxB = dynamic_cast<const BoxShape*>(colliderB.Shape().get());
    if (boxA && boxB) {
        int edgeA, edgeB;
        gap = BoxSeparation(*boxA, tfA, *boxB, tfB, normal, edgeA, edgeB);
        if (gap > 0.0f && gap < 0.002f) {
            GjkDistanceResult distance;
            // Refine the SAT lower bound near skew corners; an unconverged distance
            // leaves the independently valid SAT plane in place.
            if (RunGjkDistance(colliderA, tfA, colliderB, tfB,
                std::numeric_limits<float>::max(), distance)) {
                const float planeGap = glm::dot(
                    colliderB.Shape()->Support(tfB, -distance.normal) -
                    colliderA.Shape()->Support(tfA, distance.normal), distance.normal);
                if (planeGap > gap) {
                    gap = planeGap;
                    normal = distance.normal;
                }
            }
        }
        return true;
    }
    GjkDistanceResult distance;
    if (!RunGjkDistance(colliderA, tfA, colliderB, tfB,
        std::numeric_limits<float>::max(), distance)) return false;
    normal = distance.normal;
    gap = distance.gap;
    return true;
}

// Project onto the size-specific feature and compact witnesses/weights to its source indices.
glm::vec3 GjkEpaNarrowPhase::ProjectOriginOntoSimplex(Simplex& simplex, float* outWeights) const {
    OriginFeature feature;
    switch (simplex.size()) {
    case 1:
        feature = MakeVertexFeature(simplex[0].point, 0);
        break;
    case 2:
        feature = ProjectSegmentOrigin(simplex[0].point, simplex[1].point);
        break;
    case 3:
        feature = ProjectTriangleOrigin(
            simplex[0].point, simplex[1].point, simplex[2].point, 0, 1, 2);
        break;
    default:
        feature = ProjectTetrahedronOrigin(
            simplex[0].point, simplex[1].point, simplex[2].point, simplex[3].point);
        break;
    }
    // Compact the simplex to the feature's support vertices; weights stay
    // index-aligned with the compacted slots.
    SupportPoint kept[4];
    for (int index = 0; index < feature.count; ++index) {
        kept[index] = simplex[feature.index[index]];
        outWeights[index] = feature.weight[index];
    }
    simplex.assign(kept, kept + feature.count);
    return feature.closest;
}

// Refine closest v with support(-v/|v|); stop on duplicates or converged upper/lower gap bounds.
// Reject touching, an out-of-band lower bound, or iteration exhaustion.
bool GjkEpaNarrowPhase::RunGjkDistance(
    const Collider& a,
    const ShapeTransform& tfA,
    const Collider& b,
    const ShapeTransform& tfB,
    float maxDistance,
    GjkDistanceResult& out) const {
    Simplex simplex;
    glm::vec3 direction = tfB.position - tfA.position;
    if (glm::dot(direction, direction) < kEpsilon) {
        direction = glm::vec3(1.0f, 0.0f, 0.0f);
    }
    simplex.push_back(Support(a, tfA, b, tfB, direction));

    float weights[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    for (int iteration = 0; iteration < kMaxGjkIterations; ++iteration) {
        // v: closest Minkowski-difference point to the origin seen so far.
        const glm::vec3 v = ProjectOriginOntoSimplex(simplex, weights);
        const float distanceSq = glm::dot(v, v);
        if (distanceSq < kEpsilon * kEpsilon) {
            // Touching within numeric tolerance: the penetrating path (or the
            // next step) owns this pair, no speculative contact needed.
            return false;
        }
        const float distance = std::sqrt(distanceSq);
        const glm::vec3 search = -v / distance;

        const SupportPoint next = Support(a, tfA, b, tfB, search);
        // Gap bounds from support extremality and Cauchy-Schwarz:
        //   dot(S(-v̂), v̂) <= gap <= |v|
        const float lowerBound = glm::dot(next.point, -search);
        if (lowerBound > maxDistance) {
            return false; // provably beyond the speculative band: early out
        }
        bool duplicate = false;
        for (const SupportPoint& existing : simplex) {
            const glm::vec3 delta = existing.point - next.point;
            duplicate = duplicate || glm::dot(delta, delta) < kEpsilon * kEpsilon;
        }
        if (duplicate || distance - lowerBound <= kEpsilon * std::max(1.0f, distance)) {
            // Converged. Witnesses from the barycentric weights:
            //   w = sum_i weights_i * point_i   (per shape)
            // and dot(witnessA - witnessB, normal) = -gap with normal = -v̂.
            out.normal = search;
            out.gap = distance;
            out.witnessA = glm::vec3(0.0f);
            out.witnessB = glm::vec3(0.0f);
            for (std::size_t index = 0; index < simplex.size(); ++index) {
                out.witnessA += weights[index] * simplex[index].pointA;
                out.witnessB += weights[index] * simplex[index].pointB;
            }
            return true;
        }
        simplex.push_back(next);
    }
    return false;
}

// Build a signed-gap feature patch in the enabled band, falling back to one witness pair.
bool GjkEpaNarrowPhase::GenerateSpeculativeContact(
    const Collider& colliderA,
    const ShapeTransform& tfA,
    const Collider& colliderB,
    const ShapeTransform& tfB,
    ContactManifold& outContact,
    NarrowPhaseQueryStats& outStats) const {
    if (m_speculativeContactDistance <= 0.0f) {
        return false;
    }
    GjkDistanceResult closest;
    ++outStats.gjkCallCount;
    if (!RunGjkDistance(colliderA, tfA, colliderB, tfB, m_speculativeContactDistance, closest)) {
        return false;
    }

    outContact.normal = closest.normal;
    outContact.ClearPoints();
    // Face/edge clipping reuses the penetrating path; only the acceptance
    // band widens from touching (1e-4) to the full speculative distance.
    if (BuildFeatureManifold(
            colliderA, tfA, colliderB, tfB, closest.normal, outContact,
            m_speculativeContactDistance) &&
        outContact.pointCount > 0) {
        return true;
    }
    // Vertex-ish fallback: the single witness pair.
    ContactPoint point;
    point.position = 0.5f * (closest.witnessA + closest.witnessB);
    point.surfacePointA = closest.witnessA;
    point.surfacePointB = closest.witnessB;
    point.penetration = -closest.gap;
    outContact.ClearPoints();
    outContact.AddPoint(point);
    return true;
}

// Compute supportA(d)-supportB(-d), retaining both original witnesses.
GjkEpaNarrowPhase::SupportPoint GjkEpaNarrowPhase::Support(
    const Collider& a,
    const ShapeTransform& tfA,
    const Collider& b,
    const ShapeTransform& tfB,
    const glm::vec3& direction) const {
    SupportPoint p;
    p.pointA = a.Support(tfA, direction);
    p.pointB = b.Support(tfB, -direction);
    p.point = p.pointA - p.pointB;
    return p;
}

// Search support simplices for the origin; dot(support,d)<=0 rejects,
// near-zero direction accepts, and 32 exhausted iterations report failure.
GjkEpaNarrowPhase::QueryResult GjkEpaNarrowPhase::RunGjk(
    const Collider& a,
    const ShapeTransform& tfA,
    const Collider& b,
    const ShapeTransform& tfB,
    Simplex& simplex) const {
    simplex.clear();

    glm::vec3 direction = tfB.position - tfA.position;
    if (glm::dot(direction, direction) < kEpsilon) {
        direction = glm::vec3(1.0f, 0.0f, 0.0f);
    }

    simplex.push_back(Support(a, tfA, b, tfB, direction));
    direction = -simplex.back().point;

    for (int i = 0; i < kMaxGjkIterations; ++i) {
        if (glm::dot(direction, direction) < kEpsilon) {
            return QueryResult::Intersecting;
        }

        SupportPoint newPoint = Support(a, tfA, b, tfB, direction);
        if (glm::dot(newPoint.point, direction) <= 0.0f) {
            return QueryResult::Separated;
        }

        simplex.push_back(newPoint);
        if (UpdateSimplex(simplex, direction)) {
            return QueryResult::Intersecting;
        }

        if (glm::dot(direction, direction) < kEpsilon) {
            return QueryResult::Intersecting;
        }
    }

    return QueryResult::Failed;
}

// Route the current simplex cardinality to the corresponding containment update.
bool GjkEpaNarrowPhase::UpdateSimplex(Simplex& simplex, glm::vec3& direction) const {
    switch (simplex.size()) {
    case 2:
        return HandleLine(simplex, direction);
    case 3:
        return HandleTriangle(simplex, direction);
    case 4:
        return HandleTetrahedron(simplex, direction);
    default:
        break;
    }
    return false;
}

// Use (AB cross AO) cross AB for edge search, a perpendicular on degeneracy,
// or retain only A when the origin lies behind it.
bool GjkEpaNarrowPhase::HandleLine(Simplex& simplex, glm::vec3& direction) const {
    const glm::vec3 a = simplex[1].point;
    const glm::vec3 b = simplex[0].point;
    const glm::vec3 ab = b - a;
    const glm::vec3 ao = -a;

    if (glm::dot(ab, ao) > 0.0f) {
        direction = glm::cross(glm::cross(ab, ao), ab);
        if (glm::dot(direction, direction) < kEpsilon) {
            direction = glm::normalize(
                glm::abs(ab.x) > 0.5f ? glm::vec3(-ab.y, ab.x, 0.0f) : glm::vec3(0.0f, -ab.z, ab.y));
        }
    } else {
        simplex = {simplex[1]};
        direction = ao;
    }

    return false;
}

// Reduce outside AB/AC half-spaces; otherwise orient the face normal toward the origin.
bool GjkEpaNarrowPhase::HandleTriangle(Simplex& simplex, glm::vec3& direction) const {
    const glm::vec3 a = simplex[2].point;
    const glm::vec3 b = simplex[1].point;
    const glm::vec3 c = simplex[0].point;
    const glm::vec3 ab = b - a;
    const glm::vec3 ac = c - a;
    const glm::vec3 ao = -a;

    // Triangle normal defines the two half-spaces above/below the face.
    glm::vec3 abc = glm::cross(ab, ac);

    // Test whether the origin lies outside edge AB.
    glm::vec3 abPerp = glm::cross(ab, abc);
    if (glm::dot(abPerp, ao) > 0.0f) {
        // Keep edge AB and continue with the line-case search direction.
        simplex = {simplex[1], simplex[2]};
        direction = glm::cross(glm::cross(ab, ao), ab);
        return false;
    }

    // Test whether the origin lies outside edge AC.
    glm::vec3 acPerp = glm::cross(abc, ac);
    if (glm::dot(acPerp, ao) > 0.0f) {
        // Keep edge AC and continue with the line-case search direction.
        simplex = {simplex[0], simplex[2]};
        direction = glm::cross(glm::cross(ac, ao), ac);
        return false;
    }

    // Origin projects inside the triangle prism; search along face normal.
    if (glm::dot(abc, ao) > 0.0f) {
        direction = abc;
    } else {
        // Flip winding so normal stays consistent with the new direction.
        direction = -abc;
        std::swap(simplex[0], simplex[1]);
    }
    return false;
}

// Test ABC, ACD, and ADB at the newest vertex A; a positive plane test keeps that face.
bool GjkEpaNarrowPhase::HandleTetrahedron(Simplex& simplex, glm::vec3& direction) const {
    const glm::vec3 a = simplex[3].point;
    const glm::vec3 b = simplex[2].point;
    const glm::vec3 c = simplex[1].point;
    const glm::vec3 d = simplex[0].point;
    const glm::vec3 ao = -a;

    // If origin is outside face ABC, reduce simplex to that face.
    const glm::vec3 abc = glm::cross(b - a, c - a);
    if (glm::dot(abc, ao) > 0.0f) {
        simplex = {simplex[1], simplex[2], simplex[3]};
        direction = abc;
        return false;
    }

    // If origin is outside face ACD, reduce simplex to that face.
    const glm::vec3 acd = glm::cross(c - a, d - a);
    if (glm::dot(acd, ao) > 0.0f) {
        simplex = {simplex[0], simplex[1], simplex[3]};
        direction = acd;
        return false;
    }

    // If origin is outside face ADB, reduce simplex to that face.
    const glm::vec3 adb = glm::cross(d - a, b - a);
    if (glm::dot(adb, ao) > 0.0f) {
        simplex = {simplex[2], simplex[0], simplex[3]};
        direction = adb;
        return false;
    }

    // Origin is inside tetrahedron => Minkowski difference contains origin.
    return true;
}

// Normalize (B-A) cross (C-A), flip winding for dot(n,A)>=0, and reject degenerate planes.
GjkEpaNarrowPhase::EpaFace GjkEpaNarrowPhase::BuildFace(
    const std::vector<SupportPoint>& vertices,
    int a,
    int b,
    int c) const {
    EpaFace face;
    face.a = a;
    face.b = b;
    face.c = c;

    const glm::vec3 pa = vertices[a].point;
    const glm::vec3 pb = vertices[b].point;
    const glm::vec3 pc = vertices[c].point;
    glm::vec3 n = glm::cross(pb - pa, pc - pa);
    if (glm::dot(n, n) < kEpsilon) {
        return face;
    }

    n = glm::normalize(n);
    if (glm::dot(n, pa) < 0.0f) {
        n = -n;
        std::swap(face.b, face.c);
    }

    face.normal = n;
    face.distance = glm::dot(n, pa);
    face.valid = std::isfinite(face.distance);
    return face;
}

// Project origin to n*distance; D=d00*d11-d01^2,
// wB=(d11*d20-d01*d21)/D, wC=(d00*d21-d01*d20)/D, wA=1-wB-wC.
// Interpolate paired witnesses with these weights and return their midpoint.
bool GjkEpaNarrowPhase::BuildEpaResult(
    const std::vector<SupportPoint>& vertices,
    const EpaFace& face,
    EpaResult& out) const {
    if (!face.valid) {
        return false;
    }

    const glm::vec3& a = vertices[face.a].point;
    const glm::vec3& b = vertices[face.b].point;
    const glm::vec3& c = vertices[face.c].point;
    const glm::vec3 closest = face.normal * face.distance;
    const glm::vec3 v0 = b - a;
    const glm::vec3 v1 = c - a;
    const glm::vec3 v2 = closest - a;
    const float d00 = glm::dot(v0, v0);
    const float d01 = glm::dot(v0, v1);
    const float d11 = glm::dot(v1, v1);
    const float d20 = glm::dot(v2, v0);
    const float d21 = glm::dot(v2, v1);
    const float denominator = d00 * d11 - d01 * d01;
    if (std::abs(denominator) < kEpsilon) {
        return false;
    }

    const float weightB = (d11 * d20 - d01 * d21) / denominator;
    const float weightC = (d00 * d21 - d01 * d20) / denominator;
    const float weightA = 1.0f - weightB - weightC;
    const glm::vec3 witnessA =
        weightA * vertices[face.a].pointA +
        weightB * vertices[face.b].pointA +
        weightC * vertices[face.c].pointA;
    const glm::vec3 witnessB =
        weightA * vertices[face.a].pointB +
        weightB * vertices[face.b].pointB +
        weightC * vertices[face.c].pointB;

    out.normal = face.normal;
    out.penetration = std::max(face.distance, 0.0f);
    out.witnessA = witnessA;
    out.witnessB = witnessB;
    out.contactPoint = 0.5f * (witnessA + witnessB);
    return true;
}

// Expand the closest polytope face until support-plane improvement<=1e-4 or duplicate support.
// Rebuild visible-face horizons; insufficient simplex or resource/iteration limits fail.
GjkEpaNarrowPhase::QueryResult GjkEpaNarrowPhase::RunEpa(
    const Collider& a,
    const ShapeTransform& tfA,
    const Collider& b,
    const ShapeTransform& tfB,
    const Simplex& simplex,
    EpaResult& out) const {
    if (simplex.size() < 4) {
        return QueryResult::Failed;
    }

    std::vector<SupportPoint> vertices = simplex;
    std::vector<EpaFace> faces;
    faces.reserve(16);
    // Add only finite, nondegenerate outward faces to the initial tetrahedron.
    auto addFace = [&](int faceA, int faceB, int faceC) {
        EpaFace face = BuildFace(vertices, faceA, faceB, faceC);
        if (face.valid) {
            faces.push_back(face);
        }
    };
    addFace(0, 1, 2);
    addFace(0, 3, 1);
    addFace(0, 2, 3);
    addFace(1, 3, 2);
    if (faces.size() < 4) {
        return QueryResult::Failed;
    }

    for (int iter = 0; iter < kMaxEpaIterations; ++iter) {
        // Pick the face whose supporting plane is currently closest to the origin.
        int bestFace = -1;
        float minDistance = std::numeric_limits<float>::max();
        for (int i = 0; i < static_cast<int>(faces.size()); ++i) {
            if (faces[i].valid && faces[i].distance < minDistance) {
                minDistance = faces[i].distance;
                bestFace = i;
            }
        }

        if (bestFace < 0) {
            return QueryResult::Failed;
        }

        const EpaFace& face = faces[bestFace];
        SupportPoint p = Support(a, tfA, b, tfB, face.normal);
        const float distance = glm::dot(face.normal, p.point);

        bool duplicate = false;
        for (const SupportPoint& vertex : vertices) {
            const glm::vec3 delta = p.point - vertex.point;
            if (glm::dot(delta, delta) <= kEpsilon * kEpsilon) {
                duplicate = true;
                break;
            }
        }

        // If pushing along the best-face normal no longer expands meaningfully,
        // treat this face as converged and export its normal/penetration.
        if (duplicate || distance - face.distance <= 1e-4f) {
            return BuildEpaResult(vertices, face, out)
                ? QueryResult::Intersecting
                : QueryResult::Failed;
        }

        if (vertices.size() >= kMaxEpaVertices) {
            return QueryResult::Failed;
        }

        const int newIndex = static_cast<int>(vertices.size());
        vertices.push_back(p);

        // Collect horizon edges: edges shared by one removed face remain,
        // while opposite duplicate edges cancel out.
        std::vector<std::array<int, 2>> boundary;
        boundary.reserve(24);

        for (int i = static_cast<int>(faces.size()) - 1; i >= 0; --i) {
            const EpaFace& f = faces[i];
            const glm::vec3 pa = vertices[f.a].point;
            // A face is visible if the new point lies in front of its plane.
            if (glm::dot(f.normal, p.point - pa) <= 0.0f) {
                continue;
            }

            // Cancel reverse directed edges shared by removed faces, leaving the horizon.
            auto addOrRemoveEdge = [&boundary](int e0, int e1) {
                for (size_t k = 0; k < boundary.size(); ++k) {
                    if (boundary[k][0] == e1 && boundary[k][1] == e0) {
                        boundary.erase(boundary.begin() + static_cast<std::ptrdiff_t>(k));
                        return;
                    }
                }
                boundary.push_back({e0, e1});
            };

            addOrRemoveEdge(f.a, f.b);
            addOrRemoveEdge(f.b, f.c);
            addOrRemoveEdge(f.c, f.a);

            // Remove visible faces; they form the hole to be re-triangulated.
            faces.erase(faces.begin() + static_cast<std::ptrdiff_t>(i));
        }

        // Stitch the hole by connecting each boundary edge to the new support point.
        for (const auto& e : boundary) {
            if (faces.size() >= kMaxEpaFaces) {
                return QueryResult::Failed;
            }
            EpaFace newFace = BuildFace(vertices, e[0], e[1], newIndex);
            if (newFace.valid) {
                faces.push_back(newFace);
            }
        }
        if (faces.empty()) {
            return QueryResult::Failed;
        }
    }

    return QueryResult::Failed;
}

} // namespace Physics
} // namespace Runtime
