#include "Physics/NarrowPhase.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace Runtime {
namespace Physics {

namespace {
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
    std::sort(face.begin(), face.end(), [&](const glm::vec3& lhs, const glm::vec3& rhs) {
        const glm::vec3 lhsOffset = lhs - center;
        const glm::vec3 rhsOffset = rhs - center;
        const float lhsAngle = std::atan2(glm::dot(lhsOffset, bitangent), glm::dot(lhsOffset, tangent));
        const float rhsAngle = std::atan2(glm::dot(rhsOffset, bitangent), glm::dot(rhsOffset, tangent));
        return lhsAngle < rhsAngle;
    });
}

std::vector<glm::vec3> ClipPolygonAgainstPlane(
    const std::vector<glm::vec3>& polygon,
    const glm::vec3& planePoint,
    const glm::vec3& planeNormal) {
    std::vector<glm::vec3> clipped;
    if (polygon.empty()) {
        return clipped;
    }

    glm::vec3 previous = polygon.back();
    float previousDistance = glm::dot(planeNormal, previous - planePoint);
    for (const glm::vec3& current : polygon) {
        const float currentDistance = glm::dot(planeNormal, current - planePoint);
        const bool previousInside = previousDistance <= 1e-5f;
        const bool currentInside = currentDistance <= 1e-5f;
        if (previousInside != currentInside) {
            const float denominator = previousDistance - currentDistance;
            if (std::abs(denominator) > 1e-8f) {
                clipped.push_back(previous + (current - previous) * (previousDistance / denominator));
            }
        }
        if (currentInside) {
            clipped.push_back(current);
        }
        previous = current;
        previousDistance = currentDistance;
    }
    return clipped;
}

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

    const glm::vec3 tangent = glm::normalize(
        std::abs(normal.x) < 0.8f
            ? glm::cross(normal, glm::vec3(1.0f, 0.0f, 0.0f))
            : glm::cross(normal, glm::vec3(0.0f, 1.0f, 0.0f)));
    const glm::vec3 bitangent = glm::cross(normal, tangent);
    const glm::vec3 directions[] = {tangent, -tangent, bitangent, -bitangent};
    for (const glm::vec3& direction : directions) {
        const ContactPoint* best = nullptr;
        float bestProjection = std::numeric_limits<float>::lowest();
        for (const ContactPoint& candidate : candidates) {
            const float projection = glm::dot(candidate.position, direction);
            if (projection > bestProjection) {
                bestProjection = projection;
                best = &candidate;
            }
        }
        bool duplicate = false;
        for (std::size_t index = 0; best && index < manifold.pointCount; ++index) {
            const glm::vec3 delta = manifold.Point(index).position - best->position;
            duplicate = duplicate || glm::dot(delta, delta) <= 1e-8f;
        }
        if (best && !duplicate) {
            manifold.AddPoint(*best);
        }
    }
}

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

bool BuildFeatureManifold(
    const Collider& colliderA,
    const ShapeTransform& tfA,
    const Collider& colliderB,
    const ShapeTransform& tfB,
    const glm::vec3& normal,
    ContactManifold& manifold) {
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
        manifold.ClearPoints();
        ContactPoint point;
        point.position = 0.5f * (closestA + closestB);
        point.penetration = std::max(glm::dot(closestA - closestB, normal), 0.0f);
        manifold.AddPoint(point);
        return true;
    }

    const bool aIsFace = featureA.type == SupportFeatureType::Face;
    const bool bIsFace = featureB.type == SupportFeatureType::Face;
    if (!aIsFace && !bIsFace) {
        return false;
    }

    const bool useAAsReference = aIsFace && (!bIsFace || featureA.vertices.size() >= featureB.vertices.size());
    std::vector<glm::vec3> reference = useAAsReference
        ? std::move(featureA.vertices)
        : std::move(featureB.vertices);
    std::vector<glm::vec3> incident = useAAsReference
        ? std::move(featureB.vertices)
        : std::move(featureA.vertices);
    const glm::vec3 referenceNormal = useAAsReference ? normal : -normal;
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
        const float separation = glm::dot(incidentPoint - referencePoint, referenceNormal);
        if (separation <= 1e-4f) {
            ContactPoint point;
            point.position = incidentPoint - 0.5f * separation * referenceNormal;
            point.penetration = std::max(-separation, 0.0f);
            candidates.push_back(point);
        }
    }

    if (candidates.empty()) {
        return false;
    }

    glm::vec3 centroid(0.0f);
    float deepestPenetration = 0.0f;
    for (const ContactPoint& candidate : candidates) {
        centroid += candidate.position;
        deepestPenetration = std::max(deepestPenetration, candidate.penetration);
    }
    manifold.ClearPoints();
    AddReducedContactPoints(candidates, normal, manifold);
    centroid /= static_cast<float>(candidates.size());

    // The first point doubles as the manifold's representative contact: it
    // carries the deepest penetration and the candidates' centroid. Under the
    // sequential-impulse solver the zero-angular-arm centroid acts as the
    // load-distributing sink that keeps stacked configurations stable at low
    // iteration counts; Phase 4 warm start relies on this behavior until a
    // block solver or stable feature IDs can balance per-point loads.
    if (manifold.pointCount > 0) {
        ContactPoint& primary = manifold.Point(0);
        primary.position = centroid;
        primary.penetration = deepestPenetration;
    }
    return manifold.pointCount > 0;
}

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

    if (!colliderA.ComputeAABB(tfA).Intersects(colliderB.ComputeAABB(tfB))) {
        return false;
    }

    Simplex simplex;
    ++outStats.gjkCallCount;
    const QueryResult gjkResult = RunGjk(colliderA, tfA, colliderB, tfB, simplex);
    if (gjkResult == QueryResult::Separated) {
        return false;
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
    primaryPoint.position = RefineContactPointForSpheres(
        colliderA,
        tfA,
        colliderB,
        tfB,
        outContact.normal,
        epa.contactPoint);
    outContact.AddPoint(primaryPoint);
    BuildFeatureManifold(colliderA, tfA, colliderB, tfB, outContact.normal, outContact);
    return true;
}

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
    out.contactPoint = 0.5f * (witnessA + witnessB);
    return true;
}

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
