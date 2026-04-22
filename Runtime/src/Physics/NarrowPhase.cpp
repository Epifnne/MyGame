#include "Physics/NarrowPhase.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace Runtime {
namespace Physics {

namespace {

bool BuildFallbackContact(
    const Collider& colliderA,
    const ShapeTransform& tfA,
    const RigidBody& bodyA,
    const Collider& colliderB,
    const ShapeTransform& tfB,
    const RigidBody& bodyB,
    ContactManifold& outContact) {
    const AABB a = colliderA.ComputeAABB(tfA);
    const AABB b = colliderB.ComputeAABB(tfB);
    if (!a.Intersects(b)) {
        return false;
    }

    const float overlapX = std::min(a.max.x, b.max.x) - std::max(a.min.x, b.min.x);
    const float overlapY = std::min(a.max.y, b.max.y) - std::max(a.min.y, b.min.y);
    const float overlapZ = std::min(a.max.z, b.max.z) - std::max(a.min.z, b.min.z);
    if (overlapX <= 0.0f || overlapY <= 0.0f || overlapZ <= 0.0f) {
        return false;
    }

    glm::vec3 normal = tfB.position - tfA.position;
    if (glm::dot(normal, normal) < 1e-8f) {
        normal = glm::vec3(0.0f, 1.0f, 0.0f);
    } else {
        normal = glm::normalize(normal);
    }

    const glm::vec3 relativeVelocity = bodyB.LinearVelocity() - bodyA.LinearVelocity();
    if (glm::dot(relativeVelocity, normal) > 0.0f) {
        normal = -normal;
    }

    outContact.normal = normal;
    outContact.point.penetration = std::min(overlapX, std::min(overlapY, overlapZ));
    outContact.point.position = 0.5f * (tfA.position + tfB.position);
    outContact.point.normalImpulse = 0.0f;
    return true;
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
    ContactManifold& outContact) const {
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
    const QueryResult gjkResult = RunGjk(colliderA, tfA, colliderB, tfB, simplex);
    if (gjkResult == QueryResult::Separated) {
        return false;
    }
    if (gjkResult == QueryResult::Failed) {
        return BuildFallbackContact(colliderA, tfA, bodyA, colliderB, tfB, bodyB, outContact);
    }

    EpaResult epa;
    const QueryResult epaResult = RunEpa(colliderA, tfA, colliderB, tfB, simplex, epa);
    if (epaResult == QueryResult::Separated) {
        return false;
    }
    if (epaResult == QueryResult::Failed) {
        return BuildFallbackContact(colliderA, tfA, bodyA, colliderB, tfB, bodyB, outContact);
    }

    outContact.normal = epa.normal;
    outContact.point.penetration = epa.penetration;
    outContact.point.position = RefineContactPointForSpheres(
        colliderA,
        tfA,
        colliderB,
        tfB,
        outContact.normal,
        epa.contactPoint);
    outContact.point.normalImpulse = 0.0f;
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
    glm::vec3 abPerp = glm::cross(abc, ab);
    if (glm::dot(abPerp, ao) > 0.0f) {
        // Keep edge AB and continue with the line-case search direction.
        simplex = {simplex[1], simplex[2]};
        direction = glm::cross(glm::cross(ab, ao), ab);
        return false;
    }

    // Test whether the origin lies outside edge AC.
    glm::vec3 acPerp = glm::cross(ac, abc);
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
        face.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        face.distance = 0.0f;
        return face;
    }

    n = glm::normalize(n);
    if (glm::dot(n, pa) < 0.0f) {
        n = -n;
        std::swap(face.b, face.c);
    }

    face.normal = n;
    face.distance = glm::dot(n, pa);
    return face;
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
    faces.push_back(BuildFace(vertices, 0, 1, 2));
    faces.push_back(BuildFace(vertices, 0, 3, 1));
    faces.push_back(BuildFace(vertices, 0, 2, 3));
    faces.push_back(BuildFace(vertices, 1, 3, 2));

    for (int iter = 0; iter < kMaxEpaIterations; ++iter) {
        // Pick the face whose supporting plane is currently closest to the origin.
        int bestFace = -1;
        float minDistance = std::numeric_limits<float>::max();
        for (int i = 0; i < static_cast<int>(faces.size()); ++i) {
            if (faces[i].distance < minDistance) {
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

        // If pushing along the best-face normal no longer expands meaningfully,
        // treat this face as converged and export its normal/penetration.
        if (distance - face.distance <= 1e-4f) {
            out.normal = glm::normalize(face.normal);
            out.penetration = std::max(distance, 0.0f);
            out.contactPoint = 0.5f * (p.pointA + p.pointB);
            return QueryResult::Intersecting;
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
            faces.push_back(BuildFace(vertices, e[0], e[1], newIndex));
        }
    }

    return QueryResult::Failed;
}

} // namespace Physics
} // namespace Runtime
