#pragma once

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace Runtime {
namespace Physics {

// Axis-aligned inclusive bounds used by overlap tests and tree heuristics.
struct AABB {
	glm::vec3 min = glm::vec3(0.0f);
	glm::vec3 max = glm::vec3(0.0f);

	// Test overlap between two axis-aligned bounding boxes.
	bool Intersects(const AABB& other) const {
		return min.x <= other.max.x && max.x >= other.min.x &&
			   min.y <= other.max.y && max.y >= other.min.y &&
			   min.z <= other.max.z && max.z >= other.min.z;
	}

	// Return minimal AABB that encloses both inputs.
	static AABB Merge(const AABB& a, const AABB& b) {
		return {
			glm::min(a.min, b.min),
			glm::max(a.max, b.max)
		};
	}

	// Area=2*(xy+yz+zx), with negative extents clamped to zero.
	float SurfaceArea() const {
		const glm::vec3 ext = glm::max(max - min, glm::vec3(0.0f));
		return 2.0f * (ext.x * ext.y + ext.y * ext.z + ext.z * ext.x);
	}

	// Inflate bounds by a positive margin on all axes.
	AABB Expanded(float margin) const {
		const glm::vec3 pad(std::max(0.0f, margin));
		return {min - pad, max + pad};
	}
};

// Rigid world pose, without scale, supplied to shape queries.
struct ShapeTransform {
	// World-space position and orientation of the collision shape.
	glm::vec3 position = glm::vec3(0.0f);
	glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
};

enum class SupportFeatureType {
	Vertex,
	Edge,
	Face,
};

// World vertices and a cardinality-based feature kind for manifold clipping.
struct SupportFeature {
	SupportFeatureType type = SupportFeatureType::Vertex;
	std::vector<glm::vec3> vertices;
};

// Convex-query interface for world bounds, support points, and supporting features.
class CollisionShape {
public:
	// Destroy concrete geometry safely through the base interface.
	virtual ~CollisionShape() = default;
	// Compute world-space AABB for broad-phase usage.
	virtual AABB ComputeAABB(const ShapeTransform& transform) const = 0;
	// Return furthest point along direction in world-space.
	virtual glm::vec3 Support(const ShapeTransform& transform, const glm::vec3& direction) const = 0;
	// Default to a single world-space support vertex; derived shapes may return larger features.
	virtual SupportFeature GetSupportFeature(
		const ShapeTransform& transform,
		const glm::vec3& direction) const {
		return {SupportFeatureType::Vertex, {Support(transform, direction)}};
	}
};

// Centered sphere whose queries ignore orientation.
class SphereShape final : public CollisionShape {
public:
	// Radius is clamped to non-negative values.
	explicit SphereShape(float radius) : m_radius(std::max(0.0f, radius)) {}

	// Sphere radius accessor.
	float Radius() const { return m_radius; }

	// Compute sphere bounds around center.
	AABB ComputeAABB(const ShapeTransform& transform) const override {
		const glm::vec3 extents(m_radius);
		return {transform.position - extents, transform.position + extents};
	}

	// Return center+r*d/|d|, or center when dot(d,d)<=1e-12.
	glm::vec3 Support(const ShapeTransform& transform, const glm::vec3& direction) const override {
		const float len2 = glm::dot(direction, direction);
		if (len2 <= 1e-12f) {
			return transform.position;
		}
		return transform.position + glm::normalize(direction) * m_radius;
	}

private:
	float m_radius = 0.5f;
};

// Origin-centered box represented by nonnegative local half extents.
class BoxShape final : public CollisionShape {
public:
	// Half extents are clamped per-axis to non-negative values.
	explicit BoxShape(const glm::vec3& halfExtents)
		: m_halfExtents(glm::max(halfExtents, glm::vec3(0.0f))) {}

	// Local half extents accessor.
	const glm::vec3& HalfExtents() const { return m_halfExtents; }

	// World half extents are |R|*halfExtents for the normalized orientation R.
	AABB ComputeAABB(const ShapeTransform& transform) const override {
		const glm::mat3 r = glm::mat3_cast(glm::normalize(transform.orientation));
		const glm::mat3 absR = glm::mat3(
			glm::abs(r[0]),
			glm::abs(r[1]),
			glm::abs(r[2]));
		const glm::vec3 extents = absR * m_halfExtents;
		return {transform.position - extents, transform.position + extents};
	}

	// Rotate direction by inverse(q), choose +/-halfExtent by each component's sign,
	// then return position+q*localVertex using normalized q.
	glm::vec3 Support(const ShapeTransform& transform, const glm::vec3& direction) const override {
		const glm::quat q = glm::normalize(transform.orientation);
		const glm::vec3 localDir = glm::inverse(q) * direction;
		const glm::vec3 localSupport(
			localDir.x >= 0.0f ? m_halfExtents.x : -m_halfExtents.x,
			localDir.y >= 0.0f ? m_halfExtents.y : -m_halfExtents.y,
			localDir.z >= 0.0f ? m_halfExtents.z : -m_halfExtents.z);
		return transform.position + (q * localSupport);
	}

	// Return four world vertices of the face selected by the dominant local direction.
	SupportFeature GetSupportFeature(
		const ShapeTransform& transform,
		const glm::vec3& direction) const override {
		const glm::quat orientation = glm::normalize(transform.orientation);
		const glm::vec3 localDirection = glm::inverse(orientation) * direction;

		// Always return a full face, avoiding feature collapse from small direction noise.
		const glm::vec3 absDir = glm::abs(localDirection);
		int axis = 0;
		if (absDir.y > absDir.x && absDir.y >= absDir.z) {
			axis = 1;
		} else if (absDir.z > absDir.x && absDir.z >= absDir.y) {
			axis = 2;
		}
		const float sign = localDirection[axis] >= 0.0f ? 1.0f : -1.0f;

		// The 4 vertices of the dominant face: fixed [axis], free others.
		SupportFeature feature;
		feature.type = SupportFeatureType::Face;
		const int u = (axis + 1) % 3;
		const int v = (axis + 2) % 3;
		for (const float su : {-1.0f, 1.0f}) {
			for (const float sv : {-1.0f, 1.0f}) {
				glm::vec3 vertex(0.0f);
				vertex[axis] = sign * m_halfExtents[axis];
				vertex[u] = su * m_halfExtents[u];
				vertex[v] = sv * m_halfExtents[v];
				feature.vertices.push_back(transform.position + orientation * vertex);
			}
		}
		return feature;
	}

private:
	glm::vec3 m_halfExtents = glm::vec3(0.5f);
};

// Vertex-cloud support mapping of its convex hull; no face connectivity is stored.
class ConvexHullShape final : public CollisionShape {
public:
	// Take ownership of local vertices without validating or rebuilding the hull.
	explicit ConvexHullShape(std::vector<glm::vec3> vertices)
		: m_vertices(std::move(vertices)) {}

	// Read the original local vertex cloud.
	const std::vector<glm::vec3>& Vertices() const { return m_vertices; }

	// Bound all transformed vertices; an empty cloud yields a point at the world position.
	AABB ComputeAABB(const ShapeTransform& transform) const override {
		if (m_vertices.empty()) {
			return {transform.position, transform.position};
		}

		const glm::quat orientation = glm::normalize(transform.orientation);
		glm::vec3 boundsMin(std::numeric_limits<float>::max());
		glm::vec3 boundsMax(std::numeric_limits<float>::lowest());
		for (const glm::vec3& vertex : m_vertices) {
			const glm::vec3 worldVertex = transform.position + orientation * vertex;
			boundsMin = glm::min(boundsMin, worldVertex);
			boundsMax = glm::max(boundsMax, worldVertex);
		}
		return {boundsMin, boundsMax};
	}

	// Maximize dot(vertex,inverse(q)*direction); ties keep the first vertex, empty returns position.
	glm::vec3 Support(const ShapeTransform& transform, const glm::vec3& direction) const override {
		if (m_vertices.empty()) {
			return transform.position;
		}

		const glm::quat orientation = glm::normalize(transform.orientation);
		const glm::vec3 localDirection = glm::inverse(orientation) * direction;
		const glm::vec3* supportVertex = &m_vertices.front();
		float bestProjection = glm::dot(*supportVertex, localDirection);
		for (const glm::vec3& vertex : m_vertices) {
			const float projection = glm::dot(vertex, localDirection);
			if (projection > bestProjection) {
				bestProjection = projection;
				supportVertex = &vertex;
			}
		}
		return transform.position + orientation * *supportVertex;
	}

	// Collect near-maximal projections and classify by count, without testing coplanarity.
	SupportFeature GetSupportFeature(
		const ShapeTransform& transform,
		const glm::vec3& direction) const override {
		if (m_vertices.empty()) {
			return {};
		}

		const glm::quat orientation = glm::normalize(transform.orientation);
		const glm::vec3 localDirection = glm::inverse(orientation) * direction;
		float bestProjection = std::numeric_limits<float>::lowest();
		for (const glm::vec3& vertex : m_vertices) {
			bestProjection = std::max(bestProjection, glm::dot(vertex, localDirection));
		}

		// Projection tolerance=0.02*max(largest local half extent,1e-3).
		float maxHalfExtent = 0.0f;
		glm::vec3 boundsMin(std::numeric_limits<float>::max());
		glm::vec3 boundsMax(std::numeric_limits<float>::lowest());
		for (const glm::vec3& vertex : m_vertices) {
			boundsMin = glm::min(boundsMin, vertex);
			boundsMax = glm::max(boundsMax, vertex);
		}
		const glm::vec3 halfExtents = 0.5f * (boundsMax - boundsMin);
		maxHalfExtent = std::max({halfExtents.x, halfExtents.y, halfExtents.z});
		const float tolerance = 0.02f * std::max(maxHalfExtent, 1e-3f);
		SupportFeature feature;
		for (const glm::vec3& vertex : m_vertices) {
			if (bestProjection - glm::dot(vertex, localDirection) <= tolerance) {
				feature.vertices.push_back(transform.position + orientation * vertex);
			}
		}
		feature.type = feature.vertices.size() >= 3
			? SupportFeatureType::Face
			: (feature.vertices.size() == 2 ? SupportFeatureType::Edge : SupportFeatureType::Vertex);
		return feature;
	}

private:
	std::vector<glm::vec3> m_vertices;
};

} // namespace Physics
} // namespace Runtime
