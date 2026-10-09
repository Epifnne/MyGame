#pragma once

#include <algorithm>
#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace Runtime {
namespace Physics {

// Initial pose, velocities, principal inertia and simulation policy.
struct RigidBodyDesc {
	// Initial transform.
	glm::vec3 position = glm::vec3(0.0f);
	glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	// Initial velocities.
	glm::vec3 linearVelocity = glm::vec3(0.0f);
	glm::vec3 angularVelocity = glm::vec3(0.0f);
	// Principal inertia diagonal in local space.
	glm::vec3 inertiaTensorDiagonal = glm::vec3(1.0f);
	float mass = 1.0f;
	bool isStatic = false;
	bool useGravity = true;
	bool allowSleep = true;
};

// World-managed activity states; Candidate accumulates rest time before Sleeping.
enum class SleepState : uint8_t {
	Awake,
	Candidate,
	Sleeping,
};

// Stores pose, forces and momentum, with cached world inertia and island-sleep state.
class RigidBody {
public:
	// Create a dynamic unit-mass body at rest with identity pose and inertia.
	RigidBody() = default;

	// Normalize the descriptor pose, clamp mass/inertia and derive L = Iworld * omega.
	explicit RigidBody(const RigidBodyDesc& desc);

	// Return the runtime body identifier.
	uint32_t Id() const { return m_id; }
	// Assign the identifier without marking external activity.
	void SetId(uint32_t id) { m_id = id; }

	// Return the world-space center position.
	const glm::vec3& Position() const { return m_position; }
	// Teleport the center, increment pose/transform versions and mark external activity.
	void SetPosition(const glm::vec3& position) {
		m_position = position;
		++m_poseRevision;
		++m_transformVersion;
		m_externalActivity = true;
	}

	// Return the world-space orientation quaternion.
	const glm::quat& Orientation() const { return m_orientation; }
	// Normalize the new pose, refresh inertia and omega, and mark an external teleport.
	void SetOrientation(const glm::quat& orientation);

	// Write the center and increment only the transform version, preserving contact caches.
	void SetPositionInternal(const glm::vec3& position) {
		m_position = position;
		++m_transformVersion;
	}

	// Apply x += dx and q = normalize(q + 0.5 * (0, dtheta) * q); preserve v, omega and L.
	// Refresh inertia after rotation; static bodies ignore the correction.
	void ApplyPositionStep(const glm::vec3& deltaPosition, const glm::vec3& deltaRotation);

	// Return the external teleport revision used for contact-cache invalidation.
	uint32_t PoseRevision() const { return m_poseRevision; }
	// Return the revision incremented by mass, inertia and static-state setters.
	uint32_t StructureRevision() const { return m_structureRevision; }

	// Return the version incremented by pose writes, including integration/correction.
	uint32_t TransformVersion() const { return m_transformVersion; }

	// Test |dx| and |dx + (R - Rref) * r_i| <= maxDisplacement for three axial probes.
	// Changing the probe arm reanchors and succeeds; failure does not reset the reference.
	bool SleepDisplacementWithin(float maxDisplacement, float probeArmLength);
	// Anchor the displacement window at the current pose without changing its probe arm.
	void ResetSleepDisplacement();

	// Return the world-managed sleep state.
	SleepState GetSleepState() const { return m_sleepState; }
	// Test whether the body is in the Sleeping state.
	bool IsSleeping() const { return m_sleepState == SleepState::Sleeping; }
	// Return accumulated rest time.
	float SleepTimer() const { return m_sleepTimer; }
	// Return the body's sleep permission.
	bool AllowSleep() const { return m_allowSleep; }
	// Change sleep permission and mark external activity only when the value changes.
	void SetAllowSleep(bool allowSleep);

	// Return the external-write flag consumed by world wake/rest processing.
	bool HasExternalActivity() const { return m_externalActivity; }
	// Clear the external-write flag without changing sleep state.
	void ClearExternalActivity() { m_externalActivity = false; }

	// Add dt for nonstatic, nonsleeping bodies; positive accumulated time means Candidate.
	void AdvanceSleepTimer(float dt);
	// Clear rest time and change Candidate to Awake, leaving Sleeping unchanged.
	void ResetSleepTimer();
	// For dynamic bodies, enter Sleeping and zero rest time, v, omega and L.
	void EnterSleep();
	// Set Awake and clear rest time without changing motion or external activity.
	void WakeUp();

	// Return world-space linear velocity.
	const glm::vec3& LinearVelocity() const { return m_linearVelocity; }
	// Assign linear velocity and mark external activity, even for a static body.
	void SetLinearVelocity(const glm::vec3& velocity) {
		m_linearVelocity = velocity;
		m_externalActivity = true;
	}

	// Return world-space angular velocity.
	const glm::vec3& AngularVelocity() const { return m_angularVelocity; }
	// Set L = Iworld * omega, rederive omega and mark external activity; static L/omega are zero.
	void SetAngularVelocity(const glm::vec3& velocity);

	// Return world-space angular momentum.
	const glm::vec3& AngularMomentum() const { return m_angularMomentum; }
	// Assign L and derive omega = Iworld^-1 * L; static bodies retain L but have zero omega.
	void SetAngularMomentum(const glm::vec3& momentum);

	// Return stored mass (zero for bodies made static through mass/static setters).
	float Mass() const { return m_mass; }
	// Return inverse mass used by force and impulse response.
	float InverseMass() const { return m_inverseMass; }

	// Return the clamped local principal inertia diagonal.
	const glm::vec3& InertiaTensorDiagonal() const { return m_inertiaTensorDiagonal; }
	// Return component-wise reciprocals of the local principal inertia.
	const glm::vec3& InverseInertiaTensorDiagonal() const { return m_inverseInertiaTensorDiagonal; }

	// Return cached R * diag(1/I) * R^T, or zero for static bodies; mass is independent of inertia.
	const glm::mat3& InverseInertiaTensorWorld() const { return m_inverseInertiaTensorWorld; }

	// Mark structural/external changes; clamp dynamic mass to >= 0.0001, or store zero if static.
	void SetMass(float mass);

	// Clamp each inertia component to >= 0.0001, refresh inertia/omega and mark a structural write.
	void SetInertiaTensorDiagonal(const glm::vec3& inertiaDiagonal);

	// Return whether integration and physical impulse response are disabled.
	bool IsStatic() const { return m_isStatic; }
	// Reset sleep state; when static, zero mass, forces and motion; restore unit mass if needed.
	void SetStatic(bool isStatic);

	// Return whether acceleration includes gravity.
	bool UseGravity() const { return m_useGravity; }
	// Assign gravity policy and mark external activity, even if unchanged.
	void SetUseGravity(bool useGravity) {
		m_useGravity = useGravity;
		m_externalActivity = true;
	}

	// Accumulate world force and mark external activity; static bodies ignore it.
	void ApplyForce(const glm::vec3& force);

	// Accumulate world torque and mark external activity; static bodies ignore it.
	void ApplyTorque(const glm::vec3& torque);

	// Return a = F/m + optional gravity, or zero for static bodies.
	glm::vec3 LinearAcceleration(const glm::vec3& gravity) const;

	// Apply dv = impulse/m and mark external activity; static bodies ignore it.
	void ApplyLinearImpulse(const glm::vec3& impulse);

	// Apply dL = impulse and domega = Iworld^-1 * impulse; mark external activity unless static.
	void ApplyAngularImpulse(const glm::vec3& impulse);

	// Apply dv = impulse/m without external activity; static bodies ignore it.
	void ApplySolverLinearImpulse(const glm::vec3& impulse);

	// Add angular impulse to L and caller-precomputed domega to omega without external activity.
	// The caller must use the matching inertia response; static bodies ignore it.
	void ApplySolverAngularImpulse(const glm::vec3& angularImpulse, const glm::vec3& deltaAngularVelocity);

	// Zero force/torque buffers without changing external activity.
	void ClearForces();

	// Run velocity then pose integration (semi-implicit Euler), retaining force/torque buffers.
	// Integration ignores static bodies and dt <= 0; sleep gating is the caller's responsibility.
	void Integrate(float dt, const glm::vec3& gravity);

	// For dynamic bodies and dt > 0, add a*dt to v and torque*dt to L, then derive omega.
	void IntegrateVelocity(float dt, const glm::vec3& gravity);

	// For dynamic bodies and dt > 0, advance x += dt*v and normalize q + 0.5*dt*(0,omega)*q.
	// Increment transform version and refresh inertia/omega while preserving L.
	void IntegratePositions(float dt);

private:
	// Return Iworld = R * diag(I) * R^T, or zero for static bodies.
	glm::mat3 InverseInertiaTensorWorldInverse() const;

	// Set omega = cached Iworld^-1 * L, or zero if static; leave L unchanged.
	void UpdateAngularVelocityFromMomentum();

	// Cache Iworld^-1 = R * diag(1/I) * R^T, or zero for static bodies.
	void RefreshInverseInertiaTensorWorld();

	uint32_t m_id = 0;
	uint32_t m_poseRevision = 0;
	uint32_t m_structureRevision = 0;
	uint32_t m_transformVersion = 0;
	glm::vec3 m_position = glm::vec3(0.0f);
	glm::quat m_orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	glm::vec3 m_linearVelocity = glm::vec3(0.0f);
	glm::vec3 m_angularVelocity = glm::vec3(0.0f);
	glm::vec3 m_angularMomentum = glm::vec3(0.0f);
	glm::vec3 m_accumulatedForce = glm::vec3(0.0f);
	glm::vec3 m_accumulatedTorque = glm::vec3(0.0f);
	glm::vec3 m_inertiaTensorDiagonal = glm::vec3(1.0f);
	glm::vec3 m_inverseInertiaTensorDiagonal = glm::vec3(1.0f);
	// Cached world inverse inertia; identity matches the default state
	// (identity orientation, unit inertia, dynamic body).
	glm::mat3 m_inverseInertiaTensorWorld = glm::mat3(1.0f);
	float m_mass = 1.0f;
	float m_inverseMass = 1.0f;
	// Island sleep bookkeeping (transitions owned by PhysicsWorld).
	SleepState m_sleepState = SleepState::Awake;
	float m_sleepTimer = 0.0f;
	bool m_allowSleep = true;
	bool m_externalActivity = false;
	bool m_isStatic = false;
	bool m_useGravity = true;
	// Windowed sleep displacement reference (center + orientation probes).
	glm::vec3 m_sleepRefPosition = glm::vec3(0.0f);
	glm::quat m_sleepRefOrientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	float m_sleepProbeArm = 0.0f;
};

} // namespace Physics
} // namespace Runtime
