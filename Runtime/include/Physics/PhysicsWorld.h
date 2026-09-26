#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cmath>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

#include <glm/glm.hpp>

#include "Collider.h"
#include "CollisionDetector.h"
#include "ContactSolver.h"
#include "ContinuousCollision.h"
#include "ContactManifold.h"
#include "Integrator.h"
#include "PhysicsIsland.h"
#include "PhysicsMaterial.h"
#include "PhysicsSettings.h"
#include "RigidBody.h"

namespace Runtime {
namespace Physics {

struct PhysicsStepStats {
	uint64_t fixedStepCount = 0;
	uint64_t collisionDetectionPassCount = 0;
	uint64_t velocitySolverPassCount = 0;
	uint64_t positionSolverPassCount = 0;
	std::size_t staticBodyCount = 0;
	std::size_t dynamicBodyCount = 0;
	std::size_t staticBvhLeafCount = 0;
	std::size_t dynamicBvhLeafCount = 0;
	std::size_t broadPhaseCandidateCount = 0;
	std::size_t narrowPhaseTestCount = 0;
	std::size_t gjkCallCount = 0;
	std::size_t gjkFailureCount = 0;
	std::size_t epaCallCount = 0;
	std::size_t epaFailureCount = 0;
	std::size_t manifoldCount = 0;
	std::size_t contactPointCount = 0;
	std::size_t midphaseActivePairCount = 0;
	std::size_t midphaseNewPairCount = 0;
	std::size_t midphaseRemovedPairCount = 0;
	// Phase 7 physics islands (snapshot of the last sub-step's build).
	std::size_t islandCount = 0;
	std::size_t islandMaxBodyCount = 0;
	double integrationMilliseconds = 0.0;
	double broadPhaseMilliseconds = 0.0;
	double narrowPhaseMilliseconds = 0.0;
	double solverMilliseconds = 0.0;
	double islandBuildMilliseconds = 0.0;
	double totalMilliseconds = 0.0;
	// Phase 6 parallel narrow-phase telemetry (per fixed step).
	std::size_t narrowPhaseJobCount = 0;
	std::size_t narrowPhaseWorkerCount = 0;
	double narrowPhaseWorkerBusyMilliseconds = 0.0;
	double narrowPhaseTailWaitMilliseconds = 0.0;
	// Phase 8 parallel island solver telemetry (per fixed step).
	std::size_t islandSolverJobCount = 0;
	std::size_t islandSolverWorkerCount = 0;
	double islandSolverWorkerBusyMilliseconds = 0.0;
	double islandSolverTailWaitMilliseconds = 0.0;
};

class PhysicsWorld {
public:
	// Construct world with a default semi-implicit Euler integrator.
	PhysicsWorld();

	// Set and query global gravity acceleration.
	void SetGravity(const glm::vec3& gravity) { m_gravity = gravity; }
	const glm::vec3& Gravity() const { return m_gravity; }

	// Configure fixed simulation step length used by accumulator stepping.
	void SetFixedTimeStep(float dt) { m_fixedTimeStep = std::max(0.0001f, dt); }
	float FixedTimeStep() const { return m_fixedTimeStep; }

	// Enable/disable continuous collision detection for high-speed bodies.
	void SetContinuousCollisionEnabled(bool enabled) { m_enableCcd = enabled; }
	bool ContinuousCollisionEnabled() const { return m_enableCcd; }

	// Limit the maximum CCD sub-steps processed in one fixed step.
	void SetCcdMaxSubSteps(int steps) { m_ccdMaxSubSteps = std::max(1, steps); }
	int CcdMaxSubSteps() const { return m_ccdMaxSubSteps; }

	// Configure iterative contact solver pass count.
	void SetSolverIterations(int iterations) { m_solverIterations = std::max(1, iterations); }
	int SolverIterations() const { return m_solverIterations; }

	// Phase 6: parallel narrow-phase controls. workerCount counts all
	// participants including the calling thread; 1 is the fully serial
	// deterministic fallback, 0 resolves to hardware concurrency. Applied to
	// the shared fixed Core::JobSystem pool.
	void SetPhysicsWorkerCount(uint32_t workerCount);
	uint32_t PhysicsWorkerCount() const;
	void SetParallelNarrowphaseEnabled(bool enabled);
	bool ParallelNarrowphaseEnabled() const { return m_settings.parallelNarrowphaseEnabled; }
	// Phase 8: solve independent islands as job-system tasks. Disabling
	// forces in-line per-island execution on the calling thread.
	void SetParallelIslandSolverEnabled(bool enabled);
	bool ParallelIslandSolverEnabled() const { return m_settings.parallelIslandSolverEnabled; }
	// Minimum prepared constraints per island solver job; islands at or above
	// this count become dedicated jobs, smaller islands are batched (mirrors
	// the narrow-phase chunk size knob).
	void SetIslandSolverMinConstraintsPerJob(std::size_t count) {
		m_settings.islandSolverMinConstraintsPerJob = count;
	}
	const PhysicsSettings& Settings() const { return m_settings; }

	// A/B switch between the hybrid dual-tree broad-phase (default) and the
	// legacy single-tree broad-phase kept for Phase 1 comparison.
	void SetLegacyBroadPhaseEnabled(bool enabled);
	bool LegacyBroadPhaseEnabled() const { return m_legacyBroadPhase; }

	// Create a rigid body and return its unique runtime id.
	uint32_t CreateRigidBody(const RigidBodyDesc& desc);

	// Remove body and its attached collider if present.
	bool DestroyRigidBody(uint32_t bodyId);

	// Check body existence by id.
	bool HasRigidBody(uint32_t bodyId) const;

	// Get mutable or const body pointer; returns nullptr if missing.
	RigidBody* GetRigidBody(uint32_t bodyId);

	const RigidBody* GetRigidBody(uint32_t bodyId) const;

	// Attach or replace collider on a body.
	bool AttachCollider(uint32_t bodyId, const ColliderDesc& desc);

	// Remove collider from body.
	bool RemoveCollider(uint32_t bodyId);

	// Check collider existence by body id.
	bool HasCollider(uint32_t bodyId) const;

	// Get mutable or const collider pointer; returns nullptr if missing.
	Collider* GetCollider(uint32_t bodyId);

	const Collider* GetCollider(uint32_t bodyId) const;

	// Advance world time using accumulator-driven fixed steps.
	void Step(float deltaTime);

	// Get contacts generated by the latest solved step. Per the frozen publish
	// contract this is the set of unique pairs that touched during the most
	// recent completed fixed step (geometry and normalImpulse from the last
	// touching sub-step), not necessarily the pairs still touching at step end.
	const std::vector<ContactManifold>& Contacts() const { return m_contacts; }
	// Contact events (Enter/Stay/Exit per fixedStepId/queryEpoch/PairKey)
	// covering every fixed step executed by the latest Step call.
	const std::vector<ContactEvent>& ContactEvents() const { return m_contactEvents; }
	uint64_t FixedStepId() const { return m_fixedStepId; }
	const PhysicsStepStats& LastStepStats() const { return m_lastStepStats; }

	// Phase 7/8: physics islands of the most recent sub-step, stable-sorted
	// (islands by smallest body id; bodies/contacts ascending inside). Since
	// Phase 8 they are the solver's work partition for the parallel island
	// solver.
	const std::vector<PhysicsIsland>& LastIslands() const { return m_islandBuilder.Islands(); }

	// Read-only access to body/collider containers.
	const std::unordered_map<uint32_t, RigidBody>& Bodies() const { return m_bodies; }
	const std::unordered_map<uint32_t, Collider>& Colliders() const { return m_colliders; }

private:
	// Integrate all bodies for one sub-step.
	void IntegrateBodies(float dt);

	// Detect once per sub-step, then Prepare -> WarmStart once -> SolveVelocity
	// N times -> SolvePosition once -> commit accumulated impulses back to the
	// persistent midphase manifolds.
	void DetectAndSolveContacts(float substepDt, bool isToiSubstep);

	// Execute one fixed simulation tick with optional CCD splitting.
	void FixedStep(float dt);

	// Merge one detection's statistics and events into the step records.
	void AccumulateDetectionStats(const CollisionDetectionResult& result);
	// Upsert touching pairs into the PairKey-keyed fixed-step contact summary.
	void PublishTouchingContacts(const std::vector<uint32_t>& touchingSlots);

	// Phase 8: build the sub-step's prepared constraints grouped by island
	// (contiguous ranges in m_preparedConstraints) and schedule island jobs:
	// islands ordered by constraint count descending, large islands become
	// dedicated jobs, small islands are batched to the configured minimum.
	void PrepareIslandConstraints(const std::vector<uint32_t>& touchingSlots, float substepDt);
	// Warm start once, run the N velocity iterations and the single position
	// pass per island; independent islands run as job-system tasks, while
	// workerCount == 1 or the disabled toggle degrade to in-line execution on
	// the calling thread.
	void SolveIslandConstraints();
	// Store solved lambdas into the persistent caches and accumulate the
	// fixed-step net impulse totals on the midphase pairs.
	void CommitSolvedContacts();

	// Phase 7: rebuild physics islands over all dynamic bodies and the
	// touching pairs of the latest detection query.
	void BuildIslands(const std::vector<uint32_t>& touchingSlots);

	glm::vec3 m_gravity = glm::vec3(0.0f, -9.81f, 0.0f);
	float m_fixedTimeStep = 1.0f / 60.0f;
	float m_accumulator = 0.0f;
	uint32_t m_nextBodyId = 1;
	uint32_t m_nextColliderIdentity = 1;
	uint64_t m_fixedStepId = 0;
	uint64_t m_queryEpoch = 0;

	std::unordered_map<uint32_t, RigidBody> m_bodies;
	std::unordered_map<uint32_t, Collider> m_colliders;
	std::vector<ContactManifold> m_contacts;
	// PairKey-keyed fixed-step contact summary index (replaces the old O(n^2)
	// merge helper); rebuilt per fixed step, invalid after step end sorting.
	std::unordered_map<PairKey, std::size_t, PairKeyHash> m_stepContactIndex;
	std::vector<ContactEvent> m_contactEvents;
	bool m_enableCcd = true;
	bool m_legacyBroadPhase = false;
	int m_ccdMaxSubSteps = 8;
	int m_solverIterations = 8;
	PhysicsSettings m_settings;
	PhysicsStepStats m_lastStepStats;
	// Substep-temporary prepared constraints, rebuilt every sub-step.
	std::vector<PreparedContactConstraint> m_preparedConstraints;
	// Phase 7 island build scratch, rebuilt every sub-step (capacity reused).
	std::vector<uint32_t> m_dynamicBodyIds;
	std::vector<IslandContact> m_islandContacts;
	PhysicsIslandBuilder m_islandBuilder;
	// Phase 8 island solver scheduling scratch (capacity reused per sub-step).
	struct IslandConstraintRange {
		std::size_t begin = 0;
		std::size_t end = 0;
	};
	struct IslandJob {
		std::size_t scheduleBegin = 0;
		std::size_t scheduleEnd = 0;
	};
	struct IslandJobRecord {
		std::thread::id threadId;
		double startMilliseconds = 0.0;
		double endMilliseconds = 0.0;
	};
	std::vector<IslandConstraintRange> m_islandConstraintRanges;
	std::vector<std::size_t> m_islandSchedule;
	std::vector<IslandJob> m_islandJobs;
	std::vector<IslandJobRecord> m_islandJobRecords;
#ifndef NDEBUG
	// Acceptance gate runtime assertion: every island task stamps its dynamic
	// bodies with the current query epoch before solving; a duplicate stamp
	// means two tasks would write the same dynamic body.
	std::unordered_map<uint32_t, uint32_t> m_islandBodyIndex;
	std::unique_ptr<std::atomic<uint32_t>[]> m_islandBodyClaims;
	std::size_t m_islandBodyClaimCapacity = 0;
#endif

	CollisionDetector m_collisionDetector;
	ContinuousCollisionDetector m_continuousCollision;
	ContactSolver m_contactSolver;
	std::unique_ptr<Integrator> m_integrator;
};

} // namespace Physics
} // namespace Runtime
