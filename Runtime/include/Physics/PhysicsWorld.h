#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cmath>
#include <memory>
#include <thread>
#include <unordered_map>
#include <unordered_set>
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

// Work totals for the latest positive Step call, with last-query state snapshots.
struct PhysicsStepStats {
	uint64_t fixedStepCount = 0;
	uint64_t collisionDetectionPassCount = 0;
	uint64_t velocitySolverPassCount = 0;
	uint64_t positionSolverPassCount = 0;
	std::size_t staticBodyCount = 0;
	std::size_t dynamicBodyCount = 0;
	// Dynamic body activity after the last fixed step; statics never sleep.
	std::size_t awakeBodyCount = 0;
	std::size_t sleepingBodyCount = 0;
	std::size_t staticBvhLeafCount = 0;
	std::size_t dynamicBvhLeafCount = 0;
	std::size_t broadPhaseCandidateCount = 0;
	std::size_t narrowPhaseTestCount = 0;
	std::size_t satCallCount = 0;
	std::size_t primitiveCallCount = 0;
	std::size_t gjkCallCount = 0;
	std::size_t gjkFailureCount = 0;
	std::size_t epaCallCount = 0;
	std::size_t epaFailureCount = 0;
	std::size_t manifoldCount = 0;
	std::size_t contactPointCount = 0;
	std::size_t midphaseActivePairCount = 0;
	std::size_t midphaseNewPairCount = 0;
	std::size_t midphaseRemovedPairCount = 0;
	// Island sizes from the latest detection query.
	std::size_t islandCount = 0;
	std::size_t islandMaxBodyCount = 0;
	double integrationMilliseconds = 0.0;
	double broadPhaseMilliseconds = 0.0;
	double narrowPhaseMilliseconds = 0.0;
	double solverMilliseconds = 0.0;
	double islandBuildMilliseconds = 0.0;
	double totalMilliseconds = 0.0;
	// Narrow-phase job/time totals and maximum worker participation per Step.
	std::size_t narrowPhaseJobCount = 0;
	std::size_t narrowPhaseWorkerCount = 0;
	double narrowPhaseWorkerBusyMilliseconds = 0.0;
	double narrowPhaseTailWaitMilliseconds = 0.0;
	// Island solver job/time totals and maximum worker participation per Step.
	std::size_t islandSolverJobCount = 0;
	std::size_t islandSolverWorkerCount = 0;
	double islandSolverWorkerBusyMilliseconds = 0.0;
	double islandSolverTailWaitMilliseconds = 0.0;
	// CCD sweep attempts, hits, and unswept remainders due to budget exhaustion.
	uint64_t ccdSubStepCount = 0;
	uint64_t ccdToiHitCount = 0;
	uint64_t ccdBudgetExhaustionCount = 0;
};

// Fixed-step rigid-body simulation with persistent contacts, island solving, and CCD.
class PhysicsWorld {
public:
	// Construct world with a default semi-implicit Euler integrator.
	PhysicsWorld();

	// Set gravity; when sleep is enabled, a change wakes all sleeping bodies.
	void SetGravity(const glm::vec3& gravity);
	// Return the world-space gravity acceleration.
	const glm::vec3& Gravity() const { return m_gravity; }

	// Configure fixed simulation step length used by accumulator stepping.
	void SetFixedTimeStep(float dt) { m_fixedTimeStep = std::max(0.0001f, dt); }
	// Return the accumulator's fixed-step interval in seconds.
	float FixedTimeStep() const { return m_fixedTimeStep; }

	// Enable/disable continuous collision detection for high-speed bodies.
	void SetContinuousCollisionEnabled(bool enabled) { m_enableCcd = enabled; }
	// Return whether pose advancement uses time-of-impact sweeps.
	bool ContinuousCollisionEnabled() const { return m_enableCcd; }

	// Set a finite nonnegative CCD travel threshold in meters; zero uses the shape-size cutoff.
	// Invalid values throw std::invalid_argument; the speculative contact band is unchanged.
	void SetCcdMotionThreshold(float distance);
	// Return the CCD travel threshold, independent of the speculative contact band.
	float CcdMotionThreshold() const { return m_settings.ccdMotionThreshold; }

	// Limit the maximum CCD sub-steps processed in one fixed step.
	void SetCcdMaxSubSteps(int steps) { m_ccdMaxSubSteps = std::max(1, steps); }
	// Return the maximum mid-window impact solves per fixed step.
	int CcdMaxSubSteps() const { return m_ccdMaxSubSteps; }

	// Configure iterative contact solver pass count.
	void SetSolverIterations(int iterations) { m_solverIterations = std::max(1, iterations); }
	// Return the number of velocity iterations per island solve.
	int SolverIterations() const { return m_solverIterations; }

	// Configure the shared job pool: count includes the caller; 0 uses hardware concurrency.
	void SetPhysicsWorkerCount(uint32_t workerCount);
	// Return the shared pool's resolved participant count.
	uint32_t PhysicsWorkerCount() const;
	// Toggle job-based narrow-phase evaluation; disabled uses the calling thread.
	void SetParallelNarrowphaseEnabled(bool enabled);
	// Return whether narrow-phase jobs are enabled.
	bool ParallelNarrowphaseEnabled() const { return m_settings.parallelNarrowphaseEnabled; }
	// Toggle island jobs; disabled solves each island on the calling thread.
	void SetParallelIslandSolverEnabled(bool enabled);
	// Return whether independent islands may solve in parallel.
	bool ParallelIslandSolverEnabled() const { return m_settings.parallelIslandSolverEnabled; }
	// Set the batching target; effective minimum is one constraint per job.
	void SetIslandSolverMinConstraintsPerJob(std::size_t count) {
		m_settings.islandSolverMinConstraintsPerJob = count;
	}
	// Toggle island sleeping; disabling wakes all bodies immediately.
	void SetSleepEnabled(bool enabled);
	// Return whether island rest evaluation is enabled.
	bool SleepEnabled() const { return m_settings.sleepEnabled; }
	// Set the speculative band in meters, clamped to the BVH fat margin [0, 0.08].
	void SetSpeculativeContactDistance(float distance);
	// Return the speculative contact band in meters.
	float SpeculativeContactDistance() const { return m_settings.speculativeContactDistance; }
	// Store speed thresholds and rest duration; current island sleep uses displacement, not speed.
	void SetSleepThresholds(float linearSpeed, float angularSpeed, float timeToSleep) {
		m_settings.sleepLinearSpeedThreshold = linearSpeed;
		m_settings.sleepAngularSpeedThreshold = angularSpeed;
		m_settings.sleepTimeThreshold = timeToSleep;
	}
	// Inspect all simulation settings, including displacement-based sleep parameters.
	const PhysicsSettings& Settings() const { return m_settings; }

	// Switch between hybrid dual-tree and legacy single-tree broad-phase queries.
	void SetLegacyBroadPhaseEnabled(bool enabled);
	// Return whether the legacy single-tree broad phase is selected.
	bool LegacyBroadPhaseEnabled() const { return m_legacyBroadPhase; }

	// Create a rigid body and return its unique runtime id.
	uint32_t CreateRigidBody(const RigidBodyDesc& desc);

	// Remove body and its attached collider if present.
	bool DestroyRigidBody(uint32_t bodyId);

	// Check body existence by id.
	bool HasRigidBody(uint32_t bodyId) const;

	// Get a mutable body pointer; return nullptr if missing.
	RigidBody* GetRigidBody(uint32_t bodyId);

	// Inspect a body by id; return nullptr when absent.
	const RigidBody* GetRigidBody(uint32_t bodyId) const;

	// Attach or replace collider on a body.
	bool AttachCollider(uint32_t bodyId, const ColliderDesc& desc);

	// Remove collider from body.
	bool RemoveCollider(uint32_t bodyId);

	// Check collider existence by body id.
	bool HasCollider(uint32_t bodyId) const;

	// Get a mutable collider pointer; return nullptr if missing.
	Collider* GetCollider(uint32_t bodyId);

	// Inspect a collider by body id; return nullptr when absent.
	const Collider* GetCollider(uint32_t bodyId) const;

	// Accumulate positive deltaTime and run ticks while accumulator >= fixedTimeStep.
	void Step(float deltaTime);

	// Return pairs touched in the latest completed tick, using their last solved
	// sub-step geometry and fixed-step net impulses; some may have since separated.
	const std::vector<ContactManifold>& Contacts() const { return m_contacts; }
	// Contact events (Enter/Stay/Exit per fixedStepId/queryEpoch/PairKey)
	// covering every fixed step executed by the latest Step call.
	const std::vector<ContactEvent>& ContactEvents() const { return m_contactEvents; }
	// Return the monotonically increasing completed/in-progress fixed-step id.
	uint64_t FixedStepId() const { return m_fixedStepId; }
	// Return work totals and snapshots from the latest positive Step call.
	const PhysicsStepStats& LastStepStats() const { return m_lastStepStats; }

	// Return the latest awake-body solver islands, sorted by smallest body id.
	const std::vector<PhysicsIsland>& LastIslands() const { return m_islandBuilder.Islands(); }

	// Inspect the id-indexed body container.
	const std::unordered_map<uint32_t, RigidBody>& Bodies() const { return m_bodies; }
	// Inspect the body-id-indexed collider container.
	const std::unordered_map<uint32_t, Collider>& Colliders() const { return m_colliders; }

private:
	// Integrate awake bodies' velocities before detection so constraints see gravity.
	void IntegrateBodyVelocities(float dt);
	// Advance awake poses after solving velocities: x' = x + dt * v'.
	void IntegrateBodyPositions(float dt);

	// Detect at current poses, rebuild islands, and optionally prepare/solve/commit.
	// substepDt is the remaining motion horizon; detect-only updates events, not impulses.
	void DetectAndSolveContacts(float substepDt, bool solve, bool warmStart = true,
		const ContactManifold* impactContact = nullptr);

	// Apply two nonlinear Gauss-Seidel pose iterations per island after advancement.
	void SolveContactPositions();

	// Execute one fixed simulation tick with optional CCD splitting.
	void FixedStep(float dt);

	// Merge one detection's statistics and events into the step records.
	void AccumulateDetectionStats(const CollisionDetectionResult& result);
	// Upsert touching pairs into the PairKey-keyed fixed-step contact summary.
	void PublishTouchingContacts(const std::vector<uint32_t>& touchingSlots);

	// Prepare contiguous island rows and batch largest islands first; TOI solves
	// retain old impulse cache units without reapplying them to velocities.
	void PrepareIslandConstraints(const std::vector<uint32_t>& touchingSlots, float substepDt, bool warmStart,
		const ContactManifold* impactContact);
	// Warm start once and run N velocity iterations per affected island, serially or in jobs.
	void SolveIslandConstraints();
	// Store solved lambdas into the persistent caches and accumulate the
	// fixed-step net impulse totals on the midphase pairs.
	void CommitSolvedContacts();

	// Rebuild solver islands over awake dynamic bodies and current touching pairs.
	void BuildIslands(const std::vector<uint32_t>& touchingSlots);

	// Wake externally modified dynamic islands and sleepers affected by changed statics.
	void ProcessExternalActivityWakes();
	// Walk persistent non-trigger dynamic adjacency and wake the whole island.
	// Positive compensateVelocityDt catches up missed velocity integration once per sleeper.
	bool WakeIslandContaining(uint32_t bodyId, float compensateVelocityDt);
	// Wake islands linked to a static body or overlapping its current bounds.
	void WakeStaticAdjacency(uint32_t staticBodyId);
	// Sleep an island after bounded probe displacement and stable contacts for
	// sleepTimeThreshold; external activity or an Enter/Exit resets its rest window.
	void EvaluateIslandSleep(float fixedDt);

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
	// Summary indices are valid only before the fixed-step final sort.
	std::unordered_map<PairKey, std::size_t, PairKeyHash> m_stepContactIndex;
	std::vector<ContactEvent> m_contactEvents;
	bool m_enableCcd = true;
	bool m_legacyBroadPhase = false;
	int m_ccdMaxSubSteps = 8;
	int m_solverIterations = 8;
	std::vector<uint32_t> m_islandContactOrder;
	PhysicsSettings m_settings;
	PhysicsStepStats m_lastStepStats;
	// Substep-temporary prepared constraints, rebuilt every sub-step.
	std::vector<PreparedContactConstraint> m_preparedConstraints;
	// Island build scratch, rebuilt every detection query with reused capacity.
	std::vector<uint32_t> m_dynamicBodyIds;
	std::vector<IslandContact> m_islandContacts;
	PhysicsIslandBuilder m_islandBuilder;
	// Half-open prepared-constraint range belonging to one island.
	struct IslandConstraintRange {
		std::size_t begin = 0;
		std::size_t end = 0;
	};
	// Half-open range of scheduled islands assigned to one solver job.
	struct IslandJob {
		std::size_t scheduleBegin = 0;
		std::size_t scheduleEnd = 0;
	};
	// Task-private execution interval used to aggregate worker utilization.
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
	// Per-query ownership tickets detect two tasks writing the same dynamic body.
	std::unordered_map<uint32_t, uint32_t> m_islandBodyIndex;
	std::unique_ptr<std::atomic<uint32_t>[]> m_islandBodyClaims;
	std::size_t m_islandBodyClaimCapacity = 0;
#endif

	CollisionDetector m_collisionDetector;
	ContinuousCollisionDetector m_continuousCollision;
	ContactSolver m_contactSolver;
	std::unique_ptr<Integrator> m_integrator;
	// Wake/sleep traversal scratch and fixed-step velocity compensation horizon.
	float m_wakeCompensationDt = 0.0f;
	std::vector<uint32_t> m_wakeStack;
	std::vector<uint32_t> m_wakeRegionResults;
	std::unordered_set<uint32_t> m_wakeVisited;
	std::unordered_set<PairKey, PairKeyHash> m_stepTransitionPairs;
};

} // namespace Physics
} // namespace Runtime
