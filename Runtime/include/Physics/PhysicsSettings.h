#pragma once

#include <cstddef>
#include <cstdint>

namespace Runtime {
namespace Physics {

// Default tuning values for worker scheduling, speculative contacts, CCD and island sleep.
struct PhysicsSettings {
	// workerCount counts ALL participants including the calling thread;
	// 1 is the fully serial deterministic fallback, 0 resolves to hardware
	// concurrency. Applied to the fixed Core::JobSystem pool.
	uint32_t workerCount = 0;
	// Distribute the midphase work list across the job system.
	// Disabling forces in-line execution on the calling thread.
	bool parallelNarrowphaseEnabled = true;
	// Minimum pairs per narrow-phase job chunk. The job
	// system may pick larger chunks but never smaller ones.
	std::size_t narrowphaseMinPairsPerJob = 32;
	// Solve independent islands as job-system tasks. Disabling
	// forces in-line per-island execution on the calling thread.
	bool parallelIslandSolverEnabled = true;
	// Island scheduling: islands are ordered by prepared-constraint count
	// descending; an island at or above this count becomes a dedicated job,
	// smaller islands are batched until the job reaches it.
	std::size_t islandSolverMinConstraintsPerJob = 32;
	// Island sleep. Sleeping bodies stay in the dynamic BVH but skip
	// integration, active broad-phase queries, sleeping-inactive narrow-phase
	// pairs and the solver until their island is woken; disabling wakes sleeping bodies.
	bool sleepEnabled = true;
	// Speculative contact band in meters; separated points target v_n >= penetration/dt.
	// Negative penetration allows approach to touch rather than stopping at the band edge.
	float speculativeContactDistance = 0.02f;
	// CCD surface-travel threshold in meters, independent of the speculative band.
	// cutoff = min(half the smallest half extent, threshold); zero uses only the size cutoff.
	float ccdMotionThreshold = 0.02f;
	// Compatibility speed settings; PhysicsWorld::EvaluateIslandSleep consumes neither threshold.
	// Rest evaluation uses windowed displacement, external activity and contact Enter/Exit instead.
	float sleepLinearSpeedThreshold = 0.05f;
	float sleepAngularSpeedThreshold = 0.05f;
	// Island rest time required before entering sleep.
	float sleepTimeThreshold = 0.5f;
	// Maximum windowed displacement of a body's center and three orthogonal orientation probes.
	// The body test accepts |dx| and |dx + (R-Rref)*r_i| <= this distance.
	float sleepMaxDisplacement = 0.02f;
};

} // namespace Physics
} // namespace Runtime
