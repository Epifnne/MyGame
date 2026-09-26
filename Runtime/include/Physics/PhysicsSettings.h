#pragma once

#include <cstddef>
#include <cstdint>

namespace Runtime {
namespace Physics {

// Frozen physics tuning surface. Phase 6 introduces the parallel narrow-phase
// worker/chunk parameters, Phase 8 the parallel island solver parameters;
// sleep parameters join in later phases.
struct PhysicsSettings {
	// workerCount counts ALL participants including the calling thread;
	// 1 is the fully serial deterministic fallback, 0 resolves to hardware
	// concurrency. Applied to the fixed Core::JobSystem pool.
	uint32_t workerCount = 0;
	// Phase 6: distribute the midphase work list across the job system.
	// Disabling forces in-line execution on the calling thread.
	bool parallelNarrowphaseEnabled = true;
	// Minimum pairs per narrow-phase job chunk (plan: initial 32). The job
	// system may pick larger chunks but never smaller ones.
	std::size_t narrowphaseMinPairsPerJob = 32;
	// Phase 8: solve independent islands as job-system tasks. Disabling
	// forces in-line per-island execution on the calling thread.
	bool parallelIslandSolverEnabled = true;
	// Island scheduling: islands are ordered by prepared-constraint count
	// descending; an island at or above this count becomes a dedicated job,
	// smaller islands are batched until the job reaches it (plan: initial 32).
	std::size_t islandSolverMinConstraintsPerJob = 32;
};

} // namespace Physics
} // namespace Runtime
