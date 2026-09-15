// Runtime/include/Core/JobSystem.h
// Phase 5: fixed worker-pool job system with batching and a frame-level barrier.
//
// Contract:
// - workerCount counts ALL participants including the calling thread; 1 means
//   fully serial execution on the caller.
// - ParallelForRange is the barrier: it returns only after every index in
//   [0, count) has been processed exactly once.
// - No per-frame std::async / thread creation; workers are created once in
//   Initialize and reused.
// - ParallelForRange is called from a single driver thread (the frame thread);
//   it is not reentrant.
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <memory>
#include <mutex>
#include <new>
#include <thread>
#include <type_traits>
#include <vector>

namespace Runtime {
namespace Core {

class JobSystem {
public:
	// Non-owning callable view over [begin, end) range processors. Zero
	// allocation: just a data pointer plus an invoke thunk. The referenced
	// callable only needs to outlive the ParallelForRange call that consumed
	// it (the barrier guarantees it does). Accepts lambdas (including
	// mutable), functors and plain function pointers.
	class RangeFunction {
	public:
		template <typename F>
		RangeFunction(F&& fn) noexcept
			: m_data(const_cast<void*>(static_cast<const void*>(std::addressof(fn)))),
			  m_invoke([](void* data, std::size_t begin, std::size_t end) {
				  (*static_cast<std::remove_reference_t<F>*>(data))(begin, end);
			  }) {}

		void operator()(std::size_t begin, std::size_t end) const {
			m_invoke(m_data, begin, end);
		}

	private:
		void* m_data;
		void (*m_invoke)(void*, std::size_t, std::size_t);
	};

	static JobSystem& Get();

	JobSystem(const JobSystem&) = delete;
	JobSystem& operator=(const JobSystem&) = delete;

	~JobSystem();

	// Start the fixed pool. workerCount == 0 resolves to hardware_concurrency.
	// Calling Initialize again with a different count safely restarts the pool.
	void Initialize(uint32_t workerCount = 0);

	// Stop and join all workers; safe to call when idle, repeated, or never
	// having initialized.
	void Shutdown();

	bool IsInitialized() const { return m_workerCount > 0; }

	// Total participants including the calling thread (0 when uninitialized).
	uint32_t WorkerCount() const { return m_workerCount; }

	// Execute function over [0, count) in chunks of at least minItemsPerJob.
	// Blocks (frame-level barrier) until all chunks complete; the calling
	// thread participates. Small ranges and workerCount == 1 degrade to inline
	// execution on the calling thread. A throwing chunk never deadlocks the
	// barrier; the first exception is rethrown on the calling thread.
	void ParallelForRange(
		std::size_t count,
		std::size_t minItemsPerJob,
		const RangeFunction& function);

private:
	JobSystem() = default;

	// One batch of work handed to the pool. A fresh instance is created per
	// ParallelForRange call, so stale workers always operate on their own
	// snapshot and generations can never alias.
	struct Job {
		explicit Job(const RangeFunction& fn) : function(fn) {}

		// Read-only during execution; shared by all participants on one line.
		alignas(std::hardware_destructive_interference_size) RangeFunction function;
		std::size_t chunkSize = 0;
		std::size_t itemCount = 0;
		std::size_t jobCount = 0;

		// Hot claim counter: every chunk claim is a fetch_add from any thread.
		// Own cache line so claims never evict the read-only fields or the
		// completion counter.
		alignas(std::hardware_destructive_interference_size)
		std::atomic<std::size_t> nextChunk{0};

		// Completion counter: written once per finished chunk; separate line
		// to avoid ping-ponging with nextChunk.
		alignas(std::hardware_destructive_interference_size)
		std::atomic<std::size_t> chunksDone{0};

		// Cold: touched only by the barrier wait and the exception path.
		alignas(std::hardware_destructive_interference_size) std::mutex mutex;
		std::condition_variable doneCv;
		std::exception_ptr firstException;
	};

	void WorkerMain(std::stop_token stopToken);
	// Claim and run chunks until the job is exhausted; called by workers and
	// by the participating calling thread.
	void RunChunks(Job& job);
	std::size_t ComputeChunkSize(std::size_t count, std::size_t minItemsPerJob) const;

	uint32_t m_workerCount = 0;
	std::vector<std::jthread> m_workers;

	std::mutex m_queueMutex;
	// condition_variable_any for stop_token-aware waits.
	std::condition_variable_any m_queueCv;
	// Wake-up grants only: each ticket lets one worker join the shared Job;
	// chunk claiming itself is lock-free via Job::nextChunk.
	std::deque<std::shared_ptr<Job>> m_queue;
	bool m_stop = false;
};

} // namespace Core
} // namespace Runtime
