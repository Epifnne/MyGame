// Runtime/src/Core/JobSystem.cpp
#include "Core/JobSystem.h"

#include <algorithm>

namespace Runtime {
namespace Core {

JobSystem& JobSystem::Get() {
	static JobSystem instance;
	return instance;
}

JobSystem::~JobSystem() {
	Shutdown();
}

void JobSystem::Initialize(uint32_t workerCount) {
	if (workerCount == 0) {
		workerCount = std::max(1u, std::thread::hardware_concurrency());
	}
	if (m_workerCount == workerCount) {
		return;
	}
	Shutdown();
	m_stop = false;
	m_workerCount = workerCount;
	// workerCount includes the calling thread; spawn the remaining participants.
	for (uint32_t index = 1; index < workerCount; ++index) {
		m_workers.emplace_back([this](std::stop_token stopToken) { WorkerMain(stopToken); });
	}
}

void JobSystem::Shutdown() {
	{
		std::lock_guard lock(m_queueMutex);
		m_stop = true;
		// ParallelForRange drains every ticket before returning, so the queue is
		// empty at any legal Shutdown point; clear defensively for re-init.
		m_queue.clear();
	}
	m_queueCv.notify_all();
	m_workers.clear(); // jthread destructors request stop and join.
	m_workerCount = 0;
	m_stop = false;
}

void JobSystem::ParallelForRange(
	std::size_t count,
	std::size_t minItemsPerJob,
	const RangeFunction& function) {
	if (count == 0) {
		return;
	}
	const std::size_t chunkSize = ComputeChunkSize(count, minItemsPerJob);
	const std::size_t jobCount = (count + chunkSize - 1) / chunkSize;
	// Small ranges and serial configurations run inline on the calling thread.
	if (m_workerCount <= 1 || jobCount == 1) {
		function(0, count);
		return;
	}

	auto job = std::make_shared<Job>(function);
	job->chunkSize = chunkSize;
	job->itemCount = count;
	job->jobCount = jobCount;
	// One worker can drain the whole job via nextChunk, so a ticket is only a
	// wake-up grant: push just enough to engage every spare participant. The
	// calling thread participates too, hence workerCount - 1.
	const std::size_t tickets =
		std::min<std::size_t>(m_workerCount - 1, jobCount);
	{
		std::lock_guard lock(m_queueMutex);
		for (std::size_t ticket = 0; ticket < tickets; ++ticket) {
			m_queue.push_back(job);
		}
	}
	m_queueCv.notify_all();

	// Calling thread participates until all chunks are claimed.
	RunChunks(*job);

	// Frame-level barrier: wait until every chunk has finished.
	{
		std::unique_lock lock(job->mutex);
		job->doneCv.wait(lock, [&job, jobCount] {
			return job->chunksDone.load(std::memory_order_acquire) >= jobCount;
		});
	}
	if (job->firstException) {
		std::rethrow_exception(job->firstException);
	}
}

void JobSystem::WorkerMain(std::stop_token stopToken) {
	while (true) {
		std::shared_ptr<Job> job;
		{
			std::unique_lock lock(m_queueMutex);
			m_queueCv.wait(lock, stopToken, [this] { return m_stop || !m_queue.empty(); });
			if (m_stop) {
				return;
			}
			job = m_queue.front();
			m_queue.pop_front();
		}
		RunChunks(*job);
	}
}

void JobSystem::RunChunks(Job& job) {
	while (true) {
		const std::size_t chunk = job.nextChunk.fetch_add(1, std::memory_order_relaxed);
		if (chunk >= job.jobCount) {
			return;
		}
		const std::size_t begin = chunk * job.chunkSize;
		const std::size_t end = std::min(begin + job.chunkSize, job.itemCount);
		// A throwing chunk must never stall the barrier: record the first
		// exception, still count the chunk, and let the caller rethrow.
		try {
			job.function(begin, end);
		} catch (...) {
			std::lock_guard lock(job.mutex);
			if (!job.firstException) {
				job.firstException = std::current_exception();
			}
		}
		const std::size_t done = job.chunksDone.fetch_add(1, std::memory_order_acq_rel) + 1;
		if (done == job.jobCount) {
			std::lock_guard lock(job.mutex);
			job.doneCv.notify_all();
		}
	}
}

std::size_t JobSystem::ComputeChunkSize(std::size_t count, std::size_t minItemsPerJob) const {
	const std::size_t floor_ = std::max<std::size_t>(1, minItemsPerJob);
	// Target roughly 4 chunks per participant so uneven chunk costs can be
	// load balanced, while still honoring the caller's minimum chunk size.
	const std::size_t participants = std::max<uint32_t>(1, m_workerCount);
	const std::size_t target = (count + participants * 4 - 1) / (participants * 4);
	return std::max(floor_, target);
}

} // namespace Core
} // namespace Runtime
